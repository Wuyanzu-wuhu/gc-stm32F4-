# ============================================================
# K230 视觉主程序 (修复运行环境问题，基本逻辑不变)
# ============================================================
import os, sys, gc
import time
import struct
from machine import FPIOA, UART # 新增 FPIOA
from media.sensor import *
from media.display import *
from media.media import *

# ============================================================
# 修复1: FPIOA 引脚配置 (UART4 用，按你的硬件修改引脚号!)
# 注意：不同K230板子UART4引脚不一样，常见组合：
# K230-CanMV 小板: TX=36, RX=37
# 亚博开发板: TX=32, RX=33
# 如果收不到数据，优先怀疑这里引脚号不对！
# ============================================================
fpioa = FPIOA()

# ★ 按你的板子修改下面两行的引脚号 ★
fpioa.set_function(36, FPIOA.UART4_TXD) # K230_TX → TTL_RX
fpioa.set_function(37, FPIOA.UART4_RXD) # K230_RX ← TTL_TX

print("[硬件] FPIOA 引脚配置完成 (UART4 TX=36, RX=37)")

# ============================================================
# 第一部分：通信协议 (完全不变)
# ============================================================
FRAME_HEADER_0 = 0xAA
FRAME_HEADER_1 = 0x55
FRAME_TAIL = 0x0D

CMD_HEADER = 0xFF
CMD_TAIL = 0xFE
CMD_IDLE = 0x0F
CMD_TASK1 = 0x1F # TASK1 第一轮
CMD_TASK2 = 0x2F # TASK2 第一轮
CMD_NEXT = 0x3F # 下一个颜色（两关通用）
CMD_TASK3 = 0x4F # TASK1 第二轮
CMD_TASK4 = 0x5F # TASK2 第二轮

SPECIAL_ARRIVED = 32767

def send_offset_frame(uart, dx, dy):
    dx = max(-32768, min(32767, int(dx)))
    dy = max(-32768, min(32767, int(dy)))
    dx_bytes = struct.pack('>h', dx)
    dy_bytes = struct.pack('>h', dy)
    frame = bytes([
        FRAME_HEADER_0, FRAME_HEADER_1,
        dx_bytes[0], dx_bytes[1],
        dy_bytes[0], dy_bytes[1],
        FRAME_TAIL
    ])
    uart.write(frame)

def send_arrived(uart):
    send_offset_frame(uart, SPECIAL_ARRIVED, SPECIAL_ARRIVED)

class STM32CommandParser:
    def __init__(self):
        self._cmd_state = 0
        self._cmd_buf = bytearray(3)
        self._qr_state = 0
        self._qr_buf = bytearray(16)
        self._qr_len = 0
        self._pending_msg = None

    def feed(self, data):
        for b in data:
            self._feed_one_byte(b)

    def _feed_one_byte(self, b):
        # 先处理指令帧状态机
        if self._cmd_state == 0:
            if b == CMD_HEADER:
                self._cmd_buf[0] = b
                self._cmd_state = 1
                return # ★ 加了return，避免底部重复调用_try_qr
            # 不是指令头 → 交给QR解析（底部统一处理）
        elif self._cmd_state == 1:
            if b == CMD_HEADER or b == CMD_TAIL:
                self._cmd_state = 0
            else:
                self._cmd_buf[1] = b
                self._cmd_state = 2
                return # ★ 加了return
        elif self._cmd_state == 2:
            if b == CMD_TAIL:
                self._pending_msg = {
                    'type': 'CMD',
                    'cmd': self._cmd_buf[1],
                    'cmd_name': self._cmd_name(self._cmd_buf[1])
                }
                self._cmd_state = 0
        # 指令处理完，这个字节也可能是QR数据的开始，继续往下走
        # 每条字节都尝试QR解析（每个字节只到这里一次）
        self._try_qr(b)

    def _try_qr(self, b):
        # 状态0：等待帧头 '$'
        if self._qr_state == 0:
            if b == ord('$'):
                self._qr_state = 1
                self._qr_len = 0
                print(f"[QR调试] 收到帧头 $")
                return
        # 状态1：收集有效字符，直到帧尾 '#'
        if self._qr_state == 1:
            if b == ord('#'):
                # 帧尾到达，解析
                print(f"[QR调试] 收到帧尾 #, 缓冲区长度={self._qr_len}")
                self._parse_qr_string()
                self._qr_state = 0
                self._qr_len = 0
            elif (0x30 <= b <= 0x39) or b == ord('+'):
                if self._qr_len < len(self._qr_buf):
                    self._qr_buf[self._qr_len] = b
                    self._qr_len += 1
            else:
                # 非法字符，重置
                print(f"[QR调试] 非法字节 0x{b:02X}，重置状态")
                self._qr_state = 0
                self._qr_len = 0

    def _parse_qr_string(self):
        try:
            qr_str = self._qr_buf[:self._qr_len].decode('ascii')
            print(f"[QR解析] 收到字符串: '{qr_str}'")
            if '+' in qr_str:
                parts = qr_str.split('+')
                if len(parts) == 2 and len(parts[0]) == 3 and len(parts[1]) == 3:
                    color_str = parts[0] + parts[1]
                    color_sequence = [int(c) for c in color_str]
                    self._pending_msg = {
                        'type': 'QR',
                        'color_sequence': color_sequence
                    }
                    print(f"[QR解析] 成功! 颜色序列: {color_sequence}")
                else:
                    print(f"[QR解析] 格式错误: '{qr_str}'")
            else:
                print(f"[QR解析] 缺少'+'分隔符: '{qr_str}'")
        except Exception as e:
            print(f"[QR解析] 异常: {e}")

    def _cmd_name(self, cmd):
        names = {
            CMD_IDLE: 'IDLE',
            CMD_TASK1: 'TASK1',
            CMD_TASK2: 'TASK2',
            CMD_NEXT: 'NEXT',
            CMD_TASK3: 'TASK3',
            CMD_TASK4: 'TASK4'
        }
        return names.get(cmd, f'UNKNOWN(0x{cmd:02X})')

    def get_message(self):
        msg = self._pending_msg
        self._pending_msg = None
        return msg

# ============================================================
# 第二部分：视觉参数 (不变)
# ============================================================
# ROI区域：居中于800x480画面，四周留边
ROI_X, ROI_Y, ROI_W, ROI_H = 80, 40, 640, 400
ROI_CENTER_X = ROI_X + ROI_W // 2
ROI_CENTER_Y = ROI_Y + ROI_H // 2
ALLOW_RANGE = 10

COLOR_THRESHOLDS = {
    "1": [(0, 100, 25, 127, -128, 127)],
    "2": [(0, 100, -128, -15, -13, 79)],
    "3": [(0, 28, -128, 127, -128, -19)],
}
COLOR_NAMES = {"1": "红色", "2": "绿色", "3": "蓝色"}

# ============================================================
# 任务2：绿色色环参数（切换回绿色）
# ============================================================
# 绿色 LAB 阈值（★ 用户已调最优，禁止再动！）
GREEN_RING_THRESHOLD = [(0, 100, -128, -11, -128, 127)]
GREEN_LOOSE_THRESHOLD = [(0, 100, -128, -11, -128, 127)]

T2_MIN_AREA = 200
T2_MAX_AREA = 80000
T2_ALLOW_RANGE = 0
T2_DEAD_BAND = 5 # ★ 5→10：扩大死区，微小偏差不发送

# 跨帧跟踪参数（★ 终极抗抖版）
T2_TRACK_ALPHA = 0.7 # bbox平滑用，吃掉 find_blobs 的微小跳变
T2_TRACK_MAX_JUMP = 120
# 把它们改成这样：
T2_STABLE_FRAMES = 3         # ★ 3→8：连续8帧（约350ms）稳定才算稳定，防快速扫过
T2_STABLE_DIST = 4           # ★ 5→3：帧间位移必须小于3px，更严格的静止判定
T2_TRACK_TTL = 15
T2_IN_POS_FRAMES = 3         # ★ 新增：连续5帧真实坐标都在容差内，才算真到位
T2_IOU_MATCH_THRESH = 0.30 # IoU>此值视为同一blob（跨帧保持选择）

# ★ 中心坐标 — 像素级死区（最简单=最快=最粘）
T2_CENTER_DEAD_BAND = 6

# 发送前 dx/dy 低通滤波（★ 最终输出稳定性保障）
T2_SEND_ALPHA = 0.30 # ★ 0.4→0.25：发送滤波更强，STM32 收到的 dx/dy 稳如狗
T2_SEND_DEAD_BAND = 0 # ★ 10→12：微小偏差直接不发，避免微调震荡

# 绘制外扩
T2_DRAW_EXPAND = 8 # 绘制时向外扩 8px（正方形边长 = max(w,h) + 2*expand）
T2_FORCE_SQUARE = True # 强制方框为正方形（解决左右框边不齐）

# 最小尺寸筛选（防止误检过小色块/边缘碎片）
T2_MIN_W = 8
T2_MIN_H = 8

# 调试模式
T2_DEBUG_MODE = True

# ====== 中心稳定性修复参数 ======
# 中心直接平滑（不再从4个bbox值间接推导）
T2_CENTER_ALPHA_STABLE = 0.6   # 稳定跟踪时的中心EMA系数（响应快+稳定）
T2_CENTER_ALPHA_INIT =  0.7      # 首次检测/丢失恢复时：直接赋值（秒到）

# 中位数滤波（消除偶发跳变帧）
T2_MEDIAN_BUF_SIZE = 5           # 5帧滑动窗口取中位数

# 新增自适应alpha参数：
T2_CENTER_ALPHA_FAST = 0.65   # 快速移动时的高alpha（几乎无延迟）
T2_CENTER_ALPHA_SLOW = 0.3    # 慢速/静止时的平滑alpha
T2_ADAPT_THRESH = 8           # 位移>8px用快速alpha，否则用慢速alpha
# 偏置校准（如果你发现中心系统性地偏某个方向，在这里校准）
# 例如中心总是偏下3px，就设 T2_CENTER_OFFSET_Y = -3
T2_CENTER_OFFSET_X = 0           # 中心X校准偏移
T2_CENTER_OFFSET_Y = 0           # 中心Y校准偏移

# ============================================================
# 第三部分：硬件初始化 (修复 SDK API 调用)
# ============================================================
# ---- 修复2: 添加 CAM_CHN_ID_0 定义 (如果 SDK 没有自动导入) ----
try:
    _ = CAM_CHN_ID_0
except NameError:
    CAM_CHN_ID_0 = 0

# ---- 修复3: UART 初始化 (UART4 + FPIOA 引脚配置) ----
uart = UART(UART.UART4, 115200)
print("[硬件] 串口UART4初始化完成 (波特率115200)")

# ---- 修复4: Sensor 初始化对齐 main(3).py 的 API ----
sensor_id = 2
sensor = Sensor(id=sensor_id)
sensor.reset()

# 注意: 用 chn= 而非 channel=
sensor.set_framesize(width=800, height=480, chn=CAM_CHN_ID_0)
sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
sensor.set_hmirror(False)
sensor.set_vflip(False)
print(f"[硬件] 摄像头初始化完成 (800x480)")

# ---- 修复5: Display 初始化 (用大分辨率让图像铺满IDE预览区) ----
# 摄像头输出400x240, Display用800x480, show_image会自动拉伸铺满
try:
    Display.init(Display.ST7701, width=800, height=480, to_ide=True)
    print("[硬件] 显示屏初始化完成 (ST7701 800x480)")
except Exception as e:
    try:
        Display.init(Display.LT9611, width=800, height=480, to_ide=True)
        print("[硬件] 显示屏初始化完成 (LT9611 800x480)")
    except Exception as e2:
        try:
            Display.init(Display.VIRT, width=800, height=480, to_ide=True)
            print("[硬件] 虚拟显示屏初始化完成 (VIRT 800x480)")
        except Exception as e3:
            print(f"[硬件] 所有显示方式均失败，跳过: {e3}")

MediaManager.init()
sensor.run()
print("[硬件] 媒体管理器启动完成")

print("=" * 40)
print(" 初始化完成，开始主循环")
print("=" * 40)

# ============================================================
# 第四部分：应用层状态变量 (不变)
# ============================================================
state = "0"
color_sequence = []
color_idx = 0
sent_arrived = False
parser = STM32CommandParser()

# ★ 静止检测参数 ★
# 目标连续指定帧数内位移小于阈值，才算“静止”
STABLE_FRAMES = 5 # 普通检测模式：需要连续25帧稳定（约1.25秒）
TASK1_OFFSET_STABLE_FRAMES = 5 # 任务1偏移模式：需要连续35帧稳定，防止物块未静止就到位
STABLE_THRESHOLD = 2 # 每帧位置变化不超过2像素
STABLE_DRIFT_MAX = 12 # 稳定窗口内总漂移不超过12像素（防止慢速移动误判）
DEAD_BAND = 5 # 偏移死区：|dx|,|dy| 都小于此值时不发偏移，减少抖动

# 偏移模式：连续指定帧数进入中心范围且完成静止确认后，才发送到位信号
READY_FRAMES = 3
ready_count = 0

stable_count = 0 # 当前已稳定的帧数
is_stable = False # 是否已经静止
offset_started = False # offset模式：是否已经开始发微调数据

# 上一帧的目标坐标，用于计算帧间位移
prev_cx = 0
prev_cy = 0

# 稳定窗口参考坐标，用于检测慢速漂移
stable_ref_cx = 0
stable_ref_cy = 0
first_frame = True # 第一帧不计算位移

# 只追入、不追出：目标曾进入中心范围后开始远离，就就地停车
entered_range = False # 目标是否曾经进入过中心范围
prev_dist = 0 # 上一帧到中心距离（曼哈顿）
first_dist_frame = True # 第一帧不计算距离变化

# 任务2 状态变量（绿色色环跟踪）
t2_cx = 0.0
t2_cy = 0.0
t2_bbox_x = 0.0 # ★ blob bbox 的 x（跨帧平滑）
t2_bbox_y = 0.0 # ★ blob bbox 的 y
t2_bbox_w = 0.0 # ★ blob bbox 的 w（让框宽稳定）
t2_bbox_h = 0.0 # ★ blob bbox 的 h
t2_tracked = False
t2_stable = 0
t2_sent_arrived = False
t2_no_detect = 0
t2_send_dx = 0.0 # ★ 发送滤波后的 dx
t2_send_dy = 0.0 # ★ 发送滤波后的 dy
t2_draw_dim = 0.0 # ★ 绘制尺寸冻结（外框不抖）

# ★ 中心直接平滑（不再从bbox间接推导）
t2_cx_ema = 0.0
t2_cy_ema = 0.0
t2_cx_init = False
# 中位数缓冲
t2_med_buf_x = [] # 滑动窗口
t2_med_buf_y = []
t2_in_pos_count = 0

# ★ TASK2 轮次/颜色索引（与 TASK1 共用 color_sequence）
t2_ring_idx = 0 # 当前追踪第几个色环 (0~5)
t2_round_start_idx = 0 # 本轮的起始 idx（第一轮=0, 第二轮=3）

# 帧率显示
frame_count = 0
fps_display = 0
last_fps_time = 0

# ============================================================
# 第五部分：消息处理 (不变)
# ============================================================
def handle_message(msg):
    global state, color_sequence, color_idx, sent_arrived
    global stable_count, is_stable, offset_started, prev_cx, prev_cy, first_frame
    global stable_ref_cx, stable_ref_cy, ready_count, entered_range, prev_dist, first_dist_frame
    global t2_cx, t2_cy, t2_tracked, t2_stable, t2_sent_arrived, t2_no_detect
    global t2_ring_idx, t2_round_start_idx # ★ TASK2 轮次控制
    # ★ 新增：重置中心平滑状态
    global t2_cx_ema, t2_cy_ema, t2_cx_init, t2_med_buf_x, t2_med_buf_y
    global t2_bbox_x, t2_bbox_y, t2_bbox_w, t2_bbox_h, t2_draw_dim
    global t2_in_pos_count
    if msg['type'] == 'QR':
        color_sequence = msg['color_sequence']
        color_idx = 0
        sent_arrived = False
        stable_count = 0
        is_stable = False
        offset_started = False
        first_frame = True
        first_dist_frame = True
        ready_count = 0
        entered_range = False
        prev_dist = 0
        stable_ref_cx, stable_ref_cy = 0, 0
        state = "1"
        print(f"\n========== 收到QR，开始任务1 ==========")
        print(f"颜色序列: {color_sequence}")
        if color_sequence:
            print(f"当前追踪: 第{color_idx+1}个, 颜色={COLOR_NAMES.get(str(color_sequence[color_idx]), '未知')}")

    elif msg['type'] == 'CMD':
        cmd = msg['cmd']
        print(f"\n[指令] 收到: {msg['cmd_name']}")

        if cmd == CMD_IDLE:
            state = "0"
            # color_sequence 不清空——后续任务指令(TASK3/TASK4/NEXT)仍需要它
            color_idx = 0
            sent_arrived = False
            stable_count = 0
            is_stable = False
            offset_started = False
            first_frame = True
            first_dist_frame = True
            ready_count = 0
            entered_range = False
            prev_dist = 0
            stable_ref_cx, stable_ref_cy = 0, 0
            print("[指令] 复位到待机状态")

        elif cmd == CMD_TASK1:
            if len(color_sequence) > 0:
                state = "1"
                color_idx = 0
                sent_arrived = False
                stable_count = 0
                is_stable = False
                offset_started = False
                first_frame = True
                first_dist_frame = True
                ready_count = 0
                entered_range = False
                prev_dist = 0
                stable_ref_cx, stable_ref_cy = 0, 0
                print(f"[指令] 开始任务1, 追踪颜色索引={color_idx}")
            else:
                print("[指令] 任务1: 尚未收到QR数据，忽略")

        elif cmd == CMD_TASK2:
            if len(color_sequence) > 0:
                state = "2"
                t2_ring_idx = 0
                t2_round_start_idx = 0
                t2_cx = 0.0
                t2_cy = 0.0
                t2_bbox_x = 0.0
                t2_bbox_y = 0.0
                t2_bbox_w = 0.0
                t2_bbox_h = 0.0
                t2_tracked = False
                t2_stable = 0
                t2_sent_arrived = False
                t2_no_detect = 0
                t2_send_dx = 0.0
                t2_send_dy = 0.0
                t2_draw_dim = 0.0
                # ★ 重置新增变量
                t2_cx_ema = 0.0
                t2_cy_ema = 0.0
                t2_cx_init = False
                t2_med_buf_x = []
                t2_med_buf_y = []
                t2_in_pos_count = 0
                ring_color = str(color_sequence[t2_ring_idx]) if t2_ring_idx < len(color_sequence) else '?'
                print(f"[指令] 开始TASK2第一轮, 色环索引={t2_ring_idx}, 颜色={COLOR_NAMES.get(ring_color, '?')}")
            else:
                print("[指令] TASK2: 尚未收到QR数据，忽略")

        elif cmd == CMD_NEXT:
            if state == "1" and len(color_sequence) > 0:
                color_idx += 1
                sent_arrived = False
                stable_count = 0
                is_stable = False
                offset_started = False
                first_frame = True
                first_dist_frame = True
                ready_count = 0
                entered_range = False
                prev_dist = 0
                stable_ref_cx, stable_ref_cy = 0, 0
                if color_idx >= len(color_sequence):
                    print("[指令] TASK1所有颜色追踪完毕，回到待机")
                    state = "0"
                else:
                    print(f"[指令] TASK1 下一个颜色, idx={color_idx}, 颜色={COLOR_NAMES.get(str(color_sequence[color_idx]), '未知')}")

            elif state == "2" and len(color_sequence) > 0:
                t2_ring_idx += 1
                t2_sent_arrived = False
                t2_tracked = False
                t2_stable = 0
                t2_no_detect = 0
                t2_send_dx = 0.0
                t2_send_dy = 0.0
                t2_draw_dim = 0.0
                # ★ 重置新增变量
                t2_cx_ema = 0.0
                t2_cy_ema = 0.0
                t2_cx_init = False
                t2_med_buf_x = []
                t2_med_buf_y = []
                t2_in_pos_count = 0
                if t2_ring_idx >= t2_round_start_idx + 3 or t2_ring_idx >= len(color_sequence):
                    print("[指令] TASK2本轮色环对准完毕")
                    state = "0"
                else:
                    ring_color = str(color_sequence[t2_ring_idx])
                    print(f"[指令] TASK2 下一个色环, idx={t2_ring_idx}, 颜色={COLOR_NAMES.get(ring_color, '?')}")
            else:
                print(f"[指令] NEXT: 当前state={state}或颜色序列为空，忽略")

        elif cmd == CMD_TASK3:
            if len(color_sequence) >= 6:
                state = "1"
                color_idx = 3
                sent_arrived = False
                stable_count = 0
                is_stable = False
                offset_started = False
                first_frame = True
                first_dist_frame = True
                ready_count = 0
                entered_range = False
                prev_dist = 0
                stable_ref_cx, stable_ref_cy = 0, 0
                print(f"\n========== TASK1 第二轮 (index=3, 后三位颜色) ==========")
                print(f"全部颜色序列: {color_sequence}")
                print(f"第二轮颜色: {color_sequence[3:]}")
                cn = COLOR_NAMES.get(str(color_sequence[3]), '未知')
                print(f"当前追踪: idx=3, 颜色={cn}")
            else:
                print(f"[指令] TASK3: 颜色序列不足6个 (当前{len(color_sequence)}个), 无法启动第二轮")

        elif cmd == CMD_TASK4:
            if len(color_sequence) >= 6:
                state = "2"
                t2_ring_idx = 3
                t2_round_start_idx = 3
                t2_cx = 0.0
                t2_cy = 0.0
                t2_bbox_x = 0.0
                t2_bbox_y = 0.0
                t2_bbox_w = 0.0
                t2_bbox_h = 0.0
                t2_tracked = False
                t2_stable = 0
                t2_sent_arrived = False
                t2_no_detect = 0
                t2_send_dx = 0.0
                t2_send_dy = 0.0
                t2_draw_dim = 0.0
                # ★ 重置新增变量
                t2_cx_ema = 0.0
                t2_cy_ema = 0.0
                t2_cx_init = False
                t2_med_buf_x = []
                t2_med_buf_y = []
                t2_in_pos_count = 0
                ring_color = str(color_sequence[t2_ring_idx])
                print(f"\n========== TASK2 第二轮 (index=3) ==========")
                print(f"第二轮色环颜色: {color_sequence[3:]}")
                print(f"当前追踪: idx={t2_ring_idx}, 颜色={COLOR_NAMES.get(ring_color, '?')}")
            else:
                print(f"[指令] TASK4: 颜色序列不足6个 (当前{len(color_sequence)}个), 无法启动TASK2第二轮")

# ============================================================
# 第六部分：主循环 (修复 API 调用)
# ============================================================
print("\n进入主循环...\n")

while True:
    if os.exitpoint():
        break

    # 修复6: 对齐工作版的串口读取方式 + 调试输出 + 回环测试
    if uart.any():
        uart_data = uart.read()
        if uart_data and len(uart_data) > 0:
            # 调试1: 打印收到的原始字节到IDE控制台
            hex_str = ' '.join(f'{b:02X}' for b in uart_data)
            print(f"[串口RX] 收到 {len(uart_data)} 字节: {hex_str}")
            # ★ 去掉回环测试！回发会让 STM32 收到自己指令的回声，导致提前到位
            # uart.write(uart_data)
            parser.feed(uart_data)
            msg = parser.get_message()
            if msg:
                handle_message(msg)

    # 修复7: snapshot 加 chn 参数
    img = sensor.snapshot(chn=CAM_CHN_ID_0)

    # 画ROI区域 + 目标中心点标记
    img.draw_rectangle(ROI_X, ROI_Y, ROI_W, ROI_H, color=(255, 255, 255), thickness=2)

    # ★ 目标中心点：改用蓝色十字（避免绿色十字被颜色阈值误识别！）
    img.draw_line(ROI_CENTER_X - 15, ROI_CENTER_Y, ROI_CENTER_X + 15, ROI_CENTER_Y, color=(0, 0, 255), thickness=1)
    img.draw_line(ROI_CENTER_X, ROI_CENTER_Y - 15, ROI_CENTER_X, ROI_CENTER_Y + 15, color=(0, 0, 255), thickness=1)
    img.draw_cross(ROI_CENTER_X, ROI_CENTER_Y, color=(0, 0, 255), size=8, thickness=1)

    # ★ 允许误差范围框（±20像素）—— 改用白色边框，避免被任何颜色阈值捕获
    img.draw_rectangle(ROI_CENTER_X - ALLOW_RANGE, ROI_CENTER_Y - ALLOW_RANGE, ALLOW_RANGE * 2, ALLOW_RANGE * 2, color=(255, 255, 255), thickness=1)

    if state == "0":
        img.draw_string_advanced(10, 10, 2, "State: IDLE", color=(255, 255, 255))
        img.draw_string_advanced(10, 35, 1, "Waiting for QR...", color=(200, 200, 200))

    elif state == "1":
        # ★ 轮次显示：idx>=3 为第二轮
        if color_idx >= 3:
            img.draw_string_advanced(10, 10, 2, "State: TASK1 (Round 2)", color=(0, 255, 255))
        else:
            img.draw_string_advanced(10, 10, 2, "State: TASK1 (Round 1)", color=(0, 255, 0))

        if color_idx < len(color_sequence):
            color_code = str(color_sequence[color_idx])
            color_name = COLOR_NAMES.get(color_code, "?")
            thresholds = COLOR_THRESHOLDS.get(color_code, [])
        else:
            color_code = "?"
            color_name = "?"
            thresholds = []

        # ★ 显示当前追踪的颜色和索引
        img.draw_string_advanced(10, 35, 1, f"Color[{color_idx+1}/{len(color_sequence)}]: {color_name}({color_code})", color=(0, 200, 255))

        if len(thresholds) == 0:
            img.draw_string_advanced(10, 35, 1, f"Color threshold NOT SET for {color_code}!", color=(255, 0, 0))
        else:
            blobs = img.find_blobs(thresholds, roi=(ROI_X, ROI_Y, ROI_W, ROI_H), pixels_threshold=200, area_threshold=500)
            if blobs:
                max_blob = max(blobs, key=lambda b: b[2] * b[3])
                cx = max_blob[5]
                cy = max_blob[6]

                # ★ 目标位置：红色十字线
                img.draw_line(cx - 15, cy, cx + 15, cy, color=(255, 0, 0), thickness=2)
                img.draw_line(cx, cy - 15, cx, cy + 15, color=(255, 0, 0), thickness=2)

                # ★ 连接线：从目标到中心（显示偏差方向）
                img.draw_line(cx, cy, ROI_CENTER_X, ROI_CENTER_Y, color=(255, 255, 0), thickness=1)

                dx = cx - ROI_CENTER_X
                dy = cy - ROI_CENTER_Y

                # ★ 大字实时偏差显示
                img.draw_string_advanced(10, 55, 2, f"dx={dx:+4d} dy={dy:+4d}", color=(255, 255, 0))

                dist = int((dx*dx + dy*dy) ** 0.5)
                in_range = (abs(dx) <= ALLOW_RANGE and abs(dy) <= ALLOW_RANGE)
                use_offset = (color_idx == 0 or color_idx == 3)
                active_stable_frames = TASK1_OFFSET_STABLE_FRAMES if use_offset else STABLE_FRAMES # 偏移模式用35帧，普通检测模式仍用25帧

                # ===== 静止检测 =====
                if first_frame:
                    pos_delta = STABLE_THRESHOLD + 1
                    first_frame = False
                else:
                    pos_delta = abs(cx - prev_cx) + abs(cy - prev_cy)
                prev_cx, prev_cy = cx, cy

                if stable_count == 0:
                    stable_ref_cx, stable_ref_cy = cx, cy

                if pos_delta < STABLE_THRESHOLD:
                    stable_count += 1
                else:
                    stable_count = 0
                    is_stable = False

                if stable_count >= active_stable_frames and not is_stable:
                    total_drift = abs(cx - stable_ref_cx) + abs(cy - stable_ref_cy)
                    if total_drift < STABLE_DRIFT_MAX:
                        is_stable = True
                        print(f"[静止检测] 目标已稳定! 连续{active_stable_frames}帧静止, 总漂移={total_drift}px")
                    else:
                        print(f"[静止检测] 慢速移动中, 总漂移={total_drift}px, 重置计数")
                        stable_count = 0
                        is_stable = False

                # 进度条显示
                pct = min(100, int(stable_count * 100 / active_stable_frames))
                blen = int(stable_count / active_stable_frames * 10)
                bar = "#" * blen + "-" * (10 - blen)
                if is_stable:
                    scolor = (0, 255, 200)
                    sstr = "✓ STABLE"
                else:
                    sstr = f"[{bar}] {pct}%"
                    scolor = (180, 180, 180)
                img.draw_string_advanced(10, 80, 1, f"{sstr} delta={pos_delta}px", color=scolor)

                # ========== 偏移模式（颜色索引0和3）：只追入，不追出 ==========
                if use_offset:
                    if first_dist_frame:
                        first_dist_frame = False
                        prev_dist = dist
                        img.draw_string_advanced(10, 100, 1, f"First see, wait dir dist={dist}px", color=(200, 200, 200))
                    else:
                        send_dx = dx if abs(dx) > DEAD_BAND else 0
                        send_dy = dy if abs(dy) > DEAD_BAND else 0
                        approaching = dist < prev_dist
                        prev_dist = dist

                        if in_range:
                            entered_range = True

                        if sent_arrived:
                            img.draw_string_advanced(10, 100, 2, "★ ARRIVED! LOCKED ★", color=(0, 255, 0))
                        elif in_range:
                            # 已经进入中心范围时，优先等待静止确认；不要再被“距离没有继续变小”分支截走
                            send_offset_frame(uart, 0, 0)
                            if ready_count < READY_FRAMES:
                                ready_count += 1
                            if ready_count >= READY_FRAMES and is_stable and not sent_arrived:
                                send_arrived(uart)
                                sent_arrived = True
                                img.draw_string_advanced(10, 100, 2, "★ ARRIVED! ★", color=(0, 255, 0))
                                print(f"[TASK1] 到位!(追入锁定+静止确认, idx={color_idx})")
                            else:
                                img.draw_string_advanced(10, 100, 1, f"进框 {ready_count}/{READY_FRAMES} 静止 {stable_count}/{active_stable_frames} 距离={dist}px", color=(255, 200, 0))
                        elif not approaching:
                            send_offset_frame(uart, 0, 0)
                            ready_count = 0
                            if entered_range:
                                img.draw_string_advanced(10, 100, 1, f"保持: 目标正在离开 距离={dist}px", color=(255, 100, 100))
                            else:
                                img.draw_string_advanced(10, 100, 1, f"等待: 目标远离中心 距离={dist}px", color=(180, 180, 180))
                        else:
                            send_offset_frame(uart, send_dx, send_dy)
                            offset_started = True
                            ready_count = 0
                            img.draw_string_advanced(10, 100, 1, f"微调中... 距离={dist}px", color=(255, 200, 0))

                # ========== 检测模式 (idx≠0,3)：静止即到位（不检查中心，机械臂有余量） ==========
                else:
                    if is_stable and not sent_arrived:
                        send_arrived(uart)
                        sent_arrived = True
                        img.draw_string_advanced(10, 100, 2, "★ ARRIVED! (stable only) ★", color=(0, 255, 0))
                        print(f"[TASK1] 到位! (仅静止确认, idx={color_idx})")
                    elif sent_arrived:
                        img.draw_string_advanced(10, 100, 2, "★ ARRIVED! ★", color=(0, 255, 0))
                    else:
                        img.draw_string_advanced(10, 100, 1, f"Waiting stable... {stable_count}/{STABLE_FRAMES} delta={pos_delta}px", color=(255, 200, 0))
            else:
                img.draw_string_advanced(10, 35, 1, "No blob found", color=(255, 100, 100))

    elif state == "2":
        # ★ TASK2 轮次+色环显示
        round_label = "Round 2" if t2_ring_idx >= 3 else "Round 1"
        ring_color_code = str(color_sequence[t2_ring_idx]) if t2_ring_idx < len(color_sequence) else "?"
        ring_color_name = COLOR_NAMES.get(ring_color_code, "?")

        img.draw_string_advanced(10, 10, 2, f"State: TASK2 ({round_label})", color=(0, 255, 255))
        img.draw_string_advanced(10, 35, 1, f"Ring[{t2_ring_idx+1}/{len(color_sequence)}]: {ring_color_name}({ring_color_code})", color=(0, 200, 255))

        # ★ TASK2 只识别绿色色环，轮次/颜色名仅供显示，阈值不变
        img.draw_string_advanced(10, 460, 1, f"THR:L{GREEN_RING_THRESHOLD[0][0]}-{GREEN_RING_THRESHOLD[0][1]} A{GREEN_RING_THRESHOLD[0][2]}..{GREEN_RING_THRESHOLD[0][3]} B{GREEN_RING_THRESHOLD[0][4]}..{GREEN_RING_THRESHOLD[0][5]}", color=(120, 120, 120))

        # 帧率显示
        frame_count += 1
        fps_str = "FPS:--"
        try:
            now = time.ticks_ms()
            if last_fps_time == 0:
                last_fps_time = now
            dt = time.ticks_diff(now, last_fps_time)
            if dt >= 1000:
                fps_display = frame_count
                frame_count = 0
                last_fps_time = now
                fps_str = f"FPS:{fps_display}"
        except:
            fps_str = f"F:{frame_count}"

        # 黑色背景 + 白色字
        img.draw_rectangle(ROI_X + ROI_W - 115, ROI_Y + 2, 110, 24, color=(0,0,0), thickness=-1)
        img.draw_string_advanced(ROI_X + ROI_W - 110, ROI_Y + 5, 2, fps_str, color=(255, 255, 255))

        # ===== 双路径扫描：严格阈值 + 宽松阈值 =====
        det_cx, det_cy, found = None, None, False
        best_blob = None
        candidates = []
        all_raw = []
        debug_total_blobs = 0

        blobs = img.find_blobs(GREEN_RING_THRESHOLD, roi=(ROI_X, ROI_Y, ROI_W, ROI_H), pixels_threshold=10, area_threshold=T2_MIN_AREA, merge=True, margin=True)
        debug_total_blobs = len(blobs) if blobs else 0

        if blobs:
            for b in blobs:
                x, y, w, h, pixels = b[0], b[1], b[2], b[3], b[4]
                area = w * h
                if area == 0 or not (T2_MIN_AREA <= area <= T2_MAX_AREA):
                    continue
                if w < T2_MIN_W or h < T2_MIN_H:
                    continue

                density = pixels / area
                ratio = min(w, h) / max(w, h) if max(w, h) > 0 else 0
                bx = x + w // 2
                by = y + h // 2
                dist_to_center = ((bx - ROI_CENTER_X)**2 + (by - ROI_CENTER_Y)**2) ** 0.5

                all_raw.append((b, dist_to_center, density, ratio))

                if 0.08 <= density <= 0.55 and ratio >= 0.65:
                    color = (150, 150, 0)
                    candidates.append((b, dist_to_center, density, ratio))
                else:
                    color = (80, 80, 80)
                img.draw_rectangle(x, y, w, h, color=color, thickness=1)
                img.draw_string_advanced(x, max(y - 10, 0), 1, f"d{int(density*100)} r{int(ratio*100)}", color=color)

        # 路径2：宽松阈值二次扫描
        if not candidates:
            loose_blobs = img.find_blobs(GREEN_LOOSE_THRESHOLD, roi=(ROI_X, ROI_Y, ROI_W, ROI_H), pixels_threshold=10, area_threshold=100, merge=True, margin=True)
            if loose_blobs:
                for b in loose_blobs:
                    x, y, w, h, pixels = b[0], b[1], b[2], b[3], b[4]
                    area = w * h
                    if area == 0 or area > T2_MAX_AREA:
                        continue
                    if w < T2_MIN_W or h < T2_MIN_H:
                        continue

                    density = pixels / area
                    ratio = min(w, h) / max(w, h) if max(w, h) > 0 else 0

                    if 0.05 <= density <= 0.6 and ratio >= 0.5:
                        bx = x + w // 2
                        by = y + h // 2
                        dist_to_center = ((bx - ROI_CENTER_X)**2 + (by - ROI_CENTER_Y)**2) ** 0.5
                        candidates.append((b, dist_to_center, density, ratio))
                        all_raw.append((b, dist_to_center, density, ratio))
                        img.draw_rectangle(x, y, w, h, color=(0, 150, 150), thickness=1)
                        img.draw_string_advanced(x, max(y - 10, 0), 1, "LOOSE", color=(0, 150, 150))

        # ===== 关键改进：圆心计算与跨帧匹配 =====
        if candidates:
            # ★ 跨帧保持选同一个 blob（IoU 优先）
            if t2_tracked and t2_bbox_w > 0 and t2_bbox_h > 0:
                best_iou = 0.0
                best_cand_iou = None
                for c in candidates:
                    b = c[0]
                    ix1 = max(b[0], int(t2_bbox_x))
                    iy1 = max(b[1], int(t2_bbox_y))
                    ix2 = min(b[0] + b[2], int(t2_bbox_x + t2_bbox_w))
                    iy2 = min(b[1] + b[3], int(t2_bbox_y + t2_bbox_h))
                    if ix2 > ix1 and iy2 > iy1:
                        inter = (ix2 - ix1) * (iy2 - iy1)
                        area1 = b[2] * b[3]
                        area2 = int(t2_bbox_w * t2_bbox_h)
                        union = area1 + area2 - inter
                        if union > 0:
                            iou = inter / union
                            if iou > best_iou:
                                best_iou = iou
                                best_cand_iou = c

                if best_iou >= T2_IOU_MATCH_THRESH:
                    best_blob, _, best_density, best_ratio = best_cand_iou
                else:
                    candidates.sort(key=lambda x: x[1])
                    best_blob, _, best_density, best_ratio = candidates[0]
            else:
                candidates.sort(key=lambda x: x[1])
                best_blob, _, best_density, best_ratio = candidates[0]

            # ★ 修复：用质心(best_blob[5], [6])代替bbox中心
            det_cx = best_blob[0] + best_blob[2] / 2   # bbox中心X = 几何中心
            det_cy = best_blob[1] + best_blob[3] / 2   # bbox中心Y
            center_mode = "CENTROID"  # 标记使用了质心
            det_bbox_x = best_blob[0]   # bbox仅用于绘制方框
            det_bbox_y = best_blob[1]
            det_bbox_w = best_blob[2]
            det_bbox_h = best_blob[3]
            found = True

            img.draw_string_advanced(10, 160, 1, f"Candidates: {len(candidates)} (raw={len(all_raw)}) blobs={debug_total_blobs}", color=(180, 180, 180))

            if not candidates and all_raw:
                all_raw.sort(key=lambda x: -(x[0][2] * x[0][3]))
                for b, _, d, r in all_raw[:3]:
                    x, y, w, h = b[0], b[1], b[2], b[3]
                    img.draw_rectangle(x, y, w, h, color=(0, 255, 255), thickness=2)
                    img.draw_string_advanced(x, max(y - 25, 0), 1, f"MISS d{int(d*100)} r{int(r*100)}", color=(0, 255, 255))

            if debug_total_blobs == 0:
                img.draw_rectangle(ROI_CENTER_X - 100, ROI_CENTER_Y - 20, 200, 40, color=(0, 0, 0), thickness=-1)
                img.draw_string_advanced(ROI_CENTER_X - 90, ROI_CENTER_Y - 12, 2, "THR NO MATCH!", color=(255, 0, 0))

        # ===== 跨帧跟踪（★ 直接平滑中心版） =====
        if found:
            t2_no_detect = 0

            # ====== 中位数滤波：5帧窗口取中位数，消除偶发跳变 ======
            t2_med_buf_x.append(det_cx)
            t2_med_buf_y.append(det_cy)
            if len(t2_med_buf_x) > T2_MEDIAN_BUF_SIZE:
                t2_med_buf_x.pop(0)
                t2_med_buf_y.pop(0)
            # 取中位数
            med_cx = sorted(t2_med_buf_x)[len(t2_med_buf_x) // 2]
            med_cy = sorted(t2_med_buf_y)[len(t2_med_buf_y) // 2]
            # 取中位数
            med_cx = sorted(t2_med_buf_x)[len(t2_med_buf_x) // 2]
            med_cy = sorted(t2_med_buf_y)[len(t2_med_buf_y) // 2]

            # ★ 新增：跳变钳位 — 如果中位数离 EMA 太远，限制最大跳变距离
            if t2_tracked and t2_cx_init:
                jump = ((med_cx - t2_cx_ema)**2 + (med_cy - t2_cy_ema)**2) ** 0.5
                max_jump = 25  # 每帧最多跳25px（色环正常移动不会超过这个速度）
                if jump > max_jump:
                    ratio = max_jump / jump
                    med_cx = t2_cx_ema + (med_cx - t2_cx_ema) * ratio
                    med_cy = t2_cy_ema + (med_cy - t2_cy_ema) * ratio

            if t2_tracked and t2_cx_init:
                # ★ 自适应alpha：移动快时alpha大（无延迟），移动慢时alpha小（平滑）
                move_dist = abs(med_cx - t2_cx_ema) + abs(med_cy - t2_cy_ema)
                if move_dist > T2_ADAPT_THRESH:
                    adapt_alpha = T2_CENTER_ALPHA_FAST   # 快速追上
                else:
                    adapt_alpha = T2_CENTER_ALPHA_SLOW   # 平滑抗抖

                # ★ 核心：直接平滑中心，用自适应alpha
                t2_cx_ema = t2_cx_ema * (1 - adapt_alpha) + med_cx * adapt_alpha
                t2_cy_ema = t2_cy_ema * (1 - adapt_alpha) + med_cy * adapt_alpha

                # bbox 仍然平滑（仅用于绘制方框，保持视觉稳定）
                t2_bbox_x = t2_bbox_x * (1 - T2_TRACK_ALPHA) + det_bbox_x * T2_TRACK_ALPHA
                t2_bbox_y = t2_bbox_y * (1 - T2_TRACK_ALPHA) + det_bbox_y * T2_TRACK_ALPHA
                t2_bbox_w = t2_bbox_w * (1 - T2_TRACK_ALPHA) + det_bbox_w * T2_TRACK_ALPHA
                t2_bbox_h = t2_bbox_h * (1 - T2_TRACK_ALPHA) + det_bbox_h * T2_TRACK_ALPHA

                # ★ 最终中心 = 直接平滑后的值 + 校准偏移
                t2_cx = t2_cx_ema + T2_CENTER_OFFSET_X
                t2_cy = t2_cy_ema + T2_CENTER_OFFSET_Y

                # 稳定性判断
                movement = abs(det_cx - t2_cx) + abs(det_cy - t2_cy)
                if movement <= T2_STABLE_DIST:
                    t2_stable += 1
                else:
                    t2_stable = 0

                if T2_DEBUG_MODE:
                    img.draw_string_advanced(10, 135, 1, f"centroid alpha={T2_CENTER_ALPHA_STABLE:.2f} mov={movement:.1f}px med={len(t2_med_buf_x)}", color=(150, 150, 150))
            else:
                # 首次检测 / 丢失恢复：直接赋值（秒到，不等待EMA收敛）
                t2_cx_ema = float(det_cx)
                t2_cy_ema = float(det_cy)
                t2_cx = t2_cx_ema + T2_CENTER_OFFSET_X
                t2_cy = t2_cy_ema + T2_CENTER_OFFSET_Y
                t2_bbox_x = float(det_bbox_x)
                t2_bbox_y = float(det_bbox_y)
                t2_bbox_w = float(det_bbox_w)
                t2_bbox_h = float(det_bbox_h)
                t2_tracked = True
                t2_cx_init = True
                t2_stable = 1
                t2_med_buf_x = [det_cx]
                t2_med_buf_y = [det_cy]
        else:
            t2_no_detect += 1
            if t2_no_detect > T2_TRACK_TTL:
                t2_tracked = False
                t2_cx_init = False
                t2_stable = 0
                t2_med_buf_x = []
                t2_med_buf_y = []


        # ===== 偏差计算与发送（★ 发送滤波 + 死区扩大） =====
        if t2_tracked:
            # ★ 用平滑后的 bbox 绘制方框
            if T2_FORCE_SQUARE:
                raw_max_dim = max(t2_bbox_w, t2_bbox_h)
                if t2_draw_dim == 0.0:
                    t2_draw_dim = raw_max_dim
                elif abs(raw_max_dim - t2_draw_dim) > 3:
                    t2_draw_dim = t2_draw_dim * 0.8 + raw_max_dim * 0.2

                max_dim = t2_draw_dim
                cx_box = t2_cx
                cy_box = t2_cy
                half = max_dim / 2 + T2_DRAW_EXPAND
                sx = int(cx_box - half)
                sy = int(cy_box - half)
                sw = int(max_dim + T2_DRAW_EXPAND * 2)
                sh = sw
            else:
                sx = int(t2_bbox_x) - T2_DRAW_EXPAND
                sy = int(t2_bbox_y) - T2_DRAW_EXPAND
                sw = int(t2_bbox_w) + T2_DRAW_EXPAND * 2
                sh = int(t2_bbox_h) + T2_DRAW_EXPAND * 2

            if sx < ROI_X: sx = ROI_X
            if sy < ROI_Y: sy = ROI_Y
            if sx + sw > ROI_X + ROI_W: sw = ROI_X + ROI_W - sx
            if sy + sh > ROI_Y + ROI_H: sh = ROI_Y + ROI_H - sy

            img.draw_rectangle(sx, sy, sw, sh, color=(0, 255, 0), thickness=2)
            if T2_FORCE_SQUARE:
                img.draw_string_advanced(sx, max(sy - 12, 0), 1, f"square exp+{T2_DRAW_EXPAND}", color=(0, 255, 0))

            # ★ 唯一的圆心十字（青色=最终输出位置）
            img.draw_cross(int(t2_cx), int(t2_cy), color=(0, 255, 255), size=18, thickness=2)
            img.draw_circle(int(t2_cx), int(t2_cy), 3, color=(0, 255, 255), thickness=-1)

            # 用平滑点做发送和显示
            cx_i = int(t2_cx)
            cy_i = int(t2_cy)
            dx = cx_i - ROI_CENTER_X
            dy = cy_i - ROI_CENTER_Y

            # ★ 关键修改：用真实的中位数坐标判定是否在中心，防止平滑导致的假到位
            raw_dx = int(med_cx) - ROI_CENTER_X
            raw_dy = int(med_cy) - ROI_CENTER_Y
            raw_in_pos = (abs(raw_dx) <= T2_ALLOW_RANGE and abs(raw_dy) <= T2_ALLOW_RANGE)

            is_stable = (t2_stable >= T2_STABLE_FRAMES)

            # ★ 弹性帧计数器：在容差内+1，抖出容差-1（不清零）
            if raw_in_pos and t2_tracked:
                # 进入范围：累加，但封顶到阈值（防止长期停在中心时数字无限涨）
                if t2_in_pos_count < T2_IN_POS_FRAMES:
                   t2_in_pos_count += 1
                else:
                     # 抖动出界：只扣1分，不清零，给机械振动留容忍度
                     t2_in_pos_count = max(t2_in_pos_count - 1, 0)

            # ★ 发送前低通滤波
            t2_send_dx = t2_send_dx * (1 - T2_SEND_ALPHA) + dx * T2_SEND_ALPHA
            t2_send_dy = t2_send_dy * (1 - T2_SEND_ALPHA) + dy * T2_SEND_ALPHA
            send_dx_raw = int(t2_send_dx)
            send_dy_raw = int(t2_send_dy)

            img.draw_string_advanced(10, 60, 2, f"dx={dx:+4d} dy={dy:+4d}", color=(255, 255, 0))
            img.draw_string_advanced(10, 90, 1, f"{ring_color_name}({cx_i},{cy_i}) Center({ROI_CENTER_X},{ROI_CENTER_Y})", color=(100, 255, 100))
            # 用不同颜色提示当前状态：绿色=累积中，黄色=接近触发，红色=被扣分
            if t2_in_pos_count >= T2_IN_POS_FRAMES:
                in_pos_color = (0, 255, 0)
            elif t2_in_pos_count >= T2_IN_POS_FRAMES - 1:
                in_pos_color = (255, 255, 0)
            else:
                in_pos_color = (200, 200, 200)
            img.draw_string_advanced(10, 115, 1, f"Stable {t2_stable}/{T2_STABLE_FRAMES} | InPos {t2_in_pos_count}/{T2_IN_POS_FRAMES}", color=in_pos_color)

            if t2_sent_arrived:
                img.draw_string_advanced(10, 140, 2, "ARRIVED! LOCKED", color=(0, 255, 0))
            # ★ 双重确认：既要有足够的静止帧，也要有足够连续的在中心帧
            elif t2_in_pos_count >= T2_IN_POS_FRAMES:
                send_arrived(uart)
                t2_sent_arrived = True
                img.draw_string_advanced(10, 140, 2, "ARRIVED!", color=(0, 255, 0))
                print(f"[TASK2] 到位! {ring_color_name}色环({cx_i},{cy_i}) Center({ROI_CENTER_X},{ROI_CENTER_Y}) idx={t2_ring_idx}")
            else:
                if not raw_in_pos:
                    send_dx = send_dx_raw if abs(send_dx_raw) > T2_SEND_DEAD_BAND else 0
                    send_dy = send_dy_raw if abs(send_dy_raw) > T2_SEND_DEAD_BAND else 0
                    send_offset_frame(uart, send_dx, send_dy)
                    img.draw_string_advanced(10, 140, 1, f"MOVE dx={send_dx} dy={send_dy} (raw {send_dx_raw},{send_dy_raw})", color=(255, 200, 0))
                else:
                    send_offset_frame(uart, 0, 0)
                    img.draw_string_advanced(10, 140, 1, f"IN RANGE, wait stable {t2_stable}/{T2_STABLE_FRAMES} | InPos {t2_in_pos_count}/{T2_IN_POS_FRAMES}", color=(255, 200, 0))

        else:
            img.draw_string_advanced(10, 60, 1, "No green ring found", color=(255, 100, 100))
            # ★ 关键修复：找不到色环时，不发任何帧！

    # 修复9: Display.show_image 只传 img
    try:
        Display.show_image(img)
    except Exception as e:
        pass # 无屏幕时忽略

    time.sleep_ms(0)

# ============================================================
# 清理退出 (不变)
# ============================================================
print("程序退出，清理资源...")
MediaManager.deinit()
sensor.stop()
print("清理完成")
