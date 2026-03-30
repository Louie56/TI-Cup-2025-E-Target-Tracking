import time, os, sys
from media.sensor import *
from media.display import *
from media.media import *

from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
from ybUtils.YbKey import YbKey
import struct
import math

uart = YbUart(baudrate=9600)
pto = YbProtocol()
key = YbKey()

# 显示参数 / Display parameters
DISPLAY_WIDTH = 640   # LCD显示宽度 / LCD display width
DISPLAY_HEIGHT = 480  # LCD显示高度 / LCD display height

# LAB颜色空间阈值 / LAB color space thresholds
THRESHOLDS = [
    (98, 100, 126-128, 254-128, 78-128, 254-128),
    (94, 100, -12, 127, -50, 127),
    (92, 100, 122-128, 208-128, 122-128, 172-128),
    (43, 99, -43, -4, -56, -7),
    (37, 100, -128, 127, -128, -27)
]

def uart_process_frame(data):
    # 根据协议解析数据
    if len(data) < 8:  # 至少需要8个字节
        print("数据过短，忽略")
        return

    # 检查帧头
    if data[0] == 0xAA and data[1] == 0xAA:
        addr = data[2]
        cmd = data[3]
        tail = data[-3:]

        # 检查帧尾
        if tail == [0xFF, 0xFF, 0xFF]:
            payload = data[4:-3]  # 数据区，排除头尾

            # 处理 K230 的数据
            if addr == 0x02:  # 目标设备是 K230
                if cmd == 0x20:  # LAB 阈值命令
                    group_id = payload[0]-1  # 阈值组ID
                    LMin = payload[1]
                    LMax = payload[2]
                    AMin = payload[3]-128
                    AMax = payload[4]-128
                    BMin = payload[5]-128
                    BMax = payload[6]-128

                    # 打印出阈值组内容
                    print(f"接收到 LAB 阈值 - 组 {group_id}:")
                    print(f"  LMin: {LMin}, LMax: {LMax}")
                    print(f"  AMin: {AMin}, AMax: {AMax}")
                    print(f"  BMin: {BMin}, BMax: {BMax}")
                    threshold_tuple = (group_id, LMin, LMax, AMin, AMax, BMin, BMax)
                    return threshold_tuple
                else:
                    print("未知命令")
            else:
                print("未知地址")
        else:
            print("无效帧尾")
    else:
        print("无效帧头")

def uart_send_pixel_coordinates(ser, group_id, x, y):
    """
    发送像素坐标数据帧
    :param ser: 串口对象
    :param group_id: 组ID (1字节)
    :param x: X坐标 (16位整数)
    :param y: Y坐标 (16位整数)
    """
    # 构建数据帧
    frame = bytearray()

    # 帧头
    frame.append(0xAA)
    frame.append(0xAA)

    # 地址 (0x01表示目标设备是单片机)
    frame.append(0x01)

    # 命令 (0x30表示像素坐标)
    frame.append(0x30)

    # 数据区
    frame.append(group_id)  # 组ID

    # X坐标 (16位大端字节序)
    frame.extend(struct.pack('>H', x))

    # Y坐标 (16位大端字节序)
    frame.extend(struct.pack('>H', y))

    # 帧尾
    frame.append(0xFF)
    frame.append(0xFF)
    frame.append(0xFF)

    # 发送完整帧
    ser.write(frame)

    # 打印发送内容用于调试
    print(f"发送帧: {bytes(frame).hex(' ')}")
    print(f"内容: 组ID={group_id}, X={x}, Y={y}")


def init_sensor():
    """初始化摄像头 / Initialize camera sensor"""
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    return sensor

def init_display():
    """初始化显示 / Initialize display"""
    Display.init(Display.ST7701, to_ide=True)
    MediaManager.init()

def process_blobs(img, blobs, color):
    """处理检测到的色块 / Process detected color blobs"""
    for blob in blobs:
        img.draw_rectangle(blob[0:4], color=color, thickness=4)
        img.draw_cross(blob[5], blob[6], color=color, thickness=2)
#        x = blob[0]
#        y = blob[1]
#        w = blob[2]
#        h = blob[3]

def draw_fps(img, fps):
    """绘制FPS信息 / Draw FPS information"""
    img.draw_string_advanced(0, 0, 30, f'FPS: {fps:.3f}', color=(255, 255, 255))




def sort_corners(corners):
    """
    根据角点与中心点的位置关系，对四个角点进行排序：
    右上，右下，左下，左上的顺序
    """
    # 计算中心点
    center_x = sum([p[0] for p in corners]) / 4
    center_y = sum([p[1] for p in corners]) / 4

    # 根据角点与中心点的相对位置来排序
    sorted_corners = sorted(corners, key=lambda p: (math.atan2(p[1] - center_y, p[0] - center_x), p))

    return sorted_corners

def calculate_fitting_corners(outer_corners, inner_corners):
    """
    通过计算外框和内框对应角点的平均值，获得拟合框的角点。
    """
    fitting_corners = []
    for i in range(4):
        # 对应角点取平均
        avg_x = (outer_corners[i][0] + inner_corners[i][0]) / 2
        avg_y = (outer_corners[i][1] + inner_corners[i][1]) / 2
        fitting_corners.append((avg_x, avg_y))

    # 对拟合框的角点进行排序（顺时针方向）
    return sort_corners(fitting_corners)

def get_points_on_edges(sorted_corners, num_points=10):
    """
    在拟合框的边线上均匀取点，并返回这些点的集合。
    从右上角点开始，顺时针环绕。
    """
    points = []

    # 从右上角点开始，顺时针计算四条边上的点
    for i in range(4):
        # 获取边的两个端点
        start = sorted_corners[i]
        end = sorted_corners[(i + 1) % 4]

        # 计算这条边上均匀间隔的点
        for j in range(num_points):
            t = j / (num_points - 1)  # t 从 0 到 1
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            points.append((x, y))

    # 返回点集，包括四个角点和边上的点
    return points

picture_width = 640
picture_height = 480

def main():
    try:
        # 初始化设备 / Initialize devices
        sensor = init_sensor()
        init_display()
        sensor.run()

        clock = time.clock()

        last_update_time = time.ticks_ms()  # 初始化更新时间
        show_binary = False
        current_group_id = -1
        while True:
            os.exitpoint()

            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            img_bin = img.copy().binary([(0, 23, -49, 21, -22, 23)])

            # 用 blob 代替 rects 检测
            blobs = img_bin.find_blobs([(0, 23, -49, 21, -22, 23)], area_threshold=100, pixels_threshold=1000)

            print("找到 blobs 数:", len(blobs))

            for b in blobs:
                x, y, w, h = b.rect()
                cx = b.cx()
                cy = b.cy()

                aspect_ratio = w / h
                area = w * h

                if area < 700 or area > 55000:
                    continue
                if w < 17 or h < 40:
                    continue
                if not (0.8 < aspect_ratio < 1.8):
                    continue


                img.draw_rectangle(x, y, w, h, color=(255, 255, 255), thickness=2)
                img.draw_cross(cx, cy, color=(255, 0, 0), size=10)

                # 在矩形内部查找 HSV 白色色块（用 LAB 替代 HSV）
                try:
                    img_crop = img.copy(roi=(x, y, w, h))
                    blobs_white = img_crop.find_blobs([(40, 100, -57, 127, -67, 78)], area_threshold=20, pixels_threshold=20, merge=True)
                    matched_area = sum([bw.pixels() for bw in blobs_white])
                except Exception as e:
                    print("处理LAB区域失败:", e)
                    continue

                # 环状区域 = 原矩形总面积 - 中心白色色块面积
                ring_area = area - matched_area
                if matched_area <= 0 or matched_area < ring_area:
                    continue

                img.draw_rectangle(x, y, w, h, color=(0, 255, 0), thickness=2)
                img.draw_cross(cx, cy, color=(255, 0, 0), size=10)


                # 如果通过环状检测，则进一步在扩大30像素区域中识别矩形
                expand = 40
                x_ext = max(0, x - expand)
                y_ext = max(0, y - expand)
                w_ext = min(img.width() - x_ext, w + 2 * expand)
                h_ext = min(img.height() - y_ext, h + 2 * expand)

                try:
#                    ext_crop = img.copy(roi=(x_ext, y_ext, w_ext, h_ext))
#                    ext_bin = ext_crop.binary([THRESHOLDS[0]])
                    rects = img_bin.find_rects(threshold=2000,roi=(x_ext, y_ext, w_ext, h_ext))

                    if rects:

                        for r in rects:
                            rx, ry, rw, rh = r.rect()
                            rcx = rx + rw // 2 + x_ext
                            rcy = ry + rh // 2 + y_ext
                            r_area = rw * rh
                            r_aspect = rw / rh
                            if r_area < 500 or r_area > 55000:
                                continue
                            if rw < 17 or rh < 40:
                                continue
                            if not (0.4545 < r_aspect < 2.2):
                                continue

                            corners = r.corners()
                            pts = [(x + x_ext, y + y_ext) for (x, y) in corners]
                            img.draw_line(pts[0][0], pts[0][1], pts[1][0], pts[1][1], color=(0, 0, 255))
                            img.draw_line(pts[1][0], pts[1][1], pts[2][0], pts[2][1], color=(0, 0, 255))
                            img.draw_line(pts[2][0], pts[2][1], pts[3][0], pts[3][1], color=(0, 0, 255))
                            img.draw_line(pts[3][0], pts[3][1], pts[0][0], pts[0][1], color=(0, 0, 255))
                            img.draw_cross(rcx, rcy, color=(0, 255, 255), size=8)



                offset_x = picture_width // 2 - cx + 10000
                offset_y = picture_height // 2 - cy + 10000
                uart_send_pixel_coordinates(uart, 0x01, offset_x, offset_y)

            Display.show_image(img, x=int((DISPLAY_WIDTH - picture_width) / 2), y=int((DISPLAY_HEIGHT - picture_height) / 2))
            time.sleep_ms(1)

    except Exception as e:
        print(f"发生错误 / Error occurred: {e}")
    finally:
        if 'sensor' in locals() and isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        MediaManager.deinit()

if __name__ == "__main__":
    main()
