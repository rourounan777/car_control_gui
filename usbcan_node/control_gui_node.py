#!/usr/bin/env python3
"""
本程序基于 ROS2 与 CAN 总线通信，实现车辆控制，
同时通过 Tkinter 提供图形界面，使用户通过按钮和滑块发送控制指令。

主要功能：
  - 通过 ROS2 话题 "car_cmd" 发送控制指令（前进、后退、左转、右转、停止）。
  - 根据控制指令构造 CAN 数据帧，并调用 CAN 动态库传输数据。
  - Tkinter 界面支持实时调节直行速度和转向（角）速度（范围：100～1000，步长50）。
  - 方向按钮支持连续发送指令，直到点击“停止”。

作者：Your Name
日期：202X-XX-XX
"""

import rclpy                          # ROS2 客户端库
from rclpy.node import Node           # ROS2 节点基类
from std_msgs.msg import String       # ROS2 标准消息类型
from ctypes import *                  # 用于加载动态链接库及定义 C 结构体
import struct                         # 用于二进制数据打包
import threading                      # 线程管理
import queue                          # 线程安全队列
import tkinter as tk                  # Tkinter 图形界面库
from tkinter import END               # Listbox 操作常量

# 定义全局队列：用于存储构造完成的 CAN 数据帧日志（以十六进制字符串形式），供 GUI 界面显示
transmit_log_queue = queue.Queue()

# -------------------- CAN 设备相关常量 --------------------
DevType = c_uint                     # 设备类型（无符号整型）
USBCAN2 = DevType(4)                 # 表示 USB-CAN2 设备
DevIndex = c_uint(0)                 # 设备索引，0 表示第一个设备
Channel1 = c_uint(0)                 # CAN 通道1
Channel2 = c_uint(1)                 # CAN 通道2
STATUS_OK = 1                        # 操作成功返回码

# -------------------- 结构体定义 --------------------
class BoardInfo(Structure):
    _fields_ = [
        ("hw_Version", c_ushort),             # 硬件版本（2 字节）
        ("fw_Version", c_ushort),             # 固件版本（2 字节）
        ("dr_Version", c_ushort),             # 驱动版本（2 字节）
        ("in_Version", c_ushort),             # 接口版本（2 字节）
        ("irq_Num", c_ushort),                # 中断数量（2 字节）
        ("can_Num", c_byte),                  # CAN 通道数量（1 字节）
        ("str_Serial_Num", c_byte * 20),      # 序列号（20 字节）
        ("str_hw_Type", c_byte * 40),         # 硬件类型（40 字节）
        ("Reserved", c_byte * 4)              # 保留字段（4 字节）
    ]

class CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint),               # CAN 帧标识符
        ("TimeStamp", c_uint),        # 时间戳
        ("TimeFlag", c_byte),         # 时间标志
        ("SendType", c_byte),         # 发送类型
        ("RemoteFlag", c_byte),       # 远程帧标志（0：数据帧）
        ("ExternFlag", c_byte),       # 扩展帧标志（0：标准帧，1：扩展帧）
        ("DataLen", c_byte),          # 数据长度（0～8 字节）
        ("data", c_ubyte * 8),        # 数据内容，最多 8 字节
        ("Reserved", c_byte * 3)      # 保留字段
    ]

class INIT_CONFIG(Structure):
    _fields_ = [
        ("acccode", c_uint32),        # 验收码
        ("accmask", c_uint32),        # 屏蔽码
        ("reserved", c_uint32),       # 保留字段
        ("filter", c_byte),           # 滤波设置（0 或 1）
        ("timing0", c_byte),          # 定时参数0（决定波特率）
        ("timing1", c_byte),          # 定时参数1（决定波特率）
        ("mode", c_byte)              # 工作模式（0：正常模式）
    ]

# -------------------- ECAN 类封装 --------------------
class ECAN(object):
    """
    封装 CAN 设备操作，将动态链接库的函数进行包装调用
    """
    def __init__(self):
        try:
            # 尝试加载动态库 libECanVci.so.1
            self.dll = cdll.LoadLibrary("libECanVci.so.1")
        except Exception as e:
            print(f"加载 libECanVci.so.1 失败: {e}")
            self.dll = None

    def OpenDevice(self, DeviceType, DeviceIndex):
        return self.dll.OpenDevice(DeviceType, DeviceIndex, 0) if self.dll else 0

    def CloseDevice(self, DeviceType, DeviceIndex):
        return self.dll.CloseDevice(DeviceType, DeviceIndex) if self.dll else 0

    def InitCan(self, DeviceType, DeviceIndex, CanInd, Initconfig):
        return self.dll.InitCAN(DeviceType, DeviceIndex, CanInd, byref(Initconfig)) if self.dll else 0

    def StartCan(self, DeviceType, DeviceIndex, CanInd):
        return self.dll.StartCAN(DeviceType, DeviceIndex, CanInd) if self.dll else 0

    def Receivce(self, DeviceType, DeviceIndex, CanInd, length):
        recmess = (CAN_OBJ * length)()
        ret = self.dll.Receive(DeviceType, DeviceIndex, CanInd, byref(recmess), length, 0) if self.dll else 0
        return length, recmess, ret

    def Tramsmit(self, DeviceType, DeviceIndex, CanInd, mcanobj):
        return self.dll.Transmit(DeviceType, DeviceIndex, CanInd, byref(mcanobj), c_uint16(1)) if self.dll else 0

# -------------------- 车速和命令解析辅助函数 --------------------
def parse_command_data(data: str) -> (str, int):
    """
    解析控制指令字符串，格式为 "command,speed"。
    如果没有给定速度，则返回默认值 100。
    例如：
      "forward,500" 解析为 ("forward", 500)
      "stop" 解析为 ("stop", 100)
    参数:
        data: 控制指令字符串
    返回:
        command: 控制命令字符串（小写）
        speed: 数值型速度幅值（整数）
    """
    parts = data.split(',')
    command = parts[0].strip().lower()
    if len(parts) == 2:
        try:
            speed = int(parts[1].strip())
        except ValueError:
            # 若转换失败，使用默认速度100，并可记录或打印错误信息
            speed = 100
    else:
        speed = 100
    return command, speed

def normalize_angular_speed(command: str, speed: int) -> int:
    """
    规范化转向（角）速度。
    根据命令确定方向：
      - "left" 表示左转（逆时针旋转），应返回正值。
      - "right" 表示右转（顺时针旋转），应返回负值。
    参数:
        command: "left" 或 "right"
        speed: 提供的速度幅值
    返回:
        规范化后的角速度数值
    """
    if command == "left":
        return abs(speed)  # 左转返回正值
    elif command == "right":
        return -abs(speed)  # 右转返回负值
    else:
        return 0

def normalize_straight_speed(command: str, speed: int) -> int:
    """
    规范化直行速度。
    根据命令确定前进或后退：
      - "forward" 表示前进，应返回正值。
      - "backward" 表示后退，应返回负值。
    参数:
        command: "forward" 或 "backward"
        speed: 提供的速度幅值
    返回:
        规范化后的直行速度数值
    """
    if command == "forward":
        return abs(speed)
    elif command == "backward":
        return -abs(speed)
    else:
        return 0

def compute_speed_values(command: str, speed_val: int) -> (int, int):
    """
    根据控制命令和提供的速度幅值，计算直行速度和角速度。
    输出规则：
      - 若命令为 "forward" 或 "backward"，则直行速度按其方向（正前进、负后退），角速度为 0。
      - 若命令为 "left" 或 "right"，则角速度按其方向（左转为正、右转为负），直行速度为 0。
      - 若命令为 "stop"，则两者均为 0。
    参数:
        command: 控制命令（"forward", "backward", "left", "right", "stop"）
        speed_val: 提供的速度幅值
    返回:
        (straight_speed, angular_speed)
    """
    if command in ["forward", "backward"]:
        return normalize_straight_speed(command, speed_val), 0
    elif command in ["left", "right"]:
        return 0, normalize_angular_speed(command, speed_val)
    elif command == "stop":
        return 0, 0
    else:
        return 0, 0

# -------------------- ROS2 节点：CarControlNode --------------------
class CarControlNode(Node):
    """
    ROS2 节点，负责接收界面控制指令，将其解析后封装成 CAN 数据帧，
    并通过 CAN 总线发送出去。
    """
    def __init__(self):
        super().__init__('test_node')
        self.ecan = ECAN()
        self.device_open = False

        # 尝试打开并初始化 CAN 设备，参数：250k 波特率、验收码 0x00000000、屏蔽码 0xFFFFFFFF
        if self.open_device("250k", "250k", 0x00000000, 0xFFFFFFFF, 0):
            self.get_logger().info("CAN 设备打开成功")
        else:
            self.get_logger().error("CAN 设备初始化失败")

        # 订阅 ROS2 话题 "car_cmd"，接收控制消息
        self.create_subscription(String, 'car_cmd', self.car_cmd_callback, 10)

    def open_device(self, baud_can1, baud_can2, acccode, accmask, filter) -> bool:
        """
        打开 CAN 设备，并对两个通道进行初始化和启动。
        参数:
            baud_can1, baud_can2: 分别为通道1和通道2的波特率参数（如 "250k"）
            acccode, accmask: 验收码和屏蔽码
            filter: 滤波设置（0 或 1）
        返回:
            成功返回 True，否则返回 False
        """
        initconfig = INIT_CONFIG()
        initconfig.acccode = acccode
        initconfig.accmask = accmask
        initconfig.filter = filter

        # 打开设备
        if self.ecan.OpenDevice(USBCAN2, DevIndex) != STATUS_OK:
            self.get_logger().error("打开设备失败!")
            return False

        # 初始化通道1
        initconfig.timing0, initconfig.timing1 = self.get_timing(baud_can1)
        initconfig.mode = 0
        if self.ecan.InitCan(USBCAN2, DevIndex, Channel1, initconfig) != STATUS_OK:
            self.ecan.CloseDevice(USBCAN2, DevIndex)
            self.get_logger().error("初始化 CAN1 失败!")
            return False

        # 初始化通道2
        initconfig.timing0, initconfig.timing1 = self.get_timing(baud_can2)
        if self.ecan.InitCan(USBCAN2, DevIndex, Channel2, initconfig) != STATUS_OK:
            self.ecan.CloseDevice(USBCAN2, DevIndex)
            self.get_logger().error("初始化 CAN2 失败!")
            return False

        # 启动两个通道
        if (self.ecan.StartCan(USBCAN2, DevIndex, Channel1) != STATUS_OK or
            self.ecan.StartCan(USBCAN2, DevIndex, Channel2) != STATUS_OK):
            self.ecan.CloseDevice(USBCAN2, DevIndex)
            self.get_logger().error("启动 CAN 失败!")
            return False

        self.device_open = True
        return True

    def get_timing(self, baud: str) -> (int, int):
        """
        根据波特率字符串返回定时参数，用以初始化 CAN 设备。
        参数:
            baud: 波特率描述（如 "250k"）
        返回:
            (timing0, timing1) 元组
        """
        timing = {
            "1M": (0x00, 0x14),
            "800k": (0x00, 0x16),
            "500k": (0x00, 0x1C),
            "250k": (0x01, 0x1C),
            "125k": (0x03, 0x1C),
            "100k": (0x04, 0x1C),
            "50k": (0x09, 0x1C),
            "20k": (0x18, 0x1C),
            "10k": (0x31, 0x1C)
        }
        return timing.get(baud, (0x01, 0x1C))

    def send_can_command(self, can_channel, can_obj):
        """
        将构造好的 CAN 数据帧发送出去。
        参数:
            can_channel: 指定的 CAN 通道
            can_obj: 封装好的 CAN 数据帧结构体
        """
        if self.device_open:
            ret = self.ecan.Tramsmit(USBCAN2, DevIndex, can_channel, can_obj)
            if ret != STATUS_OK:
                self.get_logger().error("CAN 指令发送失败!")
            else:
                self.get_logger().info("CAN 指令发送成功")
        else:
            self.get_logger().error("设备未打开!")

    def car_cmd_callback(self, msg: String):
        """
        ROS2 回调函数，接收到控制指令后进行解析和处理，
        重点处理车速：
          - 对于直行控制，调用 normalize_straight_speed() 确定前进/后退速度。
          - 对于转向控制，调用 normalize_angular_speed() 确定角速度的正负（左转为正，右转为负）。
        然后将数据打包后发送到 CAN 总线。
        """
        # 获取消息数据并转换为小写
        data = msg.data.strip().lower()
        # 解析指令和速度（若未给定则使用默认值 100）
        command, speed_val = parse_command_data(data)
        self.get_logger().info(f"收到控制指令: {data}")

        # 根据命令计算直行速度与角速度
        straight_speed, angular_speed = compute_speed_values(command, speed_val)
        # 详细说明：
        #   对于 "forward" / "backward"：
        #     - straight_speed: 前进取正值，后退取负值
        #     - angular_speed: 0
        #   对于 "left" / "right"：
        #     - angular_speed: 左转返回正值（逆时针），右转返回负值（顺时针）
        #     - straight_speed: 0
        #   "stop" 则两者均为 0

        # 数据打包协议：
        #   - 数据包总长度 8 字节：帧头（2 字节）+ 角速度（2 字节）+ 直行速度（2 字节）+ 校验和（2 字节）
        #   - 校验和为：帧头 XOR 角速度 XOR 直行速度
        header = 0xABCD  # 固定帧头，与厂家设备数据格式对应

        # 将有符号16位转换为无符号16位（仅保留低16位）
        def to_unsigned(value: int) -> int:
            return value & 0xFFFF

        ang_val = to_unsigned(angular_speed)
        str_val = to_unsigned(straight_speed)
        checksum = header ^ ang_val ^ str_val

        # 使用小端序（Little Endian）打包为 8 字节数据包
        packet = struct.pack("<HHHH", header, ang_val, str_val, checksum)
        hex_str = packet.hex()
        self.get_logger().info(f"发送数据: {hex_str}")
        transmit_log_queue.put(f"指令: {data} -> {hex_str}")

        # 构造 CAN 数据帧，写入数据包内容
        can_obj = CAN_OBJ()
        can_obj.ID = 0x1314             # 固定扩展帧 ID，根据协议定义
        can_obj.TimeStamp = 0
        can_obj.TimeFlag = 0
        can_obj.SendType = 0
        can_obj.RemoteFlag = 0          # 数据帧（非远程帧）
        can_obj.ExternFlag = 1          # 扩展帧
        can_obj.DataLen = 8             # 数据长度 8 字节
        for i in range(8):
            can_obj.data[i] = packet[i]

        # 通过 CAN 通道1发送数据帧
        self.send_can_command(Channel1, can_obj)

    def close_device(self):
        """
        关闭 CAN 设备，释放资源
        """
        if self.device_open:
            self.ecan.CloseDevice(USBCAN2, DevIndex)
            self.device_open = False
            self.get_logger().info("CAN 设备已关闭")

# -------------------- GUI 部分 --------------------
class CarControlGUI:
    """
    使用 Tkinter 实现车辆控制图形界面：
      - 方向按钮用于连续发送前进、后退、左转、右转指令。
      - 滑块用于调节直行速度和转向速度（数值范围 100～1000）。
      - “停止”按钮用于停止连续发送命令。
      - 同时实时显示 CAN 数据帧日志。
    """
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("车辆控制")

        # 创建 ROS2 发布器，将控制指令发送到话题 "car_cmd"
        self.cmd_pub = self.node.create_publisher(String, 'car_cmd', 10)

        # 连续发送控制指令的状态变量
        self.repeat_cmd = None
        self.repeat_job = None

        # 创建方向按钮
        self.btn_forward = tk.Button(root, text="前进", width=10, command=lambda: self.start_repeat("forward"))
        self.btn_backward = tk.Button(root, text="后退", width=10, command=lambda: self.start_repeat("backward"))
        self.btn_left = tk.Button(root, text="左转", width=10, command=lambda: self.start_repeat("left"))
        self.btn_right = tk.Button(root, text="右转", width=10, command=lambda: self.start_repeat("right"))
        # 创建停止按钮，用于停止连续发送
        self.btn_stop = tk.Button(root, text="停止", width=10, command=self.stop_repeat)

        # 布局方向按钮
        self.btn_forward.grid(row=0, column=1, padx=5, pady=5)
        self.btn_left.grid(row=1, column=0, padx=5, pady=5)
        self.btn_stop.grid(row=1, column=1, padx=5, pady=5)
        self.btn_right.grid(row=1, column=2, padx=5, pady=5)
        self.btn_backward.grid(row=2, column=1, padx=5, pady=5)

        # 创建直行速度调节滑块，范围 100～1000，步长 50
        self.straight_speed_scale = tk.Scale(root, from_=100, to=1000, resolution=50,
                                             orient=tk.HORIZONTAL, label="直行速度调节")
        self.straight_speed_scale.set(100)
        self.straight_speed_scale.grid(row=3, column=0, columnspan=3, padx=5, pady=5)

        # 创建转向速度调节滑块，范围 100～1000，步长 50
        self.angular_speed_scale = tk.Scale(root, from_=100, to=1000, resolution=50,
                                            orient=tk.HORIZONTAL, label="转向速度调节")
        self.angular_speed_scale.set(100)
        self.angular_speed_scale.grid(row=4, column=0, columnspan=3, padx=5, pady=5)

        # 创建 Listbox，用于显示发送的 CAN 数据帧日志
        self.listbox = tk.Listbox(root, width=50, height=10)
        self.listbox.grid(row=5, column=0, columnspan=3, padx=10, pady=10)

        # 开启定时任务，实时更新日志
        self.update_log()

    def start_repeat(self, cmd: str):
        """
        当用户点击方向按钮时，开始连续发送对应指令。
        参数:
            cmd: 控制命令（"forward", "backward", "left", "right"）
        """
        self.repeat_cmd = cmd
        self.repeat_send()

    def repeat_send(self):
        """
        每 200 毫秒发送一次当前指令，实现连续控制。
        """
        if self.repeat_cmd:
            self.send_cmd(self.repeat_cmd)
            self.repeat_job = self.root.after(200, self.repeat_send)

    def stop_repeat(self):
        """
        停止连续发送指令，并发送一次“停止”命令。
        """
        if self.repeat_job:
            self.root.after_cancel(self.repeat_job)
            self.repeat_job = None
        self.repeat_cmd = None
        self.send_cmd("stop")

    def send_cmd(self, cmd: str):
        """
        根据命令和滑块设置的速度值，构造消息并发布到 ROS2 话题 "car_cmd"。
        逻辑：
          - 对于 "forward"/"backward"：使用直行速度滑块，前进取正数，后退取负数。
          - 对于 "left"/"right"：使用转向速度滑块，左转发送负值（逆时针），右转发送正值（顺时针）。
        参数:
            cmd: 控制命令
        """
        if cmd in ["forward", "backward"]:
            # 获取直行速度
            speed = self.straight_speed_scale.get()
            speed = speed if cmd == "forward" else -speed
            send_msg = f"{cmd},{speed}"
        elif cmd in ["left", "right"]:
            # 获取转向速度
            speed = self.angular_speed_scale.get()
            if cmd == "left":
                speed = -speed  # 左转发送负值
            else:
                speed = speed   # 右转发送正值
            send_msg = f"{cmd},{speed}"
        else:
            send_msg = cmd

        # 构造 ROS2 String 消息并发布
        msg = String()
        msg.data = send_msg
        self.cmd_pub.publish(msg)
        self.node.get_logger().info(f"发送指令: {send_msg}")

    def update_log(self):
        """
        定时检查全局队列中是否有新日志，并显示在 Listbox 中。
        """
        while not transmit_log_queue.empty():
            msg = transmit_log_queue.get()
            self.listbox.insert(END, msg)
            self.listbox.yview(END)  # 自动滚动到底部
        self.root.after(100, self.update_log)

    def on_closing(self):
        """
        GUI 窗口关闭时，调用此函数以关闭 CAN 设备和 ROS2 节点，并销毁窗口。
        """
        self.node.close_device()
        rclpy.shutdown()
        self.root.destroy()

# -------------------- 主函数 --------------------
def main():
    # 初始化 ROS2 客户端库
    rclpy.init()
    # 创建 ROS2 节点实例
    node = CarControlNode()
    
    # 启动 ROS2 循环线程，确保节点可以正常接收消息
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # 创建 Tkinter 根窗口和 GUI 界面
    root = tk.Tk()
    gui = CarControlGUI(root, node)

    # 设置窗口关闭回调
    root.protocol("WM_DELETE_WINDOW", gui.on_closing)
    root.mainloop()

if __name__ == '__main__':
    main()
