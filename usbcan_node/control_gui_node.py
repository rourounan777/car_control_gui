#!/usr/bin/env python3
import rclpy                          # ROS2 客户端库
from rclpy.node import Node           # ROS2 节点基类
from std_msgs.msg import String       # 使用 std_msgs/String 消息类型
from ctypes import *                  # 用于加载 C 动态链接库及定义结构体
import struct                         # 用于二进制数据打包
import threading                      # 用于线程管理
import queue                          # 用于线程安全队列
import tkinter as tk                  # Tkinter GUI 库
from tkinter import END               # 用于 Listbox 操作

# 全局队列：保存发送的 CAN 数据帧（HEX 字符串），GUI 定时读取该队列
transmit_log_queue = queue.Queue()

# ------------------------- CAN 设备相关定义 -------------------------
DevType = c_uint                      # 设备类型，使用无符号整型表示
USBCAN2 = DevType(4)                  # 设备类型编号4表示 USB-CAN2
DevIndex = c_uint(0)                  # 设备索引，0 表示第一个设备
Channel1 = c_uint(0)                  # 定义 CAN 通道1
Channel2 = c_uint(1)                  # 定义 CAN 通道2
STATUS_OK = 1                         # 操作成功状态码

# ------------------------- 结构体定义 -------------------------
class BoardInfo(Structure):
    _fields_ = [
        ("hw_Version", c_ushort),     # 硬件版本（2字节）
        ("fw_Version", c_ushort),     # 固件版本（2字节）
        ("dr_Version", c_ushort),     # 驱动版本（2字节）
        ("in_Version", c_ushort),     # 接口版本（2字节）
        ("irq_Num", c_ushort),        # 中断数量（2字节）
        ("can_Num", c_byte),          # CAN 通道数量（1字节）
        ("str_Serial_Num", c_byte * 20),  # 序列号（20字节）
        ("str_hw_Type", c_byte * 40),     # 硬件类型（40字节）
        ("Reserved", c_byte * 4)          # 保留字段（4字节）
    ]

# 定义 CAN 数据帧结构体
class CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint),               # CAN 帧标识符
        ("TimeStamp", c_uint),        # 时间戳
        ("TimeFlag", c_byte),         # 时间标志
        ("SendType", c_byte),         # 发送类型
        ("RemoteFlag", c_byte),       # 远程帧标志（0 表示数据帧）
        ("ExternFlag", c_byte),       # 扩展帧标志（0：标准帧，1：扩展帧）
        ("DataLen", c_byte),          # 数据长度（0～8字节）
        ("data", c_ubyte * 8),        # 数据内容，最多8字节
        ("Reserved", c_byte * 3)      # 保留字段
    ]

# 定义 CAN 初始化配置结构体
class INIT_CONFIG(Structure):
    _fields_ = [
        ("acccode", c_uint32),        # 验收码
        ("accmask", c_uint32),        # 屏蔽码
        ("reserved", c_uint32),       # 保留字段
        ("filter", c_byte),           # 滤波设置（0 或 1）
        ("timing0", c_byte),          # 定时参数0（决定波特率）
        ("timing1", c_byte),          # 定时参数1（决定波特率）
        ("mode", c_byte)              # 工作模式（0 表示正常）
    ]

# ------------------------- ECAN 类（封装 CAN 操作） -------------------------
class ECAN(object):
    def __init__(self):
        try:
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

# ------------------------- ROS2 节点：CarControlNode -------------------------
class CarControlNode(Node):
    def __init__(self):
        super().__init__('control_gui_node')
        self.ecan = ECAN()
        self.device_open = False

        # 打开 CAN 设备，使用 500k 波特率，验收码为 0x00000000，屏蔽码为 0xFFFFFFFF，不使能滤波
        if self.open_device("500k", "500k", 0x00000000, 0xFFFFFFFF, 0):
            self.get_logger().info("CAN 设备打开成功")
        else:
            self.get_logger().error("CAN 设备初始化失败")

        # 订阅 ROS2 话题 'car_cmd'，接收控制指令（例如 forward、backward、left、right、stop）
        self.create_subscription(String, 'car_cmd', self.car_cmd_callback, 10)

    def open_device(self, baud_can1, baud_can2, acccode, accmask, filter):
        initconfig = INIT_CONFIG()
        initconfig.acccode = acccode
        initconfig.accmask = accmask
        initconfig.filter = filter

        # 打开设备
        if self.ecan.OpenDevice(USBCAN2, DevIndex) != STATUS_OK:
            self.get_logger().error("打开设备失败!")
            return False

        # 初始化通道1（设置波特率参数）
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

        # 启动两个 CAN 通道
        if (self.ecan.StartCan(USBCAN2, DevIndex, Channel1) != STATUS_OK or 
            self.ecan.StartCan(USBCAN2, DevIndex, Channel2) != STATUS_OK):
            self.ecan.CloseDevice(USBCAN2, DevIndex)
            self.get_logger().error("启动 CAN 失败!")
            return False

        self.device_open = True
        return True

    def get_timing(self, baud):
        # 根据波特率字符串返回对应的定时参数（timing0, timing1）
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
        # 默认返回 500k 参数
        return timing.get(baud, (0x00, 0x1C))

    def send_can_command(self, can_channel, can_obj):
        # 调用 ECAN 库函数发送 CAN 数据帧
        if self.device_open:
            ret = self.ecan.Tramsmit(USBCAN2, DevIndex, can_channel, can_obj)
            if ret != STATUS_OK:
                self.get_logger().error("CAN 指令发送失败!")
            else:
                self.get_logger().info("CAN 指令发送成功")
        else:
            self.get_logger().error("设备未打开!")

    def car_cmd_callback(self, msg):
        # ROS2 消息回调函数，当接收到控制指令时被调用
        command = msg.data.strip().lower()  # 将指令转换为小写，方便匹配
        self.get_logger().info(f"收到控制指令: {command}")

        # 根据混控协议构造 8 字节数据包，格式为：
        # 帧头 (2B) + 角速度 (2B) + 直行速度 (2B) + 校验和 (2B)
        # 校验和 = 0xCDAB XOR (角速度【无符号】) XOR (直行速度【无符号】)
        angular_speed = 0   # 角速度（转弯命令）
        straight_speed = 0  # 直行速度

        # 根据不同指令赋予示例数值（实际数值可根据需求调整，取值范围 -1000～1000）
        if command == "forward":
            angular_speed = 0
            straight_speed = 500
        elif command == "backward":
            angular_speed = 0
            straight_speed = -500
        elif command == "left":
            angular_speed = 500
            straight_speed = 0
        elif command == "right":
            angular_speed = -500
            straight_speed = 0
        elif command == "stop":
            angular_speed = 0
            straight_speed = 0
        else:
            self.get_logger().warn(f"未知指令: {command}")
            return

        header = 0xCDAB  # 固定帧头
        # 辅助函数：将有符号16位转换为无符号16位（取低16位）
        def to_unsigned(val):
            return val & 0xFFFF
        ang_val = to_unsigned(angular_speed)
        str_val = to_unsigned(straight_speed)
        checksum = header ^ ang_val ^ str_val  # 计算校验和

        # 使用大端格式打包成 8 字节数据包
        packet = struct.pack(">HHHH", header, ang_val, str_val, checksum)
        # 将构造好的数据包 HEX 字符串写入日志（同时也放入全局队列，供 GUI 显示）
        hex_str = packet.hex()
        self.get_logger().info(f"发送数据: {hex_str}")
        transmit_log_queue.put(f"指令: {command} -> {hex_str}")

        # 构造 CAN 帧结构体，并填充数据
        can_obj = CAN_OBJ()
        can_obj.ID = 0x1314           # 按协议使用扩展帧 ID 0x1314
        can_obj.TimeStamp = 0
        can_obj.TimeFlag = 0
        can_obj.SendType = 0
        can_obj.RemoteFlag = 0        # 数据帧（非远程帧）
        can_obj.ExternFlag = 1        # 扩展帧
        can_obj.DataLen = 8           # 数据长度为8字节
        for i in range(8):
            can_obj.data[i] = packet[i]

        # 通过 CAN 通道1发送数据帧
        self.send_can_command(Channel1, can_obj)

    def close_device(self):
        # 关闭 CAN 设备，清理资源
        if self.device_open:
            self.ecan.CloseDevice(USBCAN2, DevIndex)
            self.device_open = False
            self.get_logger().info("CAN 设备已关闭")

# ------------------------- GUI 部分 -------------------------
class CarControlGUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node  # ROS2 节点实例，用于发布指令
        self.root.title("车辆控制")

        # 创建 ROS2 发布器，发布到 'car_cmd' 话题
        self.cmd_pub = self.node.create_publisher(String, 'car_cmd', 10)

        # 创建控制按钮
        btn_forward = tk.Button(root, text="前进", width=10, command=lambda: self.send_cmd("forward"))
        btn_backward = tk.Button(root, text="后退", width=10, command=lambda: self.send_cmd("backward"))
        btn_left = tk.Button(root, text="左转", width=10, command=lambda: self.send_cmd("left"))
        btn_right = tk.Button(root, text="右转", width=10, command=lambda: self.send_cmd("right"))
        btn_stop = tk.Button(root, text="停止", width=10, command=lambda: self.send_cmd("stop"))

        # 使用 grid 布局安排按钮位置
        btn_forward.grid(row=0, column=1, padx=5, pady=5)
        btn_left.grid(row=1, column=0, padx=5, pady=5)
        btn_stop.grid(row=1, column=1, padx=5, pady=5)
        btn_right.grid(row=1, column=2, padx=5, pady=5)
        btn_backward.grid(row=2, column=1, padx=5, pady=5)

        # 创建 Listbox 显示 CAN 数据日志
        self.listbox = tk.Listbox(root, width=50, height=10)
        self.listbox.grid(row=3, column=0, columnspan=3, padx=10, pady=10)

        # 启动定时更新日志
        self.update_log()

    def send_cmd(self, cmd):
        """发送控制指令到 ROS2 话题 'car_cmd'"""
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.node.get_logger().info(f"发送指令: {cmd}")

    def update_log(self):
        """从全局队列中读取 CAN 数据日志并显示在 Listbox 中"""
        while not transmit_log_queue.empty():
            msg = transmit_log_queue.get()
            self.listbox.insert(END, msg)
            self.listbox.yview(END)  # 自动滚动到最新日志
        self.root.after(100, self.update_log)  # 每 100ms 检查一次队列

    def on_closing(self):
        """窗口关闭时的清理操作"""
        self.node.close_device()  # 关闭 CAN 设备
        rclpy.shutdown()          # 关闭 ROS2
        self.root.destroy()       # 销毁 GUI 窗口

# ------------------------- 主函数 -------------------------
def main():
    rclpy.init()                     # 初始化 ROS2 客户端库
    node = CarControlNode()          # 创建 ROS2 节点实例

    # 创建 ROS2 线程，运行 rclpy.spin()，保证节点可以接收消息
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # 创建 Tkinter GUI 窗口
    root = tk.Tk()
    gui = CarControlGUI(root, node)  # 传入 ROS2 节点实例

    # 设置窗口关闭协议
    root.protocol("WM_DELETE_WINDOW", gui.on_closing)
    root.mainloop()

if __name__ == '__main__':
    main()