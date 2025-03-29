#!/usr/bin/env python3
import rclpy                          # ROS2 客户端库
from rclpy.node import Node           # ROS2 节点基类
from std_msgs.msg import String       # 使用 std_msgs/String 消息类型
from ctypes import *                  # 用于加载 C 动态链接库及定义结构体
import struct                         # 用于二进制数据打包
import threading                      # 用于线程管理（用于GUI部分）
import queue                          # 用于线程安全队列（用于GUI日志）
import tkinter as tk                  # Tkinter GUI 库
from tkinter import END               # 用于 Listbox 操作
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# 全局队列，用于保存发送的 CAN 数据帧（HEX 字符串），GUI 定时读取该队列
transmit_log_queue = queue.Queue()

# ------------------------- CAN 设备相关定义 -------------------------
DevType = c_uint                      # 设备类型（无符号整型）
USBCAN2 = DevType(4)                  # 设备类型编号4表示 USB-CAN2
DevIndex = c_uint(0)                  # 设备索引（通常0表示第一个设备）
Channel1 = c_uint(0)                  # 定义 CAN 通道1
Channel2 = c_uint(1)                  # 定义 CAN 通道2
STATUS_OK = 1                         # 操作成功状态码

# ------------------------- 结构体定义 -------------------------
class BoardInfo(Structure):
    _fields_ = [
        ("hw_Version", c_ushort),
        ("fw_Version", c_ushort),
        ("dr_Version", c_ushort),
        ("in_Version", c_ushort),
        ("irq_Num", c_ushort),
        ("can_Num", c_byte),
        ("str_Serial_Num", c_byte * 20),
        ("str_hw_Type", c_byte * 40),
        ("Reserved", c_byte * 4)
    ]

class CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint),
        ("TimeStamp", c_uint),
        ("TimeFlag", c_byte),
        ("SendType", c_byte),
        ("RemoteFlag", c_byte),
        ("ExternFlag", c_byte),
        ("DataLen", c_byte),
        ("data", c_ubyte * 8),
        ("Reserved", c_byte * 3)
    ]

class INIT_CONFIG(Structure):
    _fields_ = [
        ("acccode", c_uint32),
        ("accmask", c_uint32),
        ("reserved", c_uint32),
        ("filter", c_byte),
        ("timing0", c_byte),
        ("timing1", c_byte),
        ("mode", c_byte)
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
# 此节点接收 ROS2 控制指令，并通过定时器回调发送指令“脉冲”：当收到非stop指令时，
# 仅发送固定时长（如1秒）的运动指令，之后自动切换为stop，确保指令能被及时更新。
class CarControlNode(Node):
    def __init__(self):
        super().__init__('car_control_new_node')
        self.ecan = ECAN()
        self.device_open = False

        # 当前控制指令（默认stop）
        self.current_command = "stop"
        # 命令脉冲持续时间（秒），当 > 0 时，发送该命令；计时结束后自动转为stop
        self.command_duration = 0.0
        self.timer_period = 0.1  # 定时器周期：0.1秒

        if self.open_device("500k", "500k", 0x00000000, 0xFFFFFFFF, 0):
            self.get_logger().info("CAN 设备打开成功")
        else:
            self.get_logger().error("CAN 设备初始化失败")

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.create_subscription(String, 'car_cmd', self.car_cmd_callback, qos_profile)
        self.create_timer(self.timer_period, self.publish_command)

    def open_device(self, baud_can1, baud_can2, acccode, accmask, filter):
        initconfig = INIT_CONFIG()
        initconfig.acccode = acccode
        initconfig.accmask = accmask
        initconfig.filter = filter

        if self.ecan.OpenDevice(USBCAN2, DevIndex) != STATUS_OK:
            self.get_logger().error("打开设备失败!")
            return False

        initconfig.timing0, initconfig.timing1 = self.get_timing(baud_can1)
        initconfig.mode = 0
        if self.ecan.InitCan(USBCAN2, DevIndex, Channel1, initconfig) != STATUS_OK:
            self.ecan.CloseDevice(USBCAN2, DevIndex)
            self.get_logger().error("初始化 CAN1 失败!")
            return False

        initconfig.timing0, initconfig.timing1 = self.get_timing(baud_can2)
        if self.ecan.InitCan(USBCAN2, DevIndex, Channel2, initconfig) != STATUS_OK:
            self.ecan.CloseDevice(USBCAN2, DevIndex)
            self.get_logger().error("初始化 CAN2 失败!")
            return False

        if (self.ecan.StartCan(USBCAN2, DevIndex, Channel1) != STATUS_OK or 
            self.ecan.StartCan(USBCAN2, DevIndex, Channel2) != STATUS_OK):
            self.ecan.CloseDevice(USBCAN2, DevIndex)
            self.get_logger().error("启动 CAN 失败!")
            return False

        self.device_open = True
        return True

    def get_timing(self, baud):
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
        if self.device_open:
            ret = self.ecan.Tramsmit(USBCAN2, DevIndex, can_channel, can_obj)
            if ret != STATUS_OK:
                self.get_logger().error("CAN 指令发送失败!")
            else:
                self.get_logger().info("CAN 指令发送成功")
        else:
            self.get_logger().error("设备未打开!")

    def car_cmd_callback(self, msg):
        command = msg.data.strip().lower()
        self.get_logger().info(f"收到控制指令: {command}")
        # 若接收到非stop指令，设置脉冲持续时间（例如1秒），并更新当前指令
        if command != "stop":
            self.current_command = command
            self.command_duration = 1.0
        else:
            self.current_command = "stop"
            self.command_duration = 0.0

    def publish_command(self):
        # 如果命令持续时间到期，则自动切换为 stop
        if self.command_duration <= 0:
            self.current_command = "stop"
        else:
            # 扣减剩余持续时间
            self.command_duration -= self.timer_period

        # 根据当前指令构造运动参数
        angular_speed = 0
        straight_speed = 0

        if self.current_command == "forward":
            angular_speed = 0
            straight_speed = 500
        elif self.current_command == "backward":
            angular_speed = 0
            straight_speed = -500
        elif self.current_command == "left":
            angular_speed = 500
            straight_speed = 0
        elif self.current_command == "right":
            angular_speed = -500
            straight_speed = 0
        elif self.current_command == "stop":
            angular_speed = 0
            straight_speed = 0
        else:
            return

        header = 0xCDAB
        def to_unsigned(val):
            return val & 0xFFFF
        ang_val = to_unsigned(angular_speed)
        str_val = to_unsigned(straight_speed)
        checksum = header ^ ang_val ^ str_val
        packet = struct.pack(">HHHH", header, ang_val, str_val, checksum)
        hex_str = packet.hex()
        self.get_logger().info(f"发送数据: {hex_str}")
        transmit_log_queue.put(f"指令: {self.current_command} -> {hex_str}")

        can_obj = CAN_OBJ()
        can_obj.ID = 0x1314           # 扩展帧 ID 0x1314
        can_obj.TimeStamp = 0
        can_obj.TimeFlag = 0
        can_obj.SendType = 0
        can_obj.RemoteFlag = 0        # 数据帧
        can_obj.ExternFlag = 1        # 扩展帧
        can_obj.DataLen = 8
        for i in range(8):
            can_obj.data[i] = packet[i]

        self.send_can_command(Channel1, can_obj)

    def close_device(self):
        if self.device_open:
            self.ecan.CloseDevice(USBCAN2, DevIndex)
            self.device_open = False
            self.get_logger().info("CAN 设备已关闭")

# ------------------------- GUI 部分 -------------------------
class CANGui:
    def __init__(self, root):
        self.root = root
        self.root.title("CAN1 传输数据日志")
        self.listbox = tk.Listbox(root, width=50, height=15)
        self.listbox.pack(padx=10, pady=10)
        self.update_log()

    def update_log(self):
        while not transmit_log_queue.empty():
            msg = transmit_log_queue.get()
            self.listbox.insert(END, msg)
            self.listbox.yview(END)
        self.root.after(100, self.update_log)

# ------------------------- 主函数 -------------------------
def main():
    rclpy.init()
    node = CarControlNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    root = tk.Tk()
    gui = CANGui(root)

    def on_closing():
        node.close_device()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == '__main__':
    main()
