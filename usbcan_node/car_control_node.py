#!/usr/bin/env python3
# 这是一个 ROS2 节点，通过调用 USB-CAN 驱动库（libECanVci.so.1）与 CAN 设备通信，
# 根据接收到的 ROS2 消息构造数据帧，并通过 CAN 总线向车载控制器发送指令。
import rclpy                          # 导入 ROS2 Python 客户端库
from rclpy.node import Node           # 导入 ROS2 节点基类
from std_msgs.msg import String       # 导入 ROS2 标准消息类型 String
from ctypes import *                  # 导入 ctypes，用于调用 C 动态链接库
import struct                         # 导入 struct，用于打包/解包二进制数据

# ------------------------- CAN 设备相关定义 -------------------------
DevType = c_uint                      # 定义设备类型变量（无符号整型）
USBCAN2 = DevType(4)                  # 设备类型为 USB-CAN2（设备类型编号为4）
DevIndex = c_uint(0)                  # 设备索引（通常0表示第一个设备）
Channel1 = c_uint(0)                  # 定义 CAN 通道1
Channel2 = c_uint(1)                  # 定义 CAN 通道2
STATUS_OK = 1                         # 状态码：1 表示成功

# ------------------------- 结构体定义 -------------------------
# 板卡信息结构体，存储设备的硬件、固件等版本信息
class BoardInfo(Structure):
    _fields_ = [
        ("hw_Version", c_ushort),     # 硬件版本（2字节）
        ("fw_Version", c_ushort),     # 固件版本（2字节）
        ("dr_Version", c_ushort),     # 驱动版本（2字节）
        ("in_Version", c_ushort),     # 接口版本（2字节）
        ("irq_Num", c_ushort),        # 中断数量（2字节）
        ("can_Num", c_byte),          # CAN 通道数量（1字节）
        ("str_Serial_Num", c_byte * 20),  # 序列号（20字节）
        ("str_hw_Type", c_byte * 40),     # 硬件类型字符串（40字节）
        ("Reserved", c_byte * 4)          # 保留字段（4字节）
    ]

# CAN 帧结构体，用于表示一帧 CAN 消息的数据格式
class CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint),               # CAN 帧的标识符
        ("TimeStamp", c_uint),        # 时间戳
        ("TimeFlag", c_byte),         # 是否启用时间戳标志
        ("SendType", c_byte),         # 发送类型（由硬件/驱动定义）
        ("RemoteFlag", c_byte),       # 远程帧标志（0：数据帧，1：远程帧）
        ("ExternFlag", c_byte),       # 扩展帧标志（0：标准帧，1：扩展帧）
        ("DataLen", c_byte),          # 数据长度（0～8）
        ("data", c_ubyte * 8),        # 数据内容，最多8字节
        ("Reserved", c_byte * 3)      # 保留字段
    ]

# CAN 初始化配置结构体，包含验收码、屏蔽码、滤波、定时参数和模式
class INIT_CONFIG(Structure):
    _fields_ = [
        ("acccode", c_uint32),        # 验收码
        ("accmask", c_uint32),        # 屏蔽码
        ("reserved", c_uint32),       # 保留字段
        ("filter", c_byte),           # 滤波使能标志（0或1）
        ("timing0", c_byte),          # 波特率定时参数0
        ("timing1", c_byte),          # 波特率定时参数1
        ("mode", c_byte)              # 工作模式（0 表示正常模式）
    ]

# ------------------------- ECAN 类（封装 CAN 操作） -------------------------
# 该类通过 ctypes 调用动态库中的函数，实现对 CAN 设备的打开、初始化、启动、发送和接收操作。
class ECAN(object):
    def __init__(self):
        try:
            # 尝试加载 CAN 驱动库
            self.dll = cdll.LoadLibrary("libECanVci.so.1")
        except Exception as e:
            print(f"加载 libECanVci.so.1 失败: {e}")
            self.dll = None

    def OpenDevice(self, DeviceType, DeviceIndex):
        # 打开设备，返回状态码
        return self.dll.OpenDevice(DeviceType, DeviceIndex, 0) if self.dll else 0

    def CloseDevice(self, DeviceType, DeviceIndex):
        # 关闭设备
        return self.dll.CloseDevice(DeviceType, DeviceIndex) if self.dll else 0

    def InitCan(self, DeviceType, DeviceIndex, CanInd, Initconfig):
        # 初始化 CAN 通道，传入 CAN 配置结构体
        return self.dll.InitCAN(DeviceType, DeviceIndex, CanInd, byref(Initconfig)) if self.dll else 0

    def StartCan(self, DeviceType, DeviceIndex, CanInd):
        # 启动 CAN 通道
        return self.dll.StartCAN(DeviceType, DeviceIndex, CanInd) if self.dll else 0

    def Receivce(self, DeviceType, DeviceIndex, CanInd, length):
        # 接收 CAN 数据帧，返回长度、数据帧数组和返回状态
        recmess = (CAN_OBJ * length)()
        ret = self.dll.Receive(DeviceType, DeviceIndex, CanInd, byref(recmess), length, 0) if self.dll else 0
        return length, recmess, ret

    def Tramsmit(self, DeviceType, DeviceIndex, CanInd, mcanobj):
        # 发送 CAN 数据帧，返回状态码
        return self.dll.Transmit(DeviceType, DeviceIndex, CanInd, byref(mcanobj), c_uint16(1)) if self.dll else 0

# ------------------------- ROS2 节点：CarControlNode -------------------------
# 该节点用于接收 ROS2 的控制指令，并根据混控模式的协议构造 CAN 数据帧，
# 然后通过 CAN 总线发送给车载控制器。
class CarControlNode(Node):
    def __init__(self):
        # 初始化节点，节点名称为 'car_control_node'
        super().__init__('car_control_node')
        self.ecan = ECAN()          # 创建 ECAN 实例，用于操作 CAN 设备
        self.device_open = False    # 标记 CAN 设备是否已经成功打开

        # 打开 CAN 设备，使用 500k 波特率（两个通道均设置为 500k）
        # 验收码设为 0x00000000，屏蔽码为 0xFFFFFFFF，不启用滤波（filter=0）
        if self.open_device("500k", "500k", 0x00000000, 0xFFFFFFFF, 0):
            self.get_logger().info("CAN 设备打开成功")
        else:
            self.get_logger().error("CAN 设备初始化失败")

        # 订阅 ROS2 话题 'car_cmd'（消息类型 std_msgs/String）
        # 当接收到控制指令（例如 forward, backward, left, right, stop）时，会调用回调函数 car_cmd_callback
        self.create_subscription(String, 'car_cmd', self.car_cmd_callback, 10)

    def open_device(self, baud_can1, baud_can2, acccode, accmask, filter):
        # 创建初始化配置结构体
        initconfig = INIT_CONFIG()
        initconfig.acccode = acccode
        initconfig.accmask = accmask
        initconfig.filter = filter

        # 打开设备
        if self.ecan.OpenDevice(USBCAN2, DevIndex) != STATUS_OK:
            self.get_logger().error("打开设备失败!")
            return False

        # 设置通道1的波特率参数并初始化
        initconfig.timing0, initconfig.timing1 = self.get_timing(baud_can1)
        initconfig.mode = 0  # 设置为正常模式
        if self.ecan.InitCan(USBCAN2, DevIndex, Channel1, initconfig) != STATUS_OK:
            self.ecan.CloseDevice(USBCAN2, DevIndex)
            self.get_logger().error("初始化 CAN1 失败!")
            return False

        # 设置通道2的波特率参数并初始化
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

        self.device_open = True  # 标记设备已经成功打开
        return True

    def get_timing(self, baud):
        # 根据波特率字符串返回对应的定时参数 (timing0, timing1)
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
        # 如果输入的波特率不匹配，默认返回 250k 参数
        return timing.get(baud, (0x01, 0x1C))

    def send_can_command(self, can_channel, can_obj):
        # 通过 ECAN 类的 Tramsmit 函数发送 CAN 数据帧
        if self.device_open:
            ret = self.ecan.Tramsmit(USBCAN2, DevIndex, can_channel, can_obj)
            if ret != STATUS_OK:
                self.get_logger().error("CAN 指令发送失败!")
            else:
                self.get_logger().info("CAN 指令发送成功")
        else:
            self.get_logger().error("设备未打开!")

    def car_cmd_callback(self, msg):
        # 当节点接收到控制指令时，该回调函数被调用
        command = msg.data.strip().lower()  # 获取并标准化控制指令（小写）
        self.get_logger().info(f"收到控制指令: {command}")

        # 根据混控协议构造 8 字节数据包，格式如下：
        # 帧头 (2B) + 角速度 (2B) + 直行速度 (2B) + 校验和 (2B)
        # 校验和 = 0xCDAB XOR (角速度【无符号】) XOR (直行速度【无符号】)
        angular_speed = 0   # 转弯角速度（有符号，取值范围 -1000～1000）
        straight_speed = 0  # 直行速度（有符号）

        # 根据不同控制指令设置角速度和直行速度的示例数值（具体数值可根据实际需求调整）
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

        # 构造数据帧
        header = 0xCDAB  # 固定帧头
        # 辅助函数：将有符号16位转换为无符号16位（取低16位数据）
        def to_unsigned(val):
            return val & 0xFFFF
        ang_val = to_unsigned(angular_speed)
        str_val = to_unsigned(straight_speed)
        checksum = header ^ ang_val ^ str_val  # 按协议计算校验和

        # 使用大端（网络字节序）格式打包数据为 8 字节
        packet = struct.pack(">HHHH", header, ang_val, str_val, checksum)
        self.get_logger().info(f"发送数据: {packet.hex()}")

        # 构造 CAN 帧结构体，填充帧 ID、数据等参数
        can_obj = CAN_OBJ()
        can_obj.ID = 0x1314           # 按协议要求使用扩展帧ID 0x1314
        can_obj.TimeStamp = 0         # 时间戳设为0
        can_obj.TimeFlag = 0          # 不使用时间戳
        can_obj.SendType = 0          # 发送类型（依赖驱动库定义）
        can_obj.RemoteFlag = 0        # 数据帧（非远程帧）
        can_obj.ExternFlag = 1        # 扩展帧标志，1 表示扩展帧
        can_obj.DataLen = 8           # 数据长度固定为8字节
        for i in range(8):
            can_obj.data[i] = packet[i]  # 将打包好的数据复制到 CAN 帧数据区

        # 通过 CAN 通道1发送构造好的 CAN 数据帧
        self.send_can_command(Channel1, can_obj)

    def close_device(self):
        # 节点关闭时调用此方法，关闭 CAN 设备
        if self.device_open:
            self.ecan.CloseDevice(USBCAN2, DevIndex)
            self.device_open = False
            self.get_logger().info("CAN 设备已关闭")

# ------------------------- 主函数 -------------------------
def main():
    # 初始化 ROS2 客户端库
    rclpy.init()
    # 创建 CarControlNode 节点实例
    node = CarControlNode()
    try:
        # 进入 ROS2 事件循环，等待消息回调
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # 退出前关闭 CAN 设备
    node.close_device()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
