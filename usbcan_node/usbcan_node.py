import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from queue import Queue
import threading
import ctypes
import tkinter as tk
from ctypes import *
from tkinter import *
from tkinter import ttk
from tkinter.messagebox import *

# CAN 设备相关定义
DevType = c_uint
USBCAN2 = DevType(4)
DevIndex = c_uint(0)
Channel1 = c_uint(0)
Channel2 = c_uint(1)
STATUS_OK = 1

class BoardInfo(Structure):
    _fields_ = [("hw_Version", c_ushort),
                ("fw_Version", c_ushort),
                ("dr_Version", c_ushort),
                ("in_Version", c_ushort),
                ("irq_Num", c_ushort),
                ("can_Num", c_byte),
                ("str_Serial_Num", c_byte * 20),
                ("str_hw_Type", c_byte * 40),
                ("Reserved", c_byte * 4)]

class CAN_OBJ(Structure):
    _fields_ = [("ID", c_uint),
                ("TimeStamp", c_uint),
                ("TimeFlag", c_byte),
                ("SendType", c_byte),
                ("RemoteFlag", c_byte),
                ("ExternFlag", c_byte),
                ("DataLen", c_byte),
                ("data", c_ubyte * 8),
                ("Reserved", c_byte * 3)]

class INIT_CONFIG(Structure):
    _fields_ = [("acccode", c_uint32),
                ("accmask", c_uint32),
                ("reserved", c_uint32),
                ("filter", c_byte),
                ("timing0", c_byte),
                ("timing1", c_byte),
                ("mode", c_byte)]

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

class UsbCanNode(Node):
    def __init__(self):
        super().__init__('usbcan_node')
        self.ecan = ECAN()
        self.musbcanopen = False
        self.rx_queue_can1 = Queue()
        self.rx_queue_can2 = Queue()
        
        # 订阅 CAN 发送话题
        self.create_subscription(String, 'can1_tx', self.send_can1, 10)
        self.create_subscription(String, 'can2_tx', self.send_can2, 10)
        
        # 创建定时器读取 CAN 消息
        self.timer = self.create_timer(0.01, self.read_can)

    def open_device(self, baud_can1, baud_can2, acccode, accmask, filter):
        if not self.musbcanopen:
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
            if self.ecan.StartCan(USBCAN2, DevIndex, Channel1) != STATUS_OK or \
               self.ecan.StartCan(USBCAN2, DevIndex, Channel2) != STATUS_OK:
                self.ecan.CloseDevice(USBCAN2, DevIndex)
                self.get_logger().error("启动 CAN 失败!")
                return False
            self.musbcanopen = True
            self.get_logger().info("设备打开成功")
            return True
        return True

    def close_device(self):
        if self.musbcanopen:
            self.ecan.CloseDevice(USBCAN2, DevIndex)
            self.musbcanopen = False
            self.get_logger().info("设备已关闭")

    def get_timing(self, baud):
        timing = {
            "1M": (0x00, 0x14), "800k": (0x00, 0x16), "500k": (0x00, 0x1C),
            "250k": (0x01, 0x1C), "125k": (0x03, 0x1C), "100k": (0x04, 0x1C),
            "50k": (0x09, 0x1C), "20k": (0x18, 0x1C), "10k": (0x31, 0x1C)
        }
        return timing.get(baud, (0x01, 0x1C))  # 默认 250k

    def read_can(self):
        if self.musbcanopen:
            length, rec, ret = self.ecan.Receivce(USBCAN2, DevIndex, Channel1, 1)
            if length > 0 and ret == STATUS_OK:
                self.rx_queue_can1.put(rec[0])
            length2, rec2, ret2 = self.ecan.Receivce(USBCAN2, DevIndex, Channel2, 1)
            if length2 > 0 and ret2 == STATUS_OK:
                self.rx_queue_can2.put(rec2[0])

    def send_can1(self, msg):
        if self.musbcanopen:
            try:
                parts = msg.data.split(',')
                id_part = parts[0].split(':')[1]
                data_part = parts[1].split(':')[1].split()
                ext_part = int(parts[2].split(':')[1])
                rtr_part = int(parts[3].split(':')[1])
                canobj = CAN_OBJ()
                canobj.ID = int(id_part, 16)
                canobj.DataLen = len(data_part)
                for i, d in enumerate(data_part):
                    canobj.data[i] = int(d, 16)
                canobj.RemoteFlag = rtr_part
                canobj.ExternFlag = ext_part
                self.ecan.Tramsmit(USBCAN2, DevIndex, Channel1, canobj)
            except Exception as e:
                self.get_logger().error(f"发送 CAN1 消息失败: {e}")

    def send_can2(self, msg):
        if self.musbcanopen:
            try:
                parts = msg.data.split(',')
                id_part = parts[0].split(':')[1]
                data_part = parts[1].split(':')[1].split()
                ext_part = int(parts[2].split(':')[1])
                rtr_part = int(parts[3].split(':')[1])
                canobj = CAN_OBJ()
                canobj.ID = int(id_part, 16)
                canobj.DataLen = len(data_part)
                for i, d in enumerate(data_part):
                    canobj.data[i] = int(d, 16)
                canobj.RemoteFlag = rtr_part
                canobj.ExternFlag = ext_part
                self.ecan.Tramsmit(USBCAN2, DevIndex, Channel2, canobj)
            except Exception as e:
                self.get_logger().error(f"发送 CAN2 消息失败: {e}")

# GUI 和主逻辑
root = Tk()
root.title("EcanTest")
musbcanopen = False
rec_CAN1 = 1
rec_CAN2 = 1

def caninit():
    global musbcanopen, rec_CAN1, rec_CAN2
    if not musbcanopen:
        mbaudcan1 = baudvaluecan1.get()
        mbaudcan2 = baudvaluecan2.get()
        try:
            acccode = int(e_acccode.get(), 16)
            accmask = int(e_accmask.get(), 16)
        except ValueError:
            showerror("错误", "验收码或屏蔽码格式错误")
            return
        filter = filter_var.get()
        if node.open_device(mbaudcan1, mbaudcan2, acccode, accmask, filter):
            musbcanopen = True
            rec_CAN1 = 1
            rec_CAN2 = 1
            btopen.configure(text="关闭设备")
            bt_send_CAN1.configure(state='normal')
            bt_send_CAN2.configure(state='normal')
        else:
            showerror("错误", "设备初始化失败!")
    else:
        node.close_device()
        musbcanopen = False
        btopen.configure(text="打开设备")
        bt_send_CAN1.configure(state='disabled')
        bt_send_CAN2.configure(state='disabled')

def update_listbox_can1():
    global rec_CAN1
    while not node.rx_queue_can1.empty():
        rec = node.rx_queue_can1.get()
        mstr = f"Rec: {rec_CAN1} "
        rec_CAN1 += 1
        mstr += f"Time: {hex(rec.TimeStamp).zfill(8)} " if rec.TimeFlag else "Time: "
        mstr += f"ID: {hex(rec.ID).zfill(8 if rec.ExternFlag else 3)} Format:{'Exten' if rec.ExternFlag else 'Stand'} "
        mstr += "Type:Data  Data: " if rec.RemoteFlag == 0 else "Type:Remote  Data: Remote Request"
        if rec.RemoteFlag == 0:
            mstr += " ".join([hex(rec.data[i]).zfill(2) for i in range(rec.DataLen)])
        listreadcan1.insert(END, mstr)
        listreadcan1.see(END)
    root.after(100, update_listbox_can1)

def update_listbox_can2():
    global rec_CAN2
    while not node.rx_queue_can2.empty():
        rec = node.rx_queue_can2.get()
        mstr = f"Rec: {rec_CAN2} "
        rec_CAN2 += 1
        mstr += f"Time: {hex(rec.TimeStamp).zfill(8)} " if rec.TimeFlag else "Time: "
        mstr += f"ID: {hex(rec.ID).zfill(8 if rec.ExternFlag else 3)} Format:{'Exten' if rec.ExternFlag else 'Stand'} "
        mstr += "Type:Data  Data: " if rec.RemoteFlag == 0 else "Type:Remote  Data: Remote Request"
        if rec.RemoteFlag == 0:
            mstr += " ".join([hex(rec.data[i]).zfill(2) for i in range(rec.DataLen)])
        listreadcan2.insert(END, mstr)
        listreadcan2.see(END)
    root.after(100, update_listbox_can2)

def sendcan1():
    if not musbcanopen:
        showerror("错误", "请先打开设备")
    else:
        try:
            id_val = e_ID_CAN1.get()
            data = [e_Data0_CAN1.get(), e_Data1_CAN1.get(), e_Data2_CAN1.get(),
                    e_Data3_CAN1.get(), e_Data4_CAN1.get(), e_Data5_CAN1.get(),
                    e_Data6_CAN1.get(), e_Data7_CAN1.get()]
            data_str = " ".join([d for d in data if d][:int(e_Length_CAN1.get())])
            ext = ext_CAN1.get()
            rtr = rtr_CAN1.get()
            msg = String()
            msg.data = f"ID:{id_val},Data:{data_str},Ext:{ext},RTR:{rtr}"
            can1_tx_pub.publish(msg)
        except Exception as e:
            showerror("错误", f"发送 CAN1 消息失败: {e}")

def sendcan2():
    if not musbcanopen:
        showerror("错误", "请先打开设备")
    else:
        try:
            id_val = e_ID_CAN2.get()
            data = [e_Data0_CAN2.get(), e_Data1_CAN2.get(), e_Data2_CAN2.get(),
                    e_Data3_CAN2.get(), e_Data4_CAN2.get(), e_Data5_CAN2.get(),
                    e_Data6_CAN2.get(), e_Data7_CAN2.get()]
            data_str = " ".join([d for d in data if d][:int(e_Length_CAN2.get())])
            ext = ext_CAN2.get()
            rtr = rtr_CAN2.get()
            msg = String()
            msg.data = f"ID:{id_val},Data:{data_str},Ext:{ext},RTR:{rtr}"
            can2_tx_pub.publish(msg)
        except Exception as e:
            showerror("错误", f"发送 CAN2 消息失败: {e}")

# GUI 设置
lb1 = Label(root, text="CAN1波特率:", font=("Arial", 12))
lb1.grid(row=1, column=0)
lb2 = Label(root, text="CAN2波特率:", font=("Arial", 12))
lb2.grid(row=2, column=0)
baudvaluecan1 = StringVar(value="250k")
baudvaluecan2 = StringVar(value="250k")
baudvalues = ["1M", "800k", "500k", "250k", "125k", "100k", "50k", "20k", "10k"]
can1com = ttk.Combobox(root, textvariable=baudvaluecan1, values=baudvalues, state="readonly")
can1com.grid(row=1, column=1)
can2com = ttk.Combobox(root, textvariable=baudvaluecan2, values=baudvalues, state="readonly")
can2com.grid(row=2, column=1)
btopen = Button(root, text="打开设备", command=caninit)
btopen.grid(row=1, column=2)

lb_acccode = Label(root, text="验收码(Hex):", font=("Arial", 12))
lb_acccode.grid(row=3, column=0)
e_acccode = Entry(root, font=("Arial", 12))
e_acccode.grid(row=3, column=1)
e_acccode.insert(0, "00000000")
lb_accmask = Label(root, text="屏蔽码(Hex):", font=("Arial", 12))
lb_accmask.grid(row=4, column=0)
e_accmask = Entry(root, font=("Arial", 12))
e_accmask.grid(row=4, column=1)
e_accmask.insert(0, "FFFFFFFF")
filter_var = IntVar()
cb_filter = Checkbutton(root, text="使能滤波", variable=filter_var)
cb_filter.grid(row=5, column=0)

tabcontrol = ttk.Notebook(root)
tab1 = ttk.Frame(tabcontrol)
tab2 = ttk.Frame(tabcontrol)
tabcontrol.add(tab1, text="CAN1")
tabcontrol.add(tab2, text="CAN2")
tabcontrol.grid(row=6, columnspan=3)

# CAN1 GUI
lb_ID_CAN1 = Label(tab1, text="ID(Hex)", font=("Arial", 12))
lb_ID_CAN1.grid(row=0, column=0)
e_ID_CAN1 = Entry(tab1, font=("Arial", 12))
e_ID_CAN1.grid(row=1, column=0)
e_ID_CAN1.insert(0, "00000001")
ext_CAN1 = IntVar()
cb_Ext_CAN1 = Checkbutton(tab1, text="Extended", variable=ext_CAN1)
cb_Ext_CAN1.grid(row=0, column=1)
rtr_CAN1 = IntVar()
cb_Rtr_CAN1 = Checkbutton(tab1, text="RTR", variable=rtr_CAN1)
cb_Rtr_CAN1.grid(row=1, column=1)
lb_Length_CAN1 = Label(tab1, text="Length(0-8):", font=("Arial", 12))
lb_Length_CAN1.grid(row=0, column=2)
e_Length_CAN1 = Entry(tab1, width=3, font=("Arial", 12))
e_Length_CAN1.grid(row=1, column=2)
e_Length_CAN1.insert(0, "8")
lb_Data_CAN1 = Label(tab1, text="Data(Hex)", font=("Arial", 12))
lb_Data_CAN1.grid(row=0, column=3)
e_Data0_CAN1 = Entry(tab1, width=3, font=("Arial", 12))
e_Data0_CAN1.grid(row=1, column=3)
e_Data0_CAN1.insert(0, "00")
e_Data1_CAN1 = Entry(tab1, width=3, font=("Arial", 12))
e_Data1_CAN1.grid(row=1, column=4)
e_Data1_CAN1.insert(0, "00")
e_Data2_CAN1 = Entry(tab1, width=3, font=("Arial", 12))
e_Data2_CAN1.grid(row=1, column=5)
e_Data2_CAN1.insert(0, "00")
e_Data3_CAN1 = Entry(tab1, width=3, font=("Arial", 12))
e_Data3_CAN1.grid(row=1, column=6)
e_Data3_CAN1.insert(0, "00")
e_Data4_CAN1 = Entry(tab1, width=3, font=("Arial", 12))
e_Data4_CAN1.grid(row=1, column=7)
e_Data4_CAN1.insert(0, "00")
e_Data5_CAN1 = Entry(tab1, width=3, font=("Arial", 12))
e_Data5_CAN1.grid(row=1, column=8)
e_Data5_CAN1.insert(0, "00")
e_Data6_CAN1 = Entry(tab1, width=3, font=("Arial", 12))
e_Data6_CAN1.grid(row=1, column=9)
e_Data6_CAN1.insert(0, "00")
e_Data7_CAN1 = Entry(tab1, width=3, font=("Arial", 12))
e_Data7_CAN1.grid(row=1, column=10)
e_Data7_CAN1.insert(0, "00")
bt_send_CAN1 = Button(tab1, text="发送数据", state="disabled", command=sendcan1)
bt_send_CAN1.grid(row=1, column=11)
listreadcan1 = Listbox(tab1, height=20, width=90)
listreadcan1.grid(row=2, column=0, columnspan=12)

# CAN2 GUI（类似 CAN1）
lb_ID_CAN2 = Label(tab2, text="ID(Hex)", font=("Arial", 12))
lb_ID_CAN2.grid(row=0, column=0)
e_ID_CAN2 = Entry(tab2, font=("Arial", 12))
e_ID_CAN2.grid(row=1, column=0)
e_ID_CAN2.insert(0, "00000002")
ext_CAN2 = IntVar()
cb_Ext_CAN2 = Checkbutton(tab2, text="Extended", variable=ext_CAN2)
cb_Ext_CAN2.grid(row=0, column=1)
rtr_CAN2 = IntVar()
cb_Rtr_CAN2 = Checkbutton(tab2, text="RTR", variable=rtr_CAN2)
cb_Rtr_CAN2.grid(row=1, column=1)
lb_Length_CAN2 = Label(tab2, text="Length(0-8):", font=("Arial", 12))
lb_Length_CAN2.grid(row=0, column=2)
e_Length_CAN2 = Entry(tab2, width=3, font=("Arial", 12))
e_Length_CAN2.grid(row=1, column=2)
e_Length_CAN2.insert(0, "8")
lb_Data_CAN2 = Label(tab2, text="Data(Hex)", font=("Arial", 12))
lb_Data_CAN2.grid(row=0, column=3)
e_Data0_CAN2 = Entry(tab2, width=3, font=("Arial", 12))
e_Data0_CAN2.grid(row=1, column=3)
e_Data0_CAN2.insert(0, "00")
e_Data1_CAN2 = Entry(tab2, width=3, font=("Arial", 12))
e_Data1_CAN2.grid(row=1, column=4)
e_Data1_CAN2.insert(0, "00")
e_Data2_CAN2 = Entry(tab2, width=3, font=("Arial", 12))
e_Data2_CAN2.grid(row=1, column=5)
e_Data2_CAN2.insert(0, "00")
e_Data3_CAN2 = Entry(tab2, width=3, font=("Arial", 12))
e_Data3_CAN2.grid(row=1, column=6)
e_Data3_CAN2.insert(0, "00")
e_Data4_CAN2 = Entry(tab2, width=3, font=("Arial", 12))
e_Data4_CAN2.grid(row=1, column=7)
e_Data4_CAN2.insert(0, "00")
e_Data5_CAN2 = Entry(tab2, width=3, font=("Arial", 12))
e_Data5_CAN2.grid(row=1, column=8)
e_Data5_CAN2.insert(0, "00")
e_Data6_CAN2 = Entry(tab2, width=3, font=("Arial", 12))
e_Data6_CAN2.grid(row=1, column=9)
e_Data6_CAN2.insert(0, "00")
e_Data7_CAN2 = Entry(tab2, width=3, font=("Arial", 12))
e_Data7_CAN2.grid(row=1, column=10)
e_Data7_CAN2.insert(0, "00")
bt_send_CAN2 = Button(tab2, text="发送数据", state="disabled", command=sendcan2)
bt_send_CAN2.grid(row=1, column=11)
listreadcan2 = Listbox(tab2, height=20, width=90)
listreadcan2.grid(row=2, column=0, columnspan=12)

def main():
    global node, can1_tx_pub, can2_tx_pub
    rclpy.init()
    node = UsbCanNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    gui_node = rclpy.create_node('gui_node')
    can1_tx_pub = gui_node.create_publisher(String, 'can1_tx', 10)
    can2_tx_pub = gui_node.create_publisher(String, 'can2_tx', 10)
    update_listbox_can1()
    update_listbox_can2()
    root.mainloop()
    node.close_device()
    rclpy.shutdown()

if __name__ == '__main__':
    main()