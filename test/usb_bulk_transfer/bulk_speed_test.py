import usb.core
import usb.util
import time
import os
import sys
import threading

# 设备配置
VID = 0x1209
PID = 0x0001
INTERFACE = 0
ENDPOINTS = [0x01, 0x02]  # 使用的OUT端点

def setup_device():
    # 查找设备
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        raise ValueError("设备未找到，请检查连接")

    # 断开内核驱动
    if dev.is_kernel_driver_active(INTERFACE):
        dev.detach_kernel_driver(INTERFACE)

    # 设置配置
    try:
        dev.set_configuration()
    except usb.core.USBError as e:
        print("配置设置失败，可能已被设置", file=sys.stderr)

    # 声明接口
    usb.util.claim_interface(dev, INTERFACE)
    return dev

def generate_packet():
    """生成符合要求的数据包"""
    header = bytes([0x5A, 0x5A, 0x20, 0x48])
    payload = os.urandom(16)  # 16字节随机数据
    return header + payload

class USBSender:
    def __init__(self, dev):
        self.dev = dev
        self.running = False
        self.frequency = 10  # 默认10Hz
        self.packet_count = 0
        self.total_bytes = 0
        self.endpoint_idx = 0

    def send_loop(self):
        interval = 1.0 / self.frequency
        next_time = time.monotonic()
        
        while self.running:
            # 生成和发送数据包
            packet = generate_packet()
            endpoint = ENDPOINTS[self.endpoint_idx]
            self.dev.write(endpoint, packet)
            self.endpoint_idx = (self.endpoint_idx + 1) % len(ENDPOINTS)
            
            # 更新统计
            self.packet_count += 1
            self.total_bytes += len(packet)
            
            # 精确控制发送间隔
            next_time += interval
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.send_loop)
            self.thread.start()
            print(f"开始发送，频率: {self.frequency}Hz")

    def stop(self):
        if self.running:
            self.running = False
            self.thread.join()
            print("发送已停止")

    def set_frequency(self, freq):
        if freq > 0:
            self.frequency = freq
            print(f"频率已更新为: {freq}Hz")
        else:
            print("频率必须大于0")

def monitor_stats(sender):
    """监控并显示统计信息的线程"""
    while sender.running:
        time.sleep(1)  # 每秒更新一次
        if sender.packet_count > 0:
            print(f"\r统计: {sender.packet_count}包/秒 | {sender.total_bytes}字节/秒", end="")
            sender.packet_count = 0
            sender.total_bytes = 0

def main():
    dev = setup_device()
    sender = USBSender(dev)
    print("设备初始化完成")
    
    # 启动监控线程
    stat_thread = threading.Thread(target=monitor_stats, args=(sender,))
    stat_thread.daemon = True
    stat_thread.start()

    try:
        while True:
            cmd = input("\n输入频率(Hz)或'q'退出: ").strip().lower()
            if cmd == 'q':
                break
            try:
                freq = float(cmd)
                sender.set_frequency(freq)
                if not sender.running:
                    sender.start()
            except ValueError:
                print("请输入有效的数字频率值")
                
    except KeyboardInterrupt:
        print("\n收到中断信号")
    finally:
        sender.stop()
        usb.util.release_interface(dev, INTERFACE)
        dev.attach_kernel_driver(INTERFACE)
        print("设备资源已释放")

if __name__ == "__main__":
    main()
