import usb.core
import usb.util
import time
import os
import sys

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

    # 设置配置（部分设备需要显式设置）
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
    payload = bytes([0x00] * 60)  # 60字节的0x00    
    return header + payload

def main():
    dev = setup_device()
    print("设备初始化完成，开始发送数据...")
    
    packet_count = 0
    total_bytes = 0
    start_time = time.monotonic()
    endpoint_idx = 0

    try:
        while True:
            # 生成数据包
            packet = generate_packet()
            
            # 交替使用端点发送
            endpoint = ENDPOINTS[endpoint_idx]
            dev.write(endpoint, packet)
            endpoint_idx = (endpoint_idx + 1) % len(ENDPOINTS)

            # 统计信息
            packet_count += 1
            total_bytes += len(packet)

            # 每秒更新显示
            elapsed = time.monotonic() - start_time
            if elapsed >= 1.0:
                freq = packet_count / elapsed
                byte_rate = total_bytes / elapsed
                print(f"\r频率: {freq:.2f}包/秒 | 速率: {byte_rate/1024:.2f}KB/s", end="")
                packet_count = 0
                total_bytes = 0
                start_time = time.monotonic()

    except KeyboardInterrupt:
        print("\n用户中断操作")
    finally:
        # 清理操作
        usb.util.release_interface(dev, INTERFACE)
        dev.attach_kernel_driver(INTERFACE)
        print("设备资源已释放")

if __name__ == "__main__":
    main()
