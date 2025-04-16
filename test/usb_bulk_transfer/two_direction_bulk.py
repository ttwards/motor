import usb.core
import usb.util
import sys
import time

# --- 配置参数 (匹配 C 代码) ---
ARES_USB_VID = 0x1209
ARES_USB_PID = 0x0001
BULK_EP_OUT_ADDR = 0x01  # OUT 端点地址 (Host to Device)
BULK_EP_IN_ADDR = 0x81   # IN 端点地址 (Device to Host)

def find_device(vid, pid):
    """查找指定 VID/PID 的 USB 设备"""
    dev = usb.core.find(idVendor=vid, idProduct=pid)
    if dev is None:
        print(f"错误：找不到设备 VID={vid:#06x} PID={pid:#06x}")
        print("请确保设备已连接并且驱动程序（如果需要）已安装。")
        print("在 macOS 上，请确保已安装 libusb (brew install libusb)。")
        sys.exit(1)
    print(f"找到设备: VID={vid:#06x} PID={pid:#06x}")
    return dev

def main():
    # 1. 查找设备
    dev = find_device(ARES_USB_VID, ARES_USB_PID)

    # 2. 设置配置
    #   - 对于某些操作系统 (特别是 Linux)，内核驱动可能需要先分离
    needs_kernel_detach = False
    try:
        if dev.is_kernel_driver_active(0):
            needs_kernel_detach = True
            dev.detach_kernel_driver(0)
            print("已分离内核驱动")
    except usb.core.USBError as e:
        # is_kernel_driver_active 在某些系统（如 Windows 或特定配置的 macOS）上可能不可用或出错
        print(f"检查/分离内核驱动时出错 (可能正常): {e}")
        pass # 继续尝试

    #   - 设置设备配置 (通常是第一个配置)
    try:
        dev.set_configuration()
        print("已设置设备配置")
    except usb.core.USBError as e:
        print(f"错误：设置配置失败: {e}")
        sys.exit(1)

    # 3. 获取活动的配置和接口
    cfg = dev.get_active_configuration()
    intf = cfg[(0, 0)] # 假设是第一个接口

    # 4. 查找端点
    ep_out = usb.util.find_descriptor(
        intf,
        # 匹配 OUT 端点地址
        custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT and \
                                e.bEndpointAddress == BULK_EP_OUT_ADDR
    )

    ep_in = usb.util.find_descriptor(
        intf,
        # 匹配 IN 端点地址
        custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN and \
                                e.bEndpointAddress == BULK_EP_IN_ADDR
    )

    if ep_out is None:
        print(f"错误：找不到 Bulk OUT 端点 (地址 {BULK_EP_OUT_ADDR:#04x})")
        sys.exit(1)

    if ep_in is None:
        print(f"错误：找不到 Bulk IN 端点 (地址 {BULK_EP_IN_ADDR:#04x})")
        sys.exit(1)

    print(f"找到 OUT 端点: 地址 {ep_out.bEndpointAddress:#04x}, 最大包大小: {ep_out.wMaxPacketSize}")
    print(f"找到 IN 端点: 地址 {ep_in.bEndpointAddress:#04x}, 最大包大小: {ep_in.wMaxPacketSize}")

    # 5. 通信循环
    print("\n输入要发送的十六进制字节 (例如: 01 A2 FF)，或输入 'quit' 退出。")
    while True:
        try:
            user_input = input("发送 > ").strip()

            if user_input.lower() == 'quit':
                break

            if not user_input:
                continue

            # 清理并转换输入为字节
            hex_string = ''.join(user_input.split()) # 移除空格
            if len(hex_string) % 2 != 0:
                print("错误：十六进制字符串长度必须是偶数。")
                continue
            try:
                data_to_send = bytes.fromhex(hex_string)
            except ValueError:
                print("错误：无效的十六进制字符。")
                continue

            # 发送数据
            try:
                print(f"  发送 ({len(data_to_send)}字节): {data_to_send.hex(' ').upper()}")
                bytes_sent = ep_out.write(data_to_send)
                print(f"  实际发送: {bytes_sent} 字节")
                time.sleep(0.1) # 短暂延时，给设备处理时间

            except usb.core.USBError as e:
                print(f"  发送错误: {e}")
                # 尝试清除 halt 状态（如果端点停滞）
                if e.errno == 32: # errno.EPIPE Broken pipe
                    try:
                        print("  尝试清除 OUT 端点 Halt 状态...")
                        ep_out.clear_halt()
                    except usb.core.USBError as clear_e:
                         print(f"  清除 OUT 端点 Halt 状态失败: {clear_e}")

                continue # 继续下一次循环

            # 尝试接收数据
            try:
                # 读取最多 ep_in.wMaxPacketSize 字节，设置超时 (毫秒)
                # 注意：你的 C 代码可能不会立即回传数据，这里可能会超时
                print("  尝试接收...")
                # --- 这里是需要修改的地方 ---
                # 原来的代码可能直接用 data_received.hex()
                data_array = ep_in.read(ep_in.wMaxPacketSize, timeout=1000) # 1秒超时

                if len(data_array) > 0:
                    # *** 修改点：先转换为 bytes ***
                    data_bytes = data_array.tobytes()
                    # *** 再调用 .hex() ***
                    hex_data = data_bytes.hex(' ').upper()
                    print(f"  接收 ({len(data_bytes)}字节): {hex_data}")
                # else:
                    # 可选: 如果读取成功但长度为0
                    # print("  收到空数据包")

            except usb.core.USBTimeoutError: # 使用更具体的超时异常
                # 超时是常见情况，特别是如果设备端没有主动发送数据
                print("  接收超时 (设备端可能没有发送数据)")
            except usb.core.USBError as e:
                # 处理其他 USB 错误
                print(f"  接收错误: {e}")
                # ... (可以加上之前的清除Halt状态的代码)

        except KeyboardInterrupt:
            print("\n检测到 Ctrl+C，正在退出...")
            break
        except EOFError: # 处理管道输入结束的情况
             print("\n输入流结束，正在退出...")
             break

    # 6. 清理资源
    print("\n释放 USB 资源...")
    usb.util.dispose_resources(dev)

    # 如果之前分离了内核驱动，尝试重新附加 (虽然通常不需要手动做)
    # if needs_kernel_detach:
    #     try:
    #         dev.attach_kernel_driver(0)
    #         print("已重新附加内核驱动")
    #     except usb.core.USBError as e:
    #         print(f"重新附加内核驱动时出错 (可能正常): {e}")

    print("脚本结束。")

if __name__ == "__main__":
    main()
