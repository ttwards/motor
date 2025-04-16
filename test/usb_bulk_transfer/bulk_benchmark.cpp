#include <libusb-1.0/libusb.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include <atomic>
#include <csignal>

#define VID 0x1209
#define PID 0x0001
#define BULK_OUT_EP1 0x01
#define BULK_OUT_EP2 0x02
#define PACKET_SIZE 64

std::atomic<bool> running(true);

void signal_handler(int signum) {
    running = false;
    std::cerr << "\n收到中断信号，正在退出..." << std::endl;
}

void check_usb_error(int ret, const char* operation) {
    if (ret != LIBUSB_SUCCESS) {
        std::cerr << "USB错误: " << operation << " - " << libusb_error_name(ret) << std::endl;
        if (ret == LIBUSB_ERROR_NO_DEVICE) running = false;
    }
}

int main() {
    // 注册信号处理
    signal(SIGINT, signal_handler);
    
    // 初始化libusb
    libusb_context* ctx = nullptr;
    int ret = libusb_init(&ctx);
    if (ret < 0) {
        std::cerr << "无法初始化libusb: " << libusb_error_name(ret) << std::endl;
        return 1;
    }

    // 打开设备
    libusb_device_handle* dev = libusb_open_device_with_vid_pid(ctx, VID, PID);
    if (!dev) {
        std::cerr << "未找到设备 " << std::hex << VID << ":" << PID << std::dec << std::endl;
        libusb_exit(ctx);
        return 1;
    }

    // 配置设备
    if (libusb_kernel_driver_active(dev, 0)) {
        ret = libusb_detach_kernel_driver(dev, 0);
        check_usb_error(ret, "解除内核驱动");
    }
    
    ret = libusb_claim_interface(dev, 0);
    check_usb_error(ret, "声明接口");
    if (ret != LIBUSB_SUCCESS) {
        libusb_close(dev);
        libusb_exit(ctx);
        return 1;
    }

    // 准备数据包
    uint8_t packet[PACKET_SIZE] = {0x5A, 0x5A, 0x20, 0x48}; // 头4字节

    // 统计变量
    uint64_t total_packets = 0;
    auto start_time = std::chrono::steady_clock::now();
    
    // 主循环
    int endpoint = BULK_OUT_EP1;
    while (running) {
        int transferred = 0;
        ret = libusb_bulk_transfer(dev, endpoint, packet, sizeof(packet), &transferred, 0);
        
        if (ret == LIBUSB_SUCCESS && transferred == sizeof(packet)) {
            total_packets++;
            
            // 每秒显示统计
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start_time).count();
            if (elapsed >= 1.0) {
                double rate = total_packets / elapsed;
                std::cout << "\r" << std::fixed << std::setprecision(1)
                          << "频率: " << rate << " pkts/s | "
                          << "速度: " << (rate * PACKET_SIZE) / 1024 << " KB/s"
                          << std::flush;
                start_time = now;
                total_packets = 0;
            }
        } else {
            check_usb_error(ret, "批量传输");
        }
        
        // 切换端点
        endpoint = (endpoint == BULK_OUT_EP1) ? BULK_OUT_EP2 : BULK_OUT_EP1;
    }
    // 清理
    libusb_release_interface(dev, 0);
    libusb_close(dev);
    libusb_exit(ctx);
    std::cout << "\n设备已释放" << std::endl;
    return 0;
}
