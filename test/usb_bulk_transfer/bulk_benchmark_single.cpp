#include <libusb-1.0/libusb.h>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <csignal>
#include <atomic>

#define VID 0x1209
#define PID 0x0001
#define BULK_OUT_EP 0x01
#define PACKET_SIZE 64
#define TIMEOUT_MS 1000  // 添加超时时间

std::atomic<bool> running(true);

void signal_handler(int) {
    running = false;
    std::cerr << "\n正在停止..." << std::endl;
}

int main() {
    libusb_context* ctx = nullptr;
    if (libusb_init(&ctx) < 0) {
        std::cerr << "libusb初始化失败" << std::endl;
        return 1;
    }

    libusb_device_handle* dev = libusb_open_device_with_vid_pid(ctx, VID, PID);
    if (!dev) {
        std::cerr << "设备未连接" << std::endl;
        libusb_exit(ctx);
        return 1;
    }

    // 非必需内核驱动处理（macOS通常不需要）
    if (libusb_kernel_driver_active(dev, 0)) {
        libusb_detach_kernel_driver(dev, 0);
    }

    if (libusb_claim_interface(dev, 0) < 0) {
        std::cerr << "无法声明接口" << std::endl;
        libusb_close(dev);
        libusb_exit(ctx);
        return 1;
    }

    signal(SIGINT, signal_handler);

    uint8_t packet[PACKET_SIZE] = {0x5A, 0x5A, 0x20, 0x48};
    uint64_t total_bytes = 0;
    auto start = std::chrono::steady_clock::now();

    std::cout << "开始发送数据 (Ctrl+C停止)..." << std::endl;

    while (running) {
        int transferred = 0;
        int ret = libusb_bulk_transfer(
            dev, 
            BULK_OUT_EP, 
            packet, 
            sizeof(packet),
            &transferred, 
            TIMEOUT_MS  // 关键修改：添加超时
        );

        if (ret == LIBUSB_SUCCESS && transferred == sizeof(packet)) {
            total_bytes += transferred;
            
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start).count();
            if (elapsed >= 1.0) {
                std::cout << "\r速度: " 
                          << std::fixed << std::setprecision(1)
                          << (total_bytes/elapsed)/1024 << " KB/s"
                          << std::flush;
                start = now;
                total_bytes = 0;
            }
        } else if (ret != LIBUSB_ERROR_TIMEOUT) {
            std::cerr << "\n传输错误: " << libusb_error_name(ret) << std::endl;
            break;
        }
    }

    libusb_release_interface(dev, 0);
    libusb_close(dev);
    libusb_exit(ctx);
    std::cout << "\n已安全退出" << std::endl;
    return 0;
}
