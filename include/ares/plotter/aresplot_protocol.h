// aresplot_mcu.h

#ifndef ARESPLOT_MCU_H
#define ARESPLOT_MCU_H

#include <stddef.h> // For size_t
#include <stdint.h> // For standard integer types like uint8_t, uint32_t

// --- 用户配置区 User Configuration Section ---

// 定义MCU能支持同时监控的最大变量数量
// Define the maximum number of variables the MCU can monitor simultaneously
#define ARESPLOT_MAX_VARS_TO_MONITOR                                                               \
	(10) // 可根据MCU资源调整 Can be adjusted based on MCU resources

// 定义接收和发送缓冲区的最大大小 (应大于最大可能的帧大小)
// Define max size for receive and transmit buffers (should be larger than max
// possible frame size) 最大帧 (CMD_START_MONITOR):
// 1(SOP)+1(CMD)+2(LEN)+1(NumVars)+ARESPLOT_MAX_VARS_TO_MONITOR*5(Vars)+1(CS)+1(EOP)
// = 7 + 10*5 = 57 bytes for 10 vars.
// 选择一个安全的大小, e.g., 128. This buffer is used for RX payload and TX
// frame assembly.
#define ARESPLOT_SHARED_BUFFER_SIZE (128)

// 是否启用可选的 CMD_ERROR_REPORT 功能 (1: 启用, 0: 禁用)
// Enable optional CMD_ERROR_REPORT feature (1: enable, 0: disable)
#define ARESPLOT_ENABLE_ERROR_REPORT (0)

// 默认采样周期 (毫秒) - 如果 CMD_SET_SAMPLE_RATE 未被调用
// Default sampling period (milliseconds) - if CMD_SET_SAMPLE_RATE is not called
#define ARESPLOT_DEFAULT_SAMPLE_PERIOD_MS (10) // e.g., 10ms for 100Hz

// --- 协议常量 Protocol Constants (与 aresplot.md 一致) ---
#define ARESPLOT_SOP (0xA5) // 帧起始符 Start of Packet
#define ARESPLOT_EOP (0x5A) // 帧结束符 End of Packet

// PC -> MCU 命令ID (根据最新协议文档 v5)
#define ARESPLOT_CMD_START_MONITOR   (0x01) // 请求开始/更新/停止监控变量
#define ARESPLOT_CMD_SET_VARIABLE    (0x02) // 请求设置变量值
#define ARESPLOT_CMD_SET_SAMPLE_RATE (0x03) // (可选) 设置采样率

// MCU -> PC 命令ID (根据最新协议文档 v5)
#define ARESPLOT_CMD_MONITOR_DATA (0x81) // 发送监控数据
#define ARESPLOT_CMD_ACK          (0x82) // 命令确认/应答
#if ARESPLOT_ENABLE_ERROR_REPORT
#define ARESPLOT_CMD_ERROR_REPORT (0x8F) // (可选) MCU主动错误报告
#endif

// AresOriginalType_t 枚举 (变量原始数据类型)
typedef enum {
	ARES_TYPE_INT8 = 0x00,
	ARES_TYPE_UINT8 = 0x01,
	ARES_TYPE_INT16 = 0x02,
	ARES_TYPE_UINT16 = 0x03,
	ARES_TYPE_INT32 = 0x04,
	ARES_TYPE_UINT32 = 0x05,
	ARES_TYPE_FLOAT32 = 0x06,
	ARES_TYPE_FLOAT64 = 0x07, // 注意: MCU端通常转换为float32发送
	ARES_TYPE_BOOL = 0x08
} aresplot_original_type_t;

// ACK状态码 Status codes for CMD_ACK (根据最新协议文档 v5)
typedef enum {
	ARES_STATUS_OK = 0x00,                     // 成功
	ARES_STATUS_ERROR_CHECKSUM = 0x01,         // 接收到的帧校验和错误
	ARES_STATUS_ERROR_UNKNOWN_CMD = 0x02,      // 未知命令
	ARES_STATUS_ERROR_INVALID_PAYLOAD = 0x03,  // Payload 格式或长度错误
	ARES_STATUS_ERROR_ADDR_INVALID = 0x04,     // 无效地址 (如对齐、范围)
	ARES_STATUS_ERROR_TYPE_UNSUPPORTED = 0x05, // 不支持的数据类型
	ARES_STATUS_ERROR_RATE_UNACHIEVABLE =
		0x06, // (若CMD_SET_SAMPLE_RATE被调用) 无法达到请求的采样率
	ARES_STATUS_ERROR_MCU_BUSY_OR_LIMIT =
		0x07,                         // MCU 繁忙或内部资源限制 (如变量数量超出MCU处理能力)
	ARES_STATUS_ERROR_GENERAL_FAIL = 0xFF // 通用失败
} aresplot_ack_status_t;

// 存储单个监控变量信息的结构体
typedef struct {
	void *ptr; // 指向变量内存地址的指针 Pointer to the variable's memory address
	aresplot_original_type_t type; // 变量的原始数据类型 Original data type of the variable
} aresplot_var_info_t;

// --- Aresplot 服务函数 API ---
// --- Aresplot Service Function API ---

/**
 * @brief 初始化 Aresplot 服务
 * Initializes the Aresplot service.
 * 应在系统启动时调用一次。
 * Should be called once at system startup.
 */
void aresplot_init(int sample_rate_period_ms);

/**
 * @brief 向 Aresplot 服务喂入一个从通信接口接收到的字节 (用于基于字节流的接收)
 * Feeds a byte received from the communication interface to the Aresplot
 * service (for byte-stream based reception). 应在每次接收到一个字节时调用
 * (例如，在 UART RX 中断处理程序中，如果未使用DMA/packet-based interface)。
 * Should be called every time a byte is received (e.g., in UART RX interrupt
 * handler if not using DMA/packet-based interface).
 * @param byte 接收到的字节 The received byte.
 */
void aresplot_rx_feed_byte(uint8_t byte);

/**
 * @brief 向 Aresplot 服务喂入一个从通信接口接收到的数据包 (用于基于包的接收,
 * 如DMA, USB) Feeds a data packet received from the communication interface to
 * the Aresplot service (for packet-based reception, e.g., DMA, USB).
 * @param data 指向接收到的数据包的指针 Pointer to the received data packet.
 * @param length 数据包的长度 Length of the data packet.
 * @note 此函数会迭代调用 aresplot_rx_feed_byte 来处理包内数据。
 * This function iterates through the packet and calls aresplot_rx_feed_byte for
 * each byte.
 */
void aresplot_rx_feed_packet(const uint8_t *data, uint16_t length);

/**
 * @brief Aresplot 服务的主处理函数/周期性任务
 * Main processing function / periodic task for the Aresplot service.
 * 此函数负责发送挂起的ACK帧和定期的监控数据帧。
 * This function is responsible for sending pending ACK frames and periodic
 * monitor data frames.
 * - 对于裸机系统: 应从主循环中定期调用，或者通过定时器中断标志触发调用。
 * 其调用频率应快于或等于数据发送频率。
 * - 对于RTOS系统: 可以作为一个低优先级任务的主体。
 *
 * - For bare-metal systems: Should be called periodically from the main loop,
 * or triggered by a timer interrupt flag. Its calling frequency
 * should be faster than or equal to the data sending frequency.
 * - For RTOS systems: Can be the body of a low-priority task.
 */
void aresplot_service_tick(void);

#if ARESPLOT_ENABLE_ERROR_REPORT
/**
 * @brief (可选) 发送一个错误报告给上位机
 * (Optional) Sends an error report to the host PC.
 * @param error_code 错误码 Error code.
 * @param message 可选的错误消息 (可以为 NULL) Optional error message (can be
 * NULL).
 * @param msg_len 错误消息的长度 (如果 message 为 NULL, 则为 0) Length of the
 * error message (0 if message is NULL).
 * @return 如果成功将错误报告加入发送队列则返回1, 否则返回0 (例如队列满)
 * Returns 1 if the error report was successfully queued for sending, 0
 * otherwise (e.g., queue full).
 */
int aresplot_report_error(uint8_t error_code, const char *message, uint8_t msg_len);
#endif

#endif // ARESPLOT_MCU_H
