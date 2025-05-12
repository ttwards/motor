// aresplot_mcu.c

#include "aresplot_protocol.h"
#include "aresplot_uart.h"
#include "zephyr/logging/log.h"
#include <string.h>

LOG_MODULE_REGISTER(aresplot, LOG_LEVEL_INF);

// --- 内部状态变量 Internal State Variables ---

// 接收状态机
typedef enum {
	ARES_RX_STATE_WAIT_SOP,
	ARES_RX_STATE_WAIT_CMD,
	ARES_RX_STATE_WAIT_LEN1,
	ARES_RX_STATE_WAIT_LEN2,
	ARES_RX_STATE_WAIT_PAYLOAD,
	ARES_RX_STATE_WAIT_CHECKSUM,
	ARES_RX_STATE_WAIT_EOP
} aresplot_rx_state_t;

static volatile aresplot_rx_state_t g_rx_state; // 当前接收状态 Current receive state
static uint8_t g_rx_payload_buffer[ARESPLOT_SHARED_BUFFER_SIZE]; // 接收缓冲区仅用于Payload
								 // Receive buffer for
								 // payload only
static uint16_t g_rx_payload_len;        // 当前帧的Payload长度 Payload length of the current frame
static uint16_t g_rx_payload_idx;        // 当前接收的Payload字节计数 Payload byte counter
static uint8_t g_rx_cmd;                 // 当前帧的命令ID Command ID of the current frame
static uint8_t g_rx_checksum_calculated; // 计算出的校验和 Calculated checksum

// 监控变量列表
static aresplot_var_info_t g_monitor_vars[ARESPLOT_MAX_VARS_TO_MONITOR]; // 存储被监控变量信息的数组
									 // Array to store monitored
									 // variable info
static uint8_t g_num_monitor_vars;           // 当前正在监控的变量数量 Number of currently
					     // monitored variables
static volatile uint8_t g_monitoring_active; // 监控是否激活标志 Flag indicating
					     // if monitoring is active

// 数据发送定时
static uint32_t g_sample_period_ms;    // 采样周期 (毫秒) Sampling period (ms)
static uint32_t g_last_sample_time_ms; // 上次采样时间戳 Last sample timestamp

// 发送组装缓冲区 (用于 ACK, Monitor Data, Error Report)
// Transmit assembly buffer (for ACK, Monitor Data, Error Report)
static uint8_t g_tx_assembly_buffer[ARESPLOT_SHARED_BUFFER_SIZE];

// ACK 发送相关
// ACK transmit related
static volatile uint8_t g_ack_pending; // 是否有ACK等待发送 Flag indicating if an ACK is pending
static uint8_t g_ack_cmd_to_ack;       // 要ACK的命令ID
static aresplot_ack_status_t g_ack_status_to_send; // 要发送的ACK状态

#if ARESPLOT_ENABLE_ERROR_REPORT
// 错误报告发送相关
// Error report transmit related
static volatile uint8_t g_error_report_pending; // 是否有错误报告等待发送
static uint8_t g_error_report_code_to_send;     // 要发送的错误码
static char g_error_report_msg_to_send[ARESPLOT_SHARED_BUFFER_SIZE -
				       7];     // 错误消息缓冲区 (最大可能长度)
static uint8_t g_error_report_msg_len_to_send; // 错误消息长度
#endif

// --- 内部辅助函数 Internal Helper Functions ---

/**
 * @brief 计算帧校验和 (从CMD到PAYLOAD尾)
 * Calculates the frame checksum (from CMD to end of PAYLOAD).
 * @param cmd 命令ID Command ID.
 * @param len Payload长度 Payload length.
 * @param payload 指向Payload数据的指针 Pointer to payload data.
 * @return 计算得到的校验和 Calculated checksum.
 */
static uint8_t calculate_checksum(uint8_t cmd, uint16_t len, const uint8_t *payload)
{
	uint8_t checksum = 0;
	checksum ^= cmd;
	checksum ^= (uint8_t)(len & 0xFF);
	checksum ^= (uint8_t)((len >> 8) & 0xFF);
	for (uint16_t i = 0; i < len; ++i) {
		checksum ^= payload[i];
	}
	return checksum;
}

/**
 * @brief 组装并发送一个完整的帧 (通过 aresplot_user_send_packet)
 * Assembles and sends a complete frame (via aresplot_user_send_packet).
 * @param cmd 命令ID Command ID.
 * @param payload 指向Payload数据的指针 (可以为NULL如果len为0) Pointer to
 * payload data (can be NULL if len is 0).
 * @param len Payload长度 Payload length.
 * @note 此函数内部使用全局的 g_tx_assembly_buffer 进行帧组装。
 * This function uses the global g_tx_assembly_buffer for frame assembly
 * internally. 调用此函数前应确保 g_tx_assembly_buffer 未被其他发送操作占用
 * (通过 pending 标志等机制)。 Ensure g_tx_assembly_buffer is not in use by
 * other send operations before calling (e.g. via pending flags).
 */
static void assemble_and_send_frame_internal(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
	uint16_t frame_idx = 0;
	uint16_t total_frame_len = 1 + 1 + 2 + len + 1 + 1; // SOP+CMD+LEN+Payload+CS+EOP

	if (total_frame_len > ARESPLOT_SHARED_BUFFER_SIZE) {
		LOG_ERR("Frame too large to assemble: %d > %d", total_frame_len,
			ARESPLOT_SHARED_BUFFER_SIZE);
		return;
	}

	// 1. SOP
	g_tx_assembly_buffer[frame_idx++] = ARESPLOT_SOP;
	// 2. CMD
	g_tx_assembly_buffer[frame_idx++] = cmd;
	// 3. LEN (Little Endian)
	g_tx_assembly_buffer[frame_idx++] = (uint8_t)(len & 0xFF);
	g_tx_assembly_buffer[frame_idx++] = (uint8_t)((len >> 8) & 0xFF);
	// 4. PAYLOAD
	if (payload && len > 0) {
		memcpy(&g_tx_assembly_buffer[frame_idx], payload, len);
		frame_idx += len;
	}
	// 5. CHECKSUM
	g_tx_assembly_buffer[frame_idx++] = calculate_checksum(cmd, len, payload);
	// 6. EOP
	g_tx_assembly_buffer[frame_idx++] = ARESPLOT_EOP;

	LOG_DBG("Sending frame: cmd=0x%02x, len=%d", cmd, frame_idx);
	aresplot_user_send_packet(g_tx_assembly_buffer, frame_idx);
}

/**
 * @brief 标记一个ACK响应等待发送 (实际发送在 aresplot_service_tick 中完成)
 * Flags an ACK response to be sent (actual sending happens in
 * aresplot_service_tick).
 * @param ack_cmd_id 被ACK的命令ID The command ID being ACKed.
 * @param status ACK状态码 ACK status code.
 */
static void queue_ack_response(uint8_t ack_cmd_id, aresplot_ack_status_t status)
{
	aresplot_user_critical_enter();
	// 如果有其他发送操作正在等待 (例如错误报告)，ACK可以优先，或者简单覆盖/排队
	// For simplicity, if another ACK is pending, this new one overwrites it.
	g_ack_cmd_to_ack = ack_cmd_id;
	g_ack_status_to_send = status;
	g_ack_pending = 1;
	aresplot_user_critical_exit();
}

/**
 * @brief 处理接收到的 CMD_START_MONITOR 命令
 * Processes a received CMD_START_MONITOR command.
 */
static void handle_cmd_start_monitor(void)
{
	uint8_t num_vars_requested = g_rx_payload_buffer[0];
	aresplot_ack_status_t status = ARES_STATUS_OK;

	LOG_DBG("Start monitor request: %d variables", num_vars_requested);

	aresplot_user_critical_enter();
	g_monitoring_active = 0;
	g_num_monitor_vars = 0;

	if (num_vars_requested == 0) {
		status = ARES_STATUS_OK;
		LOG_INF("Monitor stopped");
	} else if (num_vars_requested > ARESPLOT_MAX_VARS_TO_MONITOR) {
		status = ARES_STATUS_ERROR_MCU_BUSY_OR_LIMIT;
		LOG_ERR("Too many variables requested: %d > %d", num_vars_requested,
			ARESPLOT_MAX_VARS_TO_MONITOR);
	} else {
		if (g_rx_payload_len != (1 + num_vars_requested * 5)) {
			status = ARES_STATUS_ERROR_INVALID_PAYLOAD;
			LOG_ERR("Invalid payload length: %d != %d", g_rx_payload_len,
				(1 + num_vars_requested * 5));
		} else {
			g_num_monitor_vars = num_vars_requested;
			for (uint8_t i = 0; i < g_num_monitor_vars; ++i) {
				uint8_t *p_var_info_payload = &g_rx_payload_buffer[1 + i * 5];

				uint32_t temp_addr = (uint32_t)p_var_info_payload[0] |
						     ((uint32_t)p_var_info_payload[1] << 8) |
						     ((uint32_t)p_var_info_payload[2] << 16) |
						     ((uint32_t)p_var_info_payload[3] << 24);
				g_monitor_vars[i].ptr = (void *)temp_addr;
				g_monitor_vars[i].type =
					(aresplot_original_type_t)p_var_info_payload[4];
				LOG_DBG("Added variable %d: addr=0x%08x, type=%d", i, temp_addr,
					p_var_info_payload[4]);
			}
			g_monitoring_active = 1;
			g_last_sample_time_ms = aresplot_user_get_tick_ms();
			status = ARES_STATUS_OK;
			LOG_INF("Monitor started with %d variables", num_vars_requested);
		}
	}
	aresplot_user_critical_exit();
	queue_ack_response(ARESPLOT_CMD_START_MONITOR, status);
}

/**
 * @brief 处理接收到的 CMD_SET_VARIABLE 命令
 * Processes a received CMD_SET_VARIABLE command.
 */
static void handle_cmd_set_variable(void)
{
	if (g_rx_payload_len != 9) {
		LOG_ERR("Invalid SET_VARIABLE payload length: %d != 9", g_rx_payload_len);
		queue_ack_response(ARESPLOT_CMD_SET_VARIABLE, ARES_STATUS_ERROR_INVALID_PAYLOAD);
		return;
	}

	void *addr;
	aresplot_original_type_t original_type;
	float float_val;
	uint8_t *p_payload = g_rx_payload_buffer;

	uint32_t temp_addr = (uint32_t)p_payload[0] | ((uint32_t)p_payload[1] << 8) |
			     ((uint32_t)p_payload[2] << 16) | ((uint32_t)p_payload[3] << 24);
	addr = (void *)temp_addr;

	original_type = (aresplot_original_type_t)p_payload[4];

	uint8_t temp_float_bytes[4];
	temp_float_bytes[0] = p_payload[5];
	temp_float_bytes[1] = p_payload[6];
	temp_float_bytes[2] = p_payload[7];
	temp_float_bytes[3] = p_payload[8];
	memcpy(&float_val, temp_float_bytes, sizeof(float));

	LOG_DBG("Setting variable: addr=0x%08x, type=%d, value=%f", temp_addr, original_type,
		float_val);

	aresplot_user_critical_enter();
	switch (original_type) {
	case ARES_TYPE_INT8:
		*(volatile int8_t *)addr = (int8_t)float_val;
		break;
	case ARES_TYPE_UINT8:
		*(volatile uint8_t *)addr = (uint8_t)float_val;
		break;
	case ARES_TYPE_INT16:
		*(volatile int16_t *)addr = (int16_t)float_val;
		break;
	case ARES_TYPE_UINT16:
		*(volatile uint16_t *)addr = (uint16_t)float_val;
		break;
	case ARES_TYPE_INT32:
		*(volatile int32_t *)addr = (int32_t)float_val;
		break;
	case ARES_TYPE_UINT32:
		*(volatile uint32_t *)addr = (uint32_t)float_val;
		break;
	case ARES_TYPE_FLOAT32:
		*(volatile float *)addr = float_val;
		break;
	case ARES_TYPE_BOOL:
		*(volatile uint8_t *)addr = (float_val != 0.0f) ? 1 : 0;
		break;
	default:
		LOG_ERR("Unsupported variable type: %d", original_type);
		aresplot_user_critical_exit();
		queue_ack_response(ARESPLOT_CMD_SET_VARIABLE, ARES_STATUS_ERROR_TYPE_UNSUPPORTED);
		return;
	}
	aresplot_user_critical_exit();
	queue_ack_response(ARESPLOT_CMD_SET_VARIABLE, ARES_STATUS_OK);
}

/**
 * @brief 处理接收到的 CMD_SET_SAMPLE_RATE 命令
 * Processes a received CMD_SET_SAMPLE_RATE command.
 */
static void handle_cmd_set_sample_rate(void)
{
	if (g_rx_payload_len != 4) {
		queue_ack_response(ARESPLOT_CMD_SET_SAMPLE_RATE, ARES_STATUS_ERROR_INVALID_PAYLOAD);
		return;
	}

	uint32_t rate_hz;
	uint8_t *p_payload = g_rx_payload_buffer;

	rate_hz = (uint32_t)p_payload[0] | ((uint32_t)p_payload[1] << 8) |
		  ((uint32_t)p_payload[2] << 16) | ((uint32_t)p_payload[3] << 24);

	aresplot_user_critical_enter();
	if (rate_hz == 0) {
		g_sample_period_ms = ARESPLOT_DEFAULT_SAMPLE_PERIOD_MS;
	} else {
		uint32_t period_ms = 1000 / rate_hz;
		if (period_ms == 0 && rate_hz != 0) {
			period_ms = 1;
		}
		g_sample_period_ms = period_ms;
	}
	g_last_sample_time_ms = aresplot_user_get_tick_ms();
	aresplot_user_critical_exit();

	queue_ack_response(ARESPLOT_CMD_SET_SAMPLE_RATE, ARES_STATUS_OK);
}

/**
 * @brief 处理一个完整的、校验通过的帧
 * Processes a complete, checksum-verified frame.
 */
static void process_received_frame(void)
{
	switch (g_rx_cmd) {
	case ARESPLOT_CMD_START_MONITOR:
		handle_cmd_start_monitor();
		break;
	case ARESPLOT_CMD_SET_VARIABLE:
		handle_cmd_set_variable();
		break;
	case ARESPLOT_CMD_SET_SAMPLE_RATE:
		handle_cmd_set_sample_rate();
		break;
	default:
		queue_ack_response(g_rx_cmd, ARES_STATUS_ERROR_UNKNOWN_CMD);
		break;
	}
}

// --- 公共 API 函数实现 Public API Function Implementations ---

void aresplot_init(int sample_rate_period_ms)
{
	LOG_DBG("aresplot_init");
	g_rx_state = ARES_RX_STATE_WAIT_SOP;
	g_num_monitor_vars = 0;
	g_monitoring_active = 0;
	g_sample_period_ms = sample_rate_period_ms;
	g_last_sample_time_ms = 0;
	g_ack_pending = 0;
#if ARESPLOT_ENABLE_ERROR_REPORT
	g_error_report_pending = 0;
#endif
}

void aresplot_rx_feed_byte(uint8_t byte)
{
	switch (g_rx_state) {
	case ARES_RX_STATE_WAIT_SOP:
		if (byte == ARESPLOT_SOP) {
			g_rx_state = ARES_RX_STATE_WAIT_CMD;
			g_rx_checksum_calculated = 0;
			LOG_DBG("Received SOP");
		}
		break;

	case ARES_RX_STATE_WAIT_CMD:
		g_rx_cmd = byte;
		g_rx_checksum_calculated ^= byte;
		g_rx_state = ARES_RX_STATE_WAIT_LEN1;
		LOG_DBG("Received CMD: 0x%02x", byte);
		break;

	case ARES_RX_STATE_WAIT_LEN1:
		g_rx_payload_len = byte; // LSB
		g_rx_checksum_calculated ^= byte;
		g_rx_state = ARES_RX_STATE_WAIT_LEN2;
		break;

	case ARES_RX_STATE_WAIT_LEN2:
		g_rx_payload_len |= ((uint16_t)byte << 8); // MSB
		g_rx_checksum_calculated ^= byte;
		if (g_rx_payload_len > sizeof(g_rx_payload_buffer)) {
			LOG_ERR("Payload too large: %d > %d", g_rx_payload_len,
				sizeof(g_rx_payload_buffer));
			g_rx_state = ARES_RX_STATE_WAIT_SOP;
		} else if (g_rx_payload_len == 0) {
			g_rx_state = ARES_RX_STATE_WAIT_CHECKSUM;
		} else {
			g_rx_payload_idx = 0;
			g_rx_state = ARES_RX_STATE_WAIT_PAYLOAD;
		}
		LOG_DBG("Received LEN: %d", g_rx_payload_len);
		break;

	case ARES_RX_STATE_WAIT_PAYLOAD:
		g_rx_payload_buffer[g_rx_payload_idx++] = byte;
		g_rx_checksum_calculated ^= byte;
		if (g_rx_payload_idx >= g_rx_payload_len) {
			g_rx_state = ARES_RX_STATE_WAIT_CHECKSUM;
		}
		break;

	case ARES_RX_STATE_WAIT_CHECKSUM:
		if (byte == g_rx_checksum_calculated) {
			g_rx_state = ARES_RX_STATE_WAIT_EOP;
			LOG_DBG("Checksum OK");
		} else {
			LOG_ERR("Checksum error: received=0x%02x, calculated=0x%02x", byte,
				g_rx_checksum_calculated);
			queue_ack_response(g_rx_cmd, ARES_STATUS_ERROR_CHECKSUM);
			g_rx_state = ARES_RX_STATE_WAIT_SOP;
		}
		break;

	case ARES_RX_STATE_WAIT_EOP:
		if (byte == ARESPLOT_EOP) {
			LOG_DBG("Received complete frame");
			process_received_frame();
		} else {
			LOG_ERR("Invalid EOP: 0x%02x", byte);
		}
		g_rx_state = ARES_RX_STATE_WAIT_SOP;
		break;

	default:
		LOG_ERR("Invalid RX state: %d", g_rx_state);
		g_rx_state = ARES_RX_STATE_WAIT_SOP;
		break;
	}
}

void aresplot_rx_feed_packet(const uint8_t *data, uint16_t length)
{
	for (uint16_t i = 0; i < length; ++i) {
		aresplot_rx_feed_byte(data[i]);
	}
}

static void send_monitor_data_payload_assembly(uint8_t *out_payload_buffer,
					       uint16_t *out_payload_len)
{
	uint32_t timestamp;
	uint16_t payload_idx = 0;
	float temp_float_val;
	uint8_t current_num_vars;
	aresplot_var_info_t local_monitor_vars[ARESPLOT_MAX_VARS_TO_MONITOR];

	aresplot_user_critical_enter();
	if (!g_monitoring_active || g_num_monitor_vars == 0) {
		aresplot_user_critical_exit();
		*out_payload_len = 0;
		return;
	}
	current_num_vars = g_num_monitor_vars;
	for (uint8_t i = 0; i < current_num_vars; ++i) {
		local_monitor_vars[i] = g_monitor_vars[i];
	}
	aresplot_user_critical_exit();

	timestamp = aresplot_user_get_tick_ms();

	out_payload_buffer[payload_idx++] = (uint8_t)(timestamp & 0xFF);
	out_payload_buffer[payload_idx++] = (uint8_t)((timestamp >> 8) & 0xFF);
	out_payload_buffer[payload_idx++] = (uint8_t)((timestamp >> 16) & 0xFF);
	out_payload_buffer[payload_idx++] = (uint8_t)((timestamp >> 24) & 0xFF);

	for (uint8_t i = 0; i < current_num_vars; ++i) {
		if (local_monitor_vars[i].ptr == NULL) {
			temp_float_val = 0.0f;
		} else {
			switch (local_monitor_vars[i].type) {
			case ARES_TYPE_INT8:
				temp_float_val =
					(float)(*(volatile int8_t *)local_monitor_vars[i].ptr);
				break;
			case ARES_TYPE_UINT8:
				temp_float_val =
					(float)(*(volatile uint8_t *)local_monitor_vars[i].ptr);
				break;
			case ARES_TYPE_INT16:
				temp_float_val =
					(float)(*(volatile int16_t *)local_monitor_vars[i].ptr);
				break;
			case ARES_TYPE_UINT16:
				temp_float_val =
					(float)(*(volatile uint16_t *)local_monitor_vars[i].ptr);
				break;
			case ARES_TYPE_INT32:
				temp_float_val =
					(float)(*(volatile int32_t *)local_monitor_vars[i].ptr);
				break;
			case ARES_TYPE_UINT32:
				temp_float_val =
					(float)(*(volatile uint32_t *)local_monitor_vars[i].ptr);
				break;
			case ARES_TYPE_FLOAT32:
				temp_float_val = (*(volatile float *)local_monitor_vars[i].ptr);
				break;
			case ARES_TYPE_BOOL:
				temp_float_val = (*(volatile uint8_t *)local_monitor_vars[i].ptr)
							 ? 1.0f
							 : 0.0f;
				break;
			default:
				temp_float_val = 0.0f;
				break;
			}
		}

		uint8_t temp_float_bytes[4]; // Temporary buffer for float bytes
		memcpy(temp_float_bytes, &temp_float_val, sizeof(float));
		out_payload_buffer[payload_idx++] = temp_float_bytes[0];
		out_payload_buffer[payload_idx++] = temp_float_bytes[1];
		out_payload_buffer[payload_idx++] = temp_float_bytes[2];
		out_payload_buffer[payload_idx++] = temp_float_bytes[3];
	}
	*out_payload_len = 4 + current_num_vars * 4;
}

void aresplot_service_tick(void)
{
	uint32_t current_time_ms;
	uint8_t ack_cmd_to_process = 0;
	aresplot_ack_status_t status_to_process = ARES_STATUS_OK;
	uint8_t process_ack = 0;

#if ARESPLOT_ENABLE_ERROR_REPORT
	uint8_t error_code_to_process = 0;
	char error_msg_to_process[ARESPLOT_SHARED_BUFFER_SIZE - 7];
	uint8_t error_msg_len_to_process = 0;
	uint8_t process_error_report = 0;
#endif

	// 1. 检查是否有挂起的ACK，并准备发送 (在临界区外组装，减少临界区时间)
	// Check for pending ACK and prepare for sending (assemble outside critical
	// section to reduce time in cs)
	aresplot_user_critical_enter();
	if (g_ack_pending) {
		ack_cmd_to_process = g_ack_cmd_to_ack;
		status_to_process = g_ack_status_to_send;
		process_ack = 1;
		g_ack_pending = 0; // 清除挂起标志 Clear pending flag
	}
	aresplot_user_critical_exit();

	if (process_ack) {
		uint8_t ack_payload[2];
		ack_payload[0] = ack_cmd_to_process;
		ack_payload[1] = (uint8_t)status_to_process;
		assemble_and_send_frame_internal(ARESPLOT_CMD_ACK, ack_payload, 2);
	}

#if ARESPLOT_ENABLE_ERROR_REPORT
	// 2. 检查是否有挂起的错误报告 (如果启用)
	// Check for pending error report (if enabled)
	aresplot_user_critical_enter();
	if (g_error_report_pending) {
		error_code_to_process = g_error_report_code_to_send;
		memcpy(error_msg_to_process, g_error_report_msg_to_send,
		       g_error_report_msg_len_to_send);
		error_msg_len_to_process = g_error_report_msg_len_to_send;
		process_error_report = 1;
		g_error_report_pending = 0; // 清除挂起标志
	}
	aresplot_user_critical_exit();

	if (process_error_report) {
		uint8_t error_payload[1 + ARESPLOT_SHARED_BUFFER_SIZE - 7]; // Max possible payload
		uint16_t error_payload_len = 1 + error_msg_len_to_process;
		error_payload[0] = error_code_to_process;
		if (error_msg_len_to_process > 0) {
			memcpy(&error_payload[1], error_msg_to_process, error_msg_len_to_process);
		}
		assemble_and_send_frame_internal(ARESPLOT_CMD_ERROR_REPORT, error_payload,
						 error_payload_len);
	}
#endif

	// 3. 检查是否需要发送监控数据
	if (g_monitoring_active && g_num_monitor_vars > 0) { // 再次检查，因为状态可能已改变
		current_time_ms = aresplot_user_get_tick_ms();
		uint32_t time_diff;

		aresplot_user_critical_enter();
		uint32_t last_sample = g_last_sample_time_ms;
		uint32_t sample_period = g_sample_period_ms;
		aresplot_user_critical_exit();

		if (current_time_ms >= last_sample) {
			time_diff = current_time_ms - last_sample;
		} else {
			time_diff = (0xFFFFFFFFU - last_sample) + current_time_ms + 1;
		}

		if (time_diff >= sample_period) {
			uint8_t monitor_data_payload[4 + ARESPLOT_MAX_VARS_TO_MONITOR * 4];
			uint16_t monitor_data_payload_len = 0;

			send_monitor_data_payload_assembly(monitor_data_payload,
							   &monitor_data_payload_len);

			if (monitor_data_payload_len > 0) {
				assemble_and_send_frame_internal(ARESPLOT_CMD_MONITOR_DATA,
								 monitor_data_payload,
								 monitor_data_payload_len);
			}

			aresplot_user_critical_enter();
			g_last_sample_time_ms = current_time_ms;
			aresplot_user_critical_exit();
		}
	}
}

#if ARESPLOT_ENABLE_ERROR_REPORT
int aresplot_report_error(uint8_t error_code, const char *message, uint8_t msg_len)
{
	aresplot_user_critical_enter();
	if (g_error_report_pending) {
		aresplot_user_critical_exit();
		return 0;
	}

	// 确保消息不会导致payload溢出 g_error_report_msg_to_send 缓冲区
	// Ensure message doesn't overflow g_error_report_msg_to_send buffer
	if (msg_len >= sizeof(g_error_report_msg_to_send)) {
		msg_len = sizeof(g_error_report_msg_to_send) -
			  1; // Leave space for null terminator if it were a C string
	}

	g_error_report_code_to_send = error_code;
	if (message && msg_len > 0) {
		memcpy(g_error_report_msg_to_send, message, msg_len);
		g_error_report_msg_len_to_send = msg_len;
	} else {
		g_error_report_msg_len_to_send = 0;
	}

	g_error_report_pending = 1;
	aresplot_user_critical_exit();
	return 1;
}
#endif
