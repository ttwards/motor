#include "ares/protocol/ares_protocol.h"
#include "dual_protocol.h"
#include "zephyr/kernel.h"
#include "zephyr/net_buf.h"
#include <zephyr/logging/log.h>
#include <stdint.h>

LOG_MODULE_REGISTER(dual_protocol, LOG_LEVEL_INF);

K_HEAP_DEFINE(dual_protocol_heap, 4096);

#define HEART_BEAT_DELAY 200

struct tx_msg_bck {
	uint32_t ret;
	uint16_t id;
};

#define MAKE_REPL_FRAME(id, ret, buf)                                                              \
	({                                                                                         \
		uint8_t *repl_frame = buf;                                                         \
		GET_16BITS(repl_frame, REPL_HEAD_IDX) = REPL_FRAME_HEAD;                           \
		GET_16BITS(repl_frame, REPL_FUNC_ID_IDX) = id;                                     \
		GET_32BITS(repl_frame, REPL_RET_IDX) = (uint32_t)ret;                              \
		repl_frame;                                                                        \
	})

struct k_thread ares_usb_tid;
K_THREAD_STACK_DEFINE(ares_usb_stack, 2048);

struct net_buf *alloc_and_add_data(struct AresProtocol *protocol, uint8_t *data, size_t len)
{
	if (protocol->interface->api->alloc_buf_with_data) {
		return protocol->interface->api->alloc_buf_with_data(protocol->interface, data,
								     len);
	} else {
		struct net_buf *buf = protocol->interface->api->alloc_buf(protocol->interface);
		if (buf == NULL) {
			LOG_ERR("Failed to allocate buffer.");
			return NULL;
		}
		net_buf_add_mem(buf, data, len);
		return buf;
	}
}

static void error_handle(struct AresProtocol *protocol, uint16_t req_id, uint16_t error)
{
	struct dual_protocol_data *data = protocol->priv_data;
	if (!data->online) {
		return;
	}
	if (k_mutex_lock(&data->err_frame_mutex, K_NO_WAIT) != 0) {
		LOG_ERR("Failed to lock error frame mutex.");
		return;
	}
	GET_16BITS(data->error_frame_buf, ERROR_HEAD_IDX) = ERROR_FRAME_HEAD;
	GET_8BITS(data->error_frame_buf, ERROR_ID_IDX) = req_id;
	GET_16BITS(data->error_frame_buf, ERROR_CODE_IDX) = error;
	if (error != HEART_BEAT) {
		LOG_ERR("Error code: %x, Request ID: %x", error, req_id);
	} else {
		data->last_heart_beat = k_uptime_get_32();
	}
	struct net_buf *buf = protocol->interface->api->alloc_buf(protocol->interface);
	net_buf_add_mem(buf, data->error_frame_buf, ERROR_FRAME_LENGTH);
	if (buf == NULL) {
		LOG_ERR("Failed to allocate buffer for error frame.");
		return;
	}
	int err = protocol->interface->api->send_with_lock(protocol->interface, buf,
							   &data->err_frame_mutex);
	if (err != 0) {
		// LOG_ERR("Failed to send error frame. %d", err);
	}
}

static bool proceed_tx_bck(struct AresProtocol *protocol, uint8_t ID)
{
	struct dual_protocol_data *data = protocol->priv_data;
	// 创建一个临时队列
	struct k_msgq temp_msgq;
	uint8_t *temp_msgq_buf = k_heap_aligned_alloc(&dual_protocol_heap, 4,
						      sizeof(struct tx_msg_bck) * 10, K_NO_WAIT);
	k_msgq_init(&temp_msgq, temp_msgq_buf, sizeof(struct tx_msg_bck) * 10, 10);
	bool found = false;

	// 临时存储所有数据项
	struct tx_msg_bck *item;
	while ((k_msgq_get(&data->func_tx_bckup_msgq, &item, K_NO_WAIT)) == 0) {
		if (item->id == ID && !found) {
			struct net_buf *buf =
				protocol->interface->api->alloc_buf(protocol->interface);

			if (buf == NULL) {
				LOG_ERR("Failed to allocate buffer for FUNC frame.");
				return false;
			}

			net_buf_add(buf, REPL_FRAME_LENGTH);

			MAKE_REPL_FRAME(ID, item->ret, buf->data);
			protocol->interface->api->send(protocol->interface, buf);

			found = true;
			continue;
		} else {
			// 保存到临时队列
			k_msgq_put(&temp_msgq, item, K_NO_WAIT);
		}
	}

	// 将所有项目放回原始队列
	while ((k_msgq_get(&temp_msgq, &item, K_NO_WAIT)) == 0) {
		k_msgq_put(&data->func_tx_bckup_msgq, item, K_NO_WAIT);
	}

	return found;
}

static sync_table_t *find_pack(struct dual_protocol_data *data, uint16_t ID)
{
	for (int i = 0; i < data->sync_cnt; i++) {
		if (ID == data->sync_table[i].ID) {
			return &data->sync_table[i];
		}
	}
	return NULL;
}

static func_table_t *find_func(struct dual_protocol_data *data, uint16_t ID)
{
	for (int i = 0; i < data->func_cnt; i++) {
		if (ID == data->func_table[i].id) {
			return &data->func_table[i];
		}
	}
	return NULL;
}

static void usb_offline_clean(struct AresProtocol *protocol)
{
	struct tx_msg_bck msg;
	struct dual_protocol_data *data = protocol->priv_data;
	while (k_msgq_num_used_get(&data->func_tx_bckup_msgq) > 0) {
		k_msgq_get(&data->func_tx_bckup_msgq, &msg, K_NO_WAIT);
	}
}

static void usb_trans_heart_beat(struct k_timer *timer)
{
	struct AresProtocol *protocol = k_timer_user_data_get(timer);
	struct dual_protocol_data *data = protocol->priv_data;

	if (data->online) {
		if ((k_uptime_get_32() - data->last_heart_beat) <= HEART_BEAT_DELAY &&
		    ((int32_t)(k_uptime_get_32() - data->last_receive) >= 10 * HEART_BEAT_DELAY)) {
			LOG_ERR("Connection lost. last_heart_beat: %d, last_receive: %d, current: "
				"%d",
				data->last_heart_beat, data->last_receive, k_uptime_get_32());
			data->online = false;
			usb_offline_clean(protocol);
		}
	} else {
		if ((k_uptime_get_32() - data->last_receive) <= 10 * HEART_BEAT_DELAY) {
			LOG_INF("Connection established.");
			data->online = true;
		}
	}
	error_handle(protocol, HEARTBEAT_ID, HEART_BEAT);

	data->last_heart_beat = k_uptime_get_32();
}

static void parse_sync(struct AresProtocol *protocol, uint8_t *buf, size_t len)
{
	LOG_HEXDUMP_DBG(buf, len, "SYNC frame received");

	uint16_t ID = GET_16BITS(buf, SYNC_ID_IDX);
	uint8_t *data = buf + SYNC_DATA_IDX;
	size_t data_len = len - SYNC_FRAME_LENGTH_OFFSET;

	sync_table_t *pack = find_pack(protocol->priv_data, ID);
	if (pack == NULL) {
		LOG_ERR("Cannot find corresponding ID from sync table.");
		return;
	}

	if (pack->cb != NULL) {
		pack->cb(SYNC_PACK_STATUS_WRITE);
	}

	memcpy(pack->data, data, data_len);

	return;
}

static void parse_func(struct AresProtocol *protocol, func_table_t *map)
{
	uint32_t ret = map->cb(map->arg1, map->arg2, map->arg3);

	struct dual_protocol_data *data = protocol->priv_data;
	if (data->func_tx_bckup_cnt == MAX_FUNC_COUNT) {
		struct tx_msg_bck *rm_frame = NULL;
		k_msgq_get(&data->func_tx_bckup_msgq, &rm_frame, K_NO_WAIT);
		if (rm_frame == NULL) {
			return;
		}
		data->func_tx_bckup_cnt--;
	}

	struct tx_msg_bck msg = {
		.ret = ret,
		.id = map->id,
	};
	k_msgq_put(&data->func_tx_bckup_msgq, &msg, K_NO_WAIT);
	data->func_tx_bckup_cnt++;

	uint8_t *repl_frame = map->buf;
	GET_16BITS(repl_frame, REPL_HEAD_IDX) = REPL_FRAME_HEAD;
	GET_16BITS(repl_frame, REPL_FUNC_ID_IDX) = map->id;
	GET_32BITS(repl_frame, REPL_RET_IDX) = (uint32_t)ret;
	GET_8BITS(repl_frame, REPL_REQ_ID_IDX) = map->req_id;

	int err = 0;
	struct net_buf *buf = alloc_and_add_data(protocol, repl_frame, REPL_FRAME_LENGTH);
	err = protocol->interface->api->send_with_lock(protocol->interface, buf, &map->mutex);

	if (err != 0) {
		LOG_ERR("Failed to send FUNC frame.");
		return;
	}
	return;
}

static void parse_error(struct AresProtocol *protocol, uint8_t data[], int8_t len)
{
	uint8_t req_id = GET_8BITS(data, ERROR_ID_IDX);
	if (req_id == HEARTBEAT_ID) {
		LOG_DBG("Heartbeat frame received.");
		return;
	}
	if (!proceed_tx_bck(protocol, req_id)) {
		sync_table_t *pack = find_pack(protocol->priv_data,
					       sys_be16_to_cpu(GET_16BITS(data, ERROR_ID_IDX)));
		if (pack == NULL) {
			LOG_ERR("Cannot find corresponding ID from error "
				"frame.");
			error_handle(protocol, req_id, CLEANED_PACK);
			return;
		}
		if (pack->cb != NULL) {
			pack->cb(SYNC_PACK_STATUS_READ);
		}
		dual_sync_flush(protocol, pack);
	}
	LOG_INF("Error frame received: id =%d; code=%d", req_id, GET_16BITS(data, ERROR_CODE_IDX));
	return;
}

int dual_sync_flush(struct AresProtocol *protocol, sync_table_t *pack)
{
	struct dual_protocol_data *data = protocol->priv_data;
	// if (!data->online) {
	// 	return -EUNATCH;
	// }
	if (pack == NULL) {
		LOG_ERR("Sync pack is NULL.");
		return -EINVAL;
	}
	memcpy(pack->buf + SYNC_DATA_IDX, pack->data, pack->len);

	// LOG_HEXDUMP_DBG(pack->buf, pack->len + SYNC_FRAME_LENGTH_OFFSET, "Sync data to send:");

	struct net_buf *buf =
		alloc_and_add_data(protocol, pack->buf, pack->len + SYNC_FRAME_LENGTH_OFFSET);
	int ret = protocol->interface->api->send_with_lock(protocol->interface, buf, &pack->mutex);

	if (ret != 0) {
		// LOG_ERR("Failed to send SYNC frame. %d", ret);
		return -EBUSY;
	}
	return 0;
}

sync_table_t *dual_sync_add(struct AresProtocol *protocol, uint16_t ID, uint8_t *buf, size_t len,
			    dual_trans_cb_t cb)
{
	struct dual_protocol_data *data = protocol->priv_data;
	if (data->sync_cnt >= MAX_PACK_COUNT) {
		LOG_ERR("Sync pack count exceeds maximum.");
		return NULL;
	}
	data->sync_table[data->sync_cnt].interface = protocol->interface;
	data->sync_table[data->sync_cnt].data = buf;
	data->sync_table[data->sync_cnt].ID = sys_cpu_to_be16(ID);
	data->sync_table[data->sync_cnt].len = len;
	data->sync_table[data->sync_cnt].cb = cb;
	data->sync_table[data->sync_cnt].buf = k_heap_aligned_alloc(
		&dual_protocol_heap, 4, len + SYNC_FRAME_LENGTH_OFFSET, K_NO_WAIT);
	memset(data->sync_table[data->sync_cnt].buf, 0, len + SYNC_FRAME_LENGTH_OFFSET);
	if (data->sync_table[data->sync_cnt].buf == NULL) {
		LOG_ERR("Failed to allocate memory for SYNC frame. Count: %d", data->sync_cnt);
		return NULL;
	}
	GET_16BITS(data->sync_table[data->sync_cnt].buf, SYNC_HEAD_IDX) = SYNC_FRAME_HEAD;
	GET_16BITS(data->sync_table[data->sync_cnt].buf, SYNC_ID_IDX) = ID;
	data->sync_cnt++;
	if (cb) {
		cb(SYNC_PACK_STATUS_READ);
	}
	LOG_INF("Sync pack added: ID %x, data %p, len %d", ID, (void *)data, len);
	dual_sync_flush(protocol, &data->sync_table[data->sync_cnt - 1]);

	return &data->sync_table[data->sync_cnt - 1];
}

void dual_func_add(struct AresProtocol *protocol, uint16_t header, dual_trans_func_t cb)
{
	if (cb == NULL) {
		LOG_ERR("Callback function is NULL.");
		return;
	}
	struct dual_protocol_data *data = protocol->priv_data;
	if (data->func_cnt >= MAX_PACK_COUNT) {
		LOG_ERR("Func count exceeds maximum.");
		return;
	}
	data->func_table[data->func_cnt].id = sys_cpu_to_be16(header);
	data->func_table[data->func_cnt].cb = cb;
	data->func_cnt++;
}

void dual_func_remove(struct AresProtocol *protocol, uint16_t header)
{
	struct dual_protocol_data *data = protocol->priv_data;
	for (int i = 0; i < data->func_cnt; i++) {
		if (data->func_table[i].id == header) {
			data->func_cnt--;
			data->func_table[i] = data->func_table[data->func_cnt];
			return;
		}
	}
	LOG_ERR("Cannot find corresponding ID from func table.");
	return;
}

static void rx_frame_parser(struct AresProtocol *protocol, uint8_t *buf, uint8_t len)
{
	// LOG_INF("RX frame: %d", len);
	if (len == 0) {
		return;
	}
	struct dual_protocol_data *data = protocol->priv_data;
	data->last_receive = k_uptime_get_32();
	switch (GET_16BITS(buf, 0)) {
	case FUNC_FRAME_HEAD:
		if (len != FUNC_FRAME_LENGTH) {
			LOG_ERR("FUNC frame length mismatch: %d vs %d", FUNC_FRAME_LENGTH, len);
			error_handle(protocol, GET_16BITS(buf, FUNC_ID_IDX), UNKNOWN_TAIL);
			break;
		}
		func_table_t *map = find_func(data, GET_16BITS(buf, FUNC_ID_IDX));
		LOG_HEXDUMP_DBG(buf, len, "FUNC frame received");
		LOG_DBG("FUNC frame received: ID %x", GET_16BITS(buf, FUNC_ID_IDX));
		if (map == NULL) {
			LOG_ERR("Cannot find corresponding ID 0x%x in func mappings.",
				GET_16BITS(buf, FUNC_ID_IDX));
			LOG_HEXDUMP_DBG(buf, len, "FUNC frame");
			// error_handle(0, UNKNOWN_FUNC);
			return;
		}
		map->arg1 = GET_32BITS(buf, FUNC_ARG1_IDX);
		map->arg2 = GET_32BITS(buf, FUNC_ARG2_IDX);
		map->arg3 = GET_32BITS(buf, FUNC_ARG3_IDX);
		map->req_id = GET_32BITS(buf, FUNC_REQ_IDX);
		parse_func(protocol, map);
		break;
	case SYNC_FRAME_HEAD:
		for (int i = 0; i < data->sync_cnt; i++) {
			if (data->sync_table[i].ID == GET_16BITS(buf, SYNC_ID_IDX)) {
				if (data->sync_table[i].len + SYNC_FRAME_LENGTH_OFFSET != len) {
					LOG_ERR("SYNC frame length mismatch: %d vs %d",
						data->sync_table[i].len + SYNC_FRAME_LENGTH_OFFSET,
						len);
					error_handle(protocol, data->sync_table[i].ID,
						     UNKNOWN_TAIL);
					return;
				}
				parse_sync(protocol, buf, len);
				break;
			}
		}
		LOG_DBG("SYNC frame received: ID %x", GET_16BITS(buf, SYNC_ID_IDX));

		break;
	case ERROR_FRAME_HEAD:
		if (len != ERROR_FRAME_LENGTH) {
			LOG_ERR("ERROR frame length mismatch: %d vs %d", ERROR_FRAME_LENGTH, len);
			LOG_HEXDUMP_ERR(buf, len, "ERROR frame");
			error_handle(protocol, GET_16BITS(buf, ERROR_ID_IDX), UNKNOWN_TAIL);
			break;
		}
		// LOG_HEXDUMP_DBG(buf, len, "RX frame");
		parse_error(protocol, buf, len);
		break;
	default:
		LOG_ERR("Unknown frame received: %x", GET_16BITS(buf, 0));
		LOG_HEXDUMP_ERR(buf, len, "Unknown frame");
		break;
	}
}

int ares_dual_protocol_init(struct AresProtocol *protocol)
{
	LOG_INF("init");
	struct dual_protocol_data *data = protocol->priv_data;
	k_mutex_init(&data->err_frame_mutex);
	k_timer_init(&data->heart_beat_timer, usb_trans_heart_beat, NULL);
	k_timer_user_data_set(&data->heart_beat_timer, protocol);
	k_timer_start(&data->heart_beat_timer, K_MSEC(100), K_MSEC(HEART_BEAT_DELAY));

	uint8_t *msgq_buf = k_heap_aligned_alloc(&dual_protocol_heap, 4,
						 sizeof(struct tx_msg_bck) * 10, K_NO_WAIT);
	k_msgq_init(&data->func_tx_bckup_msgq, msgq_buf, sizeof(struct tx_msg_bck), 10);
	return 0;
}

void ares_dual_protocol_handle(struct AresProtocol *protocol, struct net_buf *buf)
{
	// LOG_INF("handle");
	// struct dual_protocol_data *data = protocol->priv_data;
	rx_frame_parser(protocol, buf->data, buf->len);
	// LOG_HEXDUMP_DBG(buf->data, buf->len, "RX frame");
}

void ares_dual_protocol_event(struct AresProtocol *protocol, enum AresProtocolEvent event)
{
	// LOG_INF("event");
	struct dual_protocol_data *data = protocol->priv_data;
	if (event == ARES_PROTOCOL_EVENT_CONNECTED) {
		LOG_INF("Connection established due to event.");
		k_msleep(600);
		data->online = true;
	} else if (event == ARES_PROTOCOL_EVENT_DISCONNECTED) {
		LOG_INF("Connection lost due to event.");
		data->online = false;
	}
}