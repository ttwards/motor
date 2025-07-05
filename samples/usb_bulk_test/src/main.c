// main.c
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <ares/interface/usb/usb_bulk.h>
#include <ares/protocol/dual/dual_protocol.h>
#include <ares/ares_comm.h>

#include <zephyr/debug/thread_analyzer.h>

LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

DUAL_PROPOSE_PROTOCOL_DEFINE(dual_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);

int cnt = 0;
void func_cb(uint32_t param1, uint32_t param2, uint32_t param3)
{
	cnt++;
	// LOG_INF("func_cb");
	// LOG_INF("params: %2x,%2x,%2x", param1, param2, param3);
}

uint8_t test[59] = {0};
int main(void)
{
	// Initialize the USB stack and our async pipeline.
	// Pass the function that will handle the data.
	// int err = ares_usbd_init(&usb_bulk_interface);
	int err = ares_bind_interface(&usb_bulk_interface, &dual_protocol);
	dual_func_add(&dual_protocol, 0x1, func_cb);
	sync_table_t *pack = dual_sync_add(&dual_protocol, 0x1, test, sizeof(test), func_cb);
	if (err) {
		LOG_ERR("Failed to initialize ARES USB device (%d)", err);
		return 0;
	}

	LOG_INF("ARES Async USB Bulk device is ready.");

	// The main thread is now free to do other things.
	// All USB data processing happens in the background.
	uint32_t prev_cnt = 0;
	uint32_t prev_time = k_uptime_get_32();
	while (1) {
		k_sleep(K_SECONDS(1));
		// dual_sync_flush(&dual_protocol, pack);
		if (prev_time + 1000 < k_uptime_get_32()) {
			// 	// thread_analyzer_print(NULL);
			LOG_INF("cnt: %d, delta: %d", cnt, cnt - prev_cnt);
			prev_time = k_uptime_get_32();
		}
		prev_cnt = cnt;

		// You can also call ares_usbd_write() from here to send data proactively.
	}

	return 0;
}