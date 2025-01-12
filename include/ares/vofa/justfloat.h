#include <stdint.h>
#include <zephyr/drivers/uart.h>

const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};

struct JFData {
	struct device *uart_dev;

	float *fdata;

	int channel;
};

struct JFData *jf_send_init(const struct device *uart_dev, int delay, int channel);