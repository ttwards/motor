#include <stdint.h>
#include <zephyr/drivers/uart.h>

#define CH_COUNT 4
const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};

struct JFData {
    float fdata[CH_COUNT];

    struct device *uart_dev;

    unsigned char tail[4];
};

void jf_send_init(const struct device *uart_dev, struct JFData *data, int delay);