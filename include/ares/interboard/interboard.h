#ifndef INTERBOARD_H_
#define INTERBOARD_H_

#include <zephyr/types.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif
#define datalen 16
typedef struct {
  uint8_t sync;     // 同步头0x5A（固定值）
  uint8_t ctrl;     // [7]主1从0 [6]是否为重发
  uint8_t msg_type; // 消息类型
  uint8_t ack;      // 应答
} Frame_Header;
// Frame structure
typedef struct {
    Frame_Header frame_header; // Start byte to indicate the beginning of a frame
    uint8_t datas[datalen];  // 数据区
    
    uint16_t crc16;      // CRC16校验（覆盖帧头到datas）
    uint16_t end_mark;   // 帧尾标识0x0D0A（固定值
} Interboard_Frame;

typedef enum{
  INTERBOARD_MSG_CAN,
  INTERBOARD_MSG_IMU,

} InterboardMsgType;
//为消息队列设计的数据结构
typedef struct {
  uint8_t msg_type;
  uint8_t data[16];
} Frame_Data;
//为信号量和回调函数和线程传递数据的数据结构
typedef struct{
  struct k_sem data_ready;
  uint8_t *rxdata;
} Interboard_Context ;


void tx_thread_func(void *p1, void *p2, void *p3);

void interboardrx_callback(const struct device *spi_dev, int error_code, void *data);

void spi_init(void);
int interboard_transceive(Frame_Data *tx_data, Frame_Data *rx_data);

#ifdef __cplusplus
}
#endif

#endif // INTERBOARD_H_