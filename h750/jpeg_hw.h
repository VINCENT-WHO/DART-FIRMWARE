#ifndef DART_JPEG_HW_H
#define DART_JPEG_HW_H
#include <stdint.h>

#define JPEG_ENCODER_BACKEND "STM32H750_HW_POLL_420"
/* 在相机会话准备阶段初始化外设并重置统计，不计入首帧压缩时间。 */
int jpeg_hw_prepare(int quality);

/* 墙钟累计，单位微秒；poll不含回调中重排时间，但含FIFO搬运/中断抢占。 */
extern uint32_t jpeg_hw_pack_us, jpeg_hw_poll_us;
extern uint32_t jpeg_hw_error, jpeg_hw_hal_error;
#endif
