#ifndef DART_JPEG_MCU_H
#define DART_JPEG_MCU_H
#include <stdint.h>

/* 硬件JPEG的MCU指最小编码单元，不是单片机。
 * 每个16x16单元：4块Y、1块Cb、1块Cr，各块8x8，合计384字节。 */
#define JPEG_FRAME_WIDTH 320u
#define JPEG_FRAME_HEIGHT 240u
#define JPEG_MCU_STRIP_HEIGHT 16u
#define JPEG_MCU_STRIP_COUNT (JPEG_FRAME_HEIGHT / JPEG_MCU_STRIP_HEIGHT)
#define JPEG_MCU_STRIP_BYTES (JPEG_FRAME_WIDTH / 16u * 384u)

/* 取一条16行YUYV，以32位字读取融合亮度提取/色度垂直平均。
 * 重排为硬件4:2:0块序；不改输入，不分配整帧副本，支持非对齐缓冲。 */
int jpeg_pack_mcu_strip(const uint8_t *yuyv, uint32_t strip,
                         uint8_t *out, uint32_t capacity);

typedef struct {
    uint8_t *data;
    uint32_t capacity, used, overflow;
} jpeg_output_stream;

/* 出错后继续丢弃硬件输出以排空FIFO，不越界写入用户缓冲。 */
void jpeg_stream_begin(jpeg_output_stream *stream, uint8_t *data, uint32_t capacity);
void jpeg_stream_append(jpeg_output_stream *stream, const uint8_t *data, uint32_t count);
/* 检查SOI/EOI，去掉硬件32位输出末尾至多3字节的字对齐填充。 */
int jpeg_stream_finish(jpeg_output_stream *stream, uint32_t *size);
#endif
