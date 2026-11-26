#include "jpeg_mcu.h"
#include <string.h>

#ifndef DART_DET_C_UV_SWAP
#define DART_DET_C_UV_SWAP 0
#endif

int jpeg_pack_mcu_strip(const uint8_t *yuyv, uint32_t strip,
                         uint8_t *out, uint32_t capacity)
{
    if (!yuyv || !out || strip >= JPEG_MCU_STRIP_COUNT || capacity < JPEG_MCU_STRIP_BYTES)
        return -1;
    const uint32_t ybase = strip * 16u;
    for (uint32_t mx = 0; mx < JPEG_FRAME_WIDTH / 16u; ++mx) {
        uint8_t *mcu = out + mx * 384u;
        /* Y块顺序：左上、右上、左下、右下；块内逐行逐列，绝不转置。 */
        for (uint32_t block = 0; block < 4u; ++block) {
            uint32_t x0 = mx * 16u + (block & 1u) * 8u;
            uint32_t y0 = ybase + (block >> 1) * 8u;
            for (uint32_t r = 0; r < 8u; ++r) {
                const uint8_t *src = yuyv + ((y0 + r) * JPEG_FRAME_WIDTH + x0) * 2u;
                for (uint32_t c = 0; c < 8u; ++c)
                    mcu[block * 64u + r * 8u + c] = src[c * 2u];
            }
        }
        /* 相机色度已为水平1/2；再对相邻两行取平均，与原软件版4:2:0一致。
         * 硬件接收0..255的无符号像素；减128由外设内部完成。 */
        for (uint32_t r = 0; r < 8u; ++r) {
            const uint8_t *row0 = yuyv + ((ybase + r * 2u) * JPEG_FRAME_WIDTH + mx * 16u) * 2u;
            const uint8_t *row1 = row0 + JPEG_FRAME_WIDTH * 2u;
            for (uint32_t c = 0; c < 8u; ++c) {
                uint32_t offset = c * 4u;
                uint32_t cb = DART_DET_C_UV_SWAP ? 3u : 1u;
                uint32_t cr = DART_DET_C_UV_SWAP ? 1u : 3u;
                mcu[256u + r * 8u + c] = (uint8_t)(((unsigned)row0[offset + cb] + row1[offset + cb]) / 2u);
                mcu[320u + r * 8u + c] = (uint8_t)(((unsigned)row0[offset + cr] + row1[offset + cr]) / 2u);
            }
        }
    }
    return 0;
}

void jpeg_stream_begin(jpeg_output_stream *stream, uint8_t *data, uint32_t capacity)
{
    stream->data = data; stream->capacity = capacity; stream->used = 0;
    stream->overflow = data ? 0u : 1u;
}

void jpeg_stream_append(jpeg_output_stream *stream, const uint8_t *data, uint32_t count)
{
    if (stream->overflow) return;
    if (!data || count > stream->capacity - stream->used) { stream->overflow = 1; return; }
    memcpy(stream->data + stream->used, data, count); stream->used += count;
}

int jpeg_stream_finish(jpeg_output_stream *stream, uint32_t *size)
{
    if (!size) return -1;
    *size = 0;
    if (stream->overflow) return -2;
    if (stream->used < 4u || stream->data[0] != 0xff || stream->data[1] != 0xd8) return -3;
    for (uint32_t padding = 0; padding < 4u && stream->used >= padding + 4u; ++padding) {
        uint32_t end = stream->used - padding;
        if (stream->data[end - 2u] == 0xff && stream->data[end - 1u] == 0xd9) {
            *size = end; return 0;
        }
    }
    return -3;
}
