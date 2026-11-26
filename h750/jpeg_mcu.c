#include "jpeg_mcu.h"
#include <string.h>

#ifndef DART_DET_C_UV_SWAP
#define DART_DET_C_UV_SWAP 0
#endif

/* 用memcpy避免uint8_t缓冲强转uint32_t的别名/对齐未定义行为。
 * ARM小端-O2会内联为字读写；大端主机仍保持YUYV字节序。 */
static inline uint32_t load_le32(const uint8_t *p)
{
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    uint32_t value; memcpy(&value, p, sizeof(value)); return value;
#else
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
#endif
}
static inline void store_le32(uint8_t *p, uint32_t value)
{
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    memcpy(p, &value, sizeof(value));
#else
    p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16); p[3] = (uint8_t)(value >> 24);
#endif
}
/* 两个YUYV字 -> 连续4个Y，色度字节在另一条路径复用，无需重读原图。 */
static inline uint32_t four_y(uint32_t a, uint32_t b)
{
    return (a & 0xffu) | ((a >> 8) & 0xff00u) |
        ((b << 16) & 0xff0000u) | ((b << 8) & 0xff000000u);
}
/* 16位通道隔离，最大和510不会串扰；除2向下取整与原实现一致。 */
static inline uint32_t mean_uv(uint32_t row0, uint32_t row1)
{
    return ((((row0 >> 8) & 0x00ff00ffu) + ((row1 >> 8) & 0x00ff00ffu)) >> 1) & 0x00ff00ffu;
}

int jpeg_pack_mcu_strip(const uint8_t *yuyv, uint32_t strip,
                         uint8_t *out, uint32_t capacity)
{
    if (!yuyv || !out || strip >= JPEG_MCU_STRIP_COUNT || capacity < JPEG_MCU_STRIP_BYTES)
        return -1;
    const uint32_t ybase = strip * 16u;
    for (uint32_t mx = 0; mx < JPEG_FRAME_WIDTH / 16u; ++mx) {
        uint8_t *mcu = out + mx * 384u;
        /* 每次读取两行各8像素，同时写Y块和垂直平均色度。
         * 原实现逐字节提取Y再重新读色度；这里每4字节仅一次32位读取。
         * Y块顺序仍为左上、右上、左下、右下，Cb/Cr为8x8各64字节。 */
        for (uint32_t r = 0; r < 8u; ++r) {
            const uint8_t *row0 = yuyv + ((ybase + r * 2u) * JPEG_FRAME_WIDTH + mx * 16u) * 2u;
            const uint8_t *row1 = row0 + JPEG_FRAME_WIDTH * 2u;
            for (uint32_t half = 0; half < 2u; ++half) {
                const uint8_t *p = row0 + half * 16u, *q = row1 + half * 16u;
                uint32_t a0 = load_le32(p), a1 = load_le32(p + 4);
                uint32_t a2 = load_le32(p + 8), a3 = load_le32(p + 12);
                uint32_t b0 = load_le32(q), b1 = load_le32(q + 4);
                uint32_t b2 = load_le32(q + 8), b3 = load_le32(q + 12);
                uint8_t *yp = mcu + (r >= 4u ? 128u : 0u) + half * 64u + (r & 3u) * 16u;
                store_le32(yp, four_y(a0, a1)); store_le32(yp + 4, four_y(a2, a3));
                store_le32(yp + 8, four_y(b0, b1)); store_le32(yp + 12, four_y(b2, b3));
                uint32_t uv0 = mean_uv(a0, b0), uv1 = mean_uv(a1, b1);
                uint32_t uv2 = mean_uv(a2, b2), uv3 = mean_uv(a3, b3);
                uint32_t u = (uv0 & 0xffu) | ((uv1 & 0xffu) << 8) |
                    ((uv2 & 0xffu) << 16) | (uv3 << 24);
                uint32_t v = (uv0 >> 16) | ((uv1 >> 16) << 8) |
                    (uv2 & 0xff0000u) | ((uv3 >> 16) << 24);
                uint32_t offset = r * 8u + half * 4u;
                store_le32(mcu + 256u + offset, DART_DET_C_UV_SWAP ? v : u);
                store_le32(mcu + 320u + offset, DART_DET_C_UV_SWAP ? u : v);
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
