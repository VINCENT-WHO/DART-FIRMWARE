/* 机载基线 JPEG 编码器（纯 C）。
 * 流程：YUV422 -> 4:2:0 取样 -> 8x8 FDCT（Q14 余弦基表，行列分离）->
 * 量化（IJG 质量缩放）-> 标准 Huffman -> JFIF 头尾。
 * 输出可被所有 MJPEG 播放器与 OpenCV 解码。 */
#include "jpeg_enc.h"
#include "green_detector.h"   /* DART_DET_WIDTH/HEIGHT 与 YUYV 取样 */
#include <string.h>

#define W DART_DET_WIDTH
#define H DART_DET_HEIGHT

volatile uint32_t enc_frames, enc_bytes_total, enc_max_us, enc_max_frame_bytes;

/* ---------------- 输出位流 ---------------- */
static uint8_t *out_buf;
static uint32_t out_len, out_cap;

static int put_byte(uint8_t b)
{
    if (out_len >= out_cap) return -1;
    out_buf[out_len++] = b;
    if (b == 0xFF) {                        /* 字节填充：0xFF 后必须跟 0x00 */
        if (out_len >= out_cap) return -1;
        out_buf[out_len++] = 0x00;
    }
    return 0;
}

static uint32_t bit_buf;                   /* 位缓冲（MSB 先出） */
static int bit_count;

static int put_bits(uint32_t value, int length)
{
    if (length <= 0) return 0;
    if (length > 16) return -1;
    bit_buf = (bit_buf << length) | (value & ((1u << length) - 1u));
    bit_count += length;
    while (bit_count >= 8) {
        bit_count -= 8;
        if (put_byte((uint8_t)(bit_buf >> bit_count))) return -1;
    }
    bit_buf &= (1u << bit_count) - 1u;   /* 只保留未输出的低位 */
    return 0;
}

static int flush_bits(void)
{
    if (bit_count > 0) {
        int pad = 8 - bit_count;            /* 尾部补 1 */
        if (put_bits((1u << pad) - 1u, pad)) return -1;
    }
    return 0;
}

/* ---------------- 量化表（IJG Annex K，质量缩放） ---------------- */
static const unsigned char base_qt_luma[64] = {
    16, 11, 10, 16, 24, 40, 51, 61,
    12, 12, 14, 19, 26, 58, 60, 55,
    14, 13, 16, 24, 40, 57, 69, 56,
    14, 17, 22, 29, 51, 87, 80, 62,
    18, 22, 37, 56, 68,109,103, 77,
    24, 35, 55, 64, 81,104,113, 92,
    49, 64, 78, 87,103,121,120,101,
    72, 92, 95, 98,112,100,103, 99
};
static const unsigned char base_qt_chroma[64] = {
    17, 18, 24, 47, 99, 99, 99, 99,
    18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99,
    47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99
};

static const unsigned char zigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63
};

static unsigned char qt_luma[64], qt_chroma[64];

static void build_quant_tables(int quality)
{
    if (quality < 1) quality = 1;
    if (quality > 100) quality = 100;
    int scale = (quality < 50) ? 5000 / quality : 200 - quality * 2;
    for (int i = 0; i < 64; i++) {
        int v = (base_qt_luma[i] * scale + 50) / 100;
        qt_luma[i] = (unsigned char)(v < 1 ? 1 : (v > 255 ? 255 : v));
        v = (base_qt_chroma[i] * scale + 50) / 100;
        qt_chroma[i] = (unsigned char)(v < 1 ? 1 : (v > 255 ? 255 : v));
    }
}

/* ---------------- 标准 Huffman 表（Annex K） ---------------- */
typedef struct {
    unsigned short code[256];
    unsigned char  size[256];
} huff_table;

static const unsigned char dc_luma_bits[17] =
    {0,0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0};
static const unsigned char dc_luma_vals[12] =
    {0,1,2,3,4,5,6,7,8,9,10,11};
static const unsigned char dc_chroma_bits[17] =
    {0,0,3,1,1,1,1,1,1,1,1,1,0,0,0,0,0};
static const unsigned char dc_chroma_vals[12] =
    {0,1,2,3,4,5,6,7,8,9,10,11};
static const unsigned char ac_luma_bits[17] =
    {0,0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,0x7d};
static const unsigned char ac_luma_vals[162] = {
    0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,
    0x22,0x71,0x14,0x32,0x81,0x91,0xa1,0x08,0x23,0x42,0xb1,0xc1,0x15,0x52,0xd1,0xf0,
    0x24,0x33,0x62,0x72,0x82,0x09,0x0a,0x16,0x17,0x18,0x19,0x1a,0x25,0x26,0x27,0x28,
    0x29,0x2a,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,0x49,
    0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
    0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
    0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
    0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,0xc4,0xc5,
    0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xe1,0xe2,
    0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
    0xf9,0xfa
};
static const unsigned char ac_chroma_bits[17] =
    {0,0,2,1,2,4,4,3,4,7,5,4,4,0,1,2,0x77};
static const unsigned char ac_chroma_vals[162] = {
    0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,
    0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,0xa1,0xb1,0xc1,0x09,0x23,0x33,0x52,0xf0,
    0x15,0x62,0x72,0xd1,0x0a,0x16,0x24,0x34,0xe1,0x25,0xf1,0x17,0x18,0x19,0x1a,0x26,
    0x27,0x28,0x29,0x2a,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,
    0x49,0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,
    0x69,0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x82,0x83,0x84,0x85,0x86,0x87,
    0x88,0x89,0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,
    0xa6,0xa7,0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,
    0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,
    0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
    0xf9,0xfa
};

static huff_table h_dc_luma, h_dc_chroma, h_ac_luma, h_ac_chroma;
static int huff_ready;

static void huff_build(huff_table *h, const unsigned char *bits, const unsigned char *vals)
{
    unsigned code = 0; int k = 0;
    memset(h->size, 0, sizeof(h->size));
    for (int len = 1; len <= 16; len++) {
        for (int i = 0; i < bits[len]; i++) {
            h->code[vals[k]] = (unsigned short)code;
            h->size[vals[k]] = (unsigned char)len;
            code++; k++;
        }
        code <<= 1;
    }
}

static void huff_init(void)
{
    huff_build(&h_dc_luma, dc_luma_bits, dc_luma_vals);
    huff_build(&h_dc_chroma, dc_chroma_bits, dc_chroma_vals);
    huff_build(&h_ac_luma, ac_luma_bits, ac_luma_vals);
    huff_build(&h_ac_chroma, ac_chroma_bits, ac_chroma_vals);
    huff_ready = 1;
}

/* ---------------- FDCT：Q14 余弦基表，行列分离 ----------------
 * tab[u][n] = 0.5*C(u)*cos((2n+1)*u*pi/16) * 2^14（JPEG DCT 规范尺度），
 * 两级变换后即为规范系数：DC = 8*均值，无需额外缩放。 */
static const int16_t dct_tab[8][8] = {
    { 5793,  5793,  5793,  5793,  5793,  5793,  5793,  5793},
    { 8035,  6811,  4551,  1598, -1598, -4551, -6811, -8035},
    { 7563,  3135, -3135, -7563, -7563, -3135,  3135,  7563},
    { 6811, -1598, -8035, -4551,  4551,  8035,  1598, -6811},
    { 5793, -5793, -5793,  5793,  5793, -5793, -5793,  5793},
    { 4551, -8035,  1598,  6811, -6811, -1598,  8035, -4551},
    { 3135, -7563,  7563, -3135, -3135,  7563, -7563,  3135},
    { 1598, -4551,  6811, -8035,  8035, -6811,  4551, -1598},
};

/* 原位 2D FDCT：输入 -128..127（自然序），输出自然序系数。
 * 行变换：tmp[u][n] = 行 n 的 1D DCT（u 为行频率）；
 * 列变换：对 tmp[u][*] 沿空间列方向做 1D DCT 得到列频率 v。
 * 自然序数组第一维是垂直频率v，第二维是水平频率u；写反会使每个8x8块转置。 */
static void fdct2d(int32_t *b)
{
    int32_t tmp[64];
    for (int n = 0; n < 8; n++)
        for (int u = 0; u < 8; u++) {
            int32_t sum = 0;
            const int16_t *tab = dct_tab[u];
            for (int k = 0; k < 8; k++) sum += b[n * 8 + k] * tab[k];
            tmp[u * 8 + n] = (sum + 8192) >> 14;
        }
    for (int u = 0; u < 8; u++)
        for (int v = 0; v < 8; v++) {
            int32_t sum = 0;
            const int16_t *tab = dct_tab[v];
            for (int m = 0; m < 8; m++) sum += tmp[u * 8 + m] * tab[m];
            b[v * 8 + u] = (sum + 8192) >> 14;
        }
}

/* ---------------- 段写入 ----------------
 * 注意：标记（marker）字节序列不允许 0xFF 填充——字节填充只适用于
 * 熵编码数据段。put_marker 直接写缓冲，绕过 put_byte 的填充逻辑。 */
static int put_marker(unsigned char m1)
{
    if (out_len + 2 > out_cap) return -1;
    out_buf[out_len++] = 0xFF;
    out_buf[out_len++] = m1;
    return 0;
}

static int put_16(unsigned v)
{
    return put_byte((uint8_t)(v >> 8)) || put_byte((uint8_t)v);
}

static int put_dqt(unsigned char id, const unsigned char *qt)
{
    if (put_marker(0xDB) || put_16(67) || put_byte(id)) return -1;
    for (int i = 0; i < 64; i++)
        if (put_byte(qt[zigzag[i]])) return -1;
    return 0;
}

static int put_dht(unsigned char tc_th, const unsigned char *bits,
                   const unsigned char *vals, int nvals)
{
    if (put_marker(0xC4)) return -1;
    if (put_16((unsigned)(3 + 16 + nvals))) return -1;
    if (put_byte(tc_th)) return -1;
    for (int i = 1; i <= 16; i++)
        if (put_byte(bits[i])) return -1;
    for (int i = 0; i < nvals; i++)
        if (put_byte(vals[i])) return -1;
    return 0;
}

/* ---------------- 块编码 ---------------- */
static int encode_block(int32_t *block, const unsigned char *qt,
                       huff_table *dc, huff_table *ac, int32_t *pred)
{
    fdct2d(block);
    /* 自然序量化（四舍五入）。 */
    int32_t q[64];
    for (int i = 0; i < 64; i++) {
        int32_t c = block[i];
        q[i] = (c < 0 ? c - (qt[i] >> 1) : c + (qt[i] >> 1)) / qt[i];
    }
    /* DC 差分。 */
    int32_t diff = q[0] - *pred;
    *pred = q[0];
    int s = 0, t = diff < 0 ? -diff : diff;
    while (t) { s++; t >>= 1; }
    if (s > 11) return -1;
    if (put_bits(dc->code[s], dc->size[s])) return -1;
    if (s) {
        int ds = (diff < 0) ? diff + (1 << s) - 1 : diff;
        if (put_bits((uint32_t)ds, s)) return -1;
    }
    /* AC 游程编码（ZigZag 序）。 */
    int run = 0;
    for (int k = 1; k < 64; k++) {
        int32_t v = q[zigzag[k]];
        if (v == 0) { run++; continue; }
        while (run > 15) {
            if (put_bits(ac->code[0xF0], ac->size[0xF0])) return -1;   /* ZRL */
            run -= 16;
        }
        int a = v < 0 ? -v : v;
        int bits = 0;
        while (a) { bits++; a >>= 1; }
        if (bits > 10) { run++; continue; }   /* 超类别系数按零跳过（极低质量才会发生） */
        unsigned symbol = (unsigned)(run << 4) | (unsigned)bits;
        if (put_bits(ac->code[symbol], ac->size[symbol])) return -1;
        int ds = (v < 0) ? v + (1 << bits) - 1 : v;
        if (put_bits((uint32_t)ds, bits)) return -1;
        run = 0;
    }
    if (run) {
        if (put_bits(ac->code[0x00], ac->size[0x00])) return -1;        /* EOB */
    }
    return 0;
}

/* YUYV 取样辅助（与检测器同源）。 */
#ifndef DART_DET_C_UV_SWAP
#define DART_DET_C_UV_SWAP 0
#endif
static inline uint8_t pix_cb(const uint8_t *f, int idx)
{
    const uint8_t *p = f + (idx & ~1) * 2;
    return DART_DET_C_UV_SWAP ? p[3] : p[1];
}
static inline uint8_t pix_cr(const uint8_t *f, int idx)
{
    const uint8_t *p = f + (idx & ~1) * 2;
    return DART_DET_C_UV_SWAP ? p[1] : p[3];
}

/* ---------------- 主入口 ---------------- */

int jpeg_encode_yuv422(const uint8_t *yuv422, int quality,
                       uint8_t *out, uint32_t *out_size)
{
    if (!yuv422 || !out || !out_size || quality < 1 || quality > 100) return -1;
    if (W % 16 || H % 16) return -1;
    if (!huff_ready) huff_init();
    build_quant_tables(quality);
    out_buf = out; out_len = 0; out_cap = *out_size;
    bit_buf = 0; bit_count = 0;

    /* ---- JFIF 头 ---- */
    if (put_marker(0xD8)) return -2;                        /* SOI */
    if (put_marker(0xE0) || put_16(16)) return -2;         /* APP0 */
    if (put_byte('J') || put_byte('F') || put_byte('I') || put_byte('F') || put_byte(0)) return -2;
    if (put_16(0x0102)) return -2;                          /* 版本 1.2 */
    if (put_byte(0) || put_16(1) || put_16(1) || put_byte(0) || put_byte(0)) return -2;
    if (put_dqt(0, qt_luma) || put_dqt(1, qt_chroma)) return -2;
    /* SOF0：Y 2x2 采样，Cb/Cr 1x1。 */
    if (put_marker(0xC0) || put_16(17) || put_byte(8)) return -2;
    if (put_16((unsigned)H) || put_16((unsigned)W) || put_byte(3)) return -2;
    if (put_byte(1) || put_byte(0x22) || put_byte(0)) return -2;
    if (put_byte(2) || put_byte(0x11) || put_byte(1)) return -2;
    if (put_byte(3) || put_byte(0x11) || put_byte(1)) return -2;
    if (put_dht(0x00, dc_luma_bits, dc_luma_vals, 12)) return -2;
    if (put_dht(0x10, ac_luma_bits, ac_luma_vals, 162)) return -2;
    if (put_dht(0x01, dc_chroma_bits, dc_chroma_vals, 12)) return -2;
    if (put_dht(0x11, ac_chroma_bits, ac_chroma_vals, 162)) return -2;
    if (put_marker(0xDA) || put_16(12) || put_byte(3)) return -2;   /* SOS */
    if (put_byte(1) || put_byte(0x00)) return -2;
    if (put_byte(2) || put_byte(0x11)) return -2;
    if (put_byte(3) || put_byte(0x11)) return -2;
    if (put_byte(0) || put_byte(63) || put_byte(0)) return -2;

    /* ---- 逐 MCU（16x16）编码：4 亮度块 + Cb + Cr。 ---- */
    int32_t block[64];
    int32_t pred_y = 0, pred_cb = 0, pred_cr = 0;
    for (int my = 0; my < H / 16; my++) {
        for (int mx = 0; mx < W / 16; mx++) {
            for (int by = 0; by < 2; by++) {
                for (int bx = 0; bx < 2; bx++) {
                    for (int r = 0; r < 8; r++) {
                        const uint8_t *src = yuv422 + ((my * 16 + by * 8 + r) * W + mx * 16 + bx * 8) * 2;
                        for (int c = 0; c < 8; c++)
                            block[r * 8 + c] = (int32_t)src[c * 2] - 128;
                    }
                    if (encode_block(block, qt_luma, &h_dc_luma, &h_ac_luma, &pred_y)) return -2;
                }
            }
            /* 色度 4:2:2 -> 4:2:0：水平取偶数列色度，垂直两行平均。 */
            for (int part = 0; part < 2; part++) {
                for (int r = 0; r < 8; r++) {
                    int y_abs = my * 16 + r * 2;
                    for (int c = 0; c < 8; c++) {
                        int x_abs = mx * 16 + c * 2;
                        int i0 = y_abs * W + x_abs, i1 = i0 + W;
                        uint8_t c0 = part ? pix_cr(yuv422, i0) : pix_cb(yuv422, i0);
                        uint8_t c1 = part ? pix_cr(yuv422, i1) : pix_cb(yuv422, i1);
                        block[r * 8 + c] = ((int32_t)c0 + (int32_t)c1) / 2 - 128;
                    }
                }
                if (encode_block(block, qt_chroma, &h_dc_chroma, &h_ac_chroma,
                                 part ? &pred_cr : &pred_cb)) return -2;
            }
        }
    }
    if (flush_bits()) return -2;
    if (put_marker(0xD9)) return -2;                        /* EOI */

    enc_frames++;
    enc_bytes_total += out_len;
    if (out_len > enc_max_frame_bytes) enc_max_frame_bytes = out_len;
    *out_size = out_len;
    return 0;
}
