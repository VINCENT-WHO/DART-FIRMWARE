/* 机载标注绘制：把检测结果画到 YUV422 帧上——黄圈、红十字中心、
 * 中心坐标文本、帧号与相对时间文本、未检出提示。与电脑端 annotate 的
 * 视觉约定一致：黄圈 2 像素粗，十字与中心点为红色，坐标单位为像素。 */
#include "green_detector.h"
#include <string.h>

#define W DART_DET_WIDTH
#define H DART_DET_HEIGHT

/* OV5640 YUV422 字节序：Y0 Cb Y1 Cr。写一个像素时把它与共享色度的
 * 相邻像素同步改色（4:2:2 水平成对），线条粗细因此有 1~2 像素不均。 */
static void set_pixel(uint8_t *frame, int x, int y, uint8_t yv, uint8_t cb, uint8_t cr)
{
    if ((unsigned)x >= W || (unsigned)y >= H) return;
    int idx = y * W + x;
    frame[idx * 2] = yv;
    frame[(idx ^ 1) * 2] = yv;
    uint8_t *c = frame + (idx & ~1) * 2;
    c[1] = cb; c[3] = cr;
}

/* 画布颜色（BT.601 全幅 YCbCr）：
 * 黄 (B=0,G=255,R=255) -> Y=225 Cb=1   Cr=149
 * 红 (B=0,G=0,R=255)   -> Y=76  Cb=85  Cr=255
 * 黑                    -> Y=16  Cb=128 Cr=128（文本阴影用近似黑） */
#define YEL_Y 225, 1, 149
#define RED_Y 76, 85, 255
#define BLK_Y 16, 128, 128

static void draw_circle(uint8_t *frame, int cx, int cy, int r, uint8_t yv, uint8_t cb, uint8_t cr)
{
    if (r <= 0) return;
    int x = 0, yy = r, d = 1 - r;           /* 中点圆算法 */
    while (x <= yy) {
        set_pixel(frame, cx + x, cy + yy, yv, cb, cr);
        set_pixel(frame, cx - x, cy + yy, yv, cb, cr);
        set_pixel(frame, cx + x, cy - yy, yv, cb, cr);
        set_pixel(frame, cx - x, cy - yy, yv, cb, cr);
        set_pixel(frame, cx + yy, cy + x, yv, cb, cr);
        set_pixel(frame, cx - yy, cy + x, yv, cb, cr);
        set_pixel(frame, cx + yy, cy - x, yv, cb, cr);
        set_pixel(frame, cx - yy, cy - x, yv, cb, cr);
        if (d < 0) d += 2 * x + 3;
        else { d += 2 * (x - yy) + 5; yy--; }
        x++;
    }
}

/* 5x7 点阵字库：数字、括号、标点与所需字母（列 5 位，行 7 字节）。 */
static const unsigned char font5x7[128][7] = {
    ['0'] = {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},
    ['1'] = {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
    ['2'] = {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F},
    ['3'] = {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E},
    ['4'] = {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},
    ['5'] = {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
    ['6'] = {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},
    ['7'] = {0x1F,0x11,0x02,0x04,0x08,0x08,0x08},
    ['8'] = {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
    ['9'] = {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},
    ['('] = {0x02,0x04,0x08,0x08,0x08,0x04,0x02},
    [')'] = {0x08,0x04,0x02,0x02,0x02,0x04,0x08},
    [','] = {0x00,0x00,0x00,0x00,0x0C,0x04,0x08},
    ['.'] = {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C},
    ['='] = {0x00,0x00,0x1F,0x00,0x1F,0x00,0x00},
    ['f'] = {0x06,0x08,0x1E,0x08,0x08,0x08,0x08},
    ['r'] = {0x00,0x00,0x16,0x19,0x08,0x08,0x08},
    ['a'] = {0x00,0x0E,0x11,0x1F,0x10,0x11,0x0F},
    ['m'] = {0x00,0x1C,0x14,0x14,0x14,0x14,0x14},
    ['e'] = {0x00,0x0E,0x11,0x1F,0x10,0x11,0x0E},
    ['t'] = {0x04,0x04,0x0E,0x04,0x04,0x04,0x06},
    ['s'] = {0x00,0x0E,0x10,0x0E,0x01,0x11,0x0E},
    ['N'] = {0x11,0x19,0x15,0x13,0x11,0x11,0x11},
    ['O'] = {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
    ['T'] = {0x0E,0x04,0x04,0x04,0x04,0x04,0x04},
    ['F'] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
    ['U'] = {0x11,0x11,0x11,0x11,0x11,0x11,0x0E},
    ['D'] = {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E},
};

/* 画一段文本；先以黑色偏移一像素描影，再画主体颜色，保证亮背景可读。 */
static void draw_text_shadow(uint8_t *frame, int x0, int y0,
                             uint8_t yv, uint8_t cb, uint8_t cr, const char *text)
{
    for (int pass = 0; pass < 2; pass++) {
        int x = x0 + (pass ? 0 : 1), y = y0 + (pass ? 0 : 1);
        for (const char *s = text; *s; s++, x += 6) {
            unsigned ch = (unsigned)(unsigned char)*s;
            const unsigned char *glyph = (ch < 128) ? font5x7[ch] : 0;
            if (!glyph) continue;
            int blank = !glyph[0] && !glyph[1] && ch != ' ';
            if (blank) continue;
            for (int row = 0; row < 7; row++) {
                unsigned bits = glyph[row];
                for (int col = 0; col < 5; col++)
                    if (bits & (0x10u >> col)) {
                        if (pass) set_pixel(frame, x + col, y + row, yv, cb, cr);
                        else set_pixel(frame, x + col, y + row, BLK_Y);
                    }
            }
        }
    }
}

static int u32_to_text(char *out, uint32_t value)
{
    char tmp[12]; int n = 0;
    do { tmp[n++] = (char)('0' + value % 10u); value /= 10u; } while (value);
    for (int i = 0; i < n; i++) out[i] = tmp[n - 1 - i];
    return n;
}

/* x10 定点（真实值=value/10）转文本，保留 1 位小数。 */
static int fixed10_to_text(char *out, uint32_t value)
{
    int n = u32_to_text(out, value / 10u);
    out[n++] = '.';
    out[n++] = (char)('0' + value % 10u);
    return n;
}

/* 时间文本：time_ms_q8 为相对触发的毫秒数（Q8 定点），
 * 按"t=秒.毫秒 s"显示，与记录文件的时间含义一致。 */
static int time_to_text(char *out, int32_t time_ms_q8)
{
    uint32_t us = (uint32_t)(((int64_t)time_ms_q8 * 1000) >> 8);
    uint32_t whole_s = us / 1000000u;
    uint32_t frac_us = us % 1000000u;
    int n = 0;
    out[n++] = 't'; out[n++] = '=';
    n += u32_to_text(out + n, whole_s);
    out[n++] = '.';
    for (int shift = 100000; shift >= 1; shift /= 10)
        out[n++] = (char)('0' + (frac_us / (uint32_t)shift) % 10u);
    out[n++] = 's';
    return n;
}

void green_annotate(uint8_t *yuv422, const dart_detection *det,
                    int frame_index, int32_t time_ms_q8)
{
    /* 左上角帧信息：帧号与相对触发时间（单位：帧号、秒）。 */
    char line[40];
    int n = 0;
    const char prefix[] = "frame ";
    for (const char *s = prefix; *s; s++) line[n++] = *s;
    n += u32_to_text(line + n, (uint32_t)frame_index);
    line[n++] = ' ';
    n += time_to_text(line + n, time_ms_q8);
    line[n] = 0;
    draw_text_shadow(yuv422, 4, 4, YEL_Y, line);

    if (!det->found) {
        draw_text_shadow(yuv422, 4, 16, RED_Y, "NOT FOUND");
        return;
    }
    int cx = det->center_x_q16 >> 16;
    int cy = det->center_y_q16 >> 16;
    int radius = det->radius;

    /* 黄圈 2 像素粗（两个半径各一圈），与电脑端 cv::circle thickness=2 对应。 */
    draw_circle(yuv422, cx, cy, radius, YEL_Y);
    draw_circle(yuv422, cx, cy, radius - 1, YEL_Y);

    /* 红色十字（臂长 11）与中心实心点（半径 2）。 */
    for (int d = -5; d <= 5; d++) {
        set_pixel(yuv422, cx + d, cy, RED_Y);
        set_pixel(yuv422, cx, cy + d, RED_Y);
    }
    for (int dy = -2; dy <= 2; dy++)
        for (int dx = -2; dx <= 2; dx++)
            if (dx * dx + dy * dy <= 5)
                set_pixel(yuv422, cx + dx, cy + dy, RED_Y);

    /* 坐标文本 "(x.x, y.y)"：位于圈上方，随边界收紧防止出画。Q16 质心四舍
     * 五入到 0.1 像素（单位：像素）。 */
    uint32_t x10 = (uint32_t)(((int64_t)det->center_x_q16 * 10 + 32768) >> 16);
    uint32_t y10 = (uint32_t)(((int64_t)det->center_y_q16 * 10 + 32768) >> 16);
    n = 0;
    line[n++] = '(';
    n += fixed10_to_text(line + n, x10);
    line[n++] = ','; line[n++] = ' ';
    n += fixed10_to_text(line + n, y10);
    line[n++] = ')';
    line[n] = 0;
    int text_w = n * 6 - 1;
    int tx = cx - text_w / 2;
    if (tx < 2) tx = 2;
    if (tx > W - text_w - 2) tx = W - text_w - 2;
    int ty = cy - radius - 5 - 7;           /* 顶行制：位于圈上缘 5 像素之上 */
    if (ty < 2) ty = 2;
    if (ty > H - 9) ty = H - 9;
    draw_text_shadow(yuv422, tx, ty, YEL_Y, line);
}
