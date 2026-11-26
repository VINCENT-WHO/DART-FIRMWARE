/* 机载绿灯识别：从 OV5640 的 YUV422 帧中定位引导灯主候选。
 * 算法与电脑端 green_light_detector（config/rec00038_white_core.yml 白色灯芯配置）
 * 保持同一套阈值与判定流程：HSV 双掩膜 -> 3x3 中值 -> 外轮廓形状筛选 -> 绿色
 * 邻域验证 -> 评分选主候选。阈值不改动；仅把 OpenCV 调用替换为定点 C 实现。
 * 坐标系与电脑端一致：原点在图像左上，X 向右、Y 向下，单位为像素。
 * 本文件在 H750 固件与主机验证程序中共用，不依赖任何 HAL。 */
#include "green_detector.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ---------------- 内存布局 ----------------
 * 掩膜放 AXI SRAM（.camera_buffer 段，与相机帧缓冲共用 RAM 区）；
 * 连通域工作区放 D2 SRAM（.camera_d2 段）。主机编译时退化为普通数组。 */
#if defined(__arm__)
#define DART_DET_AXI __attribute__((section(".camera_buffer"), aligned(32)))
#define DART_DET_D2  __attribute__((section(".camera_d2"), aligned(32)))
#else
#define DART_DET_AXI
#define DART_DET_D2
#endif

#define W DART_DET_WIDTH
#define H DART_DET_HEIGHT

static uint8_t green_mask[DART_DET_PIXELS] DART_DET_AXI;
static uint8_t core_mask[DART_DET_PIXELS] DART_DET_AXI;
static uint8_t comp_raster[(DART_DET_PIXELS + 7) / 8] DART_DET_D2;  /* 当前连通域（全图坐标 1bit） */
static uint8_t dil_raster[(DART_DET_PIXELS + 7) / 8] DART_DET_D2;   /* 膨胀邻域光栅 */
static int32_t flood_stack[16384] DART_DET_D2;                       /* 淹水填充坐标栈 */
static uint8_t med_rows_a[3][W] DART_DET_D2;                         /* 中值滤波行缓存（绿晕） */
static uint8_t med_rows_b[3][W] DART_DET_D2;                         /* 中值滤波行缓存（灯芯） */

volatile uint32_t det_eval_frames, det_found_frames, det_max_us, det_draw_max_us;

void det_reset_stats(void)
{
    det_eval_frames = det_found_frames = det_max_us = det_draw_max_us = 0;
}

/* ---------------- YUV422 访问 ----------------
 * OV5640 YUV422 字节序：Y0 Cb Y1 Cr（每 4 字节 2 个像素）。
 * 若台架实测发现蓝红调换，将 DART_DET_C_UV_SWAP 置 1 即可整体交换 U/V。 */
#ifndef DART_DET_C_UV_SWAP
#define DART_DET_C_UV_SWAP 0
#endif

static inline uint8_t pix_y(const uint8_t *frame, int idx)
{
    return frame[idx * 2];
}

static inline uint8_t pix_cb(const uint8_t *frame, int idx)
{
    const uint8_t *p = frame + (idx & ~1) * 2;
    return DART_DET_C_UV_SWAP ? p[3] : p[1];
}

static inline uint8_t pix_cr(const uint8_t *frame, int idx)
{
    const uint8_t *p = frame + (idx & ~1) * 2;
    return DART_DET_C_UV_SWAP ? p[1] : p[3];
}

/* ---------------- 掩膜生成与中值滤波 ---------------- */

/* 3x3 二值中值滤波（边界复制），等价 cv::medianBlur(mask, mask, 3)。
 * 掩膜为二值（0/255）：9 个值的中值 = 多数表决（>=5 个非零输出 255），
 * 与通用 median9 排序结果逐位一致，但省去排序网络。
 * 三行缓存只保留原始输入；不能重新读取已被输出覆盖的上一行。 */
static void median3(uint8_t *mask, uint8_t rows[3][W])
{
    uint8_t *r0 = rows[0], *r1 = rows[1], *r2 = rows[2];
    /* 第一行的上邻居复制第一行；下邻居先保存尚未修改的第二行。 */
    memcpy(r0, mask, W);
    memcpy(r1, mask, W);
    memcpy(r2, mask + (H > 1 ? W : 0), W);
    for (int y = 0; y < H; y++) {
        uint8_t *dst = mask + y * W;
        for (int x = 0; x < W; x++) {
            int xl = x > 0 ? x - 1 : 0;
            int xr = x < W - 1 ? x + 1 : W - 1;
            int ones = (r0[xl] != 0) + (r0[x] != 0) + (r0[xr] != 0)
                     + (r1[xl] != 0) + (r1[x] != 0) + (r1[xr] != 0)
                     + (r2[xl] != 0) + (r2[x] != 0) + (r2[xr] != 0);
            dst[x] = ones >= 5 ? 255 : 0;
        }
        if (y + 1 < H) {
            /* 轮换指针而非覆盖原图行；新读取的y+2行尚未写入输出。
             * 到底边时复制最后一行，使上下左右均采用边界复制语义。 */
            uint8_t *reuse = r0; r0 = r1; r1 = r2; r2 = reuse;
            int next = y + 2 < H ? y + 2 : H - 1;
            memcpy(r2, mask + next * W, W);
        }
    }
}

/* 生成双掩膜：逐像素 YCbCr->RGB->HSV（OpenCV 8 位语义）后套用阈值。
 * BT.601 全幅定点换算；S、H 整除行为与 OpenCV 8 位一致（向零截断）。 */
static void build_masks(const uint8_t *frame)
{
    for (int y = 0; y < H; y++) {
        uint8_t *gm = green_mask + y * W;
        uint8_t *cm = core_mask + y * W;
        for (int x = 0; x < W; x++) {
            int idx = y * W + x;
            int Y = pix_y(frame, idx);
            int cbv = (int)pix_cb(frame, idx) - 128;
            int crv = (int)pix_cr(frame, idx) - 128;
            int r = Y + ((91881 * crv + 32768) >> 16);
            int g = Y - (((22554 * cbv + 46802 * crv) + 32768) >> 16);
            int b = Y + ((116130 * cbv + 32768) >> 16);
            r = r < 0 ? 0 : (r > 255 ? 255 : r);
            g = g < 0 ? 0 : (g > 255 ? 255 : g);
            b = b < 0 ? 0 : (b > 255 ? 255 : b);
            int mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
            int mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
            int diff = mx - mn;
            int s = mx ? (diff * 255) / mx : 0;
            int h = 0;
            if (diff) {
                if (mx == r)      h = 30 * (g - b) / diff;
                else if (mx == g) h = 60 + 30 * (b - r) / diff;
                else              h = 120 + 30 * (r - g) / diff;
                if (h < 0) h += 180;
            }
            /* 绿晕：H∈[35,95] 且 S>=45 且 V>=140；灯芯：V>=252 且 S<=35（不限 H）。 */
            gm[x] = (h >= DART_DET_HUE_MIN && h <= DART_DET_HUE_MAX &&
                     s >= DART_DET_SAT_MIN && mx >= DART_DET_GREEN_V_MIN) ? 255 : 0;
            cm[x] = (mx >= DART_DET_CORE_V_MIN && s <= DART_DET_CORE_SAT_MAX) ? 255 : 0;
        }
    }
}

/* OpenCV getStructuringElement(MORPH_ELLIPSE, 15x15) 的精确位图（169 个 1）。 */
static const unsigned char support_kernel[15][15] = {
    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
    {0,0,0,1,1,1,1,1,1,1,1,1,0,0,0},
    {0,0,1,1,1,1,1,1,1,1,1,1,1,0,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
    {0,0,1,1,1,1,1,1,1,1,1,1,1,0,0},
    {0,0,0,1,1,1,1,1,1,1,1,1,0,0,0},
    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
};

/* 核内 1 偏移表：按 |dx|+|dy| 从小到大排序，邻域判定可提前退出。 */
typedef struct { int8_t dx, dy; } kernel_offset;
static kernel_offset kernel_offsets[169];
static int kernel_offsets_ready;

static void init_kernel_offsets(void)
{
    int n = 0;
    for (int dy = -7; dy <= 7; dy++)
        for (int dx = -7; dx <= 7; dx++)
            if (support_kernel[dy + 7][dx + 7]) {
                kernel_offsets[n].dx = (int8_t)dx;
                kernel_offsets[n].dy = (int8_t)dy;
                n++;
            }
    for (int i = 1; i < n; i++) {
        kernel_offset k = kernel_offsets[i];
        int w = abs(k.dx) + abs(k.dy);
        int j = i - 1;
        while (j >= 0 && abs(kernel_offsets[j].dx) + abs(kernel_offsets[j].dy) > w) {
            kernel_offsets[j + 1] = kernel_offsets[j];
            j--;
        }
        kernel_offsets[j + 1] = k;
    }
    kernel_offsets_ready = (n == 169);
}

/* 全图坐标位图辅助。 */
static inline int raster_get(const uint8_t *raster, int idx)
{
    return (raster[idx >> 3] >> (idx & 7)) & 1;
}

static inline void raster_set(uint8_t *raster, int idx)
{
    raster[idx >> 3] |= (uint8_t)(1u << (idx & 7));
}

/* 淹水填充（8 连通）：core_mask 中包含 (sx,sy) 的连通域。
 * 像素集同步标记进 comp_raster（供轮廓跟踪与邻域判定），并从 core_mask 抹除，
 * 防止同一连通域被重复处理。返回像素数；栈溢出时返回 0（该域放弃）。 */
static uint32_t flood_component(int sx, int sy, int *bx, int *by, int *bw, int *bh,
                                int64_t *sum_x, int64_t *sum_y)
{
    memset(comp_raster, 0, sizeof(comp_raster));
    int min_x = sx, max_x = sx, min_y = sy, max_y = sy;
    uint32_t count = 0;
    int64_t acc_x = 0, acc_y = 0;
    unsigned top = 0;
    flood_stack[top++] = sy * W + sx;
    while (top) {
        int idx = flood_stack[--top];
        if (!core_mask[idx]) continue;
        core_mask[idx] = 0;
        raster_set(comp_raster, idx);
        int x = idx % W, y = idx / W;
        count++; acc_x += x; acc_y += y;
        if (x < min_x) min_x = x; if (x > max_x) max_x = x;
        if (y < min_y) min_y = y; if (y > max_y) max_y = y;
        /* 8 邻域压栈；栈满则放弃该域（返回 0），不写 core_mask 之外的副作用。 */
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (!dx && !dy) continue;
                int nx = x + dx, ny = y + dy;
                if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                if (top >= 16384) { count = 0; goto done; }
                flood_stack[top++] = ny * W + nx;
            }
        }
    }
done:
    if (!count) return 0;
    *bx = min_x; *by = min_y; *bw = max_x - min_x + 1; *bh = max_y - min_y + 1;
    *sum_x = acc_x; *sum_y = acc_y;
    return count;
}

/* ---------------- 外轮廓跟踪（Moore 邻域跟踪） ----------------
 * 从起点沿 8 邻域顺时针绕外轮廓一圈，得到有序边界像素序列。
 * 输出链长（轮廓点数）、周长（相邻点欧氏距离和，与 cv::arcLength 一致）、
 * 多边形面积（Green 公式，与 cv::contourArea 一致）以及到质心的最大距离平方。 */
static const int dirs8[8][2] = {
    {1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}
};   /* 顺时针：东、东南、南、西南、西、西北、北、东北 */

/* 新回溯方向表：扫描序列中 p 前最后被检查的背景像素相对新当前点 p
 * 的方向编号（由相邻方向差几何关系导出）。 */
static const int backtrack[8] = { 6, 6, 0, 0, 2, 2, 4, 4 };

static int trace_boundary(int sx, int sy, int cx_int, int cy_int,
                          double *perimeter, int64_t *area_x2, int32_t *max_d2)
{
    int cx = sx, cy = sy;
    int b_dir = 4;              /* 起点进入方向：西（左侧非域，扫描判据保证） */
    double peri = 0.0;
    int64_t area2 = 0;
    int32_t maxd2 = 0;
    int points = 0;
    for (int guard = 0; guard < 40000; guard++) {
        /* 从回溯方向的下一个方向开始顺时针找第一个域内像素。 */
        int found = -1;
        for (int step = 1; step <= 8; step++) {
            int d = (b_dir + step) & 7;
            int nx = cx + dirs8[d][0], ny = cy + dirs8[d][1];
            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
            if (raster_get(comp_raster, ny * W + nx)) { found = d; break; }
        }
        if (found < 0) break;   /* 单像素域：周长 0，轮廓闭合 */
        int nx = cx + dirs8[found][0], ny = cy + dirs8[found][1];
        peri += (found & 1) ? 1.41421356 : 1.0;
        area2 += (int64_t)cx * ny - (int64_t)nx * cy;
        b_dir = backtrack[found];         /* 最后检查的背景像素相对新当前点 */
        cx = nx; cy = ny;
        points++;
        /* 到整数质心的最远距离平方（画圈半径依据）。 */
        int dxq = cx - cx_int, dyq = cy - cy_int;
        int32_t d2 = dxq * dxq + dyq * dyq;
        if (d2 > maxd2) maxd2 = d2;
        /* 终止：移动后回到起点即完成一圈。 */
        if (cx == sx && cy == sy && points > 2) break;
    }
    *perimeter = peri;
    *area_x2 = area2 < 0 ? -area2 : area2;  /* 顺逆时针取绝对值 */
    *max_d2 = maxd2;
    return points;
}

/* 邻域验证：把连通域用 15x15 椭圆核膨胀（与 cv::dilate(component, kernel) 一致，
 * 图像边界外不扩张），统计膨胀集像素数与其中绿晕掩膜像素数。 */
static void green_neighborhood(int bx, int by, int bw, int bh,
                               int *support_count, int *green_count)
{
    memset(dil_raster, 0, sizeof(dil_raster));
    int x0 = bx > 7 ? bx - 7 : 0;
    int y0 = by > 7 ? by - 7 : 0;
    int x1 = (bx + bw + 7) < W ? bx + bw + 7 : W;
    int y1 = (by + bh + 7) < H ? by + bh + 7 : H;
    int support = 0, green = 0;
    for (int y = y0; y < y1; y++) {
        for (int x = x0; x < x1; x++) {
            int hit = 0;
            for (int k = 0; k < 169; k++) {
                int qx = x + kernel_offsets[k].dx;
                int qy = y + kernel_offsets[k].dy;
                if (qx < 0 || qx >= W || qy < 0 || qy >= H) continue;
                int idx = qy * W + qx;
                if (raster_get(comp_raster, idx)) { hit = 1; break; }
            }
            if (hit) {
                int idx = y * W + x;
                raster_set(dil_raster, idx);
                support++;
                if (green_mask[idx]) green++;
            }
        }
    }
    *support_count = support;
    *green_count = green;
}

/* ---------------- 主检测入口 ---------------- */

int green_detect(const uint8_t *yuv422, dart_detection *out)
{
    if (!yuv422 || !out) return -1;
    memset(out, 0, sizeof(*out));
    det_eval_frames++;
    if (!kernel_offsets_ready) init_kernel_offsets();

    build_masks(yuv422);
    median3(green_mask, med_rows_a);
    median3(core_mask, med_rows_b);

    /* 逐个处理灯芯掩膜的外连通域。起点判据：域内像素且左侧不在域内，
     * 与 cv::findContours 外轮廓起点的常见情形一致；凹陷形状的完整外边界
     * 由闭合跟踪覆盖，重复起点会在域被抹除后自然跳过。 */
    double best_score = 0.0;
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            int idx = y * W + x;
            if (!core_mask[idx]) continue;
            if (x > 0 && core_mask[idx - 1]) continue;

            int bx, by, bw, bh;
            int64_t sum_x, sum_y;
            uint32_t pixels = flood_component(x, y, &bx, &by, &bw, &bh, &sum_x, &sum_y);
            if (!pixels) continue;   /* 栈溢出：放弃该域 */

            /* 外接矩形尺寸先于面积参与筛选（与 OpenCV 判序一致：面积->长宽比）。 */
            double area = 0.0;
            int64_t area_x2;
            double perimeter;
            int32_t max_d2;
            int cx_int = (int)(sum_x / pixels);
            int cy_int = (int)(sum_y / pixels);
            if (!trace_boundary(x, y, cx_int, cy_int, &perimeter, &area_x2, &max_d2)) continue;
            area = (double)area_x2 * 0.5;

            /* 1) 轮廓多边形面积范围：[280, 0.08*总像素=6144] 像素^2。 */
            if (area < (double)DART_DET_AREA_MIN) continue;
            if (area * 10000 > 0.08 * 10000.0 * (double)DART_DET_PIXELS) continue;
            /* 2) 外接矩形长短边比 <= 2.4。 */
            {
                int longer = bw > bh ? bw : bh, shorter = bw < bh ? bw : bh;
                if (longer * 5 > shorter * DART_DET_ASPECT_MAX) continue;
            }
            /* 3) 圆度 = 4*pi*area/perimeter^2 >= 0.35。 */
            double circularity = 0.0;
            if (perimeter <= 0.0) continue;
            circularity = 4.0 * 3.14159265358979 * area / (perimeter * perimeter);
            if (circularity < 0.35) continue;
            /* 4) 绿色邻域验证：膨胀集内绿晕像素 >= 20 且占比 >= 0.08。 */
            int support_count, green_count;
            green_neighborhood(bx, by, bw, bh, &support_count, &green_count);
            if (green_count < DART_DET_GREEN_PIX_MIN) continue;
            if ((int64_t)green_count * 10000 < (int64_t)support_count * DART_DET_GREEN_FRACTION_MIN) continue;
            double green_fraction = support_count ? (double)green_count / support_count : 0.0;
            /* 5) 评分并保留最佳：score = area*(0.5+green_fraction)*(0.5+circularity)。 */
            double score = area * (0.5 + green_fraction) * (0.5 + circularity);
            if (score <= best_score) continue;
            best_score = score;
            out->found = 1;
            out->center_x_q16 = (int32_t)((sum_x << 16) / pixels);
            out->center_y_q16 = (int32_t)((sum_y << 16) / pixels);
            out->radius = (int)(sqrtf((float)max_d2) + 0.5f) + 3;
            out->box_x = bx; out->box_y = by; out->box_w = bw; out->box_h = bh;
            out->area_q8 = (int32_t)(area * 256.0 + 0.5);
            out->circularity_x500 = (int)(circularity * 500.0 + 0.5);
            out->green_fraction_x10000 = (int)(green_fraction * 10000.0 + 0.5);
            out->score_x10000 = (int32_t)(score * 10000.0 + 0.5);
        }
    }
    if (out->found) det_found_frames++;
    return 0;
}
