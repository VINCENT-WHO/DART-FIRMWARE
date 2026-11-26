/* RGB绿灯识别：YUYV转RGB -> G阈值/去白 -> 3x3滑动中值 ->
 * 8连通域像素面积筛选 -> 选择最大区域，输出绿色像素质心。
 * 检出后用1.5倍包围盒跨帧搜索，丢失/截断时回退全图。
 * 输入只读，不依赖HAL；矩形/文字由green_annotate另行绘制。 */
#include "green_detector.h"
#include <string.h>
#include <math.h>

#if defined(__arm__)
#define DART_DET_AXI __attribute__((section(".camera_buffer"), aligned(32)))
#define DART_DET_D2 __attribute__((section(".camera_d2"), aligned(32)))
#else
#define DART_DET_AXI
#define DART_DET_D2
#endif
#define W DART_DET_WIDTH
#define H DART_DET_HEIGHT
#ifndef DART_DET_C_UV_SWAP
#define DART_DET_C_UV_SWAP 0
#endif

static uint8_t green_mask[DART_DET_PIXELS] DART_DET_AXI;
static uint8_t med_rows_a[3][W] DART_DET_D2;
static int32_t flood_stack[16384] DART_DET_D2;
/* 栈满时存储已发现但未展开的像素，保证大区域完整消费，不切碎成假目标。 */
static uint8_t deferred[(DART_DET_PIXELS + 7) / 8] DART_DET_D2;
typedef struct { int x0, y0, x1, y1; } detection_region; /* 右/下界不包含 */
static const detection_region full_region = {0, 0, W, H};
static detection_region next_region = {0, 0, W, H};
void det_reset_tracking(void) { next_region = full_region; }

static int region_is_full(const detection_region *r)
{
    return r->x0 == 0 && r->y0 == 0 && r->x1 == W && r->y1 == H;
}
static detection_region expand_box(const dart_detection *d)
{
    /* 各边向外扩原尺寸的1/4，整数边界向外取整，中心不偏移。
     * 先扩再裁，触边时不把另一侧向内平移。 */
    int px = (d->box_w + 3) / 4, py = (d->box_h + 3) / 4;
    detection_region r = {d->box_x - px, d->box_y - py,
        d->box_x + d->box_w + px, d->box_y + d->box_h + py};
    if (r.x0 < 0) r.x0 = 0; if (r.y0 < 0) r.y0 = 0;
    if (r.x1 > W) r.x1 = W; if (r.y1 > H) r.y1 = H;
    return r;
}
static int box_touches_inner_edge(const dart_detection *d, const detection_region *r)
{
    return (r->x0 > 0 && d->box_x == r->x0) ||
        (r->y0 > 0 && d->box_y == r->y0) ||
        (r->x1 < W && d->box_x + d->box_w == r->x1) ||
        (r->y1 < H && d->box_y + d->box_h == r->y1);
}
volatile uint32_t det_eval_frames, det_found_frames, det_max_us, det_draw_max_us;
void det_reset_stats(void)
{
    det_eval_frames = det_found_frames = det_max_us = det_draw_max_us = 0;
}

/* 3x3 二值中值滤波（边界复制），等价 cv::medianBlur(mask, mask, 3)。
 * 掩膜为二值（0/255）：9 个值的中值 = 多数表决（>=5 个非零输出 255），
 * 滑动直方图只需两个桶：hist[255]=ones，hist[0]=9-ones。
 * 每次右移减去最左列、加入最右列，不重新统计整个窗口；每像素O(3)。
 * 三个列计数循环复用，新增列只读取3个像素，不分配256桶灰度直方图。
 * 三行缓存只保留原始输入；不能重新读取已被输出覆盖的上一行。 */
static void median3_region(uint8_t *mask, uint8_t rows[3][W], const detection_region *roi)
{
    uint8_t *r0 = rows[0], *r1 = rows[1], *r2 = rows[2];
    /* 保留ROI周围1像素真实邻居；只在图像边界复制，不在ROI边界复制。
     * 行缓存按ROI+halo宽度拷贝，输出仅覆盖ROI。 */
    int xbase = roi->x0 > 0 ? roi->x0 - 1 : 0;
    int xend = roi->x1 < W ? roi->x1 + 1 : W;
    size_t width = (size_t)(xend - xbase);
    int above = roi->y0 > 0 ? roi->y0 - 1 : 0;
    int below = roi->y0 + 1 < H ? roi->y0 + 1 : H - 1;
    memcpy(r0, mask + above * W + xbase, width);
    memcpy(r1, mask + roi->y0 * W + xbase, width);
    memcpy(r2, mask + below * W + xbase, width);
    for (int y = roi->y0; y < roi->y1; y++) {
        uint8_t *dst = mask + y * W;
        int left = (r0[0] != 0) + (r1[0] != 0) + (r2[0] != 0);
        int center_x = roi->x0 - xbase;
        int center = (r0[center_x] != 0) + (r1[center_x] != 0) + (r2[center_x] != 0);
        int right_x = (roi->x0 + 1 < W ? roi->x0 + 1 : W - 1) - xbase;
        int right = (r0[right_x] != 0) + (r1[right_x] != 0) + (r2[right_x] != 0);
        int ones = left + center + right;
        for (int x = roi->x0; x < roi->x1; x++) {
            dst[x] = ones >= 5 ? 255 : 0;
            if (x + 1 < roi->x1) {
                int incoming_x = (x + 2 < W ? x + 2 : W - 1) - xbase;
                int incoming = (r0[incoming_x] != 0) + (r1[incoming_x] != 0)
                             + (r2[incoming_x] != 0);
                ones += incoming - left;
                left = center; center = right; right = incoming;
            }
        }
        if (y + 1 < roi->y1) {
            /* 轮换指针而非覆盖原图行；新读取的y+2行尚未写入输出。
             * 到底边时复制最后一行，使上下左右均采用边界复制语义。 */
            uint8_t *reuse = r0; r0 = r1; r1 = r2; r2 = reuse;
            int next = y + 2 < H ? y + 2 : H - 1;
            memcpy(r2, mask + next * W + xbase, width);
        }
    }
}
static void median3(uint8_t *mask, uint8_t rows[3][W])
{
    median3_region(mask, rows, &full_region);
}

/* 按像素对共享色度计算RGB，BT.601全幅，与JPEG打包一致。
 * 2*G-R-B >= 2*margin 等价 G-(R+B)/2 >= margin，无浮点/除法。
 * 白/灰通道近等时差值接近0被去除；负差值也不能成为绿色。 */
static void build_green_mask(const uint8_t *frame, const detection_region *roi)
{
    int x0 = roi->x0 > 0 ? roi->x0 - 1 : 0;
    int x1 = roi->x1 < W ? roi->x1 + 1 : W;
    int y0 = roi->y0 > 0 ? roi->y0 - 1 : 0;
    int y1 = roi->y1 < H ? roi->y1 + 1 : H;
    for (int row = y0; row < y1; ++row) for (int x = x0 & ~1; x < x1; x += 2) {
        int i = row * W + x; /* YUYV色度必须按偶数列配对，ROI可从奇数列开始。 */
        const uint8_t *p = frame + i * 2;
        int cb = (int)p[DART_DET_C_UV_SWAP ? 3 : 1] - 128;
        int cr = (int)p[DART_DET_C_UV_SWAP ? 1 : 3] - 128;
        int dr = (91881 * cr + 32768) >> 16;
        int dg = (22554 * cb + 46802 * cr + 32768) >> 16;
        int db = (116130 * cb + 32768) >> 16;
        for (int k = 0; k < 2; ++k) {
            if (x + k < x0 || x + k >= x1) continue;
            int y = p[k * 2], r = y + dr, g = y - dg, b = y + db;
            r = r < 0 ? 0 : (r > 255 ? 255 : r);
            g = g < 0 ? 0 : (g > 255 ? 255 : g);
            b = b < 0 ? 0 : (b > 255 ? 255 : b);
            green_mask[i + k] = (g >= DART_DET_G_MIN &&
                2 * g - r - b >= 2 * DART_DET_GREEN_EXCESS_MIN) ? 255 : 0;
        }
    }
}

static uint32_t flood_component(int sx, int sy, int *bx, int *by, int *bw, int *bh,
                                int64_t *sum_x, int64_t *sum_y, const detection_region *roi)
{
    int min_x = sx, max_x = sx, min_y = sy, max_y = sy;
    uint32_t count = 0;
    int64_t ax = 0, ay = 0;
    unsigned top = 0;
    int pending_min = DART_DET_PIXELS;
    int first = sy * W + sx;
    green_mask[first] = 0;
    flood_stack[top++] = first;
    while (top || pending_min < DART_DET_PIXELS) {
        int idx;
        if (top) idx = flood_stack[--top];
        else {
            idx = pending_min;
            deferred[idx >> 3] &= (uint8_t)~(1u << (idx & 7));
            do { pending_min++; } while (pending_min < DART_DET_PIXELS &&
                !(deferred[pending_min >> 3] & (1u << (pending_min & 7))));
        }
        int x = idx % W, y = idx / W;
        count++; ax += x; ay += y;
        if (x < min_x) min_x = x; if (x > max_x) max_x = x;
        if (y < min_y) min_y = y; if (y > max_y) max_y = y;
        for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
            int nx = x + dx, ny = y + dy;
            if (nx < roi->x0 || nx >= roi->x1 || ny < roi->y0 || ny >= roi->y1) continue;
            int next = ny * W + nx;
            if (!green_mask[next]) continue;
            green_mask[next] = 0; /* 入栈即标记，避免重复入栈。 */
            if (top < sizeof(flood_stack) / sizeof(flood_stack[0])) flood_stack[top++] = next;
            else {
                deferred[next >> 3] |= (uint8_t)(1u << (next & 7));
                if (next < pending_min) pending_min = next;
            }
        }
    }
    *bx = min_x; *by = min_y; *bw = max_x - min_x + 1; *bh = max_y - min_y + 1;
    *sum_x = ax; *sum_y = ay;
    return count;
}

typedef struct {
    dart_detection_timing *timing;
    uint32_t (*now_us)(void);
    uint32_t last;
    unsigned stage;
} detection_clock;
static void timing_next(detection_clock *clock, unsigned stage)
{
    if (!clock->timing) return;
    uint32_t now = clock->now_us();
    clock->timing->stage_us[clock->stage] += now - clock->last;
    clock->last = now; clock->stage = stage;
}
static void detect_region(const uint8_t *frame, dart_detection *out,
                           const detection_region *roi, detection_clock *clock)
{
    memset(out, 0, sizeof(*out));
    timing_next(clock, DET_TIME_MASKS);
    build_green_mask(frame, roi);
    timing_next(clock, DET_TIME_MEDIAN_GREEN);
    if (region_is_full(roi)) median3(green_mask, med_rows_a);
    else median3_region(green_mask, med_rows_a, roi);
    timing_next(clock, DET_TIME_SCAN);
    uint32_t best_area = 0;
    for (int y = roi->y0; y < roi->y1; ++y) for (int x = roi->x0; x < roi->x1; ++x) {
        if (!green_mask[y * W + x]) continue;
        timing_next(clock, DET_TIME_FLOOD);
        int bx, by, bw, bh; int64_t sx, sy;
        uint32_t pixels = flood_component(x, y, &bx, &by, &bw, &bh, &sx, &sy, roi);
        timing_next(clock, DET_TIME_AREA);
        if (pixels < DART_DET_AREA_MIN || pixels > DART_DET_AREA_MAX) goto candidate_done;
        timing_next(clock, DET_TIME_SCORE);
        if (pixels <= best_area) goto candidate_done;
        best_area = pixels;
        out->found = 1;
        out->center_x_q16 = (int32_t)((sx << 16) / pixels);
        out->center_y_q16 = (int32_t)((sy << 16) / pixels);
        out->box_x = bx; out->box_y = by; out->box_w = bw; out->box_h = bh;
        out->area_q8 = (int32_t)(pixels * 256u);
        out->score_x10000 = (int32_t)(pixels * 10000u);
        /* 以绿色质心为圆心覆盖包围盒，不再执行轮廓追踪。 */
        int cx = out->center_x_q16 >> 16, cy = out->center_y_q16 >> 16;
        int dx = cx - bx > bx + bw - 1 - cx ? cx - bx : bx + bw - 1 - cx;
        int dy = cy - by > by + bh - 1 - cy ? cy - by : by + bh - 1 - cy;
        out->radius = (int)sqrtf((float)(dx * dx + dy * dy)) + 4;
candidate_done:
        timing_next(clock, DET_TIME_SCAN);
    }
}
int green_detect(const uint8_t *frame, dart_detection *out)
{
    return green_detect_profiled(frame, out, NULL, NULL);
}
int green_detect_profiled(const uint8_t *frame, dart_detection *out,
                          dart_detection_timing *timing, uint32_t (*now_us)(void))
{
    if (timing) memset(timing, 0, sizeof(*timing));
    if (!frame || !out || (timing && !now_us)) return -1;
    detection_clock clock = {timing, now_us, timing ? now_us() : 0, DET_TIME_INIT};
    detection_region roi = next_region;
    int local = !region_is_full(&roi);
    memset(deferred, 0, sizeof(deferred));
    det_eval_frames++;
    if (timing) {
        timing->search_x = (uint16_t)roi.x0; timing->search_y = (uint16_t)roi.y0;
        timing->search_w = (uint16_t)(roi.x1 - roi.x0); timing->search_h = (uint16_t)(roi.y1 - roi.y0);
        timing->search_mode = local ? DET_SEARCH_ROI : DET_SEARCH_FULL;
    }
    detect_region(frame, out, &roi, &clock);
    if (local && (!out->found || box_touches_inner_edge(out, &roi))) {
        /* 目标丢失或局部结果可能被裁断，当帧重新完整搜索；仅计一次帧数。 */
        if (timing) timing->search_mode = DET_SEARCH_FALLBACK;
        detect_region(frame, out, &full_region, &clock);
    }
    next_region = out->found ? expand_box(out) : full_region;
    if (out->found) det_found_frames++;
    timing_next(&clock, DET_TIME_SCAN);
    return 0;
}
