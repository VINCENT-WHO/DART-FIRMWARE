/* 生产C源直接编入：验证ROI像素等价、范围限制、1.5x取整/裁边及跨帧状态。
 * 合成输入，不使用旧曝光录像，不把主机检查当板端加速测量。 */
#include "../h750/green_detector.c"
#include <assert.h>
#include <stdio.h>

static uint8_t frame[DART_DET_PIXELS * 2], frame_copy[sizeof(frame)];
static uint8_t original[DART_DET_PIXELS], expected[DART_DET_PIXELS];
static uint8_t guarded[DART_DET_PIXELS + 64], row_guard[3 * W + 64];
static uint32_t random_state = 7163, ticks, tick_calls;
static uint32_t random32(void)
{
    random_state ^= random_state << 13; random_state ^= random_state >> 17;
    random_state ^= random_state << 5; return random_state;
}
static uint32_t now_us(void) { uint32_t n = ticks; ticks += 7; tick_calls++; return n; }
static int inside(const detection_region *r, int x, int y)
{
    return x >= r->x0 && x < r->x1 && y >= r->y0 && y < r->y1;
}
static void verify_pixels(void)
{
    detection_region regions[] = {{0,0,W,H}, {77,77,93,93}, {1,1,2,2},
        {0,0,1,1}, {W-1,H-1,W,H}, {13,0,52,11}, {0,31,17,60},
        {W-19,41,W,42}, {81,H-1,82,H}, {81,50,82,80}, {20,51,52,52}};
    for (unsigned n = 0; n < 70; ++n) {
        detection_region r;
        if (n < sizeof(regions)/sizeof(regions[0])) r = regions[n];
        else {
            r.x0 = (int)(random32() % W); r.y0 = (int)(random32() % H);
            r.x1 = r.x0 + 1 + (int)(random32() % (W - r.x0));
            r.y1 = r.y0 + 1 + (int)(random32() % (H - r.y0));
        }
        for (int i = 0; i < DART_DET_PIXELS; ++i) original[i] = random32() & 1 ? 255 : 0;
        memset(guarded, 0xa5, sizeof(guarded)); memset(row_guard, 0x5a, sizeof(row_guard));
        memcpy(guarded + 32, original, sizeof(original));
        for (int y = r.y0; y < r.y1; ++y) for (int x = r.x0; x < r.x1; ++x) {
            int ones = 0;
            for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
                int qx = x + dx, qy = y + dy;
                if (qx < 0) qx = 0; if (qx >= W) qx = W - 1;
                if (qy < 0) qy = 0; if (qy >= H) qy = H - 1;
                ones += original[qy * W + qx] != 0;
            }
            expected[y * W + x] = ones >= 5 ? 255 : 0;
        }
        median3_region(guarded + 32, (uint8_t (*)[W])(row_guard + 32), &r);
        for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) {
            int idx = y * W + x;
            assert(guarded[32 + idx] == (inside(&r,x,y) ? expected[idx] : original[idx]));
        }
        for (int i = 0; i < 32; ++i) {
            assert(guarded[i] == 0xa5 && guarded[sizeof(guarded)-1-i] == 0xa5);
            assert(row_guard[i] == 0x5a && row_guard[sizeof(row_guard)-1-i] == 0x5a);
        }
        /* 奇数列ROI必须共享正确的YUYV色度，并且只写ROI+halo。 */
        for (unsigned i = 0; i < sizeof(frame); ++i) frame[i] = (uint8_t)random32();
        build_green_mask(frame, &full_region); memcpy(expected, green_mask, sizeof(expected));
        memset(green_mask, 0xa5, sizeof(green_mask)); build_green_mask(frame, &r);
        detection_region halo = {r.x0 > 0 ? r.x0-1 : 0, r.y0 > 0 ? r.y0-1 : 0,
            r.x1 < W ? r.x1+1 : W, r.y1 < H ? r.y1+1 : H};
        for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x)
            assert(green_mask[y*W+x] == (inside(&halo,x,y) ? expected[y*W+x] : 0xa5));
    }
}
static void blank(void)
{
    for (unsigned i = 0; i < sizeof(frame); i += 4) {
        frame[i] = frame[i+2] = 0; frame[i+1] = frame[i+3] = 128;
    }
}
static void green(int x, int y, int w, int h)
{
    assert(!(x & 1) && !(w & 1));
    for (int yy = y; yy < y+h; ++yy) for (int xx = x; xx < x+w; xx += 2) {
        uint8_t *p = frame + (yy*W+xx)*2; p[0] = p[2] = 150; p[1] = 44; p[3] = 21;
    }
}
static dart_detection run(unsigned mode, int found, dart_detection_timing *timing)
{
    dart_detection d; memcpy(frame_copy, frame, sizeof(frame));
    ticks = UINT32_MAX - 10; tick_calls = 0;
    uint32_t before = det_eval_frames, hits = det_found_frames;
    assert(!green_detect_profiled(frame, &d, timing, now_us));
    assert(d.found == found && timing->search_mode == mode);
    assert(det_eval_frames == before + 1 && det_found_frames == hits + (unsigned)found);
    assert(!memcmp(frame, frame_copy, sizeof(frame)));
    uint32_t sum = 0; for (unsigned i = 0; i < DET_TIME_COUNT; ++i) sum += timing->stage_us[i];
    assert(sum == (tick_calls - 1) * 7);
    unsigned passes = mode == DET_SEARCH_FALLBACK ? 2 : 1;
    assert(timing->stage_us[DET_TIME_MASKS] == passes * 7);
    assert(timing->stage_us[DET_TIME_MEDIAN_GREEN] == passes * 7);
    return d;
}
static void verify_sequence(void)
{
    dart_detection_timing t; dart_detection d;
    det_reset_tracking(); det_reset_stats(); blank(); green(80,80,10,10);
    d = run(DET_SEARCH_FULL,1,&t); assert(d.box_x == 80 && d.box_w == 10);
    assert(t.search_x == 0 && t.search_w == W && t.search_h == H);
    blank();green(82,82,10,10);green(200,120,20,20);
    d = run(DET_SEARCH_ROI,1,&t);
    assert(t.search_x == 77 && t.search_y == 77 && t.search_w == 16 && t.search_h == 16);
    assert(d.box_x == 82 && d.box_y == 82 && d.area_q8 == 96*256); /* 框外更大目标不抢占 */
    blank();green(240,180,10,10); /* 快速移动到ROI外：当帧重新找到。 */
    d = run(DET_SEARCH_FALLBACK,1,&t);assert(d.box_x == 240 && d.box_y == 180);
    blank();d = run(DET_SEARCH_FALLBACK,0,&t);assert(!d.box_w && !d.center_x_q16);
    blank();run(DET_SEARCH_FULL,0,&t);
    green(80,80,10,10);run(DET_SEARCH_FULL,1,&t);
    blank();green(88,80,10,10); /* 局部可通过面积但被右边缘截断，必须取完整结果。 */
    d = run(DET_SEARCH_FALLBACK,1,&t);assert(d.box_x == 88 && d.box_w == 10 && d.area_q8 == 96*256);
    det_reset_tracking();blank();green(0,0,10,10);run(DET_SEARCH_FULL,1,&t);
    d = run(DET_SEARCH_ROI,1,&t);assert(t.search_x == 0 && t.search_y == 0 && t.search_w == 13 && t.search_h == 13);
    assert(d.box_x == 0 && d.box_y == 0); /* 物理图像边缘无需回退 */
    det_reset_tracking();blank();green(W-10,H-10,10,10);run(DET_SEARCH_FULL,1,&t);
    d = run(DET_SEARCH_ROI,1,&t);assert(t.search_x == W-13 && t.search_y == H-13 && t.search_w == 13 && t.search_h == 13);
    det_reset_tracking();blank();green(40,70,10,10);run(DET_SEARCH_FULL,1,&t);
    detection_region saved = next_region;
    assert(green_detect_profiled(NULL,&d,&t,now_us) == -1 && !memcmp(&saved,&next_region,sizeof(saved)));
    det_reset_tracking();blank();green(200,100,10,10);d = run(DET_SEARCH_FULL,1,&t);assert(d.box_x == 200);
}
static void verify_geometry(void)
{
    for (int w = 1; w <= W; ++w) for (int h = 1; h <= H; ++h) {
        dart_detection d = {0};d.box_x=(W-w)/2;d.box_y=(H-h)/2;d.box_w=w;d.box_h=h;
        detection_region r=expand_box(&d);
        assert(r.x0>=0 && r.y0>=0 && r.x1<=W && r.y1<=H);
        assert(r.x0<=d.box_x && r.y0<=d.box_y && r.x1>=d.box_x+w && r.y1>=d.box_y+h);
        if (r.x0>0 && r.x1<W) assert(r.x0+r.x1==2*d.box_x+w && 2*(r.x1-r.x0)>=3*w && 2*(r.x1-r.x0)<=3*w+3);
        if (r.y0>0 && r.y1<H) assert(r.y0+r.y1==2*d.box_y+h && 2*(r.y1-r.y0)>=3*h && 2*(r.y1-r.y0)<=3*h+3);
    }
}
int main(void)
{
    verify_geometry(); verify_pixels(); verify_sequence();
    puts("PASS: ROI 1.5x/clipping/halo median/odd YUYV bounds/motion/distractors/loss/reacquire/reset/timing");
    return 0;
}
