/* 合成YUYV验证计时开关不改变识别结果、拒绝路径/多候选累加及计时器回绕。
 * 假时钟只验证区间归属，不能代表H750实际性能。 */
#include "green_detector.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t frame[DART_DET_PIXELS * 2];
static uint32_t ticks, calls;

static uint32_t fake_micros(void)
{
    uint32_t value = ticks;
    ticks += 7;
    calls++;
    return value;
}

static void background(void)
{
    for (unsigned i = 0; i < sizeof(frame); i += 4) {
        frame[i] = frame[i + 2] = 0;
        frame[i + 1] = frame[i + 3] = 128;
    }
}

static void disk(int cx, int cy, int radius, int green)
{
    for (int y = 0; y < DART_DET_HEIGHT; ++y)
        for (int x = 0; x < DART_DET_WIDTH; x += 2) {
            int dx = x - cx, dy = y - cy;
            if (dx * dx + dy * dy > radius * radius) continue;
            uint8_t *p = frame + (y * DART_DET_WIDTH + x) * 2;
            p[0] = p[2] = green ? 150 : 255;
            p[1] = green ? 44 : 128;
            p[3] = green ? 21 : 128;
        }
}

static dart_detection_timing check(int found)
{
    dart_detection a, b;
    dart_detection_timing timing;
    memset(&timing, 0xa5, sizeof(timing));
    ticks = UINT32_MAX - 10; calls = 0;
    assert(green_detect_profiled(frame, &a, &timing, fake_micros) == 0);
    assert(a.found == found);
    uint32_t sum = 0;
    for (unsigned i = 0; i < DET_TIME_COUNT; ++i) sum += timing.stage_us[i];
    assert(sum == (calls - 1) * 7); /* 每个区间恰好统计一次，含回绕。 */
    assert(timing.stage_us[DET_TIME_INIT] == 7);
    assert(timing.stage_us[DET_TIME_MASKS] == 7);
    assert(timing.stage_us[DET_TIME_MEDIAN_GREEN] == 7);
    assert(timing.stage_us[DET_TIME_MEDIAN_CORE] == 7);
    assert(timing.stage_us[DET_TIME_SCAN] > 0);
    uint32_t previous_calls = calls;
    assert(green_detect(frame, &b) == 0);
    assert(!memcmp(&a, &b, sizeof(a)) && calls == previous_calls);
    return timing;
}

int main(void)
{
    background();
    dart_detection_timing timing = check(0);
    for (unsigned i = DET_TIME_FLOOD; i < DET_TIME_COUNT; ++i)
        assert(timing.stage_us[i] == 0);
    disk(80, 80, 3, 0); /* 小面积候选在形状筛选退出。 */
    timing = check(0);
    assert(timing.stage_us[DET_TIME_FLOOD] == 7);
    assert(timing.stage_us[DET_TIME_CONTOUR] == 7);
    assert(timing.stage_us[DET_TIME_SHAPE] == 7);
    assert(timing.stage_us[DET_TIME_SUPPORT] == 0 && timing.stage_us[DET_TIME_SCORE] == 0);

    background(); disk(80, 80, 12, 0); /* 无绿晕：邻域验证退出。 */
    timing = check(0);
    assert(timing.stage_us[DET_TIME_SUPPORT] == 7 && timing.stage_us[DET_TIME_SCORE] == 0);

    background(); disk(80, 80, 20, 1); disk(80, 80, 12, 0);
    timing = check(1);
    for (unsigned i = DET_TIME_FLOOD; i < DET_TIME_COUNT; ++i)
        assert(timing.stage_us[i] == 7);
    disk(220, 160, 20, 1); disk(220, 160, 12, 0);
    timing = check(1); /* 第二个同分候选也要计入评分耗时。 */
    for (unsigned i = DET_TIME_FLOOD; i < DET_TIME_COUNT; ++i)
        assert(timing.stage_us[i] == 14);

    dart_detection result;
    assert(green_detect_profiled(NULL, &result, &timing, fake_micros) == -1);
    for (unsigned i = 0; i < DET_TIME_COUNT; ++i) assert(timing.stage_us[i] == 0);
    assert(green_detect_profiled(frame, NULL, &timing, fake_micros) == -1);
    assert(green_detect_profiled(frame, &result, &timing, NULL) == -1);
    puts("PASS: detector timing/reset/wrap/rejection/multiple candidates/result equivalence");
    return 0;
}
