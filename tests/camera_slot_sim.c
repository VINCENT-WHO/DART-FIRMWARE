/* 帧槽策略仿真：相机每 20ms 出一帧，处理端单线程串行处理。
 * 对比两种策略下，"处理端取到帧时，这帧是多久前拍的"（即画面陈旧度）。
 *   旧策略：取最旧的就绪帧；槽满时丢弃新到的帧、保留旧帧（2.1.5 及更早）。
 *   新策略：取最新的就绪帧；槽满时淘汰最旧的就绪帧、保留新帧。
 * 同时检查不变量：任何时刻至多一个接收中的槽；接收目标绝不是处理中的槽。 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "camera_slots.h"

#define FRAME_US 20000u     /* 相机帧周期，约 50fps */
#define PROCESS_US 128540u   /* REC00021：处理开始至保存返回 */

static uint32_t rng_state = 12345u;
static uint32_t rnd(void) { rng_state = rng_state * 1664525u + 1013904223u; return rng_state >> 8; }

/* 旧策略的中断侧：有空闲槽就交付，否则丢掉新帧并原地重拍 */
static int old_complete(camera_slots *s, uint32_t now, uint32_t size, unsigned *drops)
{
    int next = -1;
    for (unsigned i = 0; i < CAM_SLOT_COUNT; i++) if (s->state[i] == CAM_SLOT_FREE) { next = (int)i; break; }
    if (next < 0) { (*drops)++; return 0; }
    s->sizes[s->current] = size; s->times[s->current] = now; s->state[s->current] = CAM_SLOT_READY;
    s->current = (unsigned)next; s->state[next] = CAM_SLOT_FILLING;
    return 1;
}
/* 旧策略的取帧：最旧的就绪帧 */
static int old_take(camera_slots *s, uint32_t *size, uint32_t *time)
{
    int best = -1;
    for (unsigned i = 0; i < CAM_SLOT_COUNT; i++)
        if (s->state[i] == CAM_SLOT_READY &&
            (best < 0 || (int32_t)(s->times[i] - s->times[best]) < 0)) best = (int)i;
    if (best >= 0) { *size = s->sizes[best]; *time = s->times[best]; s->state[best] = CAM_SLOT_BUSY; }
    return best;
}

static int check_invariants(const camera_slots *s, int busy_expected)
{
    unsigned filling = 0, busy = 0;
    for (unsigned i = 0; i < CAM_SLOT_COUNT; i++) {
        if (s->state[i] == CAM_SLOT_FILLING) filling++;
        if (s->state[i] == CAM_SLOT_BUSY) { busy++; if ((int)i != busy_expected) return 0; }
    }
    if ((busy_expected < 0) != (busy == 0)) return 0;   /* 处理中的槽至多一个，且正是登记的那个 */
    /* 接收目标必须是接收中的槽，且正好一个 */
    return filling == 1 && s->state[s->current] == CAM_SLOT_FILLING;
}

static void run(int policy_new, uint32_t phase, uint32_t jitter, uint32_t *age_sum, uint32_t *age_max, uint32_t *age_min, unsigned *count)
{
    camera_slots s; cam_slots_reset(&s);
    unsigned drops = 0, taken = 0; int busy = -1;
    uint32_t next_frame = phase, busy_until = 0, now = 0;
    uint32_t sum = 0, mx = 0, mn = 0xFFFFFFFFu;
    uint64_t age_at_result_sum = 0; uint32_t age_at_result_max = 0;
    while (taken < 300) {
        int r = -1;
        if (busy >= 0 && busy_until <= next_frame) {
            /* 处理先结束：释放槽，处理端空闲 */
            now = busy_until; cam_release(&s, (unsigned)busy); busy = -1;
            if (!check_invariants(&s, busy)) { fprintf(stderr, "INVARIANT BROKEN (release) policy=%d\n", policy_new); exit(1); }
        } else {
            /* 帧先完成 */
            now = next_frame; next_frame += FRAME_US + rnd() % (jitter + 1);
            r = policy_new ? cam_frame_complete(&s, now, 153600u)
                           : old_complete(&s, now, 153600u, &drops);
            if (policy_new && r != CAM_FRAME_PUBLISHED) drops++;
            if (!check_invariants(&s, busy)) { fprintf(stderr, "INVARIANT BROKEN (frame) policy=%d r=%d\n", policy_new, r); exit(1); }
        }
        /* 处理端空闲时立即取帧：取到就开始处理，取不到就等下一帧（无等待循环） */
        if (busy < 0) {
            uint32_t size, time;
            int slot = policy_new ? cam_take_newest(&s, &size, &time) : old_take(&s, &size, &time);
            if (slot >= 0) {
                uint32_t age = now - time;
                if (taken > 20) { /* 跳过启动过渡 */
                    sum += age; mx = age > mx ? age : mx; mn = age < mn ? age : mn;
                    uint32_t at_result = age + PROCESS_US;
                    age_at_result_sum += at_result; if (at_result > age_at_result_max) age_at_result_max = at_result;
                    (*count)++;
                }
                busy = slot; busy_until = now + PROCESS_US;
                if (!check_invariants(&s, busy)) { fprintf(stderr, "INVARIANT BROKEN (take) policy=%d\n", policy_new); exit(1); }
                taken++;
            }
        }
    }
    *age_sum = sum; *age_max = mx; *age_min = mn;
    if (count) {
        double mean_result_ms = (double)age_at_result_sum / *count / 1000.0;
        if (jitter == 999) printf("    输出坐标时刻的画面陈旧度 平均 %.1f ms，最大 %.1f ms\n", mean_result_ms, age_at_result_max / 1000.0);
    }
}

int main(void)
{
    const uint32_t frames_per_run = 280;
    for (int policy = 0; policy <= 1; policy++) {
        uint64_t sum = 0; uint32_t mx = 0, mn = 0xFFFFFFFFu; unsigned count = 0;
        rng_state = 777u;
        for (uint32_t phase = 0; phase < FRAME_US; phase += 137u) {      /* 覆盖各种相位对齐 */
            uint32_t a, b, c; unsigned n = 0;
            run(policy, phase, 300u, &a, &b, &c, &n);
            sum += a; mx = b > mx ? b : mx; mn = c < mn ? c : mn; count += n;
        }
        printf("%s策略：取帧时画面已陈旧 平均 %.1f ms，范围 %.1f ~ %.1f ms（%u 帧）",
               policy ? "新" : "旧", (double)sum / count / 1000.0, mn / 1000.0, mx / 1000.0, count);
        if (mn == 0xFFFFFFFFu) { printf("（无有效样本）\n"); continue; }
        uint32_t a, b, c; unsigned n = 0; rng_state = 4242u;
        run(policy, FRAME_US / 3, 999u, &a, &b, &c, &n); (void)frames_per_run;
    }
    return 0;
}
