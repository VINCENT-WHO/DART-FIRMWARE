/* 相机帧槽状态机：只做槽位登记，不碰任何硬件，可在电脑上仿真验证。
 *
 * 目的：让处理端永远拿到"最新拍完的那一帧"，而不是排队最久的那一帧。
 * 处理一帧约 128ms，相机每 20ms 出一帧，中间的帧必然大部分用不上；
 * 用不上的应该是旧帧，而不是新帧。
 *
 * 槽状态：
 *   FREE     空闲，可以接收下一帧
 *   FILLING  DMA 正在往里写（同一时刻只有 current 一个）
 *   READY    一帧完整，等待处理端取走
 *   BUSY     处理端正在读，DMA 与中断都不得改写或回收
 *
 * 共 3 个槽。处理端单线程，所以 BUSY 至多 1 个，FREE/READY/FILLING 共占其余 2 个。 */
#ifndef DART_CAMERA_SLOTS_H
#define DART_CAMERA_SLOTS_H
#include <stdint.h>

#define CAM_SLOT_COUNT 3u
#define CAM_SLOT_FREE 0u
#define CAM_SLOT_FILLING 1u
#define CAM_SLOT_READY 2u
#define CAM_SLOT_BUSY 3u

/* 一帧接收完整后的处理结果 */
#define CAM_FRAME_PUBLISHED 1   /* 交付到空闲槽，没有帧被淘汰 */
#define CAM_FRAME_SUPERSEDED 2   /* 交付到就绪槽，被淘汰的是更旧的那帧 */
#define CAM_FRAME_DROPPED 0     /* 没有可用的槽，这一帧丢弃 */

typedef struct {
    volatile unsigned state[CAM_SLOT_COUNT];
    unsigned current;                       /* 正在接收的槽 */
    volatile uint32_t sizes[CAM_SLOT_COUNT]; /* 字节数 */
    volatile uint32_t times[CAM_SLOT_COUNT]; /* 帧接收完成的时刻，微秒 */
} camera_slots;

/* 写 times/sizes 必须先于写 state 生效（处理端先看 state 再读 time）。
 * 主机上只需要阻止编译器重排。 */
#if defined(__GNUC__) && (defined(__arm__) || defined(__thumb__) || defined(__aarch64__))
#define CAM_SLOT_BARRIER() __asm__ __volatile__("dmb 0xF" ::: "memory")
#else
#define CAM_SLOT_BARRIER() __asm__ __volatile__("" ::: "memory")
#endif

/* 一帧接收完成（由帧中断调用）。
 *
 * 优先找空闲槽接收下一帧。没有空闲槽时，淘汰"最旧的就绪帧"，把它的槽
 * 拿去接收下一帧——旧帧已被更新的帧取代，留着只会让处理端等到过期画面。
 * BUSY 槽是处理端正在读的，任何情况下不动。 */
static inline int cam_frame_complete(camera_slots *s, uint32_t now, uint32_t size)
{
    if (s->state[s->current] != CAM_SLOT_FILLING) return CAM_FRAME_DROPPED;
    int next = -1, result = CAM_FRAME_PUBLISHED;
    for (unsigned i = 0; i < CAM_SLOT_COUNT; i++)
        if (s->state[i] == CAM_SLOT_FREE) { next = (int)i; break; }
    if (next < 0) {
        /* int32_t 有符号差值比较：times 是 32 位微秒计数，约 71 分钟回绕一次。 */
        for (unsigned i = 0; i < CAM_SLOT_COUNT; i++)
            if (s->state[i] == CAM_SLOT_READY &&
                (next < 0 || (int32_t)(s->times[i] - s->times[next]) < 0)) next = (int)i;
        if (next < 0) return CAM_FRAME_DROPPED;
        result = CAM_FRAME_SUPERSEDED;
    }
    s->sizes[s->current] = size;
    s->times[s->current] = now;
    CAM_SLOT_BARRIER();
    s->state[s->current] = CAM_SLOT_READY;
    s->current = (unsigned)next;
    s->state[next] = CAM_SLOT_FILLING;
    return result;
}

/* 处理端取帧：取就绪帧里最新的一个并置为 BUSY。
 * 调用方负责关中断，保证与 cam_frame_complete 互斥。返回槽号，没有就绪帧返回 -1。 */
static inline int cam_take_newest(camera_slots *s, uint32_t *size, uint32_t *time)
{
    int best = -1;
    for (unsigned i = 0; i < CAM_SLOT_COUNT; i++)
        if (s->state[i] == CAM_SLOT_READY &&
            (best < 0 || (int32_t)(s->times[i] - s->times[best]) > 0)) best = (int)i;
    if (best >= 0) {
        *size = s->sizes[best]; *time = s->times[best];
        CAM_SLOT_BARRIER();
        s->state[best] = CAM_SLOT_BUSY;
    }
    return best;
}

static inline void cam_release(camera_slots *s, unsigned slot)
{
    if (slot < CAM_SLOT_COUNT) { CAM_SLOT_BARRIER(); s->state[slot] = CAM_SLOT_FREE; }
}

static inline void cam_slots_reset(camera_slots *s)
{
    for (unsigned i = 0; i < CAM_SLOT_COUNT; i++) { s->state[i] = CAM_SLOT_FREE; s->sizes[i] = 0; s->times[i] = 0; }
    s->current = 0;
    s->state[0] = CAM_SLOT_FILLING;
}

#endif
