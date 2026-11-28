#ifndef DART_IMU_SESSION_H
#define DART_IMU_SESSION_H
#include "protocol.h"
#include <string.h>

/* H750本地接收窗口独立于F103的持续采样/PWM模式。 */
typedef struct { volatile unsigned accepting, started, released; } dart_imu_session;
static inline void dart_imu_session_begin(dart_imu_session *s)
{
    s->started = s->released = 0; s->accepting = 1;
}
static inline void dart_imu_session_end(dart_imu_session *s)
{
    s->accepting = 0; s->released = 0;
}
static inline int dart_imu_session_accepts(const dart_imu_session *s, const dart_packet *p)
{
    return s->accepting && p->type == DART_IMU && p->length == sizeof(dart_imu);
}
static inline void dart_imu_session_reply(dart_imu_session *s, const dart_packet *p)
{
    if (p->type != DART_REPLY) return;
    if ((p->length == 11 && !memcmp(p->data, "IMU STARTED", 11)) ||
        (p->length == 12 && !memcmp(p->data, "IMU WATCHING", 12))) s->started = 1;
    if ((p->length == 11 && !memcmp(p->data, "IMU STOPPED", 11)) ||
        (p->length == 14 && !memcmp(p->data, "IMU CONTINUOUS", 14))) s->released = 1;
}
#endif
