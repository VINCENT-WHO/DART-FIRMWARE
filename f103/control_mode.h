#ifndef DART_F103_CONTROL_MODE_H
#define DART_F103_CONTROL_MODE_H
#include <stdint.h>
#include <string.h>

typedef enum { F103_MODE_NORMAL = 0, F103_MODE_DEBUG = 1 } f103_mode;
/* 模式由固件构建决定，不由电脑或H750命令切换；默认正常运行。 */
#ifndef DART_F103_DEBUG_MODE
#define DART_F103_DEBUG_MODE 0
#endif
#if DART_F103_DEBUG_MODE == 1
#define F103_BOOT_MODE F103_MODE_DEBUG
#define F103_FIRMWARE_VERSION "2.1.0-debug"
#elif DART_F103_DEBUG_MODE == 0
#define F103_BOOT_MODE F103_MODE_NORMAL
#define F103_FIRMWARE_VERSION "2.1.0-normal"
#else
#error "DART_F103_DEBUG_MODE must be 0 or 1"
#endif
enum { F103_PWM_EXPIRED = 1u, F103_IMU_EXPIRED = 2u };
typedef struct {
    f103_mode mode;
    unsigned pwm_requested, imu_requested, watching;
    uint32_t pulses[4], last_manual_ms, imu_started_ms;
} f103_control;

static inline void f103_control_init(f103_control *c, f103_mode mode)
{
    memset(c, 0, sizeof(*c));
    c->mode = mode;
    c->pwm_requested = c->imu_requested = c->watching = mode == F103_MODE_NORMAL;
    for (unsigned i = 0; i < 4; ++i) c->pulses[i] = 1500;
}
static inline int f103_control_pwm(f103_control *c, unsigned on, uint32_t now)
{
    if (c->mode != F103_MODE_DEBUG) return 0;
    c->pwm_requested = on;
    c->last_manual_ms = now;
    return 1;
}
static inline int f103_control_pulse(f103_control *c, unsigned channel, unsigned us, uint32_t now)
{
    if (c->mode != F103_MODE_DEBUG || channel < 1 || channel > 4 || us < 1000 || us > 2000) return 0;
    c->pulses[channel - 1] = us;
    c->last_manual_ms = now;
    return 1;
}
/* START/WATCH在正常模式仅确认持续流；STOP只结束H750的记录窗口。
 * 调试模式可单独启停IMU，也允许与PWM并行。NULL表示非IMU命令。 */
static inline const char *f103_control_imu(f103_control *c, const char *text,
                                         unsigned ready, uint32_t now)
{
    if (!strcmp(text, "IMU STOP")) {
        if (c->mode == F103_MODE_NORMAL) return ready ? "IMU CONTINUOUS" : "IMU STOPPED";
        c->imu_requested = c->watching = 0;
        return "IMU STOPPED";
    }
    unsigned watch = !strcmp(text, "IMU WATCH");
    if (!watch && strcmp(text, "IMU START")) return NULL;
    if (!ready) return "IMU ERROR ID/CONFIG/CLOCK";
    if (c->mode == F103_MODE_DEBUG) {
        c->imu_requested = 1; c->watching = watch; c->imu_started_ms = now;
    }
    return watch ? "IMU WATCHING" : "IMU STARTED";
}
static inline unsigned f103_control_poll(f103_control *c, uint32_t now)
{
    unsigned expired = 0;
    if (c->mode != F103_MODE_DEBUG) return 0;
    if (c->pwm_requested && (uint32_t)(now - c->last_manual_ms) >= 3000u) {
        c->pwm_requested = 0; expired |= F103_PWM_EXPIRED;
    }
    if (c->imu_requested && !c->watching && (uint32_t)(now - c->imu_started_ms) >= 12000u) {
        c->imu_requested = 0; expired |= F103_IMU_EXPIRED;
    }
    return expired;
}
#endif
