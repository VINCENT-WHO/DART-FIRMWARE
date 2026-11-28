/* 生产模式策略和双板握手的主机测试，不访问外设/串口。 */
#include "control_mode.h"
#include "imu_session.h"
#include <assert.h>
#include <stdio.h>

static dart_packet wire_packet(uint8_t type, const void *data, size_t size)
{
    uint8_t bytes[DART_WIRE_MAX];
    size_t n = dart_encode(type, 1, data, size, bytes);
    dart_parser parser = {0}; dart_packet p = {0}; int result = 0;
    assert(n);
    for (size_t i = 0; i < n; ++i) result = dart_parse(&parser, bytes[i], &p);
    assert(result == 1);
    return p;
}
static void reply_to_h7(dart_imu_session *s, const char *text)
{
    dart_packet p = wire_packet(DART_REPLY, text, strlen(text));
    dart_imu_session_reply(s, &p);
}
static void normal_lifecycle(void)
{
    f103_control c; f103_control_init(&c, F103_MODE_NORMAL);
    dart_imu_session s = {0}; dart_imu sample = {0};
    assert(c.mode == F103_MODE_NORMAL && c.pwm_requested && c.imu_requested);
    for (unsigned i = 0; i < 4; ++i) assert(c.pulses[i] == 1500);
    assert(!f103_control_poll(&c, 13000));
    assert(!f103_control_poll(&c, UINT32_MAX));
    assert(!f103_control_poll(&c, 100));
    assert(c.pwm_requested && c.imu_requested);
    assert(!f103_control_pwm(&c, 0, 0));
    assert(!f103_control_pulse(&c, 1, 1900, 0));
    for (unsigned recording = 0; recording < 3; ++recording) {
        dart_imu_session_begin(&s);
        assert(!s.started && !s.released);
        const char *r = f103_control_imu(&c, "IMU WATCH", 1, recording * 30000);
        reply_to_h7(&s, r);
        assert(s.started);
        for (unsigned i = 0; i < 100; ++i) {
            sample.sequence++;
            dart_packet p = wire_packet(DART_IMU, &sample, sizeof(sample));
            assert(dart_imu_session_accepts(&s, &p));
        }
        dart_imu_session_end(&s); assert(!s.accepting && !s.released);
        r = f103_control_imu(&c, "IMU STOP", 1, recording * 30000 + 10000);
        assert(!strcmp(r, "IMU CONTINUOUS")); reply_to_h7(&s, r);
        assert(s.released && c.pwm_requested && c.imu_requested);
        /* 未记录、整理文件、USB读卡期间持续流不能进入接收队列。 */
        for (unsigned i = 0; i < 20000; ++i) {
            sample.sequence++;
            dart_packet p = wire_packet(DART_IMU, &sample, sizeof(sample));
            assert(!dart_imu_session_accepts(&s, &p));
        }
    }
    assert(!strcmp(f103_control_imu(&c, "IMU START", 1, 100), "IMU STARTED"));
    assert(c.watching && !f103_control_poll(&c, 999999));
    assert(!strcmp(f103_control_imu(&c, "IMU WATCH", 0, 0), "IMU ERROR ID/CONFIG/CLOCK"));
    assert(!f103_control_imu(&c, "IMU WATCH EXTRA", 1, 0));
}
static void debug_lifecycle(void)
{
    f103_control c; f103_control_init(&c, F103_MODE_DEBUG);
    assert(!c.pwm_requested && !c.imu_requested && !c.watching);
    assert(!f103_control_pulse(&c, 0, 1500, 0));
    assert(!f103_control_pulse(&c, 5, 1500, 0));
    assert(!f103_control_pulse(&c, 1, 999, 0));
    assert(!f103_control_pulse(&c, 1, 2001, 0));
    assert(f103_control_pulse(&c, 1, 1800, 0));
    assert(!c.pwm_requested); /* 预装脉宽不自动开启。 */
    assert(f103_control_pwm(&c, 1, 0));
    assert(!strcmp(f103_control_imu(&c, "IMU START", 1, 0), "IMU STARTED"));
    assert(c.pwm_requested && c.imu_requested); /* 并行，不再互斥。 */
    assert(c.pulses[0] == 1800 && c.pwm_requested && c.imu_requested);
    assert(!f103_control_poll(&c, 2999));
    assert(f103_control_pwm(&c, c.pwm_requested, 2999)); /* KEEP */
    assert(!f103_control_poll(&c, 5998));
    assert(f103_control_poll(&c, 5999) == F103_PWM_EXPIRED);
    assert(!c.pwm_requested && c.imu_requested);
    assert(!f103_control_poll(&c, 11999));
    assert(f103_control_poll(&c, 12000) == F103_IMU_EXPIRED);
    assert(!c.imu_requested);
    assert(!strcmp(f103_control_imu(&c, "IMU WATCH", 1, 12001), "IMU WATCHING"));
    assert(!f103_control_poll(&c, 999999) && c.imu_requested);
    assert(f103_control_pwm(&c, 1, 999999));
    assert(!strcmp(f103_control_imu(&c, "IMU STOP", 1, 999999), "IMU STOPPED"));
    assert(!c.imu_requested && c.pwm_requested);
    f103_control_init(&c, F103_MODE_NORMAL);
    assert(c.pwm_requested && c.imu_requested && c.watching && c.pulses[0] == 1500);
    f103_control_init(&c, F103_MODE_DEBUG);
    assert(!strcmp(f103_control_imu(&c, "IMU START", 0, 0), "IMU ERROR ID/CONFIG/CLOCK"));
    assert(!c.imu_requested);
    uint32_t now = UINT32_MAX - 1000u;
    assert(f103_control_pwm(&c, 1, now));
    assert(!strcmp(f103_control_imu(&c, "IMU START", 1, now), "IMU STARTED"));
    assert(!f103_control_poll(&c, now + 2999u));
    assert(f103_control_poll(&c, now + 3000u) == F103_PWM_EXPIRED);
    assert(f103_control_poll(&c, now + 12000u) == F103_IMU_EXPIRED);
}
static void handshake_validation(void)
{
    dart_imu_session s = {0}; dart_imu_session_begin(&s);
    reply_to_h7(&s, "IMU ERROR ID/CONFIG/CLOCK"); assert(!s.started);
    reply_to_h7(&s, "IMU WATCHING EXTRA"); assert(!s.started);
    reply_to_h7(&s, "IMU STARTED"); assert(s.started);
    dart_imu_session_end(&s);
    reply_to_h7(&s, "IMU STOPPED"); assert(s.released); /* 旧版/调试F103 */
    dart_imu_session_begin(&s);
    dart_packet p = wire_packet(DART_COMMAND, "IMU WATCHING", 12);
    dart_imu_session_reply(&s, &p); assert(!s.started);
    p = wire_packet(DART_IMU, "X", 1); assert(!dart_imu_session_accepts(&s, &p));
}
int main(void)
{
    f103_control boot; f103_control_init(&boot, F103_BOOT_MODE);
    assert(boot.mode == (DART_F103_DEBUG_MODE ? F103_MODE_DEBUG : F103_MODE_NORMAL));
    assert(boot.pwm_requested == !DART_F103_DEBUG_MODE && boot.imu_requested == !DART_F103_DEBUG_MODE);
    normal_lifecycle(); debug_lifecycle(); handshake_validation();
    puts("PASS: F103 normal continuous PWM/IMU, debug commands/timeouts/wrap, H750 session windows and wire acknowledgements");
    puts("NOTE: policy/protocol tests; physical PWM frequency and BMI088/UART timing require board validation");
    return 0;
}
