/* 自动触发边界测试：静止、短冲击、持续加速、无效采样、时钟回绕。 */
#include "auto_record.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static dart_imu sample(uint32_t time, int16_t ax, int16_t ay, int16_t az)
{
    dart_imu s = {0}; s.time_us = time; s.flags = IMU_ACCEL_VALID | IMU_GYRO_VALID;
    s.accel[0] = ax; s.accel[1] = ay; s.accel[2] = az; return s;
}
int main(void)
{
    dart_trigger trigger = {0}; dart_imu s;
    for (unsigned i = 0; i < 100; i++) {
        s = sample(i * 1000, 1365, 0, 0); assert(!dart_trigger_update(&trigger, &s));
        s = sample(i * 1000, 0, -1365, 0); assert(!dart_trigger_update(&trigger, &s));
        s = sample(i * 1000, 0, 0, 1365); assert(!dart_trigger_update(&trigger, &s));
    }
    memset(&trigger, 0, sizeof(trigger));
    for (unsigned i = 0; i < 20; i++) { s = sample(i * 1000, 4096, 0, 0); assert(!dart_trigger_update(&trigger, &s)); }
    s = sample(20000, 4096, 0, 0); assert(dart_trigger_update(&trigger, &s));
    s = sample(21000, 0, 0, 1365); assert(!dart_trigger_update(&trigger, &s));
    s = sample(22000, 4096, 0, 0); assert(!dart_trigger_update(&trigger, &s));
    /* 对角加速度也按合量判断；断档不能假装是持续超过20毫秒。 */
    memset(&trigger, 0, sizeof(trigger));
    s = sample(0, 2000, 2000, 0); assert(!dart_trigger_update(&trigger, &s));
    s.time_us = 25000; assert(!dart_trigger_update(&trigger, &s));
    s.time_us = 26000; s.flags |= IMU_SPI_ERROR; assert(!dart_trigger_update(&trigger, &s));
    s.time_us = 27000; s.flags = IMU_ACCEL_VALID; assert(!dart_trigger_update(&trigger, &s));
    memset(&trigger, 0, sizeof(trigger));
    for (unsigned i = 0; i <= 20; i++) {
        s = sample(0xFFFFF000u + i * 1000, -4096, 0, 0);
        assert(dart_trigger_update(&trigger, &s) == (i == 20));
    }
    s = sample(900000, -1365, 0, 0); s.gyro[0] = -16384;
    char line[256]; assert(dart_imu_csv_line(line, sizeof(line), &s, 1000000) > 0);
    assert(strstr(line, "-0.100000,-1000.000,0.000,0.000,-0.999755,0.000000,0.000000,1.000,"));
    int16_t maximum[] = {-32768, -32768, -32768};
    assert(dart_accel_norm_mg(maximum) >= 41568 && dart_accel_norm_mg(maximum) <= 41570);
    puts("PASS: trigger duration, impulse rejection, orientation, invalid/gapped data, clock wrap, readable CSV units");
    return 0;
}
