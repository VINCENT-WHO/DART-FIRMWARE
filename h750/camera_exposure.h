#ifndef DART_CAMERA_EXPOSURE_H
#define DART_CAMERA_EXPOSURE_H
#include <stdint.h>

/* 固定24MHz XCLK；PLL寄存器3034=1A,3035=11,3036=78,3037=13,3108=01。
 * sysclk=24MHz*120/3/1/2*2/5/2=96MHz，HTS=1892，行周期19.708333us。
 * 采用整数行曝光（与Linux OV5640驱动一致），3000us四舍五入为152行。
 * 3500..3502保存lines<<4，低4位保持0。参考：
 * https://github.com/torvalds/linux/blob/master/drivers/media/i2c/ov5640.c
 * ov5640_get_sysclk / ov5640_set_exposure / ov5640_set_autoexposure。 */
#define DART_CAMERA_XCLK_HZ 24000000u
#define DART_CAMERA_SYSCLK_HZ 96000000u
#define DART_CAMERA_HTS 1892u
#define DART_CAMERA_EXPOSURE_US 3000u
#define DART_CAMERA_EXPOSURE_LINES ((uint32_t)(((uint64_t)DART_CAMERA_EXPOSURE_US * \
    DART_CAMERA_SYSCLK_HZ + (uint64_t)DART_CAMERA_HTS * 500000u) / \
    ((uint64_t)DART_CAMERA_HTS * 1000000u)))
#define DART_CAMERA_EXPOSURE_RAW (DART_CAMERA_EXPOSURE_LINES << 4)
#define DART_CAMERA_EXPOSURE_NS ((uint32_t)((uint64_t)DART_CAMERA_EXPOSURE_LINES * \
    DART_CAMERA_HTS * 1000000000u / DART_CAMERA_SYSCLK_HZ))

/* 只切换曝光到手动，增益仍自动（3503 bit1=0）；保留其他控制位。 */
static inline int camera_apply_fixed_exposure(int (*read_reg)(unsigned),
                                              int (*write_reg)(unsigned, unsigned))
{
    int mode = read_reg(0x3503);
    if (mode < 0) return -1;
    if (write_reg(0x3503, ((unsigned)mode & ~3u) | 1u)) return -1;
    if (write_reg(0x3500, (DART_CAMERA_EXPOSURE_RAW >> 16) & 0x0fu) ||
        write_reg(0x3501, (DART_CAMERA_EXPOSURE_RAW >> 8) & 0xffu) ||
        write_reg(0x3502, DART_CAMERA_EXPOSURE_RAW & 0xffu)) return -1;
    return 0;
}

/* 在相机启动/稳定后回读，失败则不允许进入采集。 */
static inline int camera_verify_fixed_exposure(int (*read_reg)(unsigned))
{
    static const unsigned regs[] = {0x3034,0x3035,0x3036,0x3037,0x3108,
        0x380c,0x380d,0x3500,0x3501,0x3502};
    static const unsigned expected[] = {0x1a,0x11,0x78,0x13,0x01,
        DART_CAMERA_HTS >> 8,DART_CAMERA_HTS & 255,
        DART_CAMERA_EXPOSURE_RAW >> 16,(DART_CAMERA_EXPOSURE_RAW >> 8) & 255,
        DART_CAMERA_EXPOSURE_RAW & 255};
    for (unsigned i = 0; i < sizeof(regs)/sizeof(regs[0]); ++i)
        if (read_reg(regs[i]) != (int)expected[i]) return -1;
    int mode = read_reg(0x3503);
    return mode >= 0 && (mode & 3) == 1 ? 0 : -1;
}
#endif
