/* 只验证整数行换算、寄存器位及失败传播；不证明板端实际曝光时长。 */
#include "camera_exposure.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static unsigned regs[65536], writes, fail_read, fail_write;
static int read_reg(unsigned reg) { return reg == fail_read ? -1 : (int)regs[reg]; }
static int write_reg(unsigned reg, unsigned value)
{
    if (reg == fail_write) return -1;
    assert(value <= 255); regs[reg] = value; writes++; return 0;
}
static void reset(void)
{
    memset(regs,0,sizeof(regs));writes=0;fail_read=fail_write=0xffff;
    regs[0x3034]=0x1a;regs[0x3035]=0x11;regs[0x3036]=0x78;
    regs[0x3037]=0x13;regs[0x3108]=1;
    regs[0x380c]=DART_CAMERA_HTS>>8;regs[0x380d]=DART_CAMERA_HTS&255;
}
int main(void)
{
    assert(DART_CAMERA_EXPOSURE_LINES==152);
    assert(DART_CAMERA_EXPOSURE_RAW==0x980);
    assert(DART_CAMERA_EXPOSURE_NS==2995666);
    for (unsigned mode=0; mode<4; ++mode) {
        reset();regs[0x3503]=0xa0|mode;
        assert(camera_apply_fixed_exposure(read_reg,write_reg)==0);
        assert(regs[0x3503]==0xa1 && writes==4);
        assert(regs[0x3500]==0 && regs[0x3501]==9 && regs[0x3502]==0x80);
        assert(camera_verify_fixed_exposure(read_reg)==0);
    }
    const unsigned verify_regs[]={0x3034,0x3035,0x3036,0x3037,0x3108,0x380c,0x380d,0x3500,0x3501,0x3502,0x3503};
    for (unsigned i=0;i<sizeof(verify_regs)/sizeof(verify_regs[0]);++i) {
        reset();assert(camera_apply_fixed_exposure(read_reg,write_reg)==0);
        fail_read=verify_regs[i];assert(camera_verify_fixed_exposure(read_reg)==-1);
        fail_read=0xffff;regs[verify_regs[i]]^=1;
        assert(camera_verify_fixed_exposure(read_reg)==-1);
    }
    const unsigned written[]={0x3503,0x3500,0x3501,0x3502};
    for(unsigned i=0;i<4;++i) {
        reset();fail_write=written[i];assert(camera_apply_fixed_exposure(read_reg,write_reg)==-1);
        assert(writes==i);
    }
    reset();fail_read=0x3503;assert(camera_apply_fixed_exposure(read_reg,write_reg)==-1 && writes==0);
    puts("PASS: 3ms quantization/manual exposure/auto gain/register readback/failure propagation");
    return 0;
}
