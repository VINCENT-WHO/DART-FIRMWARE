/* 合成YUYV：RGB去白、绿色像素面积/质心/主目标选择、只读输入和分段计时。
 * 不使用旧录像，不将假时钟结果当成板端性能。 */
#include "green_detector.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t frame[DART_DET_PIXELS*2], original[sizeof(frame)];
static uint32_t ticks,calls;
static uint32_t fake_micros(void) { uint32_t v=ticks;ticks+=7;calls++;return v; }
static void rect(int x,int y,int w,int h,int yy,int cb,int cr)
{
    assert(!(x&1) && !(w&1));
    for(int r=y;r<y+h;r++) for(int c=x;c<x+w;c+=2) {
        uint8_t *p=frame+(r*DART_DET_WIDTH+c)*2;
        p[0]=p[2]=(uint8_t)yy;p[1]=(uint8_t)cb;p[3]=(uint8_t)cr;
    }
}
static void blank(void) { rect(0,0,DART_DET_WIDTH,DART_DET_HEIGHT,0,128,128); }
static void green(int x,int y,int w,int h) { rect(x,y,w,h,150,44,21); }
static dart_detection check(int found)
{
    dart_detection a,b;dart_detection_timing timing;
    memcpy(original,frame,sizeof(frame));memset(&timing,0xa5,sizeof(timing));
    ticks=UINT32_MAX-10;calls=0;
    det_reset_tracking(); /* 独立场景回归；跨帧行为由test_detector_roi覆盖。 */
    assert(green_detect_profiled(frame,&a,&timing,fake_micros)==0 && a.found==found);
    assert(!memcmp(frame,original,sizeof(frame)));
    uint32_t sum=0;
    for(unsigned i=0;i<DET_TIME_COUNT;i++)sum+=timing.stage_us[i];
    assert(sum==(calls-1)*7);
    assert(timing.stage_us[DET_TIME_INIT]==7 && timing.stage_us[DET_TIME_MASKS]==7);
    assert(timing.stage_us[DET_TIME_MEDIAN_GREEN]==7 && timing.stage_us[DET_TIME_SCAN]>0);
    uint32_t previous=calls;
    det_reset_tracking();
    assert(green_detect(frame,&b)==0 && !memcmp(&a,&b,sizeof(a)) && calls==previous);
    assert(!memcmp(frame,original,sizeof(frame)));
    if(found) assert(timing.stage_us[DET_TIME_SCORE]>0);
    return a;
}
int main(void)
{
    blank();check(0);
    rect(60,60,20,20,255,128,128);check(0);
    rect(60,60,20,20,180,128,128);check(0);
    rect(60,60,20,20,76,85,255);check(0);
    rect(60,60,20,20,29,255,107);check(0);
    rect(60,60,20,20,59,94,85);check(0); /* 深绿G约101 */
    rect(60,60,20,20,180,124,124);check(0); /* G与R/B太接近 */
    blank();green(80,80,4,10);check(0); /* 中值后36像素 */
    blank();green(80,80,4,11);
    dart_detection a=check(1);assert(a.area_q8==40*256);
    assert(a.center_x_q16==81*65536+32768 && a.center_y_q16==85*65536);
    blank();green(80,80,10,10);a=check(1);
    assert(a.area_q8==96*256 && a.score_x10000==960000);
    assert(a.center_x_q16==84*65536+32768 && a.center_y_q16==84*65536+32768);
    green(220,160,20,20);a=check(1);assert(a.box_x==220 && a.area_q8==396*256);
    green(20,20,20,20);a=check(1);assert(a.box_x==20); /* 同面积先到者 */
    blank();green(0,0,10,10);a=check(1);assert(a.box_x==0 && a.box_y==0);
    blank();green(100,100,20,20);rect(106,106,8,8,255,128,128);a=check(1);
    assert(a.area_q8==336*256 && a.center_x_q16==109*65536+32768);
    /* 绿色长条只做面积筛选，不再执行长宽比/圆度判定。 */
    blank();green(10,100,100,4);a=check(1);assert(a.area_q8==396*256);
    /* 满图大区域不能因栈满拆成假目标；大区域旁边的小目标也不能丢掉。 */
    green(0,0,DART_DET_WIDTH,DART_DET_HEIGHT);check(0);
    blank();green(0,0,200,180);green(280,200,10,10);a=check(1);assert(a.box_x==280);
    blank();green(80,80,10,10);check(1);
    dart_detection_timing timing;memset(&timing,0xa5,sizeof(timing));
    assert(green_detect_profiled(NULL,&a,&timing,fake_micros)==-1);
    for(unsigned i=0;i<DET_TIME_COUNT;i++)assert(timing.stage_us[i]==0);
    assert(green_detect_profiled(frame,NULL,&timing,fake_micros)==-1);
    assert(green_detect_profiled(frame,&a,&timing,NULL)==-1);
    puts("PASS: RGB/white rejection/area boundary/centroid/largest/border/holes/large component/read-only/timing wrap");
    return 0;
}
