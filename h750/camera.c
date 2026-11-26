/* Continuous YUV422 frame queue, adapted from the supplied board-verified driver.
 * 机载绿灯标注版：相机不再使用片上 JPEG 压缩，直接输出 YUV422 原始像素流，
 * 由固件完成检测、画圈标注与 JPEG 编码后写入 AVI。 */
#include "stm32h7xx_hal.h"
#include "camera_perf.h"
#include "camera_slots.h"
#include "camera_exposure.h"
#include <string.h>
#include <stdio.h>
extern uint32_t board_micros(void);
/* AXI SRAM 帧缓冲区：2 个 YUV 帧 slot（每帧 320x240x2=153600 字节），
 * 其余空间由绿灯检测器（掩膜）与 JPEG 编码输出缓冲使用。 */
uint8_t camera_workspace[307200u] __attribute__((section(".camera_buffer"),aligned(32)));
/* D2 SRAM：1 个帧 slot 加 IMU 队列与检测工作区。 */
static uint8_t d2_workspace[153600u] __attribute__((section(".camera_d2"),aligned(32)));
#define CAPACITY (153600u)
/* 槽位登记见 camera_slots.h：处理端永远取最新拍完的帧，用不上的旧帧被淘汰。 */
static uint8_t *buffers[CAM_SLOT_COUNT];
static camera_slots slots;
static volatile unsigned active;
volatile uint32_t camera_frames,camera_drops,camera_errors,camera_invalid;
uint32_t camera_exposure_verified;
static DMA_HandleTypeDef pdma;
static unsigned configured;
static int wr(unsigned a,unsigned b){return camera_write_register(a,b);}
static int pair(unsigned a,unsigned v){return wr(a,v>>8)||wr(a+1,v&255);}
static int change(unsigned a,unsigned mask,unsigned v){int old=camera_read_register(a);return old<0?-1:wr(a,((unsigned)old&~mask)|(v&mask));}
int camera_configure(void){
    configured=0;camera_exposure_verified=0;
    if(camera_perf_base())return -1;
    int e=0;e|=wr(0x3008,0x42);e|=wr(0x3039,0);e|=wr(0x3034,0x1A);
    e|=wr(0x3035,0x11);e|=wr(0x3036,0x78);e|=wr(0x3037,0x13);e|=wr(0x3108,1);
    e|=pair(0x3800,0);e|=pair(0x3802,4);e|=pair(0x3804,2623);e|=pair(0x3806,1951);
    e|=pair(0x3808,320);e|=pair(0x380A,240);e|=pair(0x380C,DART_CAMERA_HTS);e|=pair(0x380E,1015);
    e|=pair(0x3810,16);e|=pair(0x3812,6);e|=wr(0x3814,0x31);e|=wr(0x3815,0x31);
    e|=wr(0x3618,0);e|=wr(0x3612,0x29);e|=wr(0x3708,0x64);e|=wr(0x3709,0x52);e|=wr(0x370C,3);
    /* 翻转位（0x3820/0x3821 bit0）保留板级已验证组合；0x3821 bit5 是片上
     * JPEG 压缩使能，YUV422 模式必须清零。 */
    e|=change(0x3820,1,1);e|=change(0x3821,0x21,0x01);
    /* YUV422：0x4300=0x30 与 0x501F=0x00（Y0CbY1Cr 顺序）与 JPEG 模式取值相同。
     * JPEG 专属步骤全部撤销：0x3002 恢复通用值 0x1C（JPEG 块保持复位）、
     * 0x3006 恢复通用值 0xC3（JPEG 时钟关闭），参照 vendor 驱动 OV5640_Common。 */
    e|=wr(0x4300,0x30);e|=wr(0x501F,0);e|=wr(0x5001,0xA3);
    e|=wr(0x3002,0x1C);e|=wr(0x3006,0xC3);
    e|=wr(0x4713,3);e|=wr(0x4407,12);e|=wr(0x440E,0);
    e|=wr(0x460B,0x37);e|=wr(0x460C,0x22);e|=wr(0x3824,4);e|=wr(0x4740,0x23);
    e|=pair(0x4602,320);e|=pair(0x4604,240);
    e|=wr(0x4001,2);e|=wr(0x4004,2);e|=change(0x3A00,4,0);
    /* AEC关闭并写入固定快门，保留AGC；防频闪配置不再用于决定快门时长。 */
    const unsigned band_50_lines=507, band_60_lines=423;
    e|=pair(0x3A02,DART_CAMERA_EXPOSURE_LINES);e|=pair(0x3A14,DART_CAMERA_EXPOSURE_LINES);
    e|=pair(0x3A08,band_50_lines);e|=pair(0x3A0A,band_60_lines);
    e|=wr(0x3A0E,1);e|=wr(0x3A0D,1);
    e|=pair(0x350C,0);e|=camera_apply_fixed_exposure(camera_read_register,camera_write_register);
    e|=wr(0x503D,0);e|=wr(0x3008,2);HAL_Delay(600);
    e|=camera_verify_fixed_exposure(camera_read_register);
    camera_exposure_verified=e==0;
    configured=e==0;return e;
}
void camera_timing(void){
    static const unsigned regs[]={0x3034,0x3035,0x3036,0x3037,0x3108,
        0x3500,0x3501,0x3502,0x3503,0x350A,0x350B,0x350C,0x350D,
        0x380C,0x380D,0x380E,0x380F,0x3A00,0x3A02,0x3A03,
        0x3A08,0x3A09,0x3A0A,0x3A0B,0x3A0D,0x3A0E,0x3A14,0x3A15,0x460C,0x3824};
    char line[64];
    for(unsigned i=0;i<sizeof(regs)/sizeof(regs[0]);i++){
        int value=camera_read_register(regs[i]);
        snprintf(line,sizeof(line),"CAM TIMING reg=%04X value=%d\r\n",regs[i],value);bench_puts(line);
    }
    bench_puts("CAM TIMING END\r\n");
}
static void dma_off(void){DMA2_Stream1->CR&=~DMA_SxCR_EN;uint32_t t=DWT->CYCCNT;while(DMA2_Stream1->CR&DMA_SxCR_EN)if((uint32_t)(DWT->CYCCNT-t)>SystemCoreClock/1000){camera_errors|=8;break;}}
static void arm(void){DMA2->LIFCR=0xF40;DMA2_Stream1->M0AR=(uint32_t)buffers[slots.current];DMA2_Stream1->NDTR=CAPACITY/4;__DMB();DMA2_Stream1->CR|=DMA_SxCR_EN;}
int perf_dma_irq(void){return active?1:0;}
int perf_dcmi_irq(void){
    if(!active)return 0;
    uint32_t f=DCMI->MISR;DCMI->ICR=f;
    if(f&DCMI_RIS_OVR_RIS)camera_errors|=1;
    if(f&DCMI_RIS_ERR_RIS)camera_errors|=2;
    if(f&DCMI_RIS_FRAME_RIS){
        camera_frames++;uint32_t t=DWT->CYCCNT;
        while((DCMI->SR&DCMI_SR_FNE)&&((uint32_t)(DWT->CYCCNT-t)<SystemCoreClock/10000)){}
        dma_off();__DSB();uint32_t n=(CAPACITY/4-DMA2_Stream1->NDTR)*4;
        /* YUV422 帧没有 JPEG SOI/EOI 标记可扫描：完整帧的判定条件就是 DMA
         * 恰好传满 320x240x2 字节；不足说明行同步丢失，按无效帧丢弃。 */
        if(n==CAPACITY){
            /* 交付这一帧；没有空闲槽时淘汰最旧的就绪帧。只有两种帧计入丢帧：
             * 这一帧没交付，或者它挤掉了一帧更旧的。处理中的槽不会被改写。 */
            if(cam_frame_complete(&slots,board_micros(),n)!=CAM_FRAME_PUBLISHED)camera_drops++;
        }else camera_invalid++;
        arm();
    }
    return 1;
}
int camera_start(void){
    if(!configured)return -1;
    __HAL_RCC_D2SRAM1_CLK_ENABLE();__HAL_RCC_D2SRAM2_CLK_ENABLE();__HAL_RCC_D2SRAM3_CLK_ENABLE();
    camera_perf_bus();buffers[0]=camera_workspace;buffers[1]=camera_workspace+CAPACITY;buffers[2]=d2_workspace;
    memset(&pdma,0,sizeof(pdma));pdma.Instance=DMA2_Stream1;pdma.Init.Request=DMA_REQUEST_DCMI;
    pdma.Init.Direction=DMA_PERIPH_TO_MEMORY;pdma.Init.MemInc=DMA_MINC_ENABLE;pdma.Init.PeriphInc=DMA_PINC_DISABLE;
    pdma.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;pdma.Init.MemDataAlignment=DMA_MDATAALIGN_WORD;
    pdma.Init.Mode=DMA_NORMAL;pdma.Init.Priority=DMA_PRIORITY_VERY_HIGH;pdma.Init.FIFOMode=DMA_FIFOMODE_DISABLE;
    if(HAL_DMA_Init(&pdma)!=HAL_OK)return -1;
    DMA2_Stream1->PAR=(uint32_t)&DCMI->DR;
    DMA2_Stream1->CR&=~(DMA_SxCR_TCIE|DMA_SxCR_HTIE|DMA_SxCR_TEIE|DMA_SxCR_DMEIE);
    /* Board-verified polarity combination 5. JPEG 位不再置位：YUV422 像素流
     * 需要 DCMI 硬件按 VSYNC/HREF 同步每帧。 */
    DCMI->CR=DCMI_CR_PCKPOL|DCMI_CR_HSPOL;
    DCMI->ICR=0x1F;DCMI->IER=DCMI_IER_FRAME_IE|DCMI_IER_OVR_IE|DCMI_IER_ERR_IE;
    camera_frames=camera_drops=camera_errors=camera_invalid=0;cam_slots_reset(&slots);
    active=1;arm();HAL_NVIC_SetPriority(DCMI_IRQn,2,0);HAL_NVIC_ClearPendingIRQ(DCMI_IRQn);HAL_NVIC_EnableIRQ(DCMI_IRQn);
    DCMI->CR|=DCMI_CR_ENABLE|DCMI_CR_CAPTURE;return 0;
}
void camera_stop(void){HAL_NVIC_DisableIRQ(DCMI_IRQn);DCMI->IER=0;DCMI->CR&=~(DCMI_CR_CAPTURE|DCMI_CR_ENABLE);dma_off();active=0;DCMI->ICR=0x1F;}
/* 取最新拍完的一帧：处理一帧远慢于相机出帧，排队最久的帧是过期画面。
 * 没有就绪帧返回 -1，调用方不应等待，直接做别的事再来取。 */
int camera_take(const uint8_t **bytes,uint32_t *length,uint32_t *time){
    uint32_t irq=__get_PRIMASK();__disable_irq();
    int slot=cam_take_newest(&slots,length,time);
    if(slot>=0)*bytes=buffers[slot];
    __set_PRIMASK(irq);return slot;
}
void camera_release(unsigned slot){cam_release(&slots,slot);}
