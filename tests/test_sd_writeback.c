/* 模拟卡的DMA完成/100ms忙/故障；实际编译生产diskio_sd.c及FatFs。
 * 全部文件系统操作只在内存模拟卡，不访问真实磁盘或板子。 */
#include "stm32h7xx_hal.h"
#include "diskio.h"
#include "fs_test.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SECTORS 2048u
SD_HandleTypeDef hsd1;
static uint8_t card[SECTORS*512], source[200*512], expected[SECTORS*512], readback[200*512];
static uint8_t dma_snapshot[32*512];
static uint32_t us,step_us=20,dma_delay=1000,card_delay=1000,start_us,busy_us;
static unsigned active,is_write,advancing,card_busy,usb_host,fail_start,fail_dma,fail_card,hang_dma,hang_card;
static unsigned aborts,transfers,writes,reads;
static unsigned irq_after_sample,stop_error_on_complete;
static uint8_t *dma_buffer;
static uint32_t dma_lba,dma_count;
static void advance(uint32_t delta)
{
    us+=delta;
    if(advancing)return;
    advancing=1;
    if(active && !hang_dma && (uint32_t)(us-start_us)>=dma_delay) {
        active=0;
        if(fail_dma){HAL_SD_ErrorCallback(&hsd1);}
        else if(is_write) {
            /* 还在DMA读/卡忙阶段的缓存绝不能被生产者覆盖。 */
            assert(!memcmp(dma_buffer,dma_snapshot,dma_count*512));
            memcpy(card+dma_lba*512,dma_buffer,dma_count*512);
            card_busy=1;busy_us=us;
            /* ST HAL在多块CMD12失败时先报Error，再报TxCplt。 */
            if(stop_error_on_complete)HAL_SD_ErrorCallback(&hsd1);
            HAL_SD_TxCpltCallback(&hsd1);
        } else {
            memcpy(dma_buffer,card+dma_lba*512,dma_count*512);
            HAL_SD_RxCpltCallback(&hsd1);
        }
    }
    if(card_busy && !hang_card && (uint32_t)(us-busy_us)>=card_delay)card_busy=0;
    advancing=0;
}
uint32_t board_micros(void)
{
    if(!advancing)advance(step_us);
    uint32_t sampled=us;
    /* 模拟真实board_micros采样后恢复中断，完成IRQ抢在函数返回前执行。 */
    if(!advancing && irq_after_sample) {
        irq_after_sample=0;
        advance(dma_delay);
    }
    return sampled;
}
uint32_t HAL_GetTick(void) { advance(step_us);return us/1000; }
DWORD get_fattime(void) { return (46u<<25)|(1u<<21)|(1u<<16); }
int usb_tf_host_active(void) { return (int)usb_host; }
uint32_t HAL_SD_GetError(SD_HandleTypeDef *h) { (void)h;return 0x12; }
HAL_StatusTypeDef HAL_SD_Abort(SD_HandleTypeDef *h) { (void)h;active=card_busy=0;aborts++;return HAL_OK; }
void HAL_SD_IRQHandler(SD_HandleTypeDef *h) { (void)h; }
HAL_SD_CardStateTypeDef HAL_SD_GetCardState(SD_HandleTypeDef *h)
{
    (void)h;advance(step_us);
    if(fail_card)return HAL_SD_CARD_ERROR;
    return active||card_busy ? HAL_SD_CARD_PROGRAMMING : HAL_SD_CARD_TRANSFER;
}
static HAL_StatusTypeDef start(uint8_t *data,uint32_t lba,uint32_t count,unsigned write)
{
    assert(!active && !card_busy && count && count<=32 && lba+count<=SECTORS);
    assert(!write || lba); /* 生产桥接不允许写物理0扇区。 */
    if(fail_start)return HAL_ERROR;
    active=1;is_write=write;dma_buffer=data;dma_lba=lba;dma_count=count;start_us=us;
    if(write)memcpy(dma_snapshot,data,count*512);
    transfers++;if(write)writes++;else reads++;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SD_WriteBlocks_DMA(SD_HandleTypeDef *h,const uint8_t *data,uint32_t lba,uint32_t count)
{ (void)h;return start((uint8_t *)data,lba,count,1); }
HAL_StatusTypeDef HAL_SD_ReadBlocks_DMA(SD_HandleTypeDef *h,uint8_t *data,uint32_t lba,uint32_t count)
{ (void)h;return start(data,lba,count,0); }
static void fill(unsigned n,unsigned seed)
{
    for(unsigned i=0;i<n*512;i++)source[i]=(uint8_t)(seed+i*37+(i>>9));
}
static void setup(void)
{
    memset(card,0,sizeof(card));memset(expected,0,sizeof(expected));
    fs_disk_set_ready(1,SECTORS);fs_disk_write_enabled=1;
    /* 先扫描卡布局，与真实挂载顺序一致。 */
    assert(disk_read(0,readback,0,1)==RES_OK);
    assert(fs_storage_begin()==0);
}
static void write_pattern(unsigned lba,unsigned n,unsigned seed)
{
    fill(n,seed);memcpy(expected+lba*512,source,n*512);
    assert(disk_write(0,source,lba,n)==RES_OK);
    memset(source,0xcd,n*512); /* disk_write返回后借入缓冲立即失效。 */
}
static void sync_and_check(void)
{
    assert(disk_ioctl(0,CTRL_SYNC,NULL)==RES_OK);
    const fs_writeback_stats *s=fs_storage_stats();
    assert(!s->queued_sectors && s->completed_sectors==s->enqueued_sectors);
    assert(!memcmp(card,expected,sizeof(card)) && !active && !card_busy);
}
static void normal_cases(void)
{
    setup();card_delay=100000;
    unsigned previous_reads=reads;
    uint32_t begin=us;
    write_pattern(100,32,1);write_pattern(132,96,2);
    assert((uint32_t)(us-begin)<10000 && fs_storage_stats()->queued_sectors==128);
    /* cache命中不等卡就绪，并且要返回最后一次写入的数据。 */
    assert(disk_read(0,readback,100,1)==RES_OK);
    assert(reads==previous_reads && !memcmp(readback,expected+100*512,512));
    write_pattern(228,1,3); /* 满缓存时有界等待，不能丢弃已经接收的数据。 */
    assert(fs_storage_stats()->full_waits && fs_storage_stats()->full_wait_max_us>=90000);
    sync_and_check();assert(fs_storage_stats()->peak_sectors==128);
    card_delay=1000;
    write_pattern(50,180,4);sync_and_check(); /* 大于缓存的请求分批，环形复用。 */
    write_pattern(400,1,5);write_pattern(400,1,6);
    assert(disk_read(0,readback,400,1)==RES_OK && !memcmp(readback,expected+400*512,512));
    /* 部分命中读取必须先排空，不能把旧卡数据盖过最新缓存。 */
    unsigned before_flush=fs_storage_stats()->read_flushes;
    assert(disk_read(0,readback,400,2)==RES_OK && !memcmp(readback,expected+400*512,1024));
    assert(fs_storage_stats()->read_flushes==before_flush+1);
    for(unsigned i=0;i<30;i++)write_pattern(500+(i*13)%100,1+i%7,30+i);
    sync_and_check();
    assert(disk_write(0,source,0,1)==RES_PARERR);
    assert(disk_write(0,source,SECTORS-1,2)==RES_PARERR);
    assert(disk_write(0,source,0xffffffffu,2)==RES_PARERR);
    assert(disk_write(0,NULL,1,1)==RES_PARERR && disk_write(0,source,1,0)==RES_PARERR);
    assert(disk_read(0,NULL,1,1)==RES_PARERR && disk_read(1,readback,1,1)==RES_NOTRDY);
    assert(fs_storage_end()==0);
    usb_host=1;assert(disk_write(0,source,1,1)==RES_WRPRT && fs_storage_begin()==-1);usb_host=0;
    /* 关写回后同步路径继续可用，新会话清空统计。 */
    write_pattern(800,8,19);assert(!memcmp(card,expected,sizeof(card)));
    assert(fs_storage_begin()==0 && fs_storage_stats()->enqueued_sectors==0);
    us=UINT32_MAX-1500;write_pattern(900,10,23);sync_and_check();assert(fs_storage_end()==0);
    puts("PASS: writeback source ownership/100ms stall/full backpressure/wrap/latest read/read barrier/sync/restart");
}
static void faults(const char *name)
{
    setup();fill(4,19);
    if(!strcmp(name,"start"))fail_start=1;
    assert(disk_write(0,source,10,4)==(fail_start ? RES_ERROR : RES_OK));
    if(!strcmp(name,"dma"))fail_dma=1;
    if(!strcmp(name,"late_dma")) {
        advance(dma_delay);assert(fs_storage_poll()==0);
        HAL_SD_ErrorCallback(&hsd1); /* DMA完成后的卡忙阶段也不能漏掉错误通知。 */
    }
    if(!strcmp(name,"card"))fail_card=1;
    if(!strcmp(name,"dma_timeout"))hang_dma=1;
    if(!strcmp(name,"card_timeout"))hang_card=1;
    if(!strcmp(name,"protect"))fs_disk_write_enabled=0;
    if(!strcmp(name,"usb"))usb_host=1;
    step_us=1000;
    assert(fs_storage_end()==-1);
    assert(fs_storage_stats()->error_stage && fs_storage_stats()->queued_sectors==4);
    unsigned count=transfers;assert(fs_storage_poll()==-1 && transfers==count && aborts==1);
    assert(disk_ioctl(0,CTRL_SYNC,NULL)==RES_ERROR && fs_storage_begin()==-1);
    puts("PASS: async error is latched, pending data retained, sync fails, no silent continuation");
}
static void partition_case(void)
{
    /* MBR偏移及LBA溢出检查，所有写入必须保持在数据分区内。 */
    card[510]=0x55;card[511]=0xaa;card[450]=0x06;card[454]=32;
    fs_disk_set_ready(1,SECTORS);fs_disk_write_enabled=1;
    uint32_t sectors=0;assert(disk_ioctl(0,GET_SECTOR_COUNT,&sectors)==RES_OK && sectors==SECTORS-32);
    assert(fs_storage_begin()==0);fill(2,71);
    assert(disk_write(0,source,1,2)==RES_OK);
    assert(disk_ioctl(0,CTRL_SYNC,NULL)==RES_OK && !memcmp(card+33*512,source,1024));
    assert(card[510]==0x55 && card[454]==32);
    assert(disk_write(0,source,UINT32_MAX-10,2)==RES_PARERR);
    assert(disk_read(0,readback,SECTORS-32,1)==RES_PARERR);
    assert(fs_storage_end()==0);puts("PASS: partition translation and range overflow protection");
}
static void completion_race_case(unsigned wrap)
{
    setup();
    if(wrap)us=UINT32_MAX-500u;
    write_pattern(40,4,17);
    assert(active);
    irq_after_sample=1;
    assert(fs_storage_poll()==0); /* 过去的时间样本不能减去IRQ里的未来完成时间。 */
    sync_and_check();
    assert(!fs_storage_stats()->error_stage && fs_storage_stats()->card_busy_max_us<10000);
    assert(fs_storage_end()==0);
    puts("PASS: completion IRQ after clock sampling, including microsecond wrap");
}
static void stop_error_race_case(void)
{
    setup();write_pattern(40,4,17);
    stop_error_on_complete=1;card_delay=0;step_us=dma_delay;
    /* 在poll开头检查错误后、读完成标志前，IRQ同时发布错误和完成。 */
    assert(fs_storage_poll()==-1);
    assert(fs_storage_stats()->error_stage==2);
    assert(fs_storage_stats()->queued_sectors==4 && !fs_storage_stats()->completed_sectors);
    assert(fs_storage_end()==-1 && aborts==1);
    puts("PASS: HAL stop-command error takes priority over completion in the same IRQ");
}
static void sync_fault_case(unsigned write,unsigned timeout)
{
    setup();assert(fs_storage_end()==0);fill(4,17);
    if(timeout)hang_dma=1;else fail_dma=1;
    step_us=1000;
    DRESULT result=write ? disk_write(0,source,40,4) : disk_read(0,readback,40,4);
    assert(result==RES_ERROR);
    assert(fs_disk_last_error==(0x12u|(timeout ? HAL_SD_ERROR_TIMEOUT : 0u)));
    /* 同步准备阶段超时也必须停止IDMA，清理不能复用仍在途的bounce。 */
    assert(!active && aborts==1);
    puts("PASS: synchronous preparation I/O retains cause and aborts failed DMA");
}
static void fatfs_case(void)
{
    /* 最小FAT12内存镜像：2048扇区，保留1/FAT6/root4，1扇区每簇。 */
    card[0]=0xeb;card[1]=0x3c;card[2]=0x90;memcpy(card+3,"MSDOS5.0",8);
    card[11]=0;card[12]=2;card[13]=1;card[14]=1;card[16]=1;card[17]=64;
    card[19]=0;card[20]=8;card[21]=0xf8;card[22]=6;card[24]=32;card[26]=64;
    card[510]=0x55;card[511]=0xaa;card[512]=0xf8;card[513]=card[514]=0xff;
    FATFS fs;FIL f,log;UINT n;uint8_t log_data[37];
    fs_disk_set_ready(1,SECTORS);fs_disk_write_enabled=1;
    assert(f_mount(&fs,"0:",1)==FR_OK);
    assert(f_open(&f,"0:/TEST.BIN",FA_WRITE|FA_CREATE_NEW)==FR_OK);
    assert(f_expand(&f,160000,1)==FR_OK);
    assert(f_open(&log,"0:/LOG.BIN",FA_WRITE|FA_CREATE_NEW)==FR_OK);
    assert(f_expand(&log,4096,1)==FR_OK);
    assert(fs_storage_begin()==0);
    for(unsigned i=0;i<19;i++) {
        fill(12,i);unsigned bytes=5003+i;
        assert(f_write(&f,source,bytes,&n)==FR_OK && n==bytes);
        memset(source,0xee,sizeof(source));
        for(unsigned j=0;j<sizeof(log_data);++j)log_data[j]=(uint8_t)(i+j);
        assert(f_write(&log,log_data,sizeof(log_data),&n)==FR_OK && n==sizeof(log_data));
        memset(log_data,0xad,sizeof(log_data));
    }
    assert(f_sync(&f)==FR_OK && !fs_storage_stats()->queued_sectors);
    assert(f_truncate(&f)==FR_OK && f_close(&f)==FR_OK);
    assert(f_truncate(&log)==FR_OK && f_close(&log)==FR_OK);
    assert(fs_storage_end()==0 && f_mount(NULL,"0:",0)==FR_OK);
    assert(f_mount(&fs,"0:",1)==FR_OK && f_open(&f,"0:/TEST.BIN",FA_READ)==FR_OK);
    for(unsigned i=0;i<19;i++) {
        fill(12,i);unsigned bytes=5003+i;
        assert(f_read(&f,readback,bytes,&n)==FR_OK && n==bytes && !memcmp(source,readback,bytes));
    }
    assert(f_close(&f)==FR_OK);
    assert(f_open(&log,"0:/LOG.BIN",FA_READ)==FR_OK);
    for(unsigned i=0;i<19;++i) {
        assert(f_read(&log,log_data,sizeof(log_data),&n)==FR_OK && n==sizeof(log_data));
        for(unsigned j=0;j<sizeof(log_data);++j)assert(log_data[j]==(uint8_t)(i+j));
    }
    assert(f_close(&log)==FR_OK && f_mount(NULL,"0:",0)==FR_OK);
    puts("PASS: real FatFs preallocation/interleaved files/unaligned writes/sync/close/remount/readback");
}
int main(int argc,char **argv)
{
    if(argc==1)normal_cases();
    else if(!strcmp(argv[1],"partition"))partition_case();
    else if(!strcmp(argv[1],"fatfs"))fatfs_case();
    else if(!strcmp(argv[1],"completion_race"))completion_race_case(0);
    else if(!strcmp(argv[1],"completion_wrap"))completion_race_case(1);
    else if(!strcmp(argv[1],"stop_error_race"))stop_error_race_case();
    else if(!strcmp(argv[1],"sync_read_timeout"))sync_fault_case(0,1);
    else if(!strcmp(argv[1],"sync_write_timeout"))sync_fault_case(1,1);
    else if(!strcmp(argv[1],"sync_read_error"))sync_fault_case(0,0);
    else if(!strcmp(argv[1],"sync_write_error"))sync_fault_case(1,0);
    else faults(argv[1]);
    return 0;
}
