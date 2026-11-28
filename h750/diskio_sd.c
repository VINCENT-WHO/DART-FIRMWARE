/* FatFs SDMMC桥接：录像期64KiB有界延迟写，其余路径同步DMA。
 * Only an explicit file-test session may enable disk writes.
 * 分区自适应：现代 Windows/读卡器格式化的卡带 MBR/GPT 分区表（FAT 卷
 * 从非 0 扇区开始），而 FatFs 以 FF_MULTI_PARTITION=0 编译，只认 0 扇区
 * 起始的裸卡布局。本驱动在首次磁盘访问时扫描分区表并记录卷起始扇区，
 * 之后对所有扇区访问统一平移——裸卡、MBR、GPT 布局都能挂载。
 */
#include "stm32h7xx_hal.h"
#include "diskio.h"
#include "fs_test.h"
#include "usb_tf.h"
#include <string.h>

extern SD_HandleTypeDef hsd1;
extern uint32_t board_micros(void);
volatile uint32_t fs_disk_write_enabled;
volatile uint32_t fs_disk_last_error;
volatile uint32_t fs_disk_reads, fs_disk_writes;
volatile uint32_t fs_disk_read_sectors, fs_disk_write_sectors;
static uint32_t disk_ready, disk_sectors;
/* FAT 卷起始扇区（裸卡=0）与扫描状态；换卡/复位后重新扫描。 */
static uint32_t part_offset;
static unsigned part_scanned;
/* 同步读写用4KiB中转；写回缓存本身位于AXI，可直接交给SDMMC IDMA。
 * DTCM中的日志/文件系统缓存必须先复制。D-cache保持关闭。 */
static uint8_t sd_bounce[4096] __attribute__((section(".camera_buffer"), aligned(32)));
#define SD_CHUNK 8u
#define CACHE_SECTORS (FS_WRITEBACK_BYTES / 512u)
#define ASYNC_CHUNK_SECTORS 32u /* 连续物理LBA合并，最多16KiB；不跨环形边界。 */
#define STORAGE_TIMEOUT_US 5000000u
static uint8_t write_cache[FS_WRITEBACK_BYTES] __attribute__((section(".camera_buffer"), aligned(32)));
static uint32_t cache_lba[CACHE_SECTORS], cache_time[CACHE_SECTORS];
static unsigned cache_head, cache_count, active_sectors, async_enabled;
enum { STORE_IDLE, STORE_DMA, STORE_CARD, STORE_ERROR };
static unsigned store_state;
static uint32_t phase_start_us, last_card_poll_us;
static fs_writeback_stats write_stats;
static volatile unsigned dma_done,dma_failed;
static volatile uint32_t dma_done_us;
void SDMMC1_IRQHandler(void) {HAL_SD_IRQHandler(&hsd1);}
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *h) {if(h==&hsd1){dma_done_us=board_micros();dma_done=1;}}
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *h) {if(h==&hsd1){dma_done_us=board_micros();dma_done=1;}}
void HAL_SD_ErrorCallback(SD_HandleTypeDef *h) {if(h==&hsd1)dma_failed=1;}
static DRESULT wait_transfer(void);
static int storage_fail(uint32_t stage, uint32_t extra)
{
    if (store_state != STORE_ERROR) {
        fs_disk_last_error = HAL_SD_GetError(&hsd1) | extra;
        write_stats.error_stage = stage;
        store_state = STORE_ERROR;
        (void)HAL_SD_Abort(&hsd1);
    }
    return -1; /* 故障锁存，不清除未完成数据，不允许自动重试/乱序继续写。 */
}

const fs_writeback_stats *fs_storage_stats(void) { return &write_stats; }

int fs_storage_poll(void)
{
    if (store_state == STORE_ERROR) return -1;
    if (!async_enabled) return 0;
    if (!disk_ready || !fs_disk_write_enabled || usb_tf_host_active()) return storage_fail(1,0);
    if ((store_state==STORE_DMA || store_state==STORE_CARD) && dma_failed) return storage_fail(2,0);
    uint32_t now = board_micros();
    if (store_state == STORE_DMA) {
        if (!dma_done) {
            if ((uint32_t)(now-phase_start_us) >= STORAGE_TIMEOUT_US)
                return storage_fail(3,HAL_SD_ERROR_TIMEOUT);
            return 0;
        }
        /* IRQ可能在本轮开头检查dma_failed/采样now之后才运行。
         * ST HAL的CMD12失败还会先报Error、随后报TxCplt，必须以错误为准。 */
        __DMB();
        if (dma_failed) return storage_fail(2,0);
        uint32_t elapsed = dma_done_us-phase_start_us;
        write_stats.dma_us += elapsed;
        if (elapsed > write_stats.dma_max_us) write_stats.dma_max_us = elapsed;
        phase_start_us = dma_done_us;
        /* 观察到完成后重新取时：旧now可能早于dma_done_us，无符号相减
         * 会下溢成约4294秒，导致正常传输被立即判为5秒卡忙超时。 */
        now = board_micros();
        last_card_poll_us = now-1000u;
        store_state = STORE_CARD;
    }
    if (store_state == STORE_CARD) {
        if ((uint32_t)(now-phase_start_us) >= STORAGE_TIMEOUT_US)
            return storage_fail(4,HAL_SD_ERROR_TIMEOUT);
        if ((uint32_t)(now-last_card_poll_us) < 1000u) return 0;
        last_card_poll_us = now;
        HAL_SD_CardStateTypeDef state = HAL_SD_GetCardState(&hsd1);
        if (state == HAL_SD_CARD_ERROR) return storage_fail(5,0);
        if (state != HAL_SD_CARD_TRANSFER) return 0;
        now = board_micros();
        uint32_t elapsed = now-phase_start_us;
        write_stats.card_busy_us += elapsed;
        if (elapsed > write_stats.card_busy_max_us) write_stats.card_busy_max_us = elapsed;
        /* 卡就绪后才回收，确保DMA源稳定，排队统计包含卡忙与调度间隔。 */
        for (unsigned i=0;i<active_sectors;++i) {
            elapsed = now-cache_time[(cache_head+i)%CACHE_SECTORS];
            if (elapsed > write_stats.queue_latency_max_us) write_stats.queue_latency_max_us=elapsed;
        }
        cache_head=(cache_head+active_sectors)%CACHE_SECTORS;
        cache_count-=active_sectors;
        write_stats.completed_sectors+=active_sectors;
        write_stats.queued_sectors=cache_count;
        active_sectors=0;store_state=STORE_IDLE;
    }
    if (store_state == STORE_IDLE && cache_count) {
        unsigned n=1;
        while(n<cache_count && n<ASYNC_CHUNK_SECTORS && cache_head+n<CACHE_SECTORS &&
              cache_lba[cache_head+n]==cache_lba[cache_head]+n) n++;
        active_sectors=n;dma_done=dma_failed=0;store_state=STORE_DMA;
        phase_start_us=board_micros();__DMB();
        if (HAL_SD_WriteBlocks_DMA(&hsd1,write_cache+cache_head*512u,cache_lba[cache_head],n)!=HAL_OK)
            return storage_fail(6,0);
        write_stats.dma_transfers++;
    }
    return 0;
}

static int storage_drain(void)
{
    uint32_t begin=board_micros();
    if (fs_storage_poll()) return -1;
    while(cache_count || store_state==STORE_DMA || store_state==STORE_CARD) {
        if (fs_storage_poll()) return -1;
        if ((uint32_t)(board_micros()-begin)>=STORAGE_TIMEOUT_US)
            return storage_fail(7,HAL_SD_ERROR_TIMEOUT);
    }
    uint32_t elapsed=board_micros()-begin;
    if (elapsed>write_stats.sync_max_us) write_stats.sync_max_us=elapsed;
    return 0;
}

int fs_storage_begin(void)
{
    if (async_enabled || store_state==STORE_ERROR || !disk_ready ||
        !fs_disk_write_enabled || usb_tf_host_active() || wait_transfer()!=RES_OK) return -1;
    cache_head=cache_count=active_sectors=0;store_state=STORE_IDLE;
    memset(&write_stats,0,sizeof(write_stats));async_enabled=1;
    return 0;
}
int fs_storage_end(void)
{
    if (storage_drain()) return -1;
    async_enabled=0;
    return 0;
}

static DRESULT cache_write(const BYTE *data, uint32_t sector, UINT count)
{
    if (fs_storage_poll()) return RES_ERROR;
    while(count) {
        if (cache_count==CACHE_SECTORS) {
            uint32_t begin=board_micros();write_stats.full_waits++;
            do {
                if (fs_storage_poll()) return RES_ERROR;
                if ((uint32_t)(board_micros()-begin)>=STORAGE_TIMEOUT_US) {
                    storage_fail(8,HAL_SD_ERROR_TIMEOUT);return RES_ERROR;
                }
            } while(cache_count==CACHE_SECTORS);
            uint32_t elapsed=board_micros()-begin;
            write_stats.full_wait_us+=elapsed;
            if (elapsed>write_stats.full_wait_max_us) write_stats.full_wait_max_us=elapsed;
        }
        unsigned tail=(cache_head+cache_count)%CACHE_SECTORS;
        unsigned n=CACHE_SECTORS-cache_count;
        if(n>count)n=count;
        if(n>CACHE_SECTORS-tail)n=CACHE_SECTORS-tail;
        memcpy(write_cache+tail*512u,data,n*512u);
        uint32_t now=board_micros();
        for(unsigned i=0;i<n;++i){cache_lba[tail+i]=sector+i;cache_time[tail+i]=now;}
        cache_count+=n;write_stats.enqueued_sectors+=n;write_stats.queued_sectors=cache_count;
        if(cache_count>write_stats.peak_sectors)write_stats.peak_sectors=cache_count;
        data+=n*512u;sector+=n;count-=n;
        if(fs_storage_poll())return RES_ERROR;
    }
    return RES_OK;
}

/* 后写同一LBA不能被尚未落盘的旧版本盖住。全请求命中才直接返回；
 * 部分未命中则先排空全部写，再同步读，保证读写顺序和FatFs一致性。 */
static int cache_read(BYTE *data,uint32_t sector,UINT count)
{
    for(UINT i=0;i<count;++i) {
        int found=0;
        for(unsigned j=cache_count;j>0;--j) {
            unsigned slot=(cache_head+j-1)%CACHE_SECTORS;
            if(cache_lba[slot]==sector+i){memcpy(data+i*512u,write_cache+slot*512u,512u);found=1;break;}
        }
        if(!found)return 0;
    }
    write_stats.cache_read_sectors+=count;
    return 1;
}

static DRESULT transfer_dma(BYTE *data,LBA_t sector,UINT count,unsigned write) {
    while(count) {
        UINT n=count>SD_CHUNK?SD_CHUNK:count;
        if(write&&data!=sd_bounce)memcpy(sd_bounce,data,n*512U);
        dma_done=dma_failed=0;__DMB();
        HAL_StatusTypeDef r=write?HAL_SD_WriteBlocks_DMA(&hsd1,sd_bounce,sector,n):HAL_SD_ReadBlocks_DMA(&hsd1,sd_bounce,sector,n);
        uint32_t t=HAL_GetTick();
        if(r==HAL_OK)while(!dma_done&&!dma_failed&&HAL_GetTick()-t<5000U){}
        if(r!=HAL_OK||dma_failed||!dma_done) {
            uint32_t timeout = r==HAL_TIMEOUT || (r==HAL_OK && !dma_failed && !dma_done);
            fs_disk_last_error=HAL_SD_GetError(&hsd1)|(timeout ? HAL_SD_ERROR_TIMEOUT : 0u);
            /* 先保存原因，再终止IDMA；失败清理不能复用仍在途的bounce。 */
            (void)HAL_SD_Abort(&hsd1);
            return RES_ERROR;
        }
        if(wait_transfer()!=RES_OK)return RES_ERROR;
        if(!write&&data!=sd_bounce)memcpy(data,sd_bounce,n*512U);
        sector+=n;count-=n;data+=n*512U;
    }
    return RES_OK;
}

void fs_disk_set_ready(uint32_t ready, uint32_t sectors)
{
    if(cache_count || store_state==STORE_DMA || store_state==STORE_CARD) {
        storage_fail(9,0);disk_ready=0;return;
    }
    disk_ready = ready;
    disk_sectors = sectors;
    part_offset = 0;
    part_scanned = 0;
    fs_disk_write_enabled = 0;
    async_enabled=0;store_state=STORE_IDLE;cache_head=cache_count=active_sectors=0;
    memset(&write_stats,0,sizeof(write_stats));
}

/* 读物理 0 扇区判定卡布局：FAT 引导扇区 -> 裸卡 offset 0；MBR -> 取第一
 * 个普通主分区的起始 LBA；GPT(保护性 MBR) -> 取 GPT 第一个分区项。
 * 物理读失败时保持 part_scanned=0，下次访问自动重扫，不缓存失败状态。 */
static void scan_partition(void)
{
    part_offset = 0;
    if (!disk_ready || disk_sectors < 8) { part_scanned = 1; return; }
    if (transfer_dma(sd_bounce, 0, 1, 0) != RES_OK) return;
    part_scanned = 1;
    const uint8_t *b = sd_bounce;
    if (b[510] != 0x55 || b[511] != 0xAA) return;   /* 无引导签名 */
    unsigned bps = b[11] | (b[12] << 8);
    unsigned spc = b[13];
    if (bps >= 512 && bps <= 4096 && !(bps & (bps - 1)) &&
        spc && !(spc & (spc - 1)))
        return;                                     /* 0 扇区即 FAT 卷：裸卡 */
    for (unsigned i = 0; i < 4; i++) {
        const uint8_t *e = b + 446 + 16 * i;
        unsigned type = e[4];
        if (type == 0xEE) {                         /* GPT：分区表在 LBA2+ */
            if (transfer_dma(sd_bounce, 2, 1, 0) == RES_OK) {
                uint32_t first = (uint32_t)sd_bounce[32] | ((uint32_t)sd_bounce[33] << 8) |
                                ((uint32_t)sd_bounce[34] << 16) | ((uint32_t)sd_bounce[35] << 24);
                if (first && first < disk_sectors) part_offset = first;
            }
            return;
        }
        if (type == 0 || type == 0x05 || type == 0x0F || type == 0x82) continue; /* 空项/扩展/Solaris */
        uint32_t lba = (uint32_t)e[8] | ((uint32_t)e[9] << 8) |
                       ((uint32_t)e[10] << 16) | ((uint32_t)e[11] << 24);
        if (lba && lba < disk_sectors) { part_offset = lba; return; }
    }
}

static DRESULT wait_transfer(void)
{
    uint32_t started = HAL_GetTick();
    do {
        HAL_SD_CardStateTypeDef state = HAL_SD_GetCardState(&hsd1);
        if (state == HAL_SD_CARD_TRANSFER) return RES_OK;
        if (state == HAL_SD_CARD_ERROR) {
            fs_disk_last_error = HAL_SD_GetError(&hsd1);
            return RES_ERROR;
        }
    } while (HAL_GetTick() - started < 5000U);
    fs_disk_last_error = HAL_SD_GetError(&hsd1) | HAL_SD_ERROR_TIMEOUT;
    return RES_ERROR;
}

DSTATUS disk_status(BYTE drive)
{
    return drive == 0 && disk_ready ? 0 : STA_NOINIT;
}

DSTATUS disk_initialize(BYTE drive)
{
    /* The explicit SD bring-up already initialized the card and selected 4bit.
     * Never reset SDMMC underneath a mounted filesystem.
     */
    return disk_status(drive);
}

static int valid_range(LBA_t sector, UINT count)
{
    /* 在加分区偏移之前验证，避免32位LBA加法溢出绕回卡头。 */
    return part_offset < disk_sectors && count > 0 && sector < disk_sectors-part_offset &&
        count <= disk_sectors-part_offset-sector;
}

DRESULT disk_read(BYTE drive, BYTE *buffer, LBA_t sector, UINT count)
{
    if (disk_status(drive)) return RES_NOTRDY;
    if (!part_scanned) {
        scan_partition();
        if (!part_scanned) return RES_ERROR;   /* 物理扇区 0 读失败，下次重扫 */
    }
    if (!buffer || !valid_range(sector, count)) return RES_PARERR;
    if (fs_storage_poll()) return RES_ERROR;
    if(async_enabled && cache_count && cache_read(buffer,(uint32_t)sector+part_offset,count)) {
        ++fs_disk_reads;fs_disk_read_sectors+=count;return RES_OK;
    }
    if(cache_count)write_stats.read_flushes++;
    if(storage_drain())return RES_ERROR;
    if (wait_transfer() != RES_OK) return RES_ERROR;
    /* transfer_dma已保存软件超时位，不能用HAL错误覆盖。 */
    if (transfer_dma(buffer,sector+part_offset,count,0) != RES_OK) return RES_ERROR;
    ++fs_disk_reads; fs_disk_read_sectors += count;
    return wait_transfer();
}

DRESULT disk_write(BYTE drive, const BYTE *buffer, LBA_t sector, UINT count)
{
    if (disk_status(drive)) return RES_NOTRDY;
    if (!part_scanned) {
        scan_partition();
        if (!part_scanned) return RES_ERROR;   /* 物理扇区 0 读失败，下次重扫 */
    }
    if (!fs_disk_write_enabled || usb_tf_host_active()) return RES_WRPRT;
    /* 物理扇区 0 是 MBR/GPT 头，文件操作永远不允许写它。 */
    if (!buffer || !sector || !valid_range(sector, count)) return RES_PARERR;
    if (store_state==STORE_ERROR) return RES_ERROR;
    if(async_enabled) {
        DRESULT result=cache_write(buffer,(uint32_t)sector+part_offset,count);
        if(result==RES_OK){++fs_disk_writes;fs_disk_write_sectors+=count;}
        return result;
    }
    if (wait_transfer() != RES_OK) return RES_ERROR;
    if (transfer_dma((BYTE *)buffer,sector+part_offset,count,1) != RES_OK) return RES_ERROR;
    ++fs_disk_writes; fs_disk_write_sectors += count;
    return wait_transfer();
}

DRESULT disk_ioctl(BYTE drive, BYTE command, void *buffer)
{
    if (disk_status(drive)) return RES_NOTRDY;
    switch (command) {
    case CTRL_SYNC: return storage_drain() ? RES_ERROR : wait_transfer();
    case GET_SECTOR_COUNT:
        if (!buffer) return RES_PARERR;
        if (!part_scanned) scan_partition();
        if (!part_scanned) return RES_ERROR;
        *(LBA_t *)buffer = disk_sectors - part_offset; return RES_OK;
    case GET_SECTOR_SIZE:
        if (!buffer) return RES_PARERR;
        *(WORD *)buffer = 512; return RES_OK;
    case GET_BLOCK_SIZE:
        if (!buffer) return RES_PARERR;
        *(DWORD *)buffer = 1; return RES_OK;
    default: return RES_PARERR; /* No format, trim, erase or arbitrary control. */
    }
}

/* H7 DISK 控制台诊断：返回原始物理扇区 0 的 DMA 试读结果。
 * info[0]=disk_ready info[1]=disk_sectors info[2]=part_scanned
 * info[3]=part_offset(扫描前) info[4]=试读前卡状态 info[5]=试读前 HAL 错误
 * info[6]=试读 DRESULT info[7]=试读后 fs_disk_last_error
 * info[8..11]=物理扇区 0 前 16 字节 info[12]=510/511 签名
 * 返回 0=试读成功，1=失败。调用方保证 info 至少 13 个 uint32。 */
int fs_disk_probe(uint32_t *info)
{
    unsigned i;
    for (i = 0; i < 13; i++) info[i] = 0;
    info[0] = disk_ready;
    info[1] = disk_sectors;
    info[2] = part_scanned;
    info[3] = part_offset;
    info[4] = (uint32_t)HAL_SD_GetCardState(&hsd1);
    info[5] = HAL_SD_GetError(&hsd1);
    DRESULT r = storage_drain() ? RES_ERROR : transfer_dma(sd_bounce, 0, 1, 0);
    info[6] = (uint32_t)r;
    info[7] = fs_disk_last_error;
    memcpy(&info[8], sd_bounce, 16);
    info[12] = sd_bounce[510] | ((uint32_t)sd_bounce[511] << 8);
    return r == RES_OK ? 0 : 1;
}
