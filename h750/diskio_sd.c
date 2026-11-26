/* Polling SDMMC bridge for FatFs. No DMA/cache dependency, format, or trim.
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
volatile uint32_t fs_disk_write_enabled;
volatile uint32_t fs_disk_last_error;
volatile uint32_t fs_disk_reads, fs_disk_writes;
volatile uint32_t fs_disk_read_sectors, fs_disk_write_sectors;
static uint32_t disk_ready, disk_sectors;
/* FAT 卷起始扇区（裸卡=0）与扫描状态；换卡/复位后重新扫描。 */
static uint32_t part_offset;
static unsigned part_scanned;
/* SD 传输缓冲区：SDMMC 内部 DMA 无法访问 DTCM（log_buffer 等 .bss 数据），
 * 所有卡访问都经此缓冲中转。独占 4KB，放在 AXI SRAM 段（与 v2.0 验证过的
 * DMA 目标区一致，SDMMC IDMA 访问 AXI 已被旧固件长期验证）；每次 DMA 最多
 * 8 扇区。不与相机帧槽、检测掩膜、JPEG 缓冲重叠。 */
static uint8_t sd_bounce[4096] __attribute__((section(".camera_buffer"), aligned(32)));
#define SD_CHUNK 8u
static volatile unsigned dma_done,dma_failed;
void SDMMC1_IRQHandler(void) {HAL_SD_IRQHandler(&hsd1);}
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *h) {(void)h;dma_done=1;}
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *h) {(void)h;dma_done=1;}
void HAL_SD_ErrorCallback(SD_HandleTypeDef *h) {(void)h;dma_failed=1;}
static DRESULT wait_transfer(void);
static DRESULT transfer_dma(BYTE *data,LBA_t sector,UINT count,unsigned write) {
    while(count) {
        UINT n=count>SD_CHUNK?SD_CHUNK:count;
        if(write&&data!=sd_bounce)memcpy(sd_bounce,data,n*512U);
        dma_done=dma_failed=0;__DMB();
        HAL_StatusTypeDef r=write?HAL_SD_WriteBlocks_DMA(&hsd1,sd_bounce,sector,n):HAL_SD_ReadBlocks_DMA(&hsd1,sd_bounce,sector,n);
        uint32_t t=HAL_GetTick();
        if(r==HAL_OK)while(!dma_done&&!dma_failed&&HAL_GetTick()-t<5000U){}
        if(r!=HAL_OK||dma_failed||!dma_done) {fs_disk_last_error=HAL_SD_GetError(&hsd1)|HAL_SD_ERROR_TIMEOUT;return RES_ERROR;}
        if(wait_transfer()!=RES_OK)return RES_ERROR;
        if(!write&&data!=sd_bounce)memcpy(data,sd_bounce,n*512U);
        sector+=n;count-=n;data+=n*512U;
    }
    return RES_OK;
}

void fs_disk_set_ready(uint32_t ready, uint32_t sectors)
{
    disk_ready = ready;
    disk_sectors = sectors;
    part_offset = 0;
    part_scanned = 0;
    fs_disk_write_enabled = 0;
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
    return count > 0 && sector < disk_sectors && count <= disk_sectors - sector;
}

DRESULT disk_read(BYTE drive, BYTE *buffer, LBA_t sector, UINT count)
{
    if (disk_status(drive)) return RES_NOTRDY;
    if (!part_scanned) {
        scan_partition();
        if (!part_scanned) return RES_ERROR;   /* 物理扇区 0 读失败，下次重扫 */
    }
    if (!buffer || !valid_range(sector + part_offset, count)) return RES_PARERR;
    if (wait_transfer() != RES_OK) return RES_ERROR;
    if (transfer_dma(buffer,sector+part_offset,count,0) != RES_OK) {
        fs_disk_last_error = HAL_SD_GetError(&hsd1);
        return RES_ERROR;
    }
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
    if (!buffer || !sector || !valid_range(sector + part_offset, count)) return RES_PARERR;
    if (wait_transfer() != RES_OK) return RES_ERROR;
    if (transfer_dma((BYTE *)buffer,sector+part_offset,count,1) != RES_OK) {
        fs_disk_last_error = HAL_SD_GetError(&hsd1);
        return RES_ERROR;
    }
    ++fs_disk_writes; fs_disk_write_sectors += count;
    return wait_transfer();
}

DRESULT disk_ioctl(BYTE drive, BYTE command, void *buffer)
{
    if (disk_status(drive)) return RES_NOTRDY;
    switch (command) {
    case CTRL_SYNC: return wait_transfer();
    case GET_SECTOR_COUNT:
        if (!buffer) return RES_PARERR;
        if (!part_scanned) scan_partition();
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
    DRESULT r = transfer_dma(sd_bounce, 0, 1, 0);
    info[6] = (uint32_t)r;
    info[7] = fs_disk_last_error;
    memcpy(&info[8], sd_bounce, 16);
    info[12] = sd_bounce[510] | ((uint32_t)sd_bounce[511] << 8);
    return r == RES_OK ? 0 : 1;
}
