#pragma once
#include <stdint.h>
void fs_disk_set_ready(uint32_t ready,uint32_t sectors);
extern volatile uint32_t fs_disk_write_enabled,fs_disk_last_error;
extern volatile uint32_t fs_disk_reads,fs_disk_writes,fs_disk_read_sectors,fs_disk_write_sectors;
int fs_disk_probe(uint32_t *info);
/* 单一主循环调用的64KiB写回缓存；IRQ只通知DMA完成，不调用FatFs。
 * poll不循环等待卡就绪；缓存满/未命中读/CTRL_SYNC/end允许有界等待。 */
#define FS_WRITEBACK_BYTES 65536u
typedef struct {
    uint32_t queued_sectors, peak_sectors, enqueued_sectors, completed_sectors;
    uint32_t dma_transfers, full_waits, full_wait_max_us, read_flushes, cache_read_sectors;
    uint32_t dma_max_us, card_busy_max_us, queue_latency_max_us, sync_max_us, error_stage;
    uint64_t dma_us, card_busy_us, full_wait_us;
} fs_writeback_stats;
int fs_storage_begin(void);
int fs_storage_poll(void);
int fs_storage_end(void);
const fs_writeback_stats *fs_storage_stats(void);
void bench_puts(const char *s);
