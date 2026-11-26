#ifndef DART_RECORD_PROFILE_H
#define DART_RECORD_PROFILE_H

#include <stdint.h>
#include <stddef.h>
#include "green_detector.h"

/* 被动耗时记录：不含目标位置，不调度任务，也不在采集期间写卡/打印。
 * 600项覆盖10秒、约50fps的采样；超过上限仍统计总量，明确报告丢弃行数。 */
#define RECORD_PROFILE_CAPACITY 600u
#define RECORD_PROFILE_STAGES 4u
enum { PROFILE_OK, PROFILE_PROCESS_ERROR, PROFILE_ENCODE_ERROR, PROFILE_WRITE_ERROR,
       PROFILE_RESULT_COUNT };

typedef struct {
    uint32_t capture_us;             /* 相机帧完成时间，相对触发，非曝光开始 */
    uint32_t queue_us;               /* 帧完成至主循环开始处理的等待时间 */
    uint32_t stage_us[RECORD_PROFILE_STAGES]; /* 预处理、绘图、编码、保存 */
    uint32_t total_us;               /* 单帧处理墙钟时间，含中断抢占 */
    uint32_t avi_index;              /* 未成功保存为UINT32_MAX */
    uint32_t camera_drops;           /* 截至该帧处理结束的相机丢帧累计 */
    uint32_t result;
    uint32_t append_cycles;          /* 仅测RAM追加及汇总统计的CPU周期 */
    dart_detection_timing detection;
} record_profile_row;

typedef struct {
    uint32_t attempts, stored, dropped_rows, result_count[PROFILE_RESULT_COUNT];
    uint64_t stage_sum[RECORD_PROFILE_STAGES], queue_sum, total_sum, append_cycle_sum;
    uint32_t stage_max[RECORD_PROFILE_STAGES], queue_max, total_max, append_cycle_max;
    uint64_t detection_sum[DET_TIME_COUNT];
    uint32_t detection_max[DET_TIME_COUNT];
    record_profile_row rows[RECORD_PROFILE_CAPACITY];
} record_profile;

void record_profile_reset(record_profile *profile);
/* 单一主循环调用，不从中断调用；返回保存行号，容量满时返回UINT32_MAX。 */
uint32_t record_profile_append(record_profile *profile, const record_profile_row *row);
void record_profile_note_cycles(record_profile *profile, uint32_t slot, uint32_t cycles);
/* 仅在采集停止后调用文本格式化；头部为UTF-8 BOM，单位在列名中。 */
extern const char record_profile_csv_header[];
extern const char *const record_profile_detection_names[DET_TIME_COUNT];
/* 23列uint32十进制最大宽度及分隔符、CRLF、NUL的上界。 */
#define RECORD_PROFILE_CSV_LINE_CAPACITY 256u
int record_profile_csv_line(char *buffer, size_t capacity, uint32_t attempt,
                            const record_profile_row *row);

#endif
