#include "record_profile.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>

const char record_profile_csv_header[] = "\xEF\xBB\xBF"
    "处理序号,AVI帧号_从0开始,帧完成相对触发_us,排队_us,图像处理_us,绘图_us,"
    "JPEG编码_us,保存_us,处理总计_us,帧完成至保存_us,相机累计丢帧,结果码,RAM追加_cycles\r\n";

void record_profile_reset(record_profile *profile)
{
    /* 不清空整张表，减少准备时间；stored以内的行在使用前都会重新写入。 */
    memset(profile, 0, offsetof(record_profile, rows));
}

uint32_t record_profile_append(record_profile *profile, const record_profile_row *row)
{
    profile->attempts++;
    if (row->result < PROFILE_RESULT_COUNT) profile->result_count[row->result]++;
    for (unsigned i = 0; i < RECORD_PROFILE_STAGES; ++i) {
        profile->stage_sum[i] += row->stage_us[i];
        if (row->stage_us[i] > profile->stage_max[i]) profile->stage_max[i] = row->stage_us[i];
    }
    profile->queue_sum += row->queue_us;
    profile->total_sum += row->total_us;
    if (row->queue_us > profile->queue_max) profile->queue_max = row->queue_us;
    if (row->total_us > profile->total_max) profile->total_max = row->total_us;
    if (profile->stored >= RECORD_PROFILE_CAPACITY) {
        profile->dropped_rows++;
        return UINT32_MAX;
    }
    uint32_t slot = profile->stored++;
    profile->rows[slot] = *row;
    return slot;
}

void record_profile_note_cycles(record_profile *profile, uint32_t slot, uint32_t cycles)
{
    profile->append_cycle_sum += cycles;
    if (cycles > profile->append_cycle_max) profile->append_cycle_max = cycles;
    if (slot < profile->stored) profile->rows[slot].append_cycles = cycles;
}

int record_profile_csv_line(char *buffer, size_t capacity, uint32_t attempt,
                            const record_profile_row *row)
{
    /* 失败但未入AVI的帧留空帧号，防止把UINT32_MAX误认为真实帧号。 */
    char index[16] = "";
    if (row->avi_index != UINT32_MAX)
        snprintf(index, sizeof(index), "%lu", (unsigned long)row->avi_index);
    return snprintf(buffer, capacity, "%lu,%s,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
        (unsigned long)attempt, index, (unsigned long)row->capture_us,
        (unsigned long)row->queue_us,
        (unsigned long)row->stage_us[0], (unsigned long)row->stage_us[1],
        (unsigned long)row->stage_us[2], (unsigned long)row->stage_us[3],
        (unsigned long)row->total_us, (unsigned long)(row->queue_us + row->total_us),
        (unsigned long)row->camera_drops, (unsigned long)row->result,
        (unsigned long)row->append_cycles);
}
