/* 耗时记录回归：容量边界不能破坏相邻内存，错误帧不能从汇总中消失。 */
#include "record_profile.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static struct {
    uint32_t before;
    record_profile profile;
    uint32_t after;
} guarded;

int main(void)
{
    guarded.before = 0x12345678u; guarded.after = 0x9abcdef0u;
    record_profile *p = &guarded.profile;
    record_profile_reset(p);
    record_profile_row row = {0};
    row.capture_us = 1234; row.queue_us = 25;
    row.stage_us[0] = 10; row.stage_us[1] = 20;
    row.stage_us[2] = 30; row.stage_us[3] = 40;
    row.total_us = 100; row.avi_index = 0; row.camera_drops = 8;
    for (unsigned i = 0; i < DET_TIME_COUNT; ++i) row.detection.stage_us[i] = i + 1;
    row.detection.search_x = 17; row.detection.search_y = 25;
    row.detection.search_w = 31; row.detection.search_h = 43;
    row.detection.search_mode = DET_SEARCH_FALLBACK;
    row.filter_us=7;row.total_us+=7;
    assert(record_profile_append(p, &row) == 0);
    record_profile_note_cycles(p, 0, 400);
    assert(p->rows[0].append_cycles == 400);
    row.result = PROFILE_ENCODE_ERROR; row.avi_index = UINT32_MAX;
    row.stage_us[2] = 90; row.stage_us[3] = 0; row.filter_us=11;row.total_us = 131;
    assert(record_profile_append(p, &row) == 1);
    record_profile_note_cycles(p, 1, 600);
    assert(p->attempts == 2 && p->stored == 2 && p->dropped_rows == 0);
    assert(p->result_count[PROFILE_OK] == 1 && p->result_count[PROFILE_ENCODE_ERROR] == 1);
    assert(p->stage_sum[2] == 120 && p->stage_max[2] == 90);
    assert(p->total_sum == 238 && p->total_max == 131 && p->queue_sum == 50);
    assert(p->append_cycle_sum == 1000 && p->append_cycle_max == 600);
    for (unsigned i = 0; i < DET_TIME_COUNT; ++i) {
        assert(p->detection_sum[i] == 2 * (i + 1));
        assert(p->detection_max[i] == i + 1);
    }

    char text[RECORD_PROFILE_CSV_LINE_CAPACITY];
    assert(record_profile_csv_line(text, sizeof(text), 1, &p->rows[0]) > 0);
    assert(!strcmp(text, "1,0,1234,25,10,20,30,40,107,132,8,0,400,1,2,3,4,5,6,7,17,25,31,43,2,7\r\n"));
    assert(record_profile_csv_line(text, sizeof(text), 2, &p->rows[1]) > 0);
    assert(!strcmp(text, "2,,1234,25,10,20,90,0,131,156,8,2,600,1,2,3,4,5,6,7,17,25,31,43,2,11\r\n"));
    unsigned columns = 1;
    for (const char *c = record_profile_csv_header; *c; ++c) if (*c == ',') columns++;
    assert(columns == 19 + DET_TIME_COUNT && !memcmp(record_profile_csv_header, "\xEF\xBB\xBF", 3));
    assert(record_profile_csv_line(text, 4, 2, &p->rows[1]) >= 4);
    assert(text[3] == '\0');

    /* 满表以后仍继续汇总，不写出界，也不把最后一行误当新帧覆盖。 */
    for (uint32_t i = 2; i < RECORD_PROFILE_CAPACITY; ++i)
        assert(record_profile_append(p, &row) == i);
    p->rows[RECORD_PROFILE_CAPACITY - 1].append_cycles = 777;
    for (uint32_t i = 0; i < 19; ++i) {
        uint32_t slot = record_profile_append(p, &row);
        assert(slot == UINT32_MAX);
        record_profile_note_cycles(p, slot, 1000);
    }
    assert(p->stored == RECORD_PROFILE_CAPACITY && p->dropped_rows == 19);
    assert(p->attempts == RECORD_PROFILE_CAPACITY + 19);
    assert(p->result_count[PROFILE_ENCODE_ERROR] == RECORD_PROFILE_CAPACITY + 18);
    assert(p->rows[RECORD_PROFILE_CAPACITY - 1].append_cycles == 777);
    assert(p->detection_sum[DET_TIME_SCORE] == (uint64_t)p->attempts * DET_TIME_COUNT);
    assert(guarded.before == 0x12345678u && guarded.after == 0x9abcdef0u);

    /* 大累计值不允许退化成32位截断；reset必须允许同一上电周期第二个会话。 */
    record_profile_reset(p);
    row.total_us = UINT32_MAX;
    row.stage_us[2] = UINT32_MAX;
    row.detection.stage_us[DET_TIME_AREA] = UINT32_MAX;
    record_profile_append(p, &row); record_profile_append(p, &row);
    assert(p->total_sum == 2ull * UINT32_MAX && p->stage_sum[2] == 2ull * UINT32_MAX);
    assert(p->detection_sum[DET_TIME_AREA] == 2ull * UINT32_MAX);
    record_profile_reset(p);
    assert(p->attempts == 0 && p->stored == 0 && p->total_sum == 0 && p->append_cycle_max == 0);
    for (unsigned i = 0; i < DET_TIME_COUNT; ++i)
        assert(p->detection_sum[i] == 0 && p->detection_max[i] == 0);
    row.result = PROFILE_WRITE_ERROR;
    assert(record_profile_append(p, &row) == 0 && p->result_count[PROFILE_WRITE_ERROR] == 1);
    /* 最长数字行仍必须完整写入导出缓冲。 */
    memset(&row, 0xff, sizeof(row));
    row.avi_index = UINT32_MAX - 1;
    int length = record_profile_csv_line(text, sizeof(text), UINT32_MAX, &row);
    assert(length > 0 && (size_t)length < sizeof(text));
    assert(text[length - 2] == '\r' && text[length - 1] == '\n');
    printf("PASS: capacity/error rows/wide sums/CSV/reset; profile_bytes=%zu\n", sizeof(*p));
    return 0;
}
