#ifndef DART_GREEN_FILTER_H
#define DART_GREEN_FILTER_H
#include <stdint.h>
#include <stddef.h>

/* 图示EKF：状态[x,y,vx,vy,s,ds]，观测[x,y,A]，A=K*s*s。
 * 无实物尺寸/距离标定时K=首次面积，s=1表示首次观测时的相对距离倒数。
 * 只有用已知距离r标定K=A*r*r后，s才具有1/m的物理单位。 */
#ifndef GREEN_FILTER_CALIBRATED_K
#define GREEN_FILTER_CALIBRATED_K 0.0f
#endif
#define GREEN_FILTER_MAX_DT_US 250000u
#define GREEN_FILTER_LOST_US 500000u
#define GREEN_FILTER_NIS_LIMIT 25.0f
#define GREEN_FILTER_SIGMA_X 2.0f
#define GREEN_FILTER_SIGMA_Y 2.0f
#define GREEN_FILTER_SIGMA_AREA 20.0f
#define GREEN_FILTER_S_MIN 0.001f
#define GREEN_FILTER_S_MAX 1000.0f
enum { GF_X,GF_Y,GF_VX,GF_VY,GF_S,GF_DS,GF_DIM };
enum { GF_WAIT,GF_INIT,GF_UPDATE,GF_PREDICT,GF_REJECT,GF_LOST,GF_REINIT,
       GF_TIME_INVALID,GF_NUMERIC_RESET,GF_STATUS_COUNT };

typedef struct {
    float state[GF_DIM], covariance[GF_DIM][GF_DIM], area_scale;
    uint32_t last_us, last_update_us, active;
} green_filter;
typedef struct {
    uint32_t status, valid, measurement_valid, dt_us;
    float predicted[GF_DIM], state[GF_DIM], variance[GF_DIM];
    float area_scale, nis; /* nis=-1表示未执行观测门限计算 */
} green_filter_result;

void green_filter_reset(green_filter *filter);
/* 时间使用相机帧完成时间，uint32微秒；不使用处理开始时间或固定帧率。
 * missing/非法观测仅预测；超过丢失上限失效，后续观测重新初始化。
 * 返回状态见枚举，不更改检测结果/ROI/标注。单实例仅由主循环调用。 */
void green_filter_step(green_filter *filter, uint32_t time_us, int observed,
                       float x, float y, float area, green_filter_result *result);

/* 每条120字节，600条放AXI，占72000字节，停止采集后格式化导出。 */
#define GREEN_FILTER_LOG_CAPACITY 600u
#define GREEN_FILTER_CSV_LINE_CAPACITY 1152u
typedef struct {
    uint32_t time_us, avi_index, elapsed_us;
    float measured[3];
    green_filter_result estimate;
} green_filter_row;
_Static_assert(sizeof(green_filter_row)==120,"Filter log RAM budget changed");
typedef struct {
    uint32_t attempts,stored,dropped,max_us,status_count[GF_STATUS_COUNT];
    uint64_t sum_us;
    green_filter_row rows[GREEN_FILTER_LOG_CAPACITY];
} green_filter_log;
void green_filter_log_reset(green_filter_log *log);
void green_filter_log_append(green_filter_log *log,const green_filter_row *row);
extern const char green_filter_csv_header[];
int green_filter_csv_line(char *buffer, size_t capacity, uint32_t attempt,
                          const green_filter_row *row);
#endif
