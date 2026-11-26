#ifndef DART_GREEN_DETECTOR_H
#define DART_GREEN_DETECTOR_H
/* 机载RGB绿色区域识别，输入Y0 Cb Y1 Cr；与主机回放共用。 */
#include <stdint.h>
#define DART_DET_WIDTH 320
#define DART_DET_HEIGHT 240
#define DART_DET_PIXELS (DART_DET_WIDTH * DART_DET_HEIGHT)
/* 8位RGB：G阈值后，剔除白灰及没有绿色优势的像素。 */
#define DART_DET_G_MIN 140
#define DART_DET_GREEN_EXCESS_MIN 20  /* G-(R+B)/2 >= 20 */
#define DART_DET_MEDIAN_KERNEL 3
/* 面积为滤波后8连通域的绿色像素数，不含白色孔洞。 */
#define DART_DET_AREA_MIN 40u
#define DART_DET_AREA_MAX (DART_DET_PIXELS * 8u / 100u)

/* 一次检测的结果；坐标为像素单位，原点在图像左上，X 向右、Y 向下。
 * 质心使用16位小数的定点信息（Q16），统计绿色连通域内像素。 */
typedef struct {
    int found;          /* 1=检出主候选，0=本帧无目标 */
    int32_t center_x_q16;   /* 质心 X，Q16 定点（像素） */
    int32_t center_y_q16;   /* 质心 Y，Q16 定点（像素） */
    int radius;         /* 黄圈半径由质心到包围盒最远角点计算，像素 */
    int box_x, box_y;   /* 外接矩形左上角，像素 */
    int box_w, box_h;   /* 外接矩形尺寸，像素 */
    int32_t area_q8;    /* 绿色连通域像素数，Q8 定点 */
    int circularity_x500;  /* 兼容字段，RGB面积策略固定为0 */
    int green_fraction_x10000; /* 兼容字段，RGB面积策略固定为0 */
    int32_t score_x10000;  /* 绿色像素数*10000，仅供调试 */
} dart_detection;

/* 在 YUV422 帧上执行完整检测。返回 0 成功，非 0 参数错误。 */
int green_detect(const uint8_t *yuv422, dart_detection *out);

/* 分阶段墙钟耗时；同一帧多个候选在对应阶段累加，未执行阶段为0。
 * 时钟由调用方提供（微秒、uint32_t回绕），算法本身仍不依赖HAL。 */
enum {
    DET_TIME_INIT, DET_TIME_MASKS, DET_TIME_MEDIAN_GREEN, DET_TIME_SCAN,
    DET_TIME_FLOOD, DET_TIME_AREA, DET_TIME_SCORE, DET_TIME_COUNT
};
typedef struct { uint32_t stage_us[DET_TIME_COUNT]; } dart_detection_timing;
int green_detect_profiled(const uint8_t *yuv422, dart_detection *out,
                          dart_detection_timing *timing, uint32_t (*now_us)(void));

/* 把检测结果显示到 YUV422 帧上：黄圈、红十字、红点与中心坐标文本。
 * found=0 时只在左上角绘制 NOT FOUND。坐标文本单位为像素。 */
void green_annotate(uint8_t *yuv422, const dart_detection *det,
                    int frame_index, int32_t time_ms_q8);

/* 导出调试计数（单位：次）。 */
extern volatile uint32_t det_eval_frames, det_found_frames, det_max_us;
extern volatile uint32_t det_draw_max_us;
void det_reset_stats(void);

#endif
