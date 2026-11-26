#ifndef DART_GREEN_DETECTOR_H
#define DART_GREEN_DETECTOR_H
/* 机载绿灯识别：与电脑端 green_light_detector 的白色灯芯配置保持同一套阈值，
 * 输入为 OV5640 直接输出的 YUV422（Y0 Cb Y1 Cr）帧，输出主候选引导灯位置。
 * 本文件在 H750 固件与主机验证程序中共用。 */
#include <stdint.h>

/* 图像固定为 320x240，与相机配置一致；掩膜按此大小静态分配。 */
#define DART_DET_WIDTH  320
#define DART_DET_HEIGHT 240
#define DART_DET_PIXELS ((DART_DET_WIDTH) * (DART_DET_HEIGHT))

/* 阈值：与电脑端 config/rec00038_white_core.yml 完全一致，不改阈值。
 * HSV 使用 OpenCV 8 位语义：H 0..179，S、V 0..255。 */
#define DART_DET_HUE_MIN          35
#define DART_DET_HUE_MAX          95
#define DART_DET_SAT_MIN          45
#define DART_DET_GREEN_V_MIN      140
#define DART_DET_CORE_V_MIN       252
#define DART_DET_CORE_SAT_MAX     35
#define DART_DET_MEDIAN_KERNEL    3
#define DART_DET_SUPPORT_RADIUS   7
/* 面积与形状判据（area 为轮廓多边形面积，圆度=4*pi*area/perimeter^2）。 */
#define DART_DET_AREA_MIN         280
#define DART_DET_AREA_MAX_FRACT   819      /* 0.08*10000，面积上限=像素数*819/10000 */
#define DART_DET_ASPECT_MAX       12       /* 2.4*5，长短边比 */
#define DART_DET_CIRC_MIN         175      /* 0.35*500 */
#define DART_DET_GREEN_PIX_MIN    20
#define DART_DET_GREEN_FRACTION_MIN 800    /* 0.08*10000 */

/* 一次检测的结果；坐标为像素单位，原点在图像左上，X 向右、Y 向下。
 * 质心保留 1/16 像素的定点信息（q16），与电脑版亚像素质心对应。 */
typedef struct {
    int found;          /* 1=检出主候选，0=本帧无目标 */
    int32_t center_x_q16;   /* 质心 X，Q16 定点（像素） */
    int32_t center_y_q16;   /* 质心 Y，Q16 定点（像素） */
    int radius;         /* 黄圈半径=轮廓最远点距离+3，像素 */
    int box_x, box_y;   /* 外接矩形左上角，像素 */
    int box_w, box_h;   /* 外接矩形尺寸，像素 */
    int32_t area_q8;    /* 轮廓多边形面积（像素^2），Q8 定点 */
    int circularity_x500;  /* 圆度*500 */
    int green_fraction_x10000; /* 邻域绿色比例*10000 */
    int32_t score_x10000;  /* 候选得分*10000，仅供调试 */
} dart_detection;

/* 在 YUV422 帧上执行完整检测。返回 0 成功，非 0 参数错误。 */
int green_detect(const uint8_t *yuv422, dart_detection *out);

/* 分阶段墙钟耗时；同一帧多个候选在对应阶段累加，未执行阶段为0。
 * 时钟由调用方提供（微秒、uint32_t回绕），算法本身仍不依赖HAL。 */
enum {
    DET_TIME_INIT, DET_TIME_MASKS, DET_TIME_MEDIAN_GREEN, DET_TIME_MEDIAN_CORE,
    DET_TIME_SCAN, DET_TIME_FLOOD, DET_TIME_CONTOUR, DET_TIME_SHAPE,
    DET_TIME_SUPPORT, DET_TIME_SCORE, DET_TIME_COUNT
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
