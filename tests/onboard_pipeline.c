/* 主机验证工具：逐帧跑"检测 -> 标注 -> JPEG 编码"完整板端管线。
 * 输入：frame_XXXXX.bin（320x240x2 YUYV，由验证脚本从历史录像模拟相机生成）。
 * 输出：det_XXXXX.jpg（机内标注效果）与 centers_onboard.csv（检测结果，
 * 字段带单位）。本工具只用于主机验证，不参与固件编译。 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "green_detector.h"
#include "jpeg_enc.h"

#define FRAME_BYTES (DART_DET_WIDTH * DART_DET_HEIGHT * 2)

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: %s <input_dir> <output_dir> [quality]\n", argv[0]);
        return 1;
    }
    int quality = (argc > 3) ? atoi(argv[3]) : 75;

    static uint8_t frame[FRAME_BYTES];
    static uint8_t jpeg_buf[DART_JPEG_OUT_MAX];
    FILE *csv = NULL;
    int frames = 0, found = 0;

    for (int index = 0; index < 100000; index++) {
        char path[512];
        snprintf(path, sizeof(path), "%s/frame_%05d.bin", argv[1], index);
        FILE *f = fopen(path, "rb");
        if (!f) break;
        size_t got = fread(frame, 1, FRAME_BYTES, f);
        fclose(f);
        if (got != FRAME_BYTES) { fprintf(stderr, "bad frame %d\n", index); return 1; }

        dart_detection det;
        if (green_detect(frame, &det)) { fprintf(stderr, "detect failed\n"); return 1; }
        /* 模拟板内时间：帧号 * 40ms（约 25fps）。 */
        green_annotate(frame, &det, index, (int32_t)(index * 40 * 256));

        uint32_t jpeg_size = DART_JPEG_OUT_MAX;
        if (jpeg_encode_yuv422(frame, quality, jpeg_buf, &jpeg_size)) {
            fprintf(stderr, "encode failed at frame %d\n", index);
            return 1;
        }
        snprintf(path, sizeof(path), "%s/det_%05d.jpg", argv[2], index);
        FILE *o = fopen(path, "wb");
        if (!o) { fprintf(stderr, "cannot write %s\n", path); return 1; }
        fwrite(jpeg_buf, 1, jpeg_size, o);
        fclose(o);

        if (!csv) {
            snprintf(path, sizeof(path), "%s/centers_onboard.csv", argv[2]);
            csv = fopen(path, "wb");
            if (!csv) { fprintf(stderr, "cannot write csv\n"); return 1; }
            fprintf(csv, "\xEF\xBB\xBF帧号,检出标志,质心X_像素,质心Y_像素,半径_像素,面积_像素平方,JPEG_字节\n");
        }
        double cx = det.found ? det.center_x_q16 / 65536.0 : 0.0;
        double cy = det.found ? det.center_y_q16 / 65536.0 : 0.0;
        fprintf(csv, "%d,%d,%.2f,%.2f,%d,%.1f,%lu\n", index, det.found, cx, cy,
                det.found ? det.radius : 0, det.found ? det.area_q8 / 256.0 : 0.0,
                (unsigned long)jpeg_size);
        frames++;
        if (det.found) found++;
    }
    if (csv) fclose(csv);
    printf("frames=%d found=%d max_jpeg=%lu avg_jpeg=%lu\n", frames, found,
           (unsigned long)enc_max_frame_bytes,
           frames ? (unsigned long)(enc_bytes_total / frames) : 0);
    return frames ? 0 : 2;
}
