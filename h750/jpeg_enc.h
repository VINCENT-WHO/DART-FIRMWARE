#ifndef DART_JPEG_ENC_H
#define DART_JPEG_ENC_H
/* 录像JPEG统一接口：输入YUV422，输出完整Baseline JPEG（4:2:0），写入MJPEG AVI。
 * H750构建链接jpeg_hw.c硬件后端；电脑旧回归仍可单独编译jpeg_enc.c软件参考版。
 * 主机软件参考版测试通过不代表已在真实JPEG外设上验证。 */
#include <stdint.h>

/* JPEG 输出缓冲容量（字节）。质量 75 下 320x240 噪声帧实测最大约 48.4KB，
 * 上限留 25% 余量；超限时整帧编码失败并上抛（记录固件置错误位）。 */
/* 59392 = 48443B（实测最大帧）×1.226 余量；同时为 AXI 段腾出 SD 缓冲空间。 */
#define DART_JPEG_OUT_MAX 59392

/* 编码一帧。
 * yuv422：320x240x2 字节，字节序 Y0 Cb Y1 Cr。
 * quality：1~100，推荐 75。
 * out：输出缓冲，容量至少 DART_JPEG_OUT_MAX。
 * out_size：输入容量，成功返回实际字节数；硬件后端失败时返回长度0。
 * 返回0成功；-1参数错误；-2容量不足；-3硬件输出标记无效；
 * -4硬件初始化失败；-5硬件超时或数据传输不完整。
 * 本接口阻塞当前主循环，串口/相机中断仍可运行。 */
int jpeg_encode_yuv422(const uint8_t *yuv422, int quality,
                       uint8_t *out, uint32_t *out_size);

/* 编码统计（单位：帧、字节、微秒）。由编码器内部维护。 */
extern volatile uint32_t enc_frames, enc_bytes_total, enc_max_us, enc_max_frame_bytes;

#endif
