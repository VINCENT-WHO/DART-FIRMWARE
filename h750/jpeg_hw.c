/* 录像专用硬件JPEG后端：CPU只重排像素和搬运FIFO，变换/量化/熵编码由外设完成。
 * 第一版采用HAL轮询接口，不启动JPEG中断或MDMA，不改变已有相机/串口优先级。
 * 轮询依然占主循环，不能把它称为完全异步或零CPU开销。 */
#include "stm32h7xx_hal.h"
#include "jpeg_enc.h"
#include "jpeg_hw.h"
#include "jpeg_mcu.h"
#include "fs_test.h"
#include <string.h>

extern uint32_t board_micros(void);
static JPEG_HandleTypeDef handle;
/* 条带仅CPU访问，放DTCM；输出直接交给最终JPEG缓冲，scratch只用于溢出排空。 */
static uint8_t input_strip[JPEG_MCU_STRIP_BYTES] __attribute__((aligned(32)));
static uint8_t output_chunk[512] __attribute__((aligned(32)));
static uint8_t *active_output;
static uint32_t active_output_size, output_direct;
static const uint8_t *source_frame;
static uint32_t next_strip, prepared, completed, packing_error;
static jpeg_output_stream output;

volatile uint32_t enc_frames, enc_bytes_total, enc_max_us, enc_max_frame_bytes;
uint32_t jpeg_hw_pack_us, jpeg_hw_poll_us, jpeg_hw_error, jpeg_hw_hal_error;
uint32_t jpeg_hw_last_pack_us, jpeg_hw_last_poll_us;
static uint32_t frame_pack_us;

void HAL_JPEG_MspInit(JPEG_HandleTypeDef *hjpeg)
{
    if (hjpeg->Instance != JPEG) return;
    __HAL_RCC_JPGDECEN_CLK_ENABLE();
    __HAL_RCC_JPGDECRST_FORCE_RESET(); __DSB();
    __HAL_RCC_JPGDECRST_RELEASE_RESET(); __DSB();
}

int jpeg_hw_prepare(int quality)
{
    if (quality < 1 || quality > 100) return -1;
    prepared = 0;
    jpeg_hw_pack_us = jpeg_hw_poll_us = jpeg_hw_error = jpeg_hw_hal_error = 0;
    jpeg_hw_last_pack_us = jpeg_hw_last_poll_us = 0;
    enc_frames = enc_bytes_total = enc_max_us = enc_max_frame_bytes = 0;
    /* 每次会话独立复位外设，避免上次超时/中断后的残留FIFO影响新会话。 */
    memset(&handle, 0, sizeof(handle)); handle.Instance = JPEG;
    if (HAL_JPEG_Init(&handle) != HAL_OK) {
        jpeg_hw_error = 4; jpeg_hw_hal_error = handle.ErrorCode; return -4;
    }
    JPEG_ConfTypeDef config = {0};
    config.ColorSpace = JPEG_YCBCR_COLORSPACE;
    config.ChromaSubsampling = JPEG_420_SUBSAMPLING;
    config.ImageWidth = JPEG_FRAME_WIDTH; config.ImageHeight = JPEG_FRAME_HEIGHT;
    config.ImageQuality = (uint32_t)quality;
    if (HAL_JPEG_ConfigEncoding(&handle, &config) != HAL_OK) {
        jpeg_hw_error = 4; jpeg_hw_hal_error = handle.ErrorCode; return -4;
    }
    prepared = 1; return 0;
}

static int pack_strip(uint32_t strip)
{
    /* 主循环的JPEG轮询期间也推进写卡；不从SD IRQ调用FatFs。
     * 存储服务计入编码外围轮询时间，不混入纯像素重排计时。 */
    if (fs_storage_poll()) return -1;
    uint32_t begin = board_micros();
    int error = jpeg_pack_mcu_strip(source_frame, strip, input_strip, sizeof(input_strip));
    frame_pack_us += board_micros() - begin;
    return error;
}

void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *hjpeg, uint32_t consumed)
{
    if (hjpeg != &handle) return;
    if (consumed != sizeof(input_strip)) packing_error = 1;
    if (!packing_error && next_strip < JPEG_MCU_STRIP_COUNT) {
        if (pack_strip(next_strip++)) packing_error = 1;
        else {
            HAL_JPEG_ConfigInputBuffer(hjpeg, input_strip, sizeof(input_strip));
            return;
        }
    }
    /* 最后一条已耗尽：置0表示没有新数据，不能重复提交最后一条。 */
    HAL_JPEG_ConfigInputBuffer(hjpeg, input_strip, 0);
}

/* HAL轮询输出长度必须为4的倍数。满缓冲后提供scratch继续排空，
 * 只有实际收到超容量数据才报溢出（刚好填满后结束仍然合法）。 */
static void select_output_buffer(void)
{
    uint32_t remaining = output.capacity - output.used;
    output_direct = !output.overflow && !packing_error && remaining >= 4u;
    active_output = output_direct ? output.data + output.used : output_chunk;
    active_output_size = output_direct ? remaining & ~3u : sizeof(output_chunk);
}

void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *hjpeg, uint8_t *data, uint32_t count)
{
    if (hjpeg != &handle) return;
    if (data != active_output || count > active_output_size) {
        packing_error = 1; /* 非预期回调，不能推进最终缓冲指针。 */
    } else if (output_direct) {
        output.used += count; /* HAL已直接写入最终位置，不再memcpy。 */
    } else if (!packing_error && count) {
        jpeg_stream_append(&output, data, count);
    }
    select_output_buffer();
    HAL_JPEG_ConfigOutputBuffer(hjpeg, active_output, active_output_size);
}

void HAL_JPEG_EncodeCpltCallback(JPEG_HandleTypeDef *hjpeg)
{
    if (hjpeg == &handle) completed = 1;
}

int jpeg_encode_yuv422(const uint8_t *yuv422, int quality, uint8_t *out, uint32_t *out_size)
{
    jpeg_hw_last_pack_us = jpeg_hw_last_poll_us = 0;
    if (!out_size) return -1;
    uint32_t capacity = *out_size; *out_size = 0;
    if (!yuv422 || !out || capacity < 4u || quality < 1 || quality > 100) return -1;
    if (!prepared || handle.Conf.ImageQuality != (uint32_t)quality) {
        int result = jpeg_hw_prepare(quality); if (result) return result;
    }
    source_frame = yuv422; next_strip = 1; completed = packing_error = frame_pack_us = 0;
    jpeg_hw_error = jpeg_hw_hal_error = 0;
    jpeg_stream_begin(&output, out, capacity);
    select_output_buffer();
    uint32_t begin = board_micros();
    if (pack_strip(0)) { jpeg_hw_error = 5; return -5; }
    /* 有界超时防止外设异常使记录主循环永久卡住。它不是速度验收值。 */
    HAL_StatusTypeDef status = HAL_JPEG_Encode(&handle, input_strip, sizeof(input_strip),
                                              active_output, active_output_size, 150);
    uint32_t elapsed = board_micros() - begin;
    jpeg_hw_pack_us += frame_pack_us;
    jpeg_hw_poll_us += elapsed - frame_pack_us;
    jpeg_hw_last_pack_us = frame_pack_us;
    jpeg_hw_last_poll_us = elapsed - frame_pack_us;
    if (elapsed > enc_max_us) enc_max_us = elapsed;
    jpeg_hw_hal_error = handle.ErrorCode;
    int result = 0;
    if (status != HAL_OK || handle.ErrorCode || !completed || packing_error ||
        next_strip != JPEG_MCU_STRIP_COUNT) result = -5;
    else result = jpeg_stream_finish(&output, out_size);
    if (result) {
        jpeg_hw_error = (uint32_t)(-result);
        /* 超时后HAL可能只改State而未停止硬件；必须停止并在下次显式重新准备。 */
        if (status != HAL_OK || handle.ErrorCode || !completed || packing_error) {
            (void)HAL_JPEG_Abort(&handle); prepared = 0;
        }
        *out_size = 0; return result;
    }
    enc_frames++; enc_bytes_total += *out_size;
    if (*out_size > enc_max_frame_bytes) enc_max_frame_bytes = *out_size;
    return 0;
}
