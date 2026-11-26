#include "stm32h7xx_hal.h"
#include "jpeg_enc.h"
#include "jpeg_hw.h"
#include "jpeg_mcu.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t clock_us, aborts, init_calls, supplied_size;
static unsigned mode, fail_init, fail_config;
static uint8_t frame[JPEG_FRAME_WIDTH * JPEG_FRAME_HEIGHT * 2u];
static uint8_t *input;
static const uint8_t marker_only[] = {0xff, 0xd8, 0xff, 0xd9, 0, 0, 0};
/* marker_only仅用于测试输出回调和长度，绝不把它声称为有效可解码JPEG。 */

uint32_t board_micros(void) { return clock_us += 3u; }
HAL_StatusTypeDef HAL_JPEG_Init(JPEG_HandleTypeDef *h)
{
    init_calls++;
    if (fail_init) { h->ErrorCode = 11; return HAL_ERROR; }
    h->ErrorCode = 0; return HAL_OK;
}
HAL_StatusTypeDef HAL_JPEG_ConfigEncoding(JPEG_HandleTypeDef *h, JPEG_ConfTypeDef *c)
{
    assert(c->ImageWidth == 320 && c->ImageHeight == 240);
    assert(c->ChromaSubsampling == JPEG_420_SUBSAMPLING);
    if (fail_config) { h->ErrorCode = 12; return HAL_ERROR; }
    h->Conf = *c; return HAL_OK;
}
void HAL_JPEG_ConfigInputBuffer(JPEG_HandleTypeDef *h, uint8_t *in, uint32_t size)
{
    (void)h; input = in; supplied_size = size;
}
void HAL_JPEG_ConfigOutputBuffer(JPEG_HandleTypeDef *h, uint8_t *out, uint32_t size)
{
    (void)h; assert(out && size == 512u);
}
HAL_StatusTypeDef HAL_JPEG_Abort(JPEG_HandleTypeDef *h)
{
    (void)h; aborts++; return HAL_OK;
}
HAL_StatusTypeDef HAL_JPEG_Encode(JPEG_HandleTypeDef *h, uint8_t *in, uint32_t in_size,
                                  uint8_t *out, uint32_t out_size, uint32_t timeout)
{
    assert(in_size == JPEG_MCU_STRIP_BYTES && out_size == 512 && timeout == 150);
    input = in; supplied_size = in_size;
    if (mode == 1) { h->ErrorCode = 13; return HAL_TIMEOUT; }
    if (mode == 2) return HAL_OK;  /* 没有完成回调不能当成功 */
    for (unsigned strip = 0; strip < JPEG_MCU_STRIP_COUNT; ++strip) {
        assert(supplied_size == JPEG_MCU_STRIP_BYTES);
        /* 每条重排缓冲的首像素必须来自对应16行，验证不会重复提交旧条带。 */
        assert(input[0] == (uint8_t)(strip * 16u));
        HAL_JPEG_GetDataCallback(h, in_size);
    }
    assert(supplied_size == 0);
    memcpy(out, marker_only, sizeof(marker_only));
    if (mode == 3) out[0] = 0;  /* 非法SOI */
    HAL_JPEG_DataReadyCallback(h, out, sizeof(marker_only));
    if (mode == 4) {
        /* 输出两块必须累计，第二块若超容量不能回绕覆盖第一块。 */
        HAL_JPEG_DataReadyCallback(h, out, sizeof(marker_only));
    }
    if (mode == 5) h->ErrorCode = 14;  /* HAL返回OK也不能掩盖外设错误码 */
    HAL_JPEG_EncodeCpltCallback(h); return HAL_OK;
}

int main(void)
{
    for (unsigned y = 0; y < JPEG_FRAME_HEIGHT; ++y)
        for (unsigned x = 0; x < JPEG_FRAME_WIDTH; ++x) {
            frame[(y * JPEG_FRAME_WIDTH + x) * 2u] = (uint8_t)y;
            frame[(y * JPEG_FRAME_WIDTH + x) * 2u + 1u] = 128;
        }
    struct { uint32_t before; uint8_t data[16]; uint32_t after; } guard;
    guard.before = 0x12345678; guard.after = 0x9abcdef0;
    assert(jpeg_hw_prepare(75) == 0);
    for (unsigned i = 0; i < 2; ++i) {
        uint32_t n = sizeof(guard.data);
        assert(jpeg_encode_yuv422(frame, 75, guard.data, &n) == 0 && n == 4);
        assert(!memcmp(guard.data, marker_only, 4));
    }
    assert(enc_frames == 2 && enc_bytes_total == 8 && enc_max_frame_bytes == 4);
    uint32_t n = 6;
    assert(jpeg_encode_yuv422(frame, 75, guard.data, &n) == -2 && n == 0);
    assert(jpeg_hw_error == 2);
    mode = 4; n = 10;
    assert(jpeg_encode_yuv422(frame, 75, guard.data, &n) == -2 && n == 0);
    mode = 3; n = sizeof(guard.data);
    assert(jpeg_encode_yuv422(frame, 75, guard.data, &n) == -3 && n == 0);
    mode = 1; n = sizeof(guard.data);
    unsigned old_init = init_calls;
    assert(jpeg_encode_yuv422(frame, 75, guard.data, &n) == -5 && n == 0);
    assert(jpeg_hw_hal_error == 13 && aborts == 1);
    mode = 0; n = sizeof(guard.data);
    assert(jpeg_encode_yuv422(frame, 75, guard.data, &n) == 0 && init_calls == old_init + 1);
    mode = 2; n = sizeof(guard.data);
    assert(jpeg_encode_yuv422(frame, 75, guard.data, &n) == -5 && n == 0 && aborts == 2);
    mode = 5; n = sizeof(guard.data);
    assert(jpeg_encode_yuv422(frame, 75, guard.data, &n) == -5 && n == 0 && aborts == 3);
    assert(jpeg_hw_hal_error == 14);
    fail_init = 1; assert(jpeg_hw_prepare(75) == -4 && jpeg_hw_hal_error == 11);
    fail_init = 0; fail_config = 1;
    assert(jpeg_hw_prepare(75) == -4 && jpeg_hw_hal_error == 12);
    fail_config = 0;
    n = sizeof(guard.data); assert(jpeg_encode_yuv422(NULL, 75, guard.data, &n) == -1 && n == 0);
    assert(jpeg_encode_yuv422(frame, 75, guard.data, NULL) == -1);
    assert(guard.before == 0x12345678 && guard.after == 0x9abcdef0);
    puts("PASS: mock HAL callbacks, all strips, repeat frames, capacity, timeout/reinit, errors");
    puts("NOTE: mocked control flow only; no real hardware JPEG encode performed");
    return 0;
}
