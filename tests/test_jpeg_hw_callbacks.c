/* HAL控制流替身：不模拟JPEG压缩或外设时序，只验证缓冲和回调协议。 */
#include "stm32h7xx_hal.h"
#include "jpeg_enc.h"
#include "jpeg_hw.h"
#include "jpeg_mcu.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t clock_us, aborts, init_calls, supplied_size;
static unsigned mode, fail_init, fail_config, callback_count;
static uint8_t frame[JPEG_FRAME_WIDTH * JPEG_FRAME_HEIGHT * 2u];
static uint8_t *input, *configured_output, *expected_output;
static uint32_t configured_size, expected_capacity, payload_size;
static uint8_t payload[1200], guarded[1300];

uint32_t board_micros(void) { return clock_us += 3u; }
int fs_storage_poll(void) { return 0; }
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
    (void)h; assert(out && size >= 4 && !(size & 3u));
    configured_output = out; configured_size = size;
}
HAL_StatusTypeDef HAL_JPEG_Abort(JPEG_HandleTypeDef *h)
{
    (void)h; aborts++; return HAL_OK;
}
HAL_StatusTypeDef HAL_JPEG_Encode(JPEG_HandleTypeDef *h, uint8_t *in, uint32_t in_size,
                                  uint8_t *out, uint32_t out_size, uint32_t timeout)
{
    assert(in_size == JPEG_MCU_STRIP_BYTES && timeout == 150);
    assert(out == expected_output && out_size == (expected_capacity & ~3u));
    input = in; supplied_size = in_size;
    configured_output = out; configured_size = out_size;
    if (mode == 1) { h->ErrorCode = 13; return HAL_TIMEOUT; }
    if (mode == 2) return HAL_OK;
    for (unsigned strip = 0; strip < JPEG_MCU_STRIP_COUNT; ++strip) {
        assert(supplied_size == JPEG_MCU_STRIP_BYTES);
        assert(input[0] == (uint8_t)(strip * 16u));
        HAL_JPEG_GetDataCallback(h, in_size);
    }
    assert(supplied_size == 0);
    if (mode == 6) { /* 非预期回调长度，不实际越界写入。 */
        HAL_JPEG_DataReadyCallback(h, out, out_size + 4);
    } else {
        /* 模拟HAL填满缓冲时回调，末尾不足一缓冲的数据在EOC时回调。
         * 每次回调可切换下一输出地址，检查不会覆盖已写前缀。 */
        uint32_t sent = 0;
        while (sent < payload_size) {
            uint32_t n = payload_size - sent;
            if (n > configured_size) n = configured_size;
            uint8_t *target = configured_output;
            memcpy(target, payload + sent, n);
            sent += n; callback_count++;
            HAL_JPEG_DataReadyCallback(h, target, n);
        }
    }
    if (mode == 5) h->ErrorCode = 14;
    HAL_JPEG_EncodeCpltCallback(h); return HAL_OK;
}

static void make_payload(unsigned bytes, unsigned padding)
{
    assert(bytes <= sizeof(payload) && bytes >= 4 + padding && !(bytes & 3u));
    payload_size = bytes; memset(payload,0x6a,bytes);
    payload[0] = 0xff; payload[1] = 0xd8;
    payload[bytes-padding-2] = 0xff; payload[bytes-padding-1] = 0xd9;
    /* 只测试标记与数据流，不声称它是可解码JPEG。 */
}
static int encode(unsigned capacity, unsigned offset, uint32_t *size)
{
    memset(guarded,0xa5,sizeof(guarded));
    expected_output = guarded + 32 + offset; expected_capacity = capacity;
    callback_count = 0; *size = capacity;
    int result = jpeg_encode_yuv422(frame,75,expected_output,size);
    for (unsigned i = 0; i < 32 + offset; ++i) assert(guarded[i] == 0xa5);
    for (unsigned i = 32 + offset + capacity; i < sizeof(guarded); ++i) assert(guarded[i] == 0xa5);
    return result;
}
int main(void)
{
    for (unsigned y = 0; y < JPEG_FRAME_HEIGHT; ++y)
        for (unsigned x = 0; x < JPEG_FRAME_WIDTH; ++x) {
            frame[(y * JPEG_FRAME_WIDTH + x) * 2u] = (uint8_t)y;
            frame[(y * JPEG_FRAME_WIDTH + x) * 2u + 1u] = 128;
        }
    uint32_t n;
    assert(jpeg_hw_prepare(75) == 0);
    for (unsigned offset = 0; offset < 4; ++offset)
        for (unsigned padding = 0; padding < 4; ++padding)
            for (unsigned capacity = 4; capacity < 21; ++capacity) {
                make_payload(8,padding);
                int result = encode(capacity,offset,&n);
                assert(result == (capacity < 8 ? -2 : 0));
                assert(n == (capacity < 8 ? 0 : 8-padding));
                if (!result) assert(!memcmp(expected_output,payload,8));
            }
    make_payload(4,0); assert(encode(4,0,&n) == 0 && n == 4);
    assert(jpeg_hw_last_pack_us && jpeg_hw_last_poll_us);
    /* 大输出仅EOC回调一次，实际数据已在最终地址。 */
    make_payload(1024,2); assert(encode(1200,0,&n) == 0 && n == 1022);
    assert(callback_count == 1 && !memcmp(expected_output,payload,1024));
    /* 容量满后持续排空scratch，不覆盖已输出前缀。 */
    make_payload(1200,0); assert(encode(12,0,&n) == -2 && n == 0);
    assert(callback_count >= 3 && !memcmp(expected_output,payload,12));
    make_payload(8,1); payload[0] = 0;
    assert(encode(16,0,&n) == -3 && n == 0 && jpeg_hw_error == 3);
    make_payload(8,1);
    mode = 1; unsigned old_init = init_calls;
    assert(encode(16,0,&n) == -5 && n == 0 && jpeg_hw_hal_error == 13 && aborts == 1);
    mode = 0; assert(encode(16,0,&n) == 0 && init_calls == old_init + 1);
    mode = 2; assert(encode(16,0,&n) == -5 && n == 0 && aborts == 2);
    mode = 5; assert(encode(16,0,&n) == -5 && n == 0 && aborts == 3 && jpeg_hw_hal_error == 14);
    mode = 6; assert(encode(16,0,&n) == -5 && n == 0 && aborts == 4);
    mode = 0; assert(encode(16,0,&n) == 0);
    assert(enc_frames == 1 && enc_bytes_total == 7 && enc_max_frame_bytes == 7);
    fail_init = 1; assert(jpeg_hw_prepare(75) == -4 && jpeg_hw_hal_error == 11);
    fail_init = 0; fail_config = 1;
    assert(jpeg_hw_prepare(75) == -4 && jpeg_hw_hal_error == 12);
    fail_config = 0;
    n = 16; assert(jpeg_encode_yuv422(NULL,75,guarded,&n) == -1 && n == 0);
    assert(!jpeg_hw_last_pack_us && !jpeg_hw_last_poll_us);
    assert(jpeg_encode_yuv422(frame,75,guarded,NULL) == -1);
    puts("PASS: direct HAL output/all strips/unaligned buffers/tail padding/exact capacity/overflow drain/reinit/timing");
    puts("NOTE: mock control flow only; no real JPEG peripheral or bitstream validation");
    return 0;
}
