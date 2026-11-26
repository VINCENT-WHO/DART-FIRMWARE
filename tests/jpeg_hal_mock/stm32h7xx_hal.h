/* 主机边界测试替身，仅检查适配层控制流；不模拟真实JPEG压缩或外设时序。 */
#ifndef TEST_STM32H7XX_HAL_H
#define TEST_STM32H7XX_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t ColorSpace, ChromaSubsampling, ImageHeight, ImageWidth, ImageQuality; } JPEG_ConfTypeDef;
typedef struct { void *Instance; JPEG_ConfTypeDef Conf; uint32_t ErrorCode; } JPEG_HandleTypeDef;
#define JPEG ((void *)(uintptr_t)0x52003000u)
#define JPEG_YCBCR_COLORSPACE 1u
#define JPEG_420_SUBSAMPLING 3u
#define __HAL_RCC_JPGDECEN_CLK_ENABLE() ((void)0)
#define __HAL_RCC_JPGDECRST_FORCE_RESET() ((void)0)
#define __HAL_RCC_JPGDECRST_RELEASE_RESET() ((void)0)
#define __DSB() ((void)0)
HAL_StatusTypeDef HAL_JPEG_Init(JPEG_HandleTypeDef *h);
HAL_StatusTypeDef HAL_JPEG_ConfigEncoding(JPEG_HandleTypeDef *h, JPEG_ConfTypeDef *c);
HAL_StatusTypeDef HAL_JPEG_Encode(JPEG_HandleTypeDef *h, uint8_t *in, uint32_t in_size,
                                  uint8_t *out, uint32_t out_size, uint32_t timeout);
HAL_StatusTypeDef HAL_JPEG_Abort(JPEG_HandleTypeDef *h);
void HAL_JPEG_ConfigInputBuffer(JPEG_HandleTypeDef *h, uint8_t *in, uint32_t size);
void HAL_JPEG_ConfigOutputBuffer(JPEG_HandleTypeDef *h, uint8_t *out, uint32_t size);
void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *h, uint32_t consumed);
void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *h, uint8_t *out, uint32_t size);
void HAL_JPEG_EncodeCpltCallback(JPEG_HandleTypeDef *h);
#endif
