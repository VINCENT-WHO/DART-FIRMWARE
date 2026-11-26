#ifndef TEST_SD_HAL_H
#define TEST_SD_HAL_H
#include <stdint.h>
typedef struct { uint32_t unused; } SD_HandleTypeDef;
typedef enum { HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { HAL_SD_CARD_TRANSFER,HAL_SD_CARD_PROGRAMMING,HAL_SD_CARD_ERROR } HAL_SD_CardStateTypeDef;
#define HAL_SD_ERROR_TIMEOUT 0x80000000u
#define __DMB() ((void)0)
uint32_t HAL_GetTick(void);
uint32_t HAL_SD_GetError(SD_HandleTypeDef *h);
HAL_StatusTypeDef HAL_SD_Abort(SD_HandleTypeDef *h);
void HAL_SD_IRQHandler(SD_HandleTypeDef *h);
HAL_SD_CardStateTypeDef HAL_SD_GetCardState(SD_HandleTypeDef *h);
HAL_StatusTypeDef HAL_SD_WriteBlocks_DMA(SD_HandleTypeDef *h,const uint8_t *data,uint32_t lba,uint32_t count);
HAL_StatusTypeDef HAL_SD_ReadBlocks_DMA(SD_HandleTypeDef *h,uint8_t *data,uint32_t lba,uint32_t count);
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *h);
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *h);
void HAL_SD_ErrorCallback(SD_HandleTypeDef *h);
#endif
