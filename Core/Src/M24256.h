/*
 * M24256.h
 *
 *  Created on: Jan 6, 2026
 *      Author: Bacnk
 */

#ifndef SRC_M24256_H_
#define SRC_M24256_H_
#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"
#include <stddef.h>
#include "stm32f4xx_hal.h"
    void M24256Write(I2C_HandleTypeDef *hi2c, uint8_t addr, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
    void M24256Read(I2C_HandleTypeDef *hi2c, uint8_t addr, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
    void M24256PageErase(I2C_HandleTypeDef *hi2c, uint8_t addr, uint16_t page);
    void M24256WriteNumber(I2C_HandleTypeDef *hi2c, uint8_t addr, uint16_t page, uint16_t offset, float fdata);
    float M24256ReadNumber(I2C_HandleTypeDef *hi2c, uint8_t addr, uint16_t page, uint16_t offset);
    void M24256WriteDouble(I2C_HandleTypeDef *hi2c, uint8_t addr, uint16_t page, uint16_t offset, double data);
    double M24256ReadDouble(I2C_HandleTypeDef *hi2c, uint8_t addr, uint16_t page, uint16_t offset);

#ifdef __cplusplus
}
#endif



#endif /* SRC_M24256_H_ */
