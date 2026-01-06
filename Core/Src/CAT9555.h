/*
 * CAT9555.h
 *
 *  Created on: Jan 5, 2026
 *      Author: Bacnk
 */

#ifndef SRC_CAT9555_H_
#define SRC_CAT9555_H_

#include "main.h"
#define CAT9555_INPUT_PORT0_CMD 0x00
#define CAT9555_INPUT_PORT1_CMD 0x01
#define CAT9555_OUTPUT_PORT0_CMD 0x02
#define CAT9555_OUTPUT_PORT1_CMD 0x03
#define CAT9555_CONFIG_PORT0_CMD 0x06
#define CAT9555_CONFIG_PORT1_CMD 0x07

void CAT9555_wt_2_byte(I2C_HandleTypeDef *hi2c, uint8_t addr, uint16_t wt_data);
void CAT9555Wt1ByteHigh(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t wt_data);
void CAT9555Wt1ByteLow(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t wt_data);
uint16_t CAT9555_rd_2_bytes(I2C_HandleTypeDef *hi2c, uint8_t addr);
uint8_t CAT9555_rd_1_bit(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t pin);
void CAT9555_config_Port(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t cmd, uint8_t wt_data);
void CAT9555_wt_a_bit(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t bit_num, uint8_t wt_data);
uint16_t CAT9555_init(I2C_HandleTypeDef *hi2c, uint8_t addr);
void CAT9555_CONFIG_INPUT(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t pin);


#endif /* SRC_CAT9555_H_ */
