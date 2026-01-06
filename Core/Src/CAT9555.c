/*
 * CAT9555.c
 *
 *  Created on: Jan 5, 2026
 *      Author: Bacnk
 */
#include "CAT9555.h"
#include "stdio.h"

/**
 *
 * @brief 	    PCA9555_rd_2_bytes
 * @param [in]  addr IC config address
 * @return 		return the read out 2 bytes
 * @author 		Doan
 * @since 		2023/04/03
 */
uint16_t CAT9555_rd_2_bytes(I2C_HandleTypeDef *hi2c, uint8_t addr)
{
	uint16_t temp = 0;
	uint8_t Buff[2];
	uint8_t cmd = 0x00; // cmd byte read port 1
	if (HAL_I2C_Master_Transmit(hi2c, (addr << 1) | 0x40, &cmd, 1, 10) != HAL_OK)
	{
		return HAL_ERROR;
	}
	if (HAL_I2C_Master_Receive(hi2c, (addr << 1) | 0x41, Buff, 2, 10) != HAL_OK)
	{
		return HAL_ERROR;
	}
	temp = ((uint16_t)Buff[1] << 8) | Buff[0];
	return temp;
}
/**
 *
 * @brief 	    CAT9555_rd_1_bit
 * @param [in]  addr IC config address
 * @param [in]  pin port io read
 * @return 		return the read input 1 BIT
 * @author 		bacnk
 * @since 		2024/10/15
 */
uint8_t CAT9555_rd_1_bit(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t pin)
{
	if (pin > 16 || pin < 1)
	{
		return 0xFF;
	}
	//	uint16_t temp2 = 0;
	uint8_t Buff[2];
	pin = pin - 1;
	uint8_t cmd = (pin < 8) ? CAT9555_INPUT_PORT0_CMD : CAT9555_INPUT_PORT1_CMD;
	pin %= 8;
	HAL_I2C_Master_Transmit(hi2c, 0x40 | (addr << 1), &cmd, 1, 10);
	HAL_I2C_Master_Receive(hi2c, 0x41 | (addr << 1), Buff, 2, 10);
	return (Buff[0] >> pin) & 0x01;
}
/**
 *
 * @brief 		PCA9555_wt_2_bytes
 * @param [in]  addr IC config address
 * @param [in]  wt_data write data byte
 * @return 			void
 * @author 			Doan
 * @since 			2023/04/03
 */
void CAT9555_wt_2_byte(I2C_HandleTypeDef *hi2c, uint8_t addr, uint16_t wt_data)
{
	uint8_t addr_CAT9555 = 0x40 | (addr << 1);
	uint8_t Buff[3];

	Buff[0] = CAT9555_OUTPUT_PORT0_CMD;			// start register
	Buff[1] = (uint8_t)(wt_data & 0xFF);		// Port0
	Buff[2] = (uint8_t)((wt_data >> 8) & 0xFF); // Port1

	HAL_I2C_Master_Transmit(hi2c, addr_CAT9555, Buff, 3, 10);
}
/// @brief
/// @param hi2c
/// @param addr
/// @param bit_num
/// @param wt_data
void CAT9555_wt_a_bit(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t bit_num, uint8_t wt_data)
{
	uint8_t addr_CAT9555 = 0x40 | (addr << 1); // write slave address|write command
	uint16_t temp;
	uint8_t Buff[3] = {0};
	temp = CAT9555_rd_2_bytes(hi2c, addr);
	bit_num = bit_num - 1;
	if (wt_data == 1)
		temp |= 1 << bit_num;
	else if (wt_data == 0)
		temp &= ~(1 << bit_num);
	Buff[0] = CAT9555_OUTPUT_PORT0_CMD;
	Buff[1] = (uint8_t)(temp & 0x00ff);		   // wt_p0//p2
	Buff[2] = (uint8_t)((temp & 0xff00) >> 8); // wt_p0//p2
	HAL_I2C_Master_Transmit(hi2c, addr_CAT9555, Buff, 3, 10);
}
/**
 *
 * @brief 		CAT9555_config_Port
 * @param [in]  addr IC config address
 * @param [in]  cmd cmd byte
 * @param [in]  wt_data write data byte
 * @return 			void
 * @author 			bacnk
 * @since 			18/11/2024
 */
void CAT9555_config_Port(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t cmd, uint8_t wt_data)
{
	const char *port_str = "UNKNOWN";
	HAL_StatusTypeDef ret;
	uint8_t Buff[2];
	Buff[0] = cmd;
	Buff[1] = (uint8_t)(wt_data & 0xFF);
	ret = HAL_I2C_Master_Transmit(hi2c, 0x40 | (addr << 1), Buff, 2, 10);
	if (ret != HAL_OK)
	{
		if (cmd == CAT9555_CONFIG_PORT0_CMD)
			port_str = "PORT0";
		else if (cmd == CAT9555_CONFIG_PORT1_CMD)
			port_str = "PORT1";
		printf("I2C FAIL | CAT9555 | ADDR=0x%02X | CMD=0x%02X (%s) | DATA=0x%02X | HAL=%d | ERR=0x%08lX\r\n",
			   addr,
			   cmd,
			   port_str,
			   wt_data,
			   ret,
			   hi2c->ErrorCode);
	}
}
/**
 *
 * @brief 		CONFIG OUTPUT PORTS
 * @param [in]  addr IC config address
 * @param [in]  wt_data write data byte
 * 1 la input 0 la output
 * @return 			void
 * @author 			bacnk
 * @since 			18/11/2024
 */
uint16_t CAT9555_init(I2C_HandleTypeDef *hi2c, uint8_t addr) // defaut config all output
{

	CAT9555_config_Port(hi2c, addr, CAT9555_CONFIG_PORT0_CMD, 0x00);
	HAL_Delay(1);
	CAT9555_config_Port(hi2c, addr, CAT9555_CONFIG_PORT1_CMD, 0x00);
	HAL_Delay(1);
	return 1;
	if (CAT9555_rd_2_bytes(hi2c, addr) != 0xFFFF)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

