/*
 * AD7175.c
 *
 *  Created on: Jan 5, 2026
 *      Author: Bacnk
 */

#ifndef SRC_AD7175_C_
#define SRC_AD7175_C_


#include "AD7175.h"
#include "main.h"

uint8_t ad717x_reset(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_PORT, uint16_t CS_PIN)
{
	uint8_t wtBuf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	uint8_t rdBuf[8];
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	while (hspi3.State == HAL_SPI_STATE_BUSY)
		;
	HAL_SPI_TransmitReceive(hspi, wtBuf, rdBuf, 8, 150);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	return 0;
}

//void AD7175_Setup1(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_PORT, uint16_t CS_PIN)
//{
//	/* Initialize ADC mode register */
//	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_ADCMODE_REG, 2, 0x6700); // 500
//	/* Initialize Interface mode register */
//	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, INTERFACE_MODE_REGISTER, 2, 0x0040); // 1060//0x0040
//	/* Initialize GPIO configuration register */
//	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_GPIOCON_REG, 2, 0x0000);
//	/* Initialize Channel Map registers */
//	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_CHMAP2_REG, 2, 0x00);
//	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_CHMAP3_REG, 2, 0x00);
//	/* Initialize Setup Configuration registers */
//	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_SETUPCON0_REG, 2, 0x1F00); // 1f00 enable buffer//1f80
//	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_SETUPCON1_REG, 2, 0x1F00); // 1f00
//	/* Initialize Filter registers */
//	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_FILTCON0_REG, 2, 0x8D14);
//	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_FILTCON1_REG, 2, 0x8D14);
//	// ad717xRegisterSet(0x38,2,0);
//	// ad717xRegisterSet(0x39,2,0);
//	/* Initialize offset registers */
//	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_OFFSET0_REG, 3, 0x800000);
//	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_OFFSET1_REG, 3, 0x800000);
//	/* Initialize gain registers */
//	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_GAIN0_REG, 3, 0x555180);
//	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_GAIN1_REG, 3, 0x555180);
//}
// // AD7175 FOR SPEED MESUREMENT 1000ODR
 void AD7175_Setup1(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_PORT, uint16_t CS_PIN)
 {
 	/* Initialize ADC mode register */
 	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_ADCMODE_REG, 2, 0x0000); // 500
 	/* Initialize Interface mode register */
 	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, INTERFACE_MODE_REGISTER, 2, 0x0040); // 1060//0x0040
 	/* Initialize GPIO configuration register */
 	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_GPIOCON_REG, 2, 0x0000);
 	/* Initialize Channel Map registers */
 	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_CHMAP2_REG, 2, 0x00);
 	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_CHMAP3_REG, 2, 0x00);
 	/* Initialize Setup Configuration registers */
 	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_SETUPCON0_REG, 2, 0x1F00); // 1f00 enable buffer//1f80
 	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_SETUPCON1_REG, 2, 0x1F00); // 1f00
 	/* Initialize Filter registers */
 	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_FILTCON0_REG, 2, 0x200);
 	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_FILTCON1_REG, 2, 0x200);//
 	// ad717xRegisterSet(0x38,2,0);
 	// ad717xRegisterSet(0x39,2,0);
 	/* Initialize offset registers */
 	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_OFFSET0_REG, 3, 0x800000);
 	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_OFFSET1_REG, 3, 0x800000);
 	/* Initialize gain registers */
 	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_GAIN0_REG, 3, 0x555180);
 	ad717xRegisterSet(hspi, CS_PORT, CS_PIN, AD717X_GAIN1_REG, 3, 0x555180);
 }

uint8_t AD7175_WaitReady(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_PORT, uint16_t CS_PIN)
{
	uint8_t ret;
	uint8_t ready;
	ready = ad7175_register_read(hspi, CS_PORT, CS_PIN, 0x00, 1) & 0x80;
	if (ready == 1)
	{
		ret = 1;
	}
	else if (ready == 0)
	{
		ret = 0;
	}
	return ret;
}

uint8_t AD7175_CH(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_PORT, uint16_t CS_PIN)
{
	uint8_t CH;
	CH = ad7175_register_read(hspi, CS_PORT, CS_PIN, 0x00, 1) & 0x03;
	return CH;
}

uint16_t GetChipID(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_PORT, uint16_t CS_PIN)
{
	uint16_t id_value;
	id_value = ad7175_register_read(hspi, CS_PORT, CS_PIN, 0x07, 2);
	return id_value;
}
/**
 * @brief 		ad717x_register_read
 * @param [in]	reg_addr register address
 * @param [in]	byte_num register data number
 * @param [in]	*code adc convert value
 * @return 		void
 * @since 		2023/10/17
 */
uint32_t ad7175_register_read(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_PORT, uint16_t CS_PIN, uint8_t reg_addr, uint8_t byte_num)
{
	uint32_t val = 0;
	uint8_t wt_buf[5] = {0, 0, 0, 0, 0};
	uint8_t rd_buf[5] = {0, 0, 0, 0, 0};
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	wt_buf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_RD | AD717X_COMM_REG_RA(reg_addr);
//	HAL_Delay(1000);
	// wt_buf[0] = 0x40|reg_addr;
	while (hspi->State == HAL_SPI_STATE_BUSY)
		;
	HAL_SPI_TransmitReceive(hspi, wt_buf, rd_buf, byte_num + 1, 150);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	/*Just for ad7175 */
	switch (byte_num)
	{
	case 1:
	{
		val = rd_buf[1];
	}
	break;
	case 2:
	{
		val = rd_buf[1] << 8;
		val += rd_buf[2];
	}
	break;
	case 3:
	{
		val = rd_buf[1] << 16;
		val += rd_buf[2] << 8;
		val += rd_buf[3];
	}
	break;
	default:
	{
		val = 0xFFFFFF;
	}
	break;
	}
	return val;
}
/**
 * @brief 		ad717x_register_set
 * @param [in]	reg_addr register address
 * @param [in]	byte_num register data number
 * @param [in]	*code adc convert value
 * @return 		void
 * @since 		2023/10/17
 */
uint32_t ad717xRegisterSet(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_PORT, uint16_t CS_PIN, uint8_t reg_addr, uint8_t byte_num, uint32_t code)
{
	uint32_t val;
	uint8_t wt_buf[5] = {0, 0, 0, 0, 0};
	uint8_t rd_buf[5] = {0, 0, 0, 0, 0};
	wt_buf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_WR | AD717X_COMM_REG_RA(reg_addr);
	switch (byte_num)
	{
	case 1:
	{
		code &= 0xFF; // Limit to 8 bit
		wt_buf[1] = code;
	}
	break;
	case 2:
	{
		code &= 0xFFFF; // Limit to 16 bit
		wt_buf[1] = code >> 8;
		wt_buf[2] = code % 256; // Enzo need to pxx
	}
	break;
	case 3:
	{
		code &= 0xFFFFFF; // Limit to 24 bit
		wt_buf[1] = code >> 16;
		wt_buf[2] = (code % 65536) >> 8;
		wt_buf[3] = code % 256;
	}
	break;
	default:
	{
	}
	break;
	}
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	while (hspi->State == HAL_SPI_STATE_BUSY)
		;
	HAL_SPI_TransmitReceive(hspi, wt_buf, rd_buf, byte_num + 1, 150);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	val = wt_buf[1] << 8;
	val += wt_buf[2];
	return 0;
}
/**
 * @brief 			ad717x get convert code result & error status
 * @param [in]	*device The device structure.
 * @param [out]	*code The adc convert value
 * @param [out]	*ch_info The adc convert value bind channel info
 * @param [out]	*error_status The adc convert value bind channel error info
 *		@arg 			bit[0:0] 0:HAL_OK 1:REG_ERROR(0x1)
 *		@arg 		  bit[1:1] 0:HAL_OK 1:CRC_ERROR(0x2)
 *		@arg 		  bit[2:2] 0:HAL_OK 1:ADC_ERROR(0x4)
 * @return 			Returns 0 for success or negative error code.
 * @note
 *              Send[0] 0x44(read 0x04) receive[1-3]:0x04(3 bits adc code) receive[4] (status)
 * @since 		2023/10/17
 */
float ad717x_get_code_and_error_status(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_PORT, uint16_t CS_PIN, uint8_t reg_addr, char *ch_info, char *error_status, float *Voltage)
{
	enum
	{
		pos,
		neg
	};
	float data_temp, gain_temp, data;
	uint8_t wt_buf[5] = {0, 0, 0, 0, 0};
	uint8_t rd_buf[5] = {0, 0, 0, 0, 0};
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	wt_buf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_RD | AD717X_COMM_REG_RA(reg_addr);
	while (hspi->State == HAL_SPI_STATE_BUSY)
		;
	HAL_SPI_TransmitReceive(hspi, wt_buf, rd_buf, 5, 200);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	data = rd_buf[1] << 16; // Tóm lai, &x[i] tuong duong voi p+i và x[i] tuong duong voi *(p+i) và p[i].
	data += rd_buf[2] << 8;
	data += rd_buf[3];
	*ch_info = rd_buf[4] & 0x0f;
	*error_status = (rd_buf[4] >> 4) & 0x07;
	// printf("Value (24-bit): 0x%06X\r\n", (unsigned int)data);
	if (*error_status != 0 || data == 0x000000)
	{
		*Voltage = 0.0f;
		return 0.0f;
	}
	// bibolar
	data_temp = data - 0x800000;
	// data_temp=(0.75*Vin*2*Gain)/Vref
	data_temp = data_temp * 5000.0f;
	gain_temp = 1.5 * 0x555180;
	*Voltage = (float)data_temp / gain_temp;
	// UNIBIPOLAR DATA*VREF/3GAIN;
	//  data_temp *= 200;
	//  gain_temp = (3 * 0x555180) / (5000.0f / 100.0f);//	GainTemp = (3 * AD717x_Gain[CH]) / (AD7175_2_VREF / 100);
	//*Voltage = (float)data_temp / gain_temp;
	return 0;
}
/**
 * @brief 			ad717x_measure_voltage_unipolar
 * @param [in]	data data to be convert
 * @param [in]	*voltage convert code to voltage
 * @param [in]	ch @todo channel ?
 * @return 			void
 * @since 		2023/10/17
 */
void ad717x_measure_voltage_unipolar(uint32_t data, float *voltage, uint8_t ch)
{
	uint32_t data_temp, gain_temp;
	if (ch == 0)
	{
		if (data < 0x800000)
		{
			data_temp = 0x800000 - data;
		}
		else
		{
			data_temp = data - 0x800000;
		}
	}
	else
	{
		data_temp = data / 2;
	}
	data_temp *= 200;
	gain_temp = (3 * 0x555bc0) / (5000 / 100); //	GainTemp = (3 * AD717x_Gain[CH]) / (AD7175_2_VREF / 100);
	*voltage = (float)data_temp / gain_temp;
}

/**
 * @brief 			ad717x_measure_voltage_bi_unipolar
 * @param [in]	data data to be convert
 * @param [in]	*voltage convert code to voltage
 * @param [in]	ch @todo channel ?
 * @return 			void
 *@since 		2023/10/17
 */
void ad717x_measure_voltage_bi_unipolar(uint32_t data, float *Voltage, uint8_t ch)
{
	uint32_t data_temp, gain_temp;
	if (ch == 0)
	{
		if (data < 0x800000)
		{
			data_temp = 0x800000 - data;
		}
		else
		{
			data_temp = data - 0x800000;
		}
	}
	else
	{
		data_temp = data / 2;
	}
	data_temp *= 200;
	gain_temp = (3 * 0x555bc0) / (5000 / 100); //	GainTemp = (3 * AD717x_Gain[CH]) / (AD7175_2_VREF / 100);
	*Voltage = (float)data_temp / gain_temp;
}


#endif /* SRC_AD7175_C_ */
