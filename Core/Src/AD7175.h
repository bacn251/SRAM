/*
 * AD7175.h
 *
 *  Created on: Jan 5, 2026
 *      Author: Bacnk
 */
#ifndef __AD7175_2_H__
#define __AD7175_2_H__

#include "main.h"
#include <string.h>
#include <stdio.h>
#include "stddef.h"
/* AD717X Register Map */
#define AD717X_COMM_REG        0x00
#define AD717X_STATUS_REG      0x00
#define AD717X_ADCMODE_REG     0x01
#define INTERFACE_MODE_REGISTER  0x02
#define AD717X_REGCHECK_REG   0x03
#define AD717X_DATA_REG       0x04
#define AD717X_GPIOCON_REG    0x06
#define AD717X_ID_REG         0x07
#define AD717X_CHMAP0_REG     0x10
#define AD717X_CHMAP1_REG     0x11
#define AD717X_CHMAP2_REG     0x12
#define AD717X_CHMAP3_REG     0x13
#define AD717X_CHMAP4_REG     0x14
#define AD717X_CHMAP5_REG     0x15
#define AD717X_CHMAP6_REG     0x16
#define AD717X_CHMAP7_REG     0x17
#define AD717X_CHMAP8_REG     0x18
#define AD717X_CHMAP9_REG     0x19
#define AD717X_CHMAP10_REG    0x1A
#define AD717X_CHMAP11_REG    0x1B
#define AD717X_CHMAP12_REG    0x1C
#define AD717X_CHMAP13_REG    0x1D
#define AD717X_CHMAP14_REG    0x1E
#define AD717X_CHMAP15_REG    0x1F
#define AD717X_SETUPCON0_REG  0x20
#define AD717X_SETUPCON1_REG  0x21
#define AD717X_SETUPCON2_REG  0x22
#define AD717X_SETUPCON3_REG  0x23
#define AD717X_SETUPCON4_REG  0x24
#define AD717X_SETUPCON5_REG  0x25
#define AD717X_SETUPCON6_REG  0x26
#define AD717X_SETUPCON7_REG  0x27
#define AD717X_FILTCON0_REG   0x28
#define AD717X_FILTCON1_REG   0x29
#define AD717X_FILTCON2_REG   0x2A
#define AD717X_FILTCON3_REG   0x2B
#define AD717X_FILTCON4_REG   0x2C
#define AD717X_FILTCON5_REG   0x2D
#define AD717X_FILTCON6_REG   0x2E
#define AD717X_FILTCON7_REG   0x2F
#define AD717X_OFFSET0_REG    0x30
#define AD717X_OFFSET1_REG    0x31
#define AD717X_OFFSET2_REG    0x32
#define AD717X_OFFSET3_REG    0x33
#define AD717X_OFFSET4_REG    0x34
#define AD717X_OFFSET5_REG    0x35
#define AD717X_OFFSET6_REG    0x36
#define AD717X_OFFSET7_REG    0x37
#define AD717X_GAIN0_REG      0x38
#define AD717X_GAIN1_REG      0x39
#define AD717X_GAIN2_REG      0x3A
#define AD717X_GAIN3_REG      0x3B
#define AD717X_GAIN4_REG      0x3C
#define AD717X_GAIN5_REG      0x3D
#define AD717X_GAIN6_REG      0x3E
#define AD717X_GAIN7_REG      0x3F

/* Communication Register bits */
#define AD717X_COMM_REG_WEN    				  	(0 << 7)
#define AD717X_COMM_REG_WR     				  	(0 << 6)
#define AD717X_COMM_REG_RD     				  	(1 << 6)
#define AD717X_COMM_REG_RA(x)  				  	((x) & 0x3F)

/* Status Register bits */
#define AD717X_STATUS_REG_RDY      		  	(1 << 7)
#define AD717X_STATUS_REG_ADC_ERR  		  	(1 << 6)
#define AD717X_STATUS_REG_CRC_ERR  		  	(1 << 5)
#define AD717X_STATUS_REG_REG_ERR  		  	(1 << 4)
#define AD717X_STATUS_REG_CH(x)    		  	((x) & 0x0F)

/* ADC Mode Register bits */
#define AD717X_ADCMODE_REG_REF_EN       	(1 << 15)
#define AD717X_ADCMODE_SING_CYC         	(1 << 13)
#define AD717X_ADCMODE_REG_DELAY(x)     	(((x) & 0x7) << 8)
#define AD717X_ADCMODE_REG_MODE(x)      	(((x) & 0x7) << 4)
#define AD717X_ADCMODE_REG_CLKSEL(x)    	(((x) & 0x3) << 2)

/* Interface Mode Register bits */
#define AD717X_IFMODE_REG_ALT_SYNC        (1 << 12)
#define AD717X_IFMODE_REG_IOSTRENGTH      (1 << 11)
#define AD717X_IFMODE_REG_HIDE_DELAY      (1 << 10)
#define AD717X_IFMODE_REG_DOUT_RESET      (1 << 8)
#define AD717X_IFMODE_REG_CONT_READ       (1 << 7)
#define AD717X_IFMODE_REG_DATA_STAT       (1 << 6)
#define AD717X_IFMODE_REG_REG_CHECK       (1 << 5)
#define AD717X_IFMODE_REG_XOR_EN          (0x01 << 2)
#define AD717X_IFMODE_REG_CRC_EN          (0x02 << 2)
#define AD717X_IFMODE_REG_XOR_STAT(x)     (((x) & AD717X_IFMODE_REG_XOR_EN) == AD717X_IFMODE_REG_XOR_EN)
#define AD717X_IFMODE_REG_CRC_STAT(x)     (((x) & AD717X_IFMODE_REG_CRC_EN) == AD717X_IFMODE_REG_CRC_EN)
#define AD717X_IFMODE_REG_DATA_WL16       (1 << 0)

/* GPIO Configuration Register bits */
/*enzo*/
#define AD717X_GPIOCON_REG_OP_EN2_3    	  (1 << 13)//for ad7175-8

#define AD717X_GPIOCON_REG_MUX_IO       	(1 << 12)
#define AD717X_GPIOCON_REG_SYNC_EN      	(1 << 11)
#define AD717X_GPIOCON_REG_ERR_EN(x)    	(((x) & 0x3) << 9)
#define AD717X_GPIOCON_REG_ERR_DAT      	(1 << 8)
#define AD717X_GPIOCON_REG_IP_EN1       	(1 << 5)
#define AD717X_GPIOCON_REG_IP_EN0       	(1 << 4)
#define AD717X_GPIOCON_REG_OP_EN1       	(1 << 3)
#define AD717X_GPIOCON_REG_OP_EN0       	(1 << 2)
#define AD717X_GPIOCON_REG_DATA1       	    (1 << 1)
#define AD717X_GPIOCON_REG_DATA0        	(1 << 0)

#define AD717X_GPIOCON_REG_PDSW          	(1 << 14)

/* Channel Map Register 0-3 bits */
#define AD717X_CHMAP_REG_CH_EN         		(1 << 15)
#define AD717X_CHMAP_REG_SETUP_SEL(x)  		(((x) & 0x7) << 12)
#define AD717X_CHMAP_REG_AINPOS(x)     		(((x) & 0x1F) << 5)
#define AD717X_CHMAP_REG_AINNEG(x)     		(((x) & 0x1F) << 0)

/* Channel Map Register additional bits for AD4111, AD4112 */
#define AD4111_CHMAP_REG_INPUT(x)      		(((x) & 0x3FF) << 0)

/* Setup Configuration Register 0-3 bits */
#define AD717X_SETUP_CONF_REG_BI_UNIPOLAR (1 << 12)
#define AD717X_SETUP_CONF_REG_REF_SEL(x)  (((x) & 0x3) << 4)

/* Setup Configuration Register additional bits for AD7173-8 */
#define AD717X_SETUP_CONF_REG_REF_BUF(x)  (((x) & 0x3) << 10)
#define AD717X_SETUP_CONF_REG_AIN_BUF(x)  (((x) & 0x3) << 8)
#define AD717X_SETUP_CONF_REG_BURNOUT_EN  (1 << 7)
#define AD717X_SETUP_CONF_REG_BUFCHOPMAX  (1 << 6)

/* Setup Configuration Register additional bits for AD7172-2, AD7172-4, AD7175-2 */
#define AD717X_SETUP_CONF_REG_REFBUF_P    (1 << 11)
#define AD717X_SETUP_CONF_REG_REFBUF_N    (1 << 10)
#define AD717X_SETUP_CONF_REG_AINBUF_P    (1 << 9)
#define AD717X_SETUP_CONF_REG_AINBUF_N    (1 << 8)

/* Setup Configuration Register additional bits for AD4111, AD4112 */
#define AD4111_SETUP_CONF_REG_REFPOS_BUF  (1 << 11)
#define AD4111_SETUP_CONF_REG_REFNEG_BUF  (1 << 10)
#define AD4111_SETUP_CONF_REG_AIN_BUF(x)  (((x) & 0x3) << 8)
#define AD4111_SETUP_CONF_REG_BUFCHOPMAX  (1 << 6)

/* Filter Configuration Register 0-3 bits */
#define AD717X_FILT_CONF_REG_SINC3_MAP    (1 << 15)
#define AD717X_FILT_CONF_REG_ENHFILTEN    (1 << 11)
#define AD717X_FILT_CONF_REG_ENHFILT(x)   (((x) & 0x7) << 8)
#define AD717X_FILT_CONF_REG_ORDER(x)     (((x) & 0x3) << 5)
#define AD717X_FILT_CONF_REG_ODR(x)       (((x) & 0x1F) << 0)

/*enzo*/
#define AD717X_ODR0_15K625_SPS	 		  0x06
#define AD717X_ODR0_5K_SPS	 			  0x08


/* ID register mask for relevant bits */
/* AD7175-8 ID */
#define AD7175_8_ID_REG_VALUE   				      0x3CD0


/*****************************************************************************/
/******************* AD717X Constants ****************************************/
/*****************************************************************************/
//#define AD717X_CRC8_POLYNOMIAL_REPRESENTATION 0x07 /* x8 + x2 + x + 1 */

/* Exported typedef --------------------------------------------------------*/
typedef enum {
	AD717X_DISABLE,
	AD717X_USE_CRC,
	AD717X_USE_XOR,
} ad717x_crc_mode;

/*! AD717X register info */
typedef struct {
	int addr;
	int value;
	int size;
} ad717x_st_reg;


extern SPI_HandleTypeDef hspi3;
extern uint32_t code;
extern char ch_info,error_status;

uint32_t ad7175_register_read(SPI_HandleTypeDef *hspi,GPIO_TypeDef *CS_PORT, uint16_t CS_PIN,uint8_t reg_addr, uint8_t byte_num);
uint16_t GetChipID(SPI_HandleTypeDef *hspi,GPIO_TypeDef *CS_PORT, uint16_t CS_PIN);
uint32_t AD7175_RDATA(void);
uint8_t ad717x_reset(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_PORT, uint16_t CS_PIN);
void AD7175_Setup1(SPI_HandleTypeDef *hspi,GPIO_TypeDef *CS_PORT, uint16_t CS_PIN);
float Get_Vol(void);
uint8_t AD7175_WaitReady(SPI_HandleTypeDef *hspi,GPIO_TypeDef *CS_PORT, uint16_t CS_PIN);
uint8_t AD7175_CH(SPI_HandleTypeDef *hspi,GPIO_TypeDef *CS_PORT, uint16_t CS_PIN);// xem kenh nao dang hoat dong
uint32_t ad717xRegisterSet(SPI_HandleTypeDef *hspi,GPIO_TypeDef *CS_PORT, uint16_t CS_PIN,uint8_t reg_addr, uint8_t byte_num, uint32_t code);
float ad717x_get_code_and_error_status(SPI_HandleTypeDef *hspi,GPIO_TypeDef *CS_PORT, uint16_t CS_PIN,uint8_t reg_addr, char *ch_info, char *error_status, float *Voltage);
void ad717x_measure_voltage_unipolar(uint32_t data, float *voltage, uint8_t ch);
void ad717x_measure_voltage_bi_unipolar(uint32_t data, float *voltage, uint8_t ch);



#endif /* SRC_AD7175_H_ */
