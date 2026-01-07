/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "AD7175.h"
#include "CAT9555.h"
#include "M24256.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum Compare
{
  STRCMP, // khong tham so
  STRSTR  // co tham so
};
char usb_buf[64];
char aRXbuff[48];
char aCopyBuff[48];
#define EEPROM_ADDR 0x50 << 1
char vReceive = 0;
uint8_t fReceive_ok = 0; // This flag is set on USART0 Receiver buffer overflow
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
#define SDRAM_SIZE (200 * 1024)
float sdram_buffer[SDRAM_SIZE] __attribute__((section(".sdram_data")));
volatile uint32_t sdram_index = 0;
volatile uint8_t recording = 0;
uint32_t record_start_tick = 0;
volatile uint8_t spi_done_flag = 0;
volatile uint8_t adc_start_read = 0;
void Hard_Reset_SRAM_Chip(void)
{
  memset((void *)0x60000000, 0, 2 * 1024 * 1024);
}
void CDC_Print(char *str)
{
  CDC_Transmit_FS((uint8_t *)str, strlen(str));
  HAL_Delay(2); // rất quan trọng, tránh full buffer
}
void DeleteRXBuff(void);
typedef void (*CommandHandler)(void);
typedef struct CommandNode
{
  const char *command;
  CommandHandler handler;
  uint8_t use_strstr;
  struct CommandNode *next;
} CommandNode;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t ad7175_cmd_buffer[5];
uint8_t ad7175_rx_buffer[5];
#define HASH_TABLE_SIZE 521
CommandNode *commandHashTable[HASH_TABLE_SIZE];
// Hash function
uint16_t HashFunction(const char *str)
{
  uint32_t hash = 0;
  while (*str && *str != ':') // Dừng khi gặp ':' hoặc null
  {
    hash ^= (*str);          // XOR ký tự hiện tại với giá trị hash
    hash *= 167;             // Nhân để tăng phân tán
    hash %= HASH_TABLE_SIZE; // Lấy dư để phù hợp kích thước bảng
    str++;
  }
  return hash;
}
// Add command to hash table
void AddCommandToHashTable(const char *command, CommandHandler handler, uint8_t use_strstr)
{
  uint8_t hashIndex = HashFunction(command);
  CommandNode *newNode = (CommandNode *)malloc(sizeof(CommandNode));
  // newNode->command = strdup(command);
  newNode->command = command;
  newNode->handler = handler;
  newNode->use_strstr = use_strstr;
  newNode->next = commandHashTable[hashIndex];
  commandHashTable[hashIndex] = newNode;
}

// Process command
void ProcessCommand(const char *input)
{
  uint8_t hashIndex = HashFunction(input);
  CommandNode *node = commandHashTable[hashIndex];
  while (node != NULL)
  {
    if ((node->use_strstr && strstr(input, node->command) != NULL) ||
        (!node->use_strstr && strcmp(input, node->command) == 0))
    {
      node->handler();
      DeleteRXBuff();
      fReceive_ok = 0;
      return;
    }
    node = node->next;
  }

  // ERR CMD
  printf("cmd error\r\n");
  DeleteRXBuff();
  fReceive_ok = 0;
}

void I2cScan(void)
{
  printf("start scan\n");
  for (uint8_t i = 0; i < 128; i++)
  {
    uint8_t addr = (i << 1);
    if (HAL_I2C_IsDeviceReady(&hi2c1, addr, 1, HAL_MAX_DELAY) == HAL_OK)
    {
      printf("addres see 0x%02x\n", addr);
    }
  }
  printf("done scan\n");
  float a = M24256ReadNumber(&hi2c1, EEPROM_ADDR, 8, 0);
  printf("gia tri a %f\n", a);
}
void Read()
{
	char ch_info = 0, error_status = 0;
		float Voltage_2_3Dmm = 0;
		for (int i = 0; i < 2; i++)
			{
				HAL_Delay(50);
				ad717x_get_code_and_error_status(&hspi2, CS_PD_GPIO_Port, CS_PD_Pin, 0x04,
												 &ch_info, &error_status, &Voltage_2_3Dmm);
			}

		char usb_buf[64];
		int len = snprintf(usb_buf, sizeof(usb_buf),
		                   "V=%.6f\r\n", Voltage_2_3Dmm);

		CDC_Transmit_FS((uint8_t*)usb_buf, len);
}
void AD7175_Start_DMA_Read(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_PORT, uint16_t CS_PIN)
{
	ad7175_cmd_buffer[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_RD | AD717X_COMM_REG_RA(AD717X_DATA_REG);

	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(hspi, ad7175_cmd_buffer, ad7175_rx_buffer, 4);
}
void Record()
{
	 sdram_index = 0;
	 recording = 1;
	 record_start_tick = HAL_GetTick();
	AD7175_Start_DMA_Read(&hspi2,CS_PD_GPIO_Port,CS_PD_Pin);
}
void StopRecord(void)
{
    recording = 0;
}
void AD7175_Process(void)
{
    if (!spi_done_flag) return;
    spi_done_flag = 0;

//    HAL_GPIO_WritePin(CS_PD_GPIO_Port, CS_PD_Pin, GPIO_PIN_SET);

    if (!recording) return;

    uint32_t data =
           ((uint32_t)ad7175_rx_buffer[1] << 16) |
           ((uint32_t)ad7175_rx_buffer[2] << 8) |
           ad7175_rx_buffer[3];

       data &= 0xFFFFFF;

       if (data != 0 && data != 0xFFFFFF)
          {
              float voltage = ((int32_t)data - 0x800000) * 5000.0f /
                              (1.5f * 0x555180);

              if (sdram_index < SDRAM_SIZE)
                  sdram_buffer[sdram_index++] = voltage;
          }

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
    	spi_done_flag=1;
//    	AD7175_Process();
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	  if (GPIO_Pin == EXT_Pin)
	    {
	        adc_start_read = 1;
	    }
}
void High()
{
}
void Low()
{
}
void ID_ADC_BAT(void)
{

  uint16_t id = GetChipID(&hspi2, CS_PD_GPIO_Port, CS_PD_Pin);
  printf("Chip ID : %u\r\n", id);
}
void DeleteRXBuff(void)
{
  memset(aRXbuff, 0, sizeof(aRXbuff));
}
void SendRecordToUSB(void)
{
		  for (uint32_t i = 0; i < sdram_index; i++)   // gửi thử 1000 mẫu đầu
		  {
		      int len = snprintf(usb_buf, sizeof(usb_buf),
		                         "%lu,%.3f\r\n", i, sdram_buffer[i]);

		      while (CDC_Transmit_FS((uint8_t*)usb_buf, len) == USBD_BUSY);
		      HAL_Delay(0.2);
		  }
}

void InitCommandHashTable()
{
  AddCommandToHashTable("ADC", ID_ADC_BAT, STRCMP);
  AddCommandToHashTable("Record", Record, STRCMP);
  AddCommandToHashTable("H", High, STRCMP);
  AddCommandToHashTable("L", Low, STRCMP);
  AddCommandToHashTable("I2C", I2cScan, STRCMP);
  AddCommandToHashTable("Read", Read, STRCMP);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  MX_I2C1_Init();
  MX_FSMC_Init();
  /* USER CODE BEGIN 2 */
  Hard_Reset_SRAM_Chip();
  InitCommandHashTable();
  float number1 = 5;
  M24256WriteNumber(&hi2c1, EEPROM_ADDR, 8, 0, number1);
  CAT9555_init(&hi2c1, 0x21);
  CAT9555_wt_2_byte(&hi2c1, 0x21, 0xC700); // reset cat for BATV2
  AD7175_Setup1(&hspi2, CS_PD_GPIO_Port, CS_PD_Pin);
  ad717xRegisterSet(&hspi2, CS_PD_GPIO_Port, CS_PD_Pin, AD717X_CHMAP1_REG, 2, 0x1043);
  ad717xRegisterSet(&hspi2, CS_PD_GPIO_Port, CS_PD_Pin, AD717X_CHMAP0_REG, 2, 0x0001);
  ad717xRegisterSet(&hspi2, CS_PD_GPIO_Port, CS_PD_Pin, AD717X_CHMAP1_REG, 2, 0x9043);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (adc_start_read && hspi2.State == HAL_SPI_STATE_READY)
	  {
	      adc_start_read = 0;
	      HAL_SPI_TransmitReceive_DMA(&hspi2,
	                                     ad7175_cmd_buffer,
	                                     ad7175_rx_buffer,
	                                     4);
	  }
	  AD7175_Process();

    if (fReceive_ok == 1)
    {
      ProcessCommand(aRXbuff);
    }
    if (recording)
    {
        if (HAL_GetTick() - record_start_tick >= 2000)
        {
            StopRecord();
            SendRecordToUSB();
        }
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
  uint8_t buf[1] = {ch};
  CDC_Transmit_FS(buf, 1);
  return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
