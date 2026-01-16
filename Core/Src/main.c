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
#include "i2s.h"
#include "spi.h"
#include "tim.h"
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
volatile uint32_t samples1=0; // Moved to top for safety

char usb_buf[64];
char aRXbuff[48];
char aCopyBuff[48];
#define EEPROM_ADDR 0x50 << 1
char vReceive = 0;
uint8_t fReceive_ok = 0; // This flag is set on USART0 Receiver buffer overflow
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
#define SDRAM_SIZE (1000 * 288)
int32_t sdram_buffer[SDRAM_SIZE] __attribute__((section(".sdram_data")));
volatile uint32_t sdram_index = 0;
volatile uint8_t recording = 0;
uint32_t record_start_tick = 0;
volatile uint8_t spi_done_flag = 0;
volatile uint8_t endtimer = 0;
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
#define SAMPLE_RATE 96000
#define BLOCK_MS 10
#define FRAME_PER_BLOCK (SAMPLE_RATE * BLOCK_MS / 1000)
#define DMA_LEN (FRAME_PER_BLOCK * 4) // 24-bit stereo ghép
volatile uint8_t uart_busy = 0;
uint16_t i2s_dma_buf[10];

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

void StopRecord(void)
{
  HAL_I2S_DMAStop(&hi2s3);
  endtimer=0;
  recording = 0;
}
void StartI2S()
{
//  samples1 = 0;
  printf("start\n");
  HAL_I2S_Receive_DMA(&hi2s3, (uint16_t *)i2s_dma_buf, 10);
  HAL_TIM_Base_Start_IT(&htim3);
//  endtimer = 0;
//  recording = 1;
}

// Flags for main loop processing
//volatile uint8_t process_half = 0;
//volatile uint8_t process_full = 0;
//
//void process_i2s_24bit(uint16_t *buf, uint32_t len)
//{
//  for (uint32_t i = 0; i < len; i += 4)
//  {
//    // Fix: Correct 24-bit MSB extraction
//    int32_t left =
//        ((int32_t)buf[i] << 16) |
//        ((int32_t)buf[i + 1]);
//
//    // Right channel would be at buf[i+2] & buf[i+3]
//
//    // Sign extend 24-bit to 32-bit
//     if (left & 0x80000000)
//       left |= 0xFF000000;
//
//    if (samples1 < SDRAM_SIZE)
//    {
//      sdram_buffer[samples1++] = left;
//    }
//  }
//}
//
//void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
//{
//  process_half = 1;
//}
//
//void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
//{
//  process_full = 1;
//}
//void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s)
//{
//  printf("I2S Error Code: %lu\r\n", hi2s->ErrorCode);
//}
void DeleteRXBuff(void)
{
  memset(aRXbuff, 0, sizeof(aRXbuff));
}
///* OPTIMIZED SEND2 with PING-PONG BUFFERING */
//#define UART_BLOCK 19200             // Increased buffer size for better throughput
//uint8_t uart_tx_buf[2][UART_BLOCK]; // Double buffer (Ping-Pong)

//void Send2()
//{
//  uint32_t total_bytes = samples1 * 3;
//  uint32_t offset = 0;
//  uint8_t buf_idx = 0;
//
//  // Pre-fill the first buffer
//  uint32_t samples = UART_BLOCK / 3;
//  if ((offset + samples * 3) > total_bytes)
//    samples = (total_bytes - offset) / 3;
//
//  for (uint32_t i = 0; i < samples; i++)
//  {
//    int32_t s = sdram_buffer[(offset / 3) + i];
//    uart_tx_buf[buf_idx][3 * i] = (s >> 16) & 0xFF;
//    uart_tx_buf[buf_idx][3 * i + 1] = (s >> 8) & 0xFF;
//    uart_tx_buf[buf_idx][3 * i + 2] = s & 0xFF;
//  }
//
//  // Start first transmission
//  uart_busy = 1;
//  CDC_Transmit_FS((uint8_t *)uart_tx_buf[buf_idx], samples * 3);
//
//  offset += samples * 3;
//  buf_idx = !buf_idx; // Switch to next buffer
//
//  while (offset < total_bytes)
//  {
//    // PREPARE NEXT BUFFER while current one is sending
//    samples = UART_BLOCK / 3;
//    if ((offset + samples * 3) > total_bytes)
//      samples = (total_bytes - offset) / 3;
//
//    for (uint32_t i = 0; i < samples; i++)
//    {
//      int32_t s = sdram_buffer[(offset / 3) + i];
//      // Pack into the "next" buffer (not the one currently transmitting)
//      uart_tx_buf[buf_idx][3 * i] = (s >> 16) & 0xFF;
//      uart_tx_buf[buf_idx][3 * i + 1] = (s >> 8) & 0xFF;
//      uart_tx_buf[buf_idx][3 * i + 2] = s & 0xFF;
//    }
//
//    // WAIT for previous transmission to complete
//    while (uart_busy)
//    {
//      // Optional: Add timeout here if concerned about hanging
//    }
//
//    // SEND current buffer
//    uart_busy = 1;
//    CDC_Transmit_FS((uint8_t *)uart_tx_buf[buf_idx], samples * 3);
//
//    // Advance
//    offset += samples * 3;
//    buf_idx = !buf_idx; // Swap buffers
//  }
//
//  // Wait for the very last packet to finish
//  while (uart_busy)
//    ;
//}
void Send()
{
  StartI2S();
}
uint8_t ch = 'a';
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    endtimer = 1;
    HAL_TIM_Base_Stop_IT(&htim3);
  }
}
void InitCommandHashTable()
{
  AddCommandToHashTable("Send", Send, 0);
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
  MX_I2S3_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  Hard_Reset_SRAM_Chip();
  InitCommandHashTable();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (fReceive_ok == 1)
    {
      ProcessCommand(aRXbuff);
    }
//   if (process_half)
//    {
//      process_half = 0;
////      process_i2s_24bit(&i2s_dma_buf[0], DMA_LEN / 2);
//    }
//    if (process_full)
//    {
//      process_full = 0;
////      process_i2s_24bit(&i2s_dma_buf[DMA_LEN / 2], DMA_LEN / 2);
//    }

    // Race condition removed: Processing is done in the callback
    // (Flags process_half and process_full are no longer used here)

    if (endtimer)
    {
    	  CDC_Transmit_FS(&ch, 1);
    	  endtimer=0;
        // Send2();
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
