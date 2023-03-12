/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "eth.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*commandCallback)(void);
typedef struct
{
	char *command;
	commandCallback callback;
}Command;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NEW_LINE  "\r\n<<  "
#define DELETE "\b \b"
#define TX_BUFF_SIZE 100
#define RX_BUFF_SIZE 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char znak;
char rx_buffer[RX_BUFF_SIZE];
char tx_buffer[TX_BUFF_SIZE];
uint8_t rx_buff_idx;
uint8_t tx_buff_idx;
static volatile uint8_t tx_ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void fillTxBuffer(char *msg);
void uartTransmit(char* msg, uint8_t length, UART_HandleTypeDef *huart);
void executeCmd(char *cmd);
void helpCmd(void);
void ledonCmd(void);
void ledoffCmd(void);
void pingpongCmd(void);
Command cmdArray[] =
{
		{"help", helpCmd,},
		{"ledOn", ledonCmd,},
		{"ledOff", ledoffCmd,},
		{"ping", pingpongCmd,},
};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  HAL_UART_Receive_IT(&huart3,&znak,1);
  //uartPrint(NEW_LINE);
  HAL_UART_Transmit_IT(&huart3,NEW_LINE,8);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  HAL_Delay(500);
  }
    /* USER CODE BEGIN 3 */

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART3)
	{
		tx_ready = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART3)
	{
		HAL_UART_Receive_IT(&huart3,&znak,1);

		if(znak=='\r'||znak=='\n')
		{
			rx_buffer[rx_buff_idx]='\0';
			executeCmd(rx_buffer);
		}
		else
		{
			if(rx_buff_idx<=RX_BUFF_SIZE)
			{
				if(znak=='\b')
				{
					uartTransmit(DELETE,3,&huart3);
					rx_buff_idx--;
					rx_buffer[rx_buff_idx]='/0';
				}
				else
				{
					rx_buffer[rx_buff_idx++]=znak;
					uartTransmit(&znak,1,&huart3);
				}
			}
			else
			{
				fillTxBuffer(NEW_LINE);
				fillTxBuffer(NEW_LINE);
				fillTxBuffer("Command too long");
				fillTxBuffer(NEW_LINE);
				rx_buff_idx=0;
			}
		}

	}
}
void uartTransmit(char* msg, uint8_t length, UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART3)
	{
		if(tx_ready==1)
		{
			tx_ready=0;
			HAL_StatusTypeDef retVal = HAL_UART_Transmit_IT(&huart3,(uint8_t*)msg,length);
			if(retVal== HAL_OK)
			{
				memset(&tx_buffer,'\0',100);
				tx_buff_idx=0;
			}
		}
	}
}
void fillTxBuffer(char *msg)
{
	uint8_t i=0;
	while(*(msg+i)!='\0')
	{
		if(tx_buff_idx>=TX_BUFF_SIZE-1)
		{
			tx_buff_idx = 0;
			memset(tx_buffer,'\0',TX_BUFF_SIZE);
		}
		tx_buffer[tx_buff_idx++]=msg[i++];
	}
	uartTransmit(tx_buffer,TX_BUFF_SIZE,&huart3);
}

void executeCmd(char* cmd)
{
	rx_buff_idx=0;
	uint8_t cmd_found =0;
	for(int i=0; i<sizeof(cmdArray)/sizeof(Command);i++)
	{
		if(strcmp(rx_buffer,cmdArray[i].command)== 0)
		{
			cmdArray[i].callback();
			cmd_found = 1;
		}
	}
	if(cmd_found==0)
	{
		fillTxBuffer(NEW_LINE);
		fillTxBuffer(NEW_LINE);
		fillTxBuffer("Command not found");
		fillTxBuffer(NEW_LINE);

	}
}

void helpCmd(void)
{
	fillTxBuffer(NEW_LINE);
	fillTxBuffer(NEW_LINE);
	fillTxBuffer("Help:");
	fillTxBuffer(NEW_LINE);

	for(int i=0; i<sizeof(cmdArray)/sizeof(Command);i++)
	{
		fillTxBuffer(cmdArray[i].command);
		fillTxBuffer(NEW_LINE);
	}

}
void ledonCmd(void)
{
	fillTxBuffer(NEW_LINE);
	fillTxBuffer(NEW_LINE);
	fillTxBuffer("Turning led ON");
	fillTxBuffer(NEW_LINE);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

}
void ledoffCmd(void)
{
	fillTxBuffer(NEW_LINE);
	fillTxBuffer(NEW_LINE);
	fillTxBuffer("Turning led OFF");
	fillTxBuffer(NEW_LINE);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

}
void pingpongCmd(void)
{
	fillTxBuffer(NEW_LINE);
	fillTxBuffer(NEW_LINE);
	fillTxBuffer("PONG");
	fillTxBuffer(NEW_LINE);

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

#ifdef  USE_FULL_ASSERT
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
