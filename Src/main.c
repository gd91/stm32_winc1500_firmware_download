/**
  ******************************************************************************
  * @file    UART/UART_Printf/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-November-2015
  * @brief   This example shows how to retarget the C library printf function
  *          to the UART.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "main.h"
#include "driver/include/m2m_wifi.h"
#include "conf_winc.h"
#include "stm32f4xx_hal_uart.h"


/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_Printf
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


static void SystemClock_Config(void);
static void Error_Handler(void);

#include "main.h"
#include "common/include/nm_common.h"
#include "bsp/include/nm_bsp.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"
#include "driver/source/nmbus.h"
#include "driver/include/m2m_wifi.h"

__IO ITStatus UartReady = RESET;

/* Buffer used for transmission */
uint8_t aTxBuffer[] = "12345678";
uint8_t bTxBuffer[] = "Buffer compare Ok!!!";

/** UART commands. */
enum nm_usart_event_types {
	USART_PKT_RECEIVED = 0,
	USART_PKT_TRANSMITTED,
	USART_ERROR_ON_RECEPTION,
};

enum nm_usart_cmd_process_states {
	INIT = 0,
	WAIT_SYNC,
	WAITING,
	COLLECTING_HDR,
	COLLECTING_PAYLOAD,
	PROCESSING,
};

typedef struct uart_cmd_hdr_t {
	unsigned long cmd;
	unsigned long addr;
	unsigned long val;
} uart_cmd_hdr;

#define SB_READ_REG                0
#define SB_WRITE_REG               1
#define READ_BUFF               2
#define WRITE_BUFF              3
#define RESET                   4
#define RECONFIGURE_UART        5

#define USART_CMD_HDR_LENGTH    sizeof(uart_cmd_hdr)
#define SPI_TRANSFER_SIZE       512


UART_HandleTypeDef cdc_uart_module;
static uint16_t usart_cmd_recv_buffer[13];
static uint16_t usart_payload_buffer[512];
static uint8_t usart_tx_buffer[16];
static uint8_t serial_command_pending = 0;
static uint8_t usart_pkt_received = 0;
static uint8_t usart_err_on_reception = 0;
static uint8 *uart_cmd_buf;
static uart_cmd_hdr uart_cmd;
static uint16_t schedule_rx_length = 0;
static uint8_t schedule_rx = 0;
static uint16_t *schedule_rx_buffer = NULL;
static uint8_t *usart_pkt = NULL;
static uint8_t usart_prot_handler_status = INIT;
static uint8_t new_state = INIT;
static uint8_t change_state = 0;
static uint8_t uart_reconfigure = 0;

/*
* \brief Interrupt handler for WiFi EXTI GPIO 2. Call callback API.
*
* \param[in] None.
* \retval    None.
*/
void EXTI15_10_IRQHandler(void)
{
   uint16_t GPIO_Pin;

   /* Get GPIO_Pin */
   if (__HAL_GPIO_EXTI_GET_IT(CONF_WINC_SPI_INT_PIN))
   {
       GPIO_Pin = CONF_WINC_SPI_INT_PIN;
   }

   HAL_GPIO_EXTI_IRQHandler(GPIO_Pin);
}


/**
 * @brief  EXTI line detection callback.
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == CONF_WINC_SPI_INT_PIN)
   {
       isr();
   }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (schedule_rx) {
		if (schedule_rx_length == 1) {
			HAL_UART_Receive_IT(huart,(uint8_t *)schedule_rx_buffer,1);			
		} else {
			HAL_UART_Receive_IT(huart,(uint8_t *)schedule_rx_buffer,schedule_rx_length);
		}

		schedule_rx = 0;
	}

	if (change_state) {
		usart_prot_handler_status = new_state;
		change_state = 0;
	}
}

 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	usart_pkt_received = 1;
	usart_pkt = (uint8_t *)(huart->pRxBuffPtr - schedule_rx_length);
	if (change_state) {
		usart_prot_handler_status = new_state;
		change_state = 0;
	}
}

static void nm_usart_send_regval(UART_HandleTypeDef *huart, uint8_t *tx_data, uint16_t length)
{
	uint8_t temp, i, j;
	for (i = 0, j = (length - 1); i < j; i++, j--) {
		temp = tx_data[i];
		tx_data[i] = tx_data[j];
		tx_data[j] = temp;
	}
	HAL_UART_Transmit_IT(huart,(uint8_t *)tx_data,length);	
}

static void nm_usart_protocol_handler(UART_HandleTypeDef *huart, enum nm_usart_event_types event_name)
{
	static uint16_t payload_length = 0;
	uint8 checksum = 0;
	uint16_t uartdata;
	uint8 i;

	switch (usart_prot_handler_status) {
	case INIT:
		if ((event_name == USART_PKT_RECEIVED) && (schedule_rx_length == 1)) {
			if ((usart_pkt[0] == 0x12)) {
				usart_prot_handler_status = WAIT_SYNC;
				usart_cmd_recv_buffer[0] = 0xFF;
				schedule_rx_buffer = &usart_cmd_recv_buffer[0];
				schedule_rx_length = 1;
				schedule_rx = 1;
				uartdata = 0x5B;
				HAL_UART_Transmit_IT(huart,(uint8_t *)&uartdata,1);							
			} else {
				schedule_rx_buffer = &usart_cmd_recv_buffer[0];
				schedule_rx_length = 1;
				schedule_rx = 1;
				uartdata = usart_pkt[0];
				HAL_UART_Transmit_IT(huart,(uint8_t *)&uartdata,1);	
				/* usart_read_job(module,&usart_cmd_recv_buffer[0]); */
			}
		} else {
			usart_cmd_recv_buffer[0] = 0xFF;
			schedule_rx_buffer = &usart_cmd_recv_buffer[0];
			schedule_rx_length = 1;
			schedule_rx = 1;
			uartdata = 0xEA;
			HAL_UART_Transmit_IT(huart,(uint8_t *)&uartdata,1);	
		}

		break;

	case WAIT_SYNC:
		if (event_name == USART_PKT_RECEIVED) {
			if (usart_pkt[0] == 0xA5) {
				uint8 *usart_cmd_recv_buffer_u8 = (uint8 *)&usart_cmd_recv_buffer[0];
				usart_prot_handler_status = WAITING;
				usart_cmd_recv_buffer_u8[4] = 0xFF;
				schedule_rx_length = 1;
				HAL_UART_Receive_IT(huart,(uint8_t *)&usart_cmd_recv_buffer[2],1);				
			} else if (usart_pkt[0] == 0x12) {
				/* UART identification command. */
				uartdata = 0x5B;
				usart_cmd_recv_buffer[0] = 0xFF;
				schedule_rx_buffer = &usart_cmd_recv_buffer[0];
				schedule_rx_length = 1;
				schedule_rx = 1;
				HAL_UART_Transmit_IT(huart,(uint8_t *)&uartdata,1);
			} else {
				if (!uart_reconfigure) {
					uartdata = 0x5A;
					usart_cmd_recv_buffer[0] = 0xFF;
					schedule_rx_buffer = &usart_cmd_recv_buffer[0];
					schedule_rx_length = 1;
					schedule_rx = 1;
					HAL_UART_Transmit_IT(huart,(uint8_t *)&uartdata,1);
				} else {
					schedule_rx_length = 1;
					HAL_UART_Receive_IT(huart,(uint8_t *)&usart_cmd_recv_buffer[0],1);					
				}
			}
		}

		break;

	case WAITING:
		if (event_name == USART_PKT_RECEIVED) {
			usart_prot_handler_status = COLLECTING_HDR;
			uart_cmd_buf = usart_pkt;
			schedule_rx_length = (USART_CMD_HDR_LENGTH - 1);
			HAL_UART_Receive_IT(huart, (uint8_t *)huart->pRxBuffPtr, (USART_CMD_HDR_LENGTH - 1));			
		} else {
			usart_prot_handler_status = WAIT_SYNC;
			schedule_rx_buffer = &usart_cmd_recv_buffer[0];
			schedule_rx = 1;
			schedule_rx_length = 1;
			uartdata = 0xEA;
			HAL_UART_Transmit_IT(huart,(uint8_t *)&uartdata,1);
		}

		break;

	case COLLECTING_HDR:
		if (event_name == USART_PKT_RECEIVED) {
			/* Verify checksum. */
			for (i = 0; i < (USART_CMD_HDR_LENGTH); i++) {
				checksum ^= *(((uint8_t *)uart_cmd_buf) + i);
			}
			if (checksum != 0) {
				usart_prot_handler_status = WAIT_SYNC;
				usart_cmd_recv_buffer[0] = 0xFF;
				schedule_rx_buffer = &usart_cmd_recv_buffer[0];
				schedule_rx_length = 1;
				schedule_rx = 1;
				uartdata = 0x5A;
				HAL_UART_Transmit_IT(huart,(uint8_t *)&uartdata,1);
			} else {
				memcpy(&uart_cmd, uart_cmd_buf, sizeof(uart_cmd_hdr));
				/* Process the Command. */
				if ((uart_cmd.cmd & 0xFF) == WRITE_BUFF) {
					usart_prot_handler_status = COLLECTING_PAYLOAD;
					payload_length = (uart_cmd.cmd >> 16) & 0xFFFF;
					schedule_rx = 1;
					schedule_rx_buffer = &usart_payload_buffer[0];
					schedule_rx_length = payload_length;
					uartdata = 0xAC;
					HAL_UART_Transmit_IT(huart,(uint8_t *)&uartdata,1);
				} else if ((uart_cmd.cmd & 0xFF) == SB_WRITE_REG) {
					serial_command_pending = 1;
					usart_prot_handler_status = PROCESSING;
				} else {
					serial_command_pending = 1;
					change_state = 1;
					new_state = PROCESSING;
					uartdata = 0xAC;
					HAL_UART_Transmit_IT(huart,(uint8_t *)&uartdata,1);
				}
			}
		} else if (event_name == USART_ERROR_ON_RECEPTION) {
			usart_prot_handler_status = WAIT_SYNC;
			schedule_rx_buffer = &usart_cmd_recv_buffer[0];
			schedule_rx = 1;
			schedule_rx_length = 1;
			uartdata = 0xEA;
			HAL_UART_Transmit_IT(huart,(uint8_t *)&uartdata,1);
		}

		break;

	case COLLECTING_PAYLOAD:
		if ((event_name == USART_PKT_RECEIVED) && (schedule_rx_length == payload_length)) {
			serial_command_pending = 1;
			usart_prot_handler_status = PROCESSING;
		} else if (event_name == USART_ERROR_ON_RECEPTION) {
			usart_prot_handler_status = WAIT_SYNC;
			uartdata = 0xEA;
			usart_cmd_recv_buffer[0] = 0xFF;
			schedule_rx_length = 1;
			schedule_rx_buffer = &usart_cmd_recv_buffer[0];
			schedule_rx = 1;
			HAL_UART_Transmit_IT(huart,(uint8_t *)&uartdata,1);
		} else {
			usart_prot_handler_status = WAIT_SYNC;
			uartdata = 0x5A;
			usart_cmd_recv_buffer[0] = 0xFF;
			schedule_rx_length = 1;
			schedule_rx_buffer = &usart_cmd_recv_buffer[0];
			schedule_rx = 1;
			HAL_UART_Transmit_IT(huart,(uint8_t *)&uartdata,1);
		}

		break;

	default:
		usart_prot_handler_status = WAIT_SYNC;
		break;
	}
}

/**
 * \brief Process input UART command and forward to SPI.
 */
static sint8 enter_wifi_firmware_download(void)
{
	uint16_t uartdata;
	sint8 ret;
	tstrWifiInitParam param;

	ret = m2m_wifi_download_mode();
	
	if (ret != M2M_SUCCESS) {
		puts("Failed to put the WiFi Chip in download mode!\n");
		return M2M_ERR_INIT;
	}

	/* Process UART input command and forward to SPI. */
	while (1) {
		if (usart_pkt_received) {
			usart_pkt_received = 0;
			nm_usart_protocol_handler(&cdc_uart_module, USART_PKT_RECEIVED);
		}

		if (usart_err_on_reception) {
			usart_err_on_reception = 0;
			nm_usart_protocol_handler(&cdc_uart_module, USART_ERROR_ON_RECEPTION);
		}

		if (serial_command_pending && (usart_prot_handler_status == PROCESSING)) {
			uint32_t temp;
			switch ((uart_cmd.cmd) & 0xFF) {
			/* Forward it to SPI. */
			case SB_READ_REG:
				/* Translate it to SPI Read register command. */
				temp = nm_read_reg(uart_cmd.addr);
				usart_tx_buffer[0] = (uint8)(temp >> 0);
				usart_tx_buffer[1] = (uint8)(temp >> 8);
				usart_tx_buffer[2] = (uint8)(temp >> 16);
				usart_tx_buffer[3] = (uint8)(temp >> 24);
				schedule_rx_buffer = &usart_cmd_recv_buffer[0];
				schedule_rx_length = 1;
				schedule_rx = 1;
				usart_prot_handler_status = WAIT_SYNC;
				nm_usart_send_regval(&cdc_uart_module, &usart_tx_buffer[0], sizeof(uint32_t));
				break;

			case SB_WRITE_REG:
				/* Translate it to SPI Write register command. */
				nm_write_reg(uart_cmd.addr, uart_cmd.val);
				schedule_rx_buffer = &usart_cmd_recv_buffer[0];
				schedule_rx_length = 1;
				schedule_rx = 1;
				uartdata = 0xAC;
				usart_prot_handler_status = WAIT_SYNC;
				HAL_UART_Transmit_IT(&cdc_uart_module,(uint8_t *)&uartdata,1);				
				break;

			case READ_BUFF:
				/* Translate it to SPI Read buffer command. */
				nm_read_block(uart_cmd.addr, (uint8 *)&usart_payload_buffer[0], ((uart_cmd.cmd >> 16) & 0xFFFF));
				schedule_rx_buffer = &usart_cmd_recv_buffer[0];
				schedule_rx_length = 1;
				schedule_rx = 1;
				usart_prot_handler_status = WAIT_SYNC;
				HAL_UART_Transmit_IT(&cdc_uart_module, (uint8 *)&usart_payload_buffer[0], ((uart_cmd.cmd >> 16) & 0xFFFF));
				break;

			case WRITE_BUFF:
				/* Translate it to SPI Write buffer command. */
				nm_write_block(uart_cmd.addr, (uint8 *)&usart_payload_buffer[0], ((uart_cmd.cmd >> 16) & 0xFFFF));
				schedule_rx_buffer = &usart_cmd_recv_buffer[0];
				schedule_rx_length = 1;
				schedule_rx = 1;
				uartdata = 0xAC;
				usart_prot_handler_status = WAIT_SYNC;
				HAL_UART_Transmit_IT(&cdc_uart_module,(uint8_t *)&uartdata,1);				
				break;

			case RECONFIGURE_UART:
				/* Send ACK. */
				usart_prot_handler_status = WAIT_SYNC;
				uart_reconfigure = 1;
				HAL_UART_DeInit(&cdc_uart_module);

   			    cdc_uart_module.Instance        = USARTx;   
   			    cdc_uart_module.Init.BaudRate   = uart_cmd.val;
   			    cdc_uart_module.Init.WordLength = UART_WORDLENGTH_8B;
   			    cdc_uart_module.Init.StopBits   = UART_STOPBITS_1;
   			    cdc_uart_module.Init.Parity     = UART_PARITY_NONE; //UART_PARITY_ODD
   			    cdc_uart_module.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
   			    cdc_uart_module.Init.Mode       = UART_MODE_TX_RX;
   			    cdc_uart_module.Init.OverSampling = UART_OVERSAMPLING_16;
   			    if (HAL_UART_Init(&cdc_uart_module) != HAL_OK)
   			    {
   			      /* Initialization Error */
   			      Error_Handler();
   			    }
   			    schedule_rx_length = 1;
   			    HAL_UART_Receive_IT(&cdc_uart_module,(uint8_t *)&usart_cmd_recv_buffer[0],1);
				break;

			default:
				break;
			}
			serial_command_pending = 0;
		}
	}
	return ret;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	tstrWifiInitParam param;
	tstrM2MAPConfig strM2MAPConfig;
	uint8_t mac_addr[6];
	uint8_t u8IsMacAddrValid;
	int8_t ret;
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 180 MHz */
  SystemClock_Config();

  /* Initialize BSP Led for LED2 */


  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits (7 data bit + 1 parity bit) : 
	                  BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
      - Stop Bit    = One Stop bit
      - Parity      = ODD parity
      - BaudRate    = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  cdc_uart_module.Instance        = USARTx;

  cdc_uart_module.Init.BaudRate   = 115200; //9600
  cdc_uart_module.Init.WordLength = UART_WORDLENGTH_8B;
  cdc_uart_module.Init.StopBits   = UART_STOPBITS_1;
  cdc_uart_module.Init.Parity     = UART_PARITY_NONE; //UART_PARITY_ODD
  cdc_uart_module.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  cdc_uart_module.Init.Mode       = UART_MODE_TX_RX;
  cdc_uart_module.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&cdc_uart_module) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  //SET_BIT(USART3->CR1, USART_CR1_IDLEIE);
//  HAL_DMA_MspInit(&cdc_uart_module);
  schedule_rx_length = 1;

  /* The board sends the message and expects to receive it back */

	/* GPIO Ports Clock Enable */
	__GPIOC_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
  /* Initialize the BSP. */
  nm_bsp_init();
  HAL_UART_Receive_IT(&cdc_uart_module,(uint8_t *)&usart_cmd_recv_buffer[0],1);
  /* Enter WiFi firmware download mode. */
  enter_wifi_firmware_download();
  
  return 0;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 on */

  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
