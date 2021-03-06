/**
  ******************************************************************************
  * @file    I2C/I2C_TwoBoards_ComDMA/Src/main.c 
  * @author  MCD Application Team
  * @brief   This sample code shows how to use STM32L4xx I2C HAL API to transmit
  *          and receive a data buffer with a communication process based on
  *          DMA transfer.
  *          The communication is done using 2 Boards.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 Pan Fan</center></h2>
  *
  * 1, data rate = Fs/Na, if Na=8,Fs = 100Hz, data rate = 100/8 = 12.5Hz
  * 2, function AdpdDriverBringUp(),need to re-check
  *   
  *   
  *  
  *  
  *  
  *   
  *   
  *      
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
#include "main.h"
#include "adpd.h"

/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup I2C_TwoBoards_ComDMA
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Uncomment this line to use the board as master, if not it is used as slave */
/* Size of Transmission buffer */
// #define TXBUFFERSIZE      (COUNTOF(aTxBuffer) - 1)
#define RXBUFFERSIZE       32
#define MASTER_BOARD

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 80 MHz */
/* This example use TIMING to 0x00D00E28 to reach 1 MHz speed (Rise time = 120ns, Fall time = 25ns) */
//#define I2C_TIMING      0x00D00E28	//fast plus mode 1MHz
#define I2C_TIMING      0x10801441	//fast mode 400KHz
//#define I2C_TIMING      0x10E0A1E1	//standard mode 100KHz

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;
uint32_t gnAdpdTimeCurVal = 0;
uint8_t gnAdpdDataReady = 0;
/* UART handler declaration */
UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = SET;

/* Buffer used for transmission */
uint8_t Programorder[] = {0x10, 0x00,0x01, 0x18, 0x01, 0x23};  
uint8_t Chipconfigure[] = {0x10, 0x00,0x01}; 
uint8_t readReg[] = {0x12};
uint8_t temp=0;
uint16_t counterx=0;

/* Configuration for ADPD105 */
#define ADPD_SAMPLE_MODE

uint32_t dcfg_org_105[] = {
	0x000100FF,				// enable FIFO interrupt
	0x00020004,				// always drive GPIO0, and high valid
	0x00061F00,				// FIFO 3F = 128bytes, 1F = 64bytes make an interrput 
	0x000B0101,				// GPIO0 FIFO interrput
	0x00113131,
	0x0012000A,			    // sampling rate = 400Hz(0x14) 200Hz(0x28) 100Hz(0x50)	800Hz(0x0A)
	0x00140559,				// PD1-4 connect to slot A&B, LEDx1 enable at slot A, LEDx2 enable at slot B
	0x00150330,				// 0330=8 pulses for average, 0220=4 pulses for average
	0x00181F80,				// SLOTA_CH1_OFFSET need to clear what does this mean!!!
	0x00191F80,				// SLOTA_CH2_OFFSET
	0x001A1F80,				// SLOTA_CH3_OFFSET
	0x001B1F80,				// SLOTA_CH4_OFFSET
	0x001E1F80,				// SLOTB_CH1_OFFSET
	0x001F1F80,				// SLOTB_CH2_OFFSET
	0x00201F80,				// SLOTB_CH3_OFFSET
	0x00211F80,				// SLOTB_CH4_OFFSET
	0x00221030,				// LEDX3 config 40%
	0x00233030,				// LEDX1 config 100%
	0x00243030,				// LEDX2 config 100%
	0x00250659,
	0x00300319,				// slotA:0x00300819,pulse width=20us offset = 20us
	0x00310818,				// pulse period=40us
	0x00350319,				// slotB				
	0x00360818,				// slotB
	0x003921F4,				// SLOTA_AFE: 0x003921F4,???
	0x003B21F4,				// SLOTB_AFE: 0x003921F4,???
	0X003C3006,
	0x00421C36,				// disable TIA gain ??? gain was set in 0x55
	0x00441C36,
	0x004E0040,				// ADC clock = 1MHz
	0xFFFFFFFF,
};

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];
uint16_t RxBuffer[2];
uint16_t nAdpdDataSetSize = 16;	// 16 means for 2slot*4Channles*2Bytes(16bits)=16
uint16_t nAdpdFifoLevelSize = 0;
uint16_t nRetValue = 0;
uint16_t dataIndex=0;
uint8_t readFIFO=0;
uint8_t fifoData[200] = {0};
uint8_t tempFifoData[16] = {0};
uint8_t sendData[200] = {0};
uint8_t sendflag = 0;
uint16_t sendLen = 0;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
void copyData(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
static void EXTI2_IRQHandler_Config(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
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
  
  /* Configure the system clock to 80 MHz */
  SystemClock_Config();

  /* Configure LED2 and LED3*/
  BSP_LED_Init(LED2);
	
  /*## -0- Configure External line 13 (connected to PC.13 pin) in interrupt mode */
  EXTI2_IRQHandler_Config();

  /*##-1- Configure the I2C peripheral ######################################*/
  I2cHandle.Instance              = I2Cx;
  I2cHandle.Init.Timing           = I2C_TIMING;
  I2cHandle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;  
  I2cHandle.Init.OwnAddress1      = I2C_ADDRESS;
  I2cHandle.Init.OwnAddress2      = 0xFF;

  if(HAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Enable the Analog I2C Filter */
  HAL_I2CEx_ConfigAnalogFilter(&I2cHandle,I2C_ANALOGFILTER_ENABLE);

  /* Delay to avoid that possible signal rebound is taken as button release */
  HAL_Delay(50);
  
  /*##-2- Configure the UART peripheral ######################################*/

  UartHandle.Instance            = USARTx;

  UartHandle.Init.BaudRate       = 256000;
  UartHandle.Init.WordLength     = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits       = UART_STOPBITS_1;
  UartHandle.Init.Parity         = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode           = UART_MODE_TX_RX;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }
  /*--------*/
  
  
  LoadDefaultConfig(dcfg_org_105, &I2cHandle);
  if (VerifyDefaultConfig(&I2cHandle, dcfg_org_105) != TRUE)
  {
	Error_Handler();
  }
 
  /* Write standard value of clock registers */
  AdpdDrvRegWrite(0x4B, 0x269A, &I2cHandle);
  AdpdDrvRegWrite(0x4D, 0x005E, &I2cHandle);

  AdpdDriverBringUp(ADPDDrv_4CH_16, ADPDDrv_4CH_16, &I2cHandle);
  
  
  
  /* Infinite loop */  
  while (1)
  {

	if(readFIFO == 1)
	{
		AdpdDrvGetParameter(ADPD_FIFOLEVEL, &nAdpdFifoLevelSize, &I2cHandle);
		sendLen = nAdpdFifoLevelSize;
			
		/* Read the data from the FIFO and print them */
		if (nAdpdFifoLevelSize ==0x40){
			nRetValue = AdpdDrvReadFifoData(&fifoData[0], nAdpdFifoLevelSize, &I2cHandle);

			if (nRetValue == TRUE) {
					copyData(&sendData[0],&fifoData[0],nAdpdFifoLevelSize);
					sendflag = 1;
					readFIFO = 0;
				}
		}
	}
	
	if((sendflag == 1)&&(UartReady == SET))
	{	
		UartReady = RESET; 		
		sendData[sendLen] = 0xAA;
		sendData[sendLen+1] = 0x55;
		if(HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)sendData, sendLen+2)!= HAL_OK)
			{
				Error_Handler();
			}
		sendflag = 0;
	}
	
	}	
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

/**
  * @brief  Configures EXTI lines 10 to 15 (connected to PC.13 pin) in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTI2_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Enable and set EXTI lines 2 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_2)
  {
    /* Toggle LED2 */
	readFIFO = 1;
    BSP_LED_Toggle(LED2);
  }
}

/**
  * @brief  I2C error callbacks.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
  /** Error_Handler() function is called when error occurs.
    * 1- When Slave don't acknowledge it's address, Master restarts communication.
    * 2- When Master don't acknowledge the last data transferred, Slave don't care in this example.
    */
  if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF)
  {
    Error_Handler();
  }
}

/**
  *  @brief  ADPD Driver bring up.
  *  @param  uint8_t nSlotA
  *  @param  uint8_t nSlotB
  *  @retval None
*/
void AdpdDriverBringUp(uint8_t nSlotA, uint8_t nSlotB, I2C_HandleTypeDef *hi2c)
{

	/* Set the slot modes for slot A and slot B */
	AdpdDrvSetSlot(nSlotA, nSlotB, hi2c);

    /* Set the device operation to sample mode. The data can be collected now */
	AdpdDrvSetOperationMode(ADPDDrv_MODE_SAMPLE, hi2c);	// adpd105 was set to RUN in this function
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* LED2 is slowly blinking (1 sec. period) */
  while(1)
  {    
    BSP_LED_Toggle(LED2); 
    HAL_Delay(1000);
  } 
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;

}

void copyData(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    *pBuffer1 = *pBuffer2;
    pBuffer1++;
    pBuffer2++;
  }
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
