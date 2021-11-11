/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
  typedef enum {
    WriteSpan_A= 0x00<<8|0X20,//0000 0000 0010 0000
	WriteSpan_B = 0x00 << 8 |0x22, //0000 0000 0010 0010
	WriteSpan_AB = 0x00 << 8 | 0x2E, // 0000 0000 0010 1110
	WriteCode_A = 0x00 <<8 | 0x30, // 0000 0000 0011 0000
	WriteCode_B = 0x00 <<8 |0x32, // 0000 0000 0011 0010
	WriteCode_AB = 0x00 << 8 | 0x3E, //0000 0000 0011 1110
	Update_A = 0x00 << 8 | 0x40, //0000 0000 0100 0000
	Update_B = 0x00 << 8 | 0x42, //0000 0000 0100 0010
	Update_AB = 0x00 << 8 | 0x5E, //0000 0000 0101 1110
	WriteSpan_Update_A = 0x00 << 8 | 0x60, //0000 0000 0110 0000
	WriteSpan_Update_B = 0x00 << 8 | 0x62, //0000 0000 0110 0010
	WriteCode_Update_A = 0x00 << 8 | 0x70, //0000 0000 0111 0000
	WriteCode_Update_B = 0x00 << 8 | 0x72, //0000 0000 0111 0010
	WriteSpan_A_Update_AB = 0x00 << 8 | 0x80, //0000 0000 1000 0000
	WriteSpan_B_Update_AB = 0x00 << 8 | 0x82, //0000 0000 1000 0010
	WriteCode_A_Update_AB = 0x00 << 8 | 0x90, //0000 0000 1001 0000
	WriteCode_B_Update_AB = 0x00 << 8 | 0x92, //0000 0000 1001 0010
	ReadInputSpan_A = 0x00 << 8 | 0xA0, //0000 0000 1010 0000
	ReadInputSpan_B = 0x00 << 8 | 0xA2, //0000 0000 1010 0000
	ReadInputCode_A = 0x00 << 8 | 0xB0, //0000 0000 1011 0000
	ReadInputCode_B = 0x00 << 8 | 0xB2, //0000 0000 1011 0010
	ReadDAC_Span_A = 0x00 << 8 | 0xC0, //0000 0000 1100 0000
	ReadDAC_Span_B = 0x00 << 8 | 0xC2, //0000 0000 1100 0010
	ReadDAC_Code_A = 0x00 << 8 | 0xD0, //0000 0000 1100 0000
	ReadDAC_Code_B = 0x00 << 8 | 0xD2 //0000 0000 1100 0000
  } CommandCodes_LTC2752_16bits;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
  /*---------------------------------------------------------------------*/
  /*  USB SM def                                                    */
  /*---------------------------------------------------------------------*/
#define DAC_MODEL  2758 // OR 2752. 2758:18bit 2752: 16bit
#define SET_OUTPUT_VOLTAGE_1              0x01U
#define SET_OUTPUT_VOLTAGE_2              0x02U

#define SET_SPAN_1						0x03U
#define SET_SPAN_2						0x04U

#define READ_DAC1_REGIS					0X05U
#define READ_DAC2_REGIS					0X06U

#define READ_ADC1					0X07U
#define READ_ADC2					0X09U

#define TRANX_ADC_1_BUFFER					0X0AU

#define TRANX_PID_1_CTRL				0X1AU
#define TRANX_PID_2_CTRL				0X2AU




#define SET_P_1 				0x0CU
#define SET_I_1 				0x0DU
#define SET_D_1 				0x0EU
#define SET_SETPOINT_1 				0x0FU

#define SET_P_2 				0x3CU
#define SET_I_2 				0x3DU
#define SET_D_2 				0x3EU
#define SET_SETPOINT_2 				0x3FU


#define START_PID              0xFFU
#define STOP_PID              0xAFU

#define SM_INQUIRE              0xA3

#define TOGGLE_LEDS_TEST 0xA4

#define MAX_ADC_READ 51200
#define MAX_USB_TX 512
#define MAX_BUFF_CTRL_PID 500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
uint32_t pTxData[4]={WriteCode_Update_A<<16|0x0000,WriteCode_Update_A<<16|0x4000,WriteCode_Update_A<<16|0x7FFF,WriteCode_Update_A<<16|0xFFFF};
uint32_t pRxData[1]={0x00};


uint32_t DAC_command=0x0000;
uint32_t ADC_Data[1]={0x00};//to store 1 value of ADC

uint32_t ADC_1_Data_DB[MAX_ADC_READ];// DB to store ADC 1
uint32_t PID_1_Ctrl_DB[MAX_BUFF_CTRL_PID];// DB to store ADC AND DAC DURING PID 1

uint32_t ADC_2_Data_DB[MAX_ADC_READ];// DB to store ADC 2
uint32_t PID_2_Ctrl_DB[MAX_BUFF_CTRL_PID];// DB to store ADC AND DAC DURING PID 2

//uint32_t ADC_Data_DB_USB[50];// DB to store ADC to transfer during PID
uint32_t DAC_Data_DB[MAX_BUFF_CTRL_PID];// DB to store ADC to transfer during PID

uint16_t Size=1;// size of SPI read
uint8_t timer_mark=0;

uint8_t USB_RX_Buf[10];

/*PID related params*/
uint8_t num_pid_loop=500;
float kp=1.0;
float ki=0.0;
float kd=0.0;

float tpid= 0.00001158;
float one_over_tpid=86355.8;

float k1;
float e;//e(k) current error
float e1; //e(k-1) previous error
float u=0.0;
float delta_u=0.0;
float setpoint_1=0.0;
float setpoint_2 =0.0;

float deri=0;
float integral=0;

//////////////////////

float set_DAC=0.0;
float adc_f=0.0; // float value of adc
float adc_f_2=0.0;
float dac_f=0.0; // float value of dac
float voltage_tosend=0.0; // float value of dac

uint32_t dac_set=0x00; //binary
uint32_t dac_send=0x00;

/*ADC and DAC conversion formula*/
float adc_conv_f=5.0F/2097150.0F;

float dac_conv_16bit = 65535.0F/5.0F;//for 16bit DAC
float dac_conv_18bit = 262143.0F/5.0F;//for 18bit DAC
float dac_conv = 262143.0F/5.0F;

//State machine
uint32_t sm=0;
uint32_t cmd_values=0;
uint8_t cmd_value_1=0;
uint8_t cmd_value_2=0;
uint8_t cmd_value_3=0;
uint8_t cmd_value_4=0;

//serial num
uint32_t sernum=1;


//for troubleshooting
float Vtosend[4]={0.0,0.1,0.2,0.3};
float temp=0.0;
float Vset=0.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef HAL_SPI_TransmitReceive_HM(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout);
//original close transfer
static void SPI_CloseTransfer(SPI_HandleTypeDef *hspi);

//original spi wait on flag
static HAL_StatusTypeDef SPI_WaitOnFlagUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status,
                                                    uint32_t Tickstart, uint32_t Timeout);

//configure for 16bit DAC
HAL_StatusTypeDef HAL_SPI1_TransmitReceive_HM_fast(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);

//configure for LTC2377
HAL_StatusTypeDef ADC_SPI(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint32_t *pRxData, uint16_t Size);

void SPI_Init(SPI_HandleTypeDef *hspi,uint16_t Size);// to initialize SPI
void SET_VOLT_DAC_1(float voltage);// set dac output
void SET_VOLT_DAC_1_bits(uint32_t send_dac_set);// set dac output with input as a bit strings
void SET_SPAN_DAC_1(uint8_t setspan);
void SET_VOLT_DAC_2(float voltage);// set dac output
void SET_VOLT_DAC_2_bits(uint32_t send_dac_set);// set dac output with input as a bit strings
void SET_SPAN_DAC_2(uint8_t setspan);

void set_P(uint8_t* b1);
void set_I(uint8_t* b1);
void set_D(uint8_t* b1);

void read_ADC_1(uint32_t num_run);
void tranx_ADC_1_buffer(uint32_t start_index, uint32_t num_read);//USB transfer of ADC_1_Data_DB
void USB_comm_handle(uint8_t* Buf, uint32_t *Len);// handle USB commands received
void return_sm();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USB_comm_handle(uint8_t* Buf, uint32_t *Len)// handle USB commands received
{
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	cmd_values=0;
	cmd_value_1=Buf[1];
	cmd_value_2=Buf[2];
	cmd_value_3=Buf[3];
	cmd_value_4=Buf[4];

	sm=Buf[0];
	if (*Len>1)
		{
		cmd_values=cmd_value_1  |   cmd_value_2<<8  |  cmd_value_3<<16;
		}

	//CDC_Transmit_FS(&cmd_values, 6);
}
void SPI_Init(SPI_HandleTypeDef *hspi, uint16_t Size)
{
	/* Init hspi1 */
	__HAL_LOCK(hspi);
	(hspi)->ErrorCode   = HAL_SPI_ERROR_NONE;
	hspi->TxXferCount = Size;
	hspi->RxXferCount = Size;

	#if defined (__GNUC__)
	__IO uint16_t *ptxdr_16bits = (__IO uint16_t *)(&( (hspi)->Instance->TXDR));
	__IO uint16_t *prxdr_16bits = (__IO uint16_t *)(&( (hspi)->Instance->RXDR));
	#endif /* __GNUC__ */
	/*Init field not used in handle to zero */
	(hspi)->RxISR       = NULL;
	(hspi)->TxISR       = NULL;
	MODIFY_REG((hspi)->Instance->CR2, SPI_CR2_TSIZE, Size);
	__HAL_SPI_ENABLE(hspi);
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive_HM(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout)
{
  HAL_SPI_StateTypeDef tmp_state;
  HAL_StatusTypeDef errorcode = HAL_OK;
#if defined (__GNUC__)
  __IO uint16_t *ptxdr_16bits = (__IO uint16_t *)(&(hspi->Instance->TXDR));
  __IO uint16_t *prxdr_16bits = (__IO uint16_t *)(&(hspi->Instance->RXDR));
#endif /* __GNUC__ */

  uint32_t   tickstart;
  uint32_t   tmp_mode;
  uint16_t   initial_TxXferCount;
  uint16_t   initial_RxXferCount;

  /* Check Direction parameter */
  //assert_param(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

  /* Process Locked */
  //__HAL_LOCK(hspi);

  /* Init tickstart for timeout management*/
  tickstart = HAL_GetTick();

  initial_TxXferCount = Size;
  initial_RxXferCount = Size;
  tmp_state           = hspi->State;
  tmp_mode            = hspi->Init.Mode;

  if (!((tmp_state == HAL_SPI_STATE_READY) || \
        ((tmp_mode == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES) && (tmp_state == HAL_SPI_STATE_BUSY_RX))))
  {
    errorcode = HAL_BUSY;
    __HAL_UNLOCK(hspi);
    return errorcode;
  }

  if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0UL))
  {
    errorcode = HAL_ERROR;
    __HAL_UNLOCK(hspi);
    return errorcode;
  }

  /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
  if (hspi->State != HAL_SPI_STATE_BUSY_RX)
  {
    hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
  }

  /* Set the transaction information */
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
  hspi->pRxBuffPtr  = (uint8_t *)pRxData;
  hspi->RxXferCount = Size;
  hspi->RxXferSize  = Size;
  hspi->pTxBuffPtr  = (uint8_t *)pTxData;
  hspi->TxXferCount = Size;
  hspi->TxXferSize  = Size;

  /*Init field not used in handle to zero */
  hspi->RxISR       = NULL;
  hspi->TxISR       = NULL;

  /* Set the number of data at current transfer */
  MODIFY_REG(hspi->Instance->CR2, SPI_CR2_TSIZE, Size);

  __HAL_SPI_ENABLE(hspi);

  if (hspi->Init.Mode == SPI_MODE_MASTER)
  {
    /* Master transfer start */
    SET_BIT(hspi->Instance->CR1, SPI_CR1_CSTART);
  }

  /* Transmit and Receive data in 32 Bit mode */
  //if (hspi->Init.DataSize > SPI_DATASIZE_16BIT)

    while ((initial_TxXferCount > 0UL) || (initial_RxXferCount > 0UL))
    {
      /* Check TXP flag */
      if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXP)) && (initial_TxXferCount > 0UL))
      {
        *((__IO uint32_t *)&hspi->Instance->TXDR) = *((uint32_t *)hspi->pTxBuffPtr);
        hspi->pTxBuffPtr += sizeof(uint32_t);
        hspi->TxXferCount --;
        initial_TxXferCount = hspi->TxXferCount;
      }

      /* Check RXWNE/EOT flag */
      if (((hspi->Instance->SR & (SPI_FLAG_RXWNE | SPI_FLAG_EOT)) != 0UL) && (initial_RxXferCount > 0UL))
      {
        *((uint32_t *)hspi->pRxBuffPtr) = *((__IO uint32_t *)&hspi->Instance->RXDR);
        hspi->pRxBuffPtr += sizeof(uint32_t);
        hspi->RxXferCount --;
        initial_RxXferCount = hspi->RxXferCount;
      }

      /* Timeout management */
      if ((((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
      {
        /* Call standard close procedure with error check */
        SPI_CloseTransfer(hspi);

        /* Process Unlocked */
        __HAL_UNLOCK(hspi);

        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_TIMEOUT);
        hspi->State = HAL_SPI_STATE_READY;
        return HAL_ERROR;
      }
    }

  /* Transmit and Receive data in 16 Bit mode */

  /* Transmit and Receive data in 8 Bit mode */


      /* Timeout management */
      if ((((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
      {
        /* Call standard close procedure with error check */
        SPI_CloseTransfer(hspi);

        /* Process Unlocked */
        __HAL_UNLOCK(hspi);

        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_TIMEOUT);
        hspi->State = HAL_SPI_STATE_READY;
        return HAL_ERROR;
      }

  /* Wait for Tx/Rx (and CRC) data to be sent/received */
  if (SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_EOT, RESET, tickstart, Timeout) != HAL_OK)
  {
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
  }

  /* Call standard close procedure with error check */
  SPI_CloseTransfer(hspi);


  /* Process Unlocked */
  __HAL_UNLOCK(hspi);

  hspi->State = HAL_SPI_STATE_READY;

  if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
  {
    return HAL_ERROR;
  }
  return errorcode;
}

HAL_StatusTypeDef HAL_SPI1_TransmitReceive_HM_fast(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)

{



	/*Turn on CSbar pin*/

	GPIOD->ODR ^= (1 << 14);

	//hspi->TxXferCount = Size;
	//hspi->RxXferCount = Size;
	(hspi)->pRxBuffPtr  = (uint8_t *)pRxData;
	(hspi)->pTxBuffPtr  = (uint8_t *)pTxData;
	HAL_StatusTypeDef errorcode = HAL_OK;


	//if (hspi->Init.Mode == SPI_MODE_MASTER)
	//{
	/* Master transfer start */
	SET_BIT(hspi->Instance->CR1, SPI_CR1_CSTART);
	//}

  /* Transmit and Receive data in 32 Bit mode */
      /* Check TXP flag */
	if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXP)))
	{
	*((__IO uint32_t *)&hspi->Instance->TXDR) = *((uint32_t *)hspi->pTxBuffPtr);
	}

	/* Check RXWNE/EOT flag */
	//if (    (hspi->Instance->SR & (SPI_FLAG_RXWNE | SPI_FLAG_EOT)) != 0UL)
	//{
	*((uint32_t *)hspi->pRxBuffPtr) = *((__IO uint32_t *)&hspi->Instance->RXDR);
	//}
  /* Transmit and Receive data in 16 Bit mode */

  /* Transmit and Receive data in 8 Bit mode */

  /* Timeout management */

  /* Wait for Tx/Rx (and CRC) data to be sent/received */
      while ((__HAL_SPI_GET_FLAG(hspi,SPI_FLAG_EOT) ? SET : RESET) == RESET)
      {
      }

  	GPIOD->ODR ^= (1 << 14);
  /* Call standard close procedure with error check */
  //SPI_CloseTransfer(hspi);
  __HAL_SPI_CLEAR_EOTFLAG(hspi);
  __HAL_SPI_CLEAR_TXTFFLAG(hspi);
  /* Process Unlocked */
  //__HAL_UNLOCK(hspi);

  hspi->State = HAL_SPI_STATE_READY;

  if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
  {
    return HAL_ERROR;
  }
  return errorcode;
}
HAL_StatusTypeDef ADC_SPI(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint32_t *pRxData, uint16_t Size)

{



	/*Turn on Conversion pin*/


	GPIOE->ODR ^= (1 << 15);
	GPIOE->ODR ^= (0 << 15);
	GPIOE->ODR ^= (0 << 15);
	GPIOE->ODR ^= (1 << 15);

	GPIOE->ODR ^= (0 << 15);
	GPIOE->ODR ^= (0 << 15);
	GPIOE->ODR ^= (0 << 15);




	// time separation between CSbar and SCK maybe must be larger 1.5us

	//hspi->TxXferCount = Size;
	//hspi->RxXferCount = Size;
	(hspi)->pRxBuffPtr  = pRxData;
	(hspi)->pTxBuffPtr  = (uint8_t *)pTxData;
	HAL_StatusTypeDef errorcode = HAL_OK;


	if (hspi->Init.Mode == SPI_MODE_MASTER)
	{
	/* Master transfer start */
	SET_BIT(hspi->Instance->CR1, SPI_CR1_CSTART);
	}

  /* Transmit and Receive data in 32 Bit mode */
      /* Check TXP flag */
	if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXP)))
	{
	*((__IO uint32_t *)&hspi->Instance->TXDR) = *((uint32_t *)hspi->pTxBuffPtr);
	}

	/* Check RXWNE/EOT flag */
	//if (    (hspi->Instance->SR & (SPI_FLAG_RXWNE | SPI_FLAG_EOT)) != 0UL)
	//{
	//*((uint32_t *)hspi->pRxBuffPtr) = *((__IO uint32_t *)&hspi->Instance->RXDR);
	*(pRxData) = *((__IO uint32_t *)&hspi->Instance->RXDR);

	//}
  /* Transmit and Receive data in 16 Bit mode */

  /* Transmit and Receive data in 8 Bit mode */

  /* Timeout management */

  /* Wait for Tx/Rx (and CRC) data to be sent/received */
      while ((__HAL_SPI_GET_FLAG(hspi,SPI_FLAG_EOT) ? SET : RESET) == RESET)
      {
      }

  /* Call standard close procedure with error check */
  //SPI_CloseTransfer(hspi);
  __HAL_SPI_CLEAR_EOTFLAG(hspi);
  __HAL_SPI_CLEAR_TXTFFLAG(hspi);
  /* Process Unlocked */
  __HAL_UNLOCK(hspi);

  hspi->State = HAL_SPI_STATE_READY;
  //if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
  //{
  //  return HAL_ERROR;
  //}
  return errorcode;
}
static HAL_StatusTypeDef SPI_WaitOnFlagUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status,
                                                    uint32_t Tickstart, uint32_t Timeout)
{
  /* Wait until flag is set */
  while ((__HAL_SPI_GET_FLAG(hspi, Flag) ? SET : RESET) == Status)
  {
    /* Check for the Timeout */
    if ((((HAL_GetTick() - Tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
    {
      return HAL_TIMEOUT;
    }
  }
  return HAL_OK;
}

HAL_StatusTypeDef ADC_SPI_wo_CNV(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint32_t *pRxData, uint16_t Size)

{



	/*Turn on Conversion pin*/







	// time separation between CSbar and SCK maybe must be larger 1.5us

	//hspi->TxXferCount = Size;
	//hspi->RxXferCount = Size;
	(hspi)->pRxBuffPtr  = pRxData;
	(hspi)->pTxBuffPtr  = (uint8_t *)pTxData;
	HAL_StatusTypeDef errorcode = HAL_OK;


	if (hspi->Init.Mode == SPI_MODE_MASTER)
	{
	/* Master transfer start */
	SET_BIT(hspi->Instance->CR1, SPI_CR1_CSTART);
	}

  /* Transmit and Receive data in 32 Bit mode */
      /* Check TXP flag */
	if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXP)))
	{
	*((__IO uint32_t *)&hspi->Instance->TXDR) = *((uint32_t *)hspi->pTxBuffPtr);
	}

	/* Check RXWNE/EOT flag */
	//if (    (hspi->Instance->SR & (SPI_FLAG_RXWNE | SPI_FLAG_EOT)) != 0UL)
	//{
	//*((uint32_t *)hspi->pRxBuffPtr) = *((__IO uint32_t *)&hspi->Instance->RXDR);
	*(pRxData) = *((__IO uint32_t *)&hspi->Instance->RXDR);

	//}
  /* Transmit and Receive data in 16 Bit mode */

  /* Transmit and Receive data in 8 Bit mode */

  /* Timeout management */

  /* Wait for Tx/Rx (and CRC) data to be sent/received */
      while ((__HAL_SPI_GET_FLAG(hspi,SPI_FLAG_EOT) ? SET : RESET) == RESET)
      {
      }

  /* Call standard close procedure with error check */
  //SPI_CloseTransfer(hspi);
  __HAL_SPI_CLEAR_EOTFLAG(hspi);
  __HAL_SPI_CLEAR_TXTFFLAG(hspi);
  /* Process Unlocked */
  __HAL_UNLOCK(hspi);

  hspi->State = HAL_SPI_STATE_READY;
  //if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
  //{
  //  return HAL_ERROR;
  //}
  return errorcode;
}


static void SPI_CloseTransfer(SPI_HandleTypeDef *hspi)
{
  uint32_t itflag = hspi->Instance->SR;

  __HAL_SPI_CLEAR_EOTFLAG(hspi);
  __HAL_SPI_CLEAR_TXTFFLAG(hspi);

  /* Disable SPI peripheral */
  __HAL_SPI_DISABLE(hspi);

  /* Disable ITs */
  __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP | SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));

  /* Disable Tx DMA Request */
  CLEAR_BIT(hspi->Instance->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

  /* Report UnderRun error for non RX Only communication */
  if (hspi->State != HAL_SPI_STATE_BUSY_RX)
  {
    if ((itflag & SPI_FLAG_UDR) != 0UL)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_UDR);
      __HAL_SPI_CLEAR_UDRFLAG(hspi);
    }
  }

  /* Report OverRun error for non TX Only communication */
  if (hspi->State != HAL_SPI_STATE_BUSY_TX)
  {
    if ((itflag & SPI_FLAG_OVR) != 0UL)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_OVR);
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }

#if (USE_SPI_CRC != 0UL)
    /* Check if CRC error occurred */
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      if ((itflag & SPI_FLAG_CRCERR) != 0UL)
      {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
        __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
      }
    }
#endif /* USE_SPI_CRC */
  }

  /* SPI Mode Fault error interrupt occurred -------------------------------*/
  if ((itflag & SPI_FLAG_MODF) != 0UL)
  {
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_MODF);
    __HAL_SPI_CLEAR_MODFFLAG(hspi);
  }

  /* SPI Frame error interrupt occurred ------------------------------------*/
  if ((itflag & SPI_FLAG_FRE) != 0UL)
  {
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FRE);
    __HAL_SPI_CLEAR_FREFLAG(hspi);
  }

  hspi->TxXferCount = (uint16_t)0UL;
  hspi->RxXferCount = (uint16_t)0UL;
}
void pulse_DAC()
{
	  /*pulse DAC*/
	uint8_t i=0;
	  for (i=0;i<=50;i++)
	  {
	  dac_f=1.0;
	  dac_set = (dac_f*dac_conv);
	  dac_send = WriteCode_Update_A<<16|dac_set;
	  HAL_SPI1_TransmitReceive_HM_fast(&hspi1, &dac_send, pRxData, 1);
	  }

	  for (i=0;i<=50;i++)
	  {
	  dac_f=0.5;
	  dac_set = (dac_f*dac_conv);
	  dac_send = WriteCode_Update_A<<16|dac_set;
	  HAL_SPI1_TransmitReceive_HM_fast(&hspi1, &dac_send, pRxData, 1);
	  }
}

void test_delay()
{
	uint8_t i=0;
	float Vset=0.0;
	for (i=0;i<=3;i++)
	  {
		dac_set = (Vtosend[i]*dac_conv);
		Vset=Vtosend[i];
		dac_send = WriteCode_Update_A<<16|dac_set;
		HAL_SPI1_TransmitReceive_HM_fast(&hspi1, &dac_send, pRxData, 1);
		ADC_SPI(&hspi2, &pTxData[0], &ADC_1_Data_DB[i], 1); // read ADC
		ADC_SPI(&hspi2, &pTxData[0], &ADC_1_Data_DB[i], 1); // read ADC
		adc_f = ADC_1_Data_DB[i]*adc_conv_f;
	  }
}

// test function
void test_delay_2()
{
	uint8_t i=0;

	for (i=0;i<=3;i++)
	  {
		dac_set = (Vtosend[i]*dac_conv);
		Vset=Vtosend[i];
		dac_send = WriteCode_Update_A<<16|dac_set;
		HAL_SPI1_TransmitReceive_HM_fast(&hspi1, &dac_send, pRxData, 1);

		GPIOE->ODR ^= (1 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (1 << 15);

		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);

		GPIOE->ODR ^= (1 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (1 << 15);

		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		GPIOE->ODR ^= (0 << 15);
		ADC_SPI_wo_CNV(&hspi2, &pTxData[0], &ADC_1_Data_DB[i], 1); // read ADC
		adc_f = ADC_1_Data_DB[i]*adc_conv_f;
		GPIOE->ODR ^= (0 << 15);
	  }
}



void set_P(uint8_t* b1)
{
	memcpy(&kp, b1, sizeof(kp));
	CDC_Transmit_FS(&kp, 4);

}

void set_I(uint8_t* b1)
{
	memcpy(&ki, b1, sizeof(ki));
	CDC_Transmit_FS(&ki, 4);

}

void set_D(uint8_t* b1)
{
	memcpy(&kd, b1, sizeof(kd));
	CDC_Transmit_FS(&kd, 4);

}

void set_setpoint1(uint8_t* b1)
{
	memcpy(&setpoint_1, b1, sizeof(setpoint_1));
	CDC_Transmit_FS(&setpoint_1, 4);

}

void set_setpoint2(uint8_t* b1)
{
	memcpy(&setpoint_2, b1, sizeof(setpoint_2));
	CDC_Transmit_FS(&setpoint_2, 4);

}



void PID()
{
	  /*PID cal*/
	  uint8_t i=0;


	  e=0.0;
	  e1=0.0;

	  u=0.0;

	  setpoint_1 = 0.0;

	  //GPIOB->ODR ^= (1 );
	  while(sm==START_PID)
	  {


		  __disable_irq();


		  for (i=0;i<num_pid_loop;i++)
		  {
			  GPIOB->ODR = (0 );
			  GPIOB->ODR = (1 );
			  GPIOB->ODR = (0 );

		  //HAL_SPI2_TransmitReceive_HM_fast(&hspi2, &pTxData[0], &ADC_1_Data_DB[i], 1); // read ADC
			 //HAL_SPI2_TransmitReceive_HM_fast(&hspi2, &pTxData[0], &ADC_1_Data_DB[i], 1); // read ADC

		  ADC_SPI(&hspi2, &pTxData[0], &ADC_1_Data_DB[i], 1); // read ADC
		  ADC_SPI(&hspi2, &pTxData[0], &ADC_1_Data_DB[i], 1); // read ADC

		  //adc_f=ADC_Data[0]/1048575.0F*5-5;
		  //// as for ltc2377 with singled-ended to differential config, adc measure 2*vin -5V.

		  adc_f = ADC_1_Data_DB[i]*adc_conv_f;

		  e = setpoint_1 - adc_f;//for pid

		  integral = integral + e*tpid;
		  deri = (e - e1)*one_over_tpid ;

		  u = kp*e + ki*integral +kd*deri;

		  //dac_f = u; // for pid

		  dac_f=adc_f*kp;



		  dac_set = (int)(dac_f*dac_conv); //ASSUME VOLT SPAN IS 0->5V FOR NOW

		  DAC_Data_DB[i] = dac_set;

		  dac_send = WriteCode_Update_A<<16|dac_set;
		  //GPIOB->ODR ^= (1 );
		  HAL_SPI1_TransmitReceive_HM_fast(&hspi1, &dac_send, pRxData, 1);

		  e1 = e;

		  GPIOB->ODR = (0 );
		  GPIOB->ODR = (1 );
		  GPIOB->ODR = (0 );
		  }

		  __enable_irq();
	  }


}

void stop_PID()
{

}

void dual_PID()
{

}

/*
void SET_VOLT_DAC_1(float voltage)
{

	dac_set = (voltage*dac_conv);
	dac_send = WriteCode_Update_A<<16|dac_set;
	//GPIOB->ODR ^= (1 );


	HAL_SPI1_TransmitReceive_HM_fast(&hspi1, &dac_send, pRxData, 1);
}
*/

void SET_VOLT_DAC_1(float voltage)
{

	dac_set = (voltage*dac_conv);
	//dac_send = WriteCode_Update_A<<16|dac_set;
	//GPIOB->ODR ^= (1 );
	if (DAC_MODEL == 2752)
		{
			dac_send = WriteCode_Update_A<<16|dac_set;
		}
	else if (DAC_MODEL == 2758){
			dac_send = WriteCode_Update_A << 24 | dac_set<<6;
	}
	HAL_SPI1_TransmitReceive_HM_fast(&hspi1, &dac_send, pRxData, 1);

}

void SET_VOLT_DAC_1_bits(uint32_t send_dac_set)
{
	dac_send = WriteCode_Update_A<<16|send_dac_set;
	//GPIOB->ODR ^= (1 );
	HAL_SPI1_TransmitReceive_HM_fast(&hspi1, &dac_send, pRxData, 1);
}

void SET_SPAN_DAC_1(uint8_t setspan)
{
	if (DAC_MODEL == 2752)
		{
			DAC_command=WriteSpan_A<<16|setspan;
		}
	else if (DAC_MODEL == 2758){
		DAC_command=WriteSpan_A<<24| setspan << 8 ;
	}
	HAL_SPI1_TransmitReceive_HM_fast(&hspi1,&DAC_command, pRxData, 1);

}

void SET_VOLT_DAC_2(float voltage)
{

	dac_set = (voltage*dac_conv);
	dac_send = WriteCode_Update_B<<16|dac_set;
	//GPIOB->ODR ^= (1 );


	HAL_SPI1_TransmitReceive_HM_fast(&hspi1, &dac_send, pRxData, 1);
}

void SET_VOLT_DAC_2_bits(uint32_t send_dac_set)
{
	dac_send = WriteCode_Update_B<<16|send_dac_set;
	//GPIOB->ODR ^= (1 );
	HAL_SPI1_TransmitReceive_HM_fast(&hspi1, &dac_send, pRxData, 1);
}

void SET_SPAN_DAC_2(uint8_t setspan)
{
	  DAC_command=WriteSpan_B<<16|setspan;
	  HAL_SPI1_TransmitReceive_HM_fast(&hspi1,&DAC_command, pRxData, 1);
}

void return_sm()
{
	CDC_Transmit_FS(sm, 4); //return sm
}



void read_ADC_1(uint32_t cmd_run)
{
	/*
	 * cmd_run: number of runs, must be multiples of MAX_USB_TX
	 */

	//GPIOB->ODR = (0 );
	//GPIOB->ODR = (1 );
	//GPIOB->ODR = (0 );
	int i=0;
	uint32_t num_run = cmd_run * MAX_USB_TX;

	for (i=0; i < num_run & i<= (MAX_ADC_READ);i++)
	{
	ADC_SPI(&hspi2, &pTxData[0], &ADC_1_Data_DB[i], 1); // read ADC
	}

	//GPIOB->ODR = (0 );
	//GPIOB->ODR = (1 );
	//GPIOB->ODR = (0 );
	for (i=0; i<(cmd_run);i++)
	{
		while (CDC_Transmit_FS(&ADC_1_Data_DB[i*MAX_USB_TX], 4*(MAX_USB_TX) )== USBD_BUSY ); //send read ADC values to PC

	}

}

void tranx_ADC_1_buffer(uint32_t start_index, uint32_t num_read)
{
	//GPIOB->ODR = (0 );
	//GPIOB->ODR = (1 );
	//GPIOB->ODR = (0 );
	//while (CDC_Transmit_FS(&ADC_1_Data_DB[start_index], 4*(num_read) )== USBD_BUSY );
	CDC_Transmit_FS(&ADC_1_Data_DB[start_index], 4*(num_read) );
	//GPIOB->ODR = (0 );
	//GPIOB->ODR = (1 );
	//GPIOB->ODR = (0 );
}

void tranx_PID_1_buffer(uint32_t start_index, uint32_t num_read)
{
	int i=0;
	GPIOB->ODR = (0 );
	GPIOB->ODR = (1 );
	GPIOB->ODR = (0 );
	for (i=0;i<num_read;i++)
	{
		PID_1_Ctrl_DB[i]=ADC_1_Data_DB[i];
		PID_1_Ctrl_DB[i+num_read] = DAC_Data_DB[i];
	}
	//while (CDC_Transmit_FS(&ADC_1_Data_DB[start_index], 4*(num_read) )== USBD_BUSY );
	CDC_Transmit_FS(&PID_1_Ctrl_DB[start_index], 4*2*(num_read) );

	GPIOB->ODR = (0 );
	GPIOB->ODR = (1 );
	GPIOB->ODR = (0 );
}

void toggle_test(uint32_t num_run)
{
	int i=0;

	for (i=0;i<num_run;i++)
	{
		GPIOB -> ODR = (0);
		GPIOB -> ODR = (1);
		GPIOB -> ODR = (0);
	}
}

void derivative()
{


	ADC_SPI(&hspi2, &pTxData[0], &ADC_1_Data_DB[0], 1); // read ADC
	ADC_SPI(&hspi2, &pTxData[0], &ADC_1_Data_DB[1], 1);
	//HAL_SPI2_TransmitReceive_HM_fast(&hspi2, &pTxData[0], &ADC_Data[0], 1);

	adc_f = ADC_1_Data_DB[0]*adc_conv_f;
	adc_f_2= ADC_1_Data_DB[1]*adc_conv_f;

	//dac_f=(adc_f - adc_f_2)*134048.26F;
	dac_f=10.0*(adc_f - adc_f_2)+1.0;

	dac_set = (dac_f*dac_conv);
	dac_send = WriteCode_Update_A<<16|dac_set;
	//GPIOB->ODR ^= (1 );
	HAL_SPI1_TransmitReceive_HM_fast(&hspi1, &dac_send, pRxData, 1);

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
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  SPI_Init(&hspi1,Size);//dac
  GPIOD->ODR |= (1 << 14);// set csbar high
  //DAC_command=WriteSpan_A<<16|0x0000;// set span to 0 to 5V
  //
  HAL_SPI1_TransmitReceive_HM_fast(&hspi1,&DAC_command, pRxData, 1);

  SET_SPAN_DAC_1(0);

  SET_VOLT_DAC_1(0.0);

  SPI_Init(&hspi2,Size);//adc


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //sm = SET_OUTPUT_VOLTAGE_1;

  while (1)
  {
	  SET_VOLT_DAC_1(4.1);
	  ADC_SPI(&hspi2, &pTxData[0], &ADC_1_Data_DB[0], 1); // read ADC
	  switch (sm)
	  {

	  	  case SET_SPAN_1:
	  		  SET_SPAN_DAC_1(cmd_value_1);
	  		  sm=0;
	  		  break;

	  	  case SET_OUTPUT_VOLTAGE_1:
	  		  SET_VOLT_DAC_1_bits(cmd_values);
	  		  sm=0;
	  		  break;

	  	  case SET_SPAN_2:
	  		  SET_SPAN_DAC_2(cmd_value_1);
	  		  sm=0;
	  		  break;

	  	  case SET_OUTPUT_VOLTAGE_2:
	  		SET_VOLT_DAC_2_bits(cmd_values);
	  		sm=0;
	  		break;

	  	  case READ_ADC1:
	  		  read_ADC_1(cmd_value_1);
	  		  //read_ADC_1(100);
	  		  sm=0;
	  		  break;

	  	  case TRANX_ADC_1_BUFFER:
	  		  tranx_ADC_1_buffer(0,cmd_value_1);

	  		  sm=0;
	  		  break;

	  	  case TRANX_PID_1_CTRL:
	  		tranx_PID_1_buffer(0, cmd_value_1);
	  		sm=START_PID;
	  		PID();
	  		  break;

	  	  case START_PID:
	  		  		PID();
				break;

	  	  case STOP_PID:
	  		  sm=0;
	  		  break;

	  	  case SET_P_1:
	  		  set_P(&cmd_value_1);
	  		  sm=0;
	  		  break;

	  	  case SET_I_1:
	  		  set_I(&cmd_value_1);
	  		  sm=0;
	  		  break;

	  	  case SET_D_1:
	  		  set_D(&cmd_value_1);
	  		  sm=0;
	  		  break;

	  	  case SET_SETPOINT_1:
	  		  set_setpoint1(&cmd_value_1);
	  		  sm=0;
	  		  break;

	  	case SET_SETPOINT_2:
			  set_setpoint2(&cmd_value_1);
			  sm=0;
			  break;

	  	  case TOGGLE_LEDS_TEST:
	  		  toggle_test(cmd_value_1+cmd_value_2*256);
	  		  sm=0;
	  		  break;

	  }
	}
	  //PID();
	  //derivative();
	  //dac_f=1.0;
		//dac_set = (dac_f*dac_conv);
		//dac_send = WriteCode_Update_A<<16|dac_set;
		//GPIOB->ODR ^= (1 );
		//HAL_SPI1_TransmitReceive_HM_fast(&hspi1, &dac_send, pRxData, 1);
	  //GPIOB->ODR ^= (1 );
	  //__HAL_TIM_GET_COUNTER(&htim16);
	  //GPIOB->ODR ^= (1 );
	  //derivative();
	  /*Check continous ADC reading*/
	  /*
	  for (i=0;i<=199;i++)
	  {
	  HAL_SPI2_TransmitReceive_HM_fast(&hspi2, &pTxData[0], &ADC_Data_DB[i], 1); // read ADC
	  }
	  GPIOB->ODR ^= (1 );
	  /*

	  GPIOB->ODR ^= (1 );
	  /*end PID cal */



    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4|RCC_PERIPHCLK_SPI3
                              |RCC_PERIPHCLK_SPI1|RCC_PERIPHCLK_SPI2
                              |RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable USB Voltage detector
  */
  HAL_PWREx_EnableUSBVoltageDetector();
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_32BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_20BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 2000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|SPI4_CS_Pin|SPI2_RESET_Pin|SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, test_sig_Pin|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SPI1_CS_Pin|SPI1_RESET_Pin|SPI3_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 SPI4_CS_Pin SPI2_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|SPI4_CS_Pin|SPI2_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : test_sig_Pin */
  GPIO_InitStruct.Pin = test_sig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(test_sig_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_RESET_Pin SPI3_RST_Pin */
  GPIO_InitStruct.Pin = SPI1_RESET_Pin|SPI3_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim16 )
  {
	  timer_mark=1;
	  //GPIOB->ODR ^= (1);
  }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
