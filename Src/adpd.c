
/* Includes ------------------------------------------------------------------*/
#include "adpd.h"

// privite variables
static ADPDDrv_Operation_Slot_t gnSlotMode_A = (ADPDDrv_Operation_Slot_t) 0;
static ADPDDrv_Operation_Slot_t gnSlotMode_B = (ADPDDrv_Operation_Slot_t) 0;
static uint8_t gnAdpdDataSetSize_A, gnAdpdDataSetSize_B, gnChannelSize_A,
	       gnChannelSize_B, gnBytesPerSample_A, gnBytesPerSample_B;
static uint8_t gnAdpdDataSetSize = 0, gnAdpdFifoWaterMark = 1;
static uint32_t gnAccessCnt[5];
static uint8_t gnFifoLevel;
static uint32_t gnOverFlowCnt = 0;

// privite function
static int16_t AdpdDrvSetInterrupt(uint16_t nIntMask, I2C_HandleTypeDef *hi2c);

int16_t ADPD_I2C_DATA(I2C_HandleTypeDef *hi2c, uint16_t regAddress, uint8_t *pData, uint16_t Size)
{
   uint8_t regAdd[1];
   regAdd[0] = (uint8_t)regAddress;
  /*##-3- read register of adpd105 #####################################*/   
  do
  {
    if(HAL_I2C_Master_Transmit_DMA(hi2c, (uint16_t)ADPD_W, (uint8_t*)regAdd, 1)!= HAL_OK)
    {
      /* Error_Handler() function is called when error occurs. */
      printf("DCFG: IIC error");
    }

    /*##-3- Wait for the end of the transfer #################################*/  
    while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
    {
    } 

    /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
       Master restarts communication */
  }
  while(HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF);
  
  
  /*##-4- Put I2C peripheral in reception process ###########################*/  
  do
  {
    if(HAL_I2C_Master_Receive_DMA(hi2c, (uint16_t)ADPD_R, (uint8_t *)pData, Size) != HAL_OK)
    {
      /* Error_Handler() function is called when error occurs. */
      printf("DCFG: IIC error");
    }

    /*##-5- Wait for the end of the transfer #################################*/  
    while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
    {
    } 

    /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
       Master restarts communication */
  }
  while(HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF);

  /*##-5- Wait for the end of the transfer ###################################*/  
  while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
  {
  } 
  
	return TRUE;

}

int16_t ADPD_I2C_TxRx(I2C_HandleTypeDef *hi2c, uint16_t regAddress, uint8_t *pData, uint16_t Size)
{
   uint8_t regAdd[1];
   regAdd[0] = (uint8_t)regAddress;
  /*##-3- read register of adpd105 #####################################*/   
  do
  {
    if(HAL_I2C_Master_Transmit_DMA(hi2c, (uint16_t)ADPD_W, (uint8_t*)regAdd, 1)!= HAL_OK)
    {
      /* Error_Handler() function is called when error occurs. */
      printf("DCFG: IIC error");
    }

    /*##-3- Wait for the end of the transfer #################################*/  
    while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
    {
    } 

    /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
       Master restarts communication */
  }
  while(HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF);
  
  
  /*##-4- Put I2C peripheral in reception process ###########################*/  
  do
  {
    if(HAL_I2C_Master_Receive_DMA(hi2c, (uint16_t)ADPD_R, (uint8_t *)pData, 2) != HAL_OK)
    {
      /* Error_Handler() function is called when error occurs. */
      printf("DCFG: IIC error");
    }

    /*##-5- Wait for the end of the transfer #################################*/  
    while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
    {
    } 

    /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
       Master restarts communication */
  }
  while(HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF);

  /*##-5- Wait for the end of the transfer ###################################*/  
  while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
  {
  } 
  
	return TRUE;

}
/** @brief  Synchronous register read from the ADPD
  *
  * @param  nAddr 16-bit register address
  * @param  *pnData Pointer to 16-bit register data value
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t AdpdDrvRegRead(I2C_HandleTypeDef *hi2c, uint16_t nRegAddr, uint16_t *pnData)
{
	uint8_t anRxData[2];
//	nRegAddr = (uint8_t)nAddr;

	if (ADPD_I2C_TxRx(hi2c, nRegAddr, (uint8_t *)anRxData, 2) != TRUE)
		return FALSE;

	*pnData = (anRxData[0] << 8) + anRxData[1];
	return TRUE;
}

/** @brief  Synchronous register write to the ADPD
  *
  * @param  nAddr 16-bit register address
  * @param  nRegValue 16-bit register data value
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t AdpdDrvRegWrite(uint16_t nAddr, uint16_t nRegValue, I2C_HandleTypeDef *hi2c)
{
	uint8_t anI2cData[3];
	anI2cData[0] = (uint8_t)nAddr;
	anI2cData[1] = (uint8_t)(nRegValue >> 8);
	anI2cData[2] = (uint8_t)(nRegValue);
	
	do	
	  {
		if(HAL_I2C_Master_Transmit_DMA(hi2c, (uint16_t)ADPD_W, (uint8_t*)anI2cData, 3)!= HAL_OK)
		{
		  return FALSE;
		}

		while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
		{
		} 

	  }
	while(HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF);
	
	return TRUE;
}

/** @brief  Set various interrupt modes
  *
  * @param  nIntMask Interrupt mask bits will be set according to data sheet definition.
			If reserved bits are set, then keep the previous settings.
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
static int16_t AdpdDrvSetInterrupt(uint16_t nIntMask, I2C_HandleTypeDef *hi2c)
{
	int16_t nRetCode = ADPDDrv_SUCCESS;
	if ((nIntMask & 0x3E00) == 0) {
		nRetCode  = AdpdDrvRegWrite(REG_INT_MASK, nIntMask, hi2c);
	}
	return nRetCode;
}

void LoadDefaultConfig(uint32_t *cfg, I2C_HandleTypeDef *hi2c)
{
	uint8_t regAddr, i;
	uint16_t regData;
	if (cfg == 0) {
		return;
	}
	/* Clear the FIFO */
	AdpdDrvRegWrite(0x10, 0, hi2c);
	AdpdDrvRegWrite(0x5F, 1, hi2c);
	AdpdDrvRegWrite(0x00, 0x80FF, hi2c);
	AdpdDrvRegWrite(0x5F, 0, hi2c);
	i = 0;
	while (1) {
		/* Read the address and data from the config */
		regAddr = (uint8_t)(cfg[i] >> 16);
		regData = (uint16_t)(cfg[i]);
		i++;
		if (regAddr == 0xFF) {
			break;
		}
		/* Load the data into the ADPD registers */
		if (AdpdDrvRegWrite(regAddr, regData, hi2c) != TRUE) {
			break;
		}
	}
}


/**
  *  @brief    Read default configuration parameters to verify
  *  @param    uint32_t *cfg
  *  @retval   None
*/
uint8_t VerifyDefaultConfig(I2C_HandleTypeDef *hi2c, uint32_t *cfg)
{
	uint16_t def_val;
	uint8_t  i;
	uint8_t  regAddr;
	uint16_t regData;
	if (cfg == 0) {
		return FALSE;
	}
	i = 0;
	/* Read the address and data from the config */
	regAddr = (uint8_t)(cfg[0] >> 16);
	def_val = (uint16_t)(cfg[0]);
	/* Read the data from the ADPD registers and verify */
	while (regAddr != 0xFF) {
		if (AdpdDrvRegRead(hi2c,regAddr, &regData) != TRUE) {
			printf("DCFG: Read Error reg");
			return FALSE;
		} else if (regData != def_val) {
			printf("DCFG: Read mismatch reg");
			return FALSE;
		}
		i++;
		regAddr = (uint8_t)(cfg[i] >> 16);
		def_val = (uint16_t)(cfg[i]);
	}
	return TRUE;
}

/** @brief  Select operation time nSlot
  *
  * @param  nSlotA 8-bit time nSlot
  * @param  nSlotB 8-bit time nSlot
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t AdpdDrvSetSlot(uint8_t nSlotA, uint8_t nSlotB, I2C_HandleTypeDef *hi2c)
{
	uint16_t nRegValue, nDIRegValue, nRegAfeTrimA, nRegAfeTrimB;
	if (AdpdDrvRegRead(hi2c,REG_OP_MODE_CFG, &nRegValue) != TRUE)
		return FALSE;
	if (AdpdDrvRegRead(hi2c,REG_CALIBRATE, &nDIRegValue) != TRUE)
		return FALSE;
	if (AdpdDrvRegRead(hi2c,REG_AFE_TRIM_A, &nRegAfeTrimA) != TRUE)
		return FALSE;
	if (AdpdDrvRegRead(hi2c,REG_AFE_TRIM_B, &nRegAfeTrimB) != TRUE)
		return FALSE;
	gnSlotMode_A = (ADPDDrv_Operation_Slot_t)nSlotA;
	gnSlotMode_B = (ADPDDrv_Operation_Slot_t)nSlotB;
	nRegValue &= SLOT_MASK;
	if (nSlotA == ADPDDrv_4CH_16) {
		nRegValue |= SLOT_A_MODE;
		nRegValue |= RDOUT_MODE_EN;
		gnAdpdDataSetSize_A = SLOT_A_DATA_SIZE;
		gnChannelSize_A = 4;
		gnBytesPerSample_A = 2;
	}  else {
		return ADPDDrv_ERROR;
	}

	if (nSlotB == ADPDDrv_4CH_16) {
		nRegValue |= SLOT_B_MODE;
		nRegValue |= RDOUT_MODE_EN;
		gnAdpdDataSetSize_B = SLOT_B_DATA_SIZE;
		gnChannelSize_B = 4;
		gnBytesPerSample_B = 2;
	} else {
		return ADPDDrv_ERROR;
	}
	if (nSlotA & DISLOTMODEMASK) {
		nDIRegValue |= DIGITAL_INTEGRATE_A_EN;
		nRegAfeTrimA = (nRegAfeTrimA & AFE_DIG_INT_MASK) | AFE_DIG_INT_MODE;
	} else {
		nDIRegValue &= (~DIGITAL_INTEGRATE_A_EN);
		nRegAfeTrimA = (nRegAfeTrimA & AFE_DIG_INT_MASK) | AFE_NORMAL_MODE;
	}
	if (nSlotB & DISLOTMODEMASK) {
		nDIRegValue |= DIGITAL_INTEGRATE_B_EN;
		nRegAfeTrimB = (nRegAfeTrimB & AFE_DIG_INT_MASK) | AFE_DIG_INT_MODE;
	} else {
		nDIRegValue &= (~DIGITAL_INTEGRATE_B_EN);
		nRegAfeTrimB = (nRegAfeTrimB & AFE_DIG_INT_MASK) | AFE_NORMAL_MODE;
	}
	AdpdDrvRegWrite(REG_OP_MODE_CFG, nRegValue, hi2c);
	gnAdpdDataSetSize = gnAdpdDataSetSize_A + gnAdpdDataSetSize_B;
	AdpdDrvRegWrite(REG_CALIBRATE, nDIRegValue, hi2c);
	AdpdDrvRegWrite(REG_AFE_TRIM_A, nRegAfeTrimA, hi2c);
	AdpdDrvRegWrite(REG_AFE_TRIM_B, nRegAfeTrimB, hi2c);
	return ADPDDrv_SUCCESS;
}

/** @brief  Set Adpd operating mode, clear FIFO if needed
  *
  * @param  nOpMode 8-bit operating mode
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t AdpdDrvSetOperationMode(uint8_t nOpMode, I2C_HandleTypeDef *hi2c)
{
	uint8_t nLoopCnt = 0;
	int16_t nRetCode = TRUE;
	uint16_t nTemp;
	// uint16_t nRegValue1, nRegValue2;  // regValue1, nRegValue2;
	nRetCode = AdpdDrvRegWrite(REG_FIFO_CLK, FIFO_CLK_EN, hi2c);      // set clock ON
    if (nOpMode == ADPDDrv_MODE_SAMPLE) {
		nRetCode |= AdpdDrvSetInterrupt(FIFO_INT_EN, hi2c);  // FIFO interrupt mode
		nRetCode = AdpdDrvRegWrite(REG_I2CS_CTL_MATCH,
					   (((gnAdpdFifoWaterMark *
					      gnAdpdDataSetSize) - 1) << 7), hi2c);
		nRetCode |= AdpdDrvRegWrite(REG_OP_MODE, OP_PAUSE_MODE, hi2c);  // set Pause
		// enable FIFO clock
		nRetCode |= AdpdDrvRegWrite(REG_FIFO_CLK, FIFO_CLK_EN, hi2c);
		// CLR_HFIFO_SAMPLE
		nRetCode |= AdpdDrvRegRead(hi2c, REG_DATA_BUFFER, &nTemp); // Read 2 bytes dummy FIFO data
		nRetCode |= AdpdDrvRegWrite(REG_INT_STATUS, FIFO_CLR | IRQ_CLR_ALL, hi2c);
		// set GO ADPD105 RUN
		nRetCode |= AdpdDrvRegWrite(REG_OP_MODE, OP_RUN_MODE, hi2c);	// ADPD105 start to RUN
	} else {
		nRetCode = ADPDDrv_ERROR;
	}

	for (nLoopCnt = 0; nLoopCnt < 5; nLoopCnt++)
		gnAccessCnt[nLoopCnt] = 0;
	return nRetCode;
}

int16_t AdpdDrvGetParameter(AdpdCommandStruct eCommand, uint16_t *pnValue, I2C_HandleTypeDef *hi2c)
{
	int16_t nRetCode = ADPDDrv_SUCCESS;
	uint16_t nStatData;
	uint16_t nRegValue1, nRegValue2;

	if (eCommand == ADPD_WATERMARKING) {
		*pnValue = gnAdpdFifoWaterMark;
	} else if (eCommand == ADPD_FIFOLEVEL) {
		nRetCode |= AdpdDrvRegRead(hi2c, REG_INT_STATUS, &nStatData);
		nRetCode |= AdpdDrvRegWrite(REG_INT_STATUS, nStatData, hi2c);
		gnFifoLevel = nStatData >> 8;
		*pnValue = gnFifoLevel;
	} else if (eCommand == ADPD_TIMEGAP) {
		nRetCode |= AdpdDrvRegRead(hi2c, REG_SAMPLING_FREQ, &nRegValue1);
		nRetCode |= AdpdDrvRegRead(hi2c, REG_DEC_MODE, &nRegValue2);
		nRegValue2 = (nRegValue2 & 0xF0) >> 4;
		nRegValue2 = 1 << nRegValue2;
		*pnValue = ((nRegValue1 * nRegValue2) >> 3);
	} else if (eCommand == ADPD_DATASIZEA) {
		*pnValue = gnAdpdDataSetSize_A;
	} else if (eCommand == ADPD_DATASIZEB) {
		*pnValue = gnAdpdDataSetSize_B;
	} else if (eCommand == ADPD_SLOTMODEA) {
		*pnValue = gnSlotMode_A;
	} else if (eCommand == ADPD_SLOTMODEB) {
		*pnValue = gnSlotMode_B;
	} else {
		return ADPDDrv_ERROR;
	}

	return nRetCode;
}

/** @brief Read data out from Adpd FIFO
  *
  * @param  pollMode pollMode 1=continue polling.
  * @param nDataSetSize DataSet Size to be get
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t AdpdDrvReadFifoData(uint8_t *pnData, uint16_t nDataSetSize, I2C_HandleTypeDef *hi2c)
{
	uint8_t nAddr;
#ifndef NDEBUG
	if (gnFifoLevel >= 128)
		gnOverFlowCnt++;
#endif  // NDEBUG

	if (gnFifoLevel >= nDataSetSize) {
		gnAccessCnt[2]++;
		nAddr = REG_DATA_BUFFER;
		if (ADPD_I2C_DATA(hi2c, nAddr, pnData, nDataSetSize) != TRUE)
			return FALSE;

	}
	return TRUE;
}




