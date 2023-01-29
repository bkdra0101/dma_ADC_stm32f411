/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  /* ADC1 DMA Init */

  /* ADC1 Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_0);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_NOINCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_EnableFifoMode(DMA2, LL_DMA_STREAM_0);

  LL_DMA_SetFIFOThreshold(DMA2, LL_DMA_STREAM_0, LL_DMA_FIFOTHRESHOLD_1_4);

  LL_DMA_SetMemoryBurstxfer(DMA2, LL_DMA_STREAM_0, LL_DMA_MBURST_SINGLE);

  LL_DMA_SetPeriphBurstxfer(DMA2, LL_DMA_STREAM_0, LL_DMA_PBURST_SINGLE);

  /* ADC1 interrupt Init */
  NVIC_SetPriority(ADC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),4, 0));
  NVIC_EnableIRQ(ADC_IRQn);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_VREFINT);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_3CYCLES);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/* USER CODE BEGIN 1 */
#define  Test_ADC_INST  ADC1
#define ADC_Wait_Stable() do{ HAL_Delay(10);} while (0); //10ms

void ADC_Start_Conversion(void)
  {
	  //ENABLE EOC interrupt
	  LL_ADC_EnableIT_EOCS(Test_ADC_INST);
	  // Turn on ADON
	  LL_ADC_Enable(Test_ADC_INST);
	 // wait ADC stable
			 ADC_Wait_Stable();
	  //Software start convert
	  LL_ADC_REG_StartConversionSWStart(Test_ADC_INST);
  }

extern volatile uint32_t adc_data ;
extern volatile uint8_t adc_convert_done ;
static uint32_t voltage = 0 ;
static uint16_t dma_data_buffer = 0 ;

#define ADC_Ratio (0.7326)
static inline uint32_t __ADC_Convert_to_Voltage (uint32_t raw_data)
		{

	return (uint32_t) ((float) (ADC_Ratio) * (float) raw_data );
		}
void ADC_Process_Data(void)
   {
		//uint16_t sample=0;
		while (!adc_convert_done){
			asm("NOP");
		};
		//sample = (dma_data_buffer[1] <<8) | dma_data_buffer[0];
		voltage = __ADC_Convert_to_Voltage(dma_data_buffer);
		adc_convert_done =0 ;

		LL_ADC_REG_StartConversionSWStart(Test_ADC_INST);
   }

void ADC_Switch_Temperature_Channel(void)
{
	 LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_16);
	  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_16, LL_ADC_SAMPLINGTIME_3CYCLES);
	  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);

}

//confige DMA
void ADC_Enable_DMA_REQ(void)
{
	LL_ADC_REG_SetDMATransfer(DMA2,LL_ADC_REG_DMA_TRANSFER_LIMITED);
}
void ADC_Configure_DMA_Transaction(void)
{
	//Disable DMA stream
	LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_0);

	//configure địa chỉ ADC ngoại vi
	LL_DMA_SetPeriphAddress(DMA2,LL_DMA_STREAM_0,LL_ADC_DMA_GetRegAddr(ADC1,LL_ADC_DMA_REG_REGULAR_DATA));
	//config ram add
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_0, (uint32_t)(&dma_data_buffer));
	//set number of data
	LL_DMA_SetDataLength(DMA2,LL_DMA_STREAM_0,1);
	//configure dma channel - default setting init ready
	// configure direction -
	LL_DMA_SetMemoryBurstxfer(DMA2,LL_DMA_STREAM_0,LL_DMA_MBURST_SINGLE);

	LL_DMA_EnableIT_TC(DMA2,LL_DMA_STREAM_0);
	//Enable DMA stream
	LL_DMA_EnableStream(DMA2,LL_DMA_STREAM_0);
}

/* USER CODE END 1 */
