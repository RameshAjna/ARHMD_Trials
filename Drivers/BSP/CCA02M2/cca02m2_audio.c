/**
******************************************************************************
* @file    cca02m2_audio.c
* @author  SRA
* @version v1.1.1
* @date    18-Dec-2020
* @brief   This file provides the Audio driver for the cca02m2
*          board.
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

/* Includes ------------------------------------------------------------------*/
#include "cca02m2_audio.h"
#include "cca02m2_conf.h"
#include "arm_math.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup CCA02M2
  * @{
  */ 
  
/** @defgroup CCA02M2_AUDIO_ CCA02M2 AUDIO
  * @{
  */ 

/** @defgroup CCA02M2_AUDIO_Private_Macros CCA02M2_AUDIO_ Private Macros
  * @{
  */
/*### RECORD ###*/

#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))

#define DFSDM_OVER_SAMPLING(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (128U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (256U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (128U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (128U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (64U)  \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (32U) : (32U) 
	  
#define DFSDM_CLOCK_DIVIDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (17U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (16U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (16U) : (16U)

#define DFSDM_FILTER_ORDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (DFSDM_FILTER_SINC5_ORDER) \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (DFSDM_FILTER_SINC5_ORDER) : (DFSDM_FILTER_SINC5_ORDER)

#define DFSDM_MIC_BIT_SHIFT(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (8U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (5U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (8U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (8U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (10U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (10U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (10U) \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (5U) : (5U)

/**
  * @}
  */ 

/** @defgroup CCA02M2_AUDIO_Exported_Variables CCA02M2_AUDIO_ Exported Variables
  * @{
  */
/* Recording context */
AUDIO_IN_Ctx_t                         AudioInCtx[AUDIO_IN_INSTANCES_NBR] = {0};

/**
  * @}
  */
  
/** @defgroup CCA02M2_AUDIO_Private_Variables CCA02M2_AUDIO_ Private Variables
  * @{
  */

#define DECIMATOR_NUM_TAPS (16U)
#define DECIMATOR_BLOCK_SIZE (16U*N_MS_PER_INTERRUPT)
#define DECIMATOR_FACTOR 2U
#define DECIMATOR_STATE_LENGTH (DECIMATOR_BLOCK_SIZE + (DECIMATOR_NUM_TAPS) -1U)
static arm_fir_decimate_instance_q15 ARM_Decimator_State[4];

/* PDM filters params */
static PDM_Filter_Handler_t  PDM_FilterHandler[4];
static PDM_Filter_Config_t   PDM_FilterConfig[4];
I2S_HandleTypeDef               hAudioInI2s;
//static SPI_HandleTypeDef        hAudioInSPI;
static TIM_HandleTypeDef        TimDividerHandle;
static uint16_t I2S_InternalBuffer[PDM_INTERNAL_BUFFER_SIZE_I2S];
static uint16_t SPI_InternalBuffer[PDM_INTERNAL_BUFFER_SIZE_SPI];
            
static uint8_t Channel_Demux[128] = {
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f
};

/* Recording Buffer Trigger */
static __IO uint32_t    RecBuffTrigger          = 0;
static __IO uint32_t    RecBuffHalf             = 0;
static __IO uint32_t    MicBuffIndex[4];

/**
  * @}
  */ 
extern UART_HandleTypeDef huart2;

/** @defgroup CCA02M2_AUDIO_Private_Function_Prototypes CCA02M2_AUDIO_ Private Function Prototypes
  * @{
  */
static HAL_StatusTypeDef AUDIO_IN_Timer_Init(void);
static HAL_StatusTypeDef AUDIO_IN_Timer_Start(void);
static void I2S_MspInit(I2S_HandleTypeDef *hi2s);
//static void SPI_MspInit(SPI_HandleTypeDef *hspi);


/**
  * @}
  */ 

/** @defgroup CCA02M2_AUDIO_IN_Exported_Functions CCA02M2_AUDIO_IN Exported Functions
  * @{
  */ 
  
  
/**
* @brief  Initialize wave recording.
* @param  Instance  AUDIO IN Instance. It can be:
*       - 0 when I2S is used 
*       - 1 if DFSDM is used
*       - 2 if PDM is used
* @param  AudioInit Init structure
* @retval BSP status
*/
__weak int32_t CCA02M2_AUDIO_IN_Init(uint32_t Instance, CCA02M2_AUDIO_Init_t* AudioInit)
{  
	//HAL_UART_Transmit(&huart2, "audinint\r\n", 16, 100);
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;  
  }
  else
  {
    /* Store the audio record context */
    AudioInCtx[Instance].Device          = AudioInit->Device;
    AudioInCtx[Instance].ChannelsNbr     = AudioInit->ChannelsNbr;  
    AudioInCtx[Instance].SampleRate      = AudioInit->SampleRate; 
    AudioInCtx[Instance].BitsPerSample   = AudioInit->BitsPerSample;
    AudioInCtx[Instance].Volume          = AudioInit->Volume;
    AudioInCtx[Instance].State           = AUDIO_IN_STATE_RESET;
    
    if(Instance == 0U)
    { 
      uint32_t PDM_Clock_Freq;     
      
      switch (AudioInit->SampleRate)
      {
      case AUDIO_FREQUENCY_8K:
        PDM_Clock_Freq = 1280;
        break;
        
      case AUDIO_FREQUENCY_16K:
        PDM_Clock_Freq = PDM_FREQ_16K;
        break;
        
      case AUDIO_FREQUENCY_32K:
        PDM_Clock_Freq = 2048;
        break;
        
      case AUDIO_FREQUENCY_48K:
        PDM_Clock_Freq = 3072;
        break;
        
      default:        
        PDM_Clock_Freq = 0;
        break;
      }
      
      if (PDM_Clock_Freq == 0U)
      {
        return BSP_ERROR_WRONG_PARAM;
      }
      
      AudioInCtx[Instance].DecimationFactor = (PDM_Clock_Freq * 1000U)/AudioInit->SampleRate;
      /* Double buffer for 1 microphone */
      AudioInCtx[Instance].Size = (PDM_Clock_Freq/8U) * 2U * N_MS_PER_INTERRUPT;

      MX_I2S_IN_Config i2s_config;
      i2s_config.DataFormat = I2S_DATAFORMAT_32B;
      i2s_config.AudioFreq = ((PDM_Clock_Freq * 1000U) / 32U);
      i2s_config.CPOL = I2S_CPOL_HIGH;
      i2s_config.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
      i2s_config.Mode = I2S_MODE_MASTER_RX;
      i2s_config.Standard = I2S_STANDARD_MSB;
      i2s_config.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
      i2s_config.ClockSource = I2S_CLOCK_PLL;

      PDM_Clock_Freq *=2U;
      if (AUDIO_IN_Timer_Init() != HAL_OK)
      {
        return  BSP_ERROR_PERIPH_FAILURE;
      }
      /* PLL clock is set depending by the AudioFreq */ 
      if(MX_I2S_IN_ClockConfig(&hAudioInI2s, PDM_Clock_Freq) != HAL_OK)
      {
        return  BSP_ERROR_CLOCK_FAILURE;
      }      
      /* I2S Peripheral configuration */
      hAudioInI2s.Instance          = AUDIO_IN_I2S_INSTANCE;
      __HAL_I2S_DISABLE(&hAudioInI2s);
      I2S_MspInit(&hAudioInI2s);
      
      if (MX_I2S_IN_Init(&hAudioInI2s, &i2s_config)!= HAL_OK)
      {
        return  BSP_ERROR_PERIPH_FAILURE;
      }
      if (HAL_I2S_Init(&hAudioInI2s) != HAL_OK)
      {
        return  BSP_ERROR_PERIPH_FAILURE;
      }
      if (CCA02M2_AUDIO_IN_PDMToPCM_Init(Instance, AudioInCtx[0].SampleRate, AudioInCtx[0].ChannelsNbr, AudioInCtx[0].ChannelsNbr)!= BSP_ERROR_NONE)
      {
        return  BSP_ERROR_NO_INIT;
      }
    }
    else
    {
      return  BSP_ERROR_WRONG_PARAM;
    }
    
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP; 
    /* Return BSP status */ 
  }
  return BSP_ERROR_NONE;
}

/**
* @brief  Deinit the audio IN peripherals.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/

__weak int32_t CCA02M2_AUDIO_IN_DeInit(uint32_t Instance)
{  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if(Instance != 0U)
    {
      return  BSP_ERROR_WRONG_PARAM;
    }
  /* Update BSP AUDIO IN state */     
  AudioInCtx[Instance].State = AUDIO_IN_STATE_RESET;   
  }
  /* Return BSP status */
  return BSP_ERROR_NONE;
}

/**
* @brief  Clock Config.
* @param  hi2s: I2S handle if required
* @param  SampleRate: Audio frequency used to play the audio stream.
* @note   This API is called by CCA02M2_AUDIO_IN_Init() 
*         Being __weak it can be overwritten by the application     
* @retval HAL_OK if no problem during execution, HAL_ERROR otherwise
*/
__weak HAL_StatusTypeDef MX_I2S_IN_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t PDM_rate)
{ 
	//HAL_UART_Transmit(&huart2, "i2sclkcfg\r\n", 16, 100);
  UNUSED(hi2s);
  
  HAL_StatusTypeDef ret = HAL_OK;
  /*I2S PLL Configuration*/
  RCC_PeriphCLKInitTypeDef rccclkinit;
  HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit); 
  
#if defined(STM32F446xx)
  rccclkinit.PLLI2S.PLLI2SQ = 2;
  rccclkinit.PLLI2SDivQ = 1;
#endif
  if ((PDM_rate % 1280U) == 0U)
  {
#if defined(STM32F411xE) || defined (STM32F446xx)    
    rccclkinit.PLLI2S.PLLI2SM = 10;
    rccclkinit.PLLI2S.PLLI2SN = 96;
#else
    rccclkinit.PLLI2S.PLLI2SN = 192;
#endif
    rccclkinit.PLLI2S.PLLI2SR = 5;
  }
  else
  {
#if defined(STM32F411xE) || defined (STM32F446xx)
    
    rccclkinit.PLLI2S.PLLI2SM = 8;
#endif
    rccclkinit.PLLI2S.PLLI2SN = 258;
    rccclkinit.PLLI2S.PLLI2SR = 3;
  }   
  
#if defined(STM32F446xx)
  rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB2;
#else
  rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
#endif
  
  if(HAL_RCCEx_PeriphCLKConfig(&rccclkinit) != HAL_OK)
  {
    ret = HAL_ERROR;
  }  
  return ret;
}

__weak HAL_StatusTypeDef MX_I2S_IN_Init(I2S_HandleTypeDef* hi2s, MX_I2S_IN_Config *MXConfig)
{
	//HAL_UART_Transmit(&huart2, "i2sint\r\n", 16, 100);
  static DMA_HandleTypeDef hdma_i2sRx;
  HAL_StatusTypeDef ret = HAL_OK;
  
  hi2s->Init.DataFormat = MXConfig->DataFormat;
  hi2s->Init.AudioFreq = MXConfig->AudioFreq;
  hi2s->Init.ClockSource = MXConfig->ClockSource;
  hi2s->Init.CPOL = MXConfig->CPOL;
  hi2s->Init.MCLKOutput = MXConfig->MCLKOutput;
  hi2s->Init.Mode = MXConfig->Mode;
  hi2s->Init.Standard = MXConfig->Standard;
  hi2s->Init.FullDuplexMode = MXConfig->FullDuplexMode;  
  
  /* Enable the DMA clock */
  AUDIO_IN_I2S_DMAx_CLK_ENABLE();
  
  if(hi2s->Instance == AUDIO_IN_I2S_INSTANCE)
  {
    /* Configure the hdma_i2sRx handle parameters */
    hdma_i2sRx.Init.Channel             = AUDIO_IN_I2S_DMAx_CHANNEL;
    hdma_i2sRx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_i2sRx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2sRx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2sRx.Init.PeriphDataAlignment = AUDIO_IN_I2S_DMAx_PERIPH_DATA_SIZE;
    hdma_i2sRx.Init.MemDataAlignment    = AUDIO_IN_I2S_DMAx_MEM_DATA_SIZE;
    hdma_i2sRx.Init.Mode                = DMA_CIRCULAR;
    hdma_i2sRx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_i2sRx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_i2sRx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2sRx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_i2sRx.Init.PeriphBurst         = DMA_MBURST_SINGLE;
    
    hdma_i2sRx.Instance = AUDIO_IN_I2S_DMAx_STREAM;
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(hi2s, hdmarx, hdma_i2sRx);    
    /* Deinitialize the Stream for new transfer */
    if (HAL_DMA_DeInit(&hdma_i2sRx) != HAL_OK)
    {
      ret = HAL_ERROR;
    }    
    /* Configure the DMA Stream */
    if (HAL_DMA_Init(&hdma_i2sRx) != HAL_OK)
    {
      ret = HAL_ERROR;
    }
  }
  else
  {
    ret = HAL_ERROR;
  }  
  /* I2S DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(AUDIO_IN_I2S_DMAx_IRQ, CCA02M2_AUDIO_IN_IT_PRIORITY, CCA02M2_AUDIO_IN_IT_PRIORITY);
//  HAL_NVIC_SetPriority(AUDIO_IN_I2S_DMAx_IRQ, 5, 0);
  HAL_NVIC_EnableIRQ(AUDIO_IN_I2S_DMAx_IRQ);
  
  return ret;
}

/**
* @brief  Initialize the PDM library.
* @param Instance    AUDIO IN Instance
* @param  AudioFreq  Audio sampling frequency
* @param  ChnlNbrIn  Number of input audio channels in the PDM buffer
* @param  ChnlNbrOut Number of desired output audio channels in the  resulting PCM buffer
* @retval BSP status
*/
__weak int32_t CCA02M2_AUDIO_IN_PDMToPCM_Init(uint32_t Instance, uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut)
{  
	//HAL_UART_Transmit(&huart2, "pdm2pcmint\r\n", 16, 100);
  if(Instance != 0U)
  {
    return  BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
#ifdef USE_STM32L4XX_NUCLEO
    return  BSP_ERROR_WRONG_PARAM;
#else    
    uint32_t index; 
#if (ENABLE_HIGH_PERFORMANCE_MODE == 0U)    
    static int16_t aState_ARM[4][DECIMATOR_STATE_LENGTH];
    static int16_t aCoeffs[] = { -1406, 1634, -1943, 2386, -3080, 4325, -7223, 21690, 21690, -7223, 4325, -3080, 2386, -1943, 1634, -1406, };
#endif
    
    /* Enable CRC peripheral to unlock the PDM library */
    __HAL_RCC_CRC_CLK_ENABLE();
    
    for(index = 0; index < ChnlNbrIn; index++)
    {
      volatile uint32_t error = 0;
      /* Init PDM filters */
      PDM_FilterHandler[index].bit_order  = PDM_FILTER_BIT_ORDER_LSB;
      if (ChnlNbrIn == 1U)
      {
        PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_BE; /* For WB this should be LE, TODO after bugfix in PDMlib */
      }
      else
      {
        PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_LE;
      }
      PDM_FilterHandler[index].high_pass_tap = 2122358088;
      PDM_FilterHandler[index].out_ptr_channels = (uint16_t)ChnlNbrOut;
      PDM_FilterHandler[index].in_ptr_channels  = (uint16_t)ChnlNbrIn;
      
      /* PDM lib config phase */
      PDM_FilterConfig[index].output_samples_number = (uint16_t) ((AudioFreq/1000U) * N_MS_PER_INTERRUPT);
      PDM_FilterConfig[index].mic_gain = 24;
      
      switch (AudioInCtx[0].DecimationFactor)
      {
          case 64:
            PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_64_HI_PERF;
            break;
          case 96:
            PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_96_HI_PERF;
            break;
          default:
            break;
      }  

      switch(AudioInCtx[0].BitsPerSample)
      {
      case AUDIO_RESOLUTION_16b:
        PDM_FilterConfig[index].bit_depth = PDM_FILTER_BITDEPTH_16;
        break;
      case AUDIO_RESOLUTION_24b:
        PDM_FilterConfig[index].bit_depth = PDM_FILTER_BITDEPTH_24;
        break;
      case AUDIO_RESOLUTION_32b:
        PDM_FilterConfig[index].bit_depth = PDM_FILTER_BITDEPTH_24IN32;
        break;
      default:
        break;        
      }
      
      error = PDM_Filter_Init((PDM_Filter_Handler_t *)(&PDM_FilterHandler[index]));
      if (error!=0U)
      {
        return  BSP_ERROR_NO_INIT;
      }      
      //PDM_Filter_Config_t   PDM_FilterC;
      //PDM_Filter_getConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterC);
      error = PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
      if (error!=0U)
      {
        //return  BSP_ERROR_NO_INIT;
      }
    }    
#endif
  } 
  return BSP_ERROR_NONE;
}



/**
* @brief  Converts audio format from PDM to PCM.
* @param  Instance  AUDIO IN Instance  
* @param  PDMBuf    Pointer to PDM buffer data
* @param  PCMBuf    Pointer to PCM buffer data
* @retval BSP status
*/
__weak int32_t CCA02M2_AUDIO_IN_PDMToPCM(uint32_t Instance, uint16_t *PDMBuf, uint16_t *PCMBuf)
{    
	////HAL_UART_Transmit(&huart2, "audinpdmpcm\r\n", 16, 100);
  if(Instance != 0U)
  {
    return  BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
#ifdef USE_STM32L4XX_NUCLEO    
    return  BSP_ERROR_WRONG_PARAM;
#else
    uint32_t index;
    
    for(index = 0; index < AudioInCtx[Instance].ChannelsNbr; index++)
    {
      if (AudioInCtx[Instance].SampleRate == 8000U)
      {
        uint16_t Decimate_Out[8U*N_MS_PER_INTERRUPT];
        uint32_t ii;
        uint16_t PDM_Filter_Out[16U*N_MS_PER_INTERRUPT];
        
        (void)PDM_Filter(&((uint8_t*)(PDMBuf))[index], PDM_Filter_Out, &PDM_FilterHandler[index]);
        (void)arm_fir_decimate_q15 (&ARM_Decimator_State[index], (q15_t *)&(PDM_Filter_Out), (q15_t*)&(Decimate_Out), DECIMATOR_BLOCK_SIZE);
        for (ii=0; ii<(8U*N_MS_PER_INTERRUPT); ii++)
        {
          PCMBuf[(ii * AudioInCtx[Instance].ChannelsNbr) + index] = Decimate_Out[ii];
        }
      }
      else
      {
        switch(AudioInCtx[Instance].BitsPerSample)
        {
        case AUDIO_RESOLUTION_16b:
          (void)PDM_Filter(&((uint8_t*)(PDMBuf))[index], (uint16_t*)&(PCMBuf[index]), &PDM_FilterHandler[index]);
          break;
        case AUDIO_RESOLUTION_24b:
          (void)PDM_Filter(&((uint8_t*)(PDMBuf))[index], &((uint8_t*)(PCMBuf))[3U*index], &PDM_FilterHandler[index]);          
          break;
        case AUDIO_RESOLUTION_32b:
          (void)PDM_Filter(&((uint8_t*)(PDMBuf))[index], (uint32_t*)&(PCMBuf[index]), &PDM_FilterHandler[index]);          
          break;
        default:
          break;
        }
      }
    }    
#endif
  }  
  return BSP_ERROR_NONE;
}


/**
* @brief  Start audio recording.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  pbuf     Main buffer pointer for the recorded data storing  
* @param  NbrOfBytes     Size of the record buffer. Parameter not used when Instance is 0
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_Record(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes)
{  
  if(Instance >= (AUDIO_IN_INSTANCES_NBR - 1U) )
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
    AudioInCtx[Instance].pBuff = (uint16_t*)pBuf;
    
    if(Instance == 0U)
    {
      UNUSED(NbrOfBytes);

      if(AUDIO_IN_Timer_Start() != HAL_OK)
      {
        return BSP_ERROR_PERIPH_FAILURE;
      }
      
      if(HAL_I2S_Receive_DMA(&hAudioInI2s, I2S_InternalBuffer, (uint16_t)AudioInCtx[Instance].Size/2U) != HAL_OK)
      {
        return BSP_ERROR_PERIPH_FAILURE;
      }         
      /* Update BSP AUDIO IN state */     
      AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;
    }
    else
    {
      return BSP_ERROR_WRONG_PARAM;
    }
  }
  /* Return BSP status */
  return BSP_ERROR_NONE;
}

/**
* @brief  Stop audio recording.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_Stop(uint32_t Instance)
{
  if(Instance >= AUDIO_IN_INSTANCES_NBR) 
  {
    return BSP_ERROR_WRONG_PARAM;  
  }
  else
  {
    if(Instance == 0U)
    {
      if(AudioInCtx[Instance].ChannelsNbr > 2U)
      {
        //if(HAL_SPI_DMAStop(&hAudioInSPI)!= HAL_OK)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
      }
      
      if(HAL_I2S_DMAStop(&hAudioInI2s) != HAL_OK)
      {
        return BSP_ERROR_PERIPH_FAILURE;
      }      
    }
    else /*(Instance == 1U) */
    { 
      return  BSP_ERROR_WRONG_PARAM;
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP;
  } 
  /* Return BSP status */
  return BSP_ERROR_NONE;  
}


/**
* @brief  Pause the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_Pause(uint32_t Instance)
{
  if(Instance >= AUDIO_IN_INSTANCES_NBR) 
  {
    return BSP_ERROR_WRONG_PARAM;  
  }
  else
  {  
    if(Instance == 0U)
    {
      if(HAL_I2S_DMAPause(&hAudioInI2s)!= HAL_OK)
      {
        return BSP_ERROR_WRONG_PARAM;
      }      
    }
    else /* (Instance == 1U) */
    {
      return  BSP_ERROR_WRONG_PARAM;
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_PAUSE;    
  }
  /* Return BSP status */
  return BSP_ERROR_NONE;
}

/**
* @brief  Resume the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_Resume(uint32_t Instance)
{
  if(Instance >= AUDIO_IN_INSTANCES_NBR) 
  {
    return BSP_ERROR_WRONG_PARAM;  
  }
  else 
  {
    if(Instance == 0U)
    { 
      if(HAL_I2S_DMAResume(&hAudioInI2s)!= HAL_OK)
      {
        return BSP_ERROR_WRONG_PARAM;
      }
//      if(HAL_I2S_Receive_DMA(&hAudioInI2s, I2S_InternalBuffer, (uint16_t)AudioInCtx[Instance].Size/2U) != HAL_OK)
//            {
//              return BSP_ERROR_PERIPH_FAILURE;
//            }
    }
    else /* (Instance == 1U) */
    {
      return  BSP_ERROR_WRONG_PARAM;
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;
  }
  /* Return BSP status */
  return BSP_ERROR_NONE;
}

/**
* @brief  Starts audio recording.
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  pBuf      Main buffer pointer for the recorded data storing
* @param  NbrOfBytes      Size of the recorded buffer. Parameter not used when Instance is 0
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_RecordChannels(uint32_t Instance, uint8_t **pBuf, uint32_t NbrOfBytes)
{
  if(Instance != 1U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    UNUSED(pBuf);
    UNUSED(NbrOfBytes);
    return BSP_ERROR_WRONG_PARAM;
  }  
}

/**
* @brief  Stop audio recording.
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  Device    Digital input device to be stopped
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_StopChannels(uint32_t Instance, uint32_t Device)
{
  /* Stop selected devices */
  int32_t ret = CCA02M2_AUDIO_IN_PauseChannels(Instance, Device);
  /* Update BSP AUDIO IN state */     
  AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP;  
  /* Return BSP status */
  return ret; 
}

/**
* @brief  Pause the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  Device    Digital mic to be paused
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_PauseChannels(uint32_t Instance, uint32_t Device)
{
  if((Instance != 1U) || ((Device < AUDIO_IN_DIGITAL_MIC1) && (Device > AUDIO_IN_DIGITAL_MIC_LAST)))
  {
    return BSP_ERROR_WRONG_PARAM;  
  }
  else
  {
    return BSP_ERROR_WRONG_PARAM;
  }        
}

/**
* @brief  Resume the audio file stream
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  Device    Digital mic to be resumed
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_ResumeChannels(uint32_t Instance, uint32_t Device)
{
  if((Instance != 1U) || ((Device < AUDIO_IN_DIGITAL_MIC1) && (Device > AUDIO_IN_DIGITAL_MIC_LAST)))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    return BSP_ERROR_WRONG_PARAM;
  }  
}

/**
* @brief  Start audio recording.
* @param  Instance  AUDIO IN SAI PDM Instance. It can be only 2
* @param  pbuf     Main buffer pointer for the recorded data storing  
* @param  NbrOfBytes     Size of the record buffer. Parameter not used when Instance is 0
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_RecordPDM(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes)
{
	//HAL_UART_Transmit(&huart2, "audinrecpdm\r\n", 16, 100);
  if(Instance != 2U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
    UNUSED(pBuf);
    UNUSED(NbrOfBytes);
    return BSP_ERROR_WRONG_PARAM;
  }  
}


/**
* @brief  Set Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Device    The audio input device to be used
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device)
{
	//HAL_UART_Transmit(&huart2, "audinsetdev\r\n", 16, 100);
  CCA02M2_AUDIO_Init_t audio_init;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {  
    if(Instance == 1U)
    {
      return  BSP_ERROR_WRONG_PARAM;
    }
    audio_init.Device = Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;  
    audio_init.SampleRate    = AudioInCtx[Instance].SampleRate;   
    audio_init.BitsPerSample = AudioInCtx[Instance].BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume;
    
    if(CCA02M2_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      return BSP_ERROR_NO_INIT;
    }
  }
  else
  {
    return BSP_ERROR_BUSY;
  }  
  /* Return BSP status */  
  return BSP_ERROR_NONE;
}

/**
* @brief  Get Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Device    The audio input device used
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device)
{
	//HAL_UART_Transmit(&huart2, "audingetdev\r\n", 16, 100);
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Return audio Input Device */
    *Device = AudioInCtx[Instance].Device;
  }
  return BSP_ERROR_NONE;
}

/**
* @brief  Set Audio In frequency
* @param  Instance     Audio IN instance
* @param  SampleRate  Input frequency to be set
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t  SampleRate)
{
	//HAL_UART_Transmit(&huart2, "audinsetrate\r\n", 16, 100);
  CCA02M2_AUDIO_Init_t audio_init;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {
    if(Instance == 1U)
    {
      return  BSP_ERROR_WRONG_PARAM;
    }
    audio_init.Device        = AudioInCtx[Instance].Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;  
    audio_init.SampleRate    = SampleRate;   
    audio_init.BitsPerSample = AudioInCtx[Instance].BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume; 
    if(CCA02M2_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      return BSP_ERROR_NO_INIT;
    }   
  }
  else
  {
    return BSP_ERROR_BUSY;
  }  
  /* Return BSP status */
  return BSP_ERROR_NONE;  
}

/**
* @brief  Get Audio In frequency
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  SampleRate  Audio Input frequency to be returned
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate)
{
	//HAL_UART_Transmit(&huart2, "audingetrate\r\n", 16, 100);
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Return audio in frequency */
    *SampleRate = AudioInCtx[Instance].SampleRate;
  }  
  /* Return BSP status */  
  return BSP_ERROR_NONE;
}

/**
* @brief  Set Audio In Resolution
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  BitsPerSample  Input resolution to be set
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample)
{
	//HAL_UART_Transmit(&huart2, "audinsetbps\r\n", 15, 100);
  CCA02M2_AUDIO_Init_t audio_init;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {
    if(Instance == 1U)
    {
      return  BSP_ERROR_WRONG_PARAM;
    }
    audio_init.Device        = AudioInCtx[Instance].Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;  
    audio_init.SampleRate    = AudioInCtx[Instance].SampleRate;   
    audio_init.BitsPerSample = BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume; 
    if(CCA02M2_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      return BSP_ERROR_NO_INIT;
    }
  }
  else
  {
    return BSP_ERROR_BUSY;
  }  
  /* Return BSP status */  
  return BSP_ERROR_NONE;
}

/**
* @brief  Get Audio In Resolution
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  BitsPerSample  Input resolution to be returned
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample)
{
	//HAL_UART_Transmit(&huart2, "audingetbps\r\n", 15, 100);

  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Return audio in resolution */
    *BitsPerSample = AudioInCtx[Instance].BitsPerSample;
  }
  return BSP_ERROR_NONE;
}

/**
* @brief  Set Audio In Channel number
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  ChannelNbr  Channel number to be used
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr)
{
	//HAL_UART_Transmit(&huart2, "audinsetchn\r\n", 15, 100);
  if((Instance >= AUDIO_IN_INSTANCES_NBR) || (ChannelNbr > 2U))
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Update AudioIn Context */
    AudioInCtx[Instance].ChannelsNbr = ChannelNbr;
  }
  /* Return BSP status */
  return BSP_ERROR_NONE;
}

/**
* @brief  Get Audio In Channel number
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  ChannelNbr  Channel number to be used
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr)
{
	//HAL_UART_Transmit(&huart2, "audingetchn\r\n", 15, 100);
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Channel number to be returned */
    *ChannelNbr = AudioInCtx[Instance].ChannelsNbr;
  }
  return BSP_ERROR_NONE;
}

/**
* @brief  Set the current audio in volume level.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Volume    Volume level to be returnd
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume)
{
	//HAL_UART_Transmit(&huart2, "audinsetvol\r\n", 15, 100);
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else if (Instance == 0U)
  {
    uint32_t index;      
    static int16_t VolumeGain[] = 
    {
      -12,-12,-6,-3,0,2,3,5,6,7,8,9,9,10,11,11,12,12,13,13,14,14,15,15,15,
      16,16,17,17,17,17,18,18,18,19,19,19,19,19,20,20,20,20,21,21,21,21,21,
      22,22,22,22,22,22,23,23,23,23,23,23,23,24,24,24,24,24,24,24,25,25,25,
      25,25,25,25,25,25,26,26,26,26,26,26,26,26,26,27,27,27,27,27,27,27,27,
      27,27,28,28,28,28,28,28,28,28,28,28,28,28,29,29,29,29,29,29,29,29,29,
      29,29,29,29,30,30,30,30,30,30,30,31  
    };
    for (index = 0; index < AudioInCtx[Instance].ChannelsNbr; index++)
    {
      if (PDM_FilterConfig[index].mic_gain != VolumeGain[Volume])
      {
        PDM_FilterConfig[index].mic_gain = VolumeGain[Volume];
        (void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
      }
    }
  }
  else
  {
    /* Update AudioIn Context */
    AudioInCtx[Instance].Volume = Volume;
  }
  /* Return BSP status */
  return BSP_ERROR_NONE;  
}

/**
* @brief  Get the current audio in volume level.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Volume    Volume level to be returnd
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume)
{
	//HAL_UART_Transmit(&huart2, "audingetvol\r\n", 15, 100);
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }  
  else
  {
    /* Input Volume to be returned */
    *Volume = AudioInCtx[Instance].Volume;
  }
  /* Return BSP status */
  return BSP_ERROR_NONE;  
}

/**
* @brief  Get Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  State     Audio Out state
* @retval BSP status
*/
int32_t CCA02M2_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State)
{
	//HAL_UART_Transmit(&huart2, "audingetstate\r\n", 17, 100);
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Input State to be returned */
    *State = AudioInCtx[Instance].State;
  }
  return BSP_ERROR_NONE;
}

/**
* @brief Rx Transfer completed callbacks. It performs demuxing of the bit-interleaved PDM streams into 
byte-interleaved data suitable for PDM to PCM conversion. 1 ms of data for each microphone is 
written into the buffer that the user indicates when calling the CCA02M2_AUDIO_IN_Start(...) function.
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  UNUSED(hi2s);
  uint32_t index;
  ////HAL_UART_Transmit(&huart2, "rxcpclb\r\n", 11, 100);
  switch(AudioInCtx[0].ChannelsNbr)
  {
  case 1:
    {
      uint16_t * DataTempI2S = &I2S_InternalBuffer[AudioInCtx[0].Size/4U] ;
      for(index = 0; index < (AudioInCtx[0].Size/4U); index++)
      {
        AudioInCtx[0].pBuff[index] = (DataTempI2S[index]);
      }
      break;
    }
    
  case 2:
    {      
      uint16_t * DataTempI2S = &(I2S_InternalBuffer[AudioInCtx[0].Size/2U]);
      uint8_t a,b;
      for(index=0; index<(AudioInCtx[0].Size/2U); index++) 
      {
        a = ((uint8_t *)(DataTempI2S))[(index*2U)];
        b = ((uint8_t *)(DataTempI2S))[(index*2U)+1U];
        ((uint8_t *)(AudioInCtx[0].pBuff))[(index*2U)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] | (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);;
        ((uint8_t *)(AudioInCtx[0].pBuff))[(index*2U)+1U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] | (Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
      }
      break;
    }    
  case 4:
    {      
      uint16_t * DataTempI2S = &(I2S_InternalBuffer[AudioInCtx[0].Size/2U]);
      uint16_t * DataTempSPI = &(SPI_InternalBuffer[AudioInCtx[0].Size/2U]);
      uint8_t a,b;
      for(index=0; index<(AudioInCtx[0].Size/2U); index++) 
      {        
        a = ((uint8_t *)(DataTempI2S))[(index*2U)];
        b = ((uint8_t *)(DataTempI2S))[(index*2U)+1U];
        ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4U)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
          (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);;
          ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4U)+1U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
            (Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
            
            a = ((uint8_t *)(DataTempSPI))[(index*2U)];
            b = ((uint8_t *)(DataTempSPI))[(index*2U)+1U];
            ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4U)+2U] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
              (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);;
              ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4U)+3U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
                (Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
      }
      break;
    }
  default:
    {      
      break;
    }    
  }  
  CCA02M2_AUDIO_IN_TransferComplete_CallBack(0);
}

/**
* @brief Rx Transfer completed callbacks. It performs demuxing of the bit-interleaved PDM streams into 
byte-interleaved data suitable for PDM to PCM conversion. 1 ms of data for each microphone is 
written into the buffer that the user indicates when calling the CCA02M2_AUDIO_IN_Start(...) function.
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	////HAL_UART_Transmit(&huart2, "rxhacpclb\r\n", 13, 100);
  UNUSED(hi2s);
  uint32_t index;
  switch(AudioInCtx[0].ChannelsNbr)
  {
  case 1:
    {
      uint16_t * DataTempI2S = I2S_InternalBuffer;
      for(index = 0; index < (AudioInCtx[0].Size/4U); index++)
      {
        AudioInCtx[0].pBuff[index] = (DataTempI2S[index]);
      }
      break;
    }    
  case 2:
    {      
      uint16_t * DataTempI2S = I2S_InternalBuffer;
      uint8_t a,b;
      for(index=0; index<(AudioInCtx[0].Size/2U); index++) 
      {
        a = ((uint8_t *)(DataTempI2S))[(index*2U)];
        b = ((uint8_t *)(DataTempI2S))[(index*2U)+1U];
        ((uint8_t *)(AudioInCtx[0].pBuff))[(index*2U)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
          (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);;
          ((uint8_t *)(AudioInCtx[0].pBuff))[(index*2U)+1U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
            (Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
      }      
      break;
    }    
  case 4:
    {      
      uint16_t * DataTempI2S = I2S_InternalBuffer;
      uint16_t * DataTempSPI = SPI_InternalBuffer;
      uint8_t a,b;
      for(index=0; index<(AudioInCtx[0].Size/2U); index++) 
      {        
        a = ((uint8_t *)(DataTempI2S))[(index*2U)];
        b = ((uint8_t *)(DataTempI2S))[(index*2U)+1U];
        ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4U)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
          (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);;
          ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4U)+1U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
            (Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
            
            a = ((uint8_t *)(DataTempSPI))[(index*2U)];
            b = ((uint8_t *)(DataTempSPI))[(index*2U)+1U];
            ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4U)+2U] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
              (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);;
              ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4U)+3U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
                (Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
      }
      break;   
    }
  default:
    {      
      break;
    }    
  }  
  CCA02M2_AUDIO_IN_HalfTransfer_CallBack(0);
}

/**
* @brief  User callback when record buffer is filled.
* @retval None
*/
__weak void CCA02M2_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
	//HAL_UART_Transmit(&huart2, "audioincomp\r\n", 15, 100);
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
  
  /* This function should be implemented by the user application.
  It is called into this driver when the current buffer is filled
  to prepare the next buffer pointer and its size. */
}

/**
* @brief  Manages the DMA Half Transfer complete event.
* @retval None
*/
__weak void CCA02M2_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
	//HAL_UART_Transmit(&huart2, "audioinhalf\r\n", 15, 100);
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
  
  /* This function should be implemented by the user application.
  It is called into this driver when the current buffer is filled
  to prepare the next buffer pointer and its size. */
}

/**
* @brief  Audio IN Error callback function.
* @retval None
*/
__weak void CCA02M2_AUDIO_IN_Error_CallBack(uint32_t Instance)
{ 
	//HAL_UART_Transmit(&huart2, "audioinerr\r\n", 14, 100);
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
  
  /* This function is called when an Interrupt due to transfer error on or peripheral
  error occurs. */
}

/**
  * @}
  */ 
  
/** @defgroup CCA02M2_AUDIO_IN_Private_Functions CCA02M2_AUDIO_IN Private Functions
  * @{
  */ 

/*******************************************************************************
Static Functions
*******************************************************************************/

static void I2S_MspInit(I2S_HandleTypeDef *hi2s)
{	
	//HAL_UART_Transmit(&huart2, "i2sinit\r\n", 11, 100);
  UNUSED(hi2s);
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the I2S2 peripheral clock */
  AUDIO_IN_I2S_CLK_ENABLE();  
  /* Enable I2S GPIO clocks */
  AUDIO_IN_I2S_SCK_GPIO_CLK_ENABLE();
  AUDIO_IN_I2S_MOSI_GPIO_CLK_ENABLE();
  
  /* I2S2 pins configuration: SCK and MOSI pins ------------------------------*/
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  
  GPIO_InitStruct.Pin       = AUDIO_IN_I2S_SCK_PIN;
  GPIO_InitStruct.Alternate = AUDIO_IN_I2S_SCK_AF;
  HAL_GPIO_Init(AUDIO_IN_I2S_SCK_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin       = AUDIO_IN_I2S_MOSI_PIN ;
  GPIO_InitStruct.Alternate = AUDIO_IN_I2S_MOSI_AF;
  HAL_GPIO_Init(AUDIO_IN_I2S_MOSI_GPIO_PORT, &GPIO_InitStruct);
  
} 

/**
* @brief Audio Timer Init
* @param None
* @retval None
*/
static HAL_StatusTypeDef AUDIO_IN_Timer_Init(void)
{
	//HAL_UART_Transmit(&huart2, "audininit\r\n", 13, 100);
  HAL_StatusTypeDef ret =  HAL_OK;
  static TIM_SlaveConfigTypeDef   sSlaveConfig;
  static TIM_IC_InitTypeDef       sICConfig;
  static TIM_OC_InitTypeDef       sOCConfig; 
  GPIO_InitTypeDef   GPIO_InitStruct;
  
  /* Enable AUDIO_TIMER clock*/
  AUDIO_IN_TIMER_CLK_ENABLE();
  AUDIO_IN_TIMER_CHOUT_GPIO_PORT_CLK_ENABLE();
  AUDIO_IN_TIMER_CHIN_GPIO_PORT_CLK_ENABLE();
  
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  
  GPIO_InitStruct.Alternate = AUDIO_IN_TIMER_CHIN_AF;
  GPIO_InitStruct.Pin = AUDIO_IN_TIMER_CHIN_PIN;
  HAL_GPIO_Init(AUDIO_IN_TIMER_CHIN_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Alternate = AUDIO_IN_TIMER_CHOUT_AF;
  GPIO_InitStruct.Pin = AUDIO_IN_TIMER_CHOUT_PIN;
  HAL_GPIO_Init(AUDIO_IN_TIMER_CHOUT_GPIO_PORT, &GPIO_InitStruct);
  
  TimDividerHandle.Instance = AUDIO_IN_TIMER;
  
  /* Configure the Input: channel_1 */
  sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig.ICFilter = 0;
  if(HAL_TIM_IC_ConfigChannel(&TimDividerHandle, &sICConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    ret =  HAL_ERROR;
  }
  
  /* Configure TIM1 in Gated Slave mode for the external trigger (Filtered Timer
  Input 1) */
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.SlaveMode    = TIM_SLAVEMODE_EXTERNAL1;
  if( HAL_TIM_SlaveConfigSynchronization(&TimDividerHandle, &sSlaveConfig) != HAL_OK)
  {
    ret =  HAL_ERROR;
  }
  
  /* Initialize TIM3 peripheral in PWM mode*/
  TimDividerHandle.Init.Period            = 1;
  TimDividerHandle.Init.Prescaler         = 0;
  TimDividerHandle.Init.ClockDivision     = 0;
  TimDividerHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimDividerHandle.Init.RepetitionCounter = 0;
  if(HAL_TIM_PWM_Init(&TimDividerHandle) != HAL_OK)
  {
    ret =  HAL_ERROR;
  }
  
  /* Configure the PWM_channel_1  */
  sOCConfig.OCMode     = TIM_OCMODE_PWM1;
  sOCConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sOCConfig.Pulse = 1;
  if(HAL_TIM_PWM_ConfigChannel(&TimDividerHandle, &sOCConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    ret =  HAL_ERROR;
  }
  return ret;
}

/**
* @brief Audio Timer Start
* @param None
* @retval None
*/
static HAL_StatusTypeDef AUDIO_IN_Timer_Start(void)
{  
	//HAL_UART_Transmit(&huart2, "audinstart\r\n", 14, 100);
  HAL_StatusTypeDef ret =  HAL_OK;
  if(HAL_TIM_IC_Start(&TimDividerHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    ret =  HAL_ERROR;
  }
  /* Start the Output Compare */
  if(HAL_TIM_OC_Start(&TimDividerHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    ret =  HAL_ERROR;
  }
  
  return ret;
}

/**
* @}
*/ 

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
