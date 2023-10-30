/**
******************************************************************************
* @file    usbd_smart_glass_in.c
* @author  SRA
* @version v1.0.0
* @date    17-Jul-2020
* @brief   This file provides the core functions.
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under Ultimate Liberty license
* SLA0044, the "License"; You may not use this file except in compliance with
* the License. You may obtain a copy of the License at:
*                             www.st.com/SLA0044
*
******************************************************************************
*/ 

/* Includes ------------------------------------------------------------------*/

#include "usbd_smart_glass.h"

#include "usbd_desc.h"
#include "usbd_ctlreq.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
* @{
*/
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"
/** @defgroup USBD_AUDIO_IN 
*
* 	This file provides the Audio Input core functions.
*
*           This driver implements the following aspects:
*             - Device descriptor management
*             - Configuration descriptor management
*             - Standard AC Interface Descriptor management
*             - 1 Audio Streaming Interface
*             - 1 Audio Streaming Endpoint
*             - 1 Audio Terminal Input
*             - Audio Class-Specific AC Interfaces
*             - Audio Class-Specific AS Interfaces
*             - AudioControl Requests: mute and volume control
*             - Audio Synchronization type: Asynchronous
*             - Multiple frequencies and channel number configurable using ad hoc
*               init function
*
*          The current audio class version supports the following audio features:
*             - Pulse Coded Modulation (PCM) format
*             - Configurable sampling rate
*             - Bit resolution: 24
*             - Configurable Number of channels
*             - Volume control
*             - Mute/Unmute capability
*             - Asynchronous Endpoints
*
* @note     This driver has been developed starting from the usbd_audio.c file
*           included within the standard Cube Package for STM32F4
* @{
*/ 

/** @defgroup USBD_AUDIO_IN_Private_TypesDefinitions
* @{
*/ 
/**
* @}
*/ 

/** @defgroup USBD_AUDIO_IN_Private_Defines
* @{
*/ 

/**
* @}
*/ 

/** @defgroup USBD_AUDIO_IN_Private_Macros
* @{
*/ 
/**
* @}
*/ 

/** @defgroup USBD_AUDIO_IN_Private_FunctionPrototypes
* @{
*/
static uint8_t  USBD_AUDIO_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx); 
static uint8_t  USBD_AUDIO_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx); 
static uint8_t  USBD_AUDIO_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req); 
static uint8_t  *USBD_AUDIO_GetCfgDesc (uint16_t *length); 
static uint8_t  *USBD_AUDIO_GetDeviceQualifierDesc (uint16_t *length); 
static uint8_t  USBD_AUDIO_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum); 
static uint8_t  USBD_AUDIO_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum); 
static uint8_t  USBD_AUDIO_EP0_RxReady (USBD_HandleTypeDef *pdev); 
static uint8_t  USBD_AUDIO_EP0_TxReady (USBD_HandleTypeDef *pdev); 
static uint8_t  USBD_AUDIO_SOF (USBD_HandleTypeDef *pdev); 
static uint8_t  USBD_AUDIO_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum); 
static uint8_t  USBD_AUDIO_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum); 
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetMaximum(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetMinimum(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetResolution(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

/**
* @}
*/ 

/** @defgroup USBD_AUDIO_Private_Variables
* @{
*/ 
/* This dummy buffer with 0 values will be sent when there is no availble data */
static uint8_t IsocInBuffDummy[48*4*2]; 
static  int16_t VOL_CUR;
static USBD_SMART_GLASS_HandleTypeDef handleInstance;
extern UART_HandleTypeDef huart2;
extern StreamBufferHandle_t xStreamBuffer;

USBD_ClassTypeDef  USBD_AUDIO = 
{
  USBD_AUDIO_Init,
  USBD_AUDIO_DeInit,
  USBD_AUDIO_Setup,
  USBD_AUDIO_EP0_TxReady,  
  USBD_AUDIO_EP0_RxReady,
  USBD_AUDIO_DataIn,
  USBD_AUDIO_DataOut,
  USBD_AUDIO_SOF,
  USBD_AUDIO_IsoINIncomplete,
  USBD_AUDIO_IsoOutIncomplete,      
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc, 
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetDeviceQualifierDesc,
};

/* USB AUDIO device Configuration Descriptor */
/* NOTE: This descriptor has to be filled using the Descriptor Initialization function */
__ALIGN_BEGIN static uint8_t USBD_CLASS_SG_CfgDesc[USB_CLASS_SG_CONFIG_DESC_SIZ + 9] __ALIGN_END;

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CLASS_SG_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END=
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
* @}
*/ 

/** @defgroup USBD_AUDIO_IN_Private_Functions
* @{
*/ 

/**
* @brief  USBD_AUDIO_Init
*         Initialize the AUDIO interface
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/

static uint8_t  USBD_AUDIO_Init (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
  if(handleInstance.state!=STATE_USB_WAITING_FOR_INIT)
  {
    return USBD_FAIL; 
  }
  
  USBD_SMART_GLASS_HandleTypeDef   *handle;
  pdev->pClassData = &handleInstance;
  handle = (USBD_SMART_GLASS_HandleTypeDef *)pdev->pClassData;
  uint16_t packet_dim = handle->paketDimension;
  uint16_t wr_rd_offset = (AUDIO_IN_PACKET_NUM/2) * handle->dataAmount / handle->paketDimension;
  handle->wr_ptr=wr_rd_offset * packet_dim;
  handle->rd_ptr = 0;
  handle->timeout = 0;
  
  ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->Init(handle->frequency,0,handle->channels);
  
  USBD_LL_OpenEP(pdev,
                 AUDIO_IN_EP,
                 USBD_EP_TYPE_ISOC,
                 AUDIO_IN_PACKET);
  
  USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
  
  
  USBD_LL_Transmit(pdev, AUDIO_IN_EP,
                   IsocInBuffDummy,                        
                   packet_dim);      
  
  handle->state=STATE_USB_IDLE;

  //HID_Init
  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    pdev->ep_in[CUSTOM_HID_EPIN_ADDR & 0xFU].bInterval = CUSTOM_HID_HS_BINTERVAL;
    pdev->ep_out[CUSTOM_HID_EPOUT_ADDR & 0xFU].bInterval = CUSTOM_HID_HS_BINTERVAL;
  }
  else   /* LOW and FULL-speed endpoints */
  {
    pdev->ep_in[CUSTOM_HID_EPIN_ADDR & 0xFU].bInterval = CUSTOM_HID_FS_BINTERVAL;
    pdev->ep_out[CUSTOM_HID_EPOUT_ADDR & 0xFU].bInterval = CUSTOM_HID_FS_BINTERVAL;
  }

  /* Open EP IN */
  (void)USBD_LL_OpenEP(pdev, CUSTOM_HID_EPIN_ADDR, USBD_EP_TYPE_INTR,
                       CUSTOM_HID_EPIN_SIZE);
  pdev->ep_in[CUSTOM_HID_EPIN_ADDR & 0xFU].is_used = 1U;

  /* Open EP OUT */
  (void)USBD_LL_OpenEP(pdev, CUSTOM_HID_EPOUT_ADDR, USBD_EP_TYPE_INTR,
                       CUSTOM_HID_EPOUT_SIZE);
  pdev->ep_out[CUSTOM_HID_EPOUT_ADDR & 0xFU].is_used = 1U;
  handle->HID_state = CUSTOM_HID_IDLE;

  /* Prepare Out endpoint to receive 1st packet */
  (void)USBD_LL_PrepareReceive(pdev, CUSTOM_HID_EPOUT_ADDR, handle->Report_buf,
                               USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);

  //Command App Init
  USBD_LL_OpenEP(pdev, COMMAND_EPIN_ADDR, USBD_EP_TYPE_BULK, USB_FS_MAX_PACKET_SIZE);
  USBD_LL_OpenEP(pdev, COMMAND_EPOUT_ADDR, USBD_EP_TYPE_BULK, USB_FS_MAX_PACKET_SIZE);
  pdev->ep_in[COMMAND_EPIN_ADDR & 0x7F].is_used = 1;
  USBD_LL_FlushEP(pdev, COMMAND_EPIN_ADDR);
  USBD_LL_PrepareReceive(pdev, COMMAND_EPOUT_ADDR, handle->data_out_buffer, USB_FS_MAX_PACKET_SIZE);

  return USBD_OK;
}

/**
* @brief  USBD_AUDIO_Init
*         DeInitialize the AUDIO layer
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t  USBD_AUDIO_DeInit (USBD_HandleTypeDef *pdev, 
                                   uint8_t cfgidx)
{
  /* Close EP IN */
  USBD_LL_CloseEP(pdev,AUDIO_IN_EP);  
  /* DeInit  physical Interface components */
  if(pdev->pClassData != NULL)
  {
    ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->DeInit(0);
    handleInstance.state = STATE_USB_WAITING_FOR_INIT;
  }
  //HID_DeInit
  (void)USBD_LL_CloseEP(pdev, CUSTOM_HID_EPIN_ADDR);
    pdev->ep_in[CUSTOM_HID_EPIN_ADDR & 0xFU].is_used = 0U;
    pdev->ep_in[CUSTOM_HID_EPIN_ADDR & 0xFU].bInterval = 0U;

    /* Close CUSTOM_HID EP OUT */
    (void)USBD_LL_CloseEP(pdev, CUSTOM_HID_EPOUT_ADDR);
    pdev->ep_out[CUSTOM_HID_EPOUT_ADDR & 0xFU].is_used = 0U;
    pdev->ep_out[CUSTOM_HID_EPOUT_ADDR & 0xFU].bInterval = 0U;

   //Command App DeInit
    USBD_LL_CloseEP(pdev,COMMAND_EPIN_ADDR);
	pdev->ep_in[COMMAND_EPIN_ADDR & 0x7F].is_used = 0;

	pdev->pClassData = NULL;

  return USBD_OK;
}

/**
* @brief  USBD_AUDIO_Setup
*         Handle the AUDIO specific requests
* @param  pdev: instance
* @param  req: usb requests
* @retval status
*/
static uint8_t  USBD_AUDIO_Setup (USBD_HandleTypeDef *pdev, 
                                  USBD_SetupReqTypedef *req)
{
  USBD_SMART_GLASS_HandleTypeDef   *handle;
  uint16_t len;
  uint8_t *pbuf;
  uint8_t ret = USBD_OK;
  handle = pdev->pClassData;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    /* AUDIO Class Requests -------------------------------*/
  case USB_REQ_TYPE_CLASS :
    switch (req->bRequest)
    {
    case AUDIO_REQ_GET_CUR:
      AUDIO_REQ_GetCurrent(pdev, req);
      break;
      
    case AUDIO_REQ_SET_CUR:
      AUDIO_REQ_SetCurrent(pdev, req);   
      break;
      
    case AUDIO_REQ_GET_MIN:
      AUDIO_REQ_GetMinimum(pdev, req);
      break;
      
    case AUDIO_REQ_GET_MAX:
      AUDIO_REQ_GetMaximum(pdev, req);
      break;
      
    case AUDIO_REQ_GET_RES:
      AUDIO_REQ_GetResolution(pdev, req);
      break;
      
      //HID_Setup
      case CUSTOM_HID_REQ_SET_PROTOCOL:
      handle->Protocol = (uint8_t)(req->wValue);
      break;

    case CUSTOM_HID_REQ_GET_PROTOCOL:
      (void)USBD_CtlSendData(pdev, (uint8_t *)&handle->Protocol, 1U);
      break;

    case CUSTOM_HID_REQ_SET_IDLE:
      handle->IdleState = (uint8_t)(req->wValue >> 8);
      break;

    case CUSTOM_HID_REQ_GET_IDLE:
      (void)USBD_CtlSendData(pdev, (uint8_t *)&handle->IdleState, 1U);
      break;

    case CUSTOM_HID_REQ_SET_REPORT:
      handle->IsReportAvailable = 1U;
      (void)USBD_CtlPrepareRx(pdev, handle->Report_buf, req->wLength);
      break;

    default:
      USBD_CtlError (pdev, req);
      return USBD_FAIL;
    }
    break; 
    
    /* Standard Requests -------------------------------*/
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR: 
      if( (req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
      {
        
        pbuf = USBD_CLASS_SG_CfgDesc + 18;
        len = MIN(USB_AUDIO_DESC_SIZ , req->wLength);   
//
//        USBD_CtlSendData (pdev,
//                          pbuf,
//                          len);
      }
      else if ((req->wValue >> 8) == CUSTOM_HID_REPORT_DESC)
      {
         len = MIN(USBD_CUSTOM_HID_REPORT_DESC_SIZE, req->wLength);
         pbuf = ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->pReport;
      }
      else
       {
         if ((req->wValue >> 8) == CUSTOM_HID_DESCRIPTOR_TYPE)
         {
           pbuf = USBD_CLASS_SG_CfgDesc + 118;
           len = MIN(USB_CUSTOM_HID_DESC_SIZ, req->wLength);
         }
       }

       (void)USBD_CtlSendData(pdev, pbuf, len);
      break;
      
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        (uint8_t *)handle->alt_setting,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      if ((uint8_t)(req->wValue) < USBD_MAX_NUM_INTERFACES)
      {
        handle->alt_setting = (uint8_t)(req->wValue);
      }
      else
      {
        /* Call the error management function (command will be nacked */
        USBD_CtlError (pdev, req);
      }
      break;
    }

    case USB_REQ_CLEAR_FEATURE:
          break;


    default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
  }
  return ret;
}

/**
  * @brief  USBD_CUSTOM_HID_SendReport
  *         Send CUSTOM_HID Report
  * @param  pdev: device instance
  * @param  buff: pointer to report
  * @retval status
  */
uint8_t USBD_CUSTOM_HID_SendReport(USBD_HandleTypeDef *pdev,
                                   uint8_t *report, uint16_t len)
{
  USBD_SMART_GLASS_HandleTypeDef *handle;

  if (pdev->pClassData == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  handle = (USBD_SMART_GLASS_HandleTypeDef *)pdev->pClassData;

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (handle->HID_state == CUSTOM_HID_IDLE)
    {
      handle->HID_state = CUSTOM_HID_BUSY;
      (void)USBD_LL_Transmit(pdev, CUSTOM_HID_EPIN_ADDR, report, len);
    }
    else
    {
      return (uint8_t)USBD_BUSY;
    }
  }
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CUSTOM_HID_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_CUSTOM_HID_ReceivePacket(USBD_HandleTypeDef *pdev)
{
  USBD_SMART_GLASS_HandleTypeDef *hhid;

  if (pdev->pClassData == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  hhid = (USBD_SMART_GLASS_HandleTypeDef *)pdev->pClassData;

  /* Resume USB Out process */
  (void)USBD_LL_PrepareReceive(pdev, CUSTOM_HID_EPOUT_ADDR, hhid->Report_buf,
                               USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_Command_Transmit
  * 		Wrapper to USBD_LL_Transmit function
  * @param  pdev: device instance
  * @param  buf: pointer to buffer containing data
  * @param  length: amount data to be sent
  * @retval status
  */
uint8_t  USBD_Command_Transmit(USBD_HandleTypeDef *pdev, uint8_t* buf, uint16_t length)
{
	USBD_SMART_GLASS_HandleTypeDef *handle;
	handle = (USBD_SMART_GLASS_HandleTypeDef *)pdev->pClassData;

	if (handle->data_in_busy == BUSY_TRUE){
		return USBD_BUSY;
	}
	handle->data_in_busy = BUSY_TRUE;
	pdev->ep_in[COMMAND_EPIN_ADDR & 0x7F].total_length = length;
	return USBD_LL_Transmit(pdev, COMMAND_EPIN_ADDR, buf, length);
}

/**
* @brief  USBD_AUDIO_GetCfgDesc 
*         return configuration descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_AUDIO_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_CLASS_SG_CfgDesc);
  return USBD_CLASS_SG_CfgDesc;
}

/**
* @brief  USBD_AUDIO_DataIn
*         handle data IN Stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
static uint8_t USBD_AUDIO_DataIn (USBD_HandleTypeDef *pdev,
                                  uint8_t epnum)
{
  
  USBD_SMART_GLASS_HandleTypeDef   *handle;
  handle = pdev->pClassData;
//  uint32_t length_usb_pck;
//  uint16_t app;
//  uint16_t IsocInWr_app = handle->wr_ptr;
//  uint16_t true_dim = handle->buffer_length;
//  uint16_t packet_dim = handle->paketDimension;
//  uint16_t channels = handle->channels;
//  length_usb_pck = packet_dim;
  handle->timeout=0;

  if (epnum == (AUDIO_IN_EP & 0x7F))
  {    
    if (handle->state == STATE_USB_IDLE)
    {
      handle->state=STATE_USB_REQUESTS_STARTED;
//      ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->Record();
    }    
    if (handle->state == STATE_USB_BUFFER_WRITE_STARTED)
    {      
      handle->rd_ptr = handle->rd_ptr % (handle->buffer_length);
//      if(IsocInWr_app<handle->rd_ptr){
//        app = ((true_dim) - handle->rd_ptr) +  IsocInWr_app;
//      }else{
//        app = IsocInWr_app - handle->rd_ptr;
//      }
//      if(app >= (packet_dim*handle->upper_treshold)){
//        length_usb_pck += channels*3;
//      }else if(app <= (packet_dim*handle->lower_treshold)){
//        length_usb_pck -= channels*3;
//      }
      USBD_LL_Transmit (pdev,AUDIO_IN_EP,
                        (uint8_t*)(&handle->buffer[handle->rd_ptr]),
						handle->paketDimension);
      handle->rd_ptr += handle->paketDimension;

//      if(app < handle->buffer_length/10)
//      {
//        ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->Stop();
//        handle->state = STATE_USB_IDLE;
//        handle->timeout=0;
//        memset(handle->buffer,0,(handle->buffer_length + handle->dataAmount));
//      }
    }
    else 
    {      
      USBD_LL_Transmit (pdev,AUDIO_IN_EP,
                        IsocInBuffDummy,
						handle->paketDimension);
    }    
  }

  else if(epnum == (CUSTOM_HID_EPIN_ADDR & 0x7F)){
	  ((USBD_SMART_GLASS_HandleTypeDef *)pdev->pClassData)->HID_state = CUSTOM_HID_IDLE;
  }

  else if(epnum == (COMMAND_EPIN_ADDR & 0x7F)){
//	  printf("Data sent");
	  if (pdev->ep_in[epnum].total_length &&
			  !(pdev->ep_in[epnum].total_length % USB_FS_MAX_PACKET_SIZE))
	  {
		  pdev->ep_in[epnum].total_length = 0;
		  USBD_LL_Transmit(pdev, epnum, NULL, 0);
	  }else {
		  handle->data_in_busy = BUSY_FALSE;
	  }
  }
  return USBD_OK;
}

/**
* @brief  USBD_AUDIO_EP0_RxReady
*         handle EP0 Rx Ready event
* @param  pdev: device instance
* @retval status
*/

static uint8_t  USBD_AUDIO_EP0_RxReady (USBD_HandleTypeDef *pdev)
{  
  USBD_SMART_GLASS_HandleTypeDef   *handle;
  handle = pdev->pClassData;
  if (handle->control.cmd == AUDIO_REQ_SET_CUR)
  {    
    if (handle->control.unit == AUDIO_OUT_STREAMING_CTRL)
    {
      ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->VolumeCtl(VOL_CUR);    
      
      handle->control.cmd = 0;
      handle->control.len = 0;
      handle->control.unit = 0;
      handle->control.data[0]=0;
      handle->control.data[0]=0;
    }
  }

  //HID_EP0
  if (handle->IsReportAvailable == 1U){
    ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->OutEvent(handle->Report_buf[0],
                                                              handle->Report_buf[1]);
    handle->IsReportAvailable = 0U;
  }

  return USBD_OK;
}
/**
* @brief  USBD_AUDIO_EP0_TxReady
*         handle EP0 TRx Ready event
* @param  pdev: device instance
* @retval status
*/
static uint8_t  USBD_AUDIO_EP0_TxReady (USBD_HandleTypeDef *pdev)
{
  /* Only OUT control data are processed */
  return USBD_OK;
}
/**
* @brief  USBD_AUDIO_SOF
*         handle SOF event
* @param  pdev: device instance
* @retval status
*/
static uint8_t  USBD_AUDIO_SOF (USBD_HandleTypeDef *pdev)
{  
  return USBD_OK;
}


/**
* @brief  USBD_AUDIO_IsoINIncomplete
*         handle data ISO IN Incomplete event
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
static uint8_t  USBD_AUDIO_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{  
  return USBD_OK;
}
/**
* @brief  USBD_AUDIO_IsoOutIncomplete
*         handle data ISO OUT Incomplete event
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
static uint8_t  USBD_AUDIO_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{  
  return USBD_OK;
}
/**
* @brief  USBD_AUDIO_DataOut
*         handle data OUT Stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
static uint8_t  USBD_AUDIO_DataOut (USBD_HandleTypeDef *pdev, 
                                    uint8_t epnum)
{  
	USBD_SMART_GLASS_HandleTypeDef	*handle;
//	if(epnum == CUSTOM_HID_EPOUT_ADDR){
//		  if (pdev->pClassData == NULL){
//		    return (uint8_t)USBD_FAIL;
//		  }
//
//		  handle = (USBD_SMART_GLASS_HandleTypeDef *)pdev->pClassData;
//
//		  /* USB data will be immediately processed, this allow next USB traffic being
//		  NAKed till the end of the application processing */
//		  ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->OutEvent(handle->Report_buf[0],
//		                                                            handle->Report_buf[1]);
//	}
//
//	else if(epnum == COMMAND_EPOUT_ADDR){
		if(epnum == COMMAND_EPOUT_ADDR){
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		handle = (USBD_SMART_GLASS_HandleTypeDef *)pdev->pClassData;
		//const TickType_t time = pdMS_TO_TICKS(100);
		size_t const bytes_received = USBD_LL_GetRxDataSize(pdev, epnum);

//		printf("Data received\r\n");
		if(xStreamBuffer != NULL){
			xStreamBufferSendFromISR(xStreamBuffer, handle->data_out_buffer, bytes_received, &xHigherPriorityTaskWoken);
		}
		else
		{
			return USBD_FAIL;
		}
		USBD_LL_PrepareReceive(pdev, COMMAND_EPOUT_ADDR, handle->data_out_buffer, USB_FS_MAX_PACKET_SIZE);
		taskYIELD();

	}
		else
		{
			return USBD_FAIL;
		}
  return USBD_OK;
}

/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_AUDIO_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_CLASS_SG_DeviceQualifierDesc);
  return USBD_CLASS_SG_DeviceQualifierDesc;
}

/**
* @brief  AUDIO_REQ_GetMaximum
*         Handles the VOL_MAX Audio control request.
* @param  pdev: instance
* @param  req: setup class request
* @retval status
*/
static void AUDIO_REQ_GetMaximum(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_SMART_GLASS_HandleTypeDef   *handle;
  handle = pdev->pClassData;
  
  (handle->control.data)[0] = (uint16_t)VOL_MAX & 0xFF;
  (handle->control.data)[1] = ((uint16_t)VOL_MAX & 0xFF00 ) >> 8;
  
  USBD_CtlSendData (pdev, 
                    handle->control.data,
                    req->wLength);  
}

/**
* @brief  AUDIO_REQ_GetMinimum
*         Handles the VOL_MIN Audio control request.
* @param  pdev: instance
* @param  req: setup class request
* @retval status
*/
static void AUDIO_REQ_GetMinimum(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_SMART_GLASS_HandleTypeDef   *handle;
  handle = pdev->pClassData;
  (handle->control.data)[0] = (uint16_t)VOL_MIN & 0xFF;
  (handle->control.data)[1] = ((uint16_t)VOL_MIN & 0xFF00 ) >> 8;
  /* Send the current mute state */
  USBD_CtlSendData (pdev, 
                    handle->control.data,
                    req->wLength);   
}

/**
* @brief  AUDIO_Req_GetResolution
*         Handles the VOL_RES Audio control request.
* @param  pdev: instance
* @param  req: setup class request
* @retval status
*/
static void AUDIO_REQ_GetResolution(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_SMART_GLASS_HandleTypeDef   *handle;
  handle = pdev->pClassData;
  (handle->control.data)[0] = (uint16_t)VOL_RES & 0xFF;
  (handle->control.data)[1] = ((uint16_t)VOL_RES & 0xFF00 ) >> 8;
  USBD_CtlSendData (pdev, 
                    handle->control.data,
                    req->wLength);
}

/**
* @brief  AUDIO_Req_GetCurrent
*         Handles the GET_CUR Audio control request.
* @param  pdev: instance
* @param  req: setup class request
* @retval status
*/
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{  
  USBD_SMART_GLASS_HandleTypeDef   *handle;
  handle = pdev->pClassData;
  
  (handle->control.data)[0] = (uint16_t)VOL_CUR & 0xFF;
  (handle->control.data)[1] = ((uint16_t)VOL_CUR & 0xFF00 ) >> 8;
  
  USBD_CtlSendData (pdev, 
                    handle->control.data,
                    req->wLength);  
}

/**
* @brief  AUDIO_Req_SetCurrent
*         Handles the SET_CUR Audio control request.
* @param  pdev: instance
* @param  req: setup class request
* @retval status
*/
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{ 
  USBD_SMART_GLASS_HandleTypeDef   *handle;
  handle = pdev->pClassData;
  if (req->wLength)
  {
    /* Prepare the reception of the buffer over EP0 */
    USBD_CtlPrepareRx (pdev,
                       (uint8_t *)&VOL_CUR,
                       req->wLength);
    
    handle->control.cmd = AUDIO_REQ_SET_CUR;     /* Set the request value */
    handle->control.len = req->wLength;          /* Set the request data length */
    handle->control.unit = HIBYTE(req->wIndex);  /* Set the request target unit */
  }
}


/**
* @}
*/ 

/** @defgroup USBD_AUDIO_IN_Exported_Functions
* @{
*/ 

/**
* @brief  USBD_AUDIO_Data_Transfer
*         Fills the USB internal buffer with audio data from user
* @param pdev: device instance
* @param audioData: audio data to be sent via USB
* @param dataAmount: number of PCM samples to be copyed
* @note Depending on the calling frequency, a coherent amount of samples must be passed to 
*       the function. E.g.: assuming a Sampling frequency of 16 KHz and 1 channel, 
*       you can pass 16 PCM samples if the function is called each millisecond, 
*       32 samples if called every 2 milliseconds and so on. 
* @retval status
*/
uint8_t  USBD_AUDIO_Data_Transfer(USBD_HandleTypeDef *pdev, int16_t * audioData, uint16_t PCMSamples)
{
  
  USBD_SMART_GLASS_HandleTypeDef   *handle;
  handle = (USBD_SMART_GLASS_HandleTypeDef *)pdev->pClassData;
  
  if(handleInstance.state==STATE_USB_WAITING_FOR_INIT){
    return USBD_BUSY;    
  }  
  uint16_t dataAmount = PCMSamples * 3;//2; /*Bytes*/
//  uint16_t true_dim = handle->buffer_length;
//  uint16_t current_data_Amount = handle->dataAmount;
//  uint16_t packet_dim = handle->paketDimension;
  
  if(handle->state==STATE_USB_BUFFER_WRITE_STARTED){
      if(handle->timeout++==TIMEOUT_VALUE){
        handle->state=STATE_USB_IDLE;
        ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->Stop();
       handle->timeout=0;
      }
      memcpy((uint8_t * )&handle->buffer[handle->wr_ptr], (uint8_t *)(audioData), dataAmount);
      handle->wr_ptr += dataAmount;
      handle->wr_ptr = handle->wr_ptr % (handle->buffer_length);
//      if((handle->wr_ptr-dataAmount) == 0){
//        memcpy((uint8_t *)(((uint8_t *)handle->buffer)+handle->buffer_length),(uint8_t *)handle->buffer, dataAmount);
//      }
    }
  else if(handle->state==STATE_USB_REQUESTS_STARTED  || handle->dataAmount!=dataAmount){
    
    /*USB parameters definition, based on the amount of data passed*/
    handle->dataAmount=dataAmount;
    uint16_t wr_rd_offset = (AUDIO_IN_PACKET_NUM/2) * dataAmount / handle->paketDimension;
    handle->wr_ptr=wr_rd_offset * handle->paketDimension;
    handle->rd_ptr = 0;
//    handle->upper_treshold = wr_rd_offset + 1;
//    handle->lower_treshold = wr_rd_offset - 1;
    handle->buffer_length = (handle->paketDimension * (dataAmount / handle->paketDimension) * AUDIO_IN_PACKET_NUM);
    
    /*Memory allocation for data buffer, depending (also) on data amount passed to the transfer function*/
    if(handle->buffer != NULL)
    {
      USBD_free(handle->buffer);
    }
    handle->buffer = USBD_malloc(handle->buffer_length + handle->dataAmount);
    if(handle->buffer == NULL)
    {
      return USBD_FAIL;       
    }
    memset(handle->buffer,0,(handle->buffer_length + handle->dataAmount));
    handle->state=STATE_USB_BUFFER_WRITE_STARTED;
    
    
  }
  return USBD_OK;  
}


/**
* @brief  USBD_AUDIO_RegisterInterface
* @param  fops: Audio interface callback
* @retval status
*/
uint8_t  USBD_AUDIO_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                        USBD_AUDIO_ItfTypeDef *fops)
{
  if(fops != NULL)
  {
    pdev->pUserData= fops;
  }
  return 0;}

/**
* @brief  Configures the microphone descriptor on the base of the frequency 
*         and channels number informations. These parameters will be used to
*         init the audio engine, trough the USB interface functions.
* @param  samplingFrequency: sampling frequency
* @param  Channels: number of channels
* @retval status
*/
void USBD_AUDIO_Init_Microphone_Descriptor(USBD_HandleTypeDef   *pdev, uint32_t samplingFrequency, uint8_t Channels)
{
  uint16_t index = 0;
  uint8_t AUDIO_CONTROLS;
  USBD_CLASS_SG_CfgDesc[index++] = 0x09;                                                /* bLength */
  USBD_CLASS_SG_CfgDesc[index++] = 0x02;                                                /* bDescriptorType */
  USBD_CLASS_SG_CfgDesc[index++] = ((USB_CLASS_SG_CONFIG_DESC_SIZ+Channels-1)&0xff);       /* wTotalLength */
  USBD_CLASS_SG_CfgDesc[index++] = ((USB_CLASS_SG_CONFIG_DESC_SIZ+Channels-1)>>8);
  USBD_CLASS_SG_CfgDesc[index++] = 0x04;                                                /* bNumInterfaces */
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                                                /* bConfigurationValue */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                                /* iConfiguration */
  USBD_CLASS_SG_CfgDesc[index++] = 0x80;                                                /* bmAttributes  BUS Powered*/
  USBD_CLASS_SG_CfgDesc[index++] = 0x32;                                                /* bMaxPower = 100 mA*/
  /*IAD to associate the HID interfaces */
    USBD_CLASS_SG_CfgDesc[index++] = 0x08; /* bLength */
    USBD_CLASS_SG_CfgDesc[index++] = 0x0B; /* bDescriptorType */
    USBD_CLASS_SG_CfgDesc[index++] = 0x00; /* bFirstInterface */
    USBD_CLASS_SG_CfgDesc[index++] = 0x03; /* bInterfaceCount */
    USBD_CLASS_SG_CfgDesc[index++] = USB_DEVICE_CLASS_AUDIO; /* bFunctionClass */
    USBD_CLASS_SG_CfgDesc[index++] = 0x00; /* bFunctionSubClass */
    USBD_CLASS_SG_CfgDesc[index++] = 0x00; /* bFunctionProtocol */
    USBD_CLASS_SG_CfgDesc[index++] = 0x00; /* iFunction (Index of string descriptor describing this function) */
  /* USB Microphone Standard interface descriptor */
  USBD_CLASS_SG_CfgDesc[index++] = 9;                                                   /* bLength */
  USBD_CLASS_SG_CfgDesc[index++] = USB_INTERFACE_DESCRIPTOR_TYPE;                      /* bDescriptorType */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                               /* bInterfaceNumber */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                               /* bAlternateSetting */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                               /* bNumEndpoints */
  USBD_CLASS_SG_CfgDesc[index++] = USB_DEVICE_CLASS_AUDIO;                             /* bInterfaceClass */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_SUBCLASS_AUDIOCONTROL;                        /* bInterfaceSubClass */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_PROTOCOL_UNDEFINED;                           /* bInterfaceProtocol */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                               /* iInterface */
  /* USB Microphone Class-specific AC Interface Descriptor */
  USBD_CLASS_SG_CfgDesc[index++] = 9;                                                  /* bLength */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_INTERFACE_DESCRIPTOR_TYPE;                    /* bDescriptorType */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_CONTROL_HEADER;                               /* bDescriptorSubtype */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;       /* 1.00 */                              /* bcdADC */
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;
  USBD_CLASS_SG_CfgDesc[index++] = 0x25+Channels;                                      /* wTotalLength = 37+AUDIO_CHANNELS*/
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                                               /* bInCollection */
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                                               /* baInterfaceNr */
  /* USB Microphone Input Terminal Descriptor */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_INPUT_TERMINAL_DESC_SIZE;                     /* bLength */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_INTERFACE_DESCRIPTOR_TYPE;                    /* bDescriptorType */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_CONTROL_INPUT_TERMINAL;                       /* bDescriptorSubtype */
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                                               /* bTerminalID */
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                                               /* wTerminalType AUDIO_TERMINAL_USB_MICROPHONE   0x0201 */
  USBD_CLASS_SG_CfgDesc[index++] = 0x02;
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                               /* bAssocTerminal */
  USBD_CLASS_SG_CfgDesc[index++] = Channels;                                           /* bNrChannels */
  if(Channels != 2)
  {
    USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                             /* wChannelConfig 0x0000  Mono */
    USBD_CLASS_SG_CfgDesc[index++] = 0x00;
  }
  else
  {
    USBD_CLASS_SG_CfgDesc[index++] = 0x03;                                             /* wChannelConfig 0x0003  Stereo */
    USBD_CLASS_SG_CfgDesc[index++] = 0x00;
  }   
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                               /* iChannelNames */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                               /* iTerminal */
  /* USB Microphone Audio Feature Unit Descriptor */
  USBD_CLASS_SG_CfgDesc[index++] = 0x07+Channels+1;                                    /* bLength */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_INTERFACE_DESCRIPTOR_TYPE;                    /* bDescriptorType */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_CONTROL_FEATURE_UNIT;                         /* bDescriptorSubtype */
  USBD_CLASS_SG_CfgDesc[index++] = 0x02;                                               /* bUnitID */
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                                               /* bSourceID */
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                                               /* bControlSize */
  //index = 47;
  if(Channels == 1)
  {
    AUDIO_CONTROLS = (0x02);     
    USBD_CLASS_SG_CfgDesc[index++] = AUDIO_CONTROLS;
    USBD_CLASS_SG_CfgDesc[index++] = 0x00;
  }
  else
  {
    AUDIO_CONTROLS = (0x02);     
    USBD_CLASS_SG_CfgDesc[index++] = 0x00;
    USBD_CLASS_SG_CfgDesc[index++] = AUDIO_CONTROLS;
    USBD_CLASS_SG_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }   
  if(Channels > 2)
  {
    USBD_CLASS_SG_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }   
  if(Channels > 3)
  {
    USBD_CLASS_SG_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }   
  if(Channels > 4)
  {
    USBD_CLASS_SG_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }   
  if(Channels > 5)
  {
    USBD_CLASS_SG_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }   
  if(Channels > 6)
  {
    USBD_CLASS_SG_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }   
  if(Channels > 7)
  {
    USBD_CLASS_SG_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }   
  USBD_CLASS_SG_CfgDesc[index] = 0x00;                                            /* iTerminal */
  index++;
  /*USB Microphone Output Terminal Descriptor */
  USBD_CLASS_SG_CfgDesc[index++] = 0x09;                                          /* bLength */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_INTERFACE_DESCRIPTOR_TYPE;               /* bDescriptorType */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_CONTROL_OUTPUT_TERMINAL;                 /* bDescriptorSubtype */
  USBD_CLASS_SG_CfgDesc[index++] = 0x03;                                          /* bTerminalID */
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                                          /* wTerminalType AUDIO_TERMINAL_USB_STREAMING 0x0101*/
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;
  USBD_CLASS_SG_CfgDesc[index++] = 0x02;
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;
  /* USB Microphone Standard AS Interface Descriptor - Audio Streaming Zero Bandwith */
  /* Interface 1, Alternate Setting 0                                             */
  USBD_CLASS_SG_CfgDesc[index++] = 9;                                             /* bLength */
  USBD_CLASS_SG_CfgDesc[index++] = USB_INTERFACE_DESCRIPTOR_TYPE;                 /* bDescriptorType */
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                                          /* bInterfaceNumber */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                          /* bAlternateSetting */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                          /* bNumEndpoints */
  USBD_CLASS_SG_CfgDesc[index++] = USB_DEVICE_CLASS_AUDIO;                        /* bInterfaceClass */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_SUBCLASS_AUDIOSTREAMING;                 /* bInterfaceSubClass */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_PROTOCOL_UNDEFINED;                      /* bInterfaceProtocol */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;
  /* USB Microphone Standard AS Interface Descriptor - Audio Streaming Operational */
  /* Interface 1, Alternate Setting 1                                           */
  USBD_CLASS_SG_CfgDesc[index++] = 9;                                             /* bLength */
  USBD_CLASS_SG_CfgDesc[index++] = USB_INTERFACE_DESCRIPTOR_TYPE;                 /* bDescriptorType */
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                                          /* bInterfaceNumber */
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                                          /* bAlternateSetting */
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                                          /* bNumEndpoints */
  USBD_CLASS_SG_CfgDesc[index++] = USB_DEVICE_CLASS_AUDIO;                        /* bInterfaceClass */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_SUBCLASS_AUDIOSTREAMING;                 /* bInterfaceSubClass */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_PROTOCOL_UNDEFINED;                      /* bInterfaceProtocol */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                          /* iInterface */
  /* USB Microphone Audio Streaming Interface Descriptor */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_STREAMING_INTERFACE_DESC_SIZE;           /* bLength */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_INTERFACE_DESCRIPTOR_TYPE;               /* bDescriptorType */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_STREAMING_GENERAL;                       /* bDescriptorSubtype */
  USBD_CLASS_SG_CfgDesc[index++] = 0x03;                                          /* bTerminalLink */
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                                          /* bDelay */
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                                          /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;
  /* USB Microphone Audio Type I Format Interface Descriptor */                
  USBD_CLASS_SG_CfgDesc[index++] = 0x0B;                                          /* bLength */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_INTERFACE_DESCRIPTOR_TYPE;               /* bDescriptorType */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_STREAMING_FORMAT_TYPE;                   /* bDescriptorSubtype */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_FORMAT_TYPE_I;                           /* bFormatType */
  USBD_CLASS_SG_CfgDesc[index++] = Channels;                                      /* bNrChannels */
  USBD_CLASS_SG_CfgDesc[index++] = 0x03;                                          /* bSubFrameSize */
  USBD_CLASS_SG_CfgDesc[index++] = 24;                                            /* bBitResolution */
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                                           /* bSamFreqType */
  USBD_CLASS_SG_CfgDesc[index++] = samplingFrequency&0xff;                        /* tSamFreq 8000 = 0x1F40 */
  USBD_CLASS_SG_CfgDesc[index++] = (samplingFrequency>>8)&0xff;
  USBD_CLASS_SG_CfgDesc[index++] = samplingFrequency>>16;
  /* Endpoint 1 - Standard Descriptor */
  USBD_CLASS_SG_CfgDesc[index++] =  AUDIO_STANDARD_ENDPOINT_DESC_SIZE;            /* bLength */
  USBD_CLASS_SG_CfgDesc[index++] = 0x05;                                          /* bDescriptorType */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_IN_EP;                                   /* bEndpointAddress 1 in endpoint*/
  USBD_CLASS_SG_CfgDesc[index++] = 0x05;                                          /* bmAttributes */
  USBD_CLASS_SG_CfgDesc[index++] = ((samplingFrequency/1000+2)*Channels*3)&0xFF;  /* wMaxPacketSize */
  USBD_CLASS_SG_CfgDesc[index++] = ((samplingFrequency/1000+2)*Channels*3)>>8;
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                                          /* bInterval */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                          /* bRefresh */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                          /* bSynchAddress */
  /* Endpoint - Audio Streaming Descriptor*/
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_STREAMING_ENDPOINT_DESC_SIZE;            /* bLength */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_ENDPOINT_DESCRIPTOR_TYPE;                /* bDescriptorType */
  USBD_CLASS_SG_CfgDesc[index++] = AUDIO_ENDPOINT_GENERAL;                        /* bDescriptor */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                          /* bmAttributes */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                          /* bLockDelayUnits */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                                          /* wLockDelay */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;
  /* Interface */
  USBD_CLASS_SG_CfgDesc[index++] = 0x09;                     /* bLength */
  USBD_CLASS_SG_CfgDesc[index++] = USB_DESC_TYPE_INTERFACE;  /* bDescriptorType: */
  USBD_CLASS_SG_CfgDesc[index++] = 0x02;                     /* bInterfaceNumber */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                     /* bAlternateSetting */
  USBD_CLASS_SG_CfgDesc[index++] = 0x02;                     /* bNumEndpoints */
  USBD_CLASS_SG_CfgDesc[index++] = 0x03;                     /* bInterfaceClass */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                     /* bInterfaceSubClass */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                     /* bInterfaceProtocol */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                     /* iInterface */
  /* Descriptor of CUSTOM_HID */
  USBD_CLASS_SG_CfgDesc[index++] = 0x09;                     /* bLength */
  USBD_CLASS_SG_CfgDesc[index++] = CUSTOM_HID_DESCRIPTOR_TYPE;  /* bDescriptorType: */
  USBD_CLASS_SG_CfgDesc[index++] = 0x11;
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;
  USBD_CLASS_SG_CfgDesc[index++] = 0x01;
  USBD_CLASS_SG_CfgDesc[index++] = 0x22;
  USBD_CLASS_SG_CfgDesc[index++] = USBD_CUSTOM_HID_REPORT_DESC_SIZE;
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;
  /* Endpoint OUT */
  USBD_CLASS_SG_CfgDesc[index++] = 0x07;                             /* bLength */
  USBD_CLASS_SG_CfgDesc[index++] = USB_DESC_TYPE_ENDPOINT;           /* bDescriptorType */
  USBD_CLASS_SG_CfgDesc[index++] = CUSTOM_HID_EPOUT_ADDR;               /* bEndpointAddress */
  USBD_CLASS_SG_CfgDesc[index++] = 0x03;                             /* bmAttributes */
  USBD_CLASS_SG_CfgDesc[index++] = CUSTOM_HID_EPOUT_SIZE;   /* wMaxPacketSize */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;
  USBD_CLASS_SG_CfgDesc[index++] = CUSTOM_HID_FS_BINTERVAL;                              /* bInterval */
	    /* Endpoint IN */
  USBD_CLASS_SG_CfgDesc[index++] = 0x07;                             /* bLength */
  USBD_CLASS_SG_CfgDesc[index++] = USB_DESC_TYPE_ENDPOINT;           /* bDescriptorType */
  USBD_CLASS_SG_CfgDesc[index++] = CUSTOM_HID_EPIN_ADDR;               /* bEndpointAddress */
  USBD_CLASS_SG_CfgDesc[index++] = 0x03;                             /* bmAttributes */
  USBD_CLASS_SG_CfgDesc[index++] = CUSTOM_HID_EPIN_SIZE;   /* wMaxPacketSize */
  USBD_CLASS_SG_CfgDesc[index++] = 0x00;
  USBD_CLASS_SG_CfgDesc[index++] = CUSTOM_HID_FS_BINTERVAL;                              /* bInterval */
  /* Interface */
  	  USBD_CLASS_SG_CfgDesc[index++] = 0x09;                     /* bLength */
  	  USBD_CLASS_SG_CfgDesc[index++] = USB_DESC_TYPE_INTERFACE;  /* bDescriptorType: */
  	  USBD_CLASS_SG_CfgDesc[index++] = 0x03;                     /* bInterfaceNumber */
  	  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                     /* bAlternateSetting */
  	  USBD_CLASS_SG_CfgDesc[index++] = 0x02;                     /* bNumEndpoints */
  	  USBD_CLASS_SG_CfgDesc[index++] = 0x0A;                     /* bInterfaceClass */
  	  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                     /* bInterfaceSubClass */
  	  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                     /* bInterfaceProtocol */
  	  USBD_CLASS_SG_CfgDesc[index++] = 0x00;                     /* iInterface */
  	    /* Endpoint OUT */
  	  USBD_CLASS_SG_CfgDesc[index++] = 0x07;                             /* bLength */
  	  USBD_CLASS_SG_CfgDesc[index++] = USB_DESC_TYPE_ENDPOINT;           /* bDescriptorType */
  	  USBD_CLASS_SG_CfgDesc[index++] = COMMAND_EPOUT_ADDR;               /* bEndpointAddress */
  	  USBD_CLASS_SG_CfgDesc[index++] = 0x02;                             /* bmAttributes */
  	  USBD_CLASS_SG_CfgDesc[index++] = LOBYTE(USB_FS_MAX_PACKET_SIZE);   /* wMaxPacketSize */
  	  USBD_CLASS_SG_CfgDesc[index++] = HIBYTE(USB_FS_MAX_PACKET_SIZE);
  	  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                              /* bInterval */
  	    /* Endpoint IN */
  	  USBD_CLASS_SG_CfgDesc[index++] = 0x07;                             /* bLength */
  	  USBD_CLASS_SG_CfgDesc[index++] = USB_DESC_TYPE_ENDPOINT;           /* bDescriptorType */
  	  USBD_CLASS_SG_CfgDesc[index++] = COMMAND_EPIN_ADDR;               /* bEndpointAddress */
  	  USBD_CLASS_SG_CfgDesc[index++] = 0x02;                             /* bmAttributes */
  	  USBD_CLASS_SG_CfgDesc[index++] = LOBYTE(USB_FS_MAX_PACKET_SIZE);   /* wMaxPacketSize */
  	  USBD_CLASS_SG_CfgDesc[index++] = HIBYTE(USB_FS_MAX_PACKET_SIZE);
  	  USBD_CLASS_SG_CfgDesc[index++] = 0x01;                              /* bInterval */
    
  handleInstance.paketDimension = (samplingFrequency/1000*Channels*3);
  handleInstance.frequency=samplingFrequency;
  handleInstance.buffer_length = handleInstance.paketDimension * AUDIO_IN_PACKET_NUM;
  handleInstance.channels=Channels;
  handleInstance.upper_treshold = 5;
  handleInstance.lower_treshold = 2;
  handleInstance.state = STATE_USB_WAITING_FOR_INIT;
  handleInstance.wr_ptr = 3 * handleInstance.paketDimension;
  handleInstance.rd_ptr = 0;
  handleInstance.dataAmount=0;
  handleInstance.buffer = 0;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
