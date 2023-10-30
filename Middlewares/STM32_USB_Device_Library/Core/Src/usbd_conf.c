/**
  ******************************************************************************
  * @file    usbd_conf_template.c
  * @author  MCD Application Team
  * @brief   USB Device configuration and interface file
  *          This template should be copied to the user folder,
  *          renamed and customized following user needs.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"
#include "usbd_smart_glass.h" /* Include class header file */
extern UART_HandleTypeDef huart2;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */

PCD_HandleTypeDef hpcd_USB_OTG_FS;
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
	hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	  hpcd_USB_OTG_FS.Init.dev_endpoints = 3;
	  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
	  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
	  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
	  {
	  }
	  hpcd_USB_OTG_FS.pData = pdev;
	  pdev->pData = &hpcd_USB_OTG_FS;
	  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

	  /* USER CODE END USB_OTG_FS_Init 2 */

	  HAL_PCD_SetRxFiFo(&hpcd_USB_OTG_FS, 0x50);
	  HAL_PCD_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);
	  HAL_PCD_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x54);
	  HAL_PCD_SetTxFiFo(&hpcd_USB_OTG_FS, 2, 0x30);
	  HAL_PCD_SetTxFiFo(&hpcd_USB_OTG_FS, 3, 0x20);
      return USBD_OK;
}

/**
  * @brief  De-Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
	HAL_PCD_DeInit(pdev->pData);
	  return USBD_OK;
}

/**
  * @brief  Starts the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
	HAL_PCD_Start(pdev->pData);
	  return USBD_OK;
}

/**
  * @brief  Stops the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
	HAL_PCD_Stop(pdev->pData);
	  return USBD_OK;
}

/**
  * @brief  Opens an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  ep_type: Endpoint Type
  * @param  ep_mps: Endpoint Max Packet Size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                  uint8_t ep_type, uint16_t ep_mps)
{
	HAL_PCD_EP_Open(pdev->pData,
	                  ep_addr,
	                  ep_mps,
	                  ep_type);

	  return USBD_OK;
}

/**
  * @brief  Closes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
	HAL_PCD_EP_Close(pdev->pData, ep_addr);
	  return USBD_OK;
}

/**
  * @brief  Flushes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
	HAL_PCD_EP_Flush(pdev->pData, ep_addr);
	  return USBD_OK;
}

/**
  * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
	 HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
	  return USBD_OK;
}

/**
  * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev,
                                        uint8_t ep_addr)
{
	HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
	  return USBD_OK;
}

/**
  * @brief  Returns Stall condition.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Stall (1: Yes, 0: No)
  */
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
	PCD_HandleTypeDef *hpcd = pdev->pData;

	  if((ep_addr & 0x80) == 0x80)
	  {
	    return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
	  }
	  else
	  {
	    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
	  }
}

/**
  * @brief  Assigns a USB address to the device.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev,
                                         uint8_t dev_addr)
{
	HAL_PCD_SetAddress(pdev->pData, dev_addr);
	  return USBD_OK;
}

/**
  * @brief  Transmits data over an endpoint.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be sent
  * @param  size: Data size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                    uint8_t *pbuf, uint32_t size)
{
	HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
	  return USBD_OK;
}

/**
  * @brief  Prepares an endpoint for reception.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be received
  * @param  size: Data size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev,
                                          uint8_t ep_addr, uint8_t *pbuf,
                                          uint32_t size)
{
	HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
	  return USBD_OK;
}

/**
  * @brief  Returns the last transferred packet size.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Received Data Size
  */
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
	return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

/**
  * @brief  Delays routine for the USB Device Library.
  * @param  Delay: Delay in ms
  * @retval None
  */
void USBD_LL_Delay(uint32_t Delay)
{
	HAL_Delay(Delay);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

