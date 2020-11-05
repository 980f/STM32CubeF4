/**
  ******************************************************************************
  * @file    usbh_ctlreq.c
  * @author  MCD Application Team
  * @brief   This file implements the control requests for device enumeration
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
#include "usbh_ctlreq.h"

/** @addtogroup USBH_LIB
* @{
*/

/** @addtogroup USBH_LIB_CORE
* @{
*/

/** @defgroup USBH_CTLREQ
* @brief This file implements the standard requests for device enumeration
* @{
*/

/** @defgroup USBH_CTLREQ_Private_FunctionPrototypes
* @{
*/
static USBH_StatusTypeDef USBH_HandleControl(USBH_HandleTypeDef *phost);
static void USBH_ParseDevDesc(USBH_DevDescTypeDef *dev_desc, uint8_t *buf, uint16_t length);
static void USBH_ParseCfgDesc(USBH_CfgDescTypeDef *cfg_desc, uint8_t *buf, uint16_t length);
static void USBH_ParseEPDesc(USBH_EpDescTypeDef *ep_descriptor, uint8_t *buf);
static void USBH_ParseStringDesc(uint8_t *psrc, uint8_t *pdest, uint16_t length);
static void USBH_ParseInterfaceDesc(USBH_InterfaceDescTypeDef *if_descriptor, uint8_t *buf);

/**
* @}
*/

/** @defgroup USBH_CTLREQ_Private_Functions
* @{
*/

//why didn't they do this? It allows for someone to tap the OS event stream with their own simple override.
#if (USBH_USE_OS == 1U)
static void postEvent(USBH_HandleTypeDef *phost){
  phost->os_msg = (uint32_t)USBH_CONTROL_EVENT;
#if (osCMSIS < 0x20000U)
  (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
  (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
}
#else
//ignore totally or put a weak function here and override it
#define postEvent(phost)
#endif

static void changeState(USBH_HandleTypeDef *phost, CTRL_StateTypeDef controlState, _Bool andPost) {
  phost->Control.state = controlState;
  if (andPost) {
    postEvent(phost);
  }
}

/**
  * @brief  USBH_Get_DevDesc
  *         Issue Get Device Descriptor command to the device. Once the response
  *         received, it parses the device descriptor and updates the status.
  * @param  phost: Host Handle
  * @param  length: Length of the descriptor
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_Get_DevDesc(USBH_HandleTypeDef *phost, uint8_t length) {
  USBH_StatusTypeDef status=USBH_GetDescriptor(phost, USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD, USB_DESC_DEVICE, phost->device.Data, (uint16_t) length);
  if (status== USBH_OK) { /* Commands successfully sent and Response Received */
    USBH_ParseDevDesc(&phost->device.DevDesc, phost->device.Data, (uint16_t) length);
  }
  return status;
}


/**
  * @brief  USBH_Get_CfgDesc
  *         Issues Configuration Descriptor to the device. Once the response
  *         received, it parses the configuration descriptor and updates the
  *         status.
  * @param  phost: Host Handle
  * @param  length: Length of the descriptor
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_Get_CfgDesc(USBH_HandleTypeDef *phost, uint16_t length) {
  uint8_t *pData = phost->device.CfgDesc_Raw;;
  USBH_StatusTypeDef status=USBH_GetDescriptor(phost, (USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD), USB_DESC_CONFIGURATION, pData, length);
  if (status == USBH_OK) {  /* Commands successfully sent and Response Received  */
    USBH_ParseCfgDesc(&phost->device.CfgDesc, pData, length);
  }
  return status;
}


/**
  * @brief  USBH_Get_StringDesc
  *         Issues string Descriptor command to the device. Once the response
  *         received, it parses the string descriptor and updates the status.
  * @param  phost: Host Handle
  * @param  string_index: String index for the descriptor
  * @param  buff: Buffer address for the descriptor
  * @param  length: Length of the descriptor
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_Get_StringDesc(USBH_HandleTypeDef *phost,uint8_t string_index, uint8_t *buff,uint16_t length) {
  USBH_StatusTypeDef status=USBH_GetDescriptor(phost,
                                               USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD,
                                               USB_DESC_STRING | string_index,
                                               phost->device.Data, length);
  if (status == USBH_OK) { /* Commands successfully sent and Response Received  */
    USBH_ParseStringDesc(phost->device.Data, buff, length);
  }
  return status;
}


/**
  * @brief  USBH_GetDescriptor
  *         Issues Descriptor command to the device. Once the response received,
  *         it parses the descriptor and updates the status.
  * @param  phost: Host Handle
  * @param  req_type: Descriptor type
  * @param  value_idx: Value for the GetDescriptr request
  * @param  buff: Buffer to store the descriptor
  * @param  length: Length of the descriptor
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_GetDescriptor(USBH_HandleTypeDef *phost,
                                      uint8_t req_type, uint16_t value_idx,
                                      uint8_t *buff, uint16_t length) {
  if (phost->RequestState == CMD_SEND) {
    phost->Control.setup.b.bmRequestType = USB_D2H | req_type;
    phost->Control.setup.b.bRequest = USB_REQ_GET_DESCRIPTOR;
    phost->Control.setup.b.wValue.w = value_idx;
    phost->Control.setup.b.wIndex.w = (value_idx & 0xff00U) == USB_DESC_STRING ? 0x0409U : 0U;
    phost->Control.setup.b.wLength.w = length;
  }
  return USBH_CtlReq(phost, buff, length);
}


/**
  * @brief  USBH_SetAddress
  *         This command sets the address to the connected device
  * @param  phost: Host Handle
  * @param  DeviceAddress: Device address to assign
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_SetAddress(USBH_HandleTypeDef *phost,
                                   uint8_t DeviceAddress) {
  if (phost->RequestState == CMD_SEND) {
    phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD;
    phost->Control.setup.b.bRequest = USB_REQ_SET_ADDRESS;
    phost->Control.setup.b.wValue.w = (uint16_t) DeviceAddress;
    phost->Control.setup.b.wIndex.w = 0U;
    phost->Control.setup.b.wLength.w = 0U;
  }

  return USBH_CtlReq(phost, 0U, 0U);
}


/**
  * @brief  USBH_SetCfg
  *         The command sets the configuration value to the connected device
  * @param  phost: Host Handle
  * @param  cfg_idx: Configuration value
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_SetCfg(USBH_HandleTypeDef *phost, uint16_t cfg_idx) {
  if (phost->RequestState == CMD_SEND) {
    phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_RECIPIENT_DEVICE| USB_REQ_TYPE_STANDARD;
    phost->Control.setup.b.bRequest = USB_REQ_SET_CONFIGURATION;
    phost->Control.setup.b.wValue.w = cfg_idx;
    phost->Control.setup.b.wIndex.w = 0U;
    phost->Control.setup.b.wLength.w = 0U;
  }

  return USBH_CtlReq(phost, 0U, 0U);
}


/**
  * @brief  USBH_SetInterface
  *         The command sets the Interface value to the connected device
  * @param  phost: Host Handle
  * @param  altSetting: Interface value
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_SetInterface(USBH_HandleTypeDef *phost, uint8_t ep_num,
                                     uint8_t altSetting) {
  if (phost->RequestState == CMD_SEND) {
    phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_RECIPIENT_INTERFACE | USB_REQ_TYPE_STANDARD;

    phost->Control.setup.b.bRequest = USB_REQ_SET_INTERFACE;
    phost->Control.setup.b.wValue.w = altSetting;
    phost->Control.setup.b.wIndex.w = ep_num;
    phost->Control.setup.b.wLength.w = 0U;
  }

  return USBH_CtlReq(phost, 0U, 0U);
}


/**
  * @brief  USBH_SetFeature
  *         The command sets the device features (remote wakeup feature,..)
  * @param  pdev: Selected device
  * @param  itf_idx
  * @retval Status
*/
USBH_StatusTypeDef USBH_SetFeature(USBH_HandleTypeDef *phost, uint8_t wValue) {
  if (phost->RequestState == CMD_SEND) {
    phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD;

    phost->Control.setup.b.bRequest = USB_REQ_SET_FEATURE;
    phost->Control.setup.b.wValue.w = wValue;
    phost->Control.setup.b.wIndex.w = 0U;
    phost->Control.setup.b.wLength.w = 0U;
  }

  return USBH_CtlReq(phost, 0U, 0U);
}


/**
  * @brief  USBH_ClrFeature
  *         This request is used to clear or disable a specific feature.
  * @param  phost: Host Handle
  * @param  ep_num: endpoint number
  * @param  hc_num: Host channel number
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_ClrFeature(USBH_HandleTypeDef *phost, uint8_t ep_num) {
  if (phost->RequestState == CMD_SEND) {
    phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_RECIPIENT_ENDPOINT | USB_REQ_TYPE_STANDARD;

    phost->Control.setup.b.bRequest = USB_REQ_CLEAR_FEATURE;
    phost->Control.setup.b.wValue.w = FEATURE_SELECTOR_ENDPOINT;
    phost->Control.setup.b.wIndex.w = ep_num;
    phost->Control.setup.b.wLength.w = 0U;
  }
  return USBH_CtlReq(phost, 0U, 0U);
}


/**
  * @brief  USBH_ParseDevDesc
  *         This function Parses the device descriptor
  * @param  dev_desc: device_descriptor destination address
  * @param  buf: Buffer where the source descriptor is available
  * @param  length: Length of the descriptor
  * @retval None
  */
static void USBH_ParseDevDesc(USBH_DevDescTypeDef *dev_desc, uint8_t *buf, uint16_t length) {
  dev_desc->bLength = buf[0];//seriously, old code here might as well be using asm!
  dev_desc->bDescriptorType = *(uint8_t *) (buf + 1);
  dev_desc->bcdUSB = LE16(buf + 2);
  dev_desc->bDeviceClass = *(uint8_t *) (buf + 4);
  dev_desc->bDeviceSubClass = *(uint8_t *) (buf + 5);
  dev_desc->bDeviceProtocol = *(uint8_t *) (buf + 6);
  dev_desc->bMaxPacketSize = *(uint8_t *) (buf + 7);

  if (length > 8U) {
    /* For 1st time after device connection, Host may issue only 8 bytes for
    Device Descriptor Length  */
    dev_desc->idVendor = LE16(buf + 8);
    dev_desc->idProduct = LE16(buf + 10);
    dev_desc->bcdDevice = LE16(buf + 12);
    dev_desc->iManufacturer = *(uint8_t *) (buf + 14);
    dev_desc->iProduct = *(uint8_t *) (buf + 15);
    dev_desc->iSerialNumber = *(uint8_t *) (buf + 16);
    dev_desc->bNumConfigurations = *(uint8_t *) (buf + 17);
  }
}


/**
  * @brief  USBH_ParseCfgDesc
  *         This function Parses the configuration descriptor
  * @param  cfg_desc: Configuration Descriptor address
  * @param  buf: Buffer where the source descriptor is available
  * @param  length: Length of the descriptor
  * @retval None
  */
static void USBH_ParseCfgDesc(USBH_CfgDescTypeDef *cfg_desc, uint8_t *buf,
                              uint16_t length) {
  USBH_InterfaceDescTypeDef *pif;
  USBH_EpDescTypeDef *pep;
  USBH_DescHeader_t *pdesc;// = (USBH_DescHeader_t *) (void *) buf;
  uint16_t ptr;
  uint8_t if_ix = 0U;
  uint8_t ep_ix;//= 0U;

  pdesc = (USBH_DescHeader_t *) (void *) buf;

  /* Parse configuration descriptor */
  cfg_desc->bLength = *(uint8_t *) (buf + 0);
  cfg_desc->bDescriptorType = *(uint8_t *) (buf + 1);
  cfg_desc->wTotalLength = LE16(buf + 2);
  cfg_desc->bNumInterfaces = *(uint8_t *) (buf + 4);
  cfg_desc->bConfigurationValue = *(uint8_t *) (buf + 5);
  cfg_desc->iConfiguration = *(uint8_t *) (buf + 6);
  cfg_desc->bmAttributes = *(uint8_t *) (buf + 7);
  cfg_desc->bMaxPower = *(uint8_t *) (buf + 8);

  if (length > USB_CONFIGURATION_DESC_SIZE) {
    ptr = USB_LEN_CFG_DESC;
//    pif = (USBH_InterfaceDescTypeDef *) 0;

    while ((if_ix < USBH_MAX_NUM_INTERFACES) && (ptr < cfg_desc->wTotalLength)) {
      pdesc = USBH_GetNextDesc((uint8_t *) (void *) pdesc, &ptr);
      if (pdesc->bDescriptorType == USB_DESC_TYPE_INTERFACE) {
        pif = &cfg_desc->Itf_Desc[if_ix];
        USBH_ParseInterfaceDesc(pif, (uint8_t *) (void *) pdesc);

        ep_ix = 0U;
//        pep = (USBH_EpDescTypeDef *) 0;
        while ((ep_ix < pif->bNumEndpoints) && (ptr < cfg_desc->wTotalLength)) {
          pdesc = USBH_GetNextDesc((uint8_t *) (void *) pdesc, &ptr);
          if (pdesc->bDescriptorType == USB_DESC_TYPE_ENDPOINT) {
            pep = &cfg_desc->Itf_Desc[if_ix].Ep_Desc[ep_ix];
            USBH_ParseEPDesc(pep, (uint8_t *) (void *) pdesc);
            ep_ix++;
          }
        }
        if_ix++;
      }
    }
  }
}


/**
  * @brief  USBH_ParseInterfaceDesc
  *         This function Parses the interface descriptor
  * @param  if_descriptor : Interface descriptor destination
  * @param  buf: Buffer where the descriptor data is available
  * @retval None
  */
static void USBH_ParseInterfaceDesc(USBH_InterfaceDescTypeDef *if_descriptor,
                                    uint8_t *buf) {
  if_descriptor->bLength = *(uint8_t *) (buf + 0);
  if_descriptor->bDescriptorType = *(uint8_t *) (buf + 1);
  if_descriptor->bInterfaceNumber = *(uint8_t *) (buf + 2);
  if_descriptor->bAlternateSetting = *(uint8_t *) (buf + 3);
  if_descriptor->bNumEndpoints = *(uint8_t *) (buf + 4);
  if_descriptor->bInterfaceClass = *(uint8_t *) (buf + 5);
  if_descriptor->bInterfaceSubClass = *(uint8_t *) (buf + 6);
  if_descriptor->bInterfaceProtocol = *(uint8_t *) (buf + 7);
  if_descriptor->iInterface = *(uint8_t *) (buf + 8);
}


/**
  * @brief  USBH_ParseEPDesc
  *         This function Parses the endpoint descriptor
  * @param  ep_descriptor: Endpoint descriptor destination address
  * @param  buf: Buffer where the parsed descriptor stored
  * @retval None
  */
static void USBH_ParseEPDesc(USBH_EpDescTypeDef *ep_descriptor,
                             uint8_t *buf) {
  ep_descriptor->bLength = *(uint8_t *) (buf + 0);
  ep_descriptor->bDescriptorType = *(uint8_t *) (buf + 1);
  ep_descriptor->bEndpointAddress = *(uint8_t *) (buf + 2);
  ep_descriptor->bmAttributes = *(uint8_t *) (buf + 3);
  ep_descriptor->wMaxPacketSize = LE16(buf + 4);
  ep_descriptor->bInterval = *(uint8_t *) (buf + 6);
}


/**
  * @brief  USBH_ParseStringDesc
  *         This function Parses the string descriptor
  * @param  psrc: Source pointer containing the descriptor data
  * @param  pdest: Destination address pointer
  * @param  length: Length of the descriptor
  * @retval None
  */
static void USBH_ParseStringDesc(uint8_t *psrc, uint8_t *pdest, uint16_t length) {
  uint16_t strlength;
  uint16_t idx;
  /* The UNICODE string descriptor is not NULL-terminated. The string length is
  computed by substracting two from the value of the first byte of the descriptor.
  */
  /* Check which is lower size, the Size of string or the length of bytes read
  from the device */
  if (psrc[1] == USB_DESC_TYPE_STRING) {
    /* Make sure the Descriptor is String Type */
    /* psrc[0] contains Size of Descriptor, subtract 2 to get the length of string */
    strlength = ((((uint16_t) psrc[0] - 2U) <= length) ? ((uint16_t) psrc[0] - 2U) : length);
    /* Adjust the offset ignoring the String Len and Descriptor type */
    psrc += 2U;
    for (idx = 0U; idx < strlength; idx += 2U) {
      /* Copy Only the string and ignore the UNICODE ID, hence add the src */
      *pdest = psrc[idx];
      pdest++;
    }
    *pdest = 0U; /* mark end of string */
  }
}


/**
  * @brief  USBH_GetNextDesc
  *         This function return the next descriptor header
  * @param  buf: Buffer where the cfg descriptor is available
  * @param  ptr: data pointer inside the cfg descriptor
  * @retval next header
  */
USBH_DescHeader_t *USBH_GetNextDesc(uint8_t *pbuf, uint16_t *ptr) {
  USBH_DescHeader_t *pnext;

  *ptr += ((USBH_DescHeader_t *) (void *) pbuf)->bLength;
  pnext = (USBH_DescHeader_t *) (void *) ((uint8_t *) (void *) pbuf + ((USBH_DescHeader_t *) (void *) pbuf)->bLength);

  return (pnext);
}


/**
  * @brief  USBH_CtlReq
  *         USBH_CtlReq sends a control request and provide the status after
  *            completion of the request
  * @param  phost: Host Handle
  * @param  req: Setup Request Structure
  * @param  buff: data buffer address to store the response
  * @param  length: length of the response
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_CtlReq(USBH_HandleTypeDef *phost, uint8_t *buff,
                               uint16_t length) {
  USBH_StatusTypeDef status;
  status = USBH_BUSY;

  switch (phost->RequestState) {
    case CMD_SEND:
      /* Start a SETUP transfer */
      phost->Control.buff = buff;
      phost->Control.length = length;
      phost->RequestState = CMD_WAIT;
      status = USBH_BUSY;
      changeState(phost, CTRL_SETUP, 1);
      break;

    case CMD_WAIT:
      status = USBH_HandleControl(phost);
      if ((status == USBH_OK) || (status == USBH_NOT_SUPPORTED)) {
        /* Transaction completed, move control state to idle */
        phost->RequestState = CMD_SEND;
        changeState(phost, CTRL_IDLE, 1);
      } else if (status == USBH_FAIL) {
        /* Failure Mode */
        phost->RequestState = CMD_SEND;
        postEvent(phost);
      } else {
        /* .. */
      }
//was spewy:      postEvent(phost);
      break;

    default:
      break;
  }
  return status;
}


static void markError(USBH_HandleTypeDef *phost) {
  changeState(phost, CTRL_ERROR, 1);
}

/**
  * @brief  USBH_HandleControl
  *         Handles the USB control transfer state machine
  * @param  phost: Host Handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HandleControl(USBH_HandleTypeDef *phost) {
  uint8_t direction;
  USBH_StatusTypeDef status = USBH_BUSY;
  USBH_URBStateTypeDef URB_Status;//= USBH_URB_IDLE;

  switch (phost->Control.state) {
    case CTRL_SETUP:
      /* send a SETUP packet */
      USBH_CtlSendSetup(phost, (uint8_t *) (void *) phost->Control.setup.d8, phost->Control.pipe_out);
      changeState(phost, CTRL_SETUP_WAIT, 0);
      break;

    case CTRL_SETUP_WAIT:

      URB_Status = USBH_LL_GetURBState(phost, phost->Control.pipe_out);
      /* case SETUP packet sent successfully */
      if (URB_Status == USBH_URB_DONE) {
        direction = (phost->Control.setup.b.bmRequestType & USB_REQ_DIR_MASK);

        /* check if there is a data stage */
        changeState(phost, phost->Control.setup.b.wLength.w != 0U ?
                           direction == USB_D2H ? CTRL_DATA_IN : CTRL_DATA_OUT :
                           direction == USB_D2H ? CTRL_STATUS_OUT : CTRL_STATUS_IN, 1);

      } else if ((URB_Status == USBH_URB_ERROR) || (URB_Status == USBH_URB_NOTREADY)) {
        markError(phost);
      }

      break;

    case CTRL_DATA_IN:
      /* Issue an IN token */
      phost->Control.timer = (uint16_t) phost->Timer;
      USBH_CtlReceiveData(phost, phost->Control.buff, phost->Control.length, phost->Control.pipe_in);

      changeState(phost, CTRL_DATA_IN_WAIT, 0);
      break;

    case CTRL_DATA_IN_WAIT:

      URB_Status = USBH_LL_GetURBState(phost, phost->Control.pipe_in);

      /* check is DATA packet transferred successfully */
      if (URB_Status == USBH_URB_DONE) {
        changeState(phost, CTRL_STATUS_OUT, 1);
      } else if (URB_Status == USBH_URB_STALL) {
        /* In stall case, return to previous machine state*/
        status = USBH_NOT_SUPPORTED;
        //?? no state change?
        postEvent(phost);
      } else if (URB_Status == USBH_URB_ERROR) {
        /* Device error */
        markError(phost);
      }

      break;

    case CTRL_DATA_OUT:
      USBH_CtlSendData(phost, phost->Control.buff, phost->Control.length, phost->Control.pipe_out, 1U);
      phost->Control.timer = (uint16_t) phost->Timer;
      changeState(phost, CTRL_DATA_OUT_WAIT, 1);
      break;

    case CTRL_DATA_OUT_WAIT:

      URB_Status = USBH_LL_GetURBState(phost, phost->Control.pipe_out);

      if (URB_Status == USBH_URB_DONE) {
        /* If the Setup Pkt is sent successful, then change the state */
        changeState(phost, CTRL_STATUS_IN, 0);
      } else if (URB_Status == USBH_URB_STALL) {
        /* In stall case, return to previous machine state*/
        changeState(phost, CTRL_STALLED, 0);
        status = USBH_NOT_SUPPORTED;
      } else if (URB_Status == USBH_URB_NOTREADY) {
        /* Nack received from device */
        changeState(phost, CTRL_DATA_OUT, 0);
      } else if (URB_Status == USBH_URB_ERROR) {
        /* device error */
        markError(phost);
        status = USBH_FAIL;
      }
      break;

    case CTRL_STATUS_IN:
      /* Send 0 bytes out packet */
      USBH_CtlReceiveData(phost, 0U, 0U, phost->Control.pipe_in);

      phost->Control.timer = (uint16_t) phost->Timer;
      changeState(phost, CTRL_STATUS_IN_WAIT, 1);
      break;

    case CTRL_STATUS_IN_WAIT:
      URB_Status = USBH_LL_GetURBState(phost, phost->Control.pipe_in);

      if (URB_Status == USBH_URB_DONE) {
        /* Control transfers completed, Exit the State Machine */
        changeState(phost, CTRL_COMPLETE, 1);
        status = USBH_OK;
      } else if (URB_Status == USBH_URB_ERROR) {
        markError(phost);
      } else if (URB_Status == USBH_URB_STALL) {
        /* Control transfers completed, Exit the State Machine */
        status = USBH_NOT_SUPPORTED;
        postEvent(phost);
      }
      break;

    case CTRL_STATUS_OUT:
      USBH_CtlSendData(phost, 0U, 0U, phost->Control.pipe_out, 1U);

      phost->Control.timer = (uint16_t) phost->Timer;
      changeState(phost, CTRL_STATUS_OUT_WAIT, 0);
      break;

    case CTRL_STATUS_OUT_WAIT:
      URB_Status = USBH_LL_GetURBState(phost, phost->Control.pipe_out);
      if (URB_Status == USBH_URB_DONE) {
        status = USBH_OK;
        changeState(phost, CTRL_COMPLETE, 1);
      } else if (URB_Status == USBH_URB_NOTREADY) {
        changeState(phost, CTRL_STATUS_OUT, 1);
      } else if (URB_Status == USBH_URB_ERROR) {
        markError(phost);
      }
      break;

    case CTRL_ERROR:
      /*
      After a halt condition is encountered or an error is detected by the
      host, a control endpoint is allowed to recover by accepting the next Setup
      PID; i.e., recovery actions via some other pipe are not required for control
      endpoints. For the Default Control Pipe, a device reset will ultimately be
      required to clear the halt or error condition if the next Setup PID is not
      accepted.
      */
      if (++phost->Control.errorcount <= USBH_MAX_ERROR_COUNT) {
        /* Do the transmission again, starting from SETUP Packet */
        changeState(phost, CTRL_SETUP, 0);
        phost->RequestState = CMD_SEND;
      } else {
        phost->pUser(phost, HOST_USER_UNRECOVERED_ERROR);
        phost->Control.errorcount = 0U;
        USBH_ErrLog("Control error: Device not responding");

        /* Free control pipes */
        USBH_FreePipe(phost, phost->Control.pipe_out);
        USBH_FreePipe(phost, phost->Control.pipe_in);

        phost->gState = HOST_IDLE;
        status = USBH_FAIL;
      }
      break;

    default:
      break;
  }

  return status;
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