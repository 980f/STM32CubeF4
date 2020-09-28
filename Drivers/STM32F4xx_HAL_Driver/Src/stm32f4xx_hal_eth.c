/**
  ******************************************************************************
  * @file    stm32f4xx_hal_eth.c
  * @author  MCD Application Team
  * @brief   ETH HAL module driver.
  *          This file provides firmware functions to manage the following 
  *          functionalities of the Ethernet (ETH) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions 
  *           + Peripheral State and Errors functions
  *
  @verbatim
  ==============================================================================
                    ##### How to use this driver #####
  ==============================================================================
    [..]
      (#)Declare a ETH_HandleTypeDef handle structure, for example:
         ETH_HandleTypeDef  heth;
        
      (#)Fill parameters of Init structure in heth handle
  
      (#)Call HAL_ETH_Init() API to initialize the Ethernet peripheral (MAC, DMA, ...) 

      (#)Initialize the ETH low level resources through the HAL_ETH_MspInit() API:
          (##) Enable the Ethernet interface clock using 
               (+++) __HAL_RCC_ETHMAC_CLK_ENABLE();
               (+++) __HAL_RCC_ETHMACTX_CLK_ENABLE();
               (+++) __HAL_RCC_ETHMACRX_CLK_ENABLE();
           
          (##) Initialize the related GPIO clocks
          (##) Configure Ethernet pin-out
          (##) Configure Ethernet NVIC interrupt (IT mode)   
    
      (#)Initialize Ethernet DMA Descriptors in chain mode and point to allocated buffers:
          (##) HAL_ETH_DMATxDescListInit(); for Transmission process
          (##) HAL_ETH_DMARxDescListInit(); for Reception process

      (#)Enable MAC and DMA transmission and reception:
          (##) HAL_ETH_Start();

      (#)Prepare ETH DMA TX Descriptors and give the hand to ETH DMA to transfer 
         the frame to MAC TX FIFO:
         (##) HAL_ETH_TransmitFrame();

      (#)Poll for a received frame in ETH RX DMA Descriptors and get received 
         frame parameters
         (##) HAL_ETH_GetReceivedFrame(); (should be called into an infinite loop)

      (#) Get a received frame when an ETH RX interrupt occurs:
         (##) HAL_ETH_GetReceivedFrame_IT(); (called in IT mode only)

      (#) Communicate with external PHY device:
         (##) Read a specific register from the PHY  
              HAL_ETH_ReadPHYRegister();
         (##) Write data to a specific RHY register:
              HAL_ETH_WritePHYRegister();

      (#) Configure the Ethernet MAC after ETH peripheral initialization
          HAL_ETH_ConfigMAC(); all MAC parameters should be filled.
      
      (#) Configure the Ethernet DMA after ETH peripheral initialization
          HAL_ETH_ConfigDMA(); all DMA parameters should be filled.
      
      -@- The PTP protocol and the DMA descriptors ring mode are not supported 
          in this driver
*** Callback registration ***
  =============================================

  The compilation define  USE_HAL_ETH_REGISTER_CALLBACKS when set to 1
  allows the user to configure dynamically the driver callbacks.
  Use Function @ref HAL_ETH_RegisterCallback() to register an interrupt callback.

  Function @ref HAL_ETH_RegisterCallback() allows to register following callbacks:
    (+) TxCpltCallback   : Tx Complete Callback.
    (+) RxCpltCallback   : Rx Complete Callback.
    (+) DMAErrorCallback : DMA Error Callback.
    (+) MspInitCallback  : MspInit Callback.
    (+) MspDeInitCallback: MspDeInit Callback.

  This function takes as parameters the HAL peripheral handle, the Callback ID
  and a pointer to the user callback function.

  Use function @ref HAL_ETH_UnRegisterCallback() to reset a callback to the default
  weak function.
  @ref HAL_ETH_UnRegisterCallback takes as parameters the HAL peripheral handle,
  and the Callback ID.
  This function allows to reset following callbacks:
    (+) TxCpltCallback   : Tx Complete Callback.
    (+) RxCpltCallback   : Rx Complete Callback.
    (+) DMAErrorCallback : DMA Error Callback.
    (+) MspInitCallback  : MspInit Callback.
    (+) MspDeInitCallback: MspDeInit Callback.

  By default, after the HAL_ETH_Init and when the state is HAL_ETH_STATE_RESET
  all callbacks are set to the corresponding weak functions:
  examples @ref HAL_ETH_TxCpltCallback(), @ref HAL_ETH_RxCpltCallback().
  Exception done for MspInit and MspDeInit functions that are
  reset to the legacy weak function in the HAL_ETH_Init/ @ref HAL_ETH_DeInit only when
  these callbacks are null (not registered beforehand).
  if not, MspInit or MspDeInit are not null, the HAL_ETH_Init/ @ref HAL_ETH_DeInit
  keep and use the user MspInit/MspDeInit callbacks (registered beforehand)

  Callbacks can be registered/unregistered in HAL_ETH_STATE_READY state only.
  Exception done MspInit/MspDeInit that can be registered/unregistered
  in HAL_ETH_STATE_READY or HAL_ETH_STATE_RESET state,
  thus registered (user) MspInit/DeInit callbacks can be used during the Init/DeInit.
  In that case first register the MspInit/MspDeInit user callbacks
  using @ref HAL_ETH_RegisterCallback() before calling @ref HAL_ETH_DeInit
  or HAL_ETH_Init function.

  When The compilation define USE_HAL_ETH_REGISTER_CALLBACKS is set to 0 or
  not defined, the callback registration feature is not available and all callbacks
  are set to the corresponding weak functions.

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup ETH ETH 
  * @brief ETH HAL module driver
  * @{
  */

#ifdef HAL_ETH_MODULE_ENABLED

#if defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F427xx) || defined(STM32F437xx) ||\
    defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup ETH_Private_Constants ETH Private Constants
  * @{
  */
#define ETH_TIMEOUT_SWRESET               500U  
#define ETH_TIMEOUT_LINKED_STATE          5000U
#define ETH_TIMEOUT_AUTONEGO_COMPLETED    5000U

/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup ETH_Private_Functions ETH Private Functions
  * @{
  */
static void ETH_MACDMAConfig(ETH_HandleTypeDef *heth, uint32_t err);
static void ETH_MACAddressConfig(ETH_HandleTypeDef *heth, uint32_t MacAddr, const uint8_t *Addr);
static void ETH_MACReceptionEnable(ETH_HandleTypeDef *heth);
static void ETH_MACReceptionDisable(ETH_HandleTypeDef *heth);
static void ETH_MACTransmissionEnable(ETH_HandleTypeDef *heth);
static void ETH_MACTransmissionDisable(ETH_HandleTypeDef *heth);
static void ETH_DMATransmissionEnable(ETH_HandleTypeDef *heth);
static void ETH_DMATransmissionDisable(ETH_HandleTypeDef *heth);
static void ETH_DMAReceptionEnable(ETH_HandleTypeDef *heth);
static void ETH_DMAReceptionDisable(ETH_HandleTypeDef *heth);
static void ETH_FlushTransmitFIFO(ETH_HandleTypeDef *heth);

/** override the following function to be called (repeatedly!) while this ETH module is waiting for a MAC value or a Phy value to change. */
__weak void ETH_whileWaiting(){}
//the following delay routine was not implemented in a reasonable way to intercept, and was also not used for documented needs.
//static void ETH_Delay(uint32_t mdelay);
/* waiter for spinning on phy reg bits, so that other things can be polled while the ETH is waiting. */
static HAL_StatusTypeDef ETH_waitOnPhy(uint16_t phyreg, uint16_t ones, uint16_t zeroes, unsigned mdelay, ETH_HandleTypeDef *heth);
/* waiter for spinning on memory mapped register bits, so that other things can be polled while the ETH is waiting. */
static HAL_StatusTypeDef ETH_waitOnMac(volatile uint32_t *macregister, uint32_t ones, uint32_t zeroes, unsigned mdelay, ETH_HandleTypeDef *heth);

#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
static void ETH_InitCallbacksToDefault(ETH_HandleTypeDef *heth);
#endif /* USE_HAL_ETH_REGISTER_CALLBACKS */

/**
  * @}
  */
/* Private functions ---------------------------------------------------------*/

/** @defgroup ETH_Exported_Functions ETH Exported Functions
  * @{
  */

/** @defgroup ETH_Exported_Functions_Group1 Initialization and de-initialization functions 
  *  @brief   Initialization and Configuration functions 
  *
  @verbatim    
  ===============================================================================
            ##### Initialization and de-initialization functions #####
  ===============================================================================
  [..]  This section provides functions allowing to:
      (+) Initialize and configure the Ethernet peripheral
      (+) De-initialize the Ethernet peripheral

  @endverbatim
  * @{
  */


/**
  * @brief  frequently invoked reaction to failure
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
static HAL_StatusTypeDef bailOnTimeout(ETH_HandleTypeDef *heth){
  /* In case of write timeout Config MAC and DMA */
  ETH_MACDMAConfig(heth, ETH_ERROR);

  heth->State= HAL_ETH_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(heth);

  return HAL_TIMEOUT;
}

/**
  * @brief  frequently invoked reaction to failure
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
static HAL_StatusTypeDef bailOnPhyfail(ETH_HandleTypeDef *heth){
  /* In case of write timeout  Config MAC and DMA */
  ETH_MACDMAConfig(heth, ETH_ERROR);

  /* Set the ETH peripheral state to READY */
  heth->State = HAL_ETH_STATE_READY;

  /* Return HAL_ERROR */
  return HAL_ERROR;
}


/**
  * @brief  when some registers are written a little dance is performed, not documented in the ST manual for the device.
  * @param target pointer to MAC register.
  * @param heth hardware access struct, in case we end up needing it someday
  * @param ones these bits are set
  * @param zeroes these bits are cleared.  (zeroes are put where this value has ones)
*/
static void ETH_MacFiddle(const ETH_HandleTypeDef *heth, __IO uint32_t *fiddlee,unsigned ones, unsigned zeroes){
  uint32_t mixer = *fiddlee;
  mixer&=~zeroes;
  mixer|=ones;
  *fiddlee =mixer;
  UNUSED(heth);
//The following code, which is now no longer duplicated a bunch of times, is a weird way to kill time.
// reading the register takes some time, the ETH_Delay took way too much time, and the write at the end is no-op as well.
//  /* Wait until the write operation will be taken into account:
//     at least four TX_CLK/RX_CLK clock cycles */
//  tmpreg1 = (heth->Instance)->MACCR;
//  ETH_Delay(ETH_REG_WRITE_DELAY);  !! some instances use HAL_Delay, randomly selected from the duplications of this chunk.
//  (heth->Instance)->MACCR = tmpreg1;
}


/**
  * @brief  Initializes the Ethernet MAC and DMA according to default
  *         parameters.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_Init(ETH_HandleTypeDef *heth)
{
  uint32_t tmpreg1 = 0U;
  uint32_t phy_sr = 0U;
  uint32_t hclk = 60000000U;
  
  /* Check the ETH peripheral state */
  if(heth == NULL)
  {
    return HAL_ERROR;
  }
  
  /* Check parameters */
  assert_param(IS_ETH_AUTONEGOTIATION(heth->Init.AutoNegotiation));
  assert_param(IS_ETH_RX_MODE(heth->Init.RxMode));
  assert_param(IS_ETH_CHECKSUM_MODE(heth->Init.ChecksumMode));
  assert_param(IS_ETH_MEDIA_INTERFACE(heth->Init.MediaInterface));  
  
  if(heth->State == HAL_ETH_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    heth->Lock = HAL_UNLOCKED;
#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
    ETH_InitCallbacksToDefault(heth);

    if(heth->MspInitCallback == NULL)
    {
      /* Init the low level hardware : GPIO, CLOCK, NVIC. */
      heth->MspInitCallback = HAL_ETH_MspInit;
    }
    heth->MspInitCallback(heth);

#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC. */
    HAL_ETH_MspInit(heth);
#endif /* USE_HAL_ETH_REGISTER_CALLBACKS */
  }
  
  /* Enable SYSCFG Clock */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  
  /* Select MII or RMII Mode*/
  SYSCFG->PMC &= ~(SYSCFG_PMC_MII_RMII_SEL);
  SYSCFG->PMC |= (uint32_t)heth->Init.MediaInterface;
  
  /* Ethernet Software reset */
  /* Set the SWR bit: resets all MAC subsystem internal registers and logic */
  /* After reset all the registers holds their respective reset values */
  (heth->Instance)->DMABMR |= ETH_DMABMR_SR;

	/** wait for SR bit to become zero*/
  if(HAL_TIMEOUT==ETH_waitOnMac(&(heth->Instance->DMABMR),0,ETH_DMABMR_SR,ETH_TIMEOUT_SWRESET,heth)){
    return HAL_TIMEOUT;
  }

  /*-------------------------------- MAC Initialization ----------------------*/
  /* Get the ETHERNET MACMIIAR value */
  tmpreg1 = (heth->Instance)->MACMIIAR;
  /* Clear CSR Clock Range CR[2:0] bits */
  tmpreg1 &= ETH_MACMIIAR_CR_MASK;

  /* Set CR bits depending on hclk value */

  /* Get hclk frequency value */
  hclk = HAL_RCC_GetHCLKFreq();
  /* 980f: the test for >= hclk >= 20000000U and the gratuitous checks in each elseif clause left the divisor at div102 for very slow hclk.
   * a test for <20Mhz if true should report an error, not let it run with the SNI access amazingly slow */
//  if(hclk<20000000U){
//    /* mimic old code with drastic choice of divisor */
//    tmpreg1 |= (uint32_t)ETH_MACMIIAR_CR_Div102;
//  } else
  if(hclk < 35000000U)
  {
    /* CSR Clock Range between 20-35 MHz */
    tmpreg1 |= (uint32_t)ETH_MACMIIAR_CR_Div16;
  }
  else if(hclk < 60000000U)
  {
    /* CSR Clock Range between 35-60 MHz */ 
    tmpreg1 |= (uint32_t)ETH_MACMIIAR_CR_Div26;
  }  
  else if(hclk < 100000000U)
  {
    /* CSR Clock Range between 60-100 MHz */ 
    tmpreg1 |= (uint32_t)ETH_MACMIIAR_CR_Div42;
  }  
  else if(hclk < 150000000U)
  {
    /* CSR Clock Range between 100-150 MHz */ 
    tmpreg1 |= (uint32_t)ETH_MACMIIAR_CR_Div62;
  }
  else /* ((hclk >= 150000000)&&(hclk <= 183000000)) */
  {
    /* CSR Clock Range between 150-183 MHz */ 
    tmpreg1 |= (uint32_t)ETH_MACMIIAR_CR_Div102;    
  }
  
  /* Write to ETHERNET MAC MIIAR: Configure the ETHERNET CSR Clock Range */
  (heth->Instance)->MACMIIAR = (uint32_t)tmpreg1;
  
  /*-------------------- PHY initialization and configuration ----------------*/
  /* Put the PHY in reset mode */
  if((HAL_ETH_WritePHYRegister(heth, PHY_BCR, PHY_RESET)) != HAL_OK)
  {
    return bailOnPhyfail(heth);
  }
  
  /* Delay to assure PHY reset */
  HAL_Delay(PHY_RESET_DELAY);
  
  if((heth->Init).AutoNegotiation != ETH_AUTONEGOTIATION_DISABLE)
  {
    if(ETH_waitOnPhy(PHY_BSR,PHY_LINKED_STATUS,0,ETH_TIMEOUT_LINKED_STATE,heth)!=HAL_OK){
      return bailOnTimeout(heth);
    }
    
    /* Enable Auto-Negotiation */
    if((HAL_ETH_WritePHYRegister(heth, PHY_BCR, PHY_AUTONEGOTIATION)) != HAL_OK)
    {
      return bailOnPhyfail(heth);
    }

    if(ETH_waitOnPhy(PHY_BSR,PHY_AUTONEGO_COMPLETE,0,ETH_TIMEOUT_AUTONEGO_COMPLETED,heth)!=HAL_OK)
    {
      return bailOnTimeout(heth);
    }

    /* Read the result of the auto-negotiation */
    if((HAL_ETH_ReadPHYRegister(heth, PHY_SR, &phy_sr)) != HAL_OK)
    {
      return bailOnPhyfail(heth);
    }
    
    /* Configure the MAC with the Duplex Mode fixed by the auto-negotiation process */
    if((phy_sr & PHY_DUPLEX_STATUS) != 0)
    {
      /* Set Ethernet duplex mode to Full-duplex following the auto-negotiation */
      (heth->Init).DuplexMode = ETH_MODE_FULLDUPLEX;  
    }
    else
    {
      /* Set Ethernet duplex mode to Half-duplex following the auto-negotiation */
      (heth->Init).DuplexMode = ETH_MODE_HALFDUPLEX;           
    }
    /* Configure the MAC with the speed fixed by the auto-negotiation process */
    if((phy_sr & PHY_SPEED_STATUS) == PHY_SPEED_STATUS)
    {  
      /* Set Ethernet speed to 10M following the auto-negotiation */
      (heth->Init).Speed = ETH_SPEED_10M; 
    }
    else
    {   
      /* Set Ethernet speed to 100M following the auto-negotiation */ 
      (heth->Init).Speed = ETH_SPEED_100M;
    }
  }
  else /* AutoNegotiation Disable */
  {
    /* Check parameters */
    assert_param(IS_ETH_SPEED(heth->Init.Speed));
    assert_param(IS_ETH_DUPLEX_MODE(heth->Init.DuplexMode));
    
    /* Set MAC Speed and Duplex Mode */
    if(HAL_ETH_WritePHYRegister(heth, PHY_BCR, ((uint16_t)((heth->Init).DuplexMode >> 3U) |
                                                (uint16_t)((heth->Init).Speed >> 1U))) != HAL_OK)
    {
      return bailOnPhyfail(heth);
    }  
    
    /* Delay to assure PHY configuration */
    HAL_Delay(PHY_CONFIG_DELAY);
  }
  
  /* Config MAC and DMA */
  ETH_MACDMAConfig(heth, ETH_SUCCESS);
  
  /* Set ETH HAL State to Ready */
  heth->State= HAL_ETH_STATE_READY;
  
  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  De-Initializes the ETH peripheral. 
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_DeInit(ETH_HandleTypeDef *heth)
{
  /* Set the ETH peripheral state to BUSY */
  heth->State = HAL_ETH_STATE_BUSY;
  
#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
  if(heth->MspDeInitCallback == NULL)
  {
    heth->MspDeInitCallback = HAL_ETH_MspDeInit;
  }
  /* De-Init the low level hardware : GPIO, CLOCK, NVIC. */
  heth->MspDeInitCallback(heth);
#else
  /* De-Init the low level hardware : GPIO, CLOCK, NVIC. */
  HAL_ETH_MspDeInit(heth);
#endif
  
  /* Set ETH HAL state to Disabled */
  heth->State= HAL_ETH_STATE_RESET;

  /* Release Lock */
  __HAL_UNLOCK(heth);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the DMA Tx descriptors in chain mode.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module  
  * @param  DMATxDescTab Pointer to the first Tx desc list 
  * @param  TxBuff Pointer to the first TxBuffer list
  * @param  TxBuffCount Number of the used Tx desc in the list
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_DMATxDescListInit(ETH_HandleTypeDef *heth, ETH_DMADescTypeDef *DMATxDescTab, uint8_t *TxBuff, uint32_t TxBuffCount)
{
  uint32_t i = 0U;
  ETH_DMADescTypeDef *dmatxdesc;
  
  /* Process Locked */
  __HAL_LOCK(heth);
  
  /* Set the ETH peripheral state to BUSY */
  heth->State = HAL_ETH_STATE_BUSY;
  
  /* Set the DMATxDescToSet pointer with the first one of the DMATxDescTab list */
  heth->TxDesc = DMATxDescTab;
  
  /* Fill each DMATxDesc descriptor with the right values */   
  for(i=0U; i < TxBuffCount; i++)
  {
    /* Get the pointer on the ith member of the Tx Desc list */
    dmatxdesc = DMATxDescTab + i;
    
    /* Set Second Address Chained bit */
    dmatxdesc->Status = ETH_DMATXDESC_TCH;  
    
    /* Set Buffer1 address pointer */
    dmatxdesc->Buffer1Addr = (uint32_t)(&TxBuff[i*ETH_TX_BUF_SIZE]);
    
    if ((heth->Init).ChecksumMode == ETH_CHECKSUM_BY_HARDWARE)
    {
      /* Set the DMA Tx descriptors checksum insertion */
      dmatxdesc->Status |= ETH_DMATXDESC_CHECKSUMTCPUDPICMPFULL;
    }
    
    /* Initialize the next descriptor with the Next Descriptor Polling Enable */
    if(i < (TxBuffCount-1U))
    {
      /* Set next descriptor address register with next descriptor base address */
      dmatxdesc->Buffer2NextDescAddr = (uint32_t)(DMATxDescTab+i+1U);
    }
    else
    {
      /* For last descriptor, set next descriptor address register equal to the first descriptor base address */ 
      dmatxdesc->Buffer2NextDescAddr = (uint32_t) DMATxDescTab;  
    }
  }
  
  /* Set Transmit Descriptor List Address Register */
  (heth->Instance)->DMATDLAR = (uint32_t) DMATxDescTab;
  
  /* Set ETH HAL State to Ready */
  heth->State= HAL_ETH_STATE_READY;
  
  /* Process Unlocked */
  __HAL_UNLOCK(heth);
  
  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the DMA Rx descriptors in chain mode.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module  
  * @param  DMARxDescTab Pointer to the first Rx desc list 
  * @param  RxBuff Pointer to the first RxBuffer list
  * @param  RxBuffCount Number of the used Rx desc in the list
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_DMARxDescListInit(ETH_HandleTypeDef *heth, ETH_DMADescTypeDef *DMARxDescTab, uint8_t *RxBuff, uint32_t RxBuffCount)
{
  uint32_t i = 0U;
  ETH_DMADescTypeDef *DMARxDesc;
  
  /* Process Locked */
  __HAL_LOCK(heth);
  
  /* Set the ETH peripheral state to BUSY */
  heth->State = HAL_ETH_STATE_BUSY;
  
  /* Set the Ethernet RxDesc pointer with the first one of the DMARxDescTab list */
  heth->RxDesc = DMARxDescTab; 
  
  /* Fill each DMARxDesc descriptor with the right values */
  for(i=0U; i < RxBuffCount; i++)
  {
    /* Get the pointer on the ith member of the Rx Desc list */
    DMARxDesc = DMARxDescTab+i;
    
    /* Set Own bit of the Rx descriptor Status */
    DMARxDesc->Status = ETH_DMARXDESC_OWN;
    
    /* Set Buffer1 size and Second Address Chained bit */
    DMARxDesc->ControlBufferSize = ETH_DMARXDESC_RCH | ETH_RX_BUF_SIZE;  
    
    /* Set Buffer1 address pointer */
    DMARxDesc->Buffer1Addr = (uint32_t)(&RxBuff[i*ETH_RX_BUF_SIZE]);
    
    if((heth->Init).RxMode == ETH_RXINTERRUPT_MODE)
    {
      /* Enable Ethernet DMA Rx Descriptor interrupt */
      DMARxDesc->ControlBufferSize &= ~ETH_DMARXDESC_DIC;
    }
    
    /* Initialize the next descriptor with the Next Descriptor Polling Enable */
    if(i < (RxBuffCount-1U))
    {
      /* Set next descriptor address register with next descriptor base address */
      DMARxDesc->Buffer2NextDescAddr = (uint32_t)(DMARxDescTab+i+1U); 
    }
    else
    {
      /* For last descriptor, set next descriptor address register equal to the first descriptor base address */ 
      DMARxDesc->Buffer2NextDescAddr = (uint32_t)(DMARxDescTab); 
    }
  }
  
  /* Set Receive Descriptor List Address Register */
  (heth->Instance)->DMARDLAR = (uint32_t) DMARxDescTab;
  
  /* Set ETH HAL State to Ready */
  heth->State= HAL_ETH_STATE_READY;
  
  /* Process Unlocked */
  __HAL_UNLOCK(heth);
  
  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the ETH MSP.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
__weak void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(heth);
  /* NOTE : This function Should not be modified, when the callback is needed,
  the HAL_ETH_MspInit could be implemented in the user file
  */
}

/**
  * @brief  DeInitializes ETH MSP.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
__weak void HAL_ETH_MspDeInit(ETH_HandleTypeDef *heth)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(heth);
  /* NOTE : This function Should not be modified, when the callback is needed,
  the HAL_ETH_MspDeInit could be implemented in the user file
  */
}

#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User ETH Callback
  *         To be used instead of the weak predefined callback
  * @param heth eth handle
  * @param CallbackID ID of the callback to be registered
  *        This parameter can be one of the following values:
  *          @arg @ref HAL_ETH_TX_COMPLETE_CB_ID Tx Complete Callback ID
  *          @arg @ref HAL_ETH_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *          @arg @ref HAL_ETH_DMA_ERROR_CB_ID   DMA Error Callback ID
  *          @arg @ref HAL_ETH_MSPINIT_CB_ID     MspInit callback ID
  *          @arg @ref HAL_ETH_MSPDEINIT_CB_ID   MspDeInit callback ID
  * @param pCallback pointer to the Callback function
  * @retval status
  */
HAL_StatusTypeDef HAL_ETH_RegisterCallback(ETH_HandleTypeDef *heth, HAL_ETH_CallbackIDTypeDef CallbackID, pETH_CallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if(pCallback == NULL)
  {
    return HAL_ERROR;
  }
  /* Process locked */
  __HAL_LOCK(heth);

  if(heth->State == HAL_ETH_STATE_READY)
  {
    switch (CallbackID)
    {
    case HAL_ETH_TX_COMPLETE_CB_ID :
      heth->TxCpltCallback = pCallback;
      break;

    case HAL_ETH_RX_COMPLETE_CB_ID :
      heth->RxCpltCallback = pCallback;
      break;

    case HAL_ETH_DMA_ERROR_CB_ID :
      heth->DMAErrorCallback = pCallback;
      break;

    case HAL_ETH_MSPINIT_CB_ID :
      heth->MspInitCallback = pCallback;
      break;

    case HAL_ETH_MSPDEINIT_CB_ID :
      heth->MspDeInitCallback = pCallback;
      break;

    default :
      /* Return error status */
      status =  HAL_ERROR;
      break;
    }
  }
  else if(heth->State == HAL_ETH_STATE_RESET)
  {
    switch (CallbackID)
    {
    case HAL_ETH_MSPINIT_CB_ID :
      heth->MspInitCallback = pCallback;
      break;

    case HAL_ETH_MSPDEINIT_CB_ID :
      heth->MspDeInitCallback = pCallback;
      break;

    default :
      /* Return error status */
      status =  HAL_ERROR;
      break;
    }
  }
  else
  {
    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(heth);

  return status;
}

/**
  * @brief  Unregister an ETH Callback
  *         ETH callabck is redirected to the weak predefined callback
  * @param heth eth handle
  * @param CallbackID ID of the callback to be unregistered
  *        This parameter can be one of the following values:
  *          @arg @ref HAL_ETH_TX_COMPLETE_CB_ID Tx Complete Callback ID
  *          @arg @ref HAL_ETH_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *          @arg @ref HAL_ETH_DMA_ERROR_CB_ID      DMA Error Callback ID
  *          @arg @ref HAL_ETH_MSPINIT_CB_ID     MspInit callback ID
  *          @arg @ref HAL_ETH_MSPDEINIT_CB_ID   MspDeInit callback ID
  * @retval status
  */
HAL_StatusTypeDef HAL_ETH_UnRegisterCallback(ETH_HandleTypeDef *heth, HAL_ETH_CallbackIDTypeDef CallbackID)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(heth);

  if(heth->State == HAL_ETH_STATE_READY)
  {
    switch (CallbackID)
    {
    case HAL_ETH_TX_COMPLETE_CB_ID :
      heth->TxCpltCallback = HAL_ETH_TxCpltCallback;
      break;

    case HAL_ETH_RX_COMPLETE_CB_ID :
      heth->RxCpltCallback = HAL_ETH_RxCpltCallback;
      break;

    case HAL_ETH_DMA_ERROR_CB_ID :
      heth->DMAErrorCallback = HAL_ETH_ErrorCallback;
      break;

    case HAL_ETH_MSPINIT_CB_ID :
      heth->MspInitCallback = HAL_ETH_MspInit;
      break;

    case HAL_ETH_MSPDEINIT_CB_ID :
      heth->MspDeInitCallback = HAL_ETH_MspDeInit;
      break;

    default :
      /* Return error status */
      status =  HAL_ERROR;
      break;
    }
  }
  else if(heth->State == HAL_ETH_STATE_RESET)
  {
    switch (CallbackID)
    {
    case HAL_ETH_MSPINIT_CB_ID :
      heth->MspInitCallback = HAL_ETH_MspInit;
      break;

    case HAL_ETH_MSPDEINIT_CB_ID :
      heth->MspDeInitCallback = HAL_ETH_MspDeInit;
      break;

    default :
      /* Return error status */
      status =  HAL_ERROR;
      break;
    }
  }
  else
  {
    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(heth);

  return status;
}
#endif /* USE_HAL_ETH_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group2 IO operation functions 
  *  @brief   Data transfers functions 
  *
  @verbatim   
  ==============================================================================
                          ##### IO operation functions #####
  ==============================================================================  
  [..]  This section provides functions allowing to:
        (+) Transmit a frame
            HAL_ETH_TransmitFrame();
        (+) Receive a frame
            HAL_ETH_GetReceivedFrame();
            HAL_ETH_GetReceivedFrame_IT();
        (+) Read from an External PHY register
            HAL_ETH_ReadPHYRegister();
        (+) Write to an External PHY register
            HAL_ETH_WritePHYRegister();

  @endverbatim
  
  * @{
  */

/**
  * @brief  Sends an Ethernet frame. 
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  FrameLength Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_TransmitFrame(ETH_HandleTypeDef *heth, uint32_t FrameLength)
{
  uint32_t bufcount = 0U, size = 0U, i = 0U;
  
  /* Process Locked */
  __HAL_LOCK(heth);
  
  /* Set the ETH peripheral state to BUSY */
  heth->State = HAL_ETH_STATE_BUSY;
  
  if (FrameLength == 0U) 
  {
    /* Set ETH HAL state to READY */
    heth->State = HAL_ETH_STATE_READY;
    
    /* Process Unlocked */
    __HAL_UNLOCK(heth);
    
    return  HAL_ERROR;                                    
  }  
  
  /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
  if(((heth->TxDesc)->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
  {  
    /* OWN bit set */
    heth->State = HAL_ETH_STATE_BUSY_TX;
    
    /* Process Unlocked */
    __HAL_UNLOCK(heth);
    
    return HAL_ERROR;
  }
  
  /* Get the number of needed Tx buffers for the current frame */
  if (FrameLength > ETH_TX_BUF_SIZE)
  {
    bufcount = FrameLength/ETH_TX_BUF_SIZE;
    if (FrameLength % ETH_TX_BUF_SIZE) 
    {
      bufcount++;
    }
  }
  else 
  {  
    bufcount = 1U;
  }
  if (bufcount == 1U)
  {
    /* Set LAST and FIRST segment */
    heth->TxDesc->Status |=ETH_DMATXDESC_FS|ETH_DMATXDESC_LS;
    /* Set frame size */
    heth->TxDesc->ControlBufferSize = (FrameLength & ETH_DMATXDESC_TBS1);
    /* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA */
    heth->TxDesc->Status |= ETH_DMATXDESC_OWN;
    /* Point to next descriptor */
    heth->TxDesc= (ETH_DMADescTypeDef *)(heth->TxDesc->Buffer2NextDescAddr);
  }
  else
  {
    for (i=0U; i< bufcount; i++)
    {
      /* Clear FIRST and LAST segment bits */
      heth->TxDesc->Status &= ~(ETH_DMATXDESC_FS | ETH_DMATXDESC_LS);
      
      if (i == 0U) 
      {
        /* Setting the first segment bit */
        heth->TxDesc->Status |= ETH_DMATXDESC_FS;  
      }
      
      /* Program size */
      heth->TxDesc->ControlBufferSize = (ETH_TX_BUF_SIZE & ETH_DMATXDESC_TBS1);
      
      if (i == (bufcount-1U))
      {
        /* Setting the last segment bit */
        heth->TxDesc->Status |= ETH_DMATXDESC_LS;
        size = FrameLength - (bufcount-1U)*ETH_TX_BUF_SIZE;
        heth->TxDesc->ControlBufferSize = (size & ETH_DMATXDESC_TBS1);
      }
      
      /* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA */
      heth->TxDesc->Status |= ETH_DMATXDESC_OWN;
      /* point to next descriptor */
      heth->TxDesc = (ETH_DMADescTypeDef *)(heth->TxDesc->Buffer2NextDescAddr);
    }
  }
  
  /* When Tx Buffer unavailable flag is set: clear it and resume transmission */
  if (((heth->Instance)->DMASR & ETH_DMASR_TBUS) != (uint32_t)RESET)
  {
    /* Clear TBUS ETHERNET DMA flag */
    (heth->Instance)->DMASR = ETH_DMASR_TBUS;
    /* Resume DMA transmission*/
    (heth->Instance)->DMATPDR = 0U;
  }
  
  /* Set ETH HAL State to Ready */
  heth->State = HAL_ETH_STATE_READY;
  
  /* Process Unlocked */
  __HAL_UNLOCK(heth);
  
  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Checks for received frames. 
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_GetReceivedFrame(ETH_HandleTypeDef *heth)
{
  uint32_t framelength = 0U;
  
  /* Process Locked */
  __HAL_LOCK(heth);
  
  /* Check the ETH state to BUSY */
  heth->State = HAL_ETH_STATE_BUSY;
  
  /* Check if segment is not owned by DMA */
  /* (((heth->RxDesc->Status & ETH_DMARXDESC_OWN) == (uint32_t)RESET) && ((heth->RxDesc->Status & ETH_DMARXDESC_LS) != (uint32_t)RESET)) */
  if(((heth->RxDesc->Status & ETH_DMARXDESC_OWN) == (uint32_t)RESET))
  {
    /* Check if last segment */
    if(((heth->RxDesc->Status & ETH_DMARXDESC_LS) != (uint32_t)RESET)) 
    {
      /* increment segment count */
      (heth->RxFrameInfos).SegCount++;
      
      /* Check if last segment is first segment: one segment contains the frame */
      if ((heth->RxFrameInfos).SegCount == 1U)
      {
        (heth->RxFrameInfos).FSRxDesc =heth->RxDesc;
      }
      
      heth->RxFrameInfos.LSRxDesc = heth->RxDesc;
      
      /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
      framelength = (((heth->RxDesc)->Status & ETH_DMARXDESC_FL) >> ETH_DMARXDESC_FRAMELENGTHSHIFT) - 4U;
      heth->RxFrameInfos.length = framelength;
      
      /* Get the address of the buffer start address */
      heth->RxFrameInfos.buffer = ((heth->RxFrameInfos).FSRxDesc)->Buffer1Addr;
      /* point to next descriptor */
      heth->RxDesc = (ETH_DMADescTypeDef*) ((heth->RxDesc)->Buffer2NextDescAddr);
      
      /* Set HAL State to Ready */
      heth->State = HAL_ETH_STATE_READY;
      
      /* Process Unlocked */
      __HAL_UNLOCK(heth);
      
      /* Return function status */
      return HAL_OK;
    }
    /* Check if first segment */
    else if((heth->RxDesc->Status & ETH_DMARXDESC_FS) != (uint32_t)RESET)
    {
      (heth->RxFrameInfos).FSRxDesc = heth->RxDesc;
      (heth->RxFrameInfos).LSRxDesc = NULL;
      (heth->RxFrameInfos).SegCount = 1U;
      /* Point to next descriptor */
      heth->RxDesc = (ETH_DMADescTypeDef*) (heth->RxDesc->Buffer2NextDescAddr);
    }
    /* Check if intermediate segment */ 
    else
    {
      (heth->RxFrameInfos).SegCount++;
      /* Point to next descriptor */
      heth->RxDesc = (ETH_DMADescTypeDef*) (heth->RxDesc->Buffer2NextDescAddr);
    } 
  }
  
  /* Set ETH HAL State to Ready */
  heth->State = HAL_ETH_STATE_READY;
  
  /* Process Unlocked */
  __HAL_UNLOCK(heth);
  
  /* Return function status */
  return HAL_ERROR;
}

/**
  * @brief  Gets the Received frame in interrupt mode. 
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_GetReceivedFrame_IT(ETH_HandleTypeDef *heth)
{
  uint32_t descriptorscancounter = 0U;
  
  /* Process Locked */
  __HAL_LOCK(heth);
  
  /* Set ETH HAL State to BUSY */
  heth->State = HAL_ETH_STATE_BUSY;
  
  /* Scan descriptors owned by CPU */
  while (((heth->RxDesc->Status & ETH_DMARXDESC_OWN) == (uint32_t)RESET) && (descriptorscancounter < ETH_RXBUFNB))
  {
    /* Just for security */
    descriptorscancounter++;
    
    /* Check if first segment in frame */
    /* ((heth->RxDesc->Status & ETH_DMARXDESC_FS) != (uint32_t)RESET) && ((heth->RxDesc->Status & ETH_DMARXDESC_LS) == (uint32_t)RESET)) */  
    if((heth->RxDesc->Status & (ETH_DMARXDESC_FS | ETH_DMARXDESC_LS)) == (uint32_t)ETH_DMARXDESC_FS)
    { 
      heth->RxFrameInfos.FSRxDesc = heth->RxDesc;
      heth->RxFrameInfos.SegCount = 1U;   
      /* Point to next descriptor */
      heth->RxDesc = (ETH_DMADescTypeDef*) (heth->RxDesc->Buffer2NextDescAddr);
    }
    /* Check if intermediate segment */
    /* ((heth->RxDesc->Status & ETH_DMARXDESC_LS) == (uint32_t)RESET)&& ((heth->RxDesc->Status & ETH_DMARXDESC_FS) == (uint32_t)RESET)) */
    else if ((heth->RxDesc->Status & (ETH_DMARXDESC_LS | ETH_DMARXDESC_FS)) == (uint32_t)RESET)
    {
      /* Increment segment count */
      (heth->RxFrameInfos.SegCount)++;
      /* Point to next descriptor */
      heth->RxDesc = (ETH_DMADescTypeDef*)(heth->RxDesc->Buffer2NextDescAddr);
    }
    /* Should be last segment */
    else
    { 
      /* Last segment */
      heth->RxFrameInfos.LSRxDesc = heth->RxDesc;
      
      /* Increment segment count */
      (heth->RxFrameInfos.SegCount)++;
      
      /* Check if last segment is first segment: one segment contains the frame */
      if ((heth->RxFrameInfos.SegCount) == 1U)
      {
        heth->RxFrameInfos.FSRxDesc = heth->RxDesc;
      }
      
      /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
      heth->RxFrameInfos.length = (((heth->RxDesc)->Status & ETH_DMARXDESC_FL) >> ETH_DMARXDESC_FRAMELENGTHSHIFT) - 4U;
      
      /* Get the address of the buffer start address */ 
      heth->RxFrameInfos.buffer =((heth->RxFrameInfos).FSRxDesc)->Buffer1Addr;
      
      /* Point to next descriptor */      
      heth->RxDesc = (ETH_DMADescTypeDef*) (heth->RxDesc->Buffer2NextDescAddr);
      
      /* Set HAL State to Ready */
      heth->State = HAL_ETH_STATE_READY;
      
      /* Process Unlocked */
      __HAL_UNLOCK(heth);
  
      /* Return function status */
      return HAL_OK;
    }
  }

  /* Set HAL State to Ready */
  heth->State = HAL_ETH_STATE_READY;
  
  /* Process Unlocked */
  __HAL_UNLOCK(heth);
  
  /* Return function status */
  return HAL_ERROR;
}

/**
  * @brief  This function handles ETH interrupt request.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
void HAL_ETH_IRQHandler(ETH_HandleTypeDef *heth)
{
  /* Frame received */
  if (__HAL_ETH_DMA_GET_FLAG(heth, ETH_DMA_FLAG_R)) 
  {
#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
    /*Call registered Receive complete callback*/
    heth->RxCpltCallback(heth);
#else
    /* Receive complete callback */
    HAL_ETH_RxCpltCallback(heth);
#endif /* USE_HAL_ETH_REGISTER_CALLBACKS */

     /* Clear the Eth DMA Rx IT pending bits */
    __HAL_ETH_DMA_CLEAR_IT(heth, ETH_DMA_IT_R);

    /* Set HAL State to Ready */
    heth->State = HAL_ETH_STATE_READY;
    
    /* Process Unlocked */
    __HAL_UNLOCK(heth);

  }
  /* Frame transmitted */
  else if (__HAL_ETH_DMA_GET_FLAG(heth, ETH_DMA_FLAG_T)) 
  {
#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
    /*  Call resgistered Transfer complete callback*/
    heth->TxCpltCallback(heth);
#else
    /* Transfer complete callback */
    HAL_ETH_TxCpltCallback(heth);
#endif /* USE_HAL_ETH_REGISTER_CALLBACKS */

    /* Clear the Eth DMA Tx IT pending bits */
    __HAL_ETH_DMA_CLEAR_IT(heth, ETH_DMA_IT_T);

    /* Set HAL State to Ready */
    heth->State = HAL_ETH_STATE_READY;
    
    /* Process Unlocked */
    __HAL_UNLOCK(heth);
  }
  
  /* Clear the interrupt flags */
  __HAL_ETH_DMA_CLEAR_IT(heth, ETH_DMA_IT_NIS);
  
  /* ETH DMA Error */
  if(__HAL_ETH_DMA_GET_FLAG(heth, ETH_DMA_FLAG_AIS))
  {
#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
    heth->DMAErrorCallback(heth);
#else
    /* Ethernet Error callback */
    HAL_ETH_ErrorCallback(heth);
#endif /* USE_HAL_ETH_REGISTER_CALLBACKS */

    /* Clear the interrupt flags */
    __HAL_ETH_DMA_CLEAR_IT(heth, ETH_DMA_FLAG_AIS);
  
    /* Set HAL State to Ready */
    heth->State = HAL_ETH_STATE_READY;
    
    /* Process Unlocked */
    __HAL_UNLOCK(heth);
  }
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
__weak void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(heth);
  /* NOTE : This function Should not be modified, when the callback is needed,
  the HAL_ETH_TxCpltCallback could be implemented in the user file
  */ 
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
__weak void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(heth);
  /* NOTE : This function Should not be modified, when the callback is needed,
  the HAL_ETH_TxCpltCallback could be implemented in the user file
  */ 
}

/**
  * @brief  Ethernet transfer error callbacks
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
__weak void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(heth);
  /* NOTE : This function Should not be modified, when the callback is needed,
  the HAL_ETH_TxCpltCallback could be implemented in the user file
  */ 
}

/**
  * @brief  Reads or writes a PHY register
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param PHYReg PHY register address, is the index of one of the 32 PHY register.
  *                This parameter can be one of the following values:
  *                   PHY_BCR: Transceiver Basic Control Register,
  *                   PHY_BSR: Transceiver Basic Status Register.
  *                   More PHY register could be read depending on the used PHY
  * @param forWrite non zero if writing to the PHY, zero if reading.
  * @param RegValue pointer to the PHY register value to set, or where to store its value
  * @retval HAL status TIMEOUT or OK
  */
static HAL_StatusTypeDef accessPHY(ETH_HandleTypeDef *heth,uint16_t PHYReg, _Bool forWrite, uint32_t *RegValue) {
  assert_param(IS_ETH_PHY_ADDRESS(heth->Init.PhyAddress));
  /*  point to bitband address of busy bit. */
  __IO uint32_t * MiiBusy=(uint32_t *)(0x42000000 + ((0x28000+0x10)<<5)+(0<<2));//sure would be nice if the HAL had an official macro for bit band accessors
  /*the example code had separate busy flags for mii read and write, but each operation interferes with the other, you can't do either when the other is active. That code only checked for the value about to be set.
   * Also if we don't have ISR's trying to talk to the phy then we don't need to do this check,
   * OTOH if they are active then the following needs to be atomic. Since HAL didn't do that apparently no-one actually does work in ISR's (or the code is flaky) */
  if (heth->State == HAL_ETH_STATE_BUSY_WR) { //see above, there should be just one flag, I chose existing _WR.
    return HAL_BUSY;
  }
  heth->State = HAL_ETH_STATE_BUSY_WR;
  //and should also check the MiiBisy bit, if set then there is a major source code error- in this routine or a random write to the Miiar.
  //end of "should be atomic"
  if (forWrite) {//must fit between check for busy and setting the iiar
    heth->Instance->MACMIIDR = *RegValue; //breakpoint here to trap all phy writes
  }
  uint32_t miiar = heth->Instance->MACMIIAR;
  /* Keep only the CSR Clock Range CR[2:0] bits value */
  miiar &= ~ETH_MACMIIAR_CR_MASK;
  /* Prepare the MII address register value */
  miiar |= ((heth->Init.PhyAddress << 11) & ETH_MACMIIAR_PA); /* Set the PHY device address   */
  miiar |= ((PHYReg << 6) & ETH_MACMIIAR_MR);                 /* Set the PHY register address */
  if (forWrite) {
    miiar |= ETH_MACMIIAR_MW;                                 /* Set the write mode */
  }
  //is already zero due to CR_MASK preservation step
  //  else {
  //    miiar &= ~ETH_MACMIIAR_MW;                            /* Set the read mode  */
  //  }
  miiar |= ETH_MACMIIAR_MB;                                   /* Set the MII Busy bit   */
  heth->Instance->MACMIIAR = miiar; //triggers SNI activity, ~40 uSec each time, ~20 uSec if you configure to only send the synch preamble once.

  uint32_t tickstart = HAL_GetTick();
  do {
    if (!*MiiBusy) {
      if (!forWrite) {
        *RegValue = heth->Instance->MACMIIDR; //breakpoint here to trap all phy reads
      }
      heth->State = HAL_ETH_STATE_READY;
      return HAL_OK;
    }
    /* sticking in the following waitpoll call is why the PHY access got refactored.
     * This is only important if the worst case PHY busy could actually take as long as the demo code suggested.
     * Since that is probably way wrong we might want to drop this hook.
     * The worst case is 64 clocks of the MII, and the slowest clock rate is ~1.5MHz, for 42 microseconds max*/
    ETH_whileWaiting();
    /* formerly the hardcoded 2 below was either PHY_READ_TO or PHY_WRITE_TO but the given values for those
       were absurd overkills, more than a factor of 1000 too long.
       The 2 ensures at least one full ms happens.
     */
  } while (HAL_GetTick()-tickstart< 2);

  heth->State = HAL_ETH_STATE_RESET;
  /* Process Unlocked */
  __HAL_UNLOCK(heth);//who locked it?
  return HAL_TIMEOUT;
}

/**
  * @brief  Reads a PHY register
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module                  
  * @param PHYReg PHY register address, is the index of one of the 32 PHY register. 
  *                This parameter can be one of the following values: 
  *                   PHY_BCR: Transceiver Basic Control Register, 
  *                   PHY_BSR: Transceiver Basic Status Register.   
  *                   More PHY register could be read depending on the used PHY
  * @param RegValue PHY register value                  
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_ReadPHYRegister(ETH_HandleTypeDef *heth, uint16_t PHYReg, uint32_t *RegValue)
{
  return accessPHY(heth, PHYReg, 0,RegValue);
}

/**
  * @brief  Writes to a PHY register.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module  
  * @param  PHYReg PHY register address, is the index of one of the 32 PHY register. 
  *          This parameter can be one of the following values: 
  *             PHY_BCR: Transceiver Control Register.  
  *             More PHY register could be written depending on the used PHY
  * @param  RegValue the value to write
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_WritePHYRegister(ETH_HandleTypeDef *heth, uint16_t PHYReg, uint32_t RegValue)
{
  return accessPHY(heth, PHYReg, 1,&RegValue);
}

/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group3 Peripheral Control functions
 *  @brief    Peripheral Control functions 
 *
@verbatim   
 ===============================================================================
                  ##### Peripheral Control functions #####
 ===============================================================================  
    [..]  This section provides functions allowing to:
      (+) Enable MAC and DMA transmission and reception.
          HAL_ETH_Start();
      (+) Disable MAC and DMA transmission and reception. 
          HAL_ETH_Stop();
      (+) Set the MAC configuration in runtime mode
          HAL_ETH_ConfigMAC();
      (+) Set the DMA configuration in runtime mode
          HAL_ETH_ConfigDMA();

@endverbatim
  * @{
  */ 

 /**
  * @brief  Enables Ethernet MAC and DMA reception/transmission 
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_Start(ETH_HandleTypeDef *heth)
{  
  /* Process Locked */
  __HAL_LOCK(heth);
  
  /* Set the ETH peripheral state to BUSY */
  heth->State = HAL_ETH_STATE_BUSY;
  
  /* Enable transmit state machine of the MAC for transmission on the MII */
  ETH_MACTransmissionEnable(heth);
  
  /* Enable receive state machine of the MAC for reception from the MII */
  ETH_MACReceptionEnable(heth);
  
  /* Flush Transmit FIFO */
  ETH_FlushTransmitFIFO(heth);
  
  /* Start DMA transmission */
  ETH_DMATransmissionEnable(heth);
  
  /* Start DMA reception */
  ETH_DMAReceptionEnable(heth);
  
  /* Set the ETH state to READY*/
  heth->State= HAL_ETH_STATE_READY;
  
  /* Process Unlocked */
  __HAL_UNLOCK(heth);
  
  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Stop Ethernet MAC and DMA reception/transmission 
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_Stop(ETH_HandleTypeDef *heth)
{  
  /* Process Locked */
  __HAL_LOCK(heth);
  
  /* Set the ETH peripheral state to BUSY */
  heth->State = HAL_ETH_STATE_BUSY;
  
  /* Stop DMA transmission */
  ETH_DMATransmissionDisable(heth);
  
  /* Stop DMA reception */
  ETH_DMAReceptionDisable(heth);
  
  /* Disable receive state machine of the MAC for reception from the MII */
  ETH_MACReceptionDisable(heth);
  
  /* Flush Transmit FIFO */
  ETH_FlushTransmitFIFO(heth);
  
  /* Disable transmit state machine of the MAC for transmission on the MII */
  ETH_MACTransmissionDisable(heth);
  
  /* Set the ETH state*/
  heth->State = HAL_ETH_STATE_READY;
  
  /* Process Unlocked */
  __HAL_UNLOCK(heth);
  
  /* Return function status */
  return HAL_OK;
}

static void ETH_ConfigMacCR(const ETH_HandleTypeDef *heth, const ETH_MACInitTypeDef *macconf)
{/*------------------------ ETHERNET MACCR Configuration --------------------*/  //duplicate code chunk
  /* Set the WD bit according to ETH Watchdog value */
  /* Set the JD: bit according to ETH Jabber value */
  /* Set the IFG bit according to ETH InterFrameGap value */
  /* Set the DCRS bit according to ETH CarrierSense value */
  /* Set the FES bit according to ETH Speed value */
  /* Set the DO bit according to ETH ReceiveOwn value */
  /* Set the LM bit according to ETH LoopbackMode value */
  /* Set the DM bit according to ETH Mode value */
  /* Set the IPCO bit according to ETH ChecksumOffload value */
  /* Set the DR bit according to ETH RetryTransmission value */
  /* Set the ACS bit according to ETH AutomaticPadCRCStrip value */
  /* Set the BL bit according to ETH BackOffLimit value */
  /* Set the DC bit according to ETH DeferralCheck value */
  /* Clear WD, PCE, PS, TE and RE bits */
  ETH_MacFiddle(heth,&(heth->Instance->MACCR),(macconf->Watchdog |
                                               macconf->Jabber |
                                               macconf->InterFrameGap |
                                               macconf->CarrierSense |
                                               (heth->Init).Speed |
                                               macconf->ReceiveOwn |
                                               macconf->LoopbackMode |
                                               (heth->Init).DuplexMode |
                                               macconf->ChecksumOffload |
                                               macconf->RetryTransmission |
                                               macconf->AutomaticPadCRCStrip |
                                               macconf->BackOffLimit |
                                               macconf->DeferralCheck),ETH_MACCR_CLEAR_MASK);
}

static void ConfigMacFCR(const ETH_HandleTypeDef *heth, ETH_MACInitTypeDef *macinit)
{
  /* Set the PT bit according to ETH PauseTime value */
  /* Set the DZPQ bit according to ETH ZeroQuantaPause value */
  /* Set the PLT bit according to ETH PauseLowThreshold value */
  /* Set the UP bit according to ETH UnicastPauseFrameDetect value */
  /* Set the RFE bit according to ETH ReceiveFlowControl value */
  /* Set the TFE bit according to ETH TransmitFlowControl value */
  ETH_MacFiddle(heth, &(heth->Instance->MACFCR), ((*macinit).PauseTime << 16U) |
                                                 (*macinit).ZeroQuantaPause |
                                                 (*macinit).PauseLowThreshold |
                                                 (*macinit).UnicastPauseFrameDetect |
                                                 (*macinit).ReceiveFlowControl |
                                                 (*macinit).TransmitFlowControl, ETH_MACFCR_CLEAR_MASK);
}

static void ConfigMacVLAN(const ETH_HandleTypeDef *heth, ETH_MACInitTypeDef *macinit)
{
  /* Set the ETV bit according to ETH VLANTagComparison value */
  /* Set the VL bit according to ETH VLANTagIdentifier value */
  ETH_MacFiddle(heth, &(heth->Instance->MACVLANTR),
    (macinit->VLANTagComparison |
    macinit->VLANTagIdentifier),
    0);
}

static void ConfigMacFFR(const ETH_HandleTypeDef *heth, const ETH_MACInitTypeDef *macconf)
{
  /* Set the RA bit according to ETH ReceiveAll value */
  /* Set the SAF and SAIF bits according to ETH SourceAddrFilter value */
  /* Set the PCF bit according to ETH PassControlFrames value */
  /* Set the DBF bit according to ETH BroadcastFramesReception value */
  /* Set the DAIF bit according to ETH DestinationAddrFilter value */
  /* Set the PR bit according to ETH PromiscuousMode value */
  /* Set the PM, HMC and HPF bits according to ETH MulticastFramesFilter value */
  /* Set the HUC and HPF bits according to ETH UnicastFramesFilter value */
  ETH_MacFiddle(heth,&(heth->Instance->MACFFR),(macconf->ReceiveAll |
                                        macconf->SourceAddrFilter |
                                        macconf->PassControlFrames |
                                        macconf->BroadcastFramesReception |
                                        macconf->DestinationAddrFilter |
                                        macconf->PromiscuousMode |
                                        macconf->MulticastFramesFilter |
                                        macconf->UnicastFramesFilter),0);
}
/**
  * @brief  Set ETH MAC Configuration.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  macconf MAC Configuration structure  
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_ConfigMAC(ETH_HandleTypeDef *heth, ETH_MACInitTypeDef *macconf)
{
  /* Process Locked */
  __HAL_LOCK(heth);
  
  /* Set the ETH peripheral state to BUSY */
  heth->State= HAL_ETH_STATE_BUSY;
  
  assert_param(IS_ETH_SPEED(heth->Init.Speed));
  assert_param(IS_ETH_DUPLEX_MODE(heth->Init.DuplexMode)); 
  
  if (macconf != NULL)
  {
    /* Check the parameters */
    assert_param(IS_ETH_WATCHDOG(macconf->Watchdog));
    assert_param(IS_ETH_JABBER(macconf->Jabber));
    assert_param(IS_ETH_INTER_FRAME_GAP(macconf->InterFrameGap));
    assert_param(IS_ETH_CARRIER_SENSE(macconf->CarrierSense));
    assert_param(IS_ETH_RECEIVE_OWN(macconf->ReceiveOwn));
    assert_param(IS_ETH_LOOPBACK_MODE(macconf->LoopbackMode));
    assert_param(IS_ETH_CHECKSUM_OFFLOAD(macconf->ChecksumOffload));
    assert_param(IS_ETH_RETRY_TRANSMISSION(macconf->RetryTransmission));
    assert_param(IS_ETH_AUTOMATIC_PADCRC_STRIP(macconf->AutomaticPadCRCStrip));
    assert_param(IS_ETH_BACKOFF_LIMIT(macconf->BackOffLimit));
    assert_param(IS_ETH_DEFERRAL_CHECK(macconf->DeferralCheck));
    assert_param(IS_ETH_RECEIVE_ALL(macconf->ReceiveAll));
    assert_param(IS_ETH_SOURCE_ADDR_FILTER(macconf->SourceAddrFilter));
    assert_param(IS_ETH_CONTROL_FRAMES(macconf->PassControlFrames));
    assert_param(IS_ETH_BROADCAST_FRAMES_RECEPTION(macconf->BroadcastFramesReception));
    assert_param(IS_ETH_DESTINATION_ADDR_FILTER(macconf->DestinationAddrFilter));
    assert_param(IS_ETH_PROMISCUOUS_MODE(macconf->PromiscuousMode));
    assert_param(IS_ETH_MULTICAST_FRAMES_FILTER(macconf->MulticastFramesFilter));
    assert_param(IS_ETH_UNICAST_FRAMES_FILTER(macconf->UnicastFramesFilter));
    assert_param(IS_ETH_PAUSE_TIME(macconf->PauseTime));
    assert_param(IS_ETH_ZEROQUANTA_PAUSE(macconf->ZeroQuantaPause));
    assert_param(IS_ETH_PAUSE_LOW_THRESHOLD(macconf->PauseLowThreshold));
    assert_param(IS_ETH_UNICAST_PAUSE_FRAME_DETECT(macconf->UnicastPauseFrameDetect));
    assert_param(IS_ETH_RECEIVE_FLOWCONTROL(macconf->ReceiveFlowControl));
    assert_param(IS_ETH_TRANSMIT_FLOWCONTROL(macconf->TransmitFlowControl));
    assert_param(IS_ETH_VLAN_TAG_COMPARISON(macconf->VLANTagComparison));
    assert_param(IS_ETH_VLAN_TAG_IDENTIFIER(macconf->VLANTagIdentifier));
    ETH_ConfigMacCR(heth, macconf);

    /*----------------------- ETHERNET MACFFR Configuration --------------------*/
    ConfigMacFFR(heth, macconf);

    /*--------------- ETHERNET MACHTHR and MACHTLR Configuration ---------------*/
     /* Write to ETHERNET MACHTHR */
     (heth->Instance)->MACHTHR = (uint32_t)macconf->HashTableHigh;
     
     /* Write to ETHERNET MACHTLR */
     (heth->Instance)->MACHTLR = (uint32_t)macconf->HashTableLow;
     /*----------------------- ETHERNET MACFCR Configuration --------------------*/
    ConfigMacFCR(heth,macconf);

     /*----------------------- ETHERNET MACVLANTR Configuration -----------------*/
     ConfigMacVLAN(heth,macconf);
  }
  else /* macconf == NULL : here we just configure Speed and Duplex mode */
  {
    /*------------------------ ETHERNET MACCR Configuration --------------------*/
    ETH_MacFiddle(heth, &(heth->Instance->MACCR),heth->Init.Speed | heth->Init.DuplexMode,0x00004800U);/* Clear FES and DM bits */
  }
  
  /* Set the ETH state to Ready */
  heth->State= HAL_ETH_STATE_READY;
  
  /* Process Unlocked */
  __HAL_UNLOCK(heth);
  
  /* Return function status */
  return HAL_OK;  
}


static void ConfigDMAOMR(const ETH_HandleTypeDef *heth, ETH_DMAInitTypeDef *dmainit)
{
  /* Set the DT bit according to ETH DropTCPIPChecksumErrorFrame value */
  /* Set the RSF bit according to ETH ReceiveStoreForward value */
  /* Set the DFF bit according to ETH FlushReceivedFrame value */
  /* Set the TSF bit according to ETH TransmitStoreForward value */
  /* Set the TTC bit according to ETH TransmitThresholdControl value */
  /* Set the FEF bit according to ETH ForwardErrorFrames value */
  /* Set the FUF bit according to ETH ForwardUndersizedGoodFrames value */
  /* Set the RTC bit according to ETH ReceiveThresholdControl value */
  /* Set the OSF bit according to ETH SecondFrameOperate value */
  ETH_MacFiddle(heth, &(heth->Instance->DMAOMR), dmainit->DropTCPIPChecksumErrorFrame |
                                                 dmainit->ReceiveStoreForward |
                                                 dmainit->FlushReceivedFrame |
                                                 dmainit->TransmitStoreForward |
                                                 dmainit->TransmitThresholdControl |
                                                 dmainit->ForwardErrorFrames |
                                                 dmainit->ForwardUndersizedGoodFrames |
                                                 dmainit->ReceiveThresholdControl |
                                                 dmainit->SecondFrameOperate, ETH_DMAOMR_CLEAR_MASK);
}

static void ConfigDMABMR(const ETH_HandleTypeDef *heth, const ETH_DMAInitTypeDef *dmainit)
{
  /* Set the AAL bit according to ETH AddressAlignedBeats value */
  /* Set the FB bit according to ETH FixedBurst value */
  /* Set the RPBL and 4*PBL bits according to ETH RxDMABurstLength value */
  /* Set the PBL and 4*PBL bits according to ETH TxDMABurstLength value */
  /* !! if 4xPBL is selected for Tx or Rx it is applied for the other */
  /* Set the Enhanced DMA descriptors bit according to ETH EnhancedDescriptorFormat value*/
  /* Set the DSL bit according to ETH DesciptorSkipLength value */
  /* Set the PR and DA bits according to ETH DMAArbitration value */
  /* Enable use of separate PBL for Rx and Tx */
  ETH_MacFiddle(heth, &(heth->Instance->DMABMR), dmainit->AddressAlignedBeats |
                                                 dmainit->FixedBurst |
                                                 dmainit->RxDMABurstLength |
                                                 dmainit->TxDMABurstLength |
                                                 dmainit->EnhancedDescriptorFormat |
                                                 (dmainit->DescriptorSkipLength << 2U) |
                                                 dmainit->DMAArbitration |
                                                 ETH_DMABMR_USP, 0);

}

/**
  * @brief  Sets ETH DMA Configuration.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  dmaconf DMA Configuration structure  
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ETH_ConfigDMA(ETH_HandleTypeDef *heth, ETH_DMAInitTypeDef *dmaconf)
{
  /* Process Locked */
  __HAL_LOCK(heth);
  
  /* Set the ETH peripheral state to BUSY */
  heth->State= HAL_ETH_STATE_BUSY;

  /* Check parameters */
  assert_param(IS_ETH_DROP_TCPIP_CHECKSUM_FRAME(dmaconf->DropTCPIPChecksumErrorFrame));
  assert_param(IS_ETH_RECEIVE_STORE_FORWARD(dmaconf->ReceiveStoreForward));
  assert_param(IS_ETH_FLUSH_RECEIVE_FRAME(dmaconf->FlushReceivedFrame));
  assert_param(IS_ETH_TRANSMIT_STORE_FORWARD(dmaconf->TransmitStoreForward));
  assert_param(IS_ETH_TRANSMIT_THRESHOLD_CONTROL(dmaconf->TransmitThresholdControl));
  assert_param(IS_ETH_FORWARD_ERROR_FRAMES(dmaconf->ForwardErrorFrames));
  assert_param(IS_ETH_FORWARD_UNDERSIZED_GOOD_FRAMES(dmaconf->ForwardUndersizedGoodFrames));
  assert_param(IS_ETH_RECEIVE_THRESHOLD_CONTROL(dmaconf->ReceiveThresholdControl));
  assert_param(IS_ETH_SECOND_FRAME_OPERATE(dmaconf->SecondFrameOperate));
  assert_param(IS_ETH_ADDRESS_ALIGNED_BEATS(dmaconf->AddressAlignedBeats));
  assert_param(IS_ETH_FIXED_BURST(dmaconf->FixedBurst));
  assert_param(IS_ETH_RXDMA_BURST_LENGTH(dmaconf->RxDMABurstLength));
  assert_param(IS_ETH_TXDMA_BURST_LENGTH(dmaconf->TxDMABurstLength));
  assert_param(IS_ETH_ENHANCED_DESCRIPTOR_FORMAT(dmaconf->EnhancedDescriptorFormat));
  assert_param(IS_ETH_DMA_DESC_SKIP_LENGTH(dmaconf->DescriptorSkipLength));
  assert_param(IS_ETH_DMA_ARBITRATION_ROUNDROBIN_RXTX(dmaconf->DMAArbitration));
  
  /*----------------------- ETHERNET DMAOMR Configuration --------------------*/
  ConfigDMAOMR(heth,dmaconf);

  /*----------------------- ETHERNET DMABMR Configuration --------------------*/
  ConfigDMABMR(heth, dmaconf);

  /* Set the ETH state to Ready */
   heth->State= HAL_ETH_STATE_READY;
   
   /* Process Unlocked */
   __HAL_UNLOCK(heth);
   
   /* Return function status */
   return HAL_OK; 
}

/**
  * @}
  */

/** @defgroup ETH_Exported_Functions_Group4 Peripheral State functions 
  *  @brief   Peripheral State functions 
  *
  @verbatim   
  ===============================================================================
                         ##### Peripheral State functions #####
  ===============================================================================  
  [..]
  This subsection permits to get in run-time the status of the peripheral 
  and the data flow.
       (+) Get the ETH handle state:
           HAL_ETH_GetState();
           

  @endverbatim
  * @{
  */

/**
  * @brief  Return the ETH HAL state
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL state
  */
HAL_ETH_StateTypeDef HAL_ETH_GetState(ETH_HandleTypeDef *heth)
{  
  /* Return ETH state */
  return heth->State;
}

/**
  * @}
  */
  
/**
  * @}
  */
  
/** @addtogroup ETH_Private_Functions
  * @{
  */

/**
  * @brief  Configures Ethernet MAC and DMA with default parameters.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  err Ethernet Init error
  * @retval HAL status
  */
static void ETH_MACDMAConfig(ETH_HandleTypeDef *heth, uint32_t err)
{
  ETH_MACInitTypeDef macinit;
  ETH_DMAInitTypeDef dmainit;
  
  if (err != ETH_SUCCESS) /* Auto-negotiation failed */
  {
    /* Set Ethernet duplex mode to Full-duplex */
    (heth->Init).DuplexMode = ETH_MODE_FULLDUPLEX;
    
    /* Set Ethernet speed to 100M */
    (heth->Init).Speed = ETH_SPEED_100M;
  }
  
  /* Ethernet MAC default initialization **************************************/
  macinit.Watchdog = ETH_WATCHDOG_ENABLE;
  macinit.Jabber = ETH_JABBER_ENABLE;
  macinit.InterFrameGap = ETH_INTERFRAMEGAP_96BIT;
  macinit.CarrierSense = ETH_CARRIERSENCE_ENABLE;
  macinit.ReceiveOwn = ETH_RECEIVEOWN_ENABLE;
  macinit.LoopbackMode = ETH_LOOPBACKMODE_DISABLE;
  if(heth->Init.ChecksumMode == ETH_CHECKSUM_BY_HARDWARE)
  {
    macinit.ChecksumOffload = ETH_CHECKSUMOFFLAOD_ENABLE;
  }
  else
  {
    macinit.ChecksumOffload = ETH_CHECKSUMOFFLAOD_DISABLE;
  }
  macinit.RetryTransmission = ETH_RETRYTRANSMISSION_DISABLE;
  macinit.AutomaticPadCRCStrip = ETH_AUTOMATICPADCRCSTRIP_DISABLE;
  macinit.BackOffLimit = ETH_BACKOFFLIMIT_10;
  macinit.DeferralCheck = ETH_DEFFERRALCHECK_DISABLE;
  macinit.ReceiveAll = ETH_RECEIVEAll_DISABLE;
  macinit.SourceAddrFilter = ETH_SOURCEADDRFILTER_DISABLE;
  macinit.PassControlFrames = ETH_PASSCONTROLFRAMES_BLOCKALL;
  macinit.BroadcastFramesReception = ETH_BROADCASTFRAMESRECEPTION_ENABLE;
  macinit.DestinationAddrFilter = ETH_DESTINATIONADDRFILTER_NORMAL;
  macinit.PromiscuousMode = ETH_PROMISCUOUS_MODE_DISABLE;
  macinit.MulticastFramesFilter = ETH_MULTICASTFRAMESFILTER_PERFECT;
  macinit.UnicastFramesFilter = ETH_UNICASTFRAMESFILTER_PERFECT;
  macinit.HashTableHigh = 0x0U;
  macinit.HashTableLow = 0x0U;
  macinit.PauseTime = 0x0U;
  macinit.ZeroQuantaPause = ETH_ZEROQUANTAPAUSE_DISABLE;
  macinit.PauseLowThreshold = ETH_PAUSELOWTHRESHOLD_MINUS4;
  macinit.UnicastPauseFrameDetect = ETH_UNICASTPAUSEFRAMEDETECT_DISABLE;
  macinit.ReceiveFlowControl = ETH_RECEIVEFLOWCONTROL_DISABLE;
  macinit.TransmitFlowControl = ETH_TRANSMITFLOWCONTROL_DISABLE;
  macinit.VLANTagComparison = ETH_VLANTAGCOMPARISON_16BIT;
  macinit.VLANTagIdentifier = 0x0U;
  
  /*------------------------ ETHERNET MACCR Configuration --------------------*/
  ETH_ConfigMacCR(heth, &macinit);

  
  /*----------------------- ETHERNET MACFFR Configuration --------------------*/ 
  ConfigMacFFR(heth,&macinit);
   
   /*--------------- ETHERNET MACHTHR and MACHTLR Configuration --------------*/
   /* Write to ETHERNET MACHTHR */
   (heth->Instance)->MACHTHR = (uint32_t)macinit.HashTableHigh;

   /* Write to ETHERNET MACHTLR */
   (heth->Instance)->MACHTLR = (uint32_t)macinit.HashTableLow;
   /*----------------------- ETHERNET MACFCR Configuration -------------------*/
   ConfigMacFCR(heth, &macinit);

  /*----------------------- ETHERNET MACVLANTR Configuration ----------------*/
   ConfigMacVLAN(heth,&macinit);

  /* Ethernet DMA default initialization ************************************/
  dmainit.DropTCPIPChecksumErrorFrame = ETH_DROPTCPIPCHECKSUMERRORFRAME_ENABLE;
  dmainit.ReceiveStoreForward = ETH_RECEIVESTOREFORWARD_ENABLE;
  dmainit.FlushReceivedFrame = ETH_FLUSHRECEIVEDFRAME_ENABLE;
  dmainit.TransmitStoreForward = ETH_TRANSMITSTOREFORWARD_ENABLE;
  dmainit.TransmitThresholdControl = ETH_TRANSMITTHRESHOLDCONTROL_64BYTES;
  dmainit.ForwardErrorFrames = ETH_FORWARDERRORFRAMES_DISABLE;
  dmainit.ForwardUndersizedGoodFrames = ETH_FORWARDUNDERSIZEDGOODFRAMES_DISABLE;
  dmainit.ReceiveThresholdControl = ETH_RECEIVEDTHRESHOLDCONTROL_64BYTES;
  dmainit.SecondFrameOperate = ETH_SECONDFRAMEOPERARTE_ENABLE;
  dmainit.AddressAlignedBeats = ETH_ADDRESSALIGNEDBEATS_ENABLE;
  dmainit.FixedBurst = ETH_FIXEDBURST_ENABLE;
  dmainit.RxDMABurstLength = ETH_RXDMABURSTLENGTH_32BEAT;
  dmainit.TxDMABurstLength = ETH_TXDMABURSTLENGTH_32BEAT;
  dmainit.EnhancedDescriptorFormat = ETH_DMAENHANCEDDESCRIPTOR_ENABLE;
  dmainit.DescriptorSkipLength = 0;
  dmainit.DMAArbitration = ETH_DMAARBITRATION_ROUNDROBIN_RXTX_1_1;

  ConfigDMAOMR(heth, &dmainit);
  /*----------------------- ETHERNET DMABMR Configuration ------------------*/
  ConfigDMABMR(heth,&dmainit);

   if((heth->Init).RxMode == ETH_RXINTERRUPT_MODE)
   {
     /* Enable the Ethernet Rx Interrupt */
     __HAL_ETH_DMA_ENABLE_IT((heth), ETH_DMA_IT_NIS | ETH_DMA_IT_R);
   }

   /* Initialize MAC address in ethernet MAC */
   ETH_MACAddressConfig(heth, ETH_MAC_ADDRESS0, heth->Init.MACAddr);
}

/**
  * @brief  Configures the selected MAC address.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  MacAddr The MAC address to configure
  *          This parameter can be one of the following values:
  *             @arg ETH_MAC_Address0: MAC Address0 
  *             @arg ETH_MAC_Address1: MAC Address1 
  *             @arg ETH_MAC_Address2: MAC Address2
  *             @arg ETH_MAC_Address3: MAC Address3
  * @param  Addr Pointer to MAC address buffer data (6 bytes)
  * @retval HAL status
  */
static void ETH_MACAddressConfig(ETH_HandleTypeDef *heth, uint32_t MacAddr, const uint8_t *Addr)
{
  uint32_t tmpreg1;
  
  /* Prevent unused argument(s) compilation warning */
  UNUSED(heth);

  /* Check the parameters */
  assert_param(IS_ETH_MAC_ADDRESS0123(MacAddr));
  
  /* Calculate the selected MAC address high register */
  tmpreg1 = ((uint32_t)Addr[5U] << 8U) | (uint32_t)Addr[4U];
  /* Load the selected MAC address high register */
  (*(__IO uint32_t *)((uint32_t)(ETH_MAC_ADDR_HBASE + MacAddr))) = tmpreg1;
  /* Calculate the selected MAC address low register */
  tmpreg1 = ((uint32_t)Addr[3U] << 24U) | ((uint32_t)Addr[2U] << 16U) | ((uint32_t)Addr[1U] << 8U) | Addr[0U];
  
  /* Load the selected MAC address low register */
  (*(__IO uint32_t *)((uint32_t)(ETH_MAC_ADDR_LBASE + MacAddr))) = tmpreg1;
}

/**
  * @brief  Enables the MAC transmission.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module  
  * @retval None
  */
static void ETH_MACTransmissionEnable(ETH_HandleTypeDef *heth)
{
  ETH_MacFiddle(heth, &(heth->Instance->MACCR), ETH_MACCR_TE,0);
}

/**
  * @brief  Disables the MAC transmission.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module  
  * @retval None
  */
static void ETH_MACTransmissionDisable(ETH_HandleTypeDef *heth)
{
  ETH_MacFiddle(heth, &(heth->Instance->MACCR), 0,ETH_MACCR_TE);
}

/**
  * @brief  Enables the MAC reception.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module   
  * @retval None
  */
static void ETH_MACReceptionEnable(ETH_HandleTypeDef *heth)
{
  ETH_MacFiddle(heth, &(heth->Instance->MACCR), ETH_MACCR_RE,0);
}

/**
  * @brief  Disables the MAC reception.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module   
  * @retval None
  */
static void ETH_MACReceptionDisable(ETH_HandleTypeDef *heth)
{
  ETH_MacFiddle(heth, &(heth->Instance->MACCR), 0,ETH_MACCR_RE);
}

/**
  * @brief  Enables the DMA transmission.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module   
  * @retval None
  */
static void ETH_DMATransmissionEnable(ETH_HandleTypeDef *heth)
{
  /* Enable the DMA transmission */
  (heth->Instance)->DMAOMR |= ETH_DMAOMR_ST;  
}

/**
  * @brief  Disables the DMA transmission.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module   
  * @retval None
  */
static void ETH_DMATransmissionDisable(ETH_HandleTypeDef *heth)
{ 
  /* Disable the DMA transmission */
  (heth->Instance)->DMAOMR &= ~ETH_DMAOMR_ST;
}

/**
  * @brief  Enables the DMA reception.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module 
  * @retval None
  */
static void ETH_DMAReceptionEnable(ETH_HandleTypeDef *heth)
{  
  /* Enable the DMA reception */
  (heth->Instance)->DMAOMR |= ETH_DMAOMR_SR;  
}

/**
  * @brief  Disables the DMA reception.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module 
  * @retval None
  */
static void ETH_DMAReceptionDisable(ETH_HandleTypeDef *heth)
{ 
  /* Disable the DMA reception */
  (heth->Instance)->DMAOMR &= ~ETH_DMAOMR_SR;
}

/**
  * @brief  Clears the ETHERNET transmit FIFO.
  * @param  heth pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
static void ETH_FlushTransmitFIFO(ETH_HandleTypeDef *heth)
{
  /* Set the Flush Transmit FIFO bit */
  (heth->Instance)->DMAOMR |= ETH_DMAOMR_FTF;
#if 1
  /*##RM0090 says to wait until the bit we set clears itself,
   * nothing like the original code here */
  do {
    ETH_whileWaiting();//this probably takes more than 8 clocks when doing nothing.
  }
  while ((heth->Instance)->DMAOMR & ETH_DMAOMR_FTF);
#else
  /* Wait until the write operation will be taken into account:
     at least four TX_CLK/RX_CLK clock cycles */
  __IO uint32_t tmpreg1 = 0U;

  tmpreg1 = (heth->Instance)->DMAOMR;
  ETH_Delay(ETH_REG_WRITE_DELAY);
  (heth->Instance)->DMAOMR = tmpreg1;
#endif
}

/* all uses of the following were suspect.
  The only value passed in was 1 and 1 millisecond is much slower than the "4 tx/rx/clocks"
  that was commented at all points of use.
  One of the places was ignoring what it was supposed to wait for,
  the other 4 uses were in essence the same and the RM0090 manual does not suggest that the waiting is required.
  Even if the comments are correct, waiting a millisecond when 40 or 400 nanoseconds are what are described was excessive,
  and blocking other application execution for that long is unpleasant.

  AND THEN there were direct uses of HAL_Delay which should have been this guy!
  I know this as they were in duplicate code blocks.
*/
///**
//  * @brief  This function provides delay (in milliseconds) based on CPU cycles method.
//  * @param  mdelay specifies the delay time length, in milliseconds.
//  * @retval None
//  *
//  * the apparent use of this function is to wait for a millisecond after certain operations, but without the expense of
//  * synchronizing with the system timer tick.
//  * with a touch of complexity that can be more reliably done via polling the systick count register.
//  */
//void ETH_Delay(uint32_t mdelay)
//{
//  //note: this presumes the __NOP and --Delay loop takes 8 clocks.
//  __IO uint32_t Delay = mdelay * (SystemCoreClock / 8U / 1000U);
//  do
//  {
////    __NOP();
//    ETH_whileWaiting();//this probably takes more than 8 clocks when doing nothing.
//  }
//  while (Delay --);
//}


/**
  * @brief  This function times out spinning on a phy register for a given pattern
  * @param phyreg number of register of interest
  * @param ones are the OR of bits that you are waiting to be ones
  * @param zeroes are the OR of bits that you are waiting to be zero, NOT the complement of those bits.
  * @param mdelay specifies the delay time length, in milliseconds.
  * @param heth pointer to struct for accessing phy and noting failures.
  * @retval HAL_TIMEOUT or HAL_OK
  */
static HAL_StatusTypeDef ETH_waitOnPhy(uint16_t phyreg,uint16_t ones, uint16_t zeroes,unsigned mdelay,ETH_HandleTypeDef *heth){
  unsigned tickstart = HAL_GetTick();
  uint32_t value;
  do
  {
    HAL_ETH_ReadPHYRegister(heth, PHY_BSR, &value);
    if ((value & ones) == ones && (value & zeroes) == 0) {
      return HAL_OK;
    }
    ETH_whileWaiting();
  } while((HAL_GetTick() - tickstart )<mdelay);
  //the code I cadged this from set State to READY!
  return HAL_TIMEOUT;
}
/**
  * @brief  This function times out spinning on a mac register for a given pattern
  * @param macregister pointer to register of interest
  * @param ones are the OR of bits that you are waiting to be ones
  * @param zeroes are the OR of bits that you are waiting to be zero, NOT the complement of those bits.
  * @param mdelay specifies the delay time length, in milliseconds.
  * @param heth pointer to struct which has its State and lock updated on timeout.
  * @retval HAL_TIMEOUT or HAL_OK
  */

static HAL_StatusTypeDef ETH_waitOnMac(volatile uint32_t *macregister,uint32_t ones, uint32_t zeroes,unsigned mdelay,ETH_HandleTypeDef *heth){
  unsigned tickstart = HAL_GetTick();

  do
  {
    uint32_t value = *macregister;
    if (((value & ones) == ones) && ((value & zeroes) == 0)) {
      return HAL_OK;
    }
    ETH_whileWaiting();
  } while((HAL_GetTick() - tickstart )<mdelay);

  heth->State= HAL_ETH_STATE_TIMEOUT;

  /* Process Unlocked */
  __HAL_UNLOCK(heth);

  return HAL_TIMEOUT;
}




#if (USE_HAL_ETH_REGISTER_CALLBACKS == 1)
static void ETH_InitCallbacksToDefault(ETH_HandleTypeDef *heth)
{
  /* Init the ETH Callback settings */
  heth->TxCpltCallback       = HAL_ETH_TxCpltCallback; /* Legacy weak TxCpltCallback   */
  heth->RxCpltCallback       = HAL_ETH_RxCpltCallback; /* Legacy weak RxCpltCallback   */
  heth->DMAErrorCallback     = HAL_ETH_ErrorCallback;  /* Legacy weak DMAErrorCallback */
}
#endif /* USE_HAL_ETH_REGISTER_CALLBACKS */

/**
  * @}
  */

#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F427xx ||\
          STM32F437xx || STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx */
#endif /* HAL_ETH_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
