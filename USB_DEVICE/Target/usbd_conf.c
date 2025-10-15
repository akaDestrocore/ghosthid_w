/*
  Zephyr-aware usbd_conf.c
  Adapted from Cube-generated code to avoid symbol collisions under Zephyr.

  Behavior:
   - When built outside Zephyr: behaves like original Cube file (MSP init, NVIC, HAL_PCD_RegisterCallback, etc).
   - When built inside Zephyr (__ZEPHYR__ defined): we DON'T define HAL_PCD_* callback implementations or do MSP GPIO/NVIC init.
     We still expose the USBD_LL_* functions and link pdev <-> hpcd so ST middleware can call USBD_LL_Transmit / etc.
*/

#include "usbd_def.h"
#include "usbd_core.h"
#include "usbd_customhid.h"

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

/* If building under Zephyr, pull in necessary Zephyr headers */
#if defined(__ZEPHYR__)
#include <zephyr/kernel.h>
#endif

/* prototypes from other TUs */
void Error_Handler(void);
void SystemClock_Config(void);

/* Private --------------------------------------------------------- */
PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Forward: convert HAL status to USBD status */
USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status);

/* ------------------------------------------------------------------ */
/* LL Driver Callbacks (PCD -> USB Device Library)
 *
 * IMPORTANT: When building in Zephyr we MUST NOT define HAL_PCD_* callbacks
 * that Zephyr's usb_dc_stm32 driver already defines. If you define them
 * twice the linker fails. Therefore all HAL_PCD_* and HAL_PCD_MspInit
 * definitions are guarded out in Zephyr builds.
 */

/* MSP Init / DeInit - Cube normally configures GPIOA PA11/PA12 and NVIC.
 * Under Zephyr we avoid touching those hardware specifics.
 */
#if !defined(__ZEPHYR__)
void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (pcdHandle->Instance == USB_OTG_FS)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12; /* DM / DP */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
  }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef* pcdHandle)
{
  if (pcdHandle->Instance == USB_OTG_FS)
  {
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
  }
}

/* All the HAL_PCD_* callbacks below are Cube-style wrappers that call into
 * the USB Device Library. They must NOT be present when Zephyr builds its
 * own usb_dc_stm32 driver which already implements these callbacks.
 */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
#endif
{
  USBD_LL_SetupStage((USBD_HandleTypeDef*)hpcd->pData, (uint8_t *)hpcd->Setup);
}

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#else
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#endif
{
  USBD_LL_DataOutStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#else
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#endif
{
  USBD_LL_DataInStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
#endif
{
  USBD_LL_SOF((USBD_HandleTypeDef*)hpcd->pData);
}

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
#endif
{
  USBD_SpeedTypeDef speed = USBD_SPEED_FULL;

  if (hpcd->Init.speed == PCD_SPEED_HIGH) {
    speed = USBD_SPEED_HIGH;
  } else if (hpcd->Init.speed == PCD_SPEED_FULL) {
    speed = USBD_SPEED_FULL;
  } else {
    Error_Handler();
  }

  USBD_LL_SetSpeed((USBD_HandleTypeDef*)hpcd->pData, speed);
  USBD_LL_Reset((USBD_HandleTypeDef*)hpcd->pData);
}

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
#endif
{
  USBD_LL_Suspend((USBD_HandleTypeDef*)hpcd->pData);
  __HAL_PCD_GATE_PHYCLOCK(hpcd);
  if (hpcd->Init.low_power_enable) {
    SCB->SCR |= (uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk);
  }
}

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
#endif
{
  USBD_LL_Resume((USBD_HandleTypeDef*)hpcd->pData);
}

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#else
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#endif
{
  USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum);
}

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#else
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#endif
{
  USBD_LL_IsoINIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum);
}

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
#endif
{
  USBD_LL_DevConnected((USBD_HandleTypeDef*)hpcd->pData);
}

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
#endif
{
  USBD_LL_DevDisconnected((USBD_HandleTypeDef*)hpcd->pData);
}

#endif /* !__ZEPHYR__ */

/* ------------------------------------------------------------------ */
/* LL Driver Interface (USB Device Library --> PCD)
 *
 * We must expose USBD_LL_Init, USBD_LL_DeInit, USBD_LL_Start, etc.
 * Under Zephyr: do minimal linking to hpcd_USB_OTG_FS and DO NOT re-configure GPIO/NVIC.
 */

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
  /* If the device instance is full-speed (FS) */
  if (pdev->id == DEVICE_FS) {
    /* Link pdev and hpcd. */
    hpcd_USB_OTG_FS.pData = pdev;
    pdev->pData = &hpcd_USB_OTG_FS;

#if !defined(__ZEPHYR__)
    /* Non-Zephyr: initialize the PCD with Cube HAL. This includes configuring
     * FIFOs and calling HAL_PCD_Init().
     */
    hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
    hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
    hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
    hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
    hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;

    if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
      Error_Handler();
    }

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    /* Register callbacks */
    HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_SOF_CB_ID, PCD_SOFCallback);
    HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_SETUPSTAGE_CB_ID, PCD_SetupStageCallback);
    HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_RESET_CB_ID, PCD_ResetCallback);
    HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_SUSPEND_CB_ID, PCD_SuspendCallback);
    HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_RESUME_CB_ID, PCD_ResumeCallback);
    HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_CONNECT_CB_ID, PCD_ConnectCallback);
    HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_DISCONNECT_CB_ID, PCD_DisconnectCallback);

    HAL_PCD_RegisterDataOutStageCallback(&hpcd_USB_OTG_FS, PCD_DataOutStageCallback);
    HAL_PCD_RegisterDataInStageCallback(&hpcd_USB_OTG_FS, PCD_DataInStageCallback);
    HAL_PCD_RegisterIsoOutIncpltCallback(&hpcd_USB_OTG_FS, PCD_ISOOUTIncompleteCallback);
    HAL_PCD_RegisterIsoInIncpltCallback(&hpcd_USB_OTG_FS, PCD_ISOINIncompleteCallback);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    /* Set FIFOs (Cube example) */
    HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x40);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 2, 0x40);
#else
    /* Zephyr: assume usb_dc_stm32 already did low-level init.
     * We only need to ensure pdev <-> hpcd link is set (done).
     * Do not call HAL_PCD_Init, do not change NVIC or GPIO.
     */
    (void)hpcd_USB_OTG_FS; /* keep unused-warning quiet in some builds */
#endif
  }

  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  /* call HAL PCD DeInit if present (non-Zephyr) */
#if !defined(__ZEPHYR__)
  hal_status = HAL_PCD_DeInit(pdev->pData);
  usb_status = USBD_Get_USB_Status(hal_status);
#else
  /* Zephyr: don't deinit the PCD here; Zephyr's framework owns it */
  usb_status = USBD_OK;
#endif

  return usb_status;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
#if !defined(__ZEPHYR__)
  HAL_StatusTypeDef hal_status = HAL_PCD_Start(pdev->pData);
  return USBD_Get_USB_Status(hal_status);
#else
  /* Zephyr: starting/stopping device handled by Zephyr driver; assume ok. */
  return USBD_OK;
#endif
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
#if !defined(__ZEPHYR__)
  HAL_StatusTypeDef hal_status = HAL_PCD_Stop(pdev->pData);
  return USBD_Get_USB_Status(hal_status);
#else
  return USBD_OK;
#endif
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                 uint8_t ep_type, uint16_t ep_mps)
{
#if !defined(__ZEPHYR__)
  HAL_StatusTypeDef hal_status = HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);
  return USBD_Get_USB_Status(hal_status);
#else
  /* Zephyr: still call HAL_PCD_EP_Open - symbol comes from Cube HAL included by zephyr HAL module */
  HAL_StatusTypeDef hal_status = HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);
  return USBD_Get_USB_Status(hal_status);
#endif
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef hal_status = HAL_PCD_EP_Close(pdev->pData, ep_addr);
  return USBD_Get_USB_Status(hal_status);
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef hal_status = HAL_PCD_EP_Flush(pdev->pData, ep_addr);
  return USBD_Get_USB_Status(hal_status);
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef hal_status = HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
  return USBD_Get_USB_Status(hal_status);
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef hal_status = HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
  return USBD_Get_USB_Status(hal_status);
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*) pdev->pData;
  if ((ep_addr & 0x80) == 0x80) {
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
  } else {
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
  }
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
  HAL_StatusTypeDef hal_status = HAL_PCD_SetAddress(pdev->pData, dev_addr);
  return USBD_Get_USB_Status(hal_status);
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                    uint8_t *pbuf, uint32_t size)
{
  HAL_StatusTypeDef hal_status = HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
  return USBD_Get_USB_Status(hal_status);
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                          uint8_t *pbuf, uint32_t size)
{
  HAL_StatusTypeDef hal_status = HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
  return USBD_Get_USB_Status(hal_status);
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef*) pdev->pData, ep_addr);
}

/* static allocation used by middleware (one instance) */
void *USBD_static_malloc(uint32_t size)
{
  static uint32_t mem[(sizeof(USBD_CUSTOM_HID_HandleTypeDef) / 4 + 1)];
  (void)size;
  return mem;
}

void USBD_static_free(void *p)
{
  (void)p;
}

/* Delay */
void USBD_LL_Delay(uint32_t Delay)
{
#if defined(__ZEPHYR__)
  k_msleep(Delay);
#else
  HAL_Delay(Delay);
#endif
}

/* HAL -> USB status mapping */
USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status)
{
  USBD_StatusTypeDef usb_status = USBD_OK;
  switch (hal_status) {
    case HAL_OK:
      usb_status = USBD_OK; break;
    case HAL_ERROR:
      usb_status = USBD_FAIL; break;
    case HAL_BUSY:
      usb_status = USBD_BUSY; break;
    case HAL_TIMEOUT:
      usb_status = USBD_FAIL; break;
    default:
      usb_status = USBD_FAIL; break;
  }
  return usb_status;
}
