#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define CUSTOM_HID_EPIN_SIZE        0x08U
#define CUSTOM_HID_EPOUT_SIZE       0x04U

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define user_btn_Pin GPIO_PIN_0
#define user_btn_GPIO_Port GPIOA

#define CH375A_INT_PIN GPIO_PIN_14
#define CH375A_INT_PORT GPIOE
#define CH375B_INT_PIN GPIO_PIN_15
#define CH375B_INT_PORT GPIOE

#define green_led_Pin GPIO_PIN_12
#define green_led_GPIO_Port GPIOD
#define orange_led_Pin GPIO_PIN_13
#define orange_led_GPIO_Port GPIOD
#define red_led_Pin GPIO_PIN_14
#define red_led_GPIO_Port GPIOD
#define blue_led_Pin GPIO_PIN_15
#define blue_led_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
// CH375 CONFIG
#define CH375_WORK_BAUDRATE     115200
#define CH375_DEFAULT_BAUDRATE  9600
#define CH375_MODULE_NUM        2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */