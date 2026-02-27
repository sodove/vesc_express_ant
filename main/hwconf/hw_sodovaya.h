/*
	Custom HW config for sodovaya's VESC Express + ANT BMS bridge.
	Based on hw_xp_t (VESC Express T) — same pinout, custom name.
*/

#ifndef MAIN_HWCONF_HW_SODOVAYA_H_
#define MAIN_HWCONF_HW_SODOVAYA_H_

#include "driver/gpio.h"

#define HW_NAME						"sodovaya express"
#define HW_TARGET					"esp32c3"

// Override default BLE name
#ifndef CONF_BLE_NAME
#define CONF_BLE_NAME				"sodovaya express"
#endif

#define HW_INIT_HOOK()				hw_init()

// LEDs (same as Express T)
#define LED_RED_PIN					2
#define LED_BLUE_PIN				3

#define LED_RED_ON()				gpio_set_level(LED_RED_PIN, 1)
#define LED_RED_OFF()				gpio_set_level(LED_RED_PIN, 0)

#define LED_BLUE_ON()				gpio_set_level(LED_BLUE_PIN, 1)
#define LED_BLUE_OFF()				gpio_set_level(LED_BLUE_PIN, 0)

// CAN
#define CAN_TX_GPIO_NUM				1
#define CAN_RX_GPIO_NUM				0

// SD-card
#define SD_PIN_MOSI					4
#define SD_PIN_MISO					6
#define SD_PIN_SCK					5
#define SD_PIN_CS					7

// UART
#define UART_NUM					0
#define UART_BAUDRATE				115200
#define UART_TX						21
#define UART_RX						20

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_HW_SODOVAYA_H_ */
