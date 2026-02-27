/*
	Custom HW config for sodovaya's VESC Express + ANT BMS bridge.
	Based on hw_xp_t (VESC Express T) — same pinout, custom name.
*/

#include "hw_sodovaya.h"

void hw_init(void) {
	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = ((1ULL << LED_RED_PIN) | (1ULL << LED_BLUE_PIN));
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	LED_RED_OFF();
	LED_BLUE_OFF();
}
