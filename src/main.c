/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Sample app for Audio class
 */

#include <zephyr.h>
#include <logging/log.h>
#include <nrfx_clock.h>
#include "audio.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);


// Set the clock speed of the chip
// param: hfclk128 divider, 0 for 128MHz, 1 for 64MHz (default)
//        hfclk192 divider, 0 for 192MHz, 1 for 96MHz, 2 for 48MHz,	3 for 24MHz, 4 for 12MHz (default)
void set_clock_settings(uint8_t hfclk128_div, uint8_t hfclk192_div)
{
    if (hfclk128_div > 2) {
        LOG_ERR("Invalid hfclk128 divider");
        return;
    }
    if (hfclk192_div > 4) {
        LOG_ERR("Invalid hfclk192 divider");
        return;
    }
    nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, hfclk128_div);
    nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK192M, hfclk192_div);
}

void main(void)
{
	printk("build time: " __DATE__ " " __TIME__ "\n");

	audio_init();

	while (1) {
		LOG_INF(".");
		k_sleep(K_SECONDS(1));
	}
}
