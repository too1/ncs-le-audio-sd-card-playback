/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/debug/stack.h>
#include <zephyr/device.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/settings/settings.h>

#include "macros_common.h"
#include "fw_info_app.h"
#include "led.h"
#include "button_handler.h"
#include "button_assignments.h"
#include "nrfx_clock.h"
#include "ble_core.h"
#include "power_module.h"
#include "sd_card.h"
#include "board_version.h"
#include "audio_system.h"
#include "channel_assignment.h"
#include "streamctrl.h"
#include "audio_i2s.h"
#include "hw_codec.h"
#include <zephyr/sys/ring_buffer.h>

#if defined(CONFIG_AUDIO_DFU_ENABLE)
#include "dfu_entry.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

#if defined(CONFIG_INIT_STACKS)
/* Used for printing stack usage */
extern struct k_thread z_main_thread;
#endif /* defined(CONFIG_INIT_STACKS) */

static atomic_t ble_core_is_ready = (atomic_t) false;
static struct board_version board_rev;

static int hfclock_config_and_start(void)
{
	int ret;

	/* Use this to turn on 128 MHz clock for cpu_app */
	ret = nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);

	ret -= NRFX_ERROR_BASE_NUM;
	if (ret) {
		return ret;
	}

	nrfx_clock_hfclk_start();
	while (!nrfx_clock_hfclk_is_running()) {
	}

	return 0;
}

static int leds_set(void)
{
	int ret;

	/* Blink LED 3 to indicate that APP core is running */
	ret = led_blink(LED_APP_3_GREEN);
	if (ret) {
		return ret;
	}

#if (CONFIG_AUDIO_DEV == HEADSET)
	enum audio_channel channel;

	channel_assignment_get(&channel);

	if (channel == AUDIO_CH_L) {
		ret = led_on(LED_APP_RGB, LED_COLOR_BLUE);
	} else {
		ret = led_on(LED_APP_RGB, LED_COLOR_MAGENTA);
	}

	if (ret) {
		return ret;
	}
#elif (CONFIG_AUDIO_DEV == GATEWAY)
	ret = led_on(LED_APP_RGB, LED_COLOR_GREEN);
	if (ret) {
		return ret;
	}
#endif /* (CONFIG_AUDIO_DEV == HEADSET) */

	return 0;
}

static int bonding_clear_check(void)
{
	int ret;
	bool pressed;

	ret = button_pressed(BUTTON_5, &pressed);
	if (ret) {
		return ret;
	}

	if (pressed) {
		if (IS_ENABLED(CONFIG_SETTINGS)) {
			LOG_INF("Clearing all bonds");
			bt_unpair(BT_ID_DEFAULT, NULL);
		}
	}
	return 0;
}

static int channel_assign_check(void)
{
#if (CONFIG_AUDIO_DEV == HEADSET) && CONFIG_AUDIO_HEADSET_CHANNEL_RUNTIME
	int ret;
	bool pressed;

	ret = button_pressed(BUTTON_VOLUME_DOWN, &pressed);
	if (ret) {
		return ret;
	}

	if (pressed) {
		channel_assignment_set(AUDIO_CH_L);
		return 0;
	}

	ret = button_pressed(BUTTON_VOLUME_UP, &pressed);
	if (ret) {
		return ret;
	}

	if (pressed) {
		channel_assignment_set(AUDIO_CH_R);
		return 0;
	}
#endif

	return 0;
}

/* Callback from ble_core when the ble subsystem is ready */
void on_ble_core_ready(void)
{
	int ret;

	(void)atomic_set(&ble_core_is_ready, (atomic_t) true);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();

		ret = bonding_clear_check();
		ERR_CHK(ret);
	}
}

#define I2S_16BIT_SAMPLE_NUM (I2S_SAMPLES_NUM*2)
#define I2S_BUF_BYTES		 (I2S_16BIT_SAMPLE_NUM * 2)

static uint16_t m_i2s_tx_buf_a[I2S_16BIT_SAMPLE_NUM], m_i2s_rx_buf_a[I2S_16BIT_SAMPLE_NUM], m_i2s_tx_buf_b[I2S_16BIT_SAMPLE_NUM], m_i2s_rx_buf_b[I2S_16BIT_SAMPLE_NUM];

K_SEM_DEFINE(m_sem_load_from_sd, 0, 1);

#define SOUND_BUF_SIZE (I2S_16BIT_SAMPLE_NUM * 2 * 4)
#define SD_CARD_TRANSFER_SIZE (SOUND_BUF_SIZE / 2)
RING_BUF_DECLARE(m_ringbuf_sound_data, SOUND_BUF_SIZE);

void i2s_callback(uint32_t frame_start_ts, uint32_t *rx_buf_released, uint32_t const *tx_buf_released)
{
	// Update the I2S buffers by reading from the ringbuffer
	if((uint16_t *)tx_buf_released == m_i2s_tx_buf_a) {
		ring_buf_get(&m_ringbuf_sound_data, (uint8_t *)m_i2s_tx_buf_a, I2S_BUF_BYTES);
		audio_i2s_set_next_buf((const uint8_t *)m_i2s_tx_buf_a, (uint32_t *)m_i2s_rx_buf_a);
	} else if((uint16_t *)tx_buf_released == m_i2s_tx_buf_b) {
		ring_buf_get(&m_ringbuf_sound_data, (uint8_t *)m_i2s_tx_buf_b, I2S_BUF_BYTES);
		audio_i2s_set_next_buf((const uint8_t *)m_i2s_tx_buf_b, (uint32_t *)m_i2s_rx_buf_b);
	} else {
		printk("Should not happen! 0x%x\n", (int)tx_buf_released);
		return;
	}

	// Check the current free space in the buffer. 
	// If more than half the buffer is free we should move more data from the SD card
	if(ring_buf_space_get(&m_ringbuf_sound_data) > SD_CARD_TRANSFER_SIZE) {
		k_sem_give(&m_sem_load_from_sd);
	}
}

size_t sd_card_to_buffer(int numbytes)
{
	uint8_t *buf_ptr;
	size_t sd_read_length = numbytes;

	// Claim a buffer from the ringbuffer. This allows us to read the file data directly into 
	// the buffer without requiring a memcpy
	sd_read_length = ring_buf_put_claim(&m_ringbuf_sound_data, &buf_ptr, sd_read_length);

	// For simplicity, assume the claim was successful (the flow of the program ensures this)
	// Read the data from the file and move it into the ringbuffer
	sd_card_segment_read(buf_ptr, &sd_read_length);

	// Finish the claim, allowing the data to be read from the buffer in the I2S interrupt
	ring_buf_put_finish(&m_ringbuf_sound_data, sd_read_length);

	// Return the actual read length. When we reach end of file this will be lower than numbytes
	return sd_read_length;
}

int play_file_from_sd(const char *filename)
{
	printk("Opening file %s\n", filename);
	int ret = sd_card_segment_read_open(filename);
	if(ret < 0) return ret;

	// Start by filling the entire ringbuffer
	sd_card_to_buffer(SOUND_BUF_SIZE);

	// Start I2S transmission by setting up both buffersets
	ring_buf_get(&m_ringbuf_sound_data, (uint8_t *)m_i2s_tx_buf_a, I2S_16BIT_SAMPLE_NUM * 2);
	audio_i2s_start((const uint8_t *)m_i2s_tx_buf_a, (uint32_t *)m_i2s_rx_buf_a);
	ring_buf_get(&m_ringbuf_sound_data, (uint8_t *)m_i2s_tx_buf_b, I2S_16BIT_SAMPLE_NUM * 2);
	audio_i2s_set_next_buf((const uint8_t *)m_i2s_tx_buf_b, (uint32_t *)m_i2s_rx_buf_b);	

	while(1) {
		// Wait for the load from SD semaphore to be set, signalling that the ringbuffer is half empty
		k_sem_take(&m_sem_load_from_sd, K_FOREVER);
		
		// Move data from the SD card to the buffer, one half at the time
		if (sd_card_to_buffer(SD_CARD_TRANSFER_SIZE) < SD_CARD_TRANSFER_SIZE) {
			// If the function returns less bytes than requested we have reached the end of the file. 
			// Exit the while loop and stop the I2S driver
			break;
		}
	}

	printk("Stopping I2S\n");

	audio_i2s_stop();

	printk("Closing file\n");

	return sd_card_segment_read_close();
}

void main(void)
{
	int ret;

	LOG_DBG("nRF5340 APP core started");

	ret = hfclock_config_and_start();
	ERR_CHK(ret);

	ret = led_init();
	ERR_CHK(ret);

	ret = button_handler_init();
	ERR_CHK(ret);

#if 0
	channel_assignment_init();

	ret = channel_assign_check();
	ERR_CHK(ret);
#endif

	ret = fw_info_app_print();
	ERR_CHK(ret);

	ret = board_version_valid_check();
	ERR_CHK(ret);

	ret = board_version_get(&board_rev);
	ERR_CHK(ret);

	if (board_rev.mask & BOARD_VERSION_VALID_MSK_SD_CARD) {
		ret = sd_card_init();
		if (ret != -ENODEV) {
			ERR_CHK(ret);
		}
	}

	ret = power_module_init();
	ERR_CHK(ret);

#if defined(CONFIG_AUDIO_DFU_ENABLE)
	/* Check DFU BTN before Initialize BLE */
	dfu_entry_check();
#endif

#if 0
	/* Initialize BLE, with callback for when BLE is ready */
	ret = ble_core_init(on_ble_core_ready);
	ERR_CHK(ret);

	/* Wait until ble_core/NET core is ready */
	while (!(bool)atomic_get(&ble_core_is_ready)) {
		(void)k_sleep(K_MSEC(100));
	}
#endif

	ret = leds_set();
	ERR_CHK(ret);

	// List all the files on the card
	ret = sd_card_list_files("/");
	if(ret < 0) {
		LOG_ERR("Unable to read files from SD card (err %i)", ret);
	}

	// Attempt to read one file and print the content
	const int maxlength = 256;
	const uint8_t *test_file_name = "/test_1.txt";
	uint8_t file_data[maxlength+1];
	size_t length = maxlength;
	ret = sd_card_read(test_file_name, file_data, &length);
	if (ret == 0) {
		// Ensure the file_data is null terminated
		file_data[length] = 0;
		LOG_INF("Content of file %s: %s", test_file_name, file_data);
	}
	else {
		LOG_ERR("Test file not found (err %i)", ret);
	}

	// Add a delay to ensure all the logging output from the code above is printed before we continue
	k_msleep(2000);

	// Initialize the codec and I2S driver manually
	audio_i2s_blk_comp_cb_register(i2s_callback);
	audio_i2s_init();

	ret = hw_codec_init();
	ERR_CHK(ret);

	hw_codec_default_conf_enable();
	hw_codec_volume_set(100);

	LOG_INF("SD transfer size %i, total buffer size %i", SD_CARD_TRANSFER_SIZE, SOUND_BUF_SIZE);
	
	// Play test file from SD card
	play_file_from_sd("/Test88K.wav");

#if 0
	audio_system_init();

	ret = streamctrl_start();
	ERR_CHK(ret);

	while (1) {
		streamctrl_event_handler();
		STACK_USAGE_PRINT("main", &z_main_thread);
	}
#endif
}
