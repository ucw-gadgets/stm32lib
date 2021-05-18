/*
 *	Interface to DS18B20 Temperature Sensors
 *
 *	(c) 2019 Martin Mareš <mj@ucw.cz>
 */

#include "util.h"
#include "ds18b20.h"
#include "ext-timer.h"

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <string.h>

/*** Configuration ***/

// You should set the following parameters in config.h

// #define DS_TIMER TIM3
// #define DS_GPIO GPIOA
// #define DS_PIN GPIO7
// #define DS_DMA DMA1
// #define DS_DMA_CH 6

// #undef DS_DEBUG
// #undef DS_DEBUG2

// Maximum number of supported sensors
// #define DS_NUM_SENSORS 8

#ifdef DS_DEBUG
#define DEBUG debug_printf
#else
#define DEBUG(xxx, ...) do { } while (0)
#endif

#ifdef DS_DEBUG2
#define DEBUG2 debug_printf
#else
#define DEBUG2(xxx, ...) do { } while (0)
#endif

static volatile u32 ds_dma_buffer;

static bool ds_reset(void)
{
	DEBUG2("DS18B20: Reset\n");
	timer_disable_counter(DS_TIMER);
	timer_one_shot_mode(DS_TIMER);

	// DMA for reading pin state
	ds_dma_buffer = 0xdeadbeef;
	dma_set_memory_address(DS_DMA, DS_DMA_CH, (u32) &ds_dma_buffer);
	dma_set_peripheral_address(DS_DMA, DS_DMA_CH, (u32) &GPIO_IDR(DS_GPIO));
	dma_set_number_of_data(DS_DMA, DS_DMA_CH, 1);
	dma_enable_channel(DS_DMA, DS_DMA_CH);

	// CC1 is used to drive the DMA (read line state at specified time)
	timer_disable_oc_output(DS_TIMER, TIM_OC1);
	timer_set_oc_mode(DS_TIMER, TIM_OC1, TIM_OCM_FROZEN);
	timer_set_oc_value(DS_TIMER, TIM_OC1, 560);
	timer_set_dma_on_compare_event(DS_TIMER);
	timer_enable_dma_cc1(DS_TIMER);

	// CC2 is used to generate pulses (return line to idle state at specified time)
	timer_set_oc_mode(DS_TIMER, TIM_OC2, TIM_OCM_FORCE_HIGH);
	timer_enable_oc_output(DS_TIMER, TIM_OC2);
	timer_set_oc_value(DS_TIMER, TIM_OC2, 480);
	timer_set_oc_polarity_low(DS_TIMER, TIM_OC2);

	// Set timer period to the length of the whole transaction (1 ms)
	timer_set_period(DS_TIMER, 999);

	// XXX: We do not know why this is needed...
	static bool once;
	if (!once) {
		for (int i=0; i<10000; i++) __asm__ volatile ("nop");
		once = 1;
	}

	// Pull line down and start timer
	timer_generate_event(DS_TIMER, TIM_EGR_UG);
	timer_set_oc_mode(DS_TIMER, TIM_OC2, TIM_OCM_INACTIVE);
	timer_enable_counter(DS_TIMER);

	// Wait until the timer expires
	while (timer_is_counter_enabled(DS_TIMER))
		;
	// Counter is automatically disabled at the end of cycle

	// Disable DMA
	timer_disable_dma_cc1(DS_TIMER);
	dma_disable_channel(DS_DMA, DS_DMA_CH);

	DEBUG2("Init DMA: %08x [%u] (%u remains)\n",
	       ds_dma_buffer,
	       !!(ds_dma_buffer & DS_PIN),
	       dma_get_number_of_data(DS_DMA, DS_DMA_CH));

	// Did the device respond?
	if (ds_dma_buffer & DS_PIN) {
		DEBUG("DS18B20: Initialization failed\n");
		return 0;
	} else
		return 1;
}

static void ds_send_bit(bool bit)
{
	timer_set_period(DS_TIMER, 99);				// Each write slot takes 100 μs
	timer_set_oc_mode(DS_TIMER, TIM_OC2, TIM_OCM_FORCE_HIGH);
	timer_set_oc_value(DS_TIMER, TIM_OC2, (bit ? 3 : 89));	// 1: 3μs pulse, 0: 89μs pulse
	timer_generate_event(DS_TIMER, TIM_EGR_UG);
	timer_set_oc_mode(DS_TIMER, TIM_OC2, TIM_OCM_INACTIVE);
	timer_enable_counter(DS_TIMER);
	while (timer_is_counter_enabled(DS_TIMER))
		;
}

static void ds_send_byte(byte b)
{
	DEBUG2("DS write: %02x\n", b);
	for (uint m = 1; m < 0x100; m <<= 1)
		ds_send_bit(b & m);
}

static bool ds_recv_bit(void)
{
	timer_set_period(DS_TIMER, 79);			// Each read slot takes 80μs
	timer_set_oc_value(DS_TIMER, TIM_OC2, 2);	// Generate 2μs pulse to start read slot
	timer_set_oc_value(DS_TIMER, TIM_OC1, 8);	// Sample data 8μs after start of slot
	timer_enable_dma_cc1(DS_TIMER);

	ds_dma_buffer = 0xdeadbeef;
	dma_set_number_of_data(DS_DMA, DS_DMA_CH, 1);
	dma_enable_channel(DS_DMA, DS_DMA_CH);
	timer_set_oc_mode(DS_TIMER, TIM_OC2, TIM_OCM_FORCE_HIGH);
	timer_generate_event(DS_TIMER, TIM_EGR_UG);
	timer_set_oc_mode(DS_TIMER, TIM_OC2, TIM_OCM_INACTIVE);
	timer_enable_counter(DS_TIMER);
	while (timer_is_counter_enabled(DS_TIMER))
		;
	// DEBUG2("XXX %08x\n", ds_dma_buffer);
	bool out = ds_dma_buffer & DS_PIN;
	dma_disable_channel(DS_DMA, DS_DMA_CH);

	timer_disable_dma_cc1(DS_TIMER);

	return out;
}

static byte ds_recv_byte(void)
{
	uint out = 0;
	for (uint m = 1; m < 0x100; m <<= 1) {
		if (ds_recv_bit())
			out |= m;
	}

	DEBUG2("DS read: %02x\n", out);
	return out;
}

static byte ds_buf[10];

static byte ds_crc_block(uint n)
{
	/// XXX: This might be worth optimizing
	uint crc = 0;

	for (uint i = 0; i < n; i++) {
		byte b = ds_buf[i];
		for (uint j = 0; j < 8; j++) {
			uint k = (b & 1) ^ (crc >> 7);
			crc = (crc << 1) & 0xff;
			if (k)
				crc ^= 0x31;
			b >>= 1;
		}
	}

	return crc;
}

static bool ds_recv_block(uint n)
{
	for (uint i = 0; i < n; i++)
		ds_buf[i] = ds_recv_byte();

	byte crc = ds_crc_block(n);
	if (crc) {
		DEBUG("DS18B20: Invalid CRC %02x\n", crc);
		return 0;
	}
	return 1;
}

struct ds_sensor ds_sensors[DS_NUM_SENSORS];

#if DS_NUM_SENSORS == 1

static void ds_enumerate(void)
{
	if (!ds_reset())
		return;

	ds_send_byte(0x33);	// READ_ROM
	if (!ds_recv_block(8))
		return;

	DEBUG("DS18B20: Found sensor ");
	for (uint i = 0; i < 8; i++) {
		DEBUG("%02x", ds_buf[i]);
		ds_sensors[0].address[i] = ds_buf[i];
	}
	DEBUG("\n");
}

#else

static void ds_enumerate(void)
{
	/*
	 *  The enumeration algorithm roughly follows the one described in the
	 *  Book of iButton Standards (Maxim Integrated Application Note 937).
	 *
	 *  It simulates depth-first search on the trie of all device IDs.
	 *  In each pass, it walks the trie from the root and recognizes branching nodes.
	 *
	 *  The old_choice variable remembers the deepest left branch taken in the
	 *  previous pass, new_choice is the same for the current pass.
	 */

	DEBUG("DS18B20: Enumerate\n");

	uint num_sensors = 0;
	byte *addr = ds_buf;
	byte old_choice = 0;

	for (;;) {
		if (!ds_reset()) {
			DEBUG("DS18B20: Enumeration found no sensor\n");
			return;
		}

		ds_send_byte(0xf0);	// SEARCH_ROM
		byte new_choice = 0;
		for (byte i=0; i<64; i++) {
			bool have_one = ds_recv_bit();
			bool have_zero = ds_recv_bit();
			bool old_bit = addr[i/8] & (1U << (i%8));
			bool new_bit;
			switch (2*have_one + have_zero) {
				case 3:
					// This should not happen
					DEBUG("DS18B20: Enumeration failed\n");
					return;
				case 1:
					// Only 0
					new_bit = 0;
					break;
				case 2:
					// Only 1
					new_bit = 1;
					break;
				default:
					// Both
					if (i == old_choice)
						new_bit = 1;
					else if (i > old_choice) {
						new_bit = 0;
						new_choice = i;
					} else {
						new_bit = old_bit;
						if (!new_bit)
							new_choice = i;
					}
			}
			if (new_bit)
				addr[i/8] |= 1U << (i%8);
			else
				addr[i/8] &= ~(1U << (i%8));
			ds_send_bit(new_bit);
		}

		if (num_sensors >= DS_NUM_SENSORS) {
			DEBUG("DS18B20: Too many sensors\n");
			return;
		}

		DEBUG("DS18B20: Found sensor #%u: ", num_sensors);
		for (byte i=0; i<8; i++)
			DEBUG("%02x", addr[i]);
		if (ds_crc_block(8)) {
			DEBUG(" - invalid CRC!\n");
		} else if (ds_buf[0] == 0x28) {
			DEBUG("\n");
			memcpy(ds_sensors[num_sensors].address, ds_buf, 8);
			num_sensors++;
		} else {
			DEBUG(" - wrong type\n");
		}

		old_choice = new_choice;
		if (!old_choice)
			break;
	}
}

#endif

void ds_init(void)
{
	DEBUG("DS18B20: Init\n");

	for (uint i = 0; i < DS_NUM_SENSORS; i++) {
		memset(ds_sensors[i].address, 0, 8);
		ds_sensors[i].current_temp = DS_TEMP_UNKNOWN;
	}

	dma_set_read_from_peripheral(DS_DMA, DS_DMA_CH);
	dma_set_priority(DS_DMA, DS_DMA_CH, DMA_CCR_PL_VERY_HIGH);
	dma_disable_peripheral_increment_mode(DS_DMA, DS_DMA_CH);
	dma_enable_memory_increment_mode(DS_DMA, DS_DMA_CH);
	dma_set_peripheral_size(DS_DMA, DS_DMA_CH, DMA_CCR_PSIZE_16BIT);
	dma_set_memory_size(DS_DMA, DS_DMA_CH, DMA_CCR_MSIZE_16BIT);

	timer_set_prescaler(DS_TIMER, CPU_CLOCK_MHZ - 1);	// 1 tick = 1 μs
	timer_set_mode(DS_TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_disable_preload(DS_TIMER);

	gpio_set_mode(DS_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, DS_PIN);

	ds_enumerate();

	// FIXME: Configure precision?
}

#if DS_NUM_SENSORS == 1
#define ds_current_id 0
#else
	static byte ds_current_id;
#endif

static bool ds_activate(void)
{
	if (!ds_reset()) {
		DEBUG("DS18B20: Reset failed\n");
		return false;
	}
#if DS_NUM_SENSORS == 1
	ds_send_byte(0xcc);	// SKIP_ROM
#else
	ds_send_byte(0x55);	// MATCH_ROM
	for (uint i = 0; i < 8; i++)
		ds_send_byte(ds_sensors[ds_current_id].address[i]);
#endif
	return true;
}

void ds_step(void)
{
	static byte ds_running;
	static byte ds_timeout;

	if (!ds_running) {
		// Start measurement
#if DS_NUM_SENSORS != 1
		uint maxn = DS_NUM_SENSORS;
		do {
			if (!maxn--)
				return;
			ds_current_id++;
			if (ds_current_id >= DS_NUM_SENSORS) {
				ds_current_id = 0;
			}
		} while (!ds_sensors[ds_current_id].address[0]);
#endif
		if (!ds_activate()) {
			ds_sensors[ds_current_id].current_temp = DS_TEMP_UNKNOWN;
			return;
		}
		ds_send_byte(0x44);	// CONVERT_T
		ds_running = 1;
		ds_timeout = 255;
	} else {
		// Still running?
		if (!ds_recv_bit()) {
			if (!ds_timeout--) {
				DEBUG("DS18B20 #%u: Timeout\n", ds_current_id);
				ds_sensors[ds_current_id].current_temp = DS_TEMP_UNKNOWN;
				ds_running = 0;
			}
			return;
		}
		ds_running = 0;

		// Read scratch pad
		if (!ds_activate())
			return;
		ds_send_byte(0xbe);	// READ_SCRATCHPAD
		if (!ds_recv_block(9)) {
			ds_sensors[ds_current_id].current_temp = DS_TEMP_UNKNOWN;
			return;
		}
		int t = (int16_t) (ds_buf[0] | (ds_buf[1] << 8));
		t = t * 1000 / 16;

		DEBUG("DS18B20 #%u: %d.%03d degC\n", ds_current_id, t / 1000, t % 1000);
		ds_sensors[ds_current_id].current_temp = t;
	}
}
