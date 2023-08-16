/*
 *	MODBUS Bootloader
 *
 *	(c) 2023 Martin Mareš <mj@ucw.cz>
 *
 *	Based on example code from the libopencm3 project, which is
 *	Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 *	Licensed under the GNU LGPL v3 or any later version.
 */

#include "util.h"
#include "modbus.h"
#include "modbus-bootloader-proto.h"

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/desig.h>

#include <string.h>

#ifdef BOOTLOADER_DEBUG
#define DEBUG(x...) debug_printf(x)
#else
#define DEBUG(x...) do { } while (0)
#endif

// Offsets to firmware header fields (see tools/dfu-sign.c)
#define HDR_LENGTH 0x1c
#define HDR_FLASH_IN_PROGRESS 0x20

// Block size should be equal to erase block of the flash memory
#define BLOCK_SIZE 1024
static byte current_block[BLOCK_SIZE];
static uint current_block_number;

static uint timeout;
#define DEFAULT_TIMEOUT 50000000   // ms

static char usb_serial_number[13];
static uint flash_size_kb;
static uint status;

static inline u32 get_u32(u32 addr)
{
	return *(u32*)addr;
}

static inline u16 get_u16(u32 addr)
{
	return *(u16*)addr;
}

static bool verify_firmware(void)
{
	u32 len = get_u32(BOOTLOADER_APP_START + HDR_LENGTH);
	u16 flash_in_progress = get_u16(BOOTLOADER_APP_START + HDR_FLASH_IN_PROGRESS);

	// Just to be sure
	len = MIN(len, flash_size_kb * 1024);

	crc_reset();
	u32 crc = crc_calculate_block((u32 *)BOOTLOADER_APP_START, len/4);
	u32 want_crc = get_u32(BOOTLOADER_APP_START + len);
	DEBUG("BOOT: fip=%04x crc=%08x/%08x len=%u\n", (uint) flash_in_progress, (uint) crc, (uint) want_crc, (uint) len);
	if (flash_in_progress || crc != want_crc) {
		DEBUG("BOOT: Bad firmware\n");
		return 0;
	}

	return 1;
}

static bool flash_block(void)
{
	if (current_block_number >= flash_size_kb * 1024 / BLOCK_SIZE) {
		DEBUG("BOOT: Bad block nr\n");
		return false;
	}

	if (current_block_number == 0) {
		// The "flash in progress" word is programmed as 0xffff first and reset later
		*(u16*)(current_block + HDR_FLASH_IN_PROGRESS) = 0xffff;
	}

	u32 baseaddr = BOOTLOADER_APP_START + current_block_number * BLOCK_SIZE;
	DEBUG("BOOT: Block %u -> %08x\n", current_block_number, (uint) baseaddr);

	flash_unlock();
	flash_erase_page(baseaddr);
	for (uint i = 0; i < BLOCK_SIZE; i += 2)
		flash_program_half_word(baseaddr + i, *(u16*)(current_block + i));
	flash_lock();

	for (uint i = 0; i < BLOCK_SIZE; i++) {
		if (*(byte *)(baseaddr + i) != current_block[i]) {
			DEBUG("BOOT: Verification failed\n");
			return false;
		}
	}

	return true;
}

static bool flash_end(void)
{
	flash_unlock();
	flash_program_half_word(BOOTLOADER_APP_START + 0x20, 0);
	flash_lock();

	return verify_firmware();
}

/*
 *  This is a modified version of rcc_clock_setup_in_hsi_out_48mhz(),
 *  which properly turns off the PLL before setting its parameters.
 */
static void my_rcc_clock_setup_in_hsi_out_48mhz(void)
{
	/* Enable internal high-speed oscillator. */
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	/* Select HSI as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

	// XXX: Disable PLL
	rcc_osc_off(RCC_PLL);

	/*
	 * Set prescalers for AHB, ADC, ABP1, ABP2.
	 * Do this before touching the PLL (TODO: why?).
	 */
	rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);	/*Set.48MHz Max.72MHz */
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV8);	/*Set. 6MHz Max.14MHz */
	rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);	/*Set.24MHz Max.36MHz */
	rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);	/*Set.48MHz Max.72MHz */
	rcc_set_usbpre(RCC_CFGR_USBPRE_PLL_CLK_NODIV);  /*Set.48MHz Max.48MHz */

	/*
	 * Sysclk runs with 48MHz -> 1 waitstates.
	 * 0WS from 0-24MHz
	 * 1WS from 24-48MHz
	 * 2WS from 48-72MHz
	 */
	flash_set_ws(FLASH_ACR_LATENCY_1WS);

	/*
	 * Set the PLL multiplication factor to 12.
	 * 8MHz (internal) * 12 (multiplier) / 2 (PLLSRC_HSI_CLK_DIV2) = 48MHz
	 */
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL12);

	/* Select HSI/2 as PLL source. */
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSI_CLK_DIV2);

	/* Enable PLL oscillator and wait for it to stabilize. */
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);

	/* Select PLL as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

	/* Set the peripheral clock frequencies used */
	rcc_ahb_frequency = 48000000;
	rcc_apb1_frequency = 24000000;
	rcc_apb2_frequency = 48000000;
}

static void clock_plain_hsi(void)
{
	// Select HSI as SYSCLK source
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

	// Disable PLL
	rcc_osc_off(RCC_PLL);

	// Set prescalers for AHB, ADC, ABP1, ABP2, USB to defaults
	rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV2);
	rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_NODIV);
	rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);
	rcc_set_usbpre(RCC_CFGR_USBPRE_PLL_VCO_CLK_DIV3);
}

static void reset_peripherals(void)
{
	// Turn off clock to all peripherals and reset them
	RCC_AHBENR = 0x00000014;
	RCC_APB1ENR = 0;
	RCC_APB2ENR = 0;
	RCC_APB1RSTR = 0x22fec9ff;
	RCC_APB2RSTR = 0x0038fffd;
	RCC_APB1RSTR = 0;
	RCC_APB2RSTR = 0;
}

static void configure_hardware(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_CRC);

#ifdef DEBUG_USART
#if DEBUG_USART == USART1
	rcc_periph_clock_enable(RCC_USART1);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);
#elif DEBUG_USART == USART2
	rcc_periph_clock_enable(RCC_USART2);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO2);
#elif DEBUG_USART == USART3
	rcc_periph_clock_enable(RCC_USART3);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO10);
#else
#error "Unknown USART for debugging"
#endif
	usart_set_baudrate(DEBUG_USART, 115200);
	usart_set_databits(DEBUG_USART, 8);
	usart_set_stopbits(DEBUG_USART, USART_STOPBITS_1);
	usart_set_mode(DEBUG_USART, USART_MODE_TX);
	usart_set_parity(DEBUG_USART, USART_PARITY_NONE);
	usart_set_flow_control(DEBUG_USART, USART_FLOWCONTROL_NONE);
	usart_enable(DEBUG_USART);
#endif

#ifdef DEBUG_LED_BLUEPILL
	// BluePill LED
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	debug_led(1);
#endif

	// Systick: set to overflow in 1 ms, will use only the overflow flag, no interrupts
	systick_set_frequency(1000, CPU_CLOCK_MHZ * 1000000);
	systick_clear();
	systick_counter_enable();
}

/*** Modbus callbacks ***/

bool modbus_check_discrete_input(u16 addr UNUSED)
{
	return false;
}

bool modbus_get_discrete_input(u16 addr UNUSED)
{
	return false;
}

bool modbus_check_coil(u16 addr UNUSED)
{
	return false;
}

bool modbus_get_coil(u16 addr UNUSED)
{
	return false;
}

void modbus_set_coil(u16 addr UNUSED, bool value UNUSED)
{
}

bool modbus_check_input_register(u16 addr)
{
	return addr >= BL_INPUT_MAGIC_HI && addr < BL_INPUT_MAX;
}

u16 modbus_get_input_register(u16 addr)
{
	switch (addr) {
		case BL_INPUT_MAGIC_HI:
			return BL_MAGIC_HI;
		case BL_INPUT_MAGIC_LO:
			return BL_MAGIC_LO;
		case BL_INPUT_LOADER_VERSION:
			return BL_LOADER_VERSION;
		case BL_INPUT_STATUS:
			return status;
		case BL_INPUT_VENDOR_ID:
			return BOOTLOADER_VENDOR_ID;
		case BL_INPUT_DEVICE_ID:
			return BOOTLOADER_DEVICE_ID;
		case BL_INPUT_BLOCK_SIZE:
			return BLOCK_SIZE;
		case BL_INPUT_FLASH_SIZE:
			return flash_size_kb;
		default:
			return get_u16_le((byte *)usb_serial_number + 2*(addr - BL_INPUT_SERIAL_NUMBER));
	}
}

bool modbus_check_holding_register(u16 addr)
{
	return addr >= BL_HOLD_COMMAND && addr < BL_HOLD_MAX
	    || addr >= BL_HOLD_BLOCK_DATA && addr < BL_HOLD_BLOCK_DATA + BLOCK_SIZE / 2;
}

u16 modbus_get_holding_register(u16 addr UNUSED)
{
	// Reading of holding registers is not supported
	return 0;
}

static uint bl_command(uint value)
{
	switch (value) {
		case BL_COMMAND_BOOT:
			if (status == BL_STATUS_READY)
				return BL_STATUS_BOOTING;
			return BL_STATUS_ERROR;
		case BL_COMMAND_FLASH_START:
			return BL_STATUS_FLASHING;
		case BL_COMMAND_FLASH_END:
			if (flash_end())
				return BL_STATUS_READY;
			else
				return BL_STATUS_CORRUPTED;
		case BL_COMMAND_FLASH_BLOCK:
			if (flash_block())
				return BL_STATUS_FLASHING;
			else
				return BL_STATUS_ERROR;
		default:
			return BL_STATUS_ERROR;
	}
}

void modbus_set_holding_register(u16 addr, u16 value)
{
	switch (addr) {
		case BL_HOLD_COMMAND:
			status = bl_command(value);
			DEBUG("BOOT: cmd=%u status=%u\n", value, status);
			timeout = DEFAULT_TIMEOUT;
			break;
		case BL_HOLD_BLOCK_NUMBER:
			current_block_number = value;
			break;
		default:
			put_u16_le(current_block + 2*(addr - BL_HOLD_BLOCK_DATA), value);
	}
}

// These should be implemented by board-specific code
// void modbus_ready_hook(void);
// void modbus_frame_start_hook(void);
// const char * const modbus_id_strings[MODBUS_ID_MAX];

static void delay_ms(uint ms)
{
	for (uint j=0; j<ms; j++)
		while (!systick_get_countflag())
			;
}

int main(void)
{
	reset_peripherals();

	// Flash programming requires running on the internal oscillator
	my_rcc_clock_setup_in_hsi_out_48mhz();

	configure_hardware();
	custom_hw_init();
	desig_get_unique_id_as_dfu(usb_serial_number);
	flash_size_kb = desig_get_flash_size();

	// Allow ST-link to attach before we initialize the rest of hardware
	for (int i=0; i<100; i++) {
		debug_led_toggle();
		delay_ms(20);
	}

	DEBUG("BOOT: Started (SN %s, fs=%u)\n", usb_serial_number, flash_size_kb);
	modbus_init();

	DEBUG("BOOT: Ready\n");
	debug_led(0);

	if (verify_firmware())
		status = BL_STATUS_READY;
	else
		status = BL_STATUS_CORRUPTED;

	timeout = DEFAULT_TIMEOUT;
	byte led_counter = 0;
	while (status != BL_STATUS_READY || timeout) {
		modbus_loop();
		if (status == BL_STATUS_BOOTING && modbus_is_idle())
			break;
		if (systick_get_countflag()) {
			if (timeout)
				timeout--;
			if (!(led_counter++ & 0x3f))
				debug_led_toggle();
		}
	}

	u32 sp = get_u32(BOOTLOADER_APP_START);
	u32 pc = get_u32(BOOTLOADER_APP_START + 4);
	DEBUG("BOOT: Start (sp=%08x pc=%08x)\n", (uint) sp, (uint) pc);

#ifdef DEBUG_USART
	debug_flush();
#endif
	debug_led(0);

	reset_peripherals();
	clock_plain_hsi();

	/* Set vector table base address. */
	SCB_VTOR = BOOTLOADER_APP_START;

	/* Initialize master stack pointer. */
	asm volatile("msr msp, %0"::"g" (sp));

	/* Jump to application. */
	((void (*)(void)) pc)();
}
