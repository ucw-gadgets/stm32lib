/*
 *	Generic DFU Bootloader
 *
 *	(c) 2020 Martin Mareš <mj@ucw.cz>
 *
 *	Based on example code from the libopencm3 project, which is
 *	Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 *	Licensed under the GNU LGPL v3 or any later version.
 */

#include "util.h"

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
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dfu.h>

#include <string.h>

#ifdef BOOTLOADER_DEBUG
#define DEBUG(x...) debug_printf(x)
#else
#define DEBUG(x...) do { } while (0)
#endif

// Offsets to firmware header fields (see tools/dfu-sign.c)
#define HDR_LENGTH 0x1c
#define HDR_FLASH_IN_PROGRESS 0x20

// DFU blocks should be equal to erase blocks of the flash
#define BLOCK_SIZE 1024
byte usbd_control_buffer[BLOCK_SIZE];

static enum dfu_state dfu_state = STATE_DFU_IDLE;
static uint timeout;
#define DEFAULT_TIMEOUT 5000   // ms
#define DFU_TIMEOUT 2000

static struct {
	byte buf[sizeof(usbd_control_buffer)];
	u16 blocknum;
	u16 len;
} prog;

static char usb_serial_number[13];

enum usb_string {
	STR_MANUFACTURER = 1,
	STR_PRODUCT,
	STR_SERIAL,
};

static const char *usb_strings[] = {
	BOOTLOADER_MFG_NAME,
	BOOTLOADER_PROD_NAME,
	usb_serial_number,
};

const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = BOOTLOADER_MFG_ID,
	.idProduct = BOOTLOADER_PROD_ID,
	.bcdDevice = BOOTLOADER_PROD_VERSION,
	.iManufacturer = STR_MANUFACTURER,
	.iProduct = STR_PRODUCT,
	.iSerialNumber = STR_SERIAL,
	.bNumConfigurations = 1,
};

const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = BLOCK_SIZE,
	.bcdDFUVersion = 0x0100,
};

const struct usb_interface_descriptor iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE, /* Device Firmware Upgrade */
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 2,
	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = USB_CONFIG_ATTR_DEFAULT,	// bus-powered
	.bMaxPower = 50,				// multiplied by 2 mA
	.interface = ifaces,
};

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

	// FIXME: Should check if len is reasonable

	crc_reset();
	u32 crc = crc_calculate_block((u32 *)BOOTLOADER_APP_START, len/4);
	u32 want_crc = get_u32(BOOTLOADER_APP_START + len);
	DEBUG("DFU: fip=%04x crc=%08x/%08x len=%u\n", (uint) flash_in_progress, (uint) crc, (uint) want_crc, (uint) len);
	if (flash_in_progress || crc != want_crc) {
		DEBUG("DFU: Bad firmware\n");
		return 0;
	}

	return 1;
}

static byte dfu_getstatus(usbd_device *usbd_dev UNUSED, u32 *bwPollTimeout)
{
	switch (dfu_state) {
	case STATE_DFU_DNLOAD_SYNC:
		dfu_state = STATE_DFU_DNBUSY;
		*bwPollTimeout = 100;
		return DFU_STATUS_OK;
	case STATE_DFU_MANIFEST_SYNC:
		/* Device will reset when read is complete. */
		dfu_state = STATE_DFU_MANIFEST;
		return DFU_STATUS_OK;
	case STATE_DFU_ERROR:
		return DFU_STATUS_ERR_VERIFY;
	default:
		return DFU_STATUS_OK;
	}
}

static void dfu_getstatus_complete(usbd_device *usbd_dev UNUSED, struct usb_setup_data *req UNUSED)
{
	switch (dfu_state) {
	case STATE_DFU_DNBUSY:
		if (prog.blocknum == 0) {
			// The "flash in progress" word is programmed as 0xffff first and reset later
			*(u16*)(prog.buf + HDR_FLASH_IN_PROGRESS) = 0xffff;
		}
		u32 baseaddr = BOOTLOADER_APP_START + prog.blocknum * BLOCK_SIZE;
		DEBUG("DFU: Block %u -> %08x + %u\n", prog.blocknum, (uint) baseaddr, prog.len);
		flash_unlock();
		flash_erase_page(baseaddr);
		for (uint i = 0; i < prog.len; i += 2)
			flash_program_half_word(baseaddr + i, *(u16*)(prog.buf + i));
		flash_lock();
		for (uint i = 0; i < prog.len; i++) {
			if (*(byte *)(baseaddr + i) != prog.buf[i]) {
				DEBUG("DFU: Verification failed\n");
				dfu_state = STATE_DFU_ERROR;
			}
		}
		dfu_state = STATE_DFU_DNLOAD_IDLE;
		return;
	case STATE_DFU_MANIFEST:
		// At the very end, re-flash the "flash in progress" word
		flash_unlock();
		flash_program_half_word(BOOTLOADER_APP_START + 0x20, 0);
		flash_lock();
		if (verify_firmware())
			dfu_state = STATE_DFU_MANIFEST_WAIT_RESET;
		else
			dfu_state = STATE_DFU_ERROR;
		return;
	default:
		return;
	}
}

static enum usbd_request_return_codes dfu_control_request(usbd_device *usbd_dev,
	struct usb_setup_data *req,
	byte **buf,
	u16 *len,
	void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	if ((req->bmRequestType & 0x7F) != 0x21)
		return USBD_REQ_NOTSUPP; /* Only accept class request. */

	DEBUG("DFU: Request %02x in state %d\n", req->bRequest, dfu_state);
	timeout = DFU_TIMEOUT;

	switch (req->bRequest) {
	case DFU_DNLOAD:
		if (len == NULL || *len == 0) {
			dfu_state = STATE_DFU_MANIFEST_SYNC;
		} else {
			/* Copy download data for use on GET_STATUS. */
			prog.blocknum = req->wValue;
			prog.len = *len;
			memcpy(prog.buf, *buf, *len);
			dfu_state = STATE_DFU_DNLOAD_SYNC;
		}
		return USBD_REQ_HANDLED;
	case DFU_CLRSTATUS:
		/* Clear error and return to dfuIDLE. */
		if (dfu_state == STATE_DFU_ERROR)
			dfu_state = STATE_DFU_IDLE;
		return USBD_REQ_HANDLED;
	case DFU_ABORT:
		/* Abort returns to dfuIDLE state. */
		dfu_state = STATE_DFU_IDLE;
		return USBD_REQ_HANDLED;
	case DFU_UPLOAD:
		/* Upload not supported for now. */
		return USBD_REQ_NOTSUPP;
	case DFU_GETSTATUS: {
		u32 bwPollTimeout = 0; /* 24-bit number of milliseconds */
		(*buf)[0] = dfu_getstatus(usbd_dev, &bwPollTimeout);
		(*buf)[1] = bwPollTimeout & 0xFF;
		(*buf)[2] = (bwPollTimeout >> 8) & 0xFF;
		(*buf)[3] = (bwPollTimeout >> 16) & 0xFF;
		(*buf)[4] = dfu_state;
		(*buf)[5] = 0; /* iString not used here */
		*len = 6;
		*complete = dfu_getstatus_complete;
		return USBD_REQ_HANDLED;
		}
	case DFU_GETSTATE:
		/* Return state with no state transition. */
		*buf[0] = dfu_state;
		*len = 1;
		return USBD_REQ_HANDLED;
	}

	return USBD_REQ_NOTSUPP;
}

static void dfu_set_config(usbd_device *usbd_dev, u16 wValue UNUSED)
{
	usbd_register_control_callback(
		usbd_dev,
		USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
		USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
		dfu_control_request);
}

static void dfu_reset(void)
{
	dfu_state = STATE_DFU_IDLE;
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
	rcc_periph_clock_enable(RCC_USB);
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

#ifndef BOOTLOADER_CUSTOM_HW_INIT

static void usb_disconnect(void)
{
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO11 | GPIO12);
	gpio_clear(GPIOA, GPIO11 | GPIO12);
	for (uint i=0; i<100; i++) {
		while (!systick_get_countflag())
			;
	}
}

#endif

int main(void)
{
	usbd_device *usbd_dev;

	reset_peripherals();

	// Flash programming requires running on the internal oscillator
	my_rcc_clock_setup_in_hsi_out_48mhz();

	configure_hardware();
	desig_get_unique_id_as_dfu(usb_serial_number);

	DEBUG("DFU: Started (SN %s)\n", usb_serial_number);

#ifdef BOOTLOADER_CUSTOM_HW_INIT
	custom_hw_init();
#else
	usb_disconnect();
#endif

	DEBUG("DFU: Ready\n");
	debug_led(0);

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, ARRAY_SIZE(usb_strings), usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_reset_callback(usbd_dev, dfu_reset);
	usbd_register_set_config_callback(usbd_dev, dfu_set_config);

restart: ;

	timeout = DEFAULT_TIMEOUT;
	while (timeout || (dfu_state != STATE_DFU_IDLE && dfu_state != STATE_DFU_MANIFEST_WAIT_RESET)) {
		usbd_poll(usbd_dev);
		if (timeout && systick_get_countflag()) {
			timeout--;
			// FIXME: Blink LED even after timeout
			if (!(timeout & 0x3f))
				debug_led_toggle();
		}
	}

	if (!verify_firmware())
		goto restart;

	u32 sp = get_u32(BOOTLOADER_APP_START);
	u32 pc = get_u32(BOOTLOADER_APP_START + 4);
	DEBUG("DFU: Boot (sp=%08x pc=%08x)\n", (uint) sp, (uint) pc);

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
