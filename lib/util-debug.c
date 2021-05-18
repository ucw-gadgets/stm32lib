/*
 *	Debugging Utilities for STM32
 *
 *	(c) 2018--2019 Martin Mareš <mj@ucw.cz>
 */

#include "util.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include <stdarg.h>
#include <string.h>

/*** Configuration ***/

// You should set the following parameters in config.h

// Use the semi-hosting interface for debugging messages
// #define DEBUG_SEMIHOSTING

// Use this USART for debugging messages
// #define DEBUG_USART USART1

// Use this LED for debugging
#ifdef DEBUG_LED_BLUEPILL
#define DEBUG_LED_GPIO GPIOC
#define DEBUG_LED_PIN GPIO13
#define DEBUG_LED_INVERSE
#endif

/*** Implementation ***/

#ifdef DEBUG_SEMIHOSTING

void semi_put_char(char c)
{
	// This is tricky, we need to work around GCC bugs
	volatile char cc = c;
	asm volatile (
		"mov r0, #0x03\n"   /* SYS_WRITEC */
		"mov r1, %[msg]\n"
		"bkpt #0xAB\n"
		:
		: [msg] "r" (&cc)
		: "r0", "r1"
	);
}

void semi_write_string(char *c)
{
	asm volatile (
		"mov r0, #0x04\n"   /* SYS_WRITE0 */
		"mov r1, %[msg]\n"
		"bkpt #0xAB\n"
		:
		: [msg] "r" (c)
		: "r0", "r1"
	);
}

#endif

void debug_putc(int c)
{
#ifdef DEBUG_SEMIHOSTING
	static char debug_buf[128];
	static int debug_i;
	debug_buf[debug_i++] = c;
	if (c == '\n' || debug_i >= sizeof(debug_buf) - 1) {
		debug_buf[debug_i] = 0;
		semi_write_string(debug_buf);
		debug_i = 0;
	}
#endif
#ifdef DEBUG_USART
	if (c == '\n')
		usart_send_blocking(DEBUG_USART, '\r');
	usart_send_blocking(DEBUG_USART, c);
#endif
}

void debug_flush(void)
{
#ifdef DEBUG_USART
	while (!usart_get_flag(DEBUG_USART, USART_FLAG_TC))
		;
#endif
}

void debug_puts(const char *s)
{
	while (*s)
		debug_putc(*s++);
}

enum printf_flags {
	PF_ZERO_PAD = 1,
	PF_SIGNED = 2,
	PF_NEGATIVE = 4,
	PF_UPPERCASE = 8,
	PF_LEFT = 16,
};

static void printf_string(const char *s, uint width, uint flags)
{
	uint len = strlen(s);
	uint pad = (len < width) ? width - len : 0;
	char pad_char = (flags & PF_ZERO_PAD) ? '0' : ' ';

	if (flags & PF_LEFT)
		debug_puts(s);
	while (pad--)
		debug_putc(pad_char);
	if (!(flags & PF_LEFT))
		debug_puts(s);
}

static void printf_number(uint i, uint width, uint flags, uint base)
{
	char buf[16];
	char *w = buf + sizeof(buf);

	if (flags & PF_SIGNED) {
		if ((int) i < 0) {
			i = - (int) i;
			flags |= PF_NEGATIVE;
		}
	}

	*--w = 0;
	do {
		uint digit = i % base;
		if (digit < 10)
			*--w = '0' + digit;
		else
			*--w = ((flags & PF_UPPERCASE) ? 'A' : 'a') + digit - 10;
		i /= base;
	}
	while (i);

	if (flags & PF_NEGATIVE)
		*--w = '-';

	printf_string(w, width, flags);
}

void debug_printf(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	while (*fmt) {
		int c = *fmt++;
		if (c != '%') {
			debug_putc(c);
			continue;
		}

		uint width = 0;
		uint flags = 0;

		if (*fmt == '-') {
			fmt++;
			flags |= PF_LEFT;
		}

		if (*fmt == '0') {
			fmt++;
			flags |= PF_ZERO_PAD;
		}

		while (*fmt >= '0' && *fmt <= '9')
			width = 10*width + *fmt++ - '0';

		c = *fmt++;
		switch (c) {
			case 'c':
				debug_putc(va_arg(args, int));
				break;
			case 'd':
				printf_number(va_arg(args, int), width, flags | PF_SIGNED, 10);
				break;
			case 'u':
				printf_number(va_arg(args, int), width, flags, 10);
				break;
			case 'X':
				flags |= PF_UPPERCASE;
				// fall-thru
			case 'x':
				printf_number(va_arg(args, int), width, flags, 16);
				break;
			case 's':
				printf_string(va_arg(args, char *), width, flags);
				break;
			default:
				debug_putc(c);
				continue;
		}
	}

	va_end(args);
}

void debug_led(bool light)
{
#ifdef DEBUG_LED_GPIO
#ifdef DEBUG_LED_INVERSE
	light = !light;
#endif
	if (light)
		gpio_set(DEBUG_LED_GPIO, DEBUG_LED_PIN);
	else
		gpio_clear(DEBUG_LED_GPIO, DEBUG_LED_PIN);
#endif
}

void debug_led_toggle(void)
{
#ifdef DEBUG_LED_GPIO
	gpio_toggle(DEBUG_LED_GPIO, DEBUG_LED_PIN);
#endif
}
