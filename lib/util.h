/*
 *	General Utility Library for STM32
 *
 *	(c) 2018--2019 Martin Mareš <mj@ucw.cz>
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config.h"

// Types

typedef unsigned int uint;
typedef uint8_t byte;
typedef uint16_t u16;
typedef int16_t s16;
typedef uint32_t u32;
typedef int32_t s32;

// Macros

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))
#define CLAMP(x,min,max) ({ typeof(x) _t=x; (_t < min) ? min : (_t > max) ? max : _t; })
#define ARRAY_SIZE(ary) (sizeof(ary)/sizeof((ary)[0]))

#define UNUSED __attribute__((unused))

// Unaligned access to data

static inline uint get_u16_le(byte *p)
{
	return (p[1] << 8) | p[0];
}

static inline uint get_u16_be(byte *p)
{
	return (p[0] << 8) | p[1];
}

static inline uint get_u32_le(byte *p)
{
	return (p[3] << 24) | (p[2] << 16) | (p[1] << 8) | p[0];
}

static inline uint get_u32_be(byte *p)
{
	return (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
}

static inline void put_u16_le(byte *p, u16 x)
{
	p[0] = x;
	p[1] = x >> 8;
}

static inline void put_u16_be(byte *p, u16 x)
{
	p[0] = x >> 8;
	p[1] = x;
}

static inline void put_u32_be(byte *p, u32 x)
{
	p[0] = x >> 24;
	p[1] = (x >> 16) & 0xff;
	p[2] = (x >> 8) & 0xff;
	p[3] = x & 0xff;
}

static inline void put_u32_le(byte *p, u32 x)
{
	p[3] = x >> 24;
	p[2] = (x >> 16) & 0xff;
	p[1] = (x >> 8) & 0xff;
	p[0] = x & 0xff;
}

// CPU instructions not covered by libopencm3

static inline void wait_for_interrupt(void)
{
	asm volatile ("wfi");
}

// A compiler memory barrier

static inline void barrier(void)
{
	asm volatile ("" : : : "memory");
}

// util-debug.c

void debug_printf(const char *fmt, ...) __attribute__((format(printf,1,2)));
void debug_puts(const char *s);
void debug_putc(int c);
void debug_flush(void);

void debug_led(bool light);
void debug_led_toggle(void);
