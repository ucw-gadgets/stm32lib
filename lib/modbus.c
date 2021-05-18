/*
 *	Generic MODBUS Library for STM32
 *
 *	(c) 2019 Martin Mareš <mj@ucw.cz>
 */

#include "util.h"
#include "modbus.h"

#include <stddef.h>
#include <string.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>

/*** Configuration ***/

// You should set the following parameters in config.h

// USART (pins are expected to be configured by the caller)
// #define MODBUS_USART USART2
// #define MODBUS_NVIC_USART_IRQ NVIC_USART2_IRQ
// #define MODBUS_USART_ISR usart2_isr

// GPIO pin for transmitter enable (pins is expected to be configured by the caller)
// #define MODBUS_TXEN_GPIO_PORT GPIOA
// #define MODBUS_TXEN_GPIO_PIN GPIO1

// Timer
// #define MODBUS_TIMER TIM2
// #define MODBUS_NVIC_TIMER_IRQ NVIC_TIM2_IRQ
// #define MODBUS_TIMER_ISR tim2_isr

// Slave address we are responding at
// #define MODBUS_OUR_ADDRESS 42

// Baud rate
#ifndef MODBUS_BAUD_RATE
#define MODBUS_BAUD_RATE 19200
#endif

// CPU clock frequency
// #define CPU_CLOCK_MHZ 72

// Receive buffer size (standard specifies 256 bytes, you can make it shorter if necessary)
#ifndef MODBUS_RX_BUFSIZE
#define MODBUS_RX_BUFSIZE 256
#endif

// Transmit buffer size (standard specifies 256 bytes, you can make it shorter if necessary)
#ifndef MODBUS_TX_BUFSIZE
#define MODBUS_TX_BUFSIZE 256
#endif

// Receive timeout in microseconds
#ifndef MODBUS_RX_TIMEOUT
#if MODBUS_BAUD_RATE <= 19200
// For low baud rates, the standard specifies timeout of 1.5 character times
// (1 character = start bit + 8 data bits + parity bit + stop bit = 11 bits)
#define MODBUS_RX_TIMEOUT (1000000*11*3/2/MODBUS_BAUD_RATE)
#else
// For high rates, the timeout is fixed to 750 μs
#define MODBUS_RX_TIMEOUT 750
#endif
#endif

// Debugging
// #define MODBUS_DEBUG

#ifdef MODBUS_DEBUG
#define DEBUG debug_printf
#else
#define DEBUG(xxx, ...) do { } while (0)
#endif
/*** State ***/

enum mb_state {
	STATE_RX,
	STATE_RX_DONE,
	STATE_PROCESSING,
	STATE_TX,
	STATE_TX_LAST,
	STATE_TX_DONE,
};

static byte rx_buf[MODBUS_RX_BUFSIZE];
static u16 rx_size;
static byte rx_bad;
static byte state;		// STATE_xxx

static byte *rx_frame;
static byte *rx_frame_end;

static byte tx_buf[MODBUS_TX_BUFSIZE];
static u16 tx_size;
static u16 tx_pos;
static byte pending_error;

static bool check_frame(void);
static void process_frame(void);

/*** Low-level layer ***/

static void rx_init(void)
{
	state = STATE_RX;
	rx_size = 0;
	rx_bad = 0;
	usart_set_mode(MODBUS_USART, USART_MODE_RX);
	usart_enable_rx_interrupt(MODBUS_USART);
	modbus_ready_hook();
}

static void rx_done(void)
{
	state = STATE_RX_DONE;
	usart_disable_rx_interrupt(MODBUS_USART);
}

static void tx_init(void)
{
	state = STATE_TX;
	tx_pos = 0;
	gpio_set(MODBUS_TXEN_GPIO_PORT, MODBUS_TXEN_GPIO_PIN);
	usart_set_mode(MODBUS_USART, USART_MODE_TX);
	usart_enable_tx_interrupt(MODBUS_USART);
}

static void tx_done(void)
{
	state = STATE_TX_DONE;
	// usart_disable_tx_interrupt(MODBUS_USART);		// Already done by irq handler
	gpio_clear(MODBUS_TXEN_GPIO_PORT, MODBUS_TXEN_GPIO_PIN);
}

void modbus_init(void)
{
	DEBUG("MODBUS: Init\n");

	timer_set_prescaler(MODBUS_TIMER, CPU_CLOCK_MHZ-1);	// 1 tick = 1 μs
	timer_set_mode(MODBUS_TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_DOWN);
	timer_update_on_overflow(MODBUS_TIMER);
	timer_disable_preload(MODBUS_TIMER);
	timer_one_shot_mode(MODBUS_TIMER);
	timer_enable_irq(MODBUS_TIMER, TIM_DIER_UIE);
	nvic_enable_irq(MODBUS_NVIC_TIMER_IRQ);

	gpio_clear(MODBUS_TXEN_GPIO_PORT, MODBUS_TXEN_GPIO_PIN);

	usart_set_baudrate(MODBUS_USART, MODBUS_BAUD_RATE);
	usart_set_databits(MODBUS_USART, 9);
	usart_set_stopbits(MODBUS_USART, USART_STOPBITS_1);
	usart_set_parity(MODBUS_USART, USART_PARITY_EVEN);
	usart_set_flow_control(MODBUS_USART, USART_FLOWCONTROL_NONE);

	rx_init();

	nvic_enable_irq(MODBUS_NVIC_USART_IRQ);
	usart_enable(MODBUS_USART);
}

void MODBUS_USART_ISR(void)
{
	u32 status = USART_SR(MODBUS_USART);

	if (status & USART_SR_RXNE) {
		uint ch = usart_recv(MODBUS_USART);
		if (state == STATE_RX) {
			if (status & (USART_SR_FE | USART_SR_ORE | USART_SR_NE)) {
				rx_bad = 1;
			} else if (rx_size < MODBUS_RX_BUFSIZE) {
				if (!rx_size)
					modbus_frame_start_hook();
				rx_buf[rx_size++] = ch;
			} else {
				// Frame too long
				rx_bad = 2;
			}
			timer_set_period(MODBUS_TIMER, MODBUS_RX_TIMEOUT);
			timer_generate_event(MODBUS_TIMER, TIM_EGR_UG);
			timer_enable_counter(MODBUS_TIMER);
		}
	}

	if (state == STATE_TX) {
		if (status & USART_SR_TXE) {
			if (tx_pos < tx_size) {
				usart_send(MODBUS_USART, tx_buf[tx_pos++]);
			} else {
				// The transmitter is double-buffered, so at this moment, it is transmitting
				// the last byte of the frame. Wait until transfer is completed.
				usart_disable_tx_interrupt(MODBUS_USART);
				USART_CR1(MODBUS_USART) |= USART_CR1_TCIE;
				state = STATE_TX_LAST;
			}
		}
	} else if (state == STATE_TX_LAST) {
		if (status & USART_SR_TC) {
			// Transfer of the last byte is complete. Release the bus.
			USART_CR1(MODBUS_USART) &= ~USART_CR1_TCIE;
			tx_done();
			rx_init();
		}
	}
}

void MODBUS_TIMER_ISR(void)
{
	if (TIM_SR(MODBUS_TIMER) & TIM_SR_UIF) {
		TIM_SR(MODBUS_TIMER) &= ~TIM_SR_UIF;
		if (state == STATE_RX)
			rx_done();
	}
}

void modbus_loop(void)
{
	if (state != STATE_RX_DONE)
		return;
	state = STATE_PROCESSING;

	if (!check_frame()) {
		rx_init();
		return;
	}

	DEBUG("MODBUS: < dest=%02x func=%02x len=%u\n", rx_buf[0], rx_buf[1], rx_size);

	if (rx_buf[0] == MODBUS_OUR_ADDRESS) {
		// Frame addressed to us: process and reply
		process_frame();
		DEBUG("MODBUS: > status=%02x len=%u\n", tx_buf[1], tx_size);
		tx_init();
	} else if (rx_buf[0] == 0x00) {
		// Broadcast frame: process, but do not reply
		process_frame();
		rx_init();
	} else {
		// Somebody else's frame: discard
		rx_init();
	}
}

/** CRC ***/

static const byte crc_hi[] = {
	0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0,
	0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
	0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0,
	0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
	0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1,
	0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41,
	0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1,
	0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
	0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0,
	0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40,
	0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1,
	0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
	0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0,
	0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40,
	0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0,
	0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
	0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0,
	0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
	0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0,
	0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
	0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0,
	0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40,
	0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1,
	0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
	0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0,
	0x80, 0x41, 0x00, 0xc1, 0x81, 0x40
};

static const byte crc_lo[] = {
	0x00, 0xc0, 0xc1, 0x01, 0xc3, 0x03, 0x02, 0xc2, 0xc6, 0x06,
	0x07, 0xc7, 0x05, 0xc5, 0xc4, 0x04, 0xcc, 0x0c, 0x0d, 0xcd,
	0x0f, 0xcf, 0xce, 0x0e, 0x0a, 0xca, 0xcb, 0x0b, 0xc9, 0x09,
	0x08, 0xc8, 0xd8, 0x18, 0x19, 0xd9, 0x1b, 0xdb, 0xda, 0x1a,
	0x1e, 0xde, 0xdf, 0x1f, 0xdd, 0x1d, 0x1c, 0xdc, 0x14, 0xd4,
	0xd5, 0x15, 0xd7, 0x17, 0x16, 0xd6, 0xd2, 0x12, 0x13, 0xd3,
	0x11, 0xd1, 0xd0, 0x10, 0xf0, 0x30, 0x31, 0xf1, 0x33, 0xf3,
	0xf2, 0x32, 0x36, 0xf6, 0xf7, 0x37, 0xf5, 0x35, 0x34, 0xf4,
	0x3c, 0xfc, 0xfd, 0x3d, 0xff, 0x3f, 0x3e, 0xfe, 0xfa, 0x3a,
	0x3b, 0xfb, 0x39, 0xf9, 0xf8, 0x38, 0x28, 0xe8, 0xe9, 0x29,
	0xeb, 0x2b, 0x2a, 0xea, 0xee, 0x2e, 0x2f, 0xef, 0x2d, 0xed,
	0xec, 0x2c, 0xe4, 0x24, 0x25, 0xe5, 0x27, 0xe7, 0xe6, 0x26,
	0x22, 0xe2, 0xe3, 0x23, 0xe1, 0x21, 0x20, 0xe0, 0xa0, 0x60,
	0x61, 0xa1, 0x63, 0xa3, 0xa2, 0x62, 0x66, 0xa6, 0xa7, 0x67,
	0xa5, 0x65, 0x64, 0xa4, 0x6c, 0xac, 0xad, 0x6d, 0xaf, 0x6f,
	0x6e, 0xae, 0xaa, 0x6a, 0x6b, 0xab, 0x69, 0xa9, 0xa8, 0x68,
	0x78, 0xb8, 0xb9, 0x79, 0xbb, 0x7b, 0x7a, 0xba, 0xbe, 0x7e,
	0x7f, 0xbf, 0x7d, 0xbd, 0xbc, 0x7c, 0xb4, 0x74, 0x75, 0xb5,
	0x77, 0xb7, 0xb6, 0x76, 0x72, 0xb2, 0xb3, 0x73, 0xb1, 0x71,
	0x70, 0xb0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
	0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9c, 0x5c,
	0x5d, 0x9d, 0x5f, 0x9f, 0x9e, 0x5e, 0x5a, 0x9a, 0x9b, 0x5b,
	0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4b, 0x8b,
	0x8a, 0x4a, 0x4e, 0x8e, 0x8f, 0x4f, 0x8d, 0x4d, 0x4c, 0x8c,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
	0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

static u16 crc16(byte *buf, u16 len)
{
	byte hi = 0xff, lo = 0xff;

	while (len--) {
		byte i = hi ^ *buf++;
		hi = lo ^ crc_hi[i];
		lo = crc_lo[i];
	}

	return (hi << 8 | lo);
}

/*** High-level layer ***/

static bool check_frame(void)
{
	if (rx_bad) {
		// FIXME: Error counters?
		DEBUG("MODBUS: RX bad\n");
		return false;
	}
	
	if (rx_size < 4) {
		// FIXME: Error counters?
		DEBUG("MODBUS: RX undersize\n");
		return false;
	}

	u16 crc = crc16(rx_buf, rx_size - 2);
	u16 rx_crc = (rx_buf[rx_size-2] << 8) | rx_buf[rx_size-1];
	if (crc != rx_crc) {
		// FIXME: Error counters?
		DEBUG("MODBUS: Bad CRC\n");
		return false;
	}

	rx_frame = rx_buf + 1;
	rx_frame_end = rx_frame + rx_size - 2;
	return true;
}

enum mb_function {
	FUNC_READ_COILS = 0x01,
	FUNC_READ_DISCRETE_INPUTS = 0x02,
	FUNC_READ_HOLDING_REGISTERS = 0x03,
	FUNC_READ_INPUT_REGISTERS = 0x04,
	FUNC_WRITE_SINGLE_COIL = 0x05,
	FUNC_WRITE_SINGLE_REGISTER = 0x06,
	FUNC_READ_EXCEPTION_STATUS = 0x07,
	FUNC_DIAGNOSTICS = 0x08,
	FUNC_GET_COMM_EVENT_COUNTER = 0x0b,
	FUNC_GET_COMM_EVENT_LOG = 0x0c,
	FUNC_WRITE_MULTIPLE_COILS = 0x0f,
	FUNC_WRITE_MULTIPLE_REGISTERS = 0x10,
	FUNC_REPORT_SLAVE_ID = 0x11,
	FUNC_READ_FILE_RECORD = 0x14,
	FUNC_WRITE_FILE_RECORD = 0x15,
	FUNC_MASK_WRITE_REGISTER = 0x16,
	FUNC_READ_WRITE_MULTIPLE_REGISTERS = 0x17,
	FUNC_READ_FIFO_QUEUE = 0x18,
	FUNC_ENCAPSULATED_INTERFACE_TRANSPORT = 0x2b,
};

enum mb_error {
	ERR_ILLEGAL_FUNCTION = 0x01,
	ERR_ILLEGAL_DATA_ADDRESS = 0x02,
	ERR_ILLEGAL_DATA_VALUE = 0x03,
	ERR_SLAVE_DEVICE_FAILURE = 0x04,
};

static uint read_remains(void)
{
	return rx_frame_end - rx_frame;
}

static byte read_byte(void)
{
	return *rx_frame++;
}

static u16 read_u16(void)
{
	byte hi = *rx_frame++;
	byte lo = *rx_frame++;
	return (hi << 8) | lo;
}

static void write_byte(byte v)
{
	tx_buf[tx_size++] = v;
}

static void write_u16(u16 v)
{
	write_byte(v >> 8);
	write_byte(v);
}

static bool body_fits(uint body_len)
{
	// body_len excludes slave address, function code, and CRC
	return (2 + body_len + 2 <= MODBUS_TX_BUFSIZE);
}

static void report_error(byte code)
{
	// Discard the partially constructed body of the reply and rewrite the header
	tx_buf[1] |= 0x80;
	tx_buf[2] = code;
	tx_size = 3;
}

static void func_read_bits(bool coils)
{
	if (read_remains() < 4)
		return report_error(ERR_ILLEGAL_DATA_VALUE);

	u16 start = read_u16();
	u16 count = read_u16();

	uint bytes = (count+7) / 8;
	if (!body_fits(1 + bytes))
		return report_error(ERR_ILLEGAL_DATA_VALUE);

	for (u16 i = 0; i < count; i++)
		if (!(coils ? modbus_check_coil : modbus_check_discrete_input)(start + i))
			return report_error(ERR_ILLEGAL_DATA_ADDRESS);

	write_byte(bytes);
	for (u16 i = 0; i < bytes; i++) {
		byte b = 0;
		for (byte j = 0; j < 8 && 8*i + j < count; j++) {
			uint addr = start + 8*i + j;
			if ((coils ? modbus_get_coil : modbus_get_discrete_input)(addr))
				b |= 1 << j;
		}
		write_byte(b);
	}
}

static void func_read_registers(byte holding)
{
	if (read_remains() < 4)
		return report_error(ERR_ILLEGAL_DATA_VALUE);

	u16 start = read_u16();
	u16 count = read_u16();

	uint bytes = 2*count;
	if (!body_fits(1 + bytes))
		return report_error(ERR_ILLEGAL_DATA_VALUE);

	for (u16 i = 0; i < count; i++)
		if (!(holding ? modbus_check_holding_register : modbus_check_input_register)(start + i))
			return report_error(ERR_ILLEGAL_DATA_ADDRESS);

	// FIXME: Reporting of slave failures?
	write_byte(bytes);
	for (u16 i = 0; i < count; i++)
		write_u16((holding ? modbus_get_holding_register : modbus_get_input_register)(start + i));
}

static void func_write_single_coil(void)
{
	if (read_remains() < 4)
		return report_error(ERR_ILLEGAL_DATA_VALUE);

	u16 addr = read_u16();
	u16 value = read_u16();

	if (!modbus_check_coil(addr))
		return report_error(ERR_ILLEGAL_DATA_ADDRESS);
	if (value != 0x0000 && value != 0xff00)
		return report_error(ERR_ILLEGAL_DATA_VALUE);

	modbus_set_coil(addr, value);

	write_u16(addr);
	write_u16(value);
}

static void func_write_single_register(void)
{
	if (read_remains() < 4)
		return report_error(ERR_ILLEGAL_DATA_VALUE);

	u16 addr = read_u16();
	u16 value = read_u16();

	if (!modbus_check_holding_register(addr))
		return report_error(ERR_ILLEGAL_DATA_ADDRESS);

	modbus_set_holding_register(addr, value);

	write_u16(addr);
	write_u16(value);
}

static void func_write_multiple_coils(void)
{
	if (read_remains() < 5)
		return report_error(ERR_ILLEGAL_DATA_VALUE);

	u16 start = read_u16();
	u16 count = read_u16();
	byte bytes = read_byte();

	if (read_remains() < bytes || bytes != (count+7) / 8)
		return report_error(ERR_ILLEGAL_DATA_VALUE);

	for (u16 i = 0; i < count; i++)
		if (!modbus_check_coil(start + i))
			return report_error(ERR_ILLEGAL_DATA_ADDRESS);

	for (u16 i = 0; i < count; i++)
		modbus_set_coil(start + i, rx_frame[i/8] & (1U << (i%8)));

	write_u16(start);
	write_u16(count);
}

static void func_write_multiple_registers(void)
{
	if (read_remains() < 5)
		return report_error(ERR_ILLEGAL_DATA_VALUE);

	u16 start = read_u16();
	u16 count = read_u16();
	byte bytes = read_byte();

	if (read_remains() < bytes || bytes != 2*count)
		return report_error(ERR_ILLEGAL_DATA_VALUE);

	for (u16 i = 0; i < count; i++)
		if (!modbus_check_holding_register(start + i))
			return report_error(ERR_ILLEGAL_DATA_ADDRESS);

	for (u16 i = 0; i < count; i++)
		modbus_set_holding_register(start + i, read_u16());

	write_u16(start);
	write_u16(count);
}

static void func_mask_write_register(void)
{
	if (read_remains() < 6)
		return report_error(ERR_ILLEGAL_DATA_VALUE);

	u16 addr = read_u16();
	u16 and_mask = read_u16();
	u16 or_mask = read_u16();

	if (!modbus_check_holding_register(addr))
		return report_error(ERR_ILLEGAL_DATA_ADDRESS);

	u16 reg = modbus_get_holding_register(addr);
	reg = (reg & and_mask) | (or_mask & ~and_mask);
	modbus_set_holding_register(addr, reg);

	write_u16(addr);
	write_u16(and_mask);
	write_u16(or_mask);
}

static void func_read_write_multiple_registers(void)
{
	if (read_remains() < 9)
		return report_error(ERR_ILLEGAL_DATA_VALUE);

	u16 read_start = read_u16();
	u16 read_count = read_u16();
	u16 write_start = read_u16();
	u16 write_count = read_u16();
	byte write_bytes = read_byte();

	if (read_remains() < write_bytes || write_bytes != 2*write_count)
		return report_error(ERR_ILLEGAL_DATA_VALUE);

	for (u16 i = 0; i < read_count; i++)
		if (!modbus_check_holding_register(read_start + i))
			return report_error(ERR_ILLEGAL_DATA_ADDRESS);

	for (u16 i = 0; i < write_count; i++)
		if (!modbus_check_holding_register(write_start + i))
			return report_error(ERR_ILLEGAL_DATA_ADDRESS);

	byte read_bytes = 2*write_count;
	if (!body_fits(1 + read_bytes))
		return report_error(ERR_ILLEGAL_DATA_VALUE);

	for (u16 i = 0; i < write_count; i++)
		modbus_set_holding_register(write_start + i, read_u16());

	write_byte(read_bytes);
	for (u16 i = 0; i < read_count; i++)
		modbus_get_holding_register(read_start + i);
}

static void func_encapsulated_interface_transport(void)
{
	if (read_remains() < 3 ||
	    read_byte() != 0x0e)
		return report_error(ERR_ILLEGAL_DATA_VALUE);

	byte action = read_byte();
	byte id = read_byte();

	byte range_min, range_max;
	switch (action) {
		case 1:
			// Streaming access to basic identification
			range_min = MODBUS_ID_VENDOR_NAME;
			range_max = MODBUS_ID_MAJOR_MINOR_REVISION;
			break;
		case 2:
			// Streaming access to regular identification
			range_min = MODBUS_ID_VENDOR_URL;
			range_max = MODBUS_ID_USER_APP_NAME;
			break;
		case 4:
			// Individual access
			if (id >= MODBUS_ID_MAX || !modbus_id_strings[id])
				return report_error(ERR_ILLEGAL_DATA_ADDRESS);
			range_min = range_max = id;
			break;
		default:
			return report_error(ERR_ILLEGAL_DATA_VALUE);
	}

	if (action != 4) {
		if (id < range_min || id > range_max)
			id = range_min;
	}

	write_byte(0x0e);	// Repeat a part of the request
	write_byte(action);

	// Conformity level
	if (modbus_id_strings[MODBUS_ID_VENDOR_URL] ||
	    modbus_id_strings[MODBUS_ID_PRODUCT_NAME] ||
	    modbus_id_strings[MODBUS_ID_USER_APP_NAME])
		write_byte(0x82);	// Regular identification, both stream and individual access supported
	else
		write_byte(0x81);	// Basic identification only

	u16 more_follows_at = tx_size;
	write_byte(0);		// More follows: so far not
	write_byte(0);		// Next object ID: so far none
	write_byte(0);		// Number of objects

	for (id = range_min; id <= range_max; id++) {
		if (modbus_id_strings[id]) {
			byte len = strlen(modbus_id_strings[id]);
			byte remains = MODBUS_TX_BUFSIZE - 4 - tx_size;	// 2 for CRC, 2 for object header
			if (len > remains) {
				// If it is the only object, cut it
				if (!tx_buf[more_follows_at + 2])
					len = remains;
				else {
					// More follows, report the next ID
					tx_buf[more_follows_at] = 0xff;
					tx_buf[more_follows_at + 1] = id;
					break;
				}
			}
			tx_buf[more_follows_at + 2] ++;
			write_byte(id);
			write_byte(len);
			memcpy(tx_buf + tx_size, modbus_id_strings[id], len);
			tx_size += len;
		}
	}
}

static void process_frame(void)
{
	byte func = read_byte();

	// Prepare reply frame
	tx_buf[0] = MODBUS_OUR_ADDRESS;
	tx_buf[1] = rx_buf[1];
	tx_size = 2;
	pending_error = 0;

	switch (func) {
		case FUNC_READ_COILS:
			func_read_bits(true);
			break;
		case FUNC_READ_DISCRETE_INPUTS:
			func_read_bits(false);
			break;
		case FUNC_READ_HOLDING_REGISTERS:
			func_read_registers(true);
			break;
		case FUNC_READ_INPUT_REGISTERS:
			func_read_registers(false);
			break;
		case FUNC_WRITE_SINGLE_COIL:
			func_write_single_coil();
			break;
		case FUNC_WRITE_SINGLE_REGISTER:
			func_write_single_register();
			break;
		case FUNC_WRITE_MULTIPLE_COILS:
			func_write_multiple_coils();
			break;
		case FUNC_WRITE_MULTIPLE_REGISTERS:
			func_write_multiple_registers();
			break;
		case FUNC_MASK_WRITE_REGISTER:
			func_mask_write_register();
			break;
		case FUNC_READ_WRITE_MULTIPLE_REGISTERS:
			func_read_write_multiple_registers();
			break;
		case FUNC_ENCAPSULATED_INTERFACE_TRANSPORT:
			func_encapsulated_interface_transport();
			break;
		default:
			report_error(ERR_ILLEGAL_FUNCTION);
	}

	// Is there a deferred error pending?
	if (pending_error)
		report_error(pending_error);

	// Finish reply frame
	write_u16(crc16(tx_buf, tx_size));
}

void modbus_slave_error(void)
{
	pending_error = ERR_SLAVE_DEVICE_FAILURE;
}
