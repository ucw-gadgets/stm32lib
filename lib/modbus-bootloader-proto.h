/*
 *	MODBUS Bootloader -- Protocol
 *
 *	(c) 2023 Martin Mareš <mj@ucw.cz>
 *
 *	Licensed under the GNU LGPL v3 or any later version.
 */

enum bl_input_reg {
	BL_INPUT_MAGIC_HI = 0xe000,
	BL_INPUT_MAGIC_LO,
	BL_INPUT_LOADER_VERSION,
	BL_INPUT_STATUS,
	// The following registers are not available in application mode
	BL_INPUT_VENDOR_ID,
	BL_INPUT_DEVICE_ID,
	BL_INPUT_SERIAL_NUMBER,		// 12 characters as in USB
	BL_INPUT_BLOCK_SIZE = BL_INPUT_SERIAL_NUMBER + 6,
	BL_INPUT_FLASH_SIZE,		// kB
	BL_INPUT_MAX,
};

enum bl_holding_reg {
	BL_HOLD_COMMAND = 0xe000,
	BL_HOLD_BLOCK_NUMBER,
	BL_HOLD_MAX,
	BL_HOLD_BLOCK_DATA = BL_HOLD_COMMAND + 0x100,	// next BL_BLOCK_SIZE/2 registers contain block data
};

#define BL_MAGIC_HI 0x426f	// "Bo"
#define BL_MAGIC_LO 0x6f54	// "oT"
#define BL_LOADER_VERSION 0x0001

enum bl_status {
	BL_STATUS_APP = 1,	// In application code, needs exit command
	BL_STATUS_READY,	// Boot loader ready
	BL_STATUS_FLASHING,	// Flashing in progress
	BL_STATUS_ERROR,	// Error occurred
	BL_STATUS_CORRUPTED,	// Corrupted firmware found
	BL_STATUS_BOOTING,	// About to boot current firmware
};

enum bl_command {
	BL_COMMAND_BOOT = 1,	// Boot current firmware
	BL_COMMAND_FLASH_START,	// Enter flash mode
	BL_COMMAND_FLASH_END,	// Quit flash mode and verify checksum
	BL_COMMAND_FLASH_BLOCK,	// Flash one block
	BL_COMMAND_EXIT_APP,	// Exit from application code to boot loader
};

static inline bool bl_app_check_input_register(u16 addr)
{
	switch (addr) {
		case BL_INPUT_MAGIC_HI:
		case BL_INPUT_MAGIC_LO:
		case BL_INPUT_LOADER_VERSION:
		case BL_INPUT_STATUS:
			return true;
		default:
			return false;
	}
}

static inline u16 bl_app_get_input_register(u16 addr)
{
	switch (addr) {
		case BL_INPUT_MAGIC_HI:
			return BL_MAGIC_HI;
		case BL_INPUT_MAGIC_LO:
			return BL_MAGIC_LO;
		case BL_INPUT_LOADER_VERSION:
			return BL_LOADER_VERSION;
		case BL_INPUT_STATUS:
			return BL_STATUS_APP;
		default:
			return 0;
	}
}

static inline bool bl_app_check_holding_register(u16 addr)
{
	return (addr == BL_HOLD_COMMAND);
}

static inline bool bl_app_set_holding_register(u16 addr, u16 value)
{
	return (addr == BL_HOLD_COMMAND && value == BL_COMMAND_EXIT_APP);
}
