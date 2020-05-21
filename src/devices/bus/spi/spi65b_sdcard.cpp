// license:GPL-v2
// copyright-holders:Tormod Volden, tim lindner
/***************************************************************************

    SD Card connected to 65SPI/B controller chip

***************************************************************************/

#include "emu.h"
#include "spi65b_sdcard.h"

#define LOG_SDCARD         (1 << 0 )

#define VERBOSE 0 //(LOG_SDCARD)
#include "logmacro.h"

#define SPI65B_APP_FLAG 0x100 /* our flag */
#define SPI65B_CMD(x) (0x40 | x)
#define SPI65B_ACMD(x) (SPI65B_APP_FLAG | SPI65B_CMD(x))

#define SPI65B_STBY        0
#define SPI65B_CMD_FRAME   1
#define SPI65B_RESP        2
#define SPI65B_RESP_R7     3
#define SPI65B_SEND_CSD    4
#define SPI65B_TOKEN       5
#define SPI65B_SBLK_READ   6
#define SPI65B_R_TOKEN     7
#define SPI65B_SBLK_WRITE  8
#define SPI65B_DATA_RESP   9

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(SPI65BBUS_SDCARD, spi65b_bus_sdcard_device, "spi65b_bus_sdcard", "65SPI/B SD Card")

static constexpr int SPI65B_OCR  = 0x40300000;
static constexpr uint8_t SPI65B_CSD[16] = { 0x40, 0x0e, 0x00, 0x32, 0x5b, 0x59, 0x00, 0x00, 0x39, 0xb7, 0x7f, 0x80, 0x0a, 0x40, 0x00, 0x01 };

//-------------------------------------------------
//  dragon_sprites_device - constructor
//-------------------------------------------------

spi65b_bus_sdcard_device::spi65b_bus_sdcard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SPI65BBUS_SDCARD, tag, owner, clock)
	, device_spi65b_bus_interface(mconfig, *this)
	, device_image_interface(mconfig, *this)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void spi65b_bus_sdcard_device::device_start()
{
    m_idle_state = 0;
    m_acmd = 0;
	m_state_sd = SPI65B_STBY;

    save_item(NAME(m_state_sd));
    save_item(NAME(m_current_cmd));
    save_item(NAME(m_cmd_count));
    save_item(NAME(m_cmd_arg));
    save_item(NAME(m_block_buffer));
    save_item(NAME(m_address));
    save_item(NAME(m_block_count));
    save_item(NAME(m_resp_count));
    save_item(NAME(m_csd_count));
    save_item(NAME(m_idle_state));
    save_item(NAME(m_acmd));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void spi65b_bus_sdcard_device::device_reset()
{
	m_state_sd = SPI65B_STBY;
}

//-------------------------------------------------
//  i/o
//-------------------------------------------------

uint8_t spi65b_bus_sdcard_device::spi_transfer(uint8_t data_out, int slave_select_active)
{
	int next = m_state_sd;
	uint8_t data_in = 0xFF;

	if (!slave_select_active) {
		next = SPI65B_STBY;
	}

	if (m_state_sd < SPI65B_CMD_FRAME && slave_select_active && (data_out & 0xC0) == 0x40) {
		/* start of command frame */
		if (m_acmd)
			m_current_cmd = SPI65B_ACMD(data_out);
		else
			m_current_cmd = data_out;
		m_acmd = 0;
		m_cmd_count = 0;
		next = SPI65B_CMD_FRAME;
	} else if (m_state_sd == SPI65B_CMD_FRAME && slave_select_active) {
		/* inside command frame */
		m_cmd_arg[m_cmd_count] = data_out;
		m_cmd_count++;
		if (m_cmd_count == 6) {
			m_address = (m_cmd_arg[0] << 24) | (m_cmd_arg[1] << 16) | (m_cmd_arg[2] << 8) | m_cmd_arg[3];
			next = SPI65B_RESP;
		}
	} else if (m_state_sd == SPI65B_RESP) {
		next = SPI65B_STBY; /* default R1 response */
		if (m_current_cmd == SPI65B_CMD(0))         /* GO_IDLE_STATE */
			m_idle_state = 1;
		else if (m_current_cmd == SPI65B_ACMD(41))  /* APP_SEND_OP_COND */
			m_idle_state = 0;
		else if (m_current_cmd == SPI65B_CMD(55))   /* APP_CMD */
			m_acmd = 1;
		else if (m_current_cmd == SPI65B_CMD(17))   /* READ_SINGLE_BLOCK */
			next = SPI65B_TOKEN;
		else if (m_current_cmd == SPI65B_CMD(24))   /* WRITE_BLOCK */
			next = SPI65B_R_TOKEN;
		else if (m_current_cmd == SPI65B_CMD(9))    /* SPI65B_SEND_CSD */
			next = SPI65B_TOKEN;
		else if (m_current_cmd == SPI65B_CMD(8)) {  /* SEND_IF_COND */
			next = SPI65B_RESP_R7;
			m_address = 0x1AA; /* voltage (use m_cmd_arg?) */
			m_resp_count = 0;
		} else if (m_current_cmd == SPI65B_CMD(58)) { /* READ_OCR */
			next = SPI65B_RESP_R7;
			m_address = SPI65B_OCR; /* FIXME use ocr array */
			m_resp_count = 0;
		}
		data_in = m_idle_state; /* signal Success + Idle State in R1 */
		LOGMASKED(LOG_SDCARD, "spi65b_bus_sdcard_device::spi_transfer (%0x %d) ", m_current_cmd, m_idle_state );
	} else if (m_state_sd == SPI65B_RESP_R7) {
		data_in = (m_address >> ((3 - m_resp_count) * 8)) & 0xFF;
		if (++m_resp_count == 4)
			next = SPI65B_STBY;
	} else if (m_state_sd == SPI65B_TOKEN && m_current_cmd == SPI65B_CMD(9)) {
		m_csd_count = 0;
		data_in = 0xFE;
		next = SPI65B_SEND_CSD;
	} else if (m_state_sd == SPI65B_TOKEN && m_current_cmd == SPI65B_CMD(17)) {
        fseek(m_address * 512, SEEK_SET);
        fread(m_block_buffer, 512);
		m_block_count = 0;
		data_in = 0xFE;
		next = SPI65B_SBLK_READ;
	} else if (m_state_sd == SPI65B_R_TOKEN && m_current_cmd == SPI65B_CMD(24)) {
		if (data_out == 0xFE) {
			m_block_count = 0;
			next = SPI65B_SBLK_WRITE;
		}
	} else if (m_state_sd == SPI65B_SBLK_READ) {
		if (m_block_count < 512)
			data_in = m_block_buffer[m_block_count];
		else if (m_block_count == 512)
			data_in = 0xAA; /* fake CRC 1 */
		else if (m_block_count == 512 + 1) {
			data_in = 0xAA; /* fake CRC 2 */
			next = SPI65B_STBY;
		}
		m_block_count++;
	} else if (m_state_sd == SPI65B_SBLK_WRITE) {
		if (m_block_count < 512)
			m_block_buffer[m_block_count] = data_out;
		else if (m_block_count == 512 + 1) { /* CRC ignored */
            fseek(m_address * 512, SEEK_SET);
            fwrite(m_block_buffer, 512);
			next = SPI65B_DATA_RESP;
		}
		m_block_count++;
	} else if (m_state_sd == SPI65B_DATA_RESP) {
		data_in = 0x05; /* Data Accepted */
		next = SPI65B_STBY;
	} else if (m_state_sd == SPI65B_SEND_CSD) {
		if (m_csd_count < 16)
			data_in = SPI65B_CSD[m_csd_count];
		else {
			data_in = 0xAA; /* fake CRC */
			next = SPI65B_STBY;
		}
		m_csd_count++;
	}
	LOGMASKED(LOG_SDCARD, " <- %02x\n", data_in);
	m_state_sd = next;
	return data_in;
}
