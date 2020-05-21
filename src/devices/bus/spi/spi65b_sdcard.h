// license:GPL-v2
// copyright-holders:Tormod Volden, tim lindner
#ifndef MAME_BUS_SPI65B_SDCARD_H
#define MAME_BUS_SPI65B_SDCARD_H

#pragma once

#include "spi65b.h"

//**************************************************************************
//	TYPE DEFINITIONS
//**************************************************************************

// ======================> spi65b_bus_sdcard_device

class spi65b_bus_sdcard_device :
		public device_t,
		public device_spi65b_bus_interface,
		public device_image_interface
{
	public:
		// construction/destruction
		spi65b_bus_sdcard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

		virtual void device_start() override;
		virtual void device_reset() override;

		// i/o
		virtual uint8_t spi_transfer(uint8_t data_out, int slave_select_active) override;

		// image-level overrides
		virtual iodevice_t image_type() const noexcept override { return IO_MEMCARD; }
		virtual bool is_readable()	const noexcept override { return 1; }
		virtual bool is_writeable() const noexcept override { return 1; }
		virtual bool is_creatable() const noexcept override { return 0; }
		virtual bool must_be_loaded() const noexcept override { return 0; }
		virtual bool is_reset_on_load() const noexcept override { return 1; }
		virtual const char *file_extensions() const noexcept override { return "img"; }

	private:
		int m_state_sd;
		int m_current_cmd;
		int m_cmd_count;
		uint8_t m_cmd_arg[6];
		uint8_t m_block_buffer[512];
		int32_t m_address;
		int m_block_count;
		int m_resp_count;
		int m_csd_count;
		int m_idle_state;
		int m_acmd;
};

// device type definition
DECLARE_DEVICE_TYPE(SPI65BBUS_SDCARD, spi65b_bus_sdcard_device)

#endif // MAME_BUS_SPI65B_SDCARD_H
