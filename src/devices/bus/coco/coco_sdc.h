// license:BSD-3-Clause
// copyright-holders:Tim Lindner
#ifndef MAME_BUS_COCO_COCO_SDC_H
#define MAME_BUS_COCO_COCO_SDC_H

#pragma once

#include "cococart.h"
#include "cpu/avr8/avr8.h"
#include "machine/intelfsh.h"
#include "machine/spi_sdcard.h"

class coco_sdc_device : public device_t, public device_cococart_interface
{
public:
	coco_sdc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	static constexpr feature_type imperfect_features() { return feature::DISK; }

	virtual u8 *get_cart_base() override;
	virtual u32 get_cart_size() override;
	virtual bool cts_read_is_dynamic() override { return true; }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_reset_after_children() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual u8 cts_read(offs_t offset) override;
	virtual u8 scs_read(offs_t offset) override;
	virtual void scs_write(offs_t offset, u8 data) override;

private:
	void program_map(address_map &map) ATTR_COLD;
	void data_map(address_map &map) ATTR_COLD;
	void port_b_w(u8 data);
	void port_c_w(u8 data);
	void port_d_w(u8 data);
	u8 port_d_r();
	u8 mcu_read(u8 reg) const;
	void mcu_write(u8 reg, u8 data);
	void update_lines();
	void update_mcu_irq();

	required_device<atmega328_device> m_mcu;
	required_device<sst_39sf010_device> m_flash;
	required_device<spi_sdcard_device> m_sdcard;
	required_ioport m_switches;
	required_ioport m_autostart;
	bool m_power_on = true;
	u8 m_bank = 0;
	u8 m_port_c = 0;
	u8 m_port_d = 0;
	int m_miso = 1;
	u8 m_registers[4] = { 0, 0, 1, 0 };
	u8 m_command = 0;
	u8 m_dskreg = 0;
	u8 m_flash_data = 0;
	bool m_flash_programming = false;
	bool m_command_pending = false;
	bool m_drq = false;
	bool m_intrq = false;
	bool m_strobe_write = false;
	u8 m_strobe_reg = 0;
	u8 m_read_data = 0xff;
	bool m_transfer_active = false;
	bool m_type1 = true;
};

DECLARE_DEVICE_TYPE(COCO_SDC, coco_sdc_device)

#endif // MAME_BUS_COCO_COCO_SDC_H
