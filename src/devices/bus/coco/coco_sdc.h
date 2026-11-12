// license:BSD-3-Clause
// copyright-holders:Tim Lindner
#ifndef MAME_BUS_COCO_COCO_SDC_H
#define MAME_BUS_COCO_COCO_SDC_H

#pragma once

#include "cococart.h"
#include "cpu/avr8/avr8.h"
#include "machine/intelfsh.h"
#include "machine/spi_sdcard.h"

class coco_sdc_device_base : public device_t, public device_cococart_interface
{
public:
	static constexpr feature_type imperfect_features() { return feature::DISK; }

	virtual u8 *get_cart_base() override;
	virtual u32 get_cart_size() override;
	virtual bool cts_read_is_dynamic() override;

protected:
	coco_sdc_device_base(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, bool dragon);

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_reset_after_children() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual u8 cts_read(offs_t offset) override;
	virtual u8 scs_read(offs_t offset) override;
	virtual void scs_write(offs_t offset, u8 data) override;
	virtual void update_lines() = 0;

	// Host register address as seen in CoCo mode (FF40 control, FF48 WD).
	virtual offs_t host_offset(offs_t offset) const;

	// Control register as the CPLD presents it on the MCU bus.
	virtual u8 mcu_dskreg() const;

	u8 m_dskreg;
	bool m_drq;
	bool m_intrq;

private:
	void program_map(address_map &map) ATTR_COLD;
	void port_b_w(u8 data);
	void port_c_w(u8 data);
	void port_d_w(u8 data);
	u8 port_d_r();
	u8 mcu_read(u8 reg) const;
	void mcu_write(u8 reg, u8 data);
	void update_mcu_irq();

	required_device<atmega328_device> m_mcu;
	required_device<sst_39sf010_device> m_flash;
	required_device<spi_sdcard_device> m_sdcard;
	required_ioport m_switches;
	bool const m_dragon;
	bool m_power_on;
	u8 m_bank;
	u8 m_port_c;
	u8 m_port_d;
	bool m_miso;
	u8 m_registers[4];
	u8 m_command;
	u8 m_flash_data;
	bool m_flash_programming;
	bool m_command_pending;
	bool m_strobe_write;
	u8 m_strobe_reg;
	u8 m_read_data;
	bool m_transfer_active;
	bool m_type1;
};

class coco_sdc_device : public coco_sdc_device_base
{
public:
	coco_sdc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void update_lines() override;

private:
	required_ioport m_autostart;
};

class dragon_sdc_device : public coco_sdc_device_base
{
public:
	dragon_sdc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void update_lines() override;

	virtual offs_t host_offset(offs_t offset) const override;
	virtual u8 mcu_dskreg() const override;
};

DECLARE_DEVICE_TYPE(COCO_SDC, coco_sdc_device)
DECLARE_DEVICE_TYPE(DRAGON_SDC, dragon_sdc_device)

#endif // MAME_BUS_COCO_COCO_SDC_H
