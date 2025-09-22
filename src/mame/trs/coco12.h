// license:BSD-3-Clause
// copyright-holders:Nathan Woods
/***************************************************************************

    coco12.h

    TRS-80 Radio Shack Color Computer 1/2 Family

***************************************************************************/

#ifndef MAME_TRS_COCO12_H
#define MAME_TRS_COCO12_H

#pragma once

#include "coco.h"

#include "machine/6883sam.h"
#include "machine/mos6551.h"
#include "machine/timer.h"
#include "sound/ay8910.h"
#include "video/mc6847.h"



//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

#define ACIA_TAG        "acia"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class coco12_state : public coco_state, public device_sam_map_host_interface

{
public:
	coco12_state(const machine_config &mconfig, device_type type, const char *tag)
		: coco_state(mconfig, type, tag)
		, device_sam_map_host_interface(*this, finder_base::DUMMY_TAG)
		, m_sam(*this, "sam")
		, m_vdg(*this, "vdg")
	{
	}

	uint8_t sam_read(offs_t offset);

	void horizontal_sync(int state);
	void field_sync(int state);

	void coco(machine_config &config);
	void cocoh(machine_config &config);
	void coco2b(machine_config &config);
	void coco2bh(machine_config &config);
	void cp400(machine_config &config);
	void t4426(machine_config &config);
	void cd6809(machine_config &config);

protected:
	virtual void device_start() override ATTR_COLD;

	// PIA1
	virtual void pia1_pb_changed(uint8_t data) override;

	sam6883_device &sam() { return *m_sam; }
	required_device<sam6883_device> m_sam;

	void coco_mem(address_map &map) ATTR_COLD;
	virtual void s1_rom0_map(address_map &map) override ATTR_COLD;
	virtual void s2_rom1_map(address_map &map) override ATTR_COLD;
	virtual void s3_rom2_map(address_map &map) override ATTR_COLD;
	virtual void s4_io0_map(address_map &map) override ATTR_COLD;
	virtual void s5_io1_map(address_map &map) override ATTR_COLD;
	virtual void s6_io2_map(address_map &map) override ATTR_COLD;
	virtual void s7_res_map(address_map &map) override ATTR_COLD;

protected:
	required_device<mc6847_base_device> m_vdg;
};

class deluxecoco_state : public coco12_state
{
public:
	deluxecoco_state(const machine_config &mconfig, device_type type, const char *tag)
		: coco12_state(mconfig, type, tag)
		, m_acia(*this, "mosacia")
		, m_psg(*this, "psg")
		, m_timer(*this, "timer")
		, m_ram_bank(*this, "ram_bank")
		, m_rom_view(*this, "rom_view")
	{
	}

	void deluxecoco(machine_config &config);
	void ff30_write(offs_t offset, uint8_t data);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;

	virtual void s0_ram_map(address_map &map) override ATTR_COLD;
	virtual void s3_rom2_map(address_map &map) override ATTR_COLD;
	virtual void s5_io1_map(address_map &map) override ATTR_COLD;

	required_device<mos6551_device> m_acia;
	required_device<ay8913_device> m_psg;
	required_device<timer_device> m_timer;

	TIMER_DEVICE_CALLBACK_MEMBER(perodic_timer);

private:
	memory_bank_creator m_ram_bank;
	memory_view m_rom_view;
};

class ms1600_state : public coco12_state
{
public:
	ms1600_state(const machine_config &mconfig, device_type type, const char *tag)
		: coco12_state(mconfig, type, tag)
	{
	}
protected:
	virtual void s3_rom2_map(address_map &map) override ATTR_COLD;
};

#endif // MAME_TRS_COCO12_H
