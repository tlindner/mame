// license:BSD-3-Clause
// copyright-holders:Nathan Woods
/***************************************************************************

    6883sam.h

    Motorola 6883 Synchronous Address Multiplexer

***************************************************************************/

#ifndef MAME_MACHINE_6883SAM_H
#define MAME_MACHINE_6883SAM_H

#pragma once

#include "machine/ram.h"

//**************************************************************************
//  SAM6883 CORE
//**************************************************************************

class device_sam_map_host_interface
{
public:
	virtual void s1_rom0_map(address_map &map) {};
	virtual void s2_rom1_map(address_map &map) {};
	virtual void s3_rom2_map(address_map &map) {};
	virtual void s4_io0_map(address_map &map) {};
	virtual void s5_io1_map(address_map &map) {};
	virtual void s6_io2_map(address_map &map) {};
	virtual void s7_res_map(address_map &map) {};
};

// base class so that GIME emulation can use some functionality
class sam6883_friend_device_interface : public device_interface
{
public:
	sam6883_friend_device_interface(const machine_config &mconfig, device_t &device, int divider);

protected:
	// SAM state constants
	static const uint16_t SAM_STATE_TY = 0x8000;
	static const uint16_t SAM_STATE_M1 = 0x4000;
	static const uint16_t SAM_STATE_M0 = 0x2000;
	static const uint16_t SAM_STATE_R1 = 0x1000;
	static const uint16_t SAM_STATE_R0 = 0x0800;
	static const uint16_t SAM_STATE_P1 = 0x0400;
	static const uint16_t SAM_STATE_F6 = 0x0200;
	static const uint16_t SAM_STATE_F5 = 0x0100;
	static const uint16_t SAM_STATE_F4 = 0x0080;
	static const uint16_t SAM_STATE_F3 = 0x0040;
	static const uint16_t SAM_STATE_F2 = 0x0020;
	static const uint16_t SAM_STATE_F1 = 0x0010;
	static const uint16_t SAM_STATE_F0 = 0x0008;
	static const uint16_t SAM_STATE_V2 = 0x0004;
	static const uint16_t SAM_STATE_V1 = 0x0002;
	static const uint16_t SAM_STATE_V0 = 0x0001;

	static constexpr int SAM_BIT_V0 = 0;
	static constexpr int SAM_BIT_V1 = 1;
	static constexpr int SAM_BIT_V2 = 2;
	static constexpr int SAM_BIT_F0 = 3;
	static constexpr int SAM_BIT_F1 = 4;
	static constexpr int SAM_BIT_F2 = 5;
	static constexpr int SAM_BIT_F3 = 6;
	static constexpr int SAM_BIT_F4 = 7;
	static constexpr int SAM_BIT_F5 = 8;
	static constexpr int SAM_BIT_F6 = 9;
	static constexpr int SAM_BIT_P1 = 10;
	static constexpr int SAM_BIT_R0 = 11;
	static constexpr int SAM_BIT_R1 = 12;
	static constexpr int SAM_BIT_M0 = 13;
	static constexpr int SAM_BIT_M1 = 14;
	static constexpr int SAM_BIT_TY = 15;

	// incidentals
	required_device<cpu_device> m_cpu;
	required_device<ram_device> m_ram;


	// device state
	uint16_t m_sam_state;

	// base clock divider (/4 for MC6883, /8 for GIME)
	int m_divider;

	ATTR_FORCE_INLINE uint16_t display_offset()
	{
		return ((m_sam_state & (SAM_STATE_F0|SAM_STATE_F1|SAM_STATE_F2|SAM_STATE_F3|SAM_STATE_F4|SAM_STATE_F5|SAM_STATE_F6)) / SAM_STATE_F0) << 9;
	}

	ATTR_FORCE_INLINE uint16_t alter_sam_state(offs_t offset)
	{
		/* determine the mask */
		uint16_t mask = 1 << (offset >> 1);

		/* determine the new state */
		uint16_t new_state;
		if (offset & 0x0001)
			new_state = m_sam_state | mask;
		else
			new_state = m_sam_state & ~mask;

		/* specify the new state */
		uint16_t xorval = m_sam_state ^ new_state;
		m_sam_state = new_state;
		return xorval;
	}

	void update_cpu_clock();
	device_sam_map_host_interface *m_host;
};

class sam6883_device : public device_t, public device_memory_interface, public sam6883_friend_device_interface
{
public:
	template <typename T, typename U>
	sam6883_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock, T &&cpu_tag, U &&ram_tag)
		: sam6883_device(mconfig, tag, owner, clock)
	{
		m_cpu.set_tag(std::forward<T>(cpu_tag));
		m_ram.set_tag(std::forward<U>(ram_tag));
	}

	sam6883_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// CPU read/write handlers
	uint8_t read(offs_t offset);
	void write(offs_t offset, uint8_t data);

	// Disabled S decoding handlers
	uint8_t endc_read(offs_t offset);
	void endc_write(offs_t offset, uint8_t data);

	uint8_t vector_read(offs_t offset);
	uint8_t rom_read(offs_t offset);
	void rom_write(offs_t offset, uint8_t data);
	uint8_t io_read(offs_t offset);
	void io_write(offs_t offset, uint8_t data);

	void sam_mem(address_map &map);

	// typically called by VDG
	ATTR_FORCE_INLINE uint8_t display_read(offs_t offset)
	{
		if (offset == (offs_t) ~0)
		{
			/* the VDG is telling the counter to reset */
			m_counter = display_offset();
			m_counter_xdiv = 0;
			m_counter_ydiv = 0;
		}
		else if ((offset & 1) != (m_counter & 0x0001))
		{
			/* DA0 has been toggled - first bump B0-B3 of the counter */
			bool bit3_carry = (m_counter & 0x000F) == 0x000F;
			m_counter = (m_counter & ~0x000F) | ((m_counter + 1) & 0x000F);

			/* and apply the carry (if applicable */
			if (bit3_carry)
				counter_carry_bit3();
		}
		return m_ram_view.space()->read_byte(m_counter & m_counter_mask);
// 		return m_ram_space.read_byte(m_counter & m_counter_mask);

	}

	void hs_w(int state);

protected:
	// device-level overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_post_load() override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

private:
	memory_view m_ram_view;
	memory_view m_rom_view;
	memory_view m_io_view;

	// memory space configuration
	address_space_config        m_s0_ram_config;
	address_space_config        m_s1_rom0_config;
	address_space_config        m_s2_rom1_config;
	address_space_config        m_s3_rom2_config;
	address_space_config        m_s4_io0_config;
	address_space_config        m_s5_io1_config;
	address_space_config        m_s6_io2_config;
	address_space_config        m_s7_reserved_config;

	// memory spaces
	memory_access<16, 0, 0, ENDIANNESS_BIG>::cache m_ram_space;
	memory_access<14, 0, 0, ENDIANNESS_BIG>::cache m_rom_space[3];
	memory_access< 5, 0, 0, ENDIANNESS_BIG>::specific m_io_space[3];
	memory_access< 7, 0, 0, ENDIANNESS_BIG>::cache m_reserved_space;
	uint16_t                    m_counter_mask = 0;

	// SAM state
	uint16_t                    m_counter = 0;
	uint8_t                     m_counter_xdiv = 0;
	uint8_t                     m_counter_ydiv = 0;

	// typically called by CPU
	void internal_write(offs_t offset, uint8_t data);

	// called when there is a carry out of bit 3 on the counter
	ATTR_FORCE_INLINE void counter_carry_bit3()
	{
		uint8_t x_division;
// 		switch((m_sam_state & (SAM_STATE_V2|SAM_STATE_V1|SAM_STATE_V0)) / SAM_STATE_V0)
		switch(BIT(m_sam_state, SAM_BIT_V0, 3))
		{
			case 0x00:  x_division = 1; break;
			case 0x01:  x_division = 3; break;
			case 0x02:  x_division = 1; break;
			case 0x03:  x_division = 2; break;
			case 0x04:  x_division = 1; break;
			case 0x05:  x_division = 1; break;
			case 0x06:  x_division = 1; break;
			case 0x07:  x_division = 1; break;
			default:
				fatalerror("Should not get here\n");
				return;
		}

		if (++m_counter_xdiv >= x_division)
		{
			m_counter_xdiv = 0;
			m_counter ^= 0x0010;
			if ((m_counter & 0x0010) == 0x0000)
				counter_carry_bit4();
		}
	}

	// called when there is a carry out of bit 4 on the counter
	ATTR_FORCE_INLINE void counter_carry_bit4()
	{
		uint8_t y_division;
// 		switch((m_sam_state & (SAM_STATE_V2|SAM_STATE_V1|SAM_STATE_V0)) / SAM_STATE_V0)
		switch(BIT(m_sam_state, SAM_BIT_V0, 3))
		{
			case 0x00:  y_division = 12;    break;
			case 0x01:  y_division = 1;     break;
			case 0x02:  y_division = 3;     break;
			case 0x03:  y_division = 1;     break;
			case 0x04:  y_division = 2;     break;
			case 0x05:  y_division = 1;     break;
			case 0x06:  y_division = 1;     break;
			case 0x07:  y_division = 1;     break;
			default:
				fatalerror("Should not get here\n");
				return;
		}

		if (++m_counter_ydiv >= y_division)
		{
			m_counter_ydiv = 0;
			m_counter += 0x0020;
		}
	}

	// other members
	void horizontal_sync();
	void update_state();
	void update_memory();
};

DECLARE_DEVICE_TYPE(SAM6883, sam6883_device)

#endif // MAME_MACHINE_6883SAM_H
