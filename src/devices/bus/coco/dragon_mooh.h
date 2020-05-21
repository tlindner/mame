// license:BSD-3-Clause
// copyright-holders:tim lindner
#ifndef MAME_BUS_COCO_DRAGON_MOOH_H
#define MAME_BUS_COCO_DRAGON_MOOH_H

#include "cococart.h"
#include "machine/6883sam.h"
#include "machine/ram.h"
#include "bus/spi/spi65b.h"

#pragma once

// device type definition
namespace
{
	// ======================> dragon_mooh_device

	class dragon_mooh_device
			: public device_t
			, public device_cococart_interface
//			, public sam6883_friend_device_interface
	{
	public:
		// construction/destruction
		dragon_mooh_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

		// optional information overrides
		virtual const tiny_rom_entry *device_rom_region() const override;
		virtual void device_add_mconfig(machine_config &config) override;
		virtual void device_reset() override;

	protected:
		// device-level overrides
		virtual void device_start() override;
		DECLARE_READ8_MEMBER(mmu_read);
		DECLARE_WRITE8_MEMBER(mmu_write);
		DECLARE_READ8_MEMBER(mmu_task_read);
		DECLARE_WRITE8_MEMBER(mmu_task_write);
		DECLARE_READ8_MEMBER(rom_selection_read);
		DECLARE_WRITE8_MEMBER(rom_selection_write);
        virtual uint8_t *get_cart_base() override;
        virtual uint32_t get_cart_size() override;
    	virtual memory_region* get_cart_memregion() override;

	private:
		uint8_t                                 m_mmu[2];
		uint8_t                                 m_mmu_task[8 * 2];
		uint8_t                                 m_rom_selection;
 		required_device<ram_device>             m_staticram;
 		required_device<spi65b_bus_device>      m_spi65b;
//         memory_bank *                           m_read_banks[9];
//         memory_bank *                           m_write_banks[9];
//
//         // memory
//     	required_device<ram_device> m_ram;
// 	    required_device<cococart_slot_device>   m_cart_device;
//         uint8_t *                               m_rom;
//         required_memory_region                  m_rom_region;
//         uint8_t *                               m_cart_rom;
//         uint32_t                                m_cart_size;
//
//         void update_memory(void);
//         void update_memory(int bank);
//         uint8_t *memory_pointer(uint32_t address);
        DECLARE_WRITE_LINE_MEMBER(irq_w);
	};

};

DECLARE_DEVICE_TYPE(DRAGON_MOOH, device_cococart_interface)


#endif // MAME_BUS_COCO_DRAGON_MOOH_H
