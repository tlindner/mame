// license:GPL-v2
// copyright-holders:Tormod Volden, tim lindner
/*********************************************************************

	spi65b.h: 65SPI/B

	An SPI interface for the 65C02 family of microprocessors.

	http://www.6502.org/users/andre/spi65b/index.html

	This device was created to provide a basic SPI interface for the 65xx family of
	microprocessors. Currently, the only way to provide SPI is to bit-bang it using a
	6522 or equivalent device. That uses a lot of microprocessor time and program space.

	This device takes care of the data loading, shifting, clocking, and control - freeing
	the microprocessor for more important duties. There is interrupt support to allow an
	ISR to handle the SPI interface. The status register provides signals for polling
	if interrupts are not desired.


*********************************************************************/

#ifndef MAME_BUS_SPI_SPI65B_H
#define MAME_BUS_SPI_SPI65B_H

#pragma once

/***************************************************************************
	TYPE DEFINITIONS
***************************************************************************/

class spi65b_bus_device;

// ======================> spi65b_bus_slot_device
class spi65b_bus_slot_device final : public device_t,
									public device_slot_interface
{
	public:
		// construction/destruction
		template <typename T, typename U>
		spi65b_bus_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&spi65b_bus_tag, U &&opts, const char *dflt)
		: spi65b_bus_slot_device(mconfig, tag, owner, DERIVED_CLOCK(1, 1), std::forward<T>(spi65b_bus_tag), std::forward<U>(opts), dflt)
		{
		}

		template <typename T, typename U>
		spi65b_bus_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock, T &&spi65b_bus_tag, U &&opts, const char *dflt)
		: spi65b_bus_slot_device(mconfig, tag, owner, clock)
		{
			option_reset();
			opts(*this);
			set_default_option(dflt);
			set_fixed(false);
			m_spi65b_bus.set_tag(std::forward<T>(spi65b_bus_tag));
		}

		spi65b_bus_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	protected:
		spi65b_bus_slot_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);
		// device-level overrides
		virtual void device_validity_check(validity_checker &valid) const override;
		virtual void device_resolve_objects() override;
		virtual void device_start() override;

		// configuration
		required_device<spi65b_bus_device> m_spi65b_bus;
};

// device type definition
DECLARE_DEVICE_TYPE(SPI65BBUS_SLOT, spi65b_bus_slot_device)

class device_spi65b_bus_interface;

class spi65b_bus_device : public device_t,
						  public device_single_card_slot_interface<device_spi65b_bus_interface>
{
	public:
		// construction/destruction
		spi65b_bus_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

		// inline configuration
		auto irq_w() { return m_out_irq_cb.bind(); }

		void add_spi65b_bus_card(int slot, device_spi65b_bus_interface *card);
		device_spi65b_bus_interface *get_spi65b_bus_card(int slot);

		void set_irq_line(int state, int slot, bool update);

		DECLARE_WRITE_LINE_MEMBER( irq_w );

		// reading and writing to registers
		DECLARE_READ8_MEMBER(spi65b_bus_read);
		DECLARE_WRITE8_MEMBER(spi65b_bus_write);

		// device-level overrides
		virtual void device_resolve_objects() override;
		virtual void device_start() override;
		virtual void device_reset() override;

	protected:
		spi65b_bus_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);


		// internal state
		devcb_write_line            m_out_irq_cb;
		device_spi65b_bus_interface *m_device_list[4];

	private:
		void update_line(void);

		// device references
		uint8_t                     m_reg_data_in;
		uint8_t                     m_reg_data_out;
		uint8_t                     m_status;
		uint8_t                     m_clock_divisor;
		uint8_t                     m_slave_select_interrupt_enable;
};


// device type definition
DECLARE_DEVICE_TYPE(SPI65BBUS, spi65b_bus_device)

// ======================> device_spi65b_bus_interface

class device_spi65b_bus_interface : public device_interface
{
	friend class spi65b_bus_device;
	public:
		// construction/destruction
		device_spi65b_bus_interface(const machine_config &mconfig, device_t *device);
		virtual ~device_spi65b_bus_interface();

		device_spi65b_bus_interface *next() const { return m_next; }

		virtual uint8_t spi_transfer(uint8_t data_out, int slave_select_active);

		// inline configuration
		void set_spi65b_bus(spi65b_bus_device *spi65b_bus, const char *slottag) { m_spi65b_bus = spi65b_bus; m_spi65b_bus_slottag = slottag; }
		template <typename T> void set_onboard(T &&spi65b_bus) { m_spi65b_bus_finder.set_tag(std::forward<T>(spi65b_bus)); m_spi65b_bus_slottag = device().tag(); }

	protected:
		void raise_slot_irq() { m_spi65b_bus->set_irq_line(ASSERT_LINE, m_slot, true); }
		void lower_slot_irq() { m_spi65b_bus->set_irq_line(CLEAR_LINE, m_slot, true); }

		device_spi65b_bus_interface(const machine_config &mconfig, device_t &device);

		virtual void interface_validity_check(validity_checker &valid) const override;
		virtual void interface_pre_start() override;

		int slotno() const { assert(m_spi65b_bus); return m_slot; }
		spi65b_bus_device &spi65b_bus() { assert(m_spi65b_bus); return *m_spi65b_bus; }

	private:
		optional_device<spi65b_bus_device> m_spi65b_bus_finder;
		spi65b_bus_device *m_spi65b_bus;
		const char *m_spi65b_bus_slottag;
		int m_slot;
		device_spi65b_bus_interface *m_next;
};

#endif // MAME_BUS_SPI_SPI65B_H
