// license:BSD-3-Clause
// copyright-holders:Nathan Woods
/***************************************************************************

    coco_multi.cpp

    Code for emulating CoCo's Multi-Pak Interface

     The Multi-Pak interface multiplexes all I/O lines from the Color
     Computer's expansion port to four identical ports. All I/O lines
     are continuously multiplexed except:

     Pin 36 - *SCS
     Pin 32 - *CTS
     Pin  8 - *CART

     These I/O lines are switched in one of two ways. First, is the front
     panel, four position switch. When adjusted the switch will direct the
     MPI to target these three I/O lines to the selected slot.

     Second, the MPI will listen to writes to $FF7F and respond accordingly:

     bit 0 --\___ Target *SCS to this slot number
     bit 1 --/
     bit 2 ------ Ignore
     bit 3 ------ Ignore
     bit 4 --\___ Target *CTS and *CART to this slot number
     bit 5 --/
     bit 6 ------ Ignore
     bit 7 ------ Ignore

     After writing to $FF7F, the position of the physical switch has no
     effect until reset.

     Reading is supported on $FF7F. It will reflect the position of the
     physical switch. Until data is written to $FF7F, then it will only
     reflect what has been written until a reset.

     A common modification users of the OS-9 operating system made was to
     tie all of the *CART pins together on the MPI motherboard. The *CART
     line is connected to the 6809's IRQ line. This allowed any hardware
     device in any slot to signal an IRQ to the CPU, no matter what the
     switch position was. OS-9 was designed from the very start to poll
     each device attached on every IRQ signal.

     Because of sloppy address decoding the original MPI also responds to
     $FF9F. No software is known to take advantage of this. After the
     introduction of the CoCo 3, which uses $FF9F internally, Tandy provided
     free upgrades to any MPI to fix this problem.  This behavior is not
     emulated (yet).

     Slots seem to be one-counted (i.e. - 1-4, not 0-3) by convention

***************************************************************************/

#include "emu.h"
#include "coco_multi.h"

#include "cococart.h"

#define SWITCH_CONFIG_TAG   "switch"

#define LOG_CART   (1U << 1) // shows cart line changes
#define LOG_SWITCH (1U << 2) // shows switch changes
//#define VERBOSE (LOG_CART|LOG_SWITCH)

#include "logmacro.h"

#define LOGCART(...)     LOGMASKED(LOG_CART,  __VA_ARGS__)
#define LOGSWITCH(...)   LOGMASKED(LOG_SWITCH,  __VA_ARGS__)

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> coco_multipak_device

// Derive a device type from this template to select 1-16 slots.
// Slot 4 (or the last slot if fewer) defaults to an FDC and the switch selects it.
// Override device_add_mconfig for other wiring.
// Each $FF7F nibble selects a slot, using only the bits needed for SlotCount.
// Unused select bits read high; selections beyond SlotCount are disconnected.
template <unsigned SlotCount>
class coco_multipak_device
	: public device_t
	, public device_cococart_interface
	, public device_cococart_host_interface
	, public device_sound_interface
{
public:
	static_assert(SlotCount >= 1 && SlotCount <= 16, "Multi-Pak requires 1 to 16 slots");
	INPUT_CHANGED_MEMBER( switch_changed );
	INPUT_CHANGED_MEMBER( cart_strap_changed );

protected:
	coco_multipak_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock);

	// device-level overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void sound_stream_update(sound_stream &stream) override ATTR_COLD;
	virtual void device_resolve_objects() override ATTR_COLD;
	virtual u8 cts_read(offs_t offset) override;
	virtual void cts_write(offs_t offset, u8 data) override;
	virtual u8 scs_read(offs_t offset) override;
	virtual void scs_write(offs_t offset, u8 data) override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	virtual u8 *get_cart_base() override;
	virtual u32 get_cart_size() override;

	virtual address_space &cartridge_space() override;
	virtual void add_sound_route(device_sound_interface &sound_device, int output_index, double gain) override;
	virtual void set_sound_gain(device_sound_interface &sound_device, int output_index, double gain) override;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device references
	required_device_array<cococart_slot_device, SlotCount> m_slots;

	void update_line(int slot_number, line ln);

private:
	static constexpr unsigned FDC_SLOT = SlotCount < 4 ? SlotCount - 1 : 3;
	static constexpr u8 SELECT_MASK = SlotCount <= 2 ? SlotCount - 1 : SlotCount <= 4 ? 3 : SlotCount <= 8 ? 7 : 15;

	// internal state
	sound_stream *m_mixer_stream;
	u8 m_select;
	u8 m_block;

	// internal accessors
	int active_scs_slot_number() const;
	int active_cts_slot_number() const;
	cococart_slot_device &slot(int slot_number);
	cococart_slot_device *active_scs_slot();
	cococart_slot_device *active_cts_slot();

	// methods
	void set_select(u8 new_select);
	void update_cart_line();
	u8 ff7f_read();
	void ff7f_write(u8 data);
};

class coco_multipak4_device : public coco_multipak_device<4>
{
public:
	coco_multipak4_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);
};

class coco_multipak16_device : public coco_multipak_device<16>
{
public:
	coco_multipak16_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);
};

// ======================> dragon_multipak_device

class dragon_multipak_device : public coco_multipak_device<4>
{
public:
	// construction/destruction
	dragon_multipak_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
};


/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

static void coco_cart_slot1_3(device_slot_interface &device)
{
	coco_cart_add_basic_devices(device);
}

static void coco_cart_slot4(device_slot_interface &device)
{
	coco_cart_add_basic_devices(device);
	coco_cart_add_fdcs(device);
}


template <unsigned SlotCount>
void coco_multipak_device<SlotCount>::device_add_mconfig(machine_config &config)
{
	for (unsigned i = 0; i < SlotCount; ++i)
	{
		bool const fdc = i == FDC_SLOT;
		COCOCART_SLOT(config, m_slots[i], DERIVED_CLOCK(1, 1), fdc ? coco_cart_slot4 : coco_cart_slot1_3, fdc ? "fdc" : nullptr);
		m_slots[i]->cart_callback().set([this, i](int state) { update_line(i + 1, line::CART); });
		m_slots[i]->nmi_callback().set([this, i](int state) { update_line(i + 1, line::NMI); });
		m_slots[i]->halt_callback().set([this, i](int state) { update_line(i + 1, line::HALT); });
	}
}


static void dragon_cart_slot1_3(device_slot_interface &device)
{
	dragon_cart_add_basic_devices(device);
}

static void dragon_cart_slot4(device_slot_interface &device)
{
	dragon_cart_add_basic_devices(device);
	dragon_cart_add_fdcs(device);
}


void dragon_multipak_device::device_add_mconfig(machine_config &config)
{
	for (unsigned i = 0; i < m_slots.size(); ++i)
	{
		bool const last = i == m_slots.size() - 1;
		COCOCART_SLOT(config, m_slots[i], DERIVED_CLOCK(1, 1), last ? dragon_cart_slot4 : dragon_cart_slot1_3, last ? "dragon_fdc" : nullptr);
		m_slots[i]->cart_callback().set([this, i](int state) { update_line(i + 1, line::CART); });
		m_slots[i]->nmi_callback().set([this, i](int state) { update_line(i + 1, line::NMI); });
		m_slots[i]->halt_callback().set([this, i](int state) { update_line(i + 1, line::HALT); });
	}
}

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE_PRIVATE(COCO_MULTIPAK, device_cococart_interface, coco_multipak4_device, "coco_multipack", "CoCo Multi-Pak Interface")
DEFINE_DEVICE_TYPE_PRIVATE(COCO_MULTIPAK16, device_cococart_interface, coco_multipak16_device, "coco_multipack16", "CoCo 16-Slot Multi-Pak Interface")
DEFINE_DEVICE_TYPE_PRIVATE(DRAGON_MULTIPAK, device_cococart_interface, dragon_multipak_device, "dragon_multipack", "Dragon Multi-Pak Interface")



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  device_resolve_objects
//-------------------------------------------------

template <unsigned SlotCount>
void coco_multipak_device<SlotCount>::device_resolve_objects()
{
	device_cococart_interface::add_sound_route(*this, 0, 1.0);
}

//-------------------------------------------------
//  add_sound_route
//-------------------------------------------------

template <unsigned SlotCount>
void coco_multipak_device<SlotCount>::add_sound_route(device_sound_interface &sound_device, int output_index, double gain)
{
	sound_device.add_route(output_index, *this, gain, 0);
}

//-------------------------------------------------
//  add_sound_route
//-------------------------------------------------

template <unsigned SlotCount>
void coco_multipak_device<SlotCount>::set_sound_gain(device_sound_interface &sound_device, int output_index, double gain)
{
	sound_device.set_route_gain(output_index, this, 0, gain);
}

//-------------------------------------------------
//  sound_stream_update
//-------------------------------------------------

template <unsigned SlotCount>
void coco_multipak_device<SlotCount>::sound_stream_update(sound_stream &stream)
{
	stream.copy(0, 0);
}

//-------------------------------------------------
//  input_ports - device-specific input ports
//-------------------------------------------------

template <unsigned SlotCount>
ioport_constructor coco_multipak_device<SlotCount>::device_input_ports() const
{
	return [](device_t &owner, ioport_list &portlist, std::ostream &errorbuf)
	{
		ioport_configurer configurer(owner, portlist, errorbuf);
		PORT_START(SWITCH_CONFIG_TAG)
		PORT_CONFNAME(SELECT_MASK ? SELECT_MASK : 1, FDC_SLOT, "Multi-Pak Slot Switch") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(coco_multipak_device<SlotCount>::switch_changed), 0)
		static char const *const names[] = { "Slot 1", "Slot 2", "Slot 3", "Slot 4", "Slot 5", "Slot 6", "Slot 7", "Slot 8", "Slot 9", "Slot 10", "Slot 11", "Slot 12", "Slot 13", "Slot 14", "Slot 15", "Slot 16" };
		for (unsigned i = 0; i < SlotCount; ++i)
			configurer.setting_alloc(i, names[i]);
		PORT_START("cart_strap")
		PORT_CONFNAME(0x01, 0x00, "Strap CART lines together") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(coco_multipak_device<SlotCount>::cart_strap_changed), 0)
		PORT_CONFSETTING(0x00, DEF_STR(No))
		PORT_CONFSETTING(0x01, DEF_STR(Yes))
	};
}

//-------------------------------------------------
//  coco_multipak_device - constructor
//-------------------------------------------------

template <unsigned SlotCount>
coco_multipak_device<SlotCount>::coco_multipak_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_cococart_interface(mconfig, *this)
	, device_sound_interface(mconfig, *this)
	, m_slots(*this, "slot%u", 1), m_select(0), m_block(0)
{
}

coco_multipak4_device::coco_multipak4_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: coco_multipak_device<4>(mconfig, COCO_MULTIPAK, tag, owner, clock)
{
}

coco_multipak16_device::coco_multipak16_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: coco_multipak_device<16>(mconfig, COCO_MULTIPAK16, tag, owner, clock)
{
}

dragon_multipak_device::dragon_multipak_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: coco_multipak_device<4>(mconfig, DRAGON_MULTIPAK, tag, owner, clock)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

template <unsigned SlotCount>
void coco_multipak_device<SlotCount>::device_start()
{
	// install $FF7F handler
	install_readwrite_handler(0xff7f, 0xff7f, read8smo_delegate(*this, FUNC(coco_multipak_device<SlotCount>::ff7f_read)), write8smo_delegate(*this, FUNC(coco_multipak_device<SlotCount>::ff7f_write)));

	// initial state
	m_select = FDC_SLOT * 0x11;
	m_block = 0;

	m_mixer_stream = stream_alloc(1, 1, machine().sample_rate());

	// save state
	save_item(NAME(m_select));
	save_item(NAME(m_block));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

template <unsigned SlotCount>
void coco_multipak_device<SlotCount>::device_reset()
{
	u8 const selection = ioport(SWITCH_CONFIG_TAG)->read();
	set_select(selection | (selection << 4));
	m_block = 0;
}


//-------------------------------------------------
//  switch_changed - panel switch changed
//-------------------------------------------------

template <unsigned SlotCount>
INPUT_CHANGED_MEMBER( coco_multipak_device<SlotCount>::switch_changed )
{
	if (m_block == 0)
		set_select(newval | (newval << 4));
}

template <unsigned SlotCount>
INPUT_CHANGED_MEMBER( coco_multipak_device<SlotCount>::cart_strap_changed )
{
	update_cart_line();
}


//**************************************************************************
//  INTERNAL ACCESSORS
//**************************************************************************

//-------------------------------------------------
//  active_scs_slot_number
//-------------------------------------------------

template <unsigned SlotCount>
int coco_multipak_device<SlotCount>::active_scs_slot_number() const
{
	return ((m_select >> 0) & SELECT_MASK) + 1;
}


//-------------------------------------------------
//  active_cts_slot_number
//-------------------------------------------------

template <unsigned SlotCount>
int coco_multipak_device<SlotCount>::active_cts_slot_number() const
{
	return ((m_select >> 4) & SELECT_MASK) + 1;
}


//-------------------------------------------------
//  slot
//-------------------------------------------------

template <unsigned SlotCount>
cococart_slot_device &coco_multipak_device<SlotCount>::slot(int slot_number)
{
	assert(slot_number >= 1 && slot_number <= SlotCount);
	return *m_slots[slot_number - 1];
}


//-------------------------------------------------
//  active_scs_slot
//-------------------------------------------------

template <unsigned SlotCount>
cococart_slot_device *coco_multipak_device<SlotCount>::active_scs_slot()
{
	int slot_number = active_scs_slot_number();
	return slot_number <= SlotCount ? &slot(slot_number) : nullptr;
}


//-------------------------------------------------
//  active_cts_slot
//-------------------------------------------------

template <unsigned SlotCount>
cococart_slot_device *coco_multipak_device<SlotCount>::active_cts_slot()
{
	int slot_number = active_cts_slot_number();
	return slot_number <= SlotCount ? &slot(slot_number) : nullptr;
}


//**************************************************************************
//  METHODS
//**************************************************************************

//-------------------------------------------------
//  set_select
//-------------------------------------------------

template <unsigned SlotCount>
void coco_multipak_device<SlotCount>::set_select(u8 new_select)
{
	LOGSWITCH( "set_select: 0x%02X\n", new_select);

	// change value
	m_select = new_select;

	update_cart_line();

	cart_base_changed();
}

template <unsigned SlotCount>
void coco_multipak_device<SlotCount>::update_cart_line()
{
	line_value cart = line_value::CLEAR;
	if (ioport("cart_strap")->read())
	{
		for (auto &slot : m_slots)
		{
			line_value const value = slot->get_line_value(line::CART);
			if (value == line_value::ASSERT)
			{
				cart = value;
				break;
			}
			if (value == line_value::Q)
				cart = value;
		}
	}
	else if (active_cts_slot())
		cart = active_cts_slot()->get_line_value(line::CART);

	owning_slot().set_line_value(line::CART, cart);
}


//-------------------------------------------------
//  ff7f_read
//-------------------------------------------------

template <unsigned SlotCount>
u8 coco_multipak_device<SlotCount>::ff7f_read()
{
	return m_select | u8(~(SELECT_MASK * 0x11));
}

//-------------------------------------------------
//  ff7f_write
//-------------------------------------------------

template <unsigned SlotCount>
void coco_multipak_device<SlotCount>::ff7f_write(u8 data)
{
	m_block = 0xff;
	set_select(data);
}


//-------------------------------------------------
//  update_line
//-------------------------------------------------

template <unsigned SlotCount>
void coco_multipak_device<SlotCount>::update_line(int slot_number, line ln)
{
	bool propagate;

	// one of our child devices set a line; we may need to propagate it upwards
	switch (ln)
	{
	case line::CART:
		if (ioport("cart_strap")->read())
		{
			update_cart_line();
			return;
		}
		// only propagate if this is coming from the slot specified
		propagate = slot_number == active_cts_slot_number();
		LOGCART( "update_line: slot: %d, line: CART, value: %s, propogate: %s\n", slot_number, owning_slot().line_value_string(slot(slot_number).get_line_value(ln)), propagate ? "yes" : "no" );
		break;

	case line::NMI:
	case line::HALT:
		// always propagate these
		propagate = true;
		break;

	default:
		// do nothing
		propagate = false;
		break;
	}

	if (propagate)
		owning_slot().set_line_value(ln, slot(slot_number).get_line_value(ln));
}


//-------------------------------------------------
//  get_cart_base
//-------------------------------------------------

template <unsigned SlotCount>
u8 *coco_multipak_device<SlotCount>::get_cart_base()
{
	return active_cts_slot() ? active_cts_slot()->get_cart_base() : nullptr;
}


//-------------------------------------------------
//  get_cart_size
//-------------------------------------------------

template <unsigned SlotCount>
u32 coco_multipak_device<SlotCount>::get_cart_size()
{
	return active_cts_slot() ? active_cts_slot()->get_cart_size() : 0;
}


//-------------------------------------------------
//  cts_read
//-------------------------------------------------

template <unsigned SlotCount>
u8 coco_multipak_device<SlotCount>::cts_read(offs_t offset)
{
	return active_cts_slot() ? active_cts_slot()->cts_read(offset) : 0xff;
}


//-------------------------------------------------
//  cts_write
//-------------------------------------------------

template <unsigned SlotCount>
void coco_multipak_device<SlotCount>::cts_write(offs_t offset, u8 data)
{
	if (active_cts_slot())
		active_cts_slot()->cts_write(offset, data);
}


//-------------------------------------------------
//  scs_read
//-------------------------------------------------

template <unsigned SlotCount>
u8 coco_multipak_device<SlotCount>::scs_read(offs_t offset)
{
	return active_scs_slot() ? active_scs_slot()->scs_read(offset) : 0xff;
}


//-------------------------------------------------
//  scs_write
//-------------------------------------------------

template <unsigned SlotCount>
void coco_multipak_device<SlotCount>::scs_write(offs_t offset, u8 data)
{
	if (active_scs_slot())
		active_scs_slot()->scs_write(offset, data);
}


//-------------------------------------------------
//  cartridge_space
//-------------------------------------------------

template <unsigned SlotCount>
address_space &coco_multipak_device<SlotCount>::cartridge_space()
{
	return device_cococart_interface::cartridge_space();
}

// Provide every supported slot count for derived Multi-Pak devices.
template class coco_multipak_device<1>;
template class coco_multipak_device<2>;
template class coco_multipak_device<3>;
template class coco_multipak_device<4>;
template class coco_multipak_device<5>;
template class coco_multipak_device<6>;
template class coco_multipak_device<7>;
template class coco_multipak_device<8>;
template class coco_multipak_device<9>;
template class coco_multipak_device<10>;
template class coco_multipak_device<11>;
template class coco_multipak_device<12>;
template class coco_multipak_device<13>;
template class coco_multipak_device<14>;
template class coco_multipak_device<15>;
template class coco_multipak_device<16>;
