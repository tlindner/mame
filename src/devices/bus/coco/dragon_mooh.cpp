// license:BSD-3-Clause
// copyright-holders:tim lindner
/***************************************************************************

    dragon_mooh.cpp

	MMU:



	Banked ROM:
		The SCS ROM space can be banked in 8K segments by writing to bit 0-2
		of FF64

		If bit 3 is set, banking is in 16k segments defined by bits 1-2.

		If bit 4 is set, the bank is not reset to zero when reset.


	SPI:




	Extension header:
***************************************************************************/

#include "emu.h"
#include "dragon_mooh.h"
#include "bus/spi/spi65b.h"
#include "bus/spi/spi65b_sdcard.h"

#define LOG_MOOH_MMU         (1 << 0 )
#define LOG_MOOH_BANKED_ROM  (1 << 1 )
#define LOG_MOOH_EXT         (1 << 2 )

#define VERBOSE 0 //(LOG_MOOH_MMU|LOG_MOOH_BANKED_ROM|LOG_MOOH_EXT)
#include "logmacro.h"

#define CARTSLOT_TAG "cart"

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE_PRIVATE(DRAGON_MOOH, device_cococart_interface, dragon_mooh_device, "dragon_mooh", "Dragon MOOH PAK");


//**************************************************************************
//  MACHINE FRAGMENTS AND ADDRESS MAPS
//**************************************************************************

//-------------------------------------------------
//  SLOT_INTERFACE_START(spi65b_bus)
//-------------------------------------------------

void spi65b_slaves(device_slot_interface &device)
{
	device.option_add("sdcard", SPI65BBUS_SDCARD);
}

void dragon_mooh_device::device_add_mconfig(machine_config &config)
{
	RAM(config, "staticram").set_default_size("512K").set_default_value(0);

	SPI65BBUS(config, m_spi65b, 0);

	m_spi65b->irq_w().set(FUNC(dragon_mooh_device::irq_w));

	SPI65BBUS_SLOT(config, "slave0", m_spi65b, spi65b_slaves, "sdcard");
	SPI65BBUS_SLOT(config, "slave1", m_spi65b, spi65b_slaves, nullptr);
	SPI65BBUS_SLOT(config, "slave2", m_spi65b, spi65b_slaves, nullptr);
	SPI65BBUS_SLOT(config, "slave3", m_spi65b, spi65b_slaves, nullptr);
}

ROM_START(dragon_mooh)
	ROM_REGION(0x10000, CARTSLOT_TAG, ROMREGION_ERASE00)
	ROM_LOAD("sdbdos-eprom8-all-v1.rom", 0x0000, 0x10000, CRC(8ad667ac) SHA1(db4013d556da870362f750e3b31f172622803c64))
ROM_END

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  dragon_mooh_device - constructor
//-------------------------------------------------

dragon_mooh_device::dragon_mooh_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: device_t(mconfig, DRAGON_MOOH, tag, owner, clock),
		device_cococart_interface(mconfig, *this )
// 		, sam6883_friend_device_interface(mconfig, *this, 4)
		, m_rom_selection(0)
		, m_staticram(*this, "staticram")
		, m_spi65b(*this, "spi")
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void dragon_mooh_device::device_start()
{
// 	super::device_start();

	// install registers handlers handler
	write8_delegate wh = write8_delegate(*this, FUNC(dragon_mooh_device::mmu_write));
	read8_delegate rh = read8_delegate(*this, FUNC(dragon_mooh_device::mmu_read));
	install_readwrite_handler(0xFF90, 0xFF91, rh, wh);

	wh = write8_delegate(*this, FUNC(dragon_mooh_device::mmu_task_write));
	rh = read8_delegate(*this, FUNC(dragon_mooh_device::mmu_task_read));
	install_readwrite_handler(0xFFA0, 0xFFAF, rh, wh);

	wh = write8_delegate(*this, FUNC(dragon_mooh_device::rom_selection_write));
	rh = read8_delegate(*this, FUNC(dragon_mooh_device::rom_selection_read));
	install_readwrite_handler(0xFF64, 0xFF64, rh, wh);

// 	wh = write8_delegate(m_spi65b.target(), FUNC(spi65b_bus_device::spi65b_bus_write));
// 	rh = read8_delegate(m_spi65b.target(), FUNC(spi65b_bus_device::spi65b_bus_read));
	wh = write8_delegate(m_spi65b, FUNC(spi65b_bus_device::spi65b_bus_write));
	rh = read8_delegate(m_spi65b, FUNC(spi65b_bus_device::spi65b_bus_read));
	install_readwrite_handler(0xFF6C, 0xFF6F, rh, wh);

	save_pointer(NAME(m_mmu), ARRAY_LENGTH(m_mmu));
	save_pointer(NAME(m_mmu_task), ARRAY_LENGTH(m_mmu_task));
	save_item(NAME(m_rom_selection));

// set up ROM/RAM pointers
// 	m_rom = m_rom_region->base();
// 	m_cart_rom = m_cart_device->get_cart_base();
// 	m_cart_size = m_cart_device->get_cart_size();
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void dragon_mooh_device::device_reset()
{
	m_mmu[0] = 0;
	m_mmu[1] = 0;

	m_mmu_task[0] = 0x3f;
	m_mmu_task[1] = 0x3f;
	m_mmu_task[2] = 0x3f;
	m_mmu_task[3] = 0x3f;
	m_mmu_task[4] = 0x3f;
	m_mmu_task[5] = 0x3f;
	m_mmu_task[6] = 0x3f;
	m_mmu_task[7] = 0x3f;
	m_mmu_task[8] = 0x3f;
	m_mmu_task[9] = 0x3f;
	m_mmu_task[10] = 0x3f;
	m_mmu_task[11] = 0x3f;
	m_mmu_task[12] = 0x3f;
	m_mmu_task[13] = 0x3f;
	m_mmu_task[14] = 0x3f;
	m_mmu_task[15] = 0x3f;

	if( (m_rom_selection & 16) == 0) {
		m_rom_selection = 0;
	}

	cart_base_changed();

	// setup banks
// 	assert(ARRAY_LENGTH(m_read_banks) == ARRAY_LENGTH(m_write_banks));
// 	for (int i = 0; i < ARRAY_LENGTH(m_read_banks); i++)
// 	{
// 		char buffer[8];
// 		snprintf(buffer, ARRAY_LENGTH(buffer), "rbank%d", i);
// 		m_read_banks[i] = machine().root_device().membank(buffer);
// 		snprintf(buffer, ARRAY_LENGTH(buffer), "wbank%d", i);
// 		m_write_banks[i] = machine().root_device().membank(buffer);
// 	}
}


//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *dragon_mooh_device::device_rom_region() const
{
	return ROM_NAME( dragon_mooh );
}


//-------------------------------------------------
//  mmu_read / FF90 / FF91
//-------------------------------------------------

READ8_MEMBER(dragon_mooh_device::mmu_read)
{
	LOGMASKED(LOG_MOOH_MMU, "dragon_mooh_device::mmu_read offset %02x, data: %02x\n", offset, m_mmu[offset & 1] );

	return m_mmu[offset & 1];
}


//-------------------------------------------------
//  mmu_write // FF90 / FF91
//-------------------------------------------------

WRITE8_MEMBER(dragon_mooh_device::mmu_write)
{
	LOGMASKED(LOG_MOOH_MMU, "dragon_mooh_device::mmu_write offset %02x, data: %02x\n", offset, data );

    m_mmu[offset & 1] = data;
}

//-------------------------------------------------
//  mmu_task_read / FFA0 - FFAF
//-------------------------------------------------

READ8_MEMBER(dragon_mooh_device::mmu_task_read)
{
	LOGMASKED(LOG_MOOH_MMU, "dragon_mooh_device::mmu_task_read offset %02x, data: %02x\n", offset, m_mmu_task[offset & 15] );

	return m_mmu_task[offset & 15];
}


//-------------------------------------------------
//  mmu_task_write / FFA0 - FFAF
//-------------------------------------------------

WRITE8_MEMBER(dragon_mooh_device::mmu_task_write)
{
	LOGMASKED(LOG_MOOH_MMU, "dragon_mooh_device::mmu_task_write offset %02x, data: %02x\n", offset, data );

    m_mmu_task[offset & 15] = data;
}

//-------------------------------------------------
//  rom_selection_read / FF64
//-------------------------------------------------

READ8_MEMBER(dragon_mooh_device::rom_selection_read)
{
	LOGMASKED(LOG_MOOH_BANKED_ROM, "dragon_mooh_device::rom_selection_read offset %02x, data: %02x\n", offset, m_rom_selection );

	return m_rom_selection;
}


//-------------------------------------------------
//  rom_selection_write / FF64
//-------------------------------------------------

WRITE8_MEMBER(dragon_mooh_device::rom_selection_write)
{
	LOGMASKED(LOG_MOOH_BANKED_ROM, "dragon_mooh_device::rom_selection_write offset %02x, data: %02x\n", offset, data );

	if( m_rom_selection != data ) {
		m_rom_selection = data;
		cart_base_changed();
	}
}

/*-------------------------------------------------
    get_cart_memregion
-------------------------------------------------*/

memory_region* dragon_mooh_device::get_cart_memregion()
{
	return memregion(CARTSLOT_TAG);
}

//-------------------------------------------------
//  get_cart_size
//-------------------------------------------------

uint32_t dragon_mooh_device::get_cart_size()
{
	if( m_rom_selection & 8 ) {
		return 0x4000;
	} else {
		return 0x2000;
	}
}

//-------------------------------------------------
//  get_cart_base
//-------------------------------------------------

uint8_t *dragon_mooh_device::get_cart_base()
{
	uint8_t *rom = memregion(CARTSLOT_TAG)->base();
	uint32_t rom_length = memregion(CARTSLOT_TAG)->bytes();
	int pos;

	if( m_rom_selection & 8 ) {
		pos = (m_rom_selection & 6) >> 1;
		return &rom[(pos * 0x2000) % rom_length];
	} else {
		pos = m_rom_selection & 7;
		return &rom[(pos * 0x4000) % rom_length];
	}
}

WRITE_LINE_MEMBER(dragon_mooh_device::irq_w)
{
	// set the CART line
	owning_slot().set_line_value(line::CART, state == ASSERT_LINE ? line_value::ASSERT : line_value::CLEAR);
}
