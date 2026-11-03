// license:BSD-3-Clause
// copyright-holders:Nathan Woods
/***************************************************************************

    6883sam.cpp

    Motorola 6883 Synchronous Address Multiplexer

    The Motorola 6883 SAM has 16 bits worth of state, but the state is changed
    by writing into a 32 byte address space; odd bytes set bits and even bytes
    clear bits.  Here is the layout:

        31  Set     TY  Map Type            0: RAM/ROM  1: All RAM
        30  Clear   TY  Map Type
        29  Set     M1  Memory Size         00: 4K      10: 64K Dynamic
        28  Clear   M1  Memory Size         01: 16K     11: 64K Static
        27  Set     M0  Memory Size
        26  Clear   M0  Memory Size
        25  Set     R1  MPU Rate            00: Slow    10: Fast
        24  Clear   R1  MPU Rate            01: Dual    11: Fast
        23  Set     R0  MPU Rate
        22  Clear   R0  MPU Rate
        21  Set     P1  Page #1             0: Low      1: High
        20  Clear   P1  Page #1
        19  Set     F6  Display Offset
        18  Clear   F6  Display Offset
        17  Set     F5  Display Offset
        16  Clear   F5  Display Offset
        15  Set     F4  Display Offset
        14  Clear   F4  Display Offset
        13  Set     F3  Display Offset
        12  Clear   F3  Display Offset
        11  Set     F2  Display Offset
        10  Clear   F2  Display Offset
         9  Set     F1  Display Offset
         8  Clear   F1  Display Offset
         7  Set     F0  Display Offset
         6  Clear   F0  Display Offset
         5  Set     V2  VDG Mode
         4  Clear   V2  VDG Mode
         3  Set     V1  VDG Mode
         2  Clear   V1  VDG Mode
         1  Set     V0  VDG Mode
         0  Clear   V0  VDG Mode

    The SAM installs a memory view over $0000-$FEFF of the CPU's program
    space.  There is one view entry for every combination of address
    multiplexer mode (M1/M0, and P1 on the LS785), map type, page bit and
    address dependent rate, all built at start.  RAM is addressed through
    the multiplexer as described by the host's chip layout, so mismatched
    memory size settings alias or lose RAM as they do on hardware.

    Not emulated: DRAM refresh, VDG clock synchronisation, the loss of VDG
    data in the fast rate, and the exact E/Q stretch when the address
    dependent rate changes speed.  Address dependent slow cycles are
    approximated as one extra fast cycle.

***************************************************************************/


#include "emu.h"
#include "6883sam.h"


//**************************************************************************
//  CONSTANTS
//**************************************************************************

#define LOG_FBITS   (1U << 1)
#define LOG_VBITS   (1U << 2)
#define LOG_PBITS   (1U << 3)
#define LOG_TBITS   (1U << 4)
#define LOG_MBITS   (1U << 5)
#define LOG_RBITS   (1U << 6)

#define VERBOSE (0)
// #define VERBOSE (LOG_FBITS)
// #define VERBOSE (LOG_FBITS | LOG_VBITS | LOG_PBITS | LOG_TBITS | LOG_MBITS | LOG_RBITS)

#include "logmacro.h"

#define LOGFBITS(...) LOGMASKED(LOG_FBITS, __VA_ARGS__)
#define LOGVBITS(...) LOGMASKED(LOG_VBITS, __VA_ARGS__)
#define LOGPBITS(...) LOGMASKED(LOG_PBITS, __VA_ARGS__)
#define LOGTBITS(...) LOGMASKED(LOG_TBITS, __VA_ARGS__)
#define LOGMBITS(...) LOGMASKED(LOG_MBITS, __VA_ARGS__)
#define LOGRBITS(...) LOGMASKED(LOG_RBITS, __VA_ARGS__)

DEFINE_DEVICE_TYPE(SAM6883, sam6883_device, "sam6883", "MC6883 SAM")

// row pins, first column pin, column pins, bad half bit
const sam6883_device::chip_info sam6883_device::s_chips[] =
{
	{ 6, 0, 6, -1 },    // DRAM_4K
	{ 7, 0, 7, 13 },    // DRAM_8K
	{ 7, 0, 7, -1 },    // DRAM_16K
	{ 8, 0, 8, 15 },    // DRAM_32K
	{ 8, 0, 8, -1 },    // DRAM_64K
	{ 8, 1, 6, -1 }     // DRAM_16K_X4
};



//**************************************************************************
//  DEVICE SETUP
//**************************************************************************

//-------------------------------------------------
//  constructor
//-------------------------------------------------

sam6883_device::sam6883_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SAM6883, tag, owner, clock)
	, device_memory_interface(mconfig, *this)
	, sam6883_friend_device_interface(mconfig, *this, 4)
	, m_ram(*this, finder_base::DUMMY_TAG)
	, m_ls785(false)
	, m_view(*this, "view")
	, m_ram_config("ram", ENDIANNESS_BIG, 8, 16, 0)
	, m_rom0_config("rom0", ENDIANNESS_BIG, 8, 13, 0)
	, m_rom1_config("rom1", ENDIANNESS_BIG, 8, 13, 0)
	, m_rom2_config("rom2", ENDIANNESS_BIG, 8, 14, 0)
	, m_io0_config("io0", ENDIANNESS_BIG, 8, 5, 0)
	, m_io1_config("io1", ENDIANNESS_BIG, 8, 5, 0)
	, m_io2_config("io2", ENDIANNESS_BIG, 8, 5, 0)
	, m_boot_config("boot", ENDIANNESS_BIG, 8, 7, 0)
{
}


//-------------------------------------------------
//  set_ram_layout - describe the RAM chips used
//  for a RAM size
//-------------------------------------------------

void sam6883_device::set_ram_layout(uint32_t size, ram_chip chip, uint8_t banks)
{
	m_layouts.push_back(ram_layout{ size, chip, banks });
}

sam6883_friend_device_interface::sam6883_friend_device_interface(const machine_config &mconfig, device_t &device, int divider)
	: device_interface(device, "sam6883")
	, m_cpu(device, finder_base::DUMMY_TAG)
	, m_sam_state(0x0000)
	, m_divider(divider)
	, m_video_flush_cb(device)
{
}


//-------------------------------------------------
//  memory_space_config - return the configuration
//  for the address spaces
//-------------------------------------------------

device_memory_interface::space_config_vector sam6883_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(0, &m_ram_config),
		std::make_pair(1, &m_rom0_config),
		std::make_pair(2, &m_rom1_config),
		std::make_pair(3, &m_rom2_config),
		std::make_pair(4, &m_io0_config),
		std::make_pair(5, &m_io1_config),
		std::make_pair(6, &m_io2_config),
		std::make_pair(7, &m_boot_config)
	};
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void sam6883_device::device_start()
{
	if (m_ram.found() && !m_ram->started())
		throw device_missing_dependencies();

	// get spaces
	space(0).cache(m_ram_space);
	for (int i = 0; i < 3; i++)
		space(i + 1).cache(m_rom_space[i]);
	for (int i = 0; i < 3; i++)
		space(i + 4).specific(m_io_space[i]);
	space(7).cache(m_boot_space);

	configure_ram();
	install_cpu_map();

	// the display counter is not cleared by a SAM reset
	counter_reset(0);

	// save state support
	save_item(NAME(m_sam_state));
	save_item(NAME(m_divider));
	save_item(NAME(m_counter));
	save_item(NAME(m_counter_xdiv));
	save_item(NAME(m_counter_ydiv));
	save_item(NAME(m_counter_bit4_input));
	save_item(NAME(m_counter_bit5_input));
}


//-------------------------------------------------
//  configure_ram - find the chip layout for the
//  installed RAM
//-------------------------------------------------

void sam6883_device::configure_ram()
{
	// without a ram_device, space 0 is addressed as physical RAM
	uint32_t const size = m_ram.found() ? m_ram->size() : 0x10000;
	if (m_ram.found())
		m_ram_ptr = m_ram->pointer();

	ram_layout layout{ 0, ram_chip::DRAM_64K, 0 };
	for (ram_layout const &l : m_layouts)
		if (l.size == size)
			layout = l;

	if (!layout.banks)
	{
		switch (size)
		{
		case 0x1000:  layout = ram_layout{ size, ram_chip::DRAM_4K, 1 }; break;
		case 0x2000:  layout = ram_layout{ size, ram_chip::DRAM_4K, 2 }; break;
		case 0x4000:  layout = ram_layout{ size, ram_chip::DRAM_16K, 1 }; break;
		case 0x8000:  layout = ram_layout{ size, ram_chip::DRAM_16K, 2 }; break;
		case 0x10000: layout = ram_layout{ size, ram_chip::DRAM_64K, 1 }; break;
		default:
			fatalerror("%s: no RAM layout for %u bytes\n", tag(), size);
		}
	}

	m_chip = &s_chips[unsigned(layout.chip)];
	m_ram_banks = layout.banks;
	m_ram_chip_size = 1U << (m_chip->row_bits + m_chip->col_bits - ((m_chip->dead_bit >= 0) ? 1 : 0));

	if ((m_ram_banks < 1) || (m_ram_banks > 2) || ((m_ram_banks * m_ram_chip_size) != size))
		fatalerror("%s: RAM layout of %u bank(s) of %u bytes does not match %u bytes of RAM\n", tag(), m_ram_banks, m_ram_chip_size, size);
	if ((layout.chip == ram_chip::DRAM_16K_X4) && !m_ls785)
		fatalerror("%s: 16K x 4 RAM requires an LS785\n", tag());
}


//-------------------------------------------------
//  ram_offset - pass an address through the
//  RAM address multiplexer; returns the RAM
//  offset, or -1 when no RAM chip responds
//-------------------------------------------------

int32_t sam6883_device::ram_offset(uint8_t mux, uint16_t address) const
{
	// Figure 6.  In the 4K and 16K sizes Z7 is RAS1, which stays high unless
	// the second bank is being accessed.  In the 64K sizes, bit 15 of the
	// address is the page bit for map type 0 MPU accesses.
	unsigned row, col, bank = 0;
	switch (mux)
	{
	case MUX_4K:
		bank = BIT(address, 12);
		row = (address & 0x7f) | (bank ? 0x00 : 0x80);
		col = ((address >> 6) & 0x3f) | (bank ? 0x00 : 0x80);   // Z6 is low
		break;

	case MUX_16K:
		bank = BIT(address, 14);
		row = (address & 0x7f) | (bank ? 0x00 : 0x80);
		col = ((address >> 7) & 0x7f) | (bank ? 0x00 : 0x80);
		break;

	case MUX_16K_X4:
		row = address & 0xff;
		col = (address >> 7) & 0xff;
		break;

	default:
		row = address & 0xff;
		col = address >> 8;
		break;
	}

	if (bank >= m_ram_banks)
		return -1;

	uint32_t cell = (row & ((1U << m_chip->row_bits) - 1))
			| (((col >> m_chip->col_shift) & ((1U << m_chip->col_bits) - 1)) << m_chip->row_bits);

	// half-good chips: only the lower half of the cells work
	if (m_chip->dead_bit >= 0)
	{
		if (BIT(cell, m_chip->dead_bit))
			return -1;
		cell = (cell & ((1U << m_chip->dead_bit) - 1)) | ((cell >> 1) & ~((1U << m_chip->dead_bit) - 1));
	}

	return (bank * m_ram_chip_size) + cell;
}


//-------------------------------------------------
//  ram_read/ram_write - access RAM by offset
//-------------------------------------------------

uint8_t sam6883_device::ram_read(int32_t offset)
{
	if (offset < 0)
		return space(0).unmap();
	return m_ram_ptr ? m_ram_ptr[offset] : m_ram_space.read_byte(offset);
}


void sam6883_device::ram_write(int32_t offset, uint8_t data)
{
	if (offset < 0)
		return;
	if (m_ram_ptr)
		m_ram_ptr[offset] = data;
	else
		m_ram_space.write_byte(offset, data);
}


//-------------------------------------------------
//  current_mux - address multiplexer mode for the
//  current SAM state
//-------------------------------------------------

uint8_t sam6883_device::current_mux() const
{
	if (m_sam_state & SAM_STATE_M1)
		return MUX_64K;
	else if (m_sam_state & SAM_STATE_M0)
		return (m_ls785 && (m_sam_state & SAM_STATE_P1)) ? MUX_16K_X4 : MUX_16K;
	else
		return MUX_4K;
}


//-------------------------------------------------
//  install_cpu_map - set up the CPU program space
//-------------------------------------------------

void sam6883_device::install_cpu_map()
{
	address_space &space = m_cpu->space(AS_PROGRAM);

	// $FF00-$FFFF decodes the same way in both map types
	space.install_readwrite_handler(0xff00, 0xff1f,
			read8sm_delegate(*this, NAME([this] (offs_t offset) { if (m_rate_ad) slow_cycle(); return m_io_space[0].read_byte(offset); })),
			write8sm_delegate(*this, NAME([this] (offs_t offset, uint8_t data) { if (m_rate_ad) slow_cycle(); m_io_space[0].write_byte(offset, data); })));
	space.install_readwrite_handler(0xff20, 0xff3f,
			read8sm_delegate(*this, NAME([this] (offs_t offset) { return m_io_space[1].read_byte(offset); })),
			write8sm_delegate(*this, NAME([this] (offs_t offset, uint8_t data) { m_io_space[1].write_byte(offset, data); })));
	space.install_readwrite_handler(0xff40, 0xff5f,
			read8sm_delegate(*this, NAME([this] (offs_t offset) { return m_io_space[2].read_byte(offset); })),
			write8sm_delegate(*this, NAME([this] (offs_t offset, uint8_t data) { m_io_space[2].write_byte(offset, data); })));
	space.install_readwrite_handler(0xff60, 0xffdf,
			read8sm_delegate(*this, NAME([this] (offs_t offset) { return m_boot_space.read_byte(offset); })),
			write8sm_delegate(*this, NAME([this] (offs_t offset, uint8_t data)
			{
				m_boot_space.write_byte(offset, data);
				if (offset >= 0x60)
					internal_write(offset & 0x1f, data);
			})));
	space.install_readwrite_handler(0xffe0, 0xffff,
			read8sm_delegate(*this, NAME([this] (offs_t offset) { return m_rom_space[1].read_byte(offset + 0x1fe0); })),
			write8sm_delegate(*this, NAME([this] (offs_t offset, uint8_t data) { m_rom_space[1].write_byte(offset + 0x1fe0, data); })));

	// $0000-$FEFF depends on the SAM state
	space.install_view(0x0000, 0xfeff, m_view);
	int count = 0;
	for (uint8_t mux = 0; mux < MUX_COUNT; mux++)
	{
		if ((mux == MUX_16K_X4) && !m_ls785)
			continue;
		for (int ty = 0; ty < 2; ty++)
		{
			for (int p1 = 0; p1 < 2; p1++)
			{
				for (int ad = 0; ad < 2; ad++)
				{
					// the page bit only reaches the multiplexer in the 64K sizes with map type 0
					if (p1 && ((mux != MUX_64K) || ty))
					{
						m_view_index[mux][ty][p1][ad] = m_view_index[mux][ty][0][ad];
					}
					else
					{
						m_view_index[mux][ty][p1][ad] = count;
						install_view(count++, mux, ty, p1, ad);
					}
				}
			}
		}
	}
}


//-------------------------------------------------
//  install_view - populate one view entry
//-------------------------------------------------

void sam6883_device::install_view(int index, uint8_t mux, bool ty, bool p1, bool rate_ad)
{
	memory_view::memory_view_entry &entry = m_view[index];
	m_view_ty[index] = ty;
	m_view_count = std::max(m_view_count, index + 1);

	// $0000-$7FFF is RAM in both map types (S=0 reading, S=7 writing)
	install_ram_region(entry, 0x0000, 0x7fff, mux, (!ty && p1) ? 0x8000 : 0x0000, true);

	if (!ty)
	{
		// map type 0: ROM0, ROM1 and ROM2 (one handler, so opcode fetches
		// moving between the ROMs do not refill the CPU's cache)
		entry.install_readwrite_handler(0x8000, 0xfeff,
				read8sm_delegate(*this, NAME([this] (offs_t offset)
				{
					return (offset < 0x4000)
							? m_rom_space[BIT(offset, 13)].read_byte(offset & 0x1fff)
							: m_rom_space[2].read_byte(offset & 0x3fff);
				})),
				write8sm_delegate(*this, NAME([this] (offs_t offset, uint8_t data)
				{
					if (offset < 0x4000)
						m_rom_space[BIT(offset, 13)].write_byte(offset & 0x1fff, data);
					else
						m_rom_space[2].write_byte(offset & 0x3fff, data);
				})));
	}
	else
	{
		// map type 1: RAM is read (S=0), writes also select ROM0, ROM1 or ROM2
		install_ram_region(entry, 0x8000, 0xfeff, mux, 0x0000, false);
		entry.install_write_handler(0x8000, 0xfeff, write8sm_delegate(*this, [this, mux] (offs_t offset, uint8_t data)
		{
			offs_t const address = offset + 0x8000;
			ram_write(ram_offset(mux, address), data);
			if (address < 0xc000)
				m_rom_space[BIT(address, 13)].write_byte(address & 0x1fff, data);
			else
				m_rom_space[2].write_byte(address & 0x3fff, data);
		}, "ty1_write"));
	}

	// address dependent rate: $0000-$7FFF is accessed at the slow rate
	if (rate_ad)
	{
		entry.install_readwrite_tap(0x0000, 0x7fff, "sam_rate",
				[this] (offs_t offset, uint8_t &data, uint8_t mem_mask) { slow_cycle(); },
				[this] (offs_t offset, uint8_t &data, uint8_t mem_mask) { slow_cycle(); });
	}
}


//-------------------------------------------------
//  install_ram_region - map CPU addresses to RAM
//  through the address multiplexer
//-------------------------------------------------

void sam6883_device::install_ram_region(memory_view::memory_view_entry &entry, offs_t start, offs_t end, uint8_t mux, uint16_t page, bool writable)
{
	if (!m_ram_ptr)
	{
		// RAM in address space 0: translate every access
		entry.install_read_handler(start, end, read8sm_delegate(*this, [this, start, mux, page] (offs_t offset)
		{
			return ram_read(ram_offset(mux, (start + offset) | page));
		}, "ram_read"));
		if (writable)
		{
			entry.install_write_handler(start, end, write8sm_delegate(*this, [this, start, mux, page] (offs_t offset, uint8_t data)
			{
				ram_write(ram_offset(mux, (start + offset) | page), data);
			}, "ram_write"));
		}
		return;
	}

	// RAM from a ram_device: install each run of linearly addressed cells.
	// Z0-Z5 always carry A0-A5 on the row, so runs are whole 64 byte blocks.
	for (offs_t base = start; base <= end; )
	{
		int32_t const offset = ram_offset(mux, base | page);
		offs_t last = base + 0x3f;
		while (last < end)
		{
			int32_t const next = ram_offset(mux, (last + 1) | page);
			if ((offset < 0) ? (next >= 0) : (next != (offset + int32_t(last + 1 - base))))
				break;
			last += 0x40;
		}

		if (offset < 0)
		{
			if (writable)
				entry.nop_readwrite(base, last);
			else
				entry.nop_read(base, last);
		}
		else if (writable)
		{
			entry.install_ram(base, last, m_ram_ptr + offset);
		}
		else
		{
			entry.install_rom(base, last, m_ram_ptr + offset);
		}
		base = last + 1;
	}
}


//-------------------------------------------------
//  s_decode - S2-S0 for an MPU access (Figures
//  14-17)
//-------------------------------------------------

uint8_t sam6883_device::s_decode(bool ty, offs_t address, bool write)
{
	if (address < 0x8000)
		return write ? 7 : 0;                   // RAM
	else if (address < 0xff00)
	{
		if (ty && !write)
			return 0;                           // map type 1 RAM
		return (address < 0xa000) ? 1 : (address < 0xc000) ? 2 : 3;
	}
	else if (address < 0xff60)
		return 4 + BIT(address, 5, 2);          // I/O0-I/O2
	else if (address < 0xffe0)
		return 7;                               // including the control register
	else
		return 2;                               // vectors from ROM1
}


//-------------------------------------------------
//  install_s_tap - tap every MPU access that
//  selects one S2-S0 value
//-------------------------------------------------

void sam6883_device::install_s_tap(uint8_t s, std::string name, tap_func tapr, tap_func tapw)
{
	// S2-S0 is constant over each of these ranges for a given map type
	static constexpr std::pair<offs_t, offs_t> ranges[] =
	{
		{ 0x0000, 0x7fff }, { 0x8000, 0x9fff }, { 0xa000, 0xbfff }, { 0xc000, 0xfeff },
		{ 0xff00, 0xff1f }, { 0xff20, 0xff3f }, { 0xff40, 0xff5f }, { 0xff60, 0xffdf }, { 0xffe0, 0xffff }
	};

	if (!started())
		throw emu_fatalerror("%s: install_s_tap called before the SAM has started\n", tag());

	auto install = [&] (address_space_installer &installer, bool ty, offs_t start, offs_t end)
	{
		if (tapr && (s_decode(ty, start, false) == s))
			installer.install_read_tap(start, end, name, tapr);
		if (tapw && (s_decode(ty, start, true) == s))
			installer.install_write_tap(start, end, name, tapw);
	};

	for (auto const &[start, end] : ranges)
	{
		if (start < 0xff00)
		{
			for (int i = 0; i < m_view_count; i++)
				install(m_view[i], m_view_ty[i], start, end);
		}
		else
		{
			install(m_cpu->space(AS_PROGRAM), false, start, end);
		}
	}
}


//-------------------------------------------------
//  slow_cycle - stretch an access in the address
//  dependent rate from a fast cycle to a slow one
//-------------------------------------------------

void sam6883_device::slow_cycle()
{
	m_cpu->adjust_icount(-1);
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void sam6883_device::device_reset()
{
	// VClk held low clears the control register, nothing else
	m_sam_state = 0x0000;
	update_state();
}



//-------------------------------------------------
//  device_post_load - device-specific post load
//-------------------------------------------------

void sam6883_device::device_post_load()
{
	device_t::device_post_load();
	update_state();
}



//-------------------------------------------------
//  update_state
//-------------------------------------------------

void sam6883_device::update_state()
{
	update_memory();
	update_cpu_clock();
}



//-------------------------------------------------
//  update_memory - select the view for the
//  memory size, map type, page and rate bits
//-------------------------------------------------

void sam6883_device::update_memory()
{
	m_mux = current_mux();
	m_rate_ad = (m_sam_state & (SAM_STATE_R1 | SAM_STATE_R0)) == SAM_STATE_R0;
	m_view.select(m_view_index[m_mux][BIT(m_sam_state, 15)][BIT(m_sam_state, 10)][m_rate_ad ? 1 : 0]);
}



//-------------------------------------------------
//  update_cpu_clock - adjusts the speed of the CPU
//  clock
//-------------------------------------------------

void sam6883_friend_device_interface::update_cpu_clock()
{
	// The infamous speed up poke.
	//
	// This was a SAM switch that occupied 4 addresses:
	//
	//      $FFD9   (set)   R1
	//      $FFD8   (clear) R1
	//      $FFD7   (set)   R0
	//      $FFD6   (clear) R0
	//
	// R1:R0 formed the following states:
	//      00  - slow          0.89 MHz
	//      01  - dual speed    ???
	//      1x  - fast          1.78 MHz
	//
	// R1 controlled whether the video addressing was speeded up and R0
	// did the same for the CPU.  On pre-CoCo 3 machines, setting R1 caused
	// the screen to display garbage because the M6847 could not display
	// fast enough.
	//
	// In dual speed the CPU runs at the fast rate here; the MC6883 device
	// stretches its slow accesses itself.  The GIME does not.

	int speed = (m_sam_state & (SAM_STATE_R1|SAM_STATE_R0)) / SAM_STATE_R0;
	m_cpu->owner()->set_unscaled_clock(device().clock() / (m_divider * (speed ? 2 : 4)));
}



//-------------------------------------------------
//  internal_write
//-------------------------------------------------

void sam6883_device::internal_write(offs_t offset, uint8_t data)
{
	// data is ignored
	(void)data;

	// Record elapsed video with the old V bits, even for same-value writes.
	if (offset < 6)
		m_video_flush_cb(1);

	// alter the SAM state
	uint8_t old_mode = m_sam_state & 7;
	uint16_t xorval = alter_sam_state(offset);
	if (xorval & (SAM_STATE_V0|SAM_STATE_V1|SAM_STATE_V2))
		counter_mode_changed(old_mode);

	// based on the mask, apply effects
	if (xorval & (SAM_STATE_TY|SAM_STATE_M1|SAM_STATE_M0|SAM_STATE_P1|SAM_STATE_R1|SAM_STATE_R0))
		update_memory();
	if (xorval & (SAM_STATE_R1|SAM_STATE_R0))
		update_cpu_clock();

	if (xorval & (SAM_STATE_F6|SAM_STATE_F5|SAM_STATE_F4|SAM_STATE_F3|SAM_STATE_F2|SAM_STATE_F1|SAM_STATE_F0))
	{
		LOGFBITS("%s: SAM F Address: $%04x\n",
			machine().describe_context(),
			display_offset());
	}

	if (xorval & (SAM_STATE_V0|SAM_STATE_V1|SAM_STATE_V2))
	{
		LOGVBITS("%s: SAM V Bits: $%02x\n",
			machine().describe_context(),
			(m_sam_state & (SAM_STATE_V0|SAM_STATE_V1|SAM_STATE_V2)));
	}

	if (xorval & (SAM_STATE_P1))
	{
		LOGPBITS("%s: SAM P1 Bit: $%02x\n",
			machine().describe_context(),
			(m_sam_state & (SAM_STATE_P1)) >> 10);
	}

	if (xorval & (SAM_STATE_TY))
	{
		LOGTBITS("%s: SAM TY Bit: $%02x\n",
			machine().describe_context(),
			(m_sam_state & (SAM_STATE_TY)) >> 15);
	}

	if (xorval & (SAM_STATE_M0|SAM_STATE_M1))
	{
		LOGMBITS("%s: SAM M Bits: $%02x\n",
			machine().describe_context(),
			(m_sam_state & (SAM_STATE_M0|SAM_STATE_M1)) >> 13);
	}

	if (xorval & (SAM_STATE_R0|SAM_STATE_R1))
	{
		LOGRBITS("%s: SAM R Bits: $%02x\n",
			machine().describe_context(),
			(m_sam_state & (SAM_STATE_R0|SAM_STATE_R1)) >> 11);
	}
}



//-------------------------------------------------
//  display counter divider network
//-------------------------------------------------

// The dividers run even when their outputs are not selected. Mode writes
// reconnect clock inputs, so a high-to-low transition can advance the address
// immediately. See the hardware investigations by Ciaran Anscomb and Stewart
// Orchard: https://www.6809.org.uk/dev/tm002/ (and its linked counter model).

void sam6883_device::counter_reset(uint16_t address)
{
	m_counter = address;
	m_counter_xdiv[0] = m_counter_xdiv[1] = 0;
	m_counter_ydiv[0] = m_counter_ydiv[1] = m_counter_ydiv[2] = 0;
	m_counter_bit4_input = false;
	m_counter_bit5_input = false;
}


bool sam6883_device::counter_x_output() const
{
	switch (m_sam_state & 7)
	{
	case 1: return BIT(m_counter_xdiv[1], 1); // /3
	case 3: return BIT(m_counter_xdiv[0], 0); // /2
	default: return BIT(m_counter, 3);
	}
}


bool sam6883_device::counter_y_output() const
{
	switch (m_sam_state & 7)
	{
	case 0: return BIT(m_counter_ydiv[2], 1); // /3 followed by /4
	case 2: return BIT(m_counter_ydiv[1], 1); // /3
	case 4: return BIT(m_counter_ydiv[0], 0); // /2
	default: return BIT(m_counter, 4);
	}
}


void sam6883_device::counter_update_bit3(bool falling)
{
	if (falling)
	{
		m_counter_xdiv[0] ^= 1;
		m_counter_xdiv[1] = (m_counter_xdiv[1] + 1) % 3;
	}
	counter_set_bit4_input(counter_x_output());
}


void sam6883_device::counter_update_bit4(bool falling)
{
	if (falling)
	{
		m_counter_ydiv[0] ^= 1;
		bool old_div3_output = BIT(m_counter_ydiv[1], 1);
		m_counter_ydiv[1] = (m_counter_ydiv[1] + 1) % 3;
		if (old_div3_output && !BIT(m_counter_ydiv[1], 1))
			m_counter_ydiv[2] = (m_counter_ydiv[2] + 1) & 3;
	}
	counter_set_bit5_input(counter_y_output());
}


void sam6883_device::counter_set_bit4_input(bool level)
{
	bool falling = m_counter_bit4_input && !level;
	m_counter_bit4_input = level;
	if (falling)
	{
		m_counter ^= 0x0010;
		counter_update_bit4(!BIT(m_counter, 4));
	}
}


void sam6883_device::counter_set_bit5_input(bool level)
{
	bool falling = m_counter_bit5_input && !level;
	m_counter_bit5_input = level;
	if (falling)
		m_counter += 0x0020;
}


void sam6883_device::counter_mode_changed(uint8_t old_mode)
{
	static constexpr uint8_t x_divisors[8] = { 1, 3, 1, 2, 1, 1, 1, 1 };
	static constexpr uint8_t y_divisors[8] = { 12, 1, 3, 1, 2, 1, 1, 1 };
	uint8_t new_mode = m_sam_state & 7;
	uint8_t old_y = y_divisors[old_mode];
	uint8_t new_y = y_divisors[new_mode];
	uint8_t old_x = x_divisors[old_mode];
	uint8_t new_x = x_divisors[new_mode];

	// The Y selector settles first. Switching between /12 and /3 briefly
	// selects low; switching between /12 and /2 briefly selects raw B4.
	if (old_y != new_y)
	{
		if ((old_y == 12 && new_y == 3) || (old_y == 3 && new_y == 12))
			counter_set_bit5_input(false);
		else if ((old_y == 12 && new_y == 2) || (old_y == 2 && new_y == 12))
			counter_set_bit5_input(BIT(m_counter, 4));
		counter_set_bit5_input(counter_y_output());
	}

	// Changing between the two divided X clocks briefly selects low.
	if (old_x != new_x)
	{
		if ((old_x == 2 && new_x == 3) || (old_x == 3 && new_x == 2))
			counter_set_bit4_input(false);
		counter_set_bit4_input(counter_x_output());
	}
}


//-------------------------------------------------
//  horizontal_sync
//-------------------------------------------------

void sam6883_device::horizontal_sync()
{
	// When horizontal sync occurs, bits B1-B3 or B1-B4 may be cleared (except in DMA mode).  The catch
	// is that the SAM's counter is a chain of flip-flops.  Clearing the counter can cause carries to
	// occur just as they can when the counter is bumped.
	//
	// This is critical in getting certain semigraphics modes to work correctly.  Guardian uses this
	// mode (see bug #1153).  Special thanks to Ciaran Anscomb and Phill Harvey-Smith for figuring this
	// out
	switch((m_sam_state & (SAM_STATE_V2|SAM_STATE_V1|SAM_STATE_V0)) / SAM_STATE_V0)
	{
		case 0x01:
		case 0x03:
		case 0x05:
			// these SAM modes clear bits B1-B3
		{
			bool falling = BIT(m_counter, 3);
			m_counter &= ~0x000F;
			counter_update_bit3(falling);
			break;
		}

		case 0x00:
		case 0x02:
		case 0x04:
		case 0x06:
			// clear bits B1-B4
		{
			bool falling = BIT(m_counter, 4);
			m_counter &= ~0x001F;
			// B3 clearing is not clocked through X in these modes.
			m_counter_bit4_input = false;
			counter_update_bit4(falling);
			break;
		}

		case 0x07:
			// DMA mode - do nothing
			break;

		default:
			fatalerror("Should not get here\n");
	}
}



//-------------------------------------------------
//  hs_w
//-------------------------------------------------

void sam6883_device::hs_w(int state)
{
	if (state)
	{
		horizontal_sync();
	}
}
