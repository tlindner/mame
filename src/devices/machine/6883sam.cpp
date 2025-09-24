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
        21  Set     P1  Page #1             0: Normal      1: A15 = 1
        20  Clear   P1  Page #1
        19  Set     F6  Display Offset      Display Address =
        18  Clear   F6  Display Offset          Display Offset << 9
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
         5  Set     V2  VDG Mode            Video buffer size =
         4  Clear   V2  VDG Mode            000: 512 bytes    001: 1024 bytes
         3  Set     V1  VDG Mode            010: 2048 bytes   011: 1536 bytes
         2  Clear   V1  VDG Mode            100: 3072 bytes   101: 3072 bytes
         1  Set     V0  VDG Mode            110: 6144 bytes   111: not used
         0  Clear   V0  VDG Mode

	Host Memory Layout (top layers overide bottom layers):

	SAM Handler:                                    ≤-------->
	I/O View[0]:                           ≤-------->
	ROM View[0]:                ≤---------->
	RAM View[0]:  ≤--4k/8k------+----------+--------+--------+----*----≥
	RAM View[1]:  ≤--16k/32k----+----------+--------+--------+----*----≥
	RAM View[2]:  ≤--32k/64k----+----------+--------+--------+----*----≥
	RAM View[3]:  ≤--64k, P1=1--+----------+--------+--------+----*----≥
	ENDC Handler: ≤-------------+----------+--------+--------+---------≥
	             $0000         $8000      $FF00    $FFC0    $FFE0     $FFFF

    * 32 byte ROM mirror

	ENDC is a signal first described in the MC6883 SAM datasheet.
	It inhibits the 74LS138 decoding of the three S (select) lines.
***************************************************************************/


#include "emu.h"
#include "6883sam.h"

#include <algorithm>

bool sam_misconfigured( int index, u32 ram_size )
{
	if (index==0 && ram_size == 4096) return false;
	if (index==0 && ram_size == 8192) return false;
	if (index==1 && ram_size == 16384) return false;
	if (index==1 && ram_size == 32768) return false;
	if (index==2 && ram_size == 32768) return false;
	if (index==2 && ram_size == 65536) return false;
	if (index==3 && ram_size == 65536) return false;

	return true;
}

//**************************************************************************
//  CONSTANTS
//**************************************************************************

#define LOG_FBITS   (1U << 1)
#define LOG_VBITS   (1U << 2)
#define LOG_PBITS   (1U << 3)
#define LOG_TBITS   (1U << 4)
#define LOG_MBITS   (1U << 5)
#define LOG_RBITS   (1U << 6)

// #define VERBOSE (0)
#define VERBOSE (LOG_MBITS)
// #define VERBOSE (LOG_FBITS | LOG_VBITS | LOG_PBITS | LOG_TBITS | LOG_MBITS | LOG_RBITS)

#include "logmacro.h"

#define LOGFBITS(...) LOGMASKED(LOG_FBITS, __VA_ARGS__)
#define LOGVBITS(...) LOGMASKED(LOG_VBITS, __VA_ARGS__)
#define LOGPBITS(...) LOGMASKED(LOG_PBITS, __VA_ARGS__)
#define LOGTBITS(...) LOGMASKED(LOG_TBITS, __VA_ARGS__)
#define LOGMBITS(...) LOGMASKED(LOG_MBITS, __VA_ARGS__)
#define LOGRBITS(...) LOGMASKED(LOG_RBITS, __VA_ARGS__)

DEFINE_DEVICE_TYPE(SAM6883, sam6883_device, "sam6883", "MC6883 SAM")



//**************************************************************************
//  DEVICE SETUP
//**************************************************************************

//-------------------------------------------------
//  constructor
//-------------------------------------------------

sam6883_friend_device_interface::sam6883_friend_device_interface(const machine_config &mconfig, device_t &device, int divider)
	: device_interface(device, "sam6883")
	, m_cpu(device, finder_base::DUMMY_TAG)
	, m_ram(device, finder_base::DUMMY_TAG)
	, m_sam_state(0x0000)
	, m_endc(0)
	, m_divider(divider)
	, m_host(dynamic_cast<device_sam_map_host_interface *>(device.owner()))
{
}

sam6883_device::sam6883_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SAM6883, tag, owner, clock)
	, device_memory_interface(mconfig, *this)
	, sam6883_friend_device_interface(mconfig, *this, 4)
	, m_ram_view(*this, "sam_ram_view")
	, m_rom_view(*this, "sam_rom_view")
	, m_io_view(*this, "sam_io_view")
	, m_m0_config("ram0", ENDIANNESS_BIG, 8, 16, 0)
	, m_m1_config("ram1", ENDIANNESS_BIG, 8, 16, 0)
	, m_m2_config("ram2", ENDIANNESS_BIG, 8, 16, 0)
	, m_m3_config("ram3", ENDIANNESS_BIG, 8, 16, 0)
	, m_s2_config("rom0", ENDIANNESS_BIG, 8, 13, 0)
{
}



//-------------------------------------------------
//  function - description
//-------------------------------------------------

void sam6883_device::sam_mem(address_map &map)
{
	map(0x0000, 0xffff).rw(FUNC(sam6883_device::endc_read), FUNC(sam6883_device::endc_write));
	map(0x0000, 0xffff).view(m_ram_view); // see device_start()
	map(0x8000, 0xfeff).view(m_rom_view);

	m_rom_view[0](0x8000, 0x9fff).m(*m_host, FUNC(device_sam_map_host_interface::s1_rom0_map));
	m_rom_view[0](0xa000, 0xbfff).m(*m_host, FUNC(device_sam_map_host_interface::s2_rom1_map));
	m_rom_view[0](0xc000, 0xfeff).m(*m_host, FUNC(device_sam_map_host_interface::s3_rom2_map));

	// This intentionally cuts a gap in the ROM view
	map(0xff00, 0xffbf).view(m_io_view);
	fprintf(stderr, "m_io_view: %p\n", &m_io_view);
	m_io_view[0](0xff00, 0xff1f).m(*m_host, FUNC(device_sam_map_host_interface::s4_io0_map));
	m_io_view[0](0xff20, 0xff3f).m(*m_host, FUNC(device_sam_map_host_interface::s5_io1_map));
	m_io_view[0](0xff40, 0xff5f).m(*m_host, FUNC(device_sam_map_host_interface::s6_io2_map));
	m_io_view[0](0xff60, 0xffbf).m(*m_host, FUNC(device_sam_map_host_interface::s7_res_map));

	// This intentionally cuts a gap in the ROM view and endc
	map(0xffc0, 0xffdf).w(FUNC(sam6883_device::internal_write)).nopr();
}



//-------------------------------------------------
//  internal_rom_map - map we use for ROM mirror
//-------------------------------------------------

void sam6883_device::internal_rom_map(address_map &map)
{
	map(0x0000, 0x1fff).m(*m_host, FUNC(device_sam_map_host_interface::s2_rom1_map));
}



//-------------------------------------------------
//  device_add_mconfig - additional config
//-------------------------------------------------

void sam6883_device::device_add_mconfig(machine_config &config)
{
	set_addrmap(4, &sam6883_device::internal_rom_map);
}



//-------------------------------------------------
//  memory_space_config - return the configuration
//  for the address spaces
//-------------------------------------------------

device_memory_interface::space_config_vector sam6883_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(0, &m_m0_config),
		std::make_pair(1, &m_m1_config),
		std::make_pair(2, &m_m2_config),
		std::make_pair(3, &m_m3_config),
		std::make_pair(4, &m_s2_config)
	};
}



//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void sam6883_device::device_start()
{
	if (!m_ram->started())
		throw device_missing_dependencies();

	if (!m_cpu->started())
		throw device_missing_dependencies();

	std::list<u32> supported_configurations = {4096, 8192, 16384, 32768, 65536};
	auto it = std::find(supported_configurations.begin(), supported_configurations.end(), m_ram->size());

	if (it == supported_configurations.end())
		throw emu_fatalerror("MC6883 only supports RAM configurations of 4096, 8192, 16384, 32768, or 65536 bytes.");

#if 0
	// setup ram views
	for (int i = 0; i<4; i++)
	{
		if (sam_misconfigured( i, m_ram->size()))
		{
			// misconfigured ram
			u32 start = 0, end = 0xff;
			for (int j = 0; j < (m_ram->size()/0x200); j++)
			{
				if (start<0xfe00)
				{
					m_ram_view[i].install_ram(start, std::min(end, 0xfeffU), 0x0100, m_ram->pointer()+start);
				}
				else
				{
					m_ram_view[i].install_ram(start, std::min(end, 0xfeffU), m_ram->pointer()+start);
				}

				space(i).install_ram(start, end, 0x0100, m_ram->pointer()+start);
				start += 0x200;
				end += 0x200;
			}

			if (end < 0xfeff)
			{
				m_ram_view[i].nop_readwrite(end+1-0x100, 0xfeff);
			}
		}
		else
		{
			if (i == 3)
			{
				// 64k of ram with P1 set
				m_ram_view[i].install_ram(0x0000, 0x7fff, m_ram->pointer()+0x8000);
				m_ram_view[i].install_ram(0x8000, 0xfeff, m_ram->pointer()+0x8000);
				space(i).install_ram(0x0000, 0xffff, m_ram->pointer());
			}
			else
			{
				// All other properly configured ram
				m_ram_view[i].install_ram(0x0000, std::min(m_ram->mask(), 0xfeffU), m_ram->pointer());
				space(i).install_ram(0x0000, m_ram->mask(), m_ram->pointer());
				if(m_ram->mask() < 0xffff)
				{
					m_ram_view[i].nop_readwrite(m_ram->size(), 0xfeff);
					space(i).nop_readwrite(m_ram->size(), 0xffff);
				}
			}
		}

		m_ram_view[i].install_device(0x0000, 0xfeff, *m_host, &device_sam_map_host_interface::s0_ram_map);
		m_ram_view[i].install_read_handler(0xffe0, 0xffff, emu::rw_delegate(*this, FUNC(sam6883_device::vector_read)));
		m_ram_view[i].nop_write(0xffe0, 0xffff);
	}
#else
	// this is used to install 64k RAMs in all modes, for testing
	m_ram_view[0].install_ram(0x0000, 0xfeff, m_ram->pointer());
	m_ram_view[0].install_read_handler(0xffe0, 0xffff, emu::rw_delegate(*this, FUNC(sam6883_device::vector_read)));
	m_ram_view[0].nop_write(0xffe0, 0xffff);
	m_ram_view[1].install_ram(0x0000, 0xfeff, m_ram->pointer());
	m_ram_view[1].install_read_handler(0xffe0, 0xffff, emu::rw_delegate(*this, FUNC(sam6883_device::vector_read)));
	m_ram_view[1].nop_write(0xffe0, 0xffff);
	m_ram_view[2].install_ram(0x0000, 0xfeff, m_ram->pointer());
	m_ram_view[2].install_read_handler(0xffe0, 0xffff, emu::rw_delegate(*this, FUNC(sam6883_device::vector_read)));
	m_ram_view[2].nop_write(0xffe0, 0xffff);
	m_ram_view[3].install_ram(0x0000, 0xfeff, m_ram->pointer());
	m_ram_view[3].install_read_handler(0xffe0, 0xffff, emu::rw_delegate(*this, FUNC(sam6883_device::vector_read)));
	m_ram_view[3].nop_write(0xffe0, 0xffff);

	space(0).install_ram(0x0000, 0xffff, m_ram->pointer());
	space(1).install_ram(0x0000, 0xffff, m_ram->pointer());
	space(2).install_ram(0x0000, 0xffff, m_ram->pointer());
	space(3).install_ram(0x0000, 0xffff, m_ram->pointer());
#endif

	// get spaces
	space(0).cache(m_ram_space[0]);
	space(1).cache(m_ram_space[1]);
	space(2).cache(m_ram_space[2]);
	space(3).cache(m_ram_space[3]);
	space(4).cache(m_rom_space);

	// save state support
	save_item(NAME(m_sam_state));
	save_item(NAME(m_divider));
	save_item(NAME(m_counter));
	save_item(NAME(m_counter_xdiv));
	save_item(NAME(m_counter_ydiv));
}



//-------------------------------------------------
//  endc_read - temporary endc read handler
//-------------------------------------------------

uint8_t sam6883_device::endc_read(offs_t offset)
{
	if (!machine().side_effects_disabled())
		fprintf(stderr,"%s endc_read: %4x\n", machine().describe_context().c_str(), offset);
	return 0;
}



//-------------------------------------------------
//  endc_write - temporary endc write handler
//-------------------------------------------------

void sam6883_device::endc_write(offs_t offset, uint8_t data)
{
	if (!machine().side_effects_disabled())
		fprintf(stderr,"%s endc_write: %4x\n", machine().describe_context().c_str(), offset);
}



//-------------------------------------------------
//  vector_read - vector ROM mirror in RAM view
//-------------------------------------------------

uint8_t sam6883_device::vector_read(offs_t offset)
{
	return m_rom_space.read_byte(offset+0x1fe0);
}



//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void sam6883_device::device_reset()
{
	sam6883_friend_device_interface::device_reset();
	m_counter = 0;
	m_counter_xdiv = 0;
	m_counter_ydiv = 0;
	m_sam_state = 0x0000;
	update_state();
}



//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void sam6883_friend_device_interface::device_reset()
{
	m_endc = 0;
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
//  update_memory
//-------------------------------------------------

void sam6883_device::update_memory()
{
	// Memory size - allowed restricting memory accesses to something less than
	// 32k
	//
	// This was a SAM switch that occupied 4 addresses:
	//
	//      $FFDD   (set)   R1
	//      $FFDC   (clear) R1
	//      $FFDB   (set)   R0
	//      $FFDA   (clear) R0
	//
	// R1:R0 formed the following states:
	//      00  - 4k dynamic
	//      01  - 16k dynamic
	//      10  - 64k dynamic
	//      11  - 64k static

	// switch depending on the M1/M0 variables
	switch(BIT(m_sam_state, SAM_BIT_M0, 2))
	{
		case 0:
			// 4K mode
			m_ram_view.select(0);
			break;

		case SAM_STATE_M0>>SAM_BIT_M0:
			// 16K mode
			m_ram_view.select(1);
			break;

		case SAM_STATE_M1>>SAM_BIT_M0:
			// 64k mode (dynamic)
		case (SAM_STATE_M1|SAM_STATE_M0)>>SAM_BIT_M0:
			// 64k mode (static)
			// full 64k RAM or ROM/RAM
			// CoCo Max requires these two be treated the same

			m_ram_view.select(2);

			if (BIT(m_sam_state, SAM_BIT_P1))
				m_ram_view.select(3);
			break;
	}

	if(BIT(m_sam_state, SAM_BIT_TY))
		m_rom_view.disable();
	else
		m_rom_view.select(0);

	m_io_view.select(0);

	if (m_endc)
	{
		m_ram_view.disable();
		m_rom_view.disable();
		m_io_view.disable();
	}
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
	//      00  - slow               0.89 MHz
	//      01  - address dependent; RAM: 0.89 MHz, ROM: 1.78 MHz
	//      1x  - fast               1.78 MHz
	//
	// R1 controlled whether the video addressing was speeded up and R0
	// did the same for the CPU.  On pre-CoCo 3 machines, setting R1 caused
	// the screen to display garbage because the M6847 could not display
	// fast enough.
	//
	// TODO:  In dual speed, ROM was a fast access but RAM was not.
	// I don't know how to simulate this.

	int speed = (BIT(m_sam_state, SAM_BIT_R0, 2));
	m_cpu->owner()->set_unscaled_clock(device().clock() / (m_divider * (speed ? 2 : 4)));
}



//-------------------------------------------------
//  internal_write
//-------------------------------------------------

void sam6883_device::internal_write(offs_t offset, uint8_t data)
{
	// data is ignored
	(void)data;

	// alter the SAM state
	uint16_t xorval = alter_sam_state(offset);

	// based on the mask, apply effects
	if (xorval & (SAM_STATE_TY|SAM_STATE_M1|SAM_STATE_M0|SAM_STATE_P1))
	{
		update_memory();
	}

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
			BIT(m_sam_state, SAM_BIT_V0, 3));
	}

	if (xorval & (SAM_STATE_P1))
	{
		LOGPBITS("%s: SAM P1 Bit: $%02x\n",
			machine().describe_context(),
			BIT(m_sam_state, SAM_BIT_P1));
	}

	if (xorval & (SAM_STATE_TY))
	{
		LOGTBITS("%s: SAM TY Bit: $%02x\n",
			machine().describe_context(),
			BIT(m_sam_state, SAM_BIT_TY));
	}

	if (xorval & (SAM_STATE_M0|SAM_STATE_M1))
	{
		LOGMBITS("%s: SAM M Bits: $%02x\n",
			machine().describe_context(),
			BIT(m_sam_state, SAM_BIT_M0, 2));
	}

	if (xorval & (SAM_STATE_R0|SAM_STATE_R1))
	{
		LOGRBITS("%s: SAM R Bits: $%02x\n",
			machine().describe_context(),
			BIT(m_sam_state, SAM_BIT_R0, 2));
	}
}



//-------------------------------------------------
//  horizontal_sync
//-------------------------------------------------

void sam6883_device::horizontal_sync()
{
	bool carry;

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
			carry = (m_counter & 0x0008) ? true : false;
			m_counter &= ~0x000F;
			if (carry)
				counter_carry_bit3();
			break;

		case 0x00:
		case 0x02:
		case 0x04:
		case 0x06:
			// clear bits B1-B4
			carry = (m_counter & 0x0010) ? true : false;
			m_counter &= ~0x001F;
			if (carry)
				counter_carry_bit4();
			break;

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



//-------------------------------------------------
//  endc_w
//-------------------------------------------------

void sam6883_friend_device_interface::endc_w(int state)
{
	m_endc = state;
	update_memory();
}
