// license:BSD-3-Clause
// copyright-holders:tim lindner
/***************************************************************************

    coco_ssc6809.cpp

    Code for emulating the ssc6809

    This is a proposed cartridge replacing the TMS7040 with a 6809.

	CoCo Address 0xFF7D is soft reset
	CoCo Address 0xFF7E is read status, write to device

	Load allophone is 6809 IRQ (TMS7040 INT1)
	Timer is 6809 FIRQ (TMS7040 INT2)
	Host byte is 6809 NMI (TMS7040 INT3)

	6809 RAM - $0000-$00ff

	6809 Peripheral file:
		$100 IO Control
				bit		read		write
				0		INT1 Enable	INT1 Enable   $01
				1		INT1 Flag	INT1 Clear    $02
				2		INT2 Enable	INT2 Enable   $04
				3		INT2 Flag	INT2 Clear    $08
				4		INT3 Enable	INT3 Enable   $10
				5		INT3 Flag	INT3 Clear    $20

		$102 Timer 1 Data
			Read: current decremented value
			Write: Timer 1 reload register

		$103 Timer 1 Control
				bit		write
				0-4		Pre-scale reload register
				6		Counting source: 0 internal, 1 external
				7		Hold: 0, Run :1

				bit		read
				0-7		Capture Latch Value

		$104 Port A data
		$106 Port B data
		$108 Port C data
		$109 Port C data direction (1: output, 0: input)
		$10a Port D data
		$10b Port D data direction (1: output, 0: input)

    All four ports of the I/O are under software control.

    Port A is input from the host CPU.
    Port B is A0-A7 for the 2k of RAM.
    Port C is the internal bus controller:
        bit 7 6 5 4 3 2 1 0
            | | | | | | | |
            | | | | | | | + A8 for RAM and BC1 of AY3-8913
            | | | | | | +-- A9 for RAM
            | | | | | +---- A10 for RAM
            | | | | +------ R/W* for RAM and BDIR for AY3-8913
            | | | +-------- CS* for RAM
            | | +---------- ALD* for SP0256
            | +------------ CS* for AY3-8913
            +-------------- BUSY* for host CPU (connects to a latch)
        * – Active low
    Port D is the 8-bit data bus.


***************************************************************************/

#include "emu.h"
#include "coco_ssc6809.h"

#include "cpu/m6809/m6809.h"
#include "machine/ram.h"
#include "sound/ay8910.h"
#include "sound/sp0256.h"
#include "machine/input_merger.h"

#include "speaker.h"

#define LOG_INTERFACE   (1U <<  1)
#define LOG_INTERNAL    (1U <<  2)
#define LOG_RAMADDRESS  (1U <<  3)
// #define VERBOSE (0)
// #define VERBOSE (LOG_INTERFACE | LOG_RAMADDRESS)
#define VERBOSE (LOG_INTERFACE | LOG_INTERNAL | LOG_RAMADDRESS)

#include "logmacro.h"

#define LOGINTERFACE(...) LOGMASKED(LOG_INTERFACE, __VA_ARGS__)
#define LOGINTERNAL(...) LOGMASKED(LOG_INTERNAL, __VA_ARGS__)
#define LOGRAM(...) LOGMASKED(LOG_RAMADDRESS, __VA_ARGS__)

#define PIC_TAG "pic6809"
#define AY_TAG "cocossc_6809_ay"
#define SP0256_TAG "sp0256_6809"

#define SP0256_GAIN 1.75
#define AY8913_GAIN 2.0

#define C_A8   0x01
#define C_BC1  0x01
#define C_A9   0x02
#define C_A10  0x04
#define C_RRW  0x08
#define C_BDR  0x08
#define C_RCS  0x10
#define C_ALD  0x20
#define C_ACS  0x40
#define C_BSY  0x80

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

namespace
{
	class cocossc_6809_sac_device;

	// ======================> coco_ssc_6809_device

	class coco_ssc_6809_device :
			public device_t,
			public device_cococart_interface
	{
	public:
		// construction/destruction
		coco_ssc_6809_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

		// optional information overrides
		virtual const tiny_rom_entry *device_rom_region() const override;
		virtual void device_add_mconfig(machine_config &config) override;
		virtual void device_reset() override;

		u8 ssc_port_a_r();
		void ssc_port_b_w(u8 data);
		u8 ssc_port_c_r();
		void ssc_port_c_w(u8 data);
		u8 ssc_port_d_r();
		void ssc_port_d_w(u8 data);

		u8 pf_r(offs_t offset);
		void pf_w(offs_t offset, uint8_t data);

	protected:
		// device-level overrides
		virtual void device_start() override;
		virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
		u8 ff7d_read(offs_t offset);
		void ff7d_write(offs_t offset, u8 data);
		virtual void set_sound_enable(bool sound_enable) override;
		static constexpr device_timer_id PF_TIMER_ID = 0;

		DECLARE_WRITE_LINE_MEMBER(load_allophone);

	private:
		void m6809_mem(address_map &map);
		u8                                      m_reset_line;
		bool                                    m_tms7000_busy;
		u8                                      m_tms7000_porta;
		u8                                      m_tms7000_portb;
		u8                                      m_tms7000_portc;
		u8                                      m_tms7000_portd;
		u8										pf_IOCNT0;
		u8										pf_T1CTL;
		u8										pf_T1_DECREMENTER;
		u8										pf_T1_RELOAD;
		u8										pf_CAP_LATCH;
		u8										pf_CDDR;
		u8										pf_DDDR;

		emu_timer                               *m_pf_timer;
		required_device<m6809_device>         	m_m6809;
		required_device<ram_device>             m_staticram;
		required_device<ay8910_device>          m_ay;
		required_device<sp0256_device>          m_spo;
		required_device<cocossc_6809_sac_device> m_sac;
		required_device<input_merger_device>	m_im_int1;
		required_device<input_merger_device>	m_im_int2;
		required_device<input_merger_device>	m_im_int3;
	};

	// ======================> Color Computer Sound Activity Circuit filter

	class cocossc_6809_sac_device : public device_t,
		public device_sound_interface
	{
	public:
		cocossc_6809_sac_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);
		~cocossc_6809_sac_device() { }
		bool sound_activity_circuit_output();

	protected:
		// device-level overrides
		virtual void device_start() override;

		// sound stream update overrides
		virtual void sound_stream_update(sound_stream &stream, std::vector<read_stream_view> const &inputs, std::vector<write_stream_view> &outputs) override;

		// Power of 2
		static constexpr int BUFFER_SIZE = 4;
	private:
		sound_stream *m_stream;
		double m_rms[BUFFER_SIZE];
		int m_index;
	};
};

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE_PRIVATE(COCO_SSC6809, device_cococart_interface, coco_ssc_6809_device, "coco_ssc_6809", "CoCo S/SC PAK (6809)");
DEFINE_DEVICE_TYPE(COCOSSC_6809_SAC, cocossc_6809_sac_device, "cocossc_6809_sac", "CoCo SSC (6809) Sound Activity Circuit");

void coco_ssc_6809_device::m6809_mem(address_map &map)
{
	map(0x0000, 0x00ff).ram();
	map(0x0100, 0x010f).rw(FUNC(coco_ssc_6809_device::pf_r), FUNC(coco_ssc_6809_device::pf_w));
	map(0xe000, 0xffff).rom().region(PIC_TAG, 0);
}

//**************************************************************************
//  MACHINE FRAGMENTS AND ADDRESS MAPS
//**************************************************************************

void coco_ssc_6809_device::device_add_mconfig(machine_config &config)
{
	M6809(config, m_m6809, DERIVED_CLOCK(2, 1));

	m_m6809->set_addrmap(AS_PROGRAM, &coco_ssc_6809_device::m6809_mem);

	RAM(config, "staticram").set_default_size("2K").set_default_value(0);

	SPEAKER(config, "ssc_6809_audio").front_center();

	INPUT_MERGER_ALL_HIGH(config, m_im_int1).output_handler().set_inputline(m_m6809, M6809_IRQ_LINE);
	INPUT_MERGER_ALL_HIGH(config, m_im_int2).output_handler().set_inputline(m_m6809, M6809_FIRQ_LINE);
	INPUT_MERGER_ALL_HIGH(config, m_im_int3).output_handler().set_inputline(m_m6809, INPUT_LINE_NMI);

	SP0256(config, m_spo, XTAL(3'120'000));
	m_spo->add_route(ALL_OUTPUTS, "ssc_6809_audio", SP0256_GAIN);

	m_spo->data_request_callback().set(FUNC(coco_ssc_6809_device::load_allophone));

	AY8913(config, m_ay, DERIVED_CLOCK(2, 1));
	m_ay->set_flags(AY8910_SINGLE_OUTPUT);
	m_ay->add_route(ALL_OUTPUTS, "coco_sac_tag", AY8913_GAIN);

	COCOSSC_6809_SAC(config, m_sac, DERIVED_CLOCK(2, 1));
	m_sac->add_route(ALL_OUTPUTS, "ssc_6809_audio", 1.0);
}

ROM_START(coco_ssc_6809)
	ROM_REGION(0x2000, PIC_TAG, 0)
	ROM_LOAD("ssc6809.bin", 0x0000, 0x2000, CRC(0) SHA1(0))
	ROM_REGION(0x10000, SP0256_TAG, 0)
	ROM_LOAD("sp0256-al2.bin", 0x1000, 0x0800, CRC(b504ac15) SHA1(e60fcb5fa16ff3f3b69d36c7a6e955744d3feafc))
ROM_END

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  coco_ssc_6809_device - constructor
//-------------------------------------------------

coco_ssc_6809_device::coco_ssc_6809_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
		: device_t(mconfig, COCO_SSC6809, tag, owner, clock),
		device_cococart_interface(mconfig, *this ),
		m_m6809(*this, PIC_TAG),
		m_staticram(*this, "staticram"),
		m_ay(*this, AY_TAG),
		m_spo(*this, SP0256_TAG),
		m_sac(*this, "coco_sac_tag"),
		m_im_int1(*this, "im_int1"),
		m_im_int2(*this, "im_int2"),
		m_im_int3(*this, "im_int3")
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void coco_ssc_6809_device::device_start()
{
	// install $ff7d-e handler
	install_readwrite_handler(0xff7d, 0xff7e,
			read8sm_delegate(*this, FUNC(coco_ssc_6809_device::ff7d_read)),
			write8sm_delegate(*this, FUNC(coco_ssc_6809_device::ff7d_write)));

	save_item(NAME(m_reset_line));
	save_item(NAME(m_tms7000_busy));
	save_item(NAME(m_tms7000_porta));
	save_item(NAME(m_tms7000_portb));
	save_item(NAME(m_tms7000_portc));
	save_item(NAME(m_tms7000_portd));

	save_item(NAME(pf_IOCNT0));
	save_item(NAME(pf_T1CTL));
	save_item(NAME(pf_T1_DECREMENTER));
	save_item(NAME(pf_T1_RELOAD));
	save_item(NAME(pf_CAP_LATCH));
	save_item(NAME(pf_CDDR));
	save_item(NAME(pf_DDDR));

	m_pf_timer = timer_alloc(PF_TIMER_ID);
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void coco_ssc_6809_device::device_reset()
{
	m_reset_line = 0;
	m_tms7000_busy = true;
	pf_IOCNT0 = 0;
	pf_T1CTL = 0;
	pf_T1_DECREMENTER = 0;
	pf_T1_RELOAD = 0;
	pf_CAP_LATCH = 0;
	pf_CDDR = 0;
	pf_DDDR = 0;

	m_pf_timer->adjust(attotime::never);
}


//-------------------------------------------------
//  device_timer - handle timer callbacks
//-------------------------------------------------

void coco_ssc_6809_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	switch(id)
	{
		case PF_TIMER_ID:
			pf_T1_DECREMENTER--;

			if( pf_T1_DECREMENTER == std::numeric_limits<u8>::max()) {
				pf_IOCNT0 = pf_IOCNT0 | 0x08;

				if( (pf_IOCNT0 & 0x04) == 0x04) {
					m_im_int2->in_w<1>(1);
				}

				pf_T1_DECREMENTER = pf_T1_RELOAD;
			}

			m_pf_timer->adjust(attotime::from_hz(clock()) * 16 * ((pf_T1CTL & 0x1f) + 1));

			break;

		default:
			break;

	}
}


//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *coco_ssc_6809_device::device_rom_region() const
{
	return ROM_NAME( coco_ssc_6809 );
}

//-------------------------------------------------
//  load_allophone
//-------------------------------------------------

WRITE_LINE_MEMBER(coco_ssc_6809_device::load_allophone)
{
	if( state == 1 )
	{
		pf_IOCNT0 = pf_IOCNT0 | 0x02;
	}

	m_im_int1->in_w<1>(state);
}

//-------------------------------------------------
//  set_sound_enable
//-------------------------------------------------

void coco_ssc_6809_device::set_sound_enable(bool sound_enable)
{
	if( sound_enable )
	{
		m_sac->set_output_gain(0, 1.0);
		m_spo->set_output_gain(0, 1.0);
	}
	else
	{
		m_sac->set_output_gain(0, 0.0);
		m_spo->set_output_gain(0, 0.0);
	}
}


//-------------------------------------------------
//  pf_r (read peripheral file)
//-------------------------------------------------

u8 coco_ssc_6809_device::pf_r(offs_t offset)
{
	u8 result = 0;

	switch( offset )
	{
		case 0x00:
			result = pf_IOCNT0;
			break;

		case 0x02:
			result = pf_T1_DECREMENTER;
			break;

		case 0x03:
			result = pf_CAP_LATCH;
			break;

		case 0x04:
			result = ssc_port_a_r();
			break;

		case 0x08:
			result = ssc_port_c_r();
			break;

		case 0x09:
			result = pf_CDDR;
			break;

		case 0x0a:
			result = ssc_port_d_r();
			break;

		case 0x0b:
			result = pf_DDDR;
			break;
	}

	return result;
}


//-------------------------------------------------
//  pf_w (write peripheral file)
//-------------------------------------------------

void coco_ssc_6809_device::pf_w(offs_t offset, uint8_t data)
{
	switch( offset )
	{
		case 0x00:
			if( (data & 0x02) == 0x02) {
				pf_IOCNT0 = pf_IOCNT0 & ~0x02;
			}

			if( (data & 0x08) == 0x08) {
				pf_IOCNT0 = pf_IOCNT0 & ~0x08;
				m_im_int2->in_w<1>(0);
			}

			if( (data & 0x20) == 0x20) {
				pf_IOCNT0 = pf_IOCNT0 & ~0x20;
			}

			pf_IOCNT0 = pf_IOCNT0 | (data & ~0x2a);
			m_im_int1->in_w<0>((pf_IOCNT0 & 0x01) ? 1 : 0);
			m_im_int2->in_w<0>((pf_IOCNT0 & 0x04) ? 1 : 0);
			m_im_int3->in_w<0>((pf_IOCNT0 & 0x10) ? 1 : 0);

			break;

		case 0x02:
			pf_T1_RELOAD = data;
			break;

		case 0x03:
			pf_T1CTL = data;

			if( (data & 0x80) == 0) {
				m_pf_timer->adjust(attotime::never);
				pf_T1CTL = pf_T1CTL & ~0x08;
				m_im_int2->in_w<1>(0);
			}
			else {
				// fOSC/16 - fOSC is freq _before_ internal clockdivider
				pf_T1_DECREMENTER = pf_T1_RELOAD;
				m_pf_timer->adjust(attotime::from_hz(clock()) * 16 * ((pf_T1CTL & 0x1f) + 1));
			}
			break;

		case 0x04:
			break;

		case 0x06:
			ssc_port_b_w( data );
			break;

		case 0x08:
			ssc_port_c_r();
			if( pf_CDDR != 0 )
			{
				ssc_port_c_w( data );
			}
			break;

		case 0x09:
			pf_CDDR = data;
			break;

		case 0x0a:
			ssc_port_d_r();
			if( pf_DDDR != 0 )
			{
				ssc_port_d_w( data );
			}
			break;

		case 0x0b:
			pf_DDDR = data;
			break;
	}

	return;
}


//-------------------------------------------------
//  ff7d_read
//-------------------------------------------------

u8 coco_ssc_6809_device::ff7d_read(offs_t offset)
{
	u8 data = 0xff;

	switch(offset)
	{
		case 0x00:
			data = 0xff;
			LOGINTERFACE( "ff7d read: %02x\n", data );
			break;

		case 0x01:
			data = 0x1f;

			if( m_tms7000_busy == false )
			{
				data |= 0x80;
			}

			if( m_spo->sby_r() )
			{
				data |= 0x40;
			}

			if(  m_sac->sound_activity_circuit_output() )
			{
				data |= 0x20;
			}

			LOGINTERFACE( "ff7e read: %c%c%c%c %c%c%c%c (%02x)\n",
					data & 0x80 ? 'b' : 'B',
					data & 0x40 ? 's' : 'S',
					data & 0x20 ? 'p' : 'P',
					data & 0x10 ? '1' : '0',
					data & 0x08 ? '1' : '0',
					data & 0x04 ? '1' : '0',
					data & 0x02 ? '1' : '0',
					data & 0x01 ? '1' : '0',
					data );

			break;
	}

	return data;
}


//-------------------------------------------------
//  ff7d_write
//-------------------------------------------------

void coco_ssc_6809_device::ff7d_write(offs_t offset, u8 data)
{
	switch(offset)
	{
		case 0x00:
			LOGINTERFACE( "ff7d write: %02x\n", data );

			if( (data & 1) == 1 )
			{
				m_spo->reset();
			}

			if( (m_reset_line & 1) == 1 )
			{
				if( (data & 1) == 0 )
				{
					m_m6809->reset();
					m_ay->reset();
					m_tms7000_busy = false;
				}
			}

			m_reset_line = data;
			break;

		case 0x01:
			LOGINTERFACE( "ff7e write: %02x\n", data );

			pf_IOCNT0 = pf_IOCNT0 | 0x20;
			m_tms7000_porta = data;
			m_tms7000_busy = true;
			m_im_int3->in_w<1>(1);
			pf_CAP_LATCH = pf_T1_DECREMENTER;
			break;
	}
}


//-------------------------------------------------
//  Handlers for secondary CPU ports
//-------------------------------------------------

u8 coco_ssc_6809_device::ssc_port_a_r()
{
	LOGINTERNAL( "[pc=%04x] port a read: %02x\n", m_m6809->pc(), m_tms7000_porta );

	if (!machine().side_effects_disabled())
	{
		m_im_int3->in_w<1>(0);
	}

	return m_tms7000_porta;
}

void coco_ssc_6809_device::ssc_port_b_w(u8 data)
{
	LOGINTERNAL( "[pc=%04x] port b write: %02x\n", m_m6809->pc(), data );

	m_tms7000_portb = data;
}

u8 coco_ssc_6809_device::ssc_port_c_r()
{
	LOGINTERNAL( "[pc=%04x] port c read: %02x\n", m_m6809->pc(), m_tms7000_portc );

	return m_tms7000_portc;
}

void coco_ssc_6809_device::ssc_port_c_w(u8 data)
{
	if( (data & C_RCS) == 0 && (data & C_RRW) == 0) /* static RAM write */
	{
		u16 address = u16(data) << 8;
		address += m_tms7000_portb;
		address &= 0x7ff;

		m_staticram->write(address, m_tms7000_portd);
		LOGRAM( "[pc=%04x] write RAM address %04x\n", m_m6809->pc(), address );
	}

	if( (data & C_ACS) == 0 ) /* chip select for AY-3-8913 */
	{
		if( (data & (C_BDR|C_BC1)) == (C_BDR|C_BC1) ) /* BDIR = 1, BC1 = 1: latch address */
		{
			m_ay->address_w(m_tms7000_portd);
		}

		if( ((data & C_BDR) == C_BDR) && ((data & C_BC1) == 0) ) /* BDIR = 1, BC1 = 0: write data */
		{
			m_ay->data_w(m_tms7000_portd);
		}
	}

	if( (data & C_ALD) == 0 )
	{
		m_spo->ald_w(m_tms7000_portd);
	}

	if( ((m_tms7000_portc & C_BSY) == 0) && ((data & C_BSY) == C_BSY) )
	{
		m_tms7000_busy = false;
	}

	LOGINTERNAL( "[pc=%04x] port c write: %c%c%c%c %c%c%c%c (%02x)\n", m_m6809->pc(),
			data & 0x80 ? '.' : 'B',
			data & 0x40 ? '.' : 'P',
			data & 0x20 ? '.' : 'V',
			data & 0x10 ? '.' : 'R',
			data & 0x40 ? (data & 0x08 ? 'R' : 'W') : (data & 0x08 ? 'D' : '.'),
			data & 0x04 ? '1' : '0',
			data & 0x02 ? '1' : '0',
			data & 0x40 ? (data & 0x01 ? '1' : '0') : (data & 0x01 ? 'C' : '.'),
			data );

	m_tms7000_portc = data;
}

u8 coco_ssc_6809_device::ssc_port_d_r()
{
	if( ((m_tms7000_portc & C_RCS) == 0) && ((m_tms7000_portc & C_ACS) == 0))
		logerror( "[%s] Warning: Reading RAM and PSG at the same time!\n", machine().describe_context() );

	if( ((m_tms7000_portc & C_RCS) == 0)  && ((m_tms7000_portc & C_RRW) == C_RRW)) /* static ram chip select (low) and static ram chip read (high) */
	{
		u16 address = u16(m_tms7000_portc) << 8;
		address += m_tms7000_portb;
		address &= 0x7ff;

		m_tms7000_portd = m_staticram->read(address);
		LOGRAM( "[pc=%04x] read RAM address %04x\n", m_m6809->pc(), address );
	}

	if( (m_tms7000_portc & C_ACS) == 0 ) /* chip select for AY-3-8913 */
	{
		if( ((m_tms7000_portc & C_BDR) == 0) && ((m_tms7000_portc & C_BC1) == C_BC1) ) /* psg read data */
		{
			m_tms7000_portd = m_ay->data_r();
		}
	}

	LOGINTERNAL( "[pc=%04x] port d read: %02x\n", m_m6809->pc(), m_tms7000_portd );

	return m_tms7000_portd;
}

void coco_ssc_6809_device::ssc_port_d_w(u8 data)
{
	LOGINTERNAL( "[pc=%04x] port d write: %02x\n", m_m6809->pc(), data );

	m_tms7000_portd = data;
}


//-------------------------------------------------
//  cocossc_6809_sac_device - constructor
//-------------------------------------------------

cocossc_6809_sac_device::cocossc_6809_sac_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, COCOSSC_6809_SAC, tag, owner, clock),
		device_sound_interface(mconfig, *this),
		m_stream(nullptr),
		m_index(0)
{
 	std::fill(std::begin(m_rms), std::end(m_rms), 0);
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void cocossc_6809_sac_device::device_start()
{
	m_stream = stream_alloc(1, 1, machine().sample_rate());
}


//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void cocossc_6809_sac_device::sound_stream_update(sound_stream &stream, std::vector<read_stream_view> const &inputs, std::vector<write_stream_view> &outputs)
{
	auto &src = inputs[0];
	auto &dst = outputs[0];

	int count = dst.samples();
	m_rms[m_index] = 0;

	if( count > 0 )
	{
		for (int sampindex = 0; sampindex < count; sampindex++)
		{
			// sum the squares
			m_rms[m_index] += src.get(sampindex) * src.get(sampindex);
			// copy from source to destination
			dst.put(sampindex, src.get(sampindex));
		}

		// calculate root mean square
		m_rms[m_index] = m_rms[m_index] / count;
		m_rms[m_index] = sqrt(m_rms[m_index]);
	}

	m_index++;
	m_index &= (BUFFER_SIZE-1);
}


//-------------------------------------------------
//  sound_activity_circuit_output - making sound
//-------------------------------------------------

bool cocossc_6809_sac_device::sound_activity_circuit_output()
{
	double sum = std::accumulate(std::begin(m_rms), std::end(m_rms), 0.0);
	double average = (sum / BUFFER_SIZE);

	return average < 0.317;
}
