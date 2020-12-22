// license:BSD-3-Clause
// copyright-holders:R. Belmont
/***************************************************************************

    drivers/mirage.c

    Ensoniq Mirage Sampler
    Preliminary driver by R. Belmont

    Map for Mirage:
    0000-7fff: 32k window on 128k of sample RAM
    8000-bfff: main RAM
    c000-dfff: optional expansion RAM
    e100-e101: 6850 UART (for MIDI)
    e200-e2ff: 6522 VIA
    e408-e40f: filter cut-off frequency
    e410-e417: filter resonance
    e418-e41f: multiplexer address pre-set
    e800-e803: WD1770 FDC
    ec00-ecef: ES5503 "DOC" sound chip
    f000-ffff: boot ROM

    NMI: IRQ from WD1772
    IRQ: DRQ from WD1772 wire-ORed with IRQ from ES5503 wire-ORed with IRQ from VIA6522
    FIRQ: IRQ from 6850 UART

    LED / switch matrix:

            A           B           C             D         E         F         G        DP
    ROW 0:  LOAD UPPER  LOAD LOWER  SAMPLE UPPER  PLAY SEQ  LOAD SEQ  SAVE SEQ  REC SEQ  SAMPLE LOWER
    ROW 1:  3           6           9             5         8         0         2        Enter
    ROW 2:  1           4           7             up arrow  PARAM     dn arrow  VALUE    CANCEL
    L. AN:  SEG A       SEG B       SEG C         SEG D     SEG E     SEG F     SEG G    SEG DP (decimal point)
    R. AN:  SEG A       SEG B       SEG C         SEG D     SEG E     SEG F     SEG G    SEG DP

    Column number in VIA port A bits 0-2 is converted to discrete lines by a 74LS145.
    Port A bit 3 is right anode, bit 4 is left anode
    ROW 0 is read on VIA port A bit 5, ROW 1 in port A bit 6, and ROW 2 in port A bit 7.

    Keyboard models talk to the R6500 through the VIA shifter: CA2 is handshake, CB1 is shift clock, CB2 is shift data.
    This is unconnected on the rackmount version.

***************************************************************************/


#include "emu.h"
#include "cpu/m6809/m6809.h"
#include "imagedev/floppy.h"
#include "machine/6850acia.h"
#include "machine/6522via.h"
#include "machine/wd_fdc.h"
#include "formats/esq8_dsk.h"
#include "sound/es5503.h"
#include "video/pwm.h"
#include "speaker.h"
#include "machine/input_merger.h"
#include "bus/midi/midi.h"
#include "imagedev/cassette.h"

#include "mirage.lh"

#define PITCH_TAG "pitch"
#define MOD_TAG "mod"

class en_sample_device : public device_t,
	public device_sound_interface
{
public:
	en_sample_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);
	~en_sample_device() { }
	uint8_t sample() {return m_sample;}

protected:
	// device-level overrides
	virtual void device_start() override;

	// sound stream update overrides
	virtual void sound_stream_update(sound_stream &stream, std::vector<read_stream_view> const &inputs, std::vector<write_stream_view> &outputs) override;

private:
	sound_stream*  m_stream;
	uint8_t m_sample;
};

class enmirage_state : public driver_device
{
public:
	enmirage_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_display(*this, "display"),
		m_fdc(*this, "wd1772"),
		m_floppy_connector(*this, "wd1772:0"),
		m_via(*this, "via6522"),
		m_irq_merge(*this, "irqmerge"),
		m_sample(*this, "en_sample_tag"),
		m_cassette(*this, "cassette"),
		m_joystick(*this, {PITCH_TAG,MOD_TAG}),
		m_key(*this, {"pb5","pb6","pb7"})
	{
	}

	void mirage(machine_config &config);

	void init_mirage();

protected:
	virtual void device_start() override;

private:

	DECLARE_FLOPPY_FORMATS( floppy_formats );

	uint8_t mirage_via_read_porta();
	uint8_t mirage_via_read_portb();
	void mirage_via_write_porta(uint8_t data);
	void mirage_via_write_portb(uint8_t data);
	uint8_t mirage_adc_read();

	void mirage_map(address_map &map);

	virtual void machine_reset() override;

	required_device<mc6809e_device> m_maincpu;
	required_device<pwm_display_device> m_display;
	required_device<wd1772_device> m_fdc;
	required_device<floppy_connector> m_floppy_connector;
	required_device<via6522_device> m_via;
	required_device<input_merger_device> m_irq_merge;
	required_device<en_sample_device> m_sample;
	required_device<cassette_image_device> m_cassette;

	required_ioport_array<2> m_joystick;
	required_ioport_array<3> m_key;

	int last_sndram_bank;
	int m_mux_value;
	int m_key_col_select;
};

DEFINE_DEVICE_TYPE(EN_SAMPLE, en_sample_device, "en_sample", "Ensonic Mirage Sampler Circuit");

FLOPPY_FORMATS_MEMBER( enmirage_state::floppy_formats )
	FLOPPY_ESQ8IMG_FORMAT
FLOPPY_FORMATS_END

static void ensoniq_floppies(device_slot_interface &device)
{
	device.option_add("35dd", FLOPPY_35_DD);
}

uint8_t enmirage_state::mirage_adc_read()
{
	uint8_t value;
	switch( m_mux_value & 0x03 )
	{
		case 0:
			value = m_cassette->input(); /* microphone */
			break;
		case 1:
			value = m_sample->sample(); /* internal audio */
			break;
		case 2:
			value = m_joystick[0]->read(); /* pitch wheel */
			break;
		case 3:
			value = m_joystick[1]->read(); /* mod wheel */
			break;
	}

	return value;
}

void enmirage_state::device_start()
{
	// call base device_start
	driver_device::device_start();
}

void enmirage_state::machine_reset()
{
	last_sndram_bank = 0;
	membank("sndbank")->set_base(memregion("es5503")->base() );
}

void enmirage_state::mirage_map(address_map &map)
{
	map(0x0000, 0x7fff).bankrw("sndbank");  // 32k window on 128k of wave RAM
	map(0x8000, 0xbfff).ram();         // main RAM
	map(0xc000, 0xdfff).ram();         // expansion RAM
	map(0xe100, 0xe101).rw("acia6850", FUNC(acia6850_device::read), FUNC(acia6850_device::write));
	map(0xe200, 0xe2ff).m(m_via, FUNC(via6522_device::map));
	map(0xe400, 0xe4ff).noprw();
	map(0xe800, 0xe803).rw(m_fdc, FUNC(wd1772_device::read), FUNC(wd1772_device::write));
	map(0xec00, 0xecef).rw("es5503", FUNC(es5503_device::read), FUNC(es5503_device::write));
	map(0xf000, 0xffff).rom().region("osrom", 0);
}

// port A:
// bits 5/6/7 keypad rows 0/1/2 return
uint8_t enmirage_state::mirage_via_read_porta()
{
	uint8_t value;

	value  = ((m_key[0]->read() >> m_key_col_select) & 0x01) << 5;
	value |= ((m_key[1]->read() >> m_key_col_select) & 0x01) << 6;
	value |= ((m_key[2]->read() >> m_key_col_select) & 0x01) << 7;

	return value;
}

// port B:
//  bit 6: IN disk load
//  bit 5: IN Q Chip sync
uint8_t enmirage_state::mirage_via_read_portb()
{
	floppy_image_device *flop = m_floppy_connector->get_device();
	return ((!flop->ready_r()) & 0x01) << 6;
}

// port A: front panel
// bits 0-2: column select from 0-7
// bits 3/4 = right and left LED enable
void enmirage_state::mirage_via_write_porta(uint8_t data)
{
	u8 segdata = data & 7;
	m_display->matrix(((data >> 3) & 3) ^ 3, (1<<segdata));

	m_key_col_select = (data & 0x07);
}

// port B:
//  bit 7: OUT UART clock
//  bit 4: OUT disk enable (motor on?)
//  bit 3: OUT sample/play
//  bit 2: OUT mic line/in
//  bit 1: OUT upper/lower bank (64k halves)
//  bit 0: OUT bank 0/bank 1 (32k halves)

void enmirage_state::mirage_via_write_portb(uint8_t data)
{
	int bank = 0;

	// handle sound RAM bank switching
	bank = (data & 2) ? (64*1024) : 0;
	bank += (data & 1) ? (32*1024) : 0;
	if (bank != last_sndram_bank)
	{
		last_sndram_bank = bank;
		membank("sndbank")->set_base(memregion("es5503")->base() + bank);
	}

	floppy_image_device *flop = m_floppy_connector->get_device();
	flop->mon_w(data & 0x10 ? 1 : 0 );

	if( m_mux_value != ((data >> 2) & 0x03) )
	{
		m_mux_value = (data >> 2) & 0x03;
		logerror( "mux value: %d\n", m_mux_value );
	}
}

void enmirage_state::mirage(machine_config &config)
{
	MC6809E(config, m_maincpu, 2000000);
	m_maincpu->set_addrmap(AS_PROGRAM, &enmirage_state::mirage_map);

	INPUT_MERGER_ANY_HIGH(config, m_irq_merge).output_handler().set_inputline(m_maincpu, M6809_IRQ_LINE);

// 	SPEAKER(config, "lspeaker").front_left();
// 	SPEAKER(config, "rspeaker").front_right();
	SPEAKER(config, "speaker").front_center();

	CASSETTE(config, m_cassette);
	m_cassette->set_default_state(CASSETTE_PLAY | CASSETTE_MOTOR_DISABLED | CASSETTE_SPEAKER_ENABLED);
	m_cassette->add_route(ALL_OUTPUTS, "speaker", 1.0);

	EN_SAMPLE(config, m_sample, 7000000);
	m_sample->add_route(ALL_OUTPUTS, "speaker", 1.0);

	es5503_device &es5503(ES5503(config, "es5503", 7000000));
 	es5503.set_channels(1);
	es5503.irq_func().set(m_irq_merge, FUNC(input_merger_device::in_w<2>));
	es5503.adc_func().set(FUNC(enmirage_state::mirage_adc_read));
// 	es5503.add_route(0, "lspeaker", 1.0);
// 	es5503.add_route(1, "rspeaker", 1.0);
 	es5503.add_route(0, "en_sample_tag", 1.0);

	VIA6522(config, m_via, 1000000);
	m_via->readpa_handler().set(FUNC(enmirage_state::mirage_via_read_porta));
	m_via->writepa_handler().set(FUNC(enmirage_state::mirage_via_write_porta));
	m_via->readpb_handler().set(FUNC(enmirage_state::mirage_via_read_portb));
	m_via->writepb_handler().set(FUNC(enmirage_state::mirage_via_write_portb));
	m_via->irq_handler().set(m_irq_merge, FUNC(input_merger_device::in_w<0>));

	PWM_DISPLAY(config, m_display).set_size(2, 8);
	m_display->set_segmask(0x3, 0xff);
	config.set_default_layout(layout_mirage);

	acia6850_device &acia6850(ACIA6850(config, "acia6850", 0));
	acia6850.txd_handler().set("mdout", FUNC(midi_port_device::write_txd));
 	acia6850.irq_handler().set_inputline(m_maincpu, M6809_FIRQ_LINE);

	MIDI_PORT(config, "mdin", midiin_slot, "midiin").rxd_handler().set(acia6850, FUNC(acia6850_device::write_rxd));
	MIDI_PORT(config, "mdout", midiout_slot, "midiout");

	WD1772(config, m_fdc, 8000000);
	m_fdc->intrq_wr_callback().set_inputline(m_maincpu, INPUT_LINE_NMI);
	m_fdc->drq_wr_callback().set(m_irq_merge, FUNC(input_merger_device::in_w<1>));

	FLOPPY_CONNECTOR(config, "wd1772:0", ensoniq_floppies, "35dd", enmirage_state::floppy_formats);
}

static INPUT_PORTS_START( mirage )
	PORT_START("pb5") /* KEY ROW 0 */
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Load Upper")      PORT_CODE(KEYCODE_A)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Load Lower")      PORT_CODE(KEYCODE_B)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Sample Upper")    PORT_CODE(KEYCODE_C)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Play Sequence")   PORT_CODE(KEYCODE_D)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Load Sequence")   PORT_CODE(KEYCODE_E)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Save Sequence")   PORT_CODE(KEYCODE_F)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Record Sequence") PORT_CODE(KEYCODE_G)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Sample Lower")    PORT_CODE(KEYCODE_H)
	PORT_START("pb6") /* KEY ROW 1 */
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("3")     PORT_CODE(KEYCODE_3)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("6")     PORT_CODE(KEYCODE_6)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("9")     PORT_CODE(KEYCODE_9)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("5")     PORT_CODE(KEYCODE_5)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("8")     PORT_CODE(KEYCODE_8)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("0")     PORT_CODE(KEYCODE_0)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("2")     PORT_CODE(KEYCODE_2)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Enter") PORT_CODE(KEYCODE_ENTER)
	PORT_START("pb7") /* KEY ROW 2 */
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("1")      PORT_CODE(KEYCODE_1)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("4")      PORT_CODE(KEYCODE_4)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("7")      PORT_CODE(KEYCODE_7)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Up")     PORT_CODE(KEYCODE_UP)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Param")  PORT_CODE(KEYCODE_I)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Down")   PORT_CODE(KEYCODE_DOWN)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Value")  PORT_CODE(KEYCODE_J)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Cancel") PORT_CODE(KEYCODE_K)

	PORT_START(PITCH_TAG)
	PORT_BIT( 0xff, 0x7f, IPT_PADDLE) PORT_NAME("Pitch Wheel") PORT_SENSITIVITY(100) PORT_KEYDELTA(10) PORT_MINMAX(0x00,0xff) PORT_CODE_INC(KEYCODE_4_PAD) PORT_CODE_DEC(KEYCODE_1_PAD)
	PORT_START(MOD_TAG)
	PORT_BIT( 0xff, 0x7f, IPT_PADDLE) PORT_NAME("Mod Wheel") PORT_SENSITIVITY(100) PORT_KEYDELTA(10) PORT_MINMAX(0x00,0xff) PORT_CODE_INC(KEYCODE_5_PAD) PORT_CODE_DEC(KEYCODE_6_PAD)
INPUT_PORTS_END

ROM_START( enmirage )
	ROM_REGION(0x1000, "osrom", 0)
	ROM_LOAD( "mirage.bin", 0x0000, 0x1000, CRC(9fc7553c) SHA1(ec6ea5613eeafd21d8f3a7431a35a6ff16eed56d) )

	ROM_REGION(0x20000, "es5503", ROMREGION_ERASE)
ROM_END

void enmirage_state::init_mirage()
{
	floppy_image_device *floppy = m_floppy_connector ? m_floppy_connector->get_device() : nullptr;
	if (floppy)
	{
		m_fdc->set_floppy(floppy);

		floppy->ss_w(0);
	}

	// port A: front panel
	m_via->write_pa0(0);
	m_via->write_pa1(0);
	m_via->write_pa2(0);
	m_via->write_pa3(0);
	m_via->write_pa4(0);
	m_via->write_pa5(0);
	m_via->write_pa6(0);
	m_via->write_pa7(0);

	// port B:
	//  bit 6: IN FDC disk ready
	//  bit 5: IN 5503 sync (?)
	m_via->write_pb0(0);
	m_via->write_pb1(0);
	m_via->write_pb2(0);
	m_via->write_pb3(0);
	m_via->write_pb4(0);
	m_via->write_pb5(1);
	m_via->write_pb6(0);    // how to determine if a disk is inserted?
	m_via->write_pb7(0);
}

//-------------------------------------------------
//  en_sample_device - constructor
//-------------------------------------------------

en_sample_device::en_sample_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, EN_SAMPLE, tag, owner, clock),
		device_sound_interface(mconfig, *this),
		m_stream(nullptr),
		m_sample(0)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void en_sample_device::device_start()
{
	m_stream = stream_alloc(1, 1, machine().sample_rate());
}

//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void en_sample_device::sound_stream_update(sound_stream &stream, std::vector<read_stream_view> const &inputs, std::vector<write_stream_view> &outputs)
{
	auto &src = inputs[0];
	auto &dst = outputs[0];

	int count = dst.samples();
	double m_rms = 0;

	if( count > 0 )
	{
		for (int sampindex = 0; sampindex < count; sampindex++)
		{
			m_rms += src.get(sampindex);
			dst.put(sampindex, src.get(sampindex));
		}

		m_rms /= count;
	}

	m_sample = 0x7f;
}

CONS( 1984, enmirage, 0, 0, mirage, mirage, enmirage_state, init_mirage, "Ensoniq", "Mirage", MACHINE_NOT_WORKING )
