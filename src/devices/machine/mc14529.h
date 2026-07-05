// license:BSD-3-Clause
// copyright-holders:Tim Lindner
/***************************************************************************

    mc14529.h

    MC14529 Dual 4-Channel Analog Data Selector, with asymmetric
    propagation delay characterized from oscilloscope measurements of a
    real MC14529 as used in the CoCo 1 audio/joystick mux.

    Measured select-to-output propagation delay (n=4 transitions each):
        tPLH (rising, L->H)  avg ~93.75 ns  -> default 94 ns
        tPHL (falling, H->L) avg ~200.00 ns -> default 200 ns

    Real hardware is switching a single analog node between 0V and 5V (or
    between two arbitrary analog levels); "L->H" and "H->L" are really
    "rising swing" and "falling swing" on that node. This device supports
    two independent per-selector modes to reflect how a caller wants to
    treat that swing:

      MODE_DIGITAL (default) -- each channel is a single bit (0/1), as
        used for e.g. the audio DAC-vs-sound-bit mux. tPLH/tPHL are
        chosen directly from the new bit value.

      MODE_ANALOG -- each channel carries a quantized multi-bit value
        (e.g. a 6-bit joystick comparator level, 0-63) representing where
        the analog swing has settled. The whole word is delivered as one
        unit after the propagation delay -- there is only one physical
        node switching, not one per bit -- but tPLH vs tPHL is chosen by
        comparing the new quantized value against the previously
        scheduled one: a numerically higher result is a rising swing
        (tPLH), a lower result is a falling swing (tPHL). Values are
        masked to the configured analog width (default 6 bits).

    Both modes share the same fixed-size per-selector timer pool (see
    mc14529.cpp), so multiple in-flight transitions are still preserved
    in chronological order in either mode.

    Pinout:
        A0, A1        -> address_w(bit, state)   (shared by both selectors)
        INH-X         -> inhibit_x_w(state)       (active high; forces Z-X to 0)
        INH-Y         -> inhibit_y_w(state)       (active high; forces Z-Y to 0)
        X0-X3         -> x_w(channel, state)          [MODE_DIGITAL]
                         x_analog_w(channel, value)    [MODE_ANALOG]
        Y0-Y3         -> y_w(channel, state)          [MODE_DIGITAL]
                         y_analog_w(channel, value)    [MODE_ANALOG]
        Z-X           -> zx_callback()        (write_line) [MODE_DIGITAL]
                         zx_analog_callback()  (write8)     [MODE_ANALOG]
        Z-Y           -> zy_callback()        (write_line) [MODE_DIGITAL]
                         zy_analog_callback()  (write8)     [MODE_ANALOG]

    This device treats the part as a 2-level digital selector or a
    quantized-analog-level selector, as appropriate to the mode -- not a
    true continuous bidirectional analog switch.

***************************************************************************/

#ifndef MAME_MACHINE_MC14529_H
#define MAME_MACHINE_MC14529_H

#pragma once

class mc14529_device : public device_t
{
public:
	enum mode_t : u8
	{
		MODE_DIGITAL = 0, // channel is a single bit
		MODE_ANALOG = 1   // channel is a quantized multi-bit level
	};

	enum
	{
		SEL_X = 0,
		SEL_Y = 1,
		NUM_SELECTORS = 2
	};

	mc14529_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	// configuration
	mc14529_device &set_propagation_delay(const attotime &tplh, const attotime &tphl);
	mc14529_device &set_mode(unsigned selector, mode_t mode);
	mc14529_device &set_analog_width(unsigned selector, unsigned bits); // default 6 bits (0-63)

	// digital-mode outputs
	auto zx_callback() { return m_write_z[SEL_X].bind(); }
	auto zy_callback() { return m_write_z[SEL_Y].bind(); }

	// analog-mode outputs (quantized multi-bit value)
	auto zx_analog_callback() { return m_write_z_analog[SEL_X].bind(); }
	auto zy_analog_callback() { return m_write_z_analog[SEL_Y].bind(); }

	auto zx_value() { return m_current_output[SEL_X] ;}
	auto zy_value() { return m_current_output[SEL_Y] ;}

	// shared address inputs (2-bit: bit 0 = A0, bit 1 = A1)
	void address_w(int bit, int state);
	void a0_w(int state) { address_w(0, state); }
	void a1_w(int state) { address_w(1, state); }

	// per-selector active-high inhibit
	void inhibit_x_w(int state);
	void inhibit_y_w(int state);

	// digital-mode data inputs, by channel number (0-3)
	void x_w(int channel, int state);
	void y_w(int channel, int state);

	void x0_w(int state) { x_w(0, state); }
	void x1_w(int state) { x_w(1, state); }
	void x2_w(int state) { x_w(2, state); }
	void x3_w(int state) { x_w(3, state); }
	void y0_w(int state) { y_w(0, state); }
	void y1_w(int state) { y_w(1, state); }
	void y2_w(int state) { y_w(2, state); }
	void y3_w(int state) { y_w(3, state); }

	// analog-mode data inputs (quantized multi-bit value), by channel number (0-3)
	void x_analog_w(int channel, u8 value);
	void y_analog_w(int channel, u8 value);

	void x0_analog_w(u8 value) { x_analog_w(0, value); }
	void x1_analog_w(u8 value) { x_analog_w(1, value); }
	void x2_analog_w(u8 value) { x_analog_w(2, value); }
	void x3_analog_w(u8 value) { x_analog_w(3, value); }
	void y0_analog_w(u8 value) { y_analog_w(0, value); }
	void y1_analog_w(u8 value) { y_analog_w(1, value); }
	void y2_analog_w(u8 value) { y_analog_w(2, value); }
	void y3_analog_w(u8 value) { y_analog_w(3, value); }

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	static constexpr unsigned NUM_CHANNELS = 4;

	// fixed-size pool of in-flight transitions per selector; sized with
	// generous headroom relative to real host timing (transitions would
	// have to arrive faster than one every ~tPHL, i.e. faster than every
	// ~200 ns of emulated time, to exhaust this)
	static constexpr unsigned MAX_PENDING = 8;

	void update_selector(unsigned selector);
	void commit(unsigned selector, unsigned slot);
	TIMER_CALLBACK_MEMBER(delay_expired);

	devcb_write_line m_write_z[NUM_SELECTORS];        // MODE_DIGITAL output
	devcb_write8 m_write_z_analog[NUM_SELECTORS];      // MODE_ANALOG output

	attotime m_tplh; // rising-swing propagation delay
	attotime m_tphl; // falling-swing propagation delay

	mode_t m_mode[NUM_SELECTORS];
	u8 m_width_mask[NUM_SELECTORS]; // e.g. 0x3F for a 6-bit quantized level

	u8 m_channel[NUM_SELECTORS][NUM_CHANNELS];       // MODE_DIGITAL: X0-X3/Y0-Y3 bit values
	u8 m_channel_value[NUM_SELECTORS][NUM_CHANNELS]; // MODE_ANALOG: X0-X3/Y0-Y3 quantized values

	u8 m_address;   // 2-bit shared address (bit0=A0, bit1=A1)
	u8 m_inhibit[NUM_SELECTORS]; // active high

	u8 m_current_output[NUM_SELECTORS]; // last output actually driven
	u8 m_last_scheduled[NUM_SELECTORS]; // target value of most recently queued (or committed) transition

	// fixed pool of pending transitions per selector
	emu_timer *m_pending_timer[NUM_SELECTORS][MAX_PENDING];
	u8         m_pending_value[NUM_SELECTORS][MAX_PENDING];
	bool       m_pending_active[NUM_SELECTORS][MAX_PENDING];
	unsigned   m_pending_next[NUM_SELECTORS]; // round-robin allocation index
};

DECLARE_DEVICE_TYPE(MC14529, mc14529_device)

#endif // MAME_MACHINE_MC14529_H
