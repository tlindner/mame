// license:BSD-3-Clause
// copyright-holders:tim lindner
/***************************************************************************

    mc14529.h

    MC14529 Dual 4-Channel Analog Data Selector, with support for asymmetric
    propagation delay. By default it will use a symmetrical 150ns delay marked
    as typical by the data sheet.

    But I characterized a part from a Color Computer using an oscilloscope to
    have real measurements.

    Measured select-to-output propagation delay (n=4 transitions each):
        tPLH (rising, L->H)  avg ~93.75 ns
        tPHL (falling, H->L) avg ~200.00 ns

    Real hardware is switching a single analog node between two levels;
    "L->H" and "H->L" are really "rising swing" and "falling swing" on
    that node. This device supports three independent per-selector modes:

      MODE_DIGITAL (default) -- each channel is a single bit (0/1).
        tPLH/tPHL chosen directly from the new bit value. Delivered via
        x_w()/y_w() and zx_callback()/zy_callback() (write_line).

      MODE_ANALOG -- each channel carries a quantized multi-bit value
        (e.g. a 6-bit joystick comparator level, 0-63). tPLH vs tPHL is
        chosen by comparing the new quantized value against the
        previously scheduled one (higher = rising = tPLH, lower =
        falling = tPHL). Delivered via x_analog_w()/y_analog_w() and
        zx_analog_callback()/zy_analog_callback() (write8).

      MODE_SOUND -- each channel is a live audio signal, routed in via
        MAME's device_sound_interface, e.g. from another sound-generating
        device with add_route(output, mux, gain, mux.x_sound_input(ch)).
        The selected channel's audio is passed straight through to this
        device's own sound output (x_sound_output()/y_sound_output()
        route index), which can in turn be routed to a SPEAKER device.
        No propagation delay is modeled in this mode. The switch is
        treated as instantaneous, cutting over exactly at the sample
        boundary corresponding to the address/inhibit change.

    Bridging MODE_ANALOG and MODE_SOUND: a MODE_ANALOG selector's
    quantized output does NOT appear on this device's sound stream --
    the two modes are independent per selector and do not feed each
    other internally. To turn a delay-correct quantized analog value
    into audio, route it through an external DAC device instead: wire
    the MODE_ANALOG selector's zx_analog_callback()/zy_analog_callback()
    to a MAME DAC device (e.g. a dac_byte_interface implementation), then
    add_route() that DAC's sound output into a *different* selector on
    this device (or another mc14529) configured as MODE_SOUND, and from
    there to a SPEAKER. This keeps the propagation delay applied exactly
    once, at the MODE_ANALOG stage, with no timing logic needed in the
    sound path itself.

    MODE_DIGITAL and MODE_ANALOG share a fixed-size per-selector timer
    pool (see mc14529.cpp) so multiple in-flight transitions are
    preserved in chronological order. MODE_SOUND does not use that pool
    at all; it is handled, without delays, entirely inside sound_stream_update().

    Pinout:
        A0, A1        -> address_w(bit, state)   (shared by both selectors)
        INH-X         -> inhibit_x_w(state)       (active high; forces Z-X to 0/silence)
        INH-Y         -> inhibit_y_w(state)       (active high; forces Z-Y to 0/silence)
        X0-X3         -> x_w(channel, state)          [MODE_DIGITAL]
                         x_analog_w(channel, value)    [MODE_ANALOG]
                         (external device routed to x_sound_input(channel)) [MODE_SOUND]
        Y0-Y3         -> y_w(channel, state)          [MODE_DIGITAL]
                         y_analog_w(channel, value)    [MODE_ANALOG]
                         (external device routed to y_sound_input(channel)) [MODE_SOUND]
        Z-X           -> zx_callback()        (write_line) [MODE_DIGITAL]
                         zx_analog_callback()  (write8)     [MODE_ANALOG]
                         route from x_sound_output() index  [MODE_SOUND]
        Z-Y           -> zy_callback()        (write_line) [MODE_DIGITAL]
                         zy_analog_callback()  (write8)     [MODE_ANALOG]
                         route from y_sound_output() index  [MODE_SOUND]

	TODO:
    This device treats the part as a 2-level digital selector, a
    quantized-analog-level selector, or an audio-stream selector,
    depending on mode. Not a true continuous bidirectional analog
    switch.

    This device does not implement the combined 8-channel mode.

***************************************************************************/

#ifndef MAME_MACHINE_MC14529_H
#define MAME_MACHINE_MC14529_H

#pragma once

class mc14529_device : public device_t, public device_sound_interface
{
public:
	enum mode_t : u8
	{
		MODE_DIGITAL = 0, // channel is a single bit
		MODE_ANALOG = 1,  // channel is a quantized multi-bit level
		MODE_SOUND = 2    // channel is a live audio stream (no propagation delay)
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
	mc14529_device &set_analog_width(unsigned selector, unsigned bits); // default 8 bits

	// digital-mode outputs
	auto zx_callback() { return m_write_z[SEL_X].bind(); }
	auto zy_callback() { return m_write_z[SEL_Y].bind(); }

	// analog-mode outputs (quantized multi-bit value)
	auto zx_analog_callback() { return m_write_z_analog[SEL_X].bind(); }
	auto zy_analog_callback() { return m_write_z_analog[SEL_Y].bind(); }

	// polling reads of the last-committed digital/analog output (bit or
	// quantized level). Does not disturb any in-flight transition, same
	// as probing a real output pin's voltage. Not meaningful in MODE_SOUND.
	u8 zx_value();
	u8 zy_value();

	// sound-mode routing helpers: pass these as the input-index argument
	// to an upstream device's add_route() call, and these as the
	// output-index argument to this device's own add_route() call
	// (e.g. into a SPEAKER device).
	static constexpr int x_sound_input(unsigned channel) { return SEL_X * NUM_CHANNELS + channel; }
	static constexpr int y_sound_input(unsigned channel) { return SEL_Y * NUM_CHANNELS + channel; }
	static constexpr int x_sound_output() { return SEL_X; }
	static constexpr int y_sound_output() { return SEL_Y; }

	// shared address inputs (2-bit: bit 0 = A0, bit 1 = A1)
	void address_w(int bit, int state);
	void a0_w(int state) { address_w(0, state); }
	void a1_w(int state) { address_w(1, state); }
	int current_address() { return m_address; }

	// per-selector active-high inhibit
	void inhibit_x_w(int state);
	void inhibit_y_w(int state);

	// digital-mode data inputs, by channel number (0-3); ignored unless
	// that selector is in MODE_DIGITAL
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

	// analog-mode data inputs (quantized multi-bit value), by channel
	// number (0-3); ignored unless that selector is in MODE_ANALOG
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

	// device_sound_interface
	virtual void sound_stream_update(sound_stream &stream) override;

private:
	static constexpr unsigned NUM_CHANNELS = 4;

	// fixed-size pool of in-flight transitions per selector (MODE_DIGITAL /
	// MODE_ANALOG only); sized with generous headroom relative to real
	// host timing (transitions would have to arrive faster than one every
	// ~tPHL, i.e. faster than every ~200 ns of emulated time, to exhaust
	// this)
	static constexpr unsigned MAX_PENDING = 8;

	void update_selector(unsigned selector);
	void commit(unsigned selector, unsigned slot);
	TIMER_CALLBACK_MEMBER(delay_expired);

	devcb_write_line m_write_z[NUM_SELECTORS];        // MODE_DIGITAL output
	devcb_write8 m_write_z_analog[NUM_SELECTORS];     // MODE_ANALOG output

	sound_stream *m_stream; // MODE_SOUND input/output

	attotime m_tplh; // rising-swing propagation delay
	attotime m_tphl; // falling-swing propagation delay

	mode_t m_mode[NUM_SELECTORS];
	u8 m_width_mask[NUM_SELECTORS]; // e.g. 0x3F for a 6-bit quantized level

	u8 m_channel[NUM_SELECTORS][NUM_CHANNELS];       // MODE_DIGITAL: X0-X3/Y0-Y3 bit values
	u8 m_channel_value[NUM_SELECTORS][NUM_CHANNELS]; // MODE_ANALOG: X0-X3/Y0-Y3 quantized values

	u8 m_address;   // 2-bit shared address (bit0=A0, bit1=A1)
	u8 m_inhibit[NUM_SELECTORS]; // active high

	u8 m_current_output[NUM_SELECTORS]; // last output actually driven (MODE_DIGITAL/MODE_ANALOG)
	u8 m_last_scheduled[NUM_SELECTORS]; // target value of most recently queued (or committed) transition

	// fixed pool of pending transitions per selector
	emu_timer *m_pending_timer[NUM_SELECTORS][MAX_PENDING];
	u8         m_pending_value[NUM_SELECTORS][MAX_PENDING];
	bool       m_pending_active[NUM_SELECTORS][MAX_PENDING];
	unsigned   m_pending_next[NUM_SELECTORS]; // round-robin allocation index
};

DECLARE_DEVICE_TYPE(MC14529, mc14529_device)

#endif // MAME_MACHINE_MC14529_H
