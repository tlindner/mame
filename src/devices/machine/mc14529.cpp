// license:BSD-3-Clause
// copyright-holders:Tim Lindner
/***************************************************************************

    mc14529.cpp

    See mc14529.h for device details.

    MODE_DIGITAL / MODE_ANALOG timing model:
    The MC14529's specified delay is switch slew time, not propagation
    delay. Once a channel is connected, changes on that input appear at
    the output immediately. The delay only occurs when the selector
    switches to a different channel (address_w() or inhibit_x_w() /
    inhibit_y_w()).

    switch_selector() models these switch events. Each selector (X and Y)
    has a small pool of timers so overlapping switch transitions are
    delivered in chronological order instead of replacing one another.
    Digital mode schedules tPLH/tPHL from the new logic level; analog
    mode uses the direction of the voltage change to choose tPLH or tPHL
    and delivers the full analog value when the timer expires.

    Writes to the currently selected channel bypass the timing model and
    update the output immediately, cancelling any pending transition,
    since the output already tracks that channel directly.

    m_last_scheduled[] records the latest scheduled output value so
    redundant writes can be ignored. If the timer pool is exhausted
    (which should never occur in normal emulation), the oldest pending
    transition is committed immediately to free a slot.

    MODE_SOUND does not model switching delay. The output is simply the
    currently selected input (or silence if inhibited). Selector changes
    call m_stream->update() first so switching occurs at the correct
    audio sample boundary.

	Inhibiting a selector (inhibit_x_w()/inhibit_y_w()) does not force the
    output to zero. Instead the output freezes at its last value: channel
    switches (address_w()) and channel-value writes on the addressed
    channel are ignored while inhibited, for both MODE_DIGITAL and
    MODE_ANALOG. When inhibit is cleared, the output reconnects to the
    currently addressed channel through the normal switch_selector()
    slew-delay path, comparing against the frozen m_last_scheduled value.

	MODE_SOUND crossfade:
    set_sound_crossfade() configures a per-selector linear ramp applied on
    channel switches and inhibit transitions, including to/from silence.
    Default is zero (instant switch, matching the real CMOS switch's negligible audio-band settling).
    Datasheet-quoted switching slew is 150ns; drivers modeling audible click
    suppression (e.g. a DAC sharing this mux with an inhibited-during-scan
    joystick comparator) can set that as a starting value. The fade snapshots
    the actual current output sample (not either fixed endpoint) when
    started, so a switch/inhibit event that interrupts an in-progress fade
    restarts cleanly from the true midpoint rather than jumping.

***************************************************************************/

#include "emu.h"
#include "mc14529.h"

//#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(MC14529, mc14529_device, "mc14529", "MC14529 Dual 4-Channel Analog Data Selector")

mc14529_device::mc14529_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, MC14529, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_write_z{ { *this }, { *this } }
	, m_write_z_analog{ { *this }, { *this } }
	, m_stream(nullptr)
	, m_tplh(attotime::from_nsec(150))
	, m_tphl(attotime::from_nsec(150))
	, m_address(0)
{
	std::fill(std::begin(m_mode), std::end(m_mode), MODE_DIGITAL);
	std::fill(std::begin(m_width_mask), std::end(m_width_mask), 0xff);
	std::fill(std::begin(m_inhibit), std::end(m_inhibit), 0);
	std::fill(std::begin(m_current_output), std::end(m_current_output), 0);
	std::fill(std::begin(m_last_scheduled), std::end(m_last_scheduled), 0);
	std::fill(std::begin(m_pending_next), std::end(m_pending_next), 0);

	std::fill(std::begin(m_sound_crossfade), std::end(m_sound_crossfade), attotime::zero);
	std::fill(std::begin(m_sound_current_source), std::end(m_sound_current_source), SOUND_SOURCE_SILENCE);
	std::fill(std::begin(m_sound_fade_active), std::end(m_sound_fade_active), false);
	std::fill(std::begin(m_sound_fade_from_sample), std::end(m_sound_fade_from_sample), 0.0);
	std::fill(std::begin(m_sound_fade_target), std::end(m_sound_fade_target), SOUND_SOURCE_SILENCE);
	std::fill(std::begin(m_sound_fade_samples_total), std::end(m_sound_fade_samples_total), 0);
	std::fill(std::begin(m_sound_fade_samples_done), std::end(m_sound_fade_samples_done), 0);

	for (auto &sel : m_channel)
		std::fill(std::begin(sel), std::end(sel), 0);
	for (auto &sel : m_channel_value)
		std::fill(std::begin(sel), std::end(sel), 0);

	for (unsigned s = 0; s < NUM_SELECTORS; s++)
	{
		std::fill(std::begin(m_pending_timer[s]), std::end(m_pending_timer[s]), nullptr);
		std::fill(std::begin(m_pending_value[s]), std::end(m_pending_value[s]), 0);
		std::fill(std::begin(m_pending_active[s]), std::end(m_pending_active[s]), false);
	}
}

mc14529_device &mc14529_device::set_propagation_delay(const attotime &tplh, const attotime &tphl)
{
	m_tplh = tplh;
	m_tphl = tphl;
	return *this;
}

mc14529_device &mc14529_device::set_sound_crossfade(unsigned selector, const attotime &time)
{
	m_sound_crossfade[selector] = time;
	return *this;
}

mc14529_device &mc14529_device::set_mode(unsigned selector, mode_t mode)
{
	m_mode[selector] = mode;
	return *this;
}

mc14529_device &mc14529_device::set_analog_width(unsigned selector, unsigned bits)
{
	m_width_mask[selector] = u8((1u << bits) - 1);
	return *this;
}

void mc14529_device::device_start()
{
	// Always allocate the sound stream (4 inputs per selector, 1 output
	// per selector). Harmless if neither selector uses MODE_SOUND -- the
	// outputs are simply filled with silence and never routed anywhere.
	m_stream = stream_alloc(NUM_SELECTORS * NUM_CHANNELS, NUM_SELECTORS, SAMPLE_RATE_INPUT_ADAPTIVE);

	for (unsigned s = 0; s < NUM_SELECTORS; s++)
		for (unsigned slot = 0; slot < MAX_PENDING; slot++)
			m_pending_timer[s][slot] = timer_alloc(FUNC(mc14529_device::delay_expired), this);

	save_item(NAME(m_channel));
	save_item(NAME(m_channel_value));
	save_item(NAME(m_address));
	save_item(NAME(m_inhibit));
	save_item(NAME(m_current_output));
	save_item(NAME(m_last_scheduled));
	save_item(NAME(m_pending_value));
	save_item(NAME(m_pending_active));
	save_item(NAME(m_pending_next));
	// m_mode / m_width_mask are configuration set once at machine-config time,
	// not runtime state, so they are not saved.

	save_item(NAME(m_sound_current_source));
	save_item(NAME(m_sound_fade_active));
	save_item(NAME(m_sound_fade_from_sample));
	save_item(NAME(m_sound_fade_target));
	save_item(NAME(m_sound_fade_samples_total));
	save_item(NAME(m_sound_fade_samples_done));
	// m_sound_crossfade is configuration set at machine-config time, not
	// runtime state, so it is not saved (same rationale as m_mode/m_width_mask).
}

void mc14529_device::device_reset()
{
	m_address = 0;
	std::fill(std::begin(m_inhibit), std::end(m_inhibit), 0);

	for (auto &sel : m_channel)
		std::fill(std::begin(sel), std::end(sel), 0);
	for (auto &sel : m_channel_value)
		std::fill(std::begin(sel), std::end(sel), 0);

	for (unsigned s = 0; s < NUM_SELECTORS; s++)
	{
		for (unsigned slot = 0; slot < MAX_PENDING; slot++)
		{
			m_pending_timer[s][slot]->adjust(attotime::never);
			m_pending_active[s][slot] = false;
			m_pending_value[s][slot] = 0;
		}

		m_pending_next[s] = 0;
		m_current_output[s] = 0;
		m_last_scheduled[s] = 0;

		if (m_mode[s] == MODE_ANALOG)
			m_write_z_analog[s](0);
		else if (m_mode[s] == MODE_DIGITAL)
			m_write_z[s](0);

		m_sound_fade_active[s] = false;
		m_sound_fade_samples_total[s] = 0;
		m_sound_fade_samples_done[s] = 0;
		m_sound_current_source[s] = m_inhibit[s] ? SOUND_SOURCE_SILENCE : m_address;
	}
}

void mc14529_device::sound_stream_update(sound_stream &stream)
{
	for (unsigned selector = 0; selector < NUM_SELECTORS; selector++)
	{
		if (m_mode[selector] != MODE_SOUND)
		{
			stream.fill(selector, 0);
			continue;
		}

		if (!m_sound_fade_active[selector])
		{
			u8 const src = m_sound_current_source[selector];
			if (src == SOUND_SOURCE_SILENCE)
				stream.fill(selector, 0);
			else
				stream.copy(selector, selector * NUM_CHANNELS + src);
			continue;
		}

		// Mid-fade: blend sample-by-sample, linear ramp.
		u8 const target = m_sound_fade_target[selector];
		unsigned const target_input = selector * NUM_CHANNELS + target;
		s32 const n = stream.samples();
		s32 i = 0;

		for (; i < n; i++)
		{
			sound_stream::sample_t const to = (target == SOUND_SOURCE_SILENCE)
				? sound_stream::sample_t(0.0) : stream.get(target_input, i);
			sound_stream::sample_t const frac = sound_stream::sample_t(m_sound_fade_samples_done[selector])
				/ sound_stream::sample_t(m_sound_fade_samples_total[selector]);

			stream.put(selector, i, m_sound_fade_from_sample[selector] + (to - m_sound_fade_from_sample[selector]) * frac);

			if (++m_sound_fade_samples_done[selector] >= m_sound_fade_samples_total[selector])
			{
				m_sound_fade_active[selector] = false;
				m_sound_current_source[selector] = target;
				i++; // fade finished on this sample; remainder handled below
				break;
			}
		}

		// Fade finished partway through this buffer -- fill the rest settled.
		if (!m_sound_fade_active[selector] && i < n)
		{
			if (target == SOUND_SOURCE_SILENCE)
				stream.fill(selector, 0, i);
			else
				stream.copy(selector, target_input, i);
		}
	}
}

void mc14529_device::commit(unsigned selector, unsigned slot)
{
	m_pending_active[selector][slot] = false;
	m_current_output[selector] = m_pending_value[selector][slot];

	if (m_mode[selector] == MODE_ANALOG)
	{
		m_write_z_analog[selector](m_current_output[selector]);
	}
	else if (m_mode[selector] == MODE_DIGITAL)
		m_write_z[selector](!!m_current_output[selector]);
	// MODE_SOUND does not use commit()/the timer pool at all.
}

void mc14529_device::switch_selector(unsigned selector)
{
	// Called when the channel connected to this selector's output changes
	// (address_w() picked a different channel, or inhibit_x_w()/
	// inhibit_y_w() connected/disconnected the output). This is the only
	// event that incurs the part's slew delay.

if (m_mode[selector] == MODE_SOUND)
	{
		u8 const target = m_inhibit[selector] ? SOUND_SOURCE_SILENCE : m_address;

		if (target == m_sound_current_source[selector] && !m_sound_fade_active[selector])
			return; // already settled here, nothing to do

		if (target == m_sound_fade_target[selector] && m_sound_fade_active[selector])
			return; // already fading to this target

		if (m_stream)
			m_stream->update(); // flush output up to now before (re)starting the fade

		if (m_sound_crossfade[selector] == attotime::zero)
		{
			// Instant switch: original behavior, unchanged.
			m_sound_current_source[selector] = target;
			m_sound_fade_active[selector] = false;
			return;
		}

		// Snapshot the actual last-written output sample -- if a fade was
		// already in progress, get_output() returns the true blended value
		// at this instant (not either endpoint), so an interrupted fade
		// restarts cleanly from its own midpoint rather than jumping.
		s32 const last = m_stream->samples() > 0 ? m_stream->samples() - 1 : 0;
		m_sound_fade_from_sample[selector] = m_stream->samples() > 0
			? m_stream->get_output(selector, last) : sound_stream::sample_t(0.0);

		m_sound_fade_target[selector] = target;
		m_sound_fade_samples_total[selector] = std::max<u32>(1,
			u32(m_sound_crossfade[selector].as_double() * m_stream->sample_rate()));
		m_sound_fade_samples_done[selector] = 0;
		m_sound_fade_active[selector] = true;
		return;
	}

	if (m_inhibit[selector])
		return; // frozen: output holds its last value until un-inhibited

	u8 new_output;

	if (m_mode[selector] == MODE_ANALOG)
		new_output = m_channel_value[selector][m_address] & m_width_mask[selector];
	else
		new_output = !!m_channel[selector][m_address];

	if (new_output == m_last_scheduled[selector])
		return; // outcome unchanged; nothing new to queue

	unsigned const slot = m_pending_next[selector];

	if (m_pending_active[selector][slot])
	{
		// pool exhausted
		osd_printf_error("mc14529: selector %u transition pool exhausted, forcing early commit of slot %u\n", selector, slot);
		m_pending_timer[selector][slot]->adjust(attotime::never);
		commit(selector, slot);
	}

	// Direction of the swing on the real analog node: for MODE_DIGITAL the
	// bit value itself gives the direction; for MODE_ANALOG we compare
	// against the previously scheduled quantized level.
	bool const rising = (m_mode[selector] == MODE_ANALOG)
		? (new_output > m_last_scheduled[selector])
		: (new_output != 0);

	attotime const delay = rising ? m_tplh : m_tphl;

	m_pending_value[selector][slot] = new_output;
	m_pending_active[selector][slot] = true;
	m_pending_timer[selector][slot]->adjust(delay, selector * MAX_PENDING + slot);

	m_last_scheduled[selector] = new_output;
	m_pending_next[selector] = (slot + 1) % MAX_PENDING;
}

void mc14529_device::update_active_value(unsigned selector)
{
	// Called when the data on the channel that is *already* selected
	// changes value (x_w()/y_w()/x_analog_w()/y_analog_w() for the
	// currently addressed channel). No switching event occurred, so real
	// propagation to the output is fast enough to treat as zero delay:
	// apply it immediately instead of going through the timer pool.

	if (m_mode[selector] == MODE_SOUND)
		return; // handled entirely in sound_stream_update()

	if (m_inhibit[selector])
		return; // frozen: ignore changes on the active channel while inhibited

	u8 new_output;

	if (m_mode[selector] == MODE_ANALOG)
		new_output = m_channel_value[selector][m_address] & m_width_mask[selector];
	else
		new_output = !!m_channel[selector][m_address];

	if (new_output == m_last_scheduled[selector])
		return; // outcome unchanged

	// The switch is already closed on this channel, so this value change
	// cannot still be "mid-slew" from some other channel -- any switch
	// transition still settling for this selector is now stale and is
	// superseded by this immediate update.
	for (unsigned slot = 0; slot < MAX_PENDING; slot++)
	{
		if (m_pending_active[selector][slot])
		{
			m_pending_timer[selector][slot]->adjust(attotime::never);
			m_pending_active[selector][slot] = false;
		}
	}

	m_last_scheduled[selector] = new_output;
	m_current_output[selector] = new_output;

	if (m_mode[selector] == MODE_ANALOG)
		m_write_z_analog[selector](new_output);
	else
		m_write_z[selector](!!new_output);
}

TIMER_CALLBACK_MEMBER(mc14529_device::delay_expired)
{
	unsigned const selector = param / MAX_PENDING;
	unsigned const slot = param % MAX_PENDING;
	commit(selector, slot);
}

u8 mc14529_device::zx_value()
{
	return m_current_output[SEL_X];
}

u8 mc14529_device::zy_value()
{
	return m_current_output[SEL_Y];
}

void mc14529_device::address_w(int bit, int state)
{
	state = !!state;
	u8 const mask = u8(1) << bit;
	u8 const new_address = state ? (m_address | mask) : (m_address & ~mask);

	if (new_address == m_address)
		return;

	if (m_stream)
		m_stream->update(); // flush MODE_SOUND output(s) at the old address before switching

	m_address = new_address;
	switch_selector(SEL_X);
	switch_selector(SEL_Y);
}

void mc14529_device::inhibit_x_w(int state)
{
	state = !!state;
	if (m_inhibit[SEL_X] == state)
		return;

	if (m_stream)
		m_stream->update();

	m_inhibit[SEL_X] = state;
	switch_selector(SEL_X);
}

void mc14529_device::inhibit_y_w(int state)
{
	state = !!state;
	if (m_inhibit[SEL_Y] == state)
		return;

	if (m_stream)
		m_stream->update();

	m_inhibit[SEL_Y] = state;
	switch_selector(SEL_Y);
}

void mc14529_device::x_w(int channel, int state)
{
	state = !!state;
	if (m_channel[SEL_X][channel] == state)
		return;
	m_channel[SEL_X][channel] = state;
	if (m_mode[SEL_X] == MODE_DIGITAL && m_address == channel)
		update_active_value(SEL_X);
}

void mc14529_device::y_w(int channel, int state)
{
	state = !!state;
	if (m_channel[SEL_Y][channel] == state)
		return;
	m_channel[SEL_Y][channel] = state;
	if (m_mode[SEL_Y] == MODE_DIGITAL && m_address == channel)
		update_active_value(SEL_Y);
}

void mc14529_device::x_analog_w(int channel, u8 value)
{
	value &= m_width_mask[SEL_X];
	if (m_channel_value[SEL_X][channel] == value)
		return;
	m_channel_value[SEL_X][channel] = value;
	if (m_mode[SEL_X] == MODE_ANALOG && m_address == channel)
		update_active_value(SEL_X);
}

void mc14529_device::y_analog_w(int channel, u8 value)
{
	value &= m_width_mask[SEL_Y];
	if (m_channel_value[SEL_Y][channel] == value)
		return;
	m_channel_value[SEL_Y][channel] = value;
	if (m_mode[SEL_Y] == MODE_ANALOG && m_address == channel)
		update_active_value(SEL_Y);
}
