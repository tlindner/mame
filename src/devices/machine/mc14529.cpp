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

#define LOG_SWITCH   (1U << 1) // Shows register setup
#define LOG_TIMER   (1U << 1) // Shows register setup
#define VERBOSE (LOG_SWITCH|LOG_TIMER)
#include "logmacro.h"
#define LOGSWITCH(...)   LOGMASKED(LOG_SWITCH,    __VA_ARGS__)
#define LOGTIMER(...)   LOGMASKED(LOG_TIMER,    __VA_ARGS__)

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
	m_pending_next = 0;

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
		std::fill(std::begin(m_pending_value[s]), std::end(m_pending_value[s]), 0);
		std::fill(std::begin(m_last_scheduled[s]), std::end(m_last_scheduled[s]), 0);

		m_capture_last_sample[s] = false;
		m_last_stream_sampe[s] = 0.0;
	}

	std::fill(std::begin(m_pending_active), std::end(m_pending_active), false);
	std::fill(std::begin(m_pending_timer_new), std::end(m_pending_timer_new), nullptr);
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

	for (unsigned slot = 0; slot < MAX_PENDING; slot++)
		m_pending_timer_new[slot] = timer_alloc(FUNC(mc14529_device::delay_expired), this);

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
	save_item(NAME(m_capture_last_sample));
	save_item(NAME(m_last_stream_sampe));
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
		for (unsigned chan = 0; chan < NUM_CHANNELS; chan++)
		{
			m_pending_value[s][chan] = 0;
			m_last_scheduled[s][chan] = 0;

		}

		m_current_output[s] = 0;

		if (m_mode[s] == MODE_ANALOG)
			m_write_z_analog[s](0);
		else if (m_mode[s] == MODE_DIGITAL)
			m_write_z[s](0);

		m_sound_fade_active[s] = false;
		m_sound_fade_samples_total[s] = 0;
		m_sound_fade_samples_done[s] = 0;
		m_sound_current_source[s] = m_inhibit[s] ? SOUND_SOURCE_SILENCE : m_address;
	}

	for (unsigned slot = 0; slot < MAX_PENDING; slot++)
	{
		m_pending_timer_new[slot]->adjust(attotime::never);
		m_pending_active[slot] = false;
	}

	m_pending_next = 0;
}

void mc14529_device::sound_stream_update(sound_stream &stream)
{
	for (unsigned selector = 0; selector < NUM_SELECTORS; selector++)
	{
		sound_stream::sample_t last_sample;
		s32 const n = stream.samples();
		s32 i = 0;

		// Mid-fade: blend sample-by-sample, linear ramp.
		u8 const target = m_sound_fade_target[selector];
		unsigned const target_input = selector * NUM_CHANNELS + target;

		if (m_mode[selector] != MODE_SOUND)
		{
			stream.fill(selector, 0);
			goto clean_up;
		}

		if (!m_sound_fade_active[selector])
		{
			u8 const src = m_sound_current_source[selector];
			if (src == SOUND_SOURCE_SILENCE)
				stream.fill(selector, m_last_stream_sampe[selector]);
			else
				stream.copy(selector, selector * NUM_CHANNELS + src);

			goto clean_up;
		}

		for (; i < n; i++)
		{
			sound_stream::sample_t const to = (target == SOUND_SOURCE_SILENCE)
				? m_last_stream_sampe[selector] : stream.get(target_input, i);
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
				stream.fill(selector, m_last_stream_sampe[selector], i);
			else
				stream.copy(selector, target_input, i);
		}


clean_up:
		last_sample = stream.get_output(selector, n - 1);
		if (m_capture_last_sample[selector])
		{
			m_capture_last_sample[selector] = false;
			m_last_stream_sampe[selector] = last_sample;
		}
	}
}

void mc14529_device::switch_selector(unsigned address)
{
	m_address == address;

	if (m_stream)
		m_stream->update(); // flush MODE_SOUND output(s) at the old address before switching

	for (unsigned selector = 0; selector < NUM_SELECTORS; selector++)
	{
		if (m_inhibit[selector])
			continue; // frozen: output holds its last value until un-inhibited

		if (m_mode[selector] == MODE_ANALOG)
		{
			u8 new_value;
			m_current_output[selector] = m_channel_value[selector][m_address];
			new_value = m_current_output[selector];
			m_write_z_analog[selector](new_value);
		}
		else if (m_mode[selector] == MODE_DIGITAL)
		{
			int new_value;
			m_current_output[selector] = m_channel[selector][m_address];
			new_value = m_current_output[selector];
			m_write_z[selector](new_value);
		}
	}
}

void mc14529_device::queue_switch_update(unsigned new_address)
{
	if (m_last_queued_address == new_address)
		return;

	m_last_queued_address = new_address;

	for(int selector=0; selector<NUM_SELECTORS; selector++)
	{
		if (m_mode[selector] == MODE_SOUND)
		{
			u8 const target = m_inhibit[selector] ? SOUND_SOURCE_SILENCE : m_address;

			if (target == m_sound_current_source[selector] && !m_sound_fade_active[selector])
				continue; // already settled here, nothing to do

			if (target == m_sound_fade_target[selector] && m_sound_fade_active[selector])
				continue; // already fading to this target

			if (m_stream)
				m_stream->update(); // flush output up to now before (re)starting the fade

			if (m_sound_crossfade[selector] == attotime::zero)
			{
				// Instant switch: original behavior, unchanged.
				m_sound_current_source[selector] = target;
				m_sound_fade_active[selector] = false;
				continue;
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

			continue;
		}
	}

	unsigned slot = m_pending_next;
	if (m_pending_active[slot])
	{
		// pool exhausted
		osd_printf_error("mc14529: transition pool exhausted, forcing early commit of slot %u\n", slot);
		m_pending_timer_new[slot]->adjust(attotime::never);
		delay_expired(m_pending_timer_new[slot]->param());
	}

	s32 param = pack(slot, 0, 0, true /* switch */, new_address);
	attotime const delay = (m_tplh + m_tphl) / 2.0;
	m_pending_active[slot] = true;
	m_pending_timer_new[slot]->adjust(delay, param);
	LOGSWITCH("queued switch change: slot %d\n", slot);
	m_pending_next = (slot + 1) % MAX_PENDING;
}

void mc14529_device::queue_value_update(unsigned selector, unsigned channel, u8 value)
{
	if (m_mode[selector] == MODE_SOUND)
		return;

	if (m_last_scheduled[selector][channel] == value)
		return;

	unsigned const slot = m_pending_next;

	if (m_pending_active[slot])
	{
		// pool exhausted
		osd_printf_error("mc14529: selector %u transition pool exhausted, forcing early commit of slot %u\n", selector, slot);
		m_pending_timer_new[slot]->adjust(attotime::never);
		delay_expired(m_pending_timer_new[slot]->param());
	}

	bool const rising = (m_mode[selector] == MODE_ANALOG) ? (value > m_last_scheduled[selector][slot]) : (value != 0);
	attotime const delay = rising ? m_tplh : m_tphl;

	m_pending_active[slot] = true;
	m_pending_timer_new[slot]->adjust(delay, pack(slot, selector, channel, false /* not switch */, value));
	m_last_scheduled[selector][channel] = value;
	m_pending_next = (slot + 1) % MAX_PENDING;
}

TIMER_CALLBACK_MEMBER(mc14529_device::delay_expired)
{
	m_pending_active[unpack_slot(param)] = false;

	LOGTIMER("delay_expired: slot: %d, switch: %d, selector: %d, channel: %d, value: %d (%11.6f)\n",
		unpack_slot(param), unpack_switch(param), unpack_selector(param), unpack_channel(param),
		unpack_value(param), machine().time().as_double());

	if (unpack_switch(param))
	{
		// this is a switch event
		switch_selector(unpack_value(param));
	}
	else
	{
		unsigned selector = unpack_selector(param);
		if (m_mode[selector] == MODE_SOUND)
			return;

		unsigned channel = unpack_channel(param);
		uint8_t value = unpack_value(param);

		m_channel_value[selector][channel] = value;

		if(m_inhibit[selector] == false)
		{
			if (m_address == channel)
			{
				m_current_output[selector] = value;

				if (m_mode[selector] == MODE_ANALOG)
					m_write_z_analog[selector](value);
				else if (m_mode[selector] == MODE_DIGITAL)
					m_write_z[selector](value);
			}
		}
	}
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
	u8 const new_address = state ? (m_last_queued_address | mask) : (m_last_queued_address & ~mask);

	LOGSWITCH("address_w: bit: %d, state: %d, new_address: %d (%11.6f)\n", bit, state, new_address, machine().time().as_double());

	queue_switch_update(new_address);
}

void mc14529_device::inhibit_x_w(int state)
{
	state = !!state;

	LOGSWITCH("inhibit_x_w: state: %d (%11.6f)\n", state, machine().time().as_double());

	if (m_inhibit[SEL_X] == state)
		return;

	m_capture_last_sample[SEL_X] = state;

	if (m_stream)
		m_stream->update();

	m_inhibit[SEL_X] = state;

	switch_selector(m_address);
}

void mc14529_device::inhibit_y_w(int state)
{
	state = !!state;

	LOGSWITCH("inhibit_y_w: state: %d (%11.6f)\n", state, machine().time().as_double());

	if (m_inhibit[SEL_Y] == state)
		return;

	m_capture_last_sample[SEL_Y] = state;

	if (m_stream)
		m_stream->update();

	m_inhibit[SEL_Y] = state;
	switch_selector(m_address);
}

void mc14529_device::x_w(int channel, int state)
{
	state = !!state;
	queue_value_update(SEL_X, channel, state);
}

void mc14529_device::y_w(int channel, int state)
{
	state = !!state;
	queue_value_update(SEL_Y, channel, state);
}

void mc14529_device::x_analog_w(int channel, u8 value)
{
	value &= m_width_mask[SEL_X];
	LOGSWITCH("x_analog_w: channel: %d, value: %d (%11.6f)\n", channel, value, machine().time().as_double());
	queue_value_update(SEL_X, channel, value);
}

void mc14529_device::y_analog_w(int channel, u8 value)
{
	value &= m_width_mask[SEL_X];
	queue_value_update(SEL_Y, channel, value);
}

int32_t mc14529_device::pack(unsigned slot, unsigned selector, unsigned channel, bool sw, uint8_t value)
{
	if (slot >= MAX_PENDING ||
		selector >= NUM_SELECTORS ||
		channel >= NUM_CHANNELS)
		return -1;

	return
		((slot     & SLOT_MASK)     << SLOT_SHIFT)     |
		((selector & SELECTOR_MASK) << SELECTOR_SHIFT) |
		((sw       & SWITCH_MASK)   << SWITCH_SHIFT)   |
		((channel  & CHANNEL_MASK)  << CHANNEL_SHIFT)  |
		((value    & VALUE_MASK)    << VALUE_SHIFT);
}

unsigned mc14529_device::unpack_selector(int32_t packed)
{
	return (packed >> SELECTOR_SHIFT) & SELECTOR_MASK;
}

unsigned mc14529_device::unpack_switch(int32_t packed)
{
	return (packed >> SWITCH_SHIFT) & SWITCH_MASK;
}

unsigned mc14529_device::unpack_channel(int32_t packed)
{
	return (packed >> CHANNEL_SHIFT) & CHANNEL_MASK;
}

uint8_t mc14529_device::unpack_value(int32_t packed)
{
	return (packed >> VALUE_SHIFT) & VALUE_MASK;
}

unsigned mc14529_device::unpack_slot(int32_t packed)
{
	return (packed >> SLOT_SHIFT) & SLOT_MASK;
}
