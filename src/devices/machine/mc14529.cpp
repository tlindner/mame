// license:BSD-3-Clause
// copyright-holders:Tim Lindner
/***************************************************************************

    mc14529.cpp

    See mc14529.h for details.

    MODE_DIGITAL / MODE_ANALOG timing model: the delay this part exhibits
    is a switch-slew delay, not a signal-propagation delay. Propagation
    through an already-closed switch is fast enough to be modeled as
    zero. The delay only applies at the moment a selector's output
    switches to a different channel -- i.e. an address_w() or
    inhibit_x_w()/inhibit_y_w() call that changes what is connected to
    the output -- because that's when the output node has to slew from
    its old level to the newly selected channel's level. A data write to
    the channel that is already selected (x_w()/y_w(),
    x_analog_w()/y_analog_w()) changes the output immediately, with no
    delay, since no switching event occurred.

    Channel-switch events are handled by switch_selector(). Each of the
    two independent selectors (X and Y) owns a small fixed-size pool of
    timers (MAX_PENDING slots) for this. Every switch that changes what
    that selector's output will eventually settle to allocates the next
    slot (round-robin) and starts a timer:

      MODE_DIGITAL -- tPLH if the new bit is 1, tPHL if it is 0.

      MODE_ANALOG  -- tPLH if the new quantized value is numerically
        greater than the previously scheduled one (rising swing on the
        real analog node), tPHL if it is lower (falling swing). The full
        multi-bit value is carried as the pending payload and delivered
        as one unit when its timer fires.

    Because MAME fires timers in absolute time order regardless of
    allocation order, multiple overlapping in-flight switch transitions
    still reach the output in the correct chronological sequence in
    either mode -- an earlier-triggered transition is not clobbered or
    dropped by a later one the way a single-timer "restart on change"
    model would do.

    Data writes to the already-selected channel are handled by
    update_active_value(), which applies the new value to the output at
    once and cancels/supersedes any switch transition still settling for
    that selector -- physically, once the switch is closed on a channel,
    the output tracks that channel's voltage immediately, so a value
    change there can't still be "mid-slew" from the old channel.

    m_last_scheduled[] tracks the target value of the most recently
    queued switch transition (or, if none pending, the most recently
    committed/immediate value), so a redundant write that doesn't
    actually change the outcome is a no-op.

    Pool exhaustion: if more than MAX_PENDING switch transitions for a
    selector are in flight at once, the oldest still-pending transition
    in that selector's pool is committed immediately (with a logerror
    warning) to free its slot. This should never happen in practice at
    emulated 6809-bus speeds against a ~200 ns worst-case delay, but is
    handled rather than left as undefined behavior.

    MODE_SOUND model: no timer pool is used. Each selector's audio output
    is just whichever input channel (0-3) is currently addressed for it,
    copied sample-for-sample, or silence if that selector is inhibited.
    address_w()/inhibit_x_w()/inhibit_y_w() call m_stream->update() before
    changing any state that a MODE_SOUND selector depends on, so the
    switch-over happens at the correct sample boundary rather than being
    deferred to the end of the current audio update -- this is what makes
    the mode delay-free while still being sample-accurate.

***************************************************************************/

#include "emu.h"
#include "mc14529.h"

#define LOG_GENERAL (1U << 0)
#define LOG_READS   (1U << 1)
//#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"

#define LOGREADS(...)   LOGMASKED(LOG_READS,    __VA_ARGS__)

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
	// devcb callbacks self-resolve; no explicit resolve_safe() call needed.

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

}

void mc14529_device::device_resolve_objects()
{
    // Callbacks are fully resolved by the time this method runs
    LOG("mc14529_device::device_resolve_objects\n");
    if (m_write_z_analog[0].isunset()) LOG("m_write_z_analog[0] unset\n");
    if (m_write_z_analog[1].isunset()) LOG("m_write_z_analog[1] unset\n");
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

		if (m_inhibit[selector])
			stream.fill(selector, 0);
		else
			stream.copy(selector, selector * NUM_CHANNELS + m_address);
	}
}

void mc14529_device::commit(unsigned selector, unsigned slot)
{
	m_pending_active[selector][slot] = false;
	m_current_output[selector] = m_pending_value[selector][slot];

	LOG("Timed analog update: selector: %d, data: %d (mt: %s)\n", selector, m_current_output[selector],
	machine().time().to_string().c_str());

	if (m_mode[selector] == MODE_ANALOG)
	{
		m_write_z_analog[selector](m_current_output[selector]);
	}
	else if (m_mode[selector] == MODE_DIGITAL)
		m_write_z[selector](m_current_output[selector] ? 1 : 0);
	// MODE_SOUND does not use commit()/the timer pool at all.
}

void mc14529_device::switch_selector(unsigned selector)
{
	// Called when the channel connected to this selector's output changes
	// (address_w() picked a different channel, or inhibit_x_w()/
	// inhibit_y_w() connected/disconnected the output). This is the only
	// event that incurs the part's slew delay.

	if (m_mode[selector] == MODE_SOUND)
		return; // handled entirely in sound_stream_update()

	u8 new_output;

	if (m_inhibit[selector])
		new_output = 0;
	else if (m_mode[selector] == MODE_ANALOG)
		new_output = m_channel_value[selector][m_address] & m_width_mask[selector];
	else
		new_output = m_channel[selector][m_address] ? 1 : 0;

	if (new_output == m_last_scheduled[selector])
		return; // outcome unchanged; nothing new to queue

	unsigned const slot = m_pending_next[selector];

	if (m_pending_active[selector][slot])
	{
		// pool exhausted: transitions are arriving faster than MAX_PENDING
		// outstanding edges can track at this part's propagation delay.
		// Force the oldest pending transition in this selector's pool to
		// complete now so its slot can be reused.
		logerror("mc14529: selector %u transition pool exhausted, forcing early commit of slot %u\n", selector, slot);
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

	u8 new_output;

	if (m_inhibit[selector])
		new_output = 0;
	else if (m_mode[selector] == MODE_ANALOG)
		new_output = m_channel_value[selector][m_address] & m_width_mask[selector];
	else
		new_output = m_channel[selector][m_address] ? 1 : 0;

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

	LOG("Immediate analog update: selector: %d, data: %d (mt: %s)\n", selector, new_output,
	machine().time().to_string().c_str());
	if (m_mode[selector] == MODE_ANALOG)
		m_write_z_analog[selector](new_output);
	else
		m_write_z[selector](new_output ? 1 : 0);
}

TIMER_CALLBACK_MEMBER(mc14529_device::delay_expired)
{
	unsigned const selector = param / MAX_PENDING;
	unsigned const slot = param % MAX_PENDING;
	commit(selector, slot);
}

u8 mc14529_device::zx_value()
{
	LOGREADS("read x value: %d\n", m_current_output[SEL_X]);
	return m_current_output[SEL_X];
}

u8 mc14529_device::zy_value()
{
	LOGREADS("read y value: %d\n", m_current_output[SEL_Y]);
	return m_current_output[SEL_Y];
}

void mc14529_device::address_w(int bit, int state)
{
	state = state ? 1 : 0;
	u8 const mask = u8(1) << bit;
	u8 const new_address = state ? (m_address | mask) : (m_address & ~mask);

	LOG("address_w: bit: %d, state: %d, new address: %d\n", bit, state, new_address);

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
	state = state ? 1 : 0;
	if (m_inhibit[SEL_X] == state)
		return;

	if (m_stream)
		m_stream->update();

	m_inhibit[SEL_X] = state;
	switch_selector(SEL_X);
}

void mc14529_device::inhibit_y_w(int state)
{
	state = state ? 1 : 0;
	if (m_inhibit[SEL_Y] == state)
		return;

	if (m_stream)
		m_stream->update();

	m_inhibit[SEL_Y] = state;
	switch_selector(SEL_Y);
}

void mc14529_device::x_w(int channel, int state)
{
	state = state ? 1 : 0;
	if (m_channel[SEL_X][channel] == state)
		return;
	m_channel[SEL_X][channel] = state;
	if (m_mode[SEL_X] == MODE_DIGITAL && m_address == channel)
		update_active_value(SEL_X);
}

void mc14529_device::y_w(int channel, int state)
{
	state = state ? 1 : 0;
	if (m_channel[SEL_Y][channel] == state)
		return;
	m_channel[SEL_Y][channel] = state;
	if (m_mode[SEL_Y] == MODE_DIGITAL && m_address == channel)
		update_active_value(SEL_Y);
}

void mc14529_device::x_analog_w(int channel, u8 value)
{
	LOG("Mux set analog: channel %d, value: %d\n", channel, value);

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
