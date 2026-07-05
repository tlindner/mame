// license:BSD-3-Clause
// copyright-holders:Tim Lindner
/***************************************************************************

    mc14529.cpp

    See mc14529.h for details.

    Timing model: each of the two independent selectors (X and Y) owns a
    small fixed-size pool of timers (MAX_PENDING slots), regardless of
    which mode that selector is running in. Every input edge that changes
    what that selector's output will eventually settle to allocates the
    next slot (round-robin) and starts a timer:

      MODE_DIGITAL -- tPLH if the new bit is 1, tPHL if it is 0.

      MODE_ANALOG  -- tPLH if the new quantized value is numerically
        greater than the previously scheduled one (rising swing on the
        real analog node), tPHL if it is lower (falling swing). The full
        multi-bit value is carried as the pending payload and delivered
        as one unit when its timer fires -- there is one physical node
        settling, not independent per-bit propagation.

    Because MAME fires timers in absolute time order regardless of
    allocation order, multiple overlapping in-flight transitions still
    reach the output in the correct chronological sequence in either
    mode -- an earlier-triggered transition is not clobbered or dropped
    by a later one the way a single-timer "restart on change" model
    would do.

    m_last_scheduled[] tracks the target value of the most recently
    queued (or, if none pending, most recently committed) transition, so
    a redundant write that doesn't actually change the pending outcome
    does not consume a pool slot.

    Pool exhaustion: if more than MAX_PENDING transitions for a selector
    are in flight at once, the oldest still-pending transition in that
    selector's pool is committed immediately (with a logerror warning)
    to free its slot. This should never happen in practice at emulated
    6809-bus speeds against a ~200 ns worst-case delay, but is handled
    rather than left as undefined behavior.

***************************************************************************/

#include "emu.h"
#include "mc14529.h"

DEFINE_DEVICE_TYPE(MC14529, mc14529_device, "mc14529", "MC14529 Dual 4-Channel Analog Data Selector (measured timing)")

mc14529_device::mc14529_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, MC14529, tag, owner, clock)
	, m_write_z{ { *this }, { *this } }
	, m_write_z_analog{ { *this }, { *this } }
	, m_tplh(attotime::from_nsec(94))
	, m_tphl(attotime::from_nsec(200))
	, m_address(0)
{
	std::fill(std::begin(m_mode), std::end(m_mode), MODE_DIGITAL);
	std::fill(std::begin(m_width_mask), std::end(m_width_mask), 0x3f); // default 6-bit
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
// 	for (auto &cb : m_write_z)
// 		cb.resolve_safe();
// 	for (auto &cb : m_write_z_analog)
// 		cb.resolve_safe();

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
		else
			m_write_z[s](0);
	}
}

void mc14529_device::commit(unsigned selector, unsigned slot)
{
	m_pending_active[selector][slot] = false;
	m_current_output[selector] = m_pending_value[selector][slot];

	if (m_mode[selector] == MODE_ANALOG)
		m_write_z_analog[selector](m_current_output[selector]);
	else
		m_write_z[selector](m_current_output[selector] ? 1 : 0);
}

void mc14529_device::update_selector(unsigned selector)
{
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

TIMER_CALLBACK_MEMBER(mc14529_device::delay_expired)
{
	unsigned const selector = param / MAX_PENDING;
	unsigned const slot = param % MAX_PENDING;
	commit(selector, slot);
}

void mc14529_device::address_w(int bit, int state)
{
	state = state ? 1 : 0;
	u8 const mask = 1 << bit;
	u8 const new_address = state ? (m_address | mask) : (m_address & ~mask);

	if (new_address == m_address)
		return;

	m_address = new_address;
	update_selector(SEL_X);
	update_selector(SEL_Y);
}

void mc14529_device::inhibit_x_w(int state)
{
	state = state ? 1 : 0;
	if (m_inhibit[SEL_X] == state)
		return;
	m_inhibit[SEL_X] = state;
	update_selector(SEL_X);
}

void mc14529_device::inhibit_y_w(int state)
{
	state = state ? 1 : 0;
	if (m_inhibit[SEL_Y] == state)
		return;
	m_inhibit[SEL_Y] = state;
	update_selector(SEL_Y);
}

void mc14529_device::x_w(int channel, int state)
{
	state = state ? 1 : 0;
	if (m_channel[SEL_X][channel] == state)
		return;
	m_channel[SEL_X][channel] = state;
	if (m_mode[SEL_X] == MODE_DIGITAL && m_address == channel)
		update_selector(SEL_X);
}

void mc14529_device::y_w(int channel, int state)
{
	state = state ? 1 : 0;
	if (m_channel[SEL_Y][channel] == state)
		return;
	m_channel[SEL_Y][channel] = state;
	if (m_mode[SEL_Y] == MODE_DIGITAL && m_address == channel)
		update_selector(SEL_Y);
}

void mc14529_device::x_analog_w(int channel, u8 value)
{
	value &= m_width_mask[SEL_X];
	if (m_channel_value[SEL_X][channel] == value)
		return;
	m_channel_value[SEL_X][channel] = value;
	if (m_mode[SEL_X] == MODE_ANALOG && m_address == channel)
		update_selector(SEL_X);
}

void mc14529_device::y_analog_w(int channel, u8 value)
{
	value &= m_width_mask[SEL_Y];
	if (m_channel_value[SEL_Y][channel] == value)
		return;
	m_channel_value[SEL_Y][channel] = value;
	if (m_mode[SEL_Y] == MODE_ANALOG && m_address == channel)
		update_selector(SEL_Y);
}
