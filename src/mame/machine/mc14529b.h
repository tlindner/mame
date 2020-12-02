// license:BSD-3-Clause
// copyright-holders:tim lindner
/**********************************************************************

    Motorola MC14529B Dual 4-Channel Analog Data Selector

***********************************************************************/

#ifndef MAME_MACHINE_MC14529B_H
#define MAME_MACHINE_MC14529B_H

#pragma once

class mc14529b_device : public device_t
{
public:
	// timers
	enum timer_id // indexes
	{
		TIMER_A1 = 0,
		TIMER_A2 = 1,
		TIMER_A3 = 2,
		TIMER_A4 = 3,
		TIMER_B1 = 4,
		TIMER_B2 = 5,
		TIMER_B3 = 6,
		TIMER_B4 = 7
	};

	enum soundmux_status_t
	{
		SOUNDMUX_SEL1 = 1,
		SOUNDMUX_SEL2 = 2,
		SOUNDMUX_ENABLE = 4
	};

	auto mux_output_handler() { return m_mux_output_handler.bind(); }

	// construction/destruction
	mc14529b_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	void selector_a_w(int state);
	void selector_b_w(int state);
	void mux_enable_w(int state);
	uint8_t mux_output();

protected:
	mc14529b_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	void set_timer(timer_id id_start, timer_id id_end, int state);

	attotime delay;
	int m_selector_a;
	int m_selector_b;
	int m_mux_enable;
	int m_mux_output;
	devcb_write8 m_mux_output_handler;
	emu_timer *m_delay_timer[8];
};

DECLARE_DEVICE_TYPE(MC14529B, mc14529b_device)

#endif // MAME_MACHINE_MC14529B_H
