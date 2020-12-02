// license:BSD-3-Clause
// copyright-holders:tim lindner
/**********************************************************************

    Motorola MC14529B Dual 4-Channel Analog Data Selector

***********************************************************************

	todo:
		Support configurable 10 and 15 volt propagation delays
		Support combined 8 channel mode

***********************************************************************/


#include "emu.h"
#include "mc14529b.h"

#include "logmacro.h"

//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

// device type definition
DEFINE_DEVICE_TYPE(MC14529B, mc14529b_device, "mc14529b", "MC14529B Dual 4-Channel Analog Data Selector")


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

mc14529b_device::mc14529b_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: mc14529b_device(mconfig, MC14529B, tag, owner, clock)
{
}

mc14529b_device::mc14529b_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, m_selector_a(0)
	, m_selector_b(0)
	, m_mux_enable(0)
	, m_mux_output(0)
	, m_mux_output_handler(*this)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void mc14529b_device::device_start()
{
	// resolve callbacks
	m_mux_output_handler.resolve_safe();

	for (int i = TIMER_A1; i <= TIMER_E4; i++)
	{
		m_delay_timer[i] = timer_alloc(i);
	}

	save_item(NAME(m_selector_a));
	save_item(NAME(m_selector_b));
	save_item(NAME(m_mux_enable));
	save_item(NAME(m_mux_output));
}


//-------------------------------------------------
//  device_timer - handler timer events
//-------------------------------------------------

void mc14529b_device::device_timer(emu_timer &timer, device_timer_id id, int32_t param, void *ptr)
{
	if( id <= TIMER_A4 )
	{
		m_selector_a = param;
	}
	else if( id <= TIMER_B4 )
	{
		m_selector_b = param;
	}
	else if( id <= TIMER_E4 )
	{
		m_mux_enable = param;
	}
	else
	{
		LOG("Unrecognized timer");
	}

	if( !m_mux_output_handler.isnull() )
	{
		m_mux_output_handler( (m_mux_enable << 2 ) | (m_selector_b << 1) | m_selector_a );
	}
}

//------------------------------------------------
//  mux_output - get current output
//------------------------------------------------

uint8_t mc14529b_device::mux_output()
{
	return( (m_mux_enable << 2 ) | (m_selector_b << 1) | m_selector_a );
}

//------------------------------------------------
//  selector_a_w - set b input line
//------------------------------------------------

void mc14529b_device::selector_a_w(int state)
{
	set_timer( TIMER_A1, TIMER_A4, state );
}


//------------------------------------------------
//  selector_b_w - set b input line
//------------------------------------------------

void mc14529b_device::selector_b_w(int state)
{
	set_timer( TIMER_B1, TIMER_B4, state );
}


//------------------------------------------------
//  mux_enable_w - set b input line
//------------------------------------------------

void mc14529b_device::mux_enable_w(int state)
{
	m_mux_enable = state;

	if( !m_mux_output_handler.isnull() )
	{
		m_mux_output_handler( (m_mux_enable << 2 ) | (m_selector_b << 1) | m_selector_a );
	}

// 	set_timer( TIMER_E1, TIMER_E4, state );
}


//------------------------------------------------
//  set_timer - set timer
//------------------------------------------------

void mc14529b_device::set_timer(timer_id id_start, timer_id id_end, int state)
{
	int i;
	for( i = id_start; i <= id_end; i++ )
	{
		if( !(m_delay_timer[i]->enabled()) )
		{
			// typical propagation delay (140ns typical, 400ns max)
			m_delay_timer[i]->adjust(attotime::from_nsec(350), state);
			break;
		}
	}

	if( i == id_end+1 )
	{
		LOG( "Timer %d enable overflow\n", id_start );
	}
}
