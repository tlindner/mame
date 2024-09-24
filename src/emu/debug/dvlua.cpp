// license:BSD-3-Clause
// copyright-holders:tim lindner
/*********************************************************************

    dvlua.cpp

    Generic debugger view controlled by a Lua script.

***************************************************************************/

#include "emu.h"
#include "dvlua.h"

#include "debugger.h"

//#include <algorithm>
//#include <cctype>
//#include <tuple>


//**************************************************************************
//  DEBUG VIEW LUA SOURCE
//**************************************************************************

//-------------------------------------------------
//  debug_view_lua_source - constructors
//-------------------------------------------------

debug_view_lua_source::debug_view_lua_source(std::string &&name, address_space &space)
	: debug_view_source(std::move(name), &space.device())
{
}



//**************************************************************************
//  DEBUG VIEW LUA
//**************************************************************************

//-------------------------------------------------
//  debug_view_lua - constructor
//-------------------------------------------------

debug_view_lua::debug_view_lua(running_machine &machine, debug_view_osd_update_func osdupdate, void *osdprivate)
	: debug_view(machine, DVT_LUA, osdupdate, osdprivate)
    , m_lua(std::make_unique<lua_engine>())
{
    m_lua->initialize();

    lua()->sol().set_function("view_print",
        [this] (std::string string)
        {
            //fprintf( stderr, "view_print: %s, %d, %d, %d, %d, %lld\n", string.c_str(), m_total.x, m_total.y, m_visible.x, m_visible.y, m_viewdata.size());
            debug_view_char *dest = &m_viewdata[0];
            
            for (char & c : string)
            {
                dest[(print_y*m_visible.y)+print_x].byte = c;
                print_x++;
            }
        });

    lua()->sol().set_function("view_set_xy",
        [this] (int x, int y)
        {
            // fprintf( stderr, "view_set_xy: %d, %d\n", x, y);
            print_x = x;
            print_y = y;
        });


	auto result = lua()->load_string(
        "cpu = manager.machine.devices[':maincpu']\n"
        "mem = cpu.spaces['program']\n"
        "print (mem:read_i8(0x8000))\n"
        "function view_update()\n"
        "print (mem:read_i8(0x8000))\n"
        "view_set_xy(10, 10)\n"
        "view_print('dump string')\n"
        "end\n" );
        
	if (!result.valid())
	{
		sol::error err = result;
		sol::load_status status = result.status();
		fatalerror("Error loading lua debug window %s: %s error\n%s\n",
				"no script",
				sol::to_string(status),
				err.what());
	}

	m_load_result.reset(new sol::load_result(std::move(result)));
	sol::protected_function func = *m_load_result;
	// sol::set_environment(lua()->make_environment(), func);

    sol::protected_function_result result2 = lua()->invoke(func);
    if (!result2.valid())
    {
        sol::error err = result2;
        sol::call_status status = result2.status();
        fatalerror("Error running lua debug window %s: %s error\n%s\n",
                "no script",
                sol::to_string(status),
                err.what());
    }
}

//-------------------------------------------------
//  debug_view_lua - destructor
//-------------------------------------------------

debug_view_lua::~debug_view_lua()
{
	m_load_result.reset();
	m_lua.reset();
}

//-------------------------------------------------
//  view_notify - handle notification of updates
//  to cursor changes
//-------------------------------------------------

void debug_view_lua::view_notify(debug_view_notification type)
{
}


//-------------------------------------------------
//  view_update - update the contents of the
//  Lua view
//-------------------------------------------------

void debug_view_lua::view_update()
{
    sol::protected_function func = lua()->sol()["view_update"];
    sol::protected_function_result call_result = func();

    if (!call_result.valid())
    {
        sol::error err = call_result;
        sol::call_status status = call_result.status();
        fatalerror("Error running view update %s: %s error\n%s\n",
                "no script",
                sol::to_string(status),
                err.what());
    }
}


//-------------------------------------------------
//  view_char - handle a character typed within
//  the current view
//-------------------------------------------------

void debug_view_lua::view_char(int chval)
{
}


//-------------------------------------------------
//  view_click - handle a mouse click within the
//  current view
//-------------------------------------------------

void debug_view_lua::view_click(const int button, const debug_view_xy& pos)
{
}


