// license:BSD-3-Clause
// copyright-holders:tim lindner
/*********************************************************************

    dvlua.cpp

    Generic debugger view controlled by a Lua script.

***************************************************************************/

#include "emu.h"
#include "dvlua.h"

#include "debugger.h"

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
    , m_viewbuffer(m_area.y * m_area.x)
{
    m_lua->initialize();

	lua()->sol().set_function("view_set_area",
		[this] (s32 x, s32 y)
		{
			m_area.x = x;
			m_area.y = y;
			m_viewbuffer.resize(x*y, 0);
		});

    lua()->sol().set_function("view_print",
        [this] (const char *str)
        {
            //fprintf( stderr, "view_print: %s, %d, %d, %d, %d, %lld\n", string.c_str(), m_total.x, m_total.y, m_visible.x, m_visible.y, m_viewdata.size());

           	if (m_location.y > m_area.y) return;

            while (*str != '\0')
            {
            	if (m_location.x > m_area.x) break;

                m_viewbuffer[(m_location.y*m_area.x)+m_location.x] = *str++;
                m_location.x++;
            }
        });

    lua()->sol().set_function("view_set_xy",
        [this] (s32 x, s32 y)
        {
            // fprintf( stderr, "view_set_xy: %d, %d\n", x, y);
            m_location.x = x;
            m_location.y = y;
        });


	auto result = lua()->load_string(
        "cpu = manager.machine.devices[':maincpu']\n"
        "mem = cpu.spaces['program']\n"
        "view_set_area(40,40)\n"
        "function view_update()\n"
        "   view_set_xy (0, 0)\n"
        "   view_print ('POW ' .. mem:read_u16(0x0217) .. '  ')\n"
        "   view_print ('DAM ' .. mem:read_u16(0x0221) .. '        ')\n"
        "   player_pos_y = mem:read_u8(0x213)\n"
        "   player_pos_x = mem:read_u8(0x214)\n"
        "   line = 1"
        "   addy = 0x05f4\n"
        "   for i = 0,31,1 \n"
        "   do\n"
        "       view_set_xy (0, line)\n"
        "       for j = 0,31,1\n"
        "       do\n"
        "           if (mem:read_u8(addy) == 255)\n"
        "           then\n"
        "               view_print(' ')\n"
        "           else\n"
        "               if ((player_pos_x==j) and (player_pos_y==i))\n"
        "               then\n"
        "                  view_print('X')\n"
        "               else\n"
        "                  view_print('O')\n"
        "               end\n"
        "           end\n"
        "           addy = addy + 1\n"
        "       end\n"
        "       line = line + 1\n"
        "   end\n"
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
    else
    {
    	for (s32 i=0; i<m_area.y; i++)
    	{
    		if (i<m_visible.y)
    		{
				for (s32 j=0; j<m_area.x; j++ )
				{
					if (j<m_visible.x)
						m_viewdata[(i*m_visible.x)+j].byte = m_viewbuffer[(i*m_area.x)+j];
				}

                for (s32 j=m_area.x; j<m_visible.x; j++)
                {
                    m_viewdata[(i*m_visible.x)+j].byte = 0;
                }
			}
    	}
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


