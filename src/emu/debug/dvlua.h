// license:BSD-3-Clause
// copyright-holders:tim lindner
/*********************************************************************

    dvlua.h

    Generic debugger view controlled by a Lua script.

***************************************************************************/

#ifndef MAME_EMU_DEBUG_DVLUA_H
#define MAME_EMU_DEBUG_DVLUA_H

#pragma once

#include "debugvw.h"
#include "../../frontend/mame/luaengine.h"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// a memory view_source
class debug_view_lua_source : public debug_view_source
{
	friend class debug_view_lua;

public:
	debug_view_lua_source(std::string &&name, address_space &space);

private:
};


// debug view for memory
class debug_view_lua : public debug_view
{
	friend class debug_view_manager;

	// construction/destruction
	debug_view_lua(running_machine &machine, debug_view_osd_update_func osdupdate, void *osdprivate);
	~debug_view_lua();

public:
	lua_engine *lua() { return m_lua.get(); }
	sol::load_result *load_result() { return *m_load_result; };

protected:
	// view overrides
	virtual void view_notify(debug_view_notification type) override;
	virtual void view_update() override;
	virtual void view_char(int chval) override;
	virtual void view_click(const int button, const debug_view_xy& pos) override;

private:
	std::unique_ptr<lua_engine> m_lua;
	std::unique_ptr<sol::load_result> m_load_result;

	debug_view_xy m_location;
	std::vector<char> m_viewbuffer;
};

#endif // MAME_EMU_DEBUG_DVLUA_H
