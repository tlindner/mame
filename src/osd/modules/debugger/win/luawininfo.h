// license:BSD-3-Clause
// copyright-holders:tim lindner, Aaron Giles, Vas Crabb
//============================================================
//
//  luawininfo.h - Win32 lua debug handling
//
//============================================================
#ifndef MAME_DEBUGGER_WIN_LUAWININFO_H
#define MAME_DEBUGGER_WIN_LUAWININFO_H

#pragma once

#include "debugwin.h"

#include "debugwininfo.h"


namespace osd::debugger::win {

class luawin_info : public debugwin_info
{
public:
	luawin_info(debugger_windows_interface &debugger);
	virtual ~luawin_info();

protected:
	virtual bool handle_command(WPARAM wparam, LPARAM lparam) override;
	virtual void save_configuration_to_node(util::xml::data_node &node) override;
};

} // namespace osd::debugger::win

#endif // MAME_DEBUGGER_WIN_LUAWININFO_H
