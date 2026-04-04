// license:BSD-3-Clause
// copyright-holders:tim lindner, Samuele Zannoli
//============================================================
//
//  luaviewinfo.h - Win32 lua debug window handling
//
//============================================================
#ifndef MAME_DEBUGGER_WIN_LUAVIEWINFO_H
#define MAME_DEBUGGER_WIN_LUAVIEWINFO_H

#pragma once

#include "debugwin.h"

#include "debugviewinfo.h"


namespace osd::debugger::win {

class luaview_info : public debugview_info
{
public:
	luaview_info(debugger_windows_interface &debugger, debugwin_info &owner, HWND parent);
	virtual ~luaview_info();

	void clear();
};

} // namespace osd::debugger::win

#endif // MAME_DEBUGGER_WIN_LUAVIEWINFO_H
