// license:BSD-3-Clause
// copyright-holders:tim lindner, Samuele Zannoli
//============================================================
//
//  luaviewinfo.cpp - Win32 lua debug window handling
//
//============================================================

#include "emu.h"
#include "luaviewinfo.h"

#include "debug/dvtext.h"


namespace osd::debugger::win {

luaview_info::luaview_info(debugger_windows_interface &debugger, debugwin_info &owner, HWND parent) :
	debugview_info(debugger, owner, parent, DVT_LUA)
{
}


luaview_info::~luaview_info()
{
}


void luaview_info::clear()
{
//	view<debug_view_lua>()->clear();
}

} // namespace osd::debugger::win
