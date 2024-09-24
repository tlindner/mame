// license:BSD-3-Clause
// copyright-holders:tim lindner
//============================================================
//
//  luaview.m - MacOS X Cocoa debug window handling
//
//============================================================

#include "emu.h"
#import "luaview.h"

#include "debug/debugvw.h"


@implementation MAMELuaView

- (id)initWithFrame:(NSRect)f machine:(running_machine &)m {
	if (!(self = [super initWithFrame:f type:DVT_LUA machine:m wholeLineScroll:NO]))
		return nil;
	return self;
}


- (void)dealloc {
	[super dealloc];
}

@end
