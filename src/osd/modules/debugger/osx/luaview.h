// license:BSD-3-Clause
// copyright-holders:tim lindner
//============================================================
//
//  luaview.h - MacOS X Cocoa debug window handling
//
//============================================================

#import "debugosx.h"

#import "debugview.h"


#import <Cocoa/Cocoa.h>


@interface MAMELuaView : MAMEDebugView
{
}

- (id)initWithFrame:(NSRect)f machine:(running_machine &)m;

@end
