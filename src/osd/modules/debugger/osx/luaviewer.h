// license:BSD-3-Clause
// copyright-holders:tim lindner
//============================================================
//
//  luaviewer.h - MacOS X Cocoa debug window handling
//
//============================================================

#import "debugosx.h"

#import "debugwindowhandler.h"


#import <Cocoa/Cocoa.h>


@class MAMEDebugConsole, MAMELuaView;

@interface MAMELuaViewer : MAMEAuxiliaryDebugWindowHandler
{
	MAMELuaView    *luaView;
}

- (id)initWithMachine:(running_machine &)m console:(MAMEDebugConsole *)c;

- (void)saveConfigurationToNode:(util::xml::data_node *)node;

@end
