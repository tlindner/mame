// license:BSD-3-Clause
// copyright-holders:tim lindner
//============================================================
//
//  luaviewer.h - MacOS X Cocoa Lua debug window handling
//
//============================================================

#import "debugosx.h"

#import "debugwindowhandler.h"


#import <Cocoa/Cocoa.h>


@class MAMEDebugConsole, MAMELuaView;

@interface MAMELuaViewer : MAMEExpressionAuxiliaryDebugWindowHandler
{
	MAMELuaView  *luaView;
	NSPopUpButton   *subviewButton;
}

- (id)initWithMachine:(running_machine &)m console:(MAMEDebugConsole *)c;

- (BOOL)selectSubviewForDevice:(device_t *)device;
- (BOOL)selectSubviewForSpace:(address_space *)space;

- (IBAction)changeSubview:(id)sender;

- (void)saveConfigurationToNode:(util::xml::data_node *)node;
- (void)restoreConfigurationFromNode:(util::xml::data_node const *)node;

@end
