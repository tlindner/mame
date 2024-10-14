// license:BSD-3-Clause
// copyright-holders:tim lindner
//============================================================
//
//  luaview.h - MacOS X Cocoa debug Lua window handling
//
//============================================================

#import "debugosx.h"

#import "debugview.h"

#include "debug/dvlua.h"

#import <Cocoa/Cocoa.h>


@interface MAMELuaView : MAMEDebugView <MAMEDebugViewSubviewSupport, MAMEDebugViewExpressionSupport>
{
}

- (id)initWithFrame:(NSRect)f machine:(running_machine &)m;

- (NSSize)maximumFrameSize;

- (NSString *)scriptRunning;
- (NSString *)selectedSubviewName;
- (int)selectedSubviewIndex;
- (void)selectSubviewAtIndex:(int)index;
- (BOOL)selectSubviewForDevice:(device_t *)device;
- (BOOL)selectSubviewForSpace:(address_space *)space;

- (NSString *)expression;
- (void)setExpression:(NSString *)exp;

- (debug_view_lua_source const *)source;

- (IBAction)loadScript:(id)sender;
- (IBAction)reloadScript:(id)sender;
- (IBAction)stopScript:(id)sender;

- (IBAction)showChunkSize:(id)sender;
- (IBAction)showPhysicalAddresses:(id)sender;
- (IBAction)showReverseView:(id)sender;
- (IBAction)showReverseViewToggle:(id)sender;
- (IBAction)changeBytesPerLine:(id)sender;

- (void)saveConfigurationToNode:(util::xml::data_node *)node;
- (void)restoreConfigurationFromNode:(util::xml::data_node const *)node;

- (void)insertActionItemsInMenu:(NSMenu *)menu atIndex:(NSInteger)index;
- (void)insertSubviewItemsInMenu:(NSMenu *)menu atIndex:(NSInteger)index;

@end
