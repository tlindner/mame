// license:BSD-3-Clause
// copyright-holders:tim lindner
//============================================================
//
//  luaviewer.m - MacOS X Cocoa Lua debug window handling
//
//============================================================

#include "emu.h"
#import "luaviewer.h"

#import "debugconsole.h"
#import "debugview.h"
#import "luaview.h"

#include "debugger.h"
#include "debug/debugcon.h"
#include "debug/dvlua.h"

#include "util/xmlfile.h"


@implementation MAMELuaViewer

- (id)initWithMachine:(running_machine &)m console:(MAMEDebugConsole *)c {
	NSScrollView    *luaScroll;
	NSView          *expressionContainer;
	NSPopUpButton   *actionButton;
	NSRect          expressionFrame;

	if (!(self = [super initWithMachine:m title:@"Lua" console:c]))
		return nil;
	NSRect const contentBounds = [[window contentView] bounds];
	NSFont *const defaultFont = [[MAMEDebugView class] defaultFontForMachine:m];

	// create the expression field
	expressionField = [[NSTextField alloc] initWithFrame:NSMakeRect(0, 0, 100, 19)];
	[expressionField setAutoresizingMask:(NSViewWidthSizable | NSViewMaxXMargin | NSViewMinYMargin)];
	[expressionField setFont:defaultFont];
	[expressionField setFocusRingType:NSFocusRingTypeNone];
	[expressionField setTarget:self];
	[expressionField setAction:@selector(doExpression:)];
	[expressionField setDelegate:self];
	[expressionField sizeToFit];

	// create the subview popup
	subviewButton = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(0, 0, 100, 19)];
	[subviewButton setAutoresizingMask:(NSViewWidthSizable | NSViewMinXMargin | NSViewMinYMargin)];
	[subviewButton setBezelStyle:NSBezelStyleShadowlessSquare];
	[subviewButton setFocusRingType:NSFocusRingTypeNone];
	[subviewButton setFont:defaultFont];
	[subviewButton setTarget:self];
	[subviewButton setAction:@selector(changeSubview:)];
	[[subviewButton cell] setArrowPosition:NSPopUpArrowAtBottom];
	[subviewButton sizeToFit];

	// adjust sizes to make it fit nicely
	expressionFrame = [expressionField frame];
	expressionFrame.size.height = std::max(expressionFrame.size.height, [subviewButton frame].size.height);
	expressionFrame.size.width = (contentBounds.size.width - expressionFrame.size.height) / 2;
	[expressionField setFrame:expressionFrame];
	expressionFrame.origin.x = expressionFrame.size.width;
	expressionFrame.size.width = contentBounds.size.width - expressionFrame.size.height - expressionFrame.origin.x;
	[subviewButton setFrame:expressionFrame];

	// create a container for the expression field and subview popup
	expressionFrame = NSMakeRect(expressionFrame.size.height,
								 contentBounds.size.height - expressionFrame.size.height,
								 contentBounds.size.width - expressionFrame.size.height,
								 expressionFrame.size.height);
	expressionContainer = [[NSView alloc] initWithFrame:expressionFrame];
	[expressionContainer setAutoresizingMask:(NSViewWidthSizable | NSViewMinYMargin)];
	[expressionContainer addSubview:expressionField];
	[expressionField release];
	[expressionContainer addSubview:subviewButton];
	[subviewButton release];
	[[window contentView] addSubview:expressionContainer];
	[expressionContainer release];

	// create the memory view
	luaView = [[MAMELuaView alloc] initWithFrame:NSMakeRect(0, 0, 100, 100)
											   machine:*machine];
	[luaView insertSubviewItemsInMenu:[subviewButton menu] atIndex:0];
	luaScroll = [[NSScrollView alloc] initWithFrame:NSMakeRect(0,
																  0,
																  contentBounds.size.width,
																  expressionFrame.origin.y)];
	[luaScroll setAutoresizingMask:(NSViewWidthSizable | NSViewHeightSizable)];
	[luaScroll setHasHorizontalScroller:YES];
	[luaScroll setHasVerticalScroller:YES];
	[luaScroll setAutohidesScrollers:YES];
	[luaScroll setBorderType:NSNoBorder];
	[luaScroll setDrawsBackground:NO];
	[luaScroll setDocumentView:luaView];
	[luaView release];
	[[window contentView] addSubview:luaScroll];
	[luaScroll release];

	// create the action popup
	actionButton = [[self class] newActionButtonWithFrame:NSMakeRect(0,
																	 expressionFrame.origin.y,
																	 expressionFrame.size.height,
																	 expressionFrame.size.height)];
	[actionButton setAutoresizingMask:(NSViewMaxXMargin | NSViewMinYMargin)];
	[actionButton setFont:[NSFont systemFontOfSize:[defaultFont pointSize]]];
	[luaView insertActionItemsInMenu:[actionButton menu] atIndex:1];
	[[window contentView] addSubview:actionButton];
	[actionButton release];

	// set default state
	[luaView selectSubviewForDevice:machine->debugger().console().get_visible_cpu()];
	[luaView setExpression:@"0"];
	[expressionField setStringValue:@"0"];
	[expressionField selectText:self];
	[subviewButton selectItemAtIndex:[subviewButton indexOfItemWithTag:[luaView selectedSubviewIndex]]];
	[window makeFirstResponder:expressionField];
	[window setTitle:[NSString stringWithFormat:@"Lua: %@", [luaView scriptName]]];

	// calculate the optimal size for everything
	NSSize const desired = [NSScrollView frameSizeForContentSize:[luaView maximumFrameSize]
										 horizontalScrollerClass:[NSScroller class]
										   verticalScrollerClass:[NSScroller class]
													  borderType:[luaScroll borderType]
													 controlSize:NSControlSizeRegular
												   scrollerStyle:NSScrollerStyleOverlay];
	[self cascadeWindowWithDesiredSize:desired forView:luaScroll];

	// don't forget the result
	return self;
}


- (void)dealloc {
	[super dealloc];
}


- (id <MAMEDebugViewExpressionSupport>)documentView {
	return luaView;
}


- (IBAction)debugNewLuaWindow:(id)sender {
	debug_view_lua_source const *source = [luaView source];
	[console debugNewLuaWindowForSpace:source->space()
								   device:source->device()
							   expression:[luaView expression]];
}

- (IBAction)debugNewDebugWindow:(id)sender {
	debug_view_lua_source const *source = [luaView source];
	[console debugNewMemoryWindowForSpace:source->space()
								   device:source->device()
							   expression:[luaView expression]];
}


- (IBAction)debugNewDisassemblyWindow:(id)sender {
	debug_view_lua_source const *source = [luaView source];
	[console debugNewDisassemblyWindowForSpace:source->space()
										device:source->device()
									expression:[luaView expression]];
}


- (BOOL)selectSubviewForDevice:(device_t *)device {
	BOOL const result = [luaView selectSubviewForDevice:device];
	[subviewButton selectItemAtIndex:[subviewButton indexOfItemWithTag:[luaView selectedSubviewIndex]]];
	[window setTitle:[NSString stringWithFormat:@"Lua: %@", [luaView scriptName]]];
	return result;
}


- (BOOL)selectSubviewForSpace:(address_space *)space {
	BOOL const result = [luaView selectSubviewForSpace:space];
	[subviewButton selectItemAtIndex:[subviewButton indexOfItemWithTag:[luaView selectedSubviewIndex]]];
	[window setTitle:[NSString stringWithFormat:@"Lua: %@", [luaView scriptName]]];
	return result;
}


- (IBAction)changeSubview:(id)sender {
	[luaView selectSubviewAtIndex:[[sender selectedItem] tag]];
	[window setTitle:[NSString stringWithFormat:@"Lua: %@", [luaView scriptName]]];
}


- (void)saveConfigurationToNode:(util::xml::data_node *)node {
	[super saveConfigurationToNode:node];
	node->set_attribute_int(osd::debugger::ATTR_WINDOW_TYPE, osd::debugger::WINDOW_TYPE_LUA_VIEWER);
	node->set_attribute_int(osd::debugger::ATTR_WINDOW_MEMORY_REGION, [luaView selectedSubviewIndex]);
	[luaView saveConfigurationToNode:node];
}


- (void)restoreConfigurationFromNode:(util::xml::data_node const *)node {
	[super restoreConfigurationFromNode:node];
	int const region = node->get_attribute_int(osd::debugger::ATTR_WINDOW_MEMORY_REGION, [luaView selectedSubviewIndex]);
	[luaView selectSubviewAtIndex:region];
	[subviewButton selectItemAtIndex:[subviewButton indexOfItemWithTag:[luaView selectedSubviewIndex]]];
	[luaView restoreConfigurationFromNode:node];
	[window setTitle:[NSString stringWithFormat:@"Lua: %@", [luaView scriptName]]];
}

@end
