// license:BSD-3-Clause
// copyright-holders:tim lindner
//============================================================
//
//  luaviewer.m - MacOS X Cocoa debug window handling
//
//============================================================

#include "emu.h"
#import "luaviewer.h"

#import "luaview.h"

#include "util/xmlfile.h"


@implementation MAMELuaViewer

- (id)initWithMachine:(running_machine &)m console:(MAMEDebugConsole *)c {
	NSScrollView    *luaScroll;
	NSString        *title;

	title = [NSString stringWithFormat:@"Error Lua: %@ [%@]",
									   [NSString stringWithUTF8String:m.system().type.fullname()],
									   [NSString stringWithUTF8String:m.system().name]];
	if (!(self = [super initWithMachine:m title:title console:c]))
		return nil;

	// create the lua view
	luaView = [[MAMELuaView alloc] initWithFrame:NSMakeRect(0, 0, 100, 100) machine:*machine];
	luaScroll = [[NSScrollView alloc] initWithFrame:NSMakeRect(0, 0, 100, 100)];
	[luaScroll setAutoresizingMask:(NSViewWidthSizable | NSViewHeightSizable)];
	[luaScroll setHasHorizontalScroller:YES];
	[luaScroll setHasVerticalScroller:YES];
	[luaScroll setAutohidesScrollers:YES];
	[luaScroll setBorderType:NSNoBorder];
	[luaScroll setDrawsBackground:NO];
	[luaScroll setDocumentView:luaView];
	[luaView release];
	[window setContentView:luaScroll];
	[luaScroll release];

	// calculate the optimal size for everything
	{
		NSSize  desired = [NSScrollView frameSizeForContentSize:[luaView maximumFrameSize]
										horizontalScrollerClass:[NSScroller class]
										  verticalScrollerClass:[NSScroller class]
													 borderType:[luaScroll borderType]
													controlSize:NSControlSizeRegular
												  scrollerStyle:NSScrollerStyleOverlay];

		// this thing starts with no content, so its preferred height may be very small
		desired.height = std::max(desired.height, CGFloat(240));
		[self cascadeWindowWithDesiredSize:desired forView:luaScroll];
	}

	// don't forget the result
	return self;
}


- (void)dealloc {
	[super dealloc];
}


- (void)saveConfigurationToNode:(util::xml::data_node *)node {
	[super saveConfigurationToNode:node];
	node->set_attribute_int(osd::debugger::ATTR_WINDOW_TYPE, osd::debugger::WINDOW_TYPE_LUA_VIEWER);
}

@end
