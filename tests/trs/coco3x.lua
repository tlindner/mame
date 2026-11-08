-- license:BSD-3-Clause
-- External integration test; does not use scripts/src/tests.lua.
-- Run with coco3x, -ram 2m, and -autoboot_script pointing to this file.
local runner = coroutine.create(function()
local function wait(frames)
 for _=1,frames do coroutine.yield() end
end
local m=manager.machine
local s=m.devices[':maincpu'].spaces.program
local cpu=m.devices[':maincpu']
local count=0
local function check(actual, expected, name)
 assert(actual==expected,string.format('%s: got %x expected %x',name,actual,expected))
 count=count+1
end
local function addr(first,a)
 s:write_u8(first,(a>>16)&31); s:write_u8(first+1,(a>>8)&255); s:write_u8(first+2,a&255)
end
-- Disable BASIC's MMU mapping before installing the CPU loop.
s:write_u8(0xff90,0)
-- Park the CPU in low RAM so test writes cannot provoke BASIC interrupt handling.
s:write_u8(0x200,0x20); s:write_u8(0x201,0xfe)
cpu.state.PC.value=0x200; cpu.state.CC.value=0x50
-- DMA byte and 16-bit accesses, independent source and destination, address rollover.
s:write_u8(0xffe7,3)
addr(0xffe1,0x1ffffe)
s:write_u16(0xffe8,0x1234); s:write_u16(0xffe8,0x5678)
addr(0xffe4,0x1ffffe)
check(s:read_u16(0xffe8),0x1234,'DMA word at end of RAM')
check(s:read_u16(0xffe8),0x5678,'DMA wraps at 2MB')
s:write_u8(0xffe7,0)
addr(0xffe1,0x12345); s:write_u16(0xffe8,0xabcd)
addr(0xffe4,0x12345)
check(s:read_u16(0xffe8),0xabcd,'DMA word with increment disabled')
check(s:read_u16(0xffe8),0xabcd,'DMA stationary source')
s:write_u8(0xffe8,0xef)
check(s:read_u16(0xffe8),0xefcd,'DMA stationary destination')
-- Full 8-bit MMU readback, all 256 blocks and both tasks.
s:write_u8(0xffe7,1)
for bank=0,255 do
 addr(0xffe1,bank*8192+0x123)
 s:write_u8(0xffe8,bank ~ 0x5a)
end
s:write_u8(0xff90,0x40)
for task=0,1 do
 s:write_u8(0xff91,task)
 for bank=0,255 do
  s:write_u8(0xffa1+task*8,bank)
  check(s:read_u8(0xffa1+task*8),bank,'8-bit MMU readback')
  check(s:read_u8(0x2123),bank ~ 0x5a,'MMU physical block')
 end
end
s:write_u8(0xff90,0);s:write_u8(0xff91,0)
print('GIME-X MMU/DMA checks passed: '..count)
-- Palette and pixels are checked asynchronously after the beam has fetched new data.
local function gxaddr(first,a) addr(first,a) end
local function gxcolor(reg,color)
 s:write_u8(0xffb0+reg,(color>>3)&63)
 s:write_u8(0xffb0+reg,0x80|((color&7)<<3))
end
local function gxfill(n,pattern)
 addr(0xffe1,0x10000);s:write_u8(0xffe7,1)
 for i=0,n-1 do s:write_u8(0xffe8,pattern(i)) end
end
local function gxmode(ff98,ff99)
 s:write_u8(0xff90,0)
 s:write_u8(0xff98,ff98);s:write_u8(0xff99,ff99)
 s:write_u8(0xff9d,0x20);s:write_u8(0xff9e,0);s:write_u8(0xff9f,0)
end

local scr=m.screens[':screen']
local function rgb(color)
 local r=(((color>>8)&1)*4+((color>>5)&1)*2+((color>>2)&1))*255//7
 local g=(((color>>7)&1)*4+((color>>4)&1)*2+((color>>1)&1))*255//7
 local b=(((color>>6)&1)*4+((color>>3)&1)*2+(color&1))*255//7
 return (r<<16)|(g<<8)|b
end
local function pixel(x,y) return scr:pixel(x,y)&0xffffff end
-- The fixed FE page follows MMU slot 7 unless INIT0.MC3 is set.
s:write_u8(0xffe7,1);gxaddr(0xffe1,0x1ffe00);s:write_u8(0xffe8,0xa7)
s:write_u8(0xffa7,255);s:write_u8(0xff90,0x40)
check(s:read_u8(0xfe00),0xa7,'MMU FE page')
s:write_u8(0xff90,0)
for i=0,15 do gxcolor(i,i*32) end
gxfill(320*225,function(i) return (((i*2)&15)<<4)|((i*2+1)&15) end)
gxmode(0xc0,0x62);wait(5)
for i=0,639 do check(pixel(40+i,40),rgb((i%16)*32),'640x225 16-colour pixel') end
-- All 256 palette entries are distinct, and each pixel spans two VGA dots.
for i=0,255 do gxcolor(0,i*2);s:write_u8(0xffe0,i) end
gxfill(320*225,function(i) return i%256 end)
gxmode(0xc0,0x63);wait(5)
for i=0,255 do
 check(pixel(40+i*2,40),rgb(i*2),'320x225 palette entry')
 check(pixel(41+i*2,40),rgb(i*2),'320x225 pixel width')
end
-- A conventional six-bit write clears the three extension bits.
gxcolor(0,511);s:write_u8(0xffb0,0);s:write_u8(0xffe0,7);wait(5)
check(pixel(54,40),0,'six-bit palette write clears low bits')
gxcolor(0,14);s:write_u8(0xffe0,7)
gxfill(640*113,function(i) return i%256 end)
gxmode(0xc3,0x67);wait(5)
for i=0,639 do check(pixel(40+i,40),rgb((i%256)*2),'640x112 palette pixel') end
-- The 640-byte mode repeats each source row twice (four VGA output lines).
gxfill(640*113,function(i) return (i//640)%256 end);wait(5)
local previous,last
local transitions=0
for y=80,300 do
 local color=pixel(110,y)
 if last and color~=last then
  if previous then check(y-previous,4,'640x112 row height') end
  previous=y;transitions=transitions+1
 end
 last=color
end
assert(transitions>30,'640x112 must advance source rows')
gxfill(640*113,function(i) return i%256 end);wait(5)
local buttons=m.ioport.ports[':gimex_buttons'].fields
buttons['GIME-X Scan Lines']:set_value(1);wait(4)
check(pixel(47,40),rgb(14),'VGA lit line')
check(pixel(47,41),0,'VGA blank alternate line')
buttons['GIME-X Scan Lines']:set_value(0)
-- Save/load must restore the indirect palette and independent DMA pointers.
s:write_u8(0xffe7,1);gxaddr(0xffe1,0x1f0000);s:write_u16(0xffe8,0x9876)
gxaddr(0xffe4,0x1f0000);s:write_u8(0xffe7,2)
local savefile=os.tmpname()..'.sta'
m:save(savefile);wait(4)
gxcolor(0,0);s:write_u8(0xffe0,7);gxaddr(0xffe4,0x11111)
m:load(savefile);wait(4)
check(pixel(47,40),rgb(14),'palette restoration')
check(s:read_u16(0xffe8),0x9876,'DMA restoration')
os.remove(savefile)
-- SAM-selected semigraphics fetches advance every 3, 2, or 1 source lines.
gxcolor(0,511);gxcolor(8,0)
gxfill(32*192,function(i) return ((i//32)%2==0) and 0x8f or 0x80 end)
for a=0xffc0,0xffd2,2 do s:write_u8(a,0) end
s:write_u8(0xffc3,0);s:write_u8(0xff23,4);s:write_u8(0xff22,0)
s:write_u8(0xff99,0);s:write_u8(0xff90,0x80);wait(5)
local function sgcheck(step)
 local last,previous
 local transitions=0
 for y=80,300,2 do
  local p=pixel(110,y)
  if last and p~=last then
   if previous then check(y-previous,step,'semigraphics row pitch') end
   previous=y;transitions=transitions+1
  end
  last=p
 end
 assert(transitions>10,'semigraphics must advance')
end
sgcheck(6)
s:write_u8(0xffc1,0);wait(5);sgcheck(4)
s:write_u8(0xffc0,0);s:write_u8(0xffc5,0);wait(5);sgcheck(2)
for a=0xffc0,0xffc4,2 do s:write_u8(a,0) end
gxfill(32*24,function() return 0xbf end)
gxcolor(2,511);gxcolor(3,0);s:write_u8(0xff22,0x10);wait(5)
check(pixel(110,80),0,'SG6 initially disabled')
buttons['GIME-X Artifact / SG6']:set_value(1);wait(5)
check(pixel(110,80),0xffffff,'SG6 enabled by button')
buttons['GIME-X Artifact / SG6']:set_value(0)
-- Measure instruction throughput over equal intervals of emulated video time.
for i,b in ipairs({0xbe,3,0,0x30,1,0xbf,3,0,0x20,0xf6}) do s:write_u8(0x1ff+i,b) end
cpu.state.PC.value=0x200;cpu.state.CC.value=0x50
local counts={}
for i,write in ipairs({{0xffd8,0},{0xffd9,0},{0xffd9,0xa5},{0xffd9,0x55},{0xffd8,0}}) do
 s:write_u8(write[1],write[2]);local first=s:read_u16(0x300)
 wait(6);counts[i]=(s:read_u16(0x300)-first)&65535
end
assert(math.abs(counts[2]/counts[1]-2)<0.04,'fast clock ratio')
assert(math.abs(counts[3]/counts[2]-1.6)<0.04,'turbo clock ratio')
assert(math.abs(counts[4]/counts[2]-1)<0.04,'FFD9 exits turbo')
assert(math.abs(counts[5]/counts[1]-1)<0.04,'FFD8 selects slow clock')
print('GIME-X integration PASSED: '..count..' exact checks plus CPU speed ratios')
m:exit()
end)
emu.register_frame_done(function()
 if coroutine.status(runner)=='dead' then return end
 local ok,err=coroutine.resume(runner)
 if not ok then
  print('GIME-X integration FAILED: '..tostring(err))
  manager.machine:exit()
 end
end)
