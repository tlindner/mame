// license:BSD-3-Clause
// copyright-holders:tim lindner
/***************************************************************************

    CoCo3FPGA 4.1 (Gary Becker, 2016)

    A Color Computer 3 compatible system on a Cyclone II FPGA (Terasic DE1),
    optionally with an analog add-on board (extra SRAM, joysticks, RS-232
    levels, DS3231 RTC, ESP8266 WiFi).

    This driver follows the 4.1 HDL (coco3fpga_top.v, version $41, as
    shipped with `define M5Meg), described in
    CoCo3FPGA_4.1/MAME_EMULATION_4.1.md.  The HDL is the reference; where
    its comments or the 4.1 user guide disagree with its logic, the logic is
    emulated.  Section numbers below refer to that document.

    Main processor: John Kent CPU09 (6809 compatible, *not* HD6309), clocked
    from 50 MHz: /56 (RATE=0), /28 (RATE=1), /2 (RATE=1 and turbo switch).
    I/O processor:  T65 in 6502 mode at 6.25 MHz running loadable firmware.
    Video:          FPGA GIME-like video at 25 MHz pixel clock, 798 dots x
                    524 lines (~59.79 Hz), 12-bit palette plus a 256-entry
                    direct-colour palette RAM.

    Explicit approximations (see the reference document, section 2):
    - Main CPU is MAME's MC6809E; CPU09 instruction timing differs.
    - Video is rendered a scanline at a time at the HSYNC falling edge.  The
      fetch pipeline is modelled by mode, not cycle by cycle.
    - Interrupt flip-flops are updated at source events; the two-stage
      synchronisers and one-cycle clear strobes are treated as immediate.
    - The block copy engine runs in CPU09 idle cycles, which MAME's 6809
      does not expose; it is emulated at one step per CPU clock.
    - The SDRAM sector store and the SPI bridge complete transfers at once,
      so the CPU HALTs the HDL asserts during them are not visible.
    - The PS/2 keyboard is one of MAME's PC keyboard models on a pc_kbdc
      connector; the receiver (ps2_keyboard.v) and translator (cocokey.v)
      are ported at event level, without the receiver's clock sampling.
    - Joystick latches are host analog axes, not the paddles.v converter.
    - The 6551 and both 6850s use MAME's MOS6551 and ACIA6850 devices.
    - Cross-clock-domain synchronisers between the 6809 and the 6502 are
      treated as immediate (with a perfect scheduling quantum).
    - The 6502's codec I2C bus always acknowledges.  The main-CPU I2C bus
      has a register-level DS3231 (time from the host clock plus an offset
      set by writes); other addresses fail.  There is no codec model.
    - The DE1 flash (Spansion S29AL032D) is emulated with MAME's AMD
      29F032, which has the same size and AMD command set.
    - No ESP8266 model; the WiFi UART is an RS-232 port.

    ROMs: per the 4.1 user guide the flash holds unmodified stock ROMs:
    CoCo 3 BASIC at $000000, Disk BASIC at $008000 (slot 4) and
    Orchestra-90CC at $00A000 (slot 1).  Slot 2 ($3F8000) and the slot-3
    cartridge banks hold user-chosen ROMs and are left erased.  The FPGA
    memory initialisers are decoded from disk02.mif, disk02_02.mif,
    VDAC.mif and coco3gen.mif.

    Emulator policies for state the HDL leaves unassigned at power on:
    border palette entry, clock fields and CLICK are zero; slot-3 bank
    mappings power up as 0..7 (as the HDL comment describes).

***************************************************************************/

#include "emu.h"

#include "bus/pc_kbd/keyboards.h"
#include "bus/pc_kbd/pc_kbdc.h"
#include "bus/rs232/rs232.h"
#include "cpu/m6502/m6502.h"
#include "cpu/m6809/m6809.h"
#include "machine/6850acia.h"
#include "machine/clock.h"
#include "machine/intelfsh.h"
#include "machine/mos6551.h"
#include "machine/spi_sdcard.h"
#include "sound/dac.h"

#include "screen.h"
#include "speaker.h"

#include <algorithm>


namespace {

class coco3fpga_state : public driver_device
{
public:
	coco3fpga_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_iocpu(*this, "iocpu")
		, m_screen(*this, "screen")
		, m_flash(*this, "flash")
		, m_acia(*this, "acia")
		, m_acia_clock(*this, "acia_clock")
		, m_wifi_acia(*this, "wifi_acia")
		, m_wifi_clock(*this, "wifi_clock")
		, m_uart(*this, "uart")
		, m_sdcard(*this, "sdcard")
		, m_dac_l(*this, "dac_l")
		, m_dac_r(*this, "dac_r")
		, m_io_ram_init(*this, "io_ram")
		, m_fw_init(*this, "firmware")
		, m_vdac_init(*this, "vdac")
		, m_chargen(*this, "chargen")
		, m_kbdc(*this, "kbd")
		, m_joy(*this, "joy%u", 0U)
		, m_joy_buttons(*this, "joy_buttons")
		, m_board(*this, "board")
		, m_switches(*this, "switches")
		, m_config(*this, "config")
	{
	}

	void coco3fpga(machine_config &config) ATTR_COLD;

	DECLARE_INPUT_CHANGED_MEMBER(reset_button);
	DECLARE_INPUT_CHANGED_MEMBER(switches_changed);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	// raster geometry (dots at 25 MHz)
	static constexpr u32 PIXEL_CLOCK = 25'000'000;
	static constexpr int LINE_DOTS = 798;
	static constexpr int FRAME_LINES = 524;
	static constexpr int VIS_WIDTH = 720;      // left border .. end of right border
	static constexpr int VIS_HEIGHT = 486;     // top border .. end of bottom border
	static constexpr int VIS_LEFT_DOTS = 64;   // dots of the previous line shown at x=0
	static constexpr int VIS_TOP_LINES = 18;   // LINE 506..523 shown above LINE 0
	static constexpr int ACTIVE_X = 80;        // first active dot on screen

	static constexpr u32 CHIP_SIZE = 0x200000; // each SRAM position is 2 MB

	enum : int { EV_LINE_START = 0, EV_FLAG, EV_HSYNC_FALL, EV_HSYNC_RISE };

	// GIME interrupt source bits
	enum : int { SRC_CART = 0, SRC_KEY = 1, SRC_VBORD = 3, SRC_HBORD = 4, SRC_TMR = 5 };

	required_device<mc6809e_device> m_maincpu;
	required_device<m6502_device> m_iocpu;
	required_device<screen_device> m_screen;
	required_device<amd_29f032_device> m_flash;
	required_device<acia6850_device> m_acia;
	required_device<clock_device> m_acia_clock;
	required_device<acia6850_device> m_wifi_acia;
	required_device<clock_device> m_wifi_clock;
	required_device<mos6551_device> m_uart;
	required_device<spi_sdcard_device> m_sdcard;
	required_device<dac_16bit_r2r_device> m_dac_l;
	required_device<dac_16bit_r2r_device> m_dac_r;
	required_region_ptr<u8> m_io_ram_init;
	required_region_ptr<u8> m_fw_init;
	required_region_ptr<u8> m_vdac_init;
	required_region_ptr<u8> m_chargen;
	required_device<pc_kbdc_device> m_kbdc;
	required_ioport_array<4> m_joy;
	required_ioport m_joy_buttons;
	required_ioport m_board;
	required_ioport m_switches;
	required_ioport m_config;

	void main_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	// physical memory (23-bit addresses, four 2 MB chip positions)
	bool chip_present(u32 phys) const { return m_chip_mask[(phys >> 21) & 3] != 0; }
	u32 chip_offset(u32 phys) const;
	u8 phys_r(u32 phys, u8 absent) const;
	void phys_w(u32 phys, u8 data);

	// main CPU bus
	u8 main_r(offs_t offset);
	void main_w(offs_t offset, u8 data);
	u8 hw_r(offs_t addr);
	void hw_w(offs_t addr, u8 data);
	bool rom_sel(offs_t addr) const;
	bool ram_cs(offs_t addr) const;
	u32 ram_phys(offs_t addr) const;
	int flash_offset(offs_t addr) const;

	// disk interface (6809 side, slots 2 and 4)
	u8 hdd_r(offs_t reg);
	void hdd_w(offs_t reg, u8 data);

	// I/O processor bus
	u8 io_r(offs_t offset);
	void io_w(offs_t offset, u8 data);
	u8 ioreg_r(offs_t addr);
	void ioreg_w(offs_t addr, u8 data);

	// SPI bridge
	u8 spi_r(offs_t reg);
	void spi_w(offs_t reg, u8 data);
	u8 spi_transfer(u8 data);
	void spi_miso_w(int state) { m_spi_miso = state; }

	// main-CPU I2C bus and DS3231
	TIMER_CALLBACK_MEMBER(rtc_i2c_done);
	bool rtc_transaction();
	u8 rtc_reg_r(u8 reg);
	void rtc_reg_w(u8 reg, u8 data);
	void rtc_now(int t[7]);

	// SDRAM sector store
	void sdram_start();

	// block copy
	TIMER_CALLBACK_MEMBER(dma_step);
	void dma_kick();

	// keyboard / joystick
	u8 keyboard_in();
	u16 joy_latch(int n) { return m_joy[n]->read() & 0xfff; }

	// PS/2 receiver (ps2_keyboard.v) and translator (cocokey.v)
	void ps2_clock_w(int state);
	void ps2_data_w(int state) { m_ps2_data = state; }
	void ps2_code(u8 code);
	void cocokey_scan(u8 scan, bool press, bool extended);
	void cocokey_reset();

	// interrupts
	bool irq_09() const;
	void update_cart();
	void update_key();
	void update_interrupts();
	void update_halt();
	void update_nmi();
	void update_io_irq();
	void update_halt_code();
	void hsync_int_edge(bool falling);
	void vsync_int_edge(bool falling);
	void set_gime(int src) { m_irq_stat[src] = true; m_firq_stat[src] = true; }

	// timer
	void timer_restart();
	void timer_catch_up();
	void timer_tick(u32 ticks);
	void timer_schedule();
	TIMER_CALLBACK_MEMBER(gime_timer_expired);

	// video
	TIMER_CALLBACK_MEMBER(line_event);
	TIMER_CALLBACK_MEMBER(cpu_reset_release);
	TIMER_CALLBACK_MEMBER(halt_state_expired);
	TIMER_CALLBACK_MEMBER(i2c_done);
	bool mode_256() const { return m_coco1 || !BIT(m_hres, 0); }
	int flag_dot() const { return mode_256() ? 594 : 658; }
	int hsync_fall_dot() const { return mode_256() ? 615 : 679; }
	int hsync_rise_dot() const { return mode_256() ? 690 : 754; }
	u8 lines_row() const;
	u32 screen_off() const;
	void reset_row();
	void step_row();
	void line_advance();
	void render_line(int line);
	void gen_active(u16 *out, int width);
	u16 border_code() const;
	rgb_t code_to_rgb(u16 code, bool dim) const;
	int current_dot() const;
	u8 vram_r(u32 offset) const;

	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	void update_cpu_clock();
	void update_acia_clocks();
	void update_audio();
	void update_sound_dtoa();
	void update_slave_reset();
	void periph_reset();

	// memories
	std::unique_ptr<u8[]> m_ram;            // 4 x 2 MB chip positions
	u32 m_chip_mask[4];                     // populated size - 1 per position, 0 = absent
	std::unique_ptr<u8[]> m_sdram;          // 8 MB sector store
	u8 m_io_ram[0x200];                     // 6502 $0000-$01FF
	u8 m_fw[2][0x1000];                     // 6502 firmware blocks A and B
	u16 m_vdac[256];                        // 256-colour palette RAM
	u8 m_rdbuf[0x200];      // 6502 -> 6809 sector buffer
	u8 m_wrbuf[0x200];      // 6809 -> 6502 sector buffer

	// FIFOs (registered output, no show-ahead)
	u8 m_rdfifo[0x200];     // 6502 writes, 6809 reads ($FF42)
	u16 m_rdfifo_head, m_rdfifo_count;
	u8 m_rdfifo_q;
	u8 m_wrfifo[0x200];     // 6809 writes, 6502 reads
	u16 m_wrfifo_head, m_wrfifo_count;
	u8 m_wrfifo_q;
	u8 m_wf_rdfifo[0x400];  // 6502 writes, 6809 reads ($FF6D), 1024 deep
	u16 m_wf_rdfifo_head, m_wf_rdfifo_count;
	u8 m_wf_rdfifo_q;
	u8 m_wf_wrfifo[0x200];  // 6809 writes, 6502 reads
	u16 m_wf_wrfifo_head, m_wf_wrfifo_count;
	u8 m_wf_wrfifo_q;

	// PIA-like registers
	u8 m_dd_reg[4];
	u8 m_ddr[4];
	u8 m_key_column;
	u8 m_dtoa_code, m_sound_dtoa;
	u8 m_sbs, m_css, m_vdg_control, m_sel;
	u8 m_hsync_int, m_hsync_pol, m_vsync_int, m_vsync_pol;
	u8 m_cas_mtr;
	u8 m_cart_int, m_cart_pol, m_sound_en;
	u8 m_pdl;

	// extensions
	u8 m_orch[4];           // left, right, left ext, right ext (active)
	u8 m_orch_ext_buf[2];   // buffered extension bytes
	u32 m_gart_write, m_gart_read;
	u8 m_gart_inc;
	u32 m_gart_cnt;
	u8 m_gart_buf;
	u8 m_w_prot, m_mpi_scs, m_mpi_cts;
	u8 m_rom_bank, m_bank[8], m_bank_size;
	u8 m_wf_irq_en, m_wf_baud;
	u8 m_slave_reset, m_slave_add_hi, m_slave_add_lo;
	u8 m_gpio_dir, m_gpio_out;

	// SDRAM
	u8 m_sdram_read;
	u16 m_sdram_din, m_sdram_dout;
	u32 m_sdram_addr;       // 22-bit word address: sector[21:7], index[6:0]

	// main-CPU I2C and DS3231
	u8 m_ck_start, m_ck_done, m_ck_fail, m_ck_complete;
	u8 m_ck_data_in, m_ck_data_out, m_ck_device, m_ck_reg;
	u8 m_rtc_regs[0x13];
	s64 m_rtc_offset;       // seconds added to host time
	emu_timer *m_rtc_timer;

	// GIME-like registers
	u8 m_rom_mode, m_st_scs, m_vec_pag_ram, m_gime_firq, m_gime_irq, m_mmu_en, m_coco1;
	u8 m_timer_ins, m_mmu_tr;
	u8 m_irq_en, m_firq_en;
	u8 m_tmr_msb, m_tmr_lsb, m_tmr_enable;
	u8 m_grmode, m_hres, m_descen, m_mono, m_lpr, m_hlpr, m_lpf, m_cres;
	u16 m_palette[17];
	u8 m_scrn_hsb, m_scrn_msb, m_scrn_lsb, m_vert_fin_scrl, m_hven, m_hor_offset, m_hor_offset_buf;
	u8 m_sam_ext;
	u16 m_mmu[16];
	u8 m_sam_v, m_sam_vert, m_rate, m_ram_mode;

	// interrupt flip-flops (true = set)
	u8 m_hs_stat, m_vs_stat, m_cart1_stat;
	u8 m_irq_stat[6], m_firq_stat[6];
	u8 m_cart_level;        // CART_INT_N level (1 = inactive)
	u8 m_key_level;         // KEY_INT_N level (1 = no key)

	// timer
	u16 m_timer;
	u8 m_blink;
	attotime m_timer_last;
	emu_timer *m_gime_timer;

	// raster state
	emu_timer *m_line_timer;
	attotime m_line_start;
	int m_line_mode_256;
	u16 m_line;
	u8 m_hsync, m_hflag, m_vsync, m_vblank, m_vborder;
	u8 m_vlpr, m_num_row, m_coco1_vlpr, m_underline, m_six_r, m_sg_lines;
	u32 m_row_add;
	bitmap_rgb32 m_bitmap;

	// clock
	u8 m_cent, m_year, m_mnth, m_dmth, m_dwk, m_hour, m_min, m_sec, m_click;

	// SPI
	u8 m_spi_en, m_spi_irq_en, m_spi_ss, m_spi_rreg, m_spi_buffer, m_spi_irq, m_spi_miso;

	// 6551
	u8 m_ser_irq;

	// disk interface
	u8 m_drive_sel_ext, m_motor, m_wrt_prec, m_density, m_halt_en;
	u8 m_command, m_track_reg_w, m_track_ext_w, m_sector, m_data_reg, m_data_ext;
	u8 m_bi_irq_en, m_irq_09_buf2;
	u16 m_buff_add;
	u8 m_track_reg_r, m_track_ext_r, m_status, m_trace;
	u8 m_track1, m_track2, m_heads;
	u8 m_ctrl;              // $F411 control latch
	u8 m_irq_02;            // 6502 command IRQ asserted
	u8 m_halt_state6;
	u8 m_halt_code;
	u8 m_halt_sig;          // HALT to the 6809, readable by the 6502
	emu_timer *m_halt_state_timer;
	u8 m_i2c_device, m_i2c_reg, m_i2c_data_in, m_i2c_data_out;
	u8 m_i2c_start, m_i2c_done, m_i2c_fail, m_i2c_complete;
	emu_timer *m_i2c_timer;

	emu_timer *m_dma_timer;

	// reset
	emu_timer *m_reset_timer;
	u8 m_cpu_reset;
	u8 m_mugs;              // easter-egg latch: ALT+CTRL held after Ctrl-Alt-Ins
	u8 m_reset_ins;         // cocokey RESET_INS (active low); survives resets

	// PS/2 receiver
	u8 m_ps2_clk, m_ps2_data;
	u8 m_ps2_state;         // 0 = hunt start, 1..8 data bits, 9 parity, 10 stop
	u8 m_ps2_shift, m_ps2_released, m_ps2_extended;
	attotime m_ps2_frame_start;

	// cocokey state
	u8 m_key[73];
	u8 m_kb_shift, m_kb_shift_override, m_kb_reset;
	u8 m_shift_arrow_l, m_shift_arrow_r;
	u8 m_slo_reset;         // VSYNC count after reset; keys ignored until 63
};


//**************************************************************************
//  PHYSICAL MEMORY (section 4.1)
//**************************************************************************

u32 coco3fpga_state::chip_offset(u32 phys) const
{
	const int chip = (phys >> 21) & 3;
	return chip * CHIP_SIZE + (phys & m_chip_mask[chip]);
}

u8 coco3fpga_state::phys_r(u32 phys, u8 absent) const
{
	return chip_present(phys) ? m_ram[chip_offset(phys)] : absent;
}

void coco3fpga_state::phys_w(u32 phys, u8 data)
{
	if (chip_present(phys))
		m_ram[chip_offset(phys)] = data;
}


//**************************************************************************
//  INTERRUPTS (section 8)
//**************************************************************************

bool coco3fpga_state::irq_09() const
{
	return (m_density && m_irq_09_buf2) || (m_rdfifo_count && m_bi_irq_en) || (m_wf_rdfifo_count && m_wf_irq_en);
}

// CART_INT_N (section 8.4).  In slots 1 and 3 it toggles every CPU cycle
// unless switch 4 is set; any disk/SPI/serial/WiFi request forces it low.
void coco3fpga_state::update_cart()
{
	const bool request = irq_09() || m_spi_irq || m_ser_irq;
	const bool osc = !BIT(m_mpi_scs, 0) && !BIT(m_switches->read(), 4) && !request;
	const u8 level = request ? 0 : 1;

	if (osc)
	{
		// continuous edges of both polarities
		set_gime(SRC_CART);
		m_cart1_stat = 1;
		m_cart_level = 1;
	}
	else
	{
		if (m_cart_level && !level)
			set_gime(SRC_CART);
		if ((m_cart_level ^ m_cart_pol) && !(level ^ m_cart_pol))
			m_cart1_stat = 1;
		m_cart_level = level;
	}
}

void coco3fpga_state::update_key()
{
	const u8 level = (keyboard_in() == 0xff) ? 1 : 0;
	if (m_key_level && !level)
		set_gime(SRC_KEY);
	m_key_level = level;
}

// HSYNC_INT_N = H_SYNC_N | !H_FLAG changed
void coco3fpga_state::hsync_int_edge(bool falling)
{
	if (falling)
		set_gime(SRC_HBORD);
	if (falling != bool(m_hsync_pol))
		m_hs_stat = 1;
}

// VSYNC_INT_N = V_SYNC_N changed
void coco3fpga_state::vsync_int_edge(bool falling)
{
	if (falling)
		set_gime(SRC_VBORD);
	if (falling != bool(m_vsync_pol))
		m_vs_stat = 1;
}

void coco3fpga_state::update_interrupts()
{
	update_cart();

	bool gime_irq = false, gime_firq = false;
	for (int b : { SRC_CART, SRC_KEY, SRC_VBORD, SRC_HBORD, SRC_TMR })
	{
		gime_irq = gime_irq || (m_irq_stat[b] && BIT(m_irq_en, b));
		gime_firq = gime_firq || (m_firq_stat[b] && BIT(m_firq_en, b));
	}
	const bool pia_irq = (m_hs_stat && m_hsync_int) || (m_vs_stat && m_vsync_int);
	const bool pia_firq = m_cart1_stat && m_cart_int;

	// $FF90 bits 5/4 select between the PIA-style and GIME sources
	const bool irq = m_gime_irq ? gime_irq : pia_irq;
	const bool firq = m_gime_firq ? gime_firq : pia_firq;

	m_maincpu->set_input_line(M6809_IRQ_LINE, irq ? ASSERT_LINE : CLEAR_LINE);
	m_maincpu->set_input_line(M6809_FIRQ_LINE, firq ? ASSERT_LINE : CLEAR_LINE);
}

void coco3fpga_state::update_halt()
{
	const bool halt = BIT(m_ctrl, 7)                       // immediate halt
			|| (m_wrfifo_count >= 0x200)                    // TX FIFO full
			|| m_halt_state6                                // halting command
			|| (BIT(m_ctrl, 6) && BIT(m_buff_add, 8));      // halt after 256 bytes
	m_maincpu->set_input_line(INPUT_LINE_HALT, (halt && !m_cpu_reset) ? ASSERT_LINE : CLEAR_LINE);
	m_halt_sig = halt;
}

void coco3fpga_state::update_nmi()
{
	m_maincpu->set_input_line(INPUT_LINE_NMI, (m_density && BIT(m_ctrl, 5)) ? ASSERT_LINE : CLEAR_LINE);
}

void coco3fpga_state::update_io_irq()
{
	m_iocpu->set_input_line(m6502_device::IRQ_LINE, m_irq_02 ? ASSERT_LINE : CLEAR_LINE);
}

// $FF6E sets SLAVE_RESET, cleared only by a system reset (section 13.3)
void coco3fpga_state::update_slave_reset()
{
	m_iocpu->set_input_line(INPUT_LINE_RESET, (m_cpu_reset || m_slave_reset) ? ASSERT_LINE : CLEAR_LINE);
}

// Command HALT classification, CoCo3IO.v:349-363
void coco3fpga_state::update_halt_code()
{
	const u8 hi = m_command >> 4;
	bool code;
	if (m_command == 0 || hi == 0x9 || hi == 0xb)
		code = false;
	else if (!BIT(m_ctrl, 2) && (hi <= 0x7 || hi == 0xc || hi == 0xd))
		code = false;
	else
		code = true;

	if (code != bool(m_halt_code))
	{
		m_halt_code = code;
		if (code)
		{
			// HALT_STATE counts CPU clocks until bit 6 sets
			m_halt_state_timer->adjust(m_maincpu->cycles_to_attotime(64));
		}
		else
		{
			m_halt_state_timer->adjust(attotime::never);
			m_halt_state6 = 0;
			update_halt();
		}
	}
}

TIMER_CALLBACK_MEMBER(coco3fpga_state::halt_state_expired)
{
	m_halt_state6 = 1;
	update_halt();
}


//**************************************************************************
//  GIME TIMER (section 8.5)
//**************************************************************************

void coco3fpga_state::timer_restart()
{
	m_timer = 0x1fff;
	m_blink = 1;
}

void coco3fpga_state::timer_tick(u32 ticks)
{
	if (!m_tmr_enable)
	{
		m_blink = 1;
		m_timer = 0x1fff;
		return;
	}
	const u16 reload = (m_tmr_msb << 8) | m_tmr_lsb;
	while (ticks)
	{
		if (m_timer == 0)
		{
			// TIMER_INT_N falls
			m_blink ^= 1;
			m_timer = 0x1fff;
			set_gime(SRC_TMR);
			ticks--;
		}
		else if (m_timer == 0x1fff)
		{
			ticks--;
			if (!reload)
				return;                 // stays at $1FFF: no further events
			m_timer = reload;
			// skip whole periods; only the final event can matter
			if (ticks > u32(reload + 2) * 2)
			{
				const u32 periods = ticks / (reload + 2) - 1;
				if (periods & 1)
					m_blink ^= 1;
				ticks -= periods * (reload + 2);
			}
		}
		else
		{
			const u32 n = std::min<u32>(ticks, m_timer);
			m_timer -= n;
			ticks -= n;
		}
	}
}

// 3.57 MHz mode: process elapsed ticks since the last update
void coco3fpga_state::timer_catch_up()
{
	if (!m_timer_ins)
		return;
	const attotime now = machine().time();
	if (now <= m_timer_last)
		return;
	const u64 ticks = (now - m_timer_last).as_ticks(50'000'000) / 14;
	if (!ticks)
		return;
	m_timer_last += attotime::from_ticks(14 * ticks, 50'000'000);
	timer_tick(u32(std::min<u64>(ticks, 0xffffffffU)));
}

void coco3fpga_state::timer_schedule()
{
	if (!m_timer_ins || !m_tmr_enable)
	{
		m_gime_timer->adjust(attotime::never);
		return;
	}
	const u32 reload = (m_tmr_msb << 8) | m_tmr_lsb;
	u32 ticks;
	if (m_timer == 0)
		ticks = 1;
	else if (m_timer == 0x1fff)
		ticks = reload ? (reload + 2) : 0;
	else
		ticks = m_timer + 1;
	if (!ticks)
	{
		m_gime_timer->adjust(attotime::never);
		return;
	}
	const attotime target = m_timer_last + attotime::from_ticks(14 * ticks, 50'000'000);
	const attotime now = machine().time();
	m_gime_timer->adjust(target > now ? target - now : attotime::zero);
}

TIMER_CALLBACK_MEMBER(coco3fpga_state::gime_timer_expired)
{
	timer_catch_up();
	update_interrupts();
	timer_schedule();
}


//**************************************************************************
//  KEYBOARD / JOYSTICK (section 10)
//**************************************************************************

u8 coco3fpga_state::keyboard_in()
{
	// coco3fpga_top.v:4165-4231: KEY[row * 8 + column]
	u8 rows = 0;
	const u8 cols = ~m_key_column;
	const bool egg = !BIT(m_board->read(), 0) || m_mugs;
	for (int r = 0; r < 7; r++)
	{
		u8 pressed = 0;
		for (int c = 0; c < 8; c++)
			if (m_key[r * 8 + c])
				pressed |= 1 << c;
		if (r == 6)
		{
			if (egg)
				pressed |= 0x18;    // button 0 / MUGS force ALT and CTRL
			// shift column: KEY[55] unless overridden, or forced shift
			pressed &= 0x7f;
			if ((m_key[55] && !m_kb_shift_override) || m_kb_shift)
				pressed |= 0x80;
		}
		if (pressed & cols)
			rows |= 1 << r;
	}

	// joystick buttons pull rows low regardless of column
	const u8 btn = ~m_joy_buttons->read();
	if (BIT(btn, 0)) rows |= 0x01;  // right button 1
	if (BIT(btn, 1)) rows |= 0x02;  // left button 1
	if (BIT(btn, 2)) rows |= 0x04;  // left button 2
	if (BIT(btn, 3)) rows |= 0x08;  // right button 2

	// comparator: SEL selects latch 0..3 directly (right X, right Y, left X, left Y)
	const bool jstick = (joy_latch(m_sel & 3) >> 6) >= m_dtoa_code;

	return (~rows & 0x7f) | (jstick ? 0x80 : 0x00);
}


//**************************************************************************
//  PS/2 KEYBOARD (ps2_keyboard.v, cocokey.v)
//**************************************************************************

// Receiver: start bit, eight data bits LSB first, odd parity, stop bit, all
// sampled on falling clock edges.  Frames that take longer than $7FD ticks
// of the 1.5625 MHz receiver clock are abandoned (the KILLER timeout).
void coco3fpga_state::ps2_clock_w(int state)
{
	const bool falling = m_ps2_clk && !state;
	m_ps2_clk = state;
	if (!falling)
		return;

	const attotime now = machine().time();
	if (m_ps2_state && (now - m_ps2_frame_start) > attotime::from_ticks(0x7fd * 32, 50'000'000))
		m_ps2_state = 0;

	switch (m_ps2_state)
	{
	case 0:
		if (!m_ps2_data)
		{
			m_ps2_state = 1;
			m_ps2_shift = 0;
			m_ps2_frame_start = now;
		}
		break;
	case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8:
		m_ps2_shift |= m_ps2_data << (m_ps2_state - 1);
		m_ps2_state++;
		break;
	case 9:
	{
		// PARITY = ~xor(data): the parity bit makes the frame odd
		u8 x = m_ps2_shift;
		x ^= x >> 4;
		x ^= x >> 2;
		x ^= x >> 1;
		if (m_ps2_data == !(x & 1))
			m_ps2_state = 10;
		else
		{
			m_ps2_released = m_ps2_extended = 0;
			m_ps2_state = 0;
		}
		break;
	}
	case 10:
		m_ps2_state = 0;
		ps2_code(m_ps2_shift);
		break;
	}
}

void coco3fpga_state::ps2_code(u8 code)
{
	if (code == 0xe0)
		m_ps2_extended = 1;
	else if (code == 0xf0)
		m_ps2_released = 1;
	else
	{
		const bool press = !m_ps2_released;
		const bool extended = m_ps2_extended;
		m_ps2_released = m_ps2_extended = 0;
		if (m_slo_reset >= 63)
			cocokey_scan(code, press, extended);
	}
}

void coco3fpga_state::cocokey_reset()
{
	std::fill(std::begin(m_key), std::end(m_key), 0);
	m_kb_shift = m_kb_shift_override = m_kb_reset = 0;
	m_shift_arrow_l = m_shift_arrow_r = 0;
}

// cocokey.v:204-704, one call per received make/break code.  Conditions read
// the KEY values from before this code (nonblocking assignment semantics).
void coco3fpga_state::cocokey_scan(u8 scan, bool press, bool extended)
{
	u8 *const k = m_key;
	const bool shifted = k[55];
	const bool arrow_l = m_shift_arrow_l, arrow_r = m_shift_arrow_r;
	const u8 p = press ? 1 : 0;

	// shift-arrow tracking (first case statement)
	switch (scan)
	{
	case 0x12: m_shift_arrow_l = !press; break;
	case 0x59: m_shift_arrow_r = !press; break;
	case 0x00: case 0x6b: case 0x72: case 0x74: case 0x75: break;
	default: m_shift_arrow_l = m_shift_arrow_r = 0; break;
	}

	auto arrows = [&]() { k[55] = arrow_l || arrow_r; k[71] = arrow_l; k[72] = arrow_r; };

	switch (scan)
	{
	case 0x01: k[62] = p; break;                    // F9
	case 0x03: k[58] = p; break;                    // F5
	case 0x04: k[56] = p; break;                    // F3
	case 0x05: k[53] = p; break;                    // F1
	case 0x06: k[54] = p; break;                    // F2
	case 0x07: k[0] = p; break;                     // F12 = @
	case 0x09: k[63] = p; break;                    // F10
	case 0x0a: k[61] = p; break;                    // F8
	case 0x0b: k[59] = p; break;                    // F6
	case 0x0c: k[57] = p; break;                    // F4
	case 0x0d: k[67] = p; break;                    // Tab
	case 0x0e:                                      // ` ~ = CTRL-3
		k[35] = k[52] = p;
		m_kb_shift_override = p;
		break;
	case 0x11: k[51] = p; break;                    // Alt
	case 0x12: k[55] = k[71] = p; break;            // left Shift
	case 0x14: k[52] = p; break;                    // Ctrl
	case 0x15: k[17] = p; break;                    // Q
	case 0x16: k[33] = p; break;                    // 1 !
	case 0x1a: k[26] = p; break;                    // Z
	case 0x1b: k[19] = p; break;                    // S
	case 0x1c: k[1] = p; break;                     // A
	case 0x1d: k[23] = p; break;                    // W
	case 0x1e:                                      // 2 @
		if (press)
		{
			if (!shifted)
				k[34] = 1;
			else
			{
				k[34] = 0;
				k[0] = 1;
				m_kb_shift_override = 1;
			}
		}
		else
			k[34] = k[0] = m_kb_shift_override = 0;
		break;
	case 0x21: k[3] = p; break;                     // C
	case 0x22: k[24] = p; break;                    // X
	case 0x23: k[4] = p; break;                     // D
	case 0x24: k[5] = p; break;                     // E
	case 0x25: k[36] = p; break;                    // 4 $
	case 0x26: k[35] = p; break;                    // 3 #
	case 0x29: k[31] = p; break;                    // Space
	case 0x2a: k[22] = p; break;                    // V
	case 0x2b: k[6] = p; break;                     // F
	case 0x2c: k[20] = p; break;                    // T
	case 0x2d: k[18] = p; break;                    // R
	case 0x2e: k[37] = p; break;                    // 5 %
	case 0x31: k[14] = p; break;                    // N
	case 0x32: k[2] = p; break;                     // B
	case 0x33: k[8] = p; break;                     // H
	case 0x34: k[7] = p; break;                     // G
	case 0x35: k[25] = p; break;                    // Y
	case 0x36:                                      // 6 ^ (^ = CTRL-7)
		if (press)
		{
			if (!shifted)
				k[38] = 1;
			else
			{
				k[39] = k[52] = 1;
				m_kb_shift_override = 1;
			}
		}
		else
			k[38] = k[39] = k[52] = m_kb_shift_override = 0;
		break;
	case 0x3a: k[13] = p; break;                    // M
	case 0x3b: k[10] = p; break;                    // J
	case 0x3c: k[21] = p; break;                    // U
	case 0x3d:                                      // 7 & (& = shift-6)
		if (press)
		{
			if (!shifted) k[39] = 1; else k[38] = 1;
		}
		else
			k[38] = k[39] = 0;
		break;
	case 0x3e:                                      // 8 * (* = shift-:)
		if (press)
		{
			if (!shifted) k[40] = 1; else k[42] = 1;
		}
		else
			k[40] = k[42] = 0;
		break;
	case 0x41: k[44] = p; break;                    // , <
	case 0x42: k[11] = p; break;                    // K
	case 0x43: k[9] = p; break;                     // I
	case 0x44: k[15] = p; break;                    // O
	case 0x45:                                      // 0 ) () = shift-9)
		if (press)
		{
			if (!shifted) k[32] = 1; else k[41] = 1;
		}
		else
			k[32] = k[41] = 0;
		break;
	case 0x46:                                      // 9 ( (( = shift-8)
		if (press)
		{
			if (!shifted) k[41] = 1; else k[40] = 1;
		}
		else
			k[40] = k[41] = 0;
		break;
	case 0x49: k[46] = p; break;                    // . >
	case 0x4a: k[47] = p; break;                    // / ?
	case 0x4b: k[12] = p; break;                    // L
	case 0x4c:                                      // ; :
		if (press)
		{
			if (!shifted)
				k[43] = 1;
			else
			{
				k[42] = 1;
				m_kb_shift_override = 1;
			}
		}
		else
			k[42] = k[43] = m_kb_shift_override = 0;
		break;
	case 0x4d: k[16] = p; break;                    // P
	case 0x4e:                                      // - _ (_ = CTRL--)
		if (press)
		{
			k[45] = 1;
			if (shifted)
			{
				m_kb_shift_override = 1;
				k[52] = 1;
			}
		}
		else
			k[45] = m_kb_shift_override = k[52] = 0;
		break;
	case 0x52:                                      // ' " (' = shift-7, " = shift-2)
		if (press)
		{
			if (!shifted)
			{
				k[39] = 1;
				m_kb_shift = 1;
			}
			else
				k[34] = 1;
		}
		else
			k[34] = k[39] = m_kb_shift = 0;
		break;
	case 0x54:                                      // [ { (CTRL-8, CTRL-,)
		if (press)
		{
			if (!shifted)
				k[40] = k[52] = 1;
			else
			{
				k[44] = k[52] = 1;
				m_kb_shift_override = 1;
			}
		}
		else
			k[40] = k[44] = k[52] = m_kb_shift_override = 0;
		break;
	case 0x55:                                      // = + (= = shift--, + = shift-;)
		if (press)
		{
			if (!shifted)
			{
				k[45] = 1;
				m_kb_shift = 1;
			}
			else
				k[43] = 1;
		}
		else
			k[43] = k[45] = m_kb_shift = 0;
		break;
	case 0x58: k[32] = k[55] = p; break;            // Caps Lock = shift-0
	case 0x59: k[55] = k[72] = p; break;            // right Shift
	case 0x5a: k[48] = p; break;                    // Enter
	case 0x5b:                                      // ] } (CTRL-9, CTRL-.)
		if (press)
		{
			if (!shifted)
				k[41] = k[52] = 1;
			else
			{
				k[46] = k[52] = 1;
				m_kb_shift_override = 1;
			}
		}
		else
			k[41] = k[46] = k[52] = m_kb_shift_override = 0;
		break;
	case 0x5d:                                      // \ | (CTRL-/, CTRL-1)
		if (press)
		{
			if (!shifted)
				k[47] = k[52] = 1;
			else
			{
				k[33] = k[52] = 1;
				m_kb_shift_override = 1;
			}
		}
		else
			k[33] = k[47] = k[52] = m_kb_shift_override = 0;
		break;
	case 0x66: k[29] = p; break;                    // Backspace = left
	case 0x69: if (extended) k[68] = p; else k[33] = p; break;  // End / keypad 1
	case 0x6b:                                      // left / keypad 4
		if (extended) k[29] = p; else k[36] = p;
		arrows();
		break;
	case 0x6c: if (extended) k[49] = p; else k[39] = p; break;  // Home (CLEAR) / keypad 7
	case 0x70:                                      // Ins / keypad 0
		if (extended) k[65] = p; else k[32] = p;
		if (k[51] && k[52] && extended)
		{
			m_reset_ins = !press;
			m_kb_reset = p;
		}
		break;
	case 0x71:                                      // Del / keypad .
		if (extended) k[66] = p; else k[46] = p;
		if (k[51] && k[52] && extended)
		{
			m_reset_ins = 1;
			m_kb_reset = p;
		}
		break;
	case 0x72:                                      // down / keypad 2
		if (extended) k[28] = p; else k[34] = p;
		arrows();
		break;
	case 0x73: if (!extended) k[37] = p; break;     // keypad 5
	case 0x74:                                      // right / keypad 6
		if (extended) k[30] = p; else k[38] = p;
		arrows();
		break;
	case 0x75:                                      // up / keypad 8
		if (extended) k[27] = p; else k[40] = p;
		arrows();
		break;
	case 0x76: k[50] = p; break;                    // Esc = BREAK
	case 0x78: k[64] = p; break;                    // F11
	case 0x7a: if (extended) k[70] = p; else k[35] = p; break;  // Page Down / keypad 3
	case 0x79: k[43] = k[55] = p; break;            // keypad +
	case 0x7b: k[45] = p; break;                    // keypad -
	case 0x7c: k[42] = k[55] = p; break;            // keypad *
	case 0x7d: if (extended) k[69] = p; else k[41] = p; break;  // Page Up / keypad 9
	case 0x7e:                                      // Scroll Lock = CTRL-W
		k[23] = k[52] = p;
		m_kb_shift_override = p;
		break;
	case 0x83: k[60] = p; break;                    // F7
	}

	update_key();
	update_interrupts();

	// Ctrl-Alt-Del / Ctrl-Alt-Ins: RESET_P resets the whole system, which in
	// turn clears cocokey, so the pulse ends by itself
	if (m_kb_reset)
		machine().schedule_soft_reset();
}


//**************************************************************************
//  MAIN CPU ADDRESS DECODE (sections 4, 5)
//**************************************************************************

bool coco3fpga_state::rom_sel(offs_t addr) const
{
	if (m_ram_mode)
		return false;
	if (m_rom_mode == 2)
		return false;
	if (!BIT(m_rom_mode, 1) && (addr & 0xc000) == 0x8000)
		return false;
	if (addr < 0x8000)
		return false;
	if (addr >= 0xfe00)
		return false;
	return true;
}

bool coco3fpga_state::ram_cs(offs_t addr) const
{
	if (rom_sel(addr))
		return false;
	if (!m_ram_mode && addr >= 0x8000 && addr < 0xfe00)
		return false;
	if (addr >= 0xff00)
		return false;
	return true;
}

u32 coco3fpga_state::ram_phys(offs_t addr) const
{
	const u16 fallback = 0x038 | (addr >> 13);
	const int task = m_mmu_tr ? 8 : 0;
	u16 block = fallback;
	if (m_mmu_en)
	{
		if (addr < 0x8000)
			block = m_mmu[task | (addr >> 13)];
		else if (addr < 0xfe00)
			block = rom_sel(addr) ? fallback : m_mmu[task | (addr >> 13)];
		else if (addr < 0xff00)
			block = m_vec_pag_ram ? fallback : m_mmu[task | 7];
	}
	return (u32(block) << 13) | (addr & 0x1fff);
}

// FLASH_CE_S / FLASH_ADDRESS, coco3fpga_top.v:995-1066
int coco3fpga_state::flash_offset(offs_t addr) const
{
	const bool rs = rom_sel(addr);

	if (rs)
	{
		switch (m_mpi_cts)
		{
		case 0: return 0x00a000 | (addr & 0x1fff);     // Orchestra-90
		case 1: return 0x3f8000 | (addr & 0x7fff);     // alternate disk ROM
		case 2:
		{
			const u32 base = u32(m_bank[m_rom_bank]) << 15;
			if (!BIT(m_bank_size, 0))
				return base | (addr & 0x7fff);
			return base | (BIT(m_bank_size, 1) << 14) | (addr & 0x3fff);
		}
		default: return 0x008000 | (addr & 0x1fff);    // Disk BASIC
		}
	}

	if (!m_ram_mode)
	{
		if (!BIT(m_rom_mode, 1) && (addr & 0xc000) == 0x8000)
			return addr & 0x7fff;       // internal 16K
		if (m_rom_mode == 2 && addr >= 0x8000 && addr < 0xfe00)
			return addr & 0x7fff;       // internal 32K
	}
	return -1;
}

u8 coco3fpga_state::main_r(offs_t offset)
{
	const offs_t addr = offset;

	if (addr == 0xff73)
	{
		const u8 data = phys_r(m_gart_read, 0x55);
		if (!machine().side_effects_disabled() && BIT(m_gart_inc, 1))
			m_gart_read = (m_gart_read + 1) & 0x7fffff;
		return data;
	}
	if (ram_cs(addr))
	{
		const u32 phys = ram_phys(addr);
		if (chip_present(phys))
			return m_ram[chip_offset(phys)];
		return 0x55;                    // nothing selected: default
	}
	const int f = flash_offset(addr);
	if (f >= 0)
		return machine().side_effects_disabled() ? m_flash->read_raw(f) : m_flash->read(f);
	if (addr < 0xff00)
		return 0x55;
	return hw_r(addr);
}

void coco3fpga_state::main_w(offs_t offset, u8 data)
{
	const offs_t addr = offset;

	if (addr == 0xff73)
	{
		phys_w(m_gart_write, data);
		if (BIT(m_gart_inc, 0))
			m_gart_write = (m_gart_write + 1) & 0x7fffff;
		return;
	}
	if (ram_cs(addr))
	{
		phys_w(ram_phys(addr), data);
		return;
	}
	if (!BIT(m_w_prot, 1))
	{
		// FLASH_WE_N = !(ROM_RW & FLASH_CE_S): writes reach the flash chip
		const int f = flash_offset(addr);
		if (f >= 0)
		{
			m_flash->write(f, data);
			return;
		}
	}
	if (addr >= 0xff00)
		hw_w(addr, data);
}

int coco3fpga_state::current_dot() const
{
	const attotime delta = machine().time() - m_line_start;
	return int(delta.as_ticks(PIXEL_CLOCK)) % LINE_DOTS;
}

u8 coco3fpga_state::hw_r(offs_t addr)
{
	const bool se = !machine().side_effects_disabled();

	// priority chain of the DATA_IN mux, coco3fpga_top.v:1171-1320
	if (BIT(m_mpi_scs, 0) && (addr & 0xfff0) == 0xff40)
		return hdd_r(addr & 0x0f);
	if ((addr & 0xfffc) == 0xff68)
		return m_uart->read(addr & 3);
	if (m_mpi_scs == 2 && (addr & 0xffe0) == 0xff40)
		return m_rom_bank & 7;
	if (addr == 0xff6d)
	{
		if (se && m_wf_rdfifo_count)
		{
			m_wf_rdfifo_q = m_wf_rdfifo[m_wf_rdfifo_head];
			m_wf_rdfifo_head = (m_wf_rdfifo_head + 1) & 0x3ff;
			m_wf_rdfifo_count--;
			update_interrupts();
		}
		return m_wf_rdfifo_q;
	}
	if ((addr & 0xfffe) == 0xff64)
		return spi_r(addr & 1);

	if (addr < 0xff40)
	{
		u8 data;
		const int reg = ((addr & 0x20) >> 4) | (addr & 3);
		switch (reg)
		{
		case 0: // FF00: reads clear the HSYNC flag
			data = m_ddr[0] ? keyboard_in() : m_dd_reg[0];
			if (se && m_hs_stat)
			{
				m_hs_stat = 0;
				update_interrupts();
			}
			return data;
		case 1:
			return (m_hs_stat ? 0x80 : 0) | 0x30 | (BIT(m_sel, 0) << 3)
					| (m_ddr[0] << 2) | (m_hsync_pol << 1) | m_hsync_int;
		case 2: // FF02: reads clear the VSYNC flag
			data = m_ddr[1] ? m_key_column : m_dd_reg[1];
			if (se && m_vs_stat)
			{
				m_vs_stat = 0;
				update_interrupts();
			}
			return data;
		case 3:
			return (m_vs_stat ? 0x80 : 0) | 0x30 | (BIT(m_sel, 1) << 3)
					| (m_ddr[1] << 2) | (m_vsync_pol << 1) | m_vsync_int;
		case 4: // FF20
			return m_ddr[2] ? ((m_dtoa_code << 2) | 0x03) : m_dd_reg[2];
		case 5:
			return 0x30 | (m_cas_mtr << 3) | (m_ddr[2] << 2);
		case 6: // FF22: any access clears the CART flag
			if (m_ddr[3])
				data = (m_vdg_control << 4) | ((BIT(m_dd_reg[3], 3) && m_css) << 3)
						| (BIT(m_key_column, 6) << 2) | (m_sbs << 1) | 0x01;
			else
				data = m_dd_reg[3];
			if (se && m_cart1_stat)
			{
				m_cart1_stat = 0;
				update_interrupts();
			}
			return data;
		case 7:
			return (m_cart1_stat ? 0x80 : 0) | 0x30 | (m_sound_en << 3)
					| (m_ddr[3] << 2) | (m_cart_pol << 1) | m_cart_int;
		}
	}

	switch (addr)
	{
	case 0xff60: return joy_latch(m_pdl ? 2 : 0) >> 4;
	case 0xff61: return (joy_latch(m_pdl ? 2 : 0) & 0x0f) << 4;
	case 0xff62: return joy_latch(m_pdl ? 3 : 1) >> 4;
	case 0xff63: return (joy_latch(m_pdl ? 3 : 1) & 0x0f) << 4;
	case 0xff6c:
		return ((m_wf_rdfifo_count && m_wf_irq_en) ? 0x80 : 0) | (m_wf_rdfifo_count ? 0x02 : 0)
				| ((m_wf_wrfifo_count >= 0x200) ? 0x01 : 0);
	case 0xff70: return (m_gart_write >> 16) & 0x7f;
	case 0xff71: return (m_gart_write >> 8) & 0xff;
	case 0xff72: return m_gart_write & 0xff;
	case 0xff74: return (m_gart_read >> 16) & 0x7f;
	case 0xff75: return (m_gart_read >> 8) & 0xff;
	case 0xff76: return m_gart_read & 0xff;
	case 0xff77: return (m_gart_cnt ? 0 : 0x80) | m_gart_inc;
	case 0xff7f: return 0xc0 | (m_mpi_cts << 4) | (m_w_prot << 2) | m_mpi_scs;
	case 0xff80: return (m_ck_done << 7) | (m_ck_fail << 6);
	case 0xff81: return m_ck_data_in;
	case 0xff84: return 0x80 | 0x02 | m_sdram_read;   // ready, state 001
	case 0xff85: return m_sdram_dout & 0xff;
	case 0xff86:
	{
		const u8 data = m_sdram_dout >> 8;
		if (se)
			sdram_start();
		return data;
	}
	case 0xff87: return (m_sdram_addr >> 15) & 0x7f;
	case 0xff88:
		if (se)
			m_sdram_addr = (m_sdram_addr & ~0x7f) | 0x7f;
		return (m_sdram_addr >> 7) & 0xff;
	case 0xff8e: return m_gpio_dir;
	case 0xff8f: return m_gpio_out & m_gpio_dir;  // inputs float low (unverified)
	case 0xff90:
		return (m_coco1 << 7) | (m_mmu_en << 6) | (m_gime_irq << 5) | (m_gime_firq << 4)
				| (m_vec_pag_ram << 3) | (m_st_scs << 2) | m_rom_mode;
	case 0xff91: return (m_timer_ins << 5) | m_mmu_tr;
	case 0xff92:
	case 0xff93:
	{
		u8 *stat = (addr == 0xff92) ? m_irq_stat : m_firq_stat;
		const u8 en = (addr == 0xff92) ? m_irq_en : m_firq_en;
		if (se)
			timer_catch_up();
		u8 data = 0;
		for (int b : { SRC_CART, SRC_KEY, SRC_VBORD, SRC_HBORD, SRC_TMR })
			if (stat[b] && BIT(en, b))
				data |= 1 << b;
		if (se)
		{
			// reads clear the flip-flops (writes do not)
			for (int b : { SRC_CART, SRC_KEY, SRC_VBORD, SRC_HBORD, SRC_TMR })
				stat[b] = 0;
			update_interrupts();
		}
		return data;
	}
	case 0xff94: return m_tmr_msb;
	case 0xff95: return m_tmr_lsb;
	case 0xffc0: return m_cent;
	case 0xffc1: return m_year;
	case 0xffc2: return m_mnth;
	case 0xffc3: return m_dmth;
	case 0xffc4: return m_dwk;
	case 0xffc5: return m_hour;
	case 0xffc6: return m_min;
	case 0xffc7: return m_sec;
	case 0xffcc:
		return (m_key[51] << 7) | (m_key[52] << 6) | (m_key[72] << 5) | (m_key[71] << 4)
				| (m_key[28] << 3) | (m_key[27] << 2) | (m_key[30] << 1) | m_key[29];
	case 0xffcd:
		return (m_key[70] << 7) | (m_key[69] << 6) | (m_key[65] << 5) | (m_key[66] << 4)
				| (m_key[67] << 3) | (m_key[68] << 2);
	case 0xffce:
		return (m_key[61] << 7) | (m_key[60] << 6) | (m_key[59] << 5) | (m_key[58] << 4)
				| (m_key[57] << 3) | (m_key[56] << 2) | (m_key[54] << 1) | m_key[53];
	case 0xffcf:
	{
		const int dot = current_dot();
		const int end = m_line_mode_256 ? 528 : 656;
		const bool hblank = !(dot >= 16 && dot < end);
		return ((!m_vsync) << 7) | (m_vblank << 6) | ((!m_hsync) << 5) | (hblank << 4)
				| (m_key[0] << 3) | (m_key[64] << 2) | (m_key[63] << 1) | m_key[62];
	}
	// fixed vectors and revision (section 4.4)
	case 0xfff0: return 0x41;
	case 0xfff1: return BIT(m_config->read(), 0) ? 0x05 : 0x01;
	}

	if (addr >= 0xfff2)
	{
		static constexpr u8 vectors[14] = { 0xfe, 0xee, 0xfe, 0xf1, 0xfe, 0xf4, 0xfe, 0xf7, 0xfe, 0xfa, 0xfe, 0xfd, 0x8c, 0x1b };
		return vectors[addr - 0xfff2];
	}
	if (addr >= 0xffa0 && addr <= 0xffaf)
		return m_mmu[addr & 0x0f] & 0xff;
	if (addr >= 0xffb0 && addr <= 0xffbf)
		return m_palette[addr & 0x0f] & 0x3f;

	return 0x55;
}

void coco3fpga_state::hw_w(offs_t addr, u8 data)
{
	if (addr < 0xff40)
	{
		const int reg = ((addr & 0x20) >> 4) | (addr & 3);
		switch (reg)
		{
		case 0:
			if (!m_ddr[0])
				m_dd_reg[0] = data;
			break;
		case 1:
		{
			// the polarity XOR feeds the flip-flop clock: a polarity change
			// can itself make an edge
			const bool src = !m_hsync && m_hflag;      // HSYNC_INT_N low
			const bool old_clk = !src ^ bool(m_hsync_pol);
			m_hsync_int = BIT(data, 0);
			m_hsync_pol = BIT(data, 1);
			m_ddr[0] = BIT(data, 2);
			m_sel = (m_sel & 2) | BIT(data, 3);
			if (old_clk && !(!src ^ bool(m_hsync_pol)))
				m_hs_stat = 1;
			update_sound_dtoa();
			break;
		}
		case 2:
			if (!m_ddr[1])
				m_dd_reg[1] = data;
			else
				m_key_column = data;
			update_key();
			break;
		case 3:
		{
			const bool old_clk = bool(m_vsync) ^ bool(m_vsync_pol);
			m_vsync_int = BIT(data, 0);
			m_vsync_pol = BIT(data, 1);
			m_ddr[1] = BIT(data, 2);
			m_sel = (m_sel & 1) | (BIT(data, 3) << 1);
			if (old_clk && !(bool(m_vsync) ^ bool(m_vsync_pol)))
				m_vs_stat = 1;
			update_sound_dtoa();
			break;
		}
		case 4:
			if (!m_ddr[2])
				m_dd_reg[2] = data;
			else
				m_dtoa_code = data >> 2;
			update_sound_dtoa();
			update_key();
			break;
		case 5:
			m_ddr[2] = BIT(data, 2);
			m_cas_mtr = BIT(data, 3);
			break;
		case 6:
			if (!m_ddr[3])
				m_dd_reg[3] = data;
			else
			{
				m_sbs = BIT(data, 1);
				m_css = BIT(data, 3);
				m_vdg_control = data >> 4;
				update_audio();
			}
			m_cart1_stat = 0;               // any access clears the CART flag
			break;
		case 7:
			if (BIT(data, 1) != m_cart_pol)
				m_cart1_stat = 0;           // polarity change resets the flip-flop
			m_cart_int = BIT(data, 0);
			m_cart_pol = BIT(data, 1);
			m_ddr[3] = BIT(data, 2);
			m_sound_en = BIT(data, 3);
			update_sound_dtoa();
			break;
		}
		update_interrupts();
		return;
	}

	if ((addr & 0xffe0) == 0xff40)
	{
		if (m_mpi_scs == 2)
		{
			switch (addr & 0x1f)
			{
			case 0x00: m_rom_bank = data & 7; break;
			case 0x02: m_bank[0] = data & 0x7f; break;
			case 0x03: m_bank_size = data & 3; break;
			case 0x04: case 0x05: case 0x06: case 0x07: case 0x08: case 0x09: case 0x0a:
				m_bank[(addr & 0x1f) - 3] = data & 0x7f;
				break;
			}
		}
		if (BIT(m_mpi_scs, 0) && (addr & 0xfff0) == 0xff40)
			hdd_w(addr & 0x0f, data);
		return;
	}

	if ((addr & 0xfffe) == 0xff64)
	{
		spi_w(addr & 1, data);
		return;
	}
	if ((addr & 0xfffc) == 0xff68)
	{
		m_uart->write(addr & 3, data);
		return;
	}

	if (addr >= 0xffa0 && addr <= 0xffaf)
	{
		m_mmu[addr & 0x0f] = (u16(m_sam_ext) << 8) | data;
		return;
	}
	if ((addr >= 0xffb0 && addr <= 0xffbf) || addr == 0xff9a)
	{
		u16 &p = m_palette[(addr == 0xff9a) ? 16 : (addr & 0x0f)];
		if (!BIT(data, 7))
			p = ((data & 0x3f) << 6) | (data & 0x3f);
		else
			p = (p & 0xfc0) | (data & 0x3f);
		return;
	}
	if (addr >= 0xffc0 && addr <= 0xffdf)
	{
		const int bit = (addr - 0xffc0) >> 1;
		const bool set = BIT(addr, 0);
		if (bit < 3)
			m_sam_v = (m_sam_v & ~(1 << bit)) | (set << bit);
		else if (bit < 10)
			m_sam_vert = (m_sam_vert & ~(1 << (bit - 3))) | (set << (bit - 3));
		else if (bit == 12)
		{
			m_rate = set;
			update_cpu_clock();
		}
		else if (bit == 15)
			m_ram_mode = set;
		return;
	}

	switch (addr)
	{
	case 0xff60: m_pdl = BIT(data, 0); break;
	case 0xff6c:
		m_wf_irq_en = BIT(data, 7);
		m_wf_baud = data & 3;
		update_acia_clocks();
		update_interrupts();
		break;
	case 0xff6d:
		if (m_wf_wrfifo_count < 0x200)
		{
			m_wf_wrfifo[(m_wf_wrfifo_head + m_wf_wrfifo_count) & 0x1ff] = data;
			m_wf_wrfifo_count++;
		}
		m_slave_add_hi = data;          // same write also sets the loader address
		break;
	case 0xff6e:
		m_slave_add_lo = data;
		m_slave_reset = 1;
		update_slave_reset();
		break;
	case 0xff6f:
		m_fw[BIT(m_slave_add_hi, 4)][((m_slave_add_hi & 0x0f) << 8) | m_slave_add_lo] = data;
		break;
	case 0xff70: m_gart_write = (m_gart_write & 0x00ffff) | (u32(data & 0x7f) << 16); break;
	case 0xff71: m_gart_write = (m_gart_write & 0x7f00ff) | (u32(data) << 8); break;
	case 0xff72: m_gart_write = (m_gart_write & 0x7fff00) | data; break;
	case 0xff74: m_gart_read = (m_gart_read & 0x00ffff) | (u32(data & 0x7f) << 16); break;
	case 0xff75: m_gart_read = (m_gart_read & 0x7f00ff) | (u32(data) << 8); break;
	case 0xff76: m_gart_read = (m_gart_read & 0x7fff00) | data; break;
	case 0xff77: m_gart_inc = data & 3; break;
	case 0xff78: m_gart_cnt = (m_gart_cnt & 0x001ff) | (u32(data) << 9); dma_kick(); break;
	case 0xff79: m_gart_cnt = (m_gart_cnt & 0x1fe00) | (u32(data) << 1); dma_kick(); break;
	case 0xff7a: m_orch[0] = data; m_orch[2] = m_orch_ext_buf[0]; update_audio(); break;
	case 0xff7b: m_orch[1] = data; m_orch[3] = m_orch_ext_buf[1]; update_audio(); break;
	case 0xff7c: m_orch_ext_buf[0] = data; break;
	case 0xff7d: m_orch_ext_buf[1] = data; break;
	case 0xff7e: m_vdac[data] = m_palette[0] & 0xfff; break;
	case 0xff7f:
	{
		// nonblocking assignment: protect1 uses the old protect0
		const u8 p0 = BIT(data, 2) | !BIT(data, 3);
		const u8 p1 = !BIT(data, 2) | BIT(data, 3) | BIT(m_w_prot, 0);
		m_w_prot = (p1 << 1) | p0;
		m_mpi_scs = data & 3;
		m_mpi_cts = (data >> 4) & 3;
		update_interrupts();
		break;
	}
	case 0xff80:
	{
		const u8 start = BIT(data, 0);
		if (start && !m_ck_start)
		{
			m_ck_done = 0;
			m_ck_fail = 0;
			m_ck_complete = 0;
			m_rtc_timer->adjust(attotime::from_usec(100));
		}
		else if (!start && m_ck_complete)
			m_ck_done = 1;
		m_ck_start = start;
		break;
	}
	case 0xff81: m_ck_data_out = data; break;
	case 0xff82: m_ck_device = data; break;
	case 0xff83: m_ck_reg = data; break;
	case 0xff84: m_sdram_read = BIT(data, 0); break;
	case 0xff85: m_sdram_din = (m_sdram_din & 0xff00) | data; break;
	case 0xff86: m_sdram_din = (m_sdram_din & 0x00ff) | (data << 8); sdram_start(); break;
	case 0xff87: m_sdram_addr = (m_sdram_addr & 0x007fff) | (u32(data & 0x7f) << 15); break;
	case 0xff88: m_sdram_addr = (m_sdram_addr & 0x3f8000) | (u32(data) << 7) | 0x7f; break;
	case 0xff8e: m_gpio_dir = data; break;
	case 0xff8f: m_gpio_out = data; break;
	case 0xff90:
		m_rom_mode = data & 3;
		m_st_scs = BIT(data, 2);
		m_vec_pag_ram = BIT(data, 3);
		m_gime_firq = BIT(data, 4);
		m_gime_irq = BIT(data, 5);
		m_mmu_en = BIT(data, 6);
		m_coco1 = BIT(data, 7);
		update_interrupts();
		break;
	case 0xff91:
		timer_catch_up();
		m_timer_ins = BIT(data, 5);
		m_mmu_tr = BIT(data, 0);
		m_timer_last = machine().time();
		timer_schedule();
		break;
	case 0xff92:
		m_irq_en = data & 0x3b;
		update_interrupts();
		break;
	case 0xff93:
		m_firq_en = data & 0x3b;
		update_interrupts();
		break;
	case 0xff94:
		m_tmr_msb = data & 0x0f;
		m_tmr_enable = 1;           // any write enables; never cleared but by reset
		timer_restart();
		m_timer_last = machine().time();
		timer_schedule();
		break;
	case 0xff95:
		m_tmr_lsb = data;
		timer_restart();
		m_timer_last = machine().time();
		timer_schedule();
		break;
	case 0xff98:
		m_grmode = BIT(data, 7);
		m_hres = (m_hres & 7) | (BIT(data, 6) << 3);
		m_descen = BIT(data, 5);
		m_mono = BIT(data, 4);
		m_lpr = data & 7;
		break;
	case 0xff99:
		m_hlpr = BIT(data, 7);
		m_lpf = (data >> 5) & 3;
		m_hres = (m_hres & 8) | ((data >> 2) & 7);
		m_cres = data & 3;
		break;
	case 0xff9b:
		m_scrn_hsb = data & 0x0f;
		m_sam_ext = (data >> 4) & 3;
		break;
	case 0xff9c: m_vert_fin_scrl = data & 0x0f; break;
	case 0xff9d: m_scrn_msb = data; break;
	case 0xff9e: m_scrn_lsb = data; break;
	case 0xff9f:
		m_hven = BIT(data, 7);
		m_hor_offset = data & 0x7f;
		break;
	}
}


//**************************************************************************
//  BLOCK COPY (section 6)
//**************************************************************************

void coco3fpga_state::dma_kick()
{
	if (m_gart_cnt)
		m_dma_timer->adjust(m_maincpu->cycles_to_attotime(16));
	else
		m_dma_timer->adjust(attotime::never);
}

// approximation: one step per CPU clock, processed 16 at a time
TIMER_CALLBACK_MEMBER(coco3fpga_state::dma_step)
{
	for (int i = 0; i < 16 && m_gart_cnt; i++)
	{
		if (!BIT(m_gart_cnt, 0))
		{
			m_gart_buf = phys_r(m_gart_read, 0x55);
			if (BIT(m_gart_inc, 1))
				m_gart_read = (m_gart_read + 1) & 0x7fffff;
		}
		else
		{
			phys_w(m_gart_write, m_gart_buf);
			if (BIT(m_gart_inc, 0))
				m_gart_write = (m_gart_write + 1) & 0x7fffff;
		}
		m_gart_cnt--;
	}
	dma_kick();
}


//**************************************************************************
//  SDRAM SECTOR STORE (section 11)
//**************************************************************************

// any $FF86 access increments the word index, then runs the operation
void coco3fpga_state::sdram_start()
{
	m_sdram_addr = (m_sdram_addr & 0x3fff80) | ((m_sdram_addr + 1) & 0x7f);
	const u32 byte = (m_sdram_addr & 0x3fffff) << 1;
	if (m_sdram_read)
		m_sdram_dout = m_sdram[byte] | (m_sdram[byte + 1] << 8);
	else
	{
		m_sdram[byte] = m_sdram_din & 0xff;
		m_sdram[byte + 1] = m_sdram_din >> 8;
	}
}


//**************************************************************************
//  MAIN-CPU I2C AND DS3231 (section 12.1)
//**************************************************************************

void coco3fpga_state::rtc_now(int t[7])
{
	system_time st;
	machine().current_datetime(st);
	const s64 secs = st.time + m_rtc_offset;
	const system_time adj(time_t(secs + 0));
	const auto &lt = adj.utc_time;
	t[0] = lt.second;
	t[1] = lt.minute;
	t[2] = lt.hour;
	t[3] = lt.weekday + 1;
	t[4] = lt.mday;
	t[5] = lt.month + 1;
	t[6] = lt.year;
}

static u8 to_bcd(int v) { return ((v / 10) << 4) | (v % 10); }
static int from_bcd(u8 v) { return (v >> 4) * 10 + (v & 0x0f); }

u8 coco3fpga_state::rtc_reg_r(u8 reg)
{
	if (reg > 0x12)
		return 0xff;
	if (reg <= 6)
	{
		int t[7];
		rtc_now(t);
		switch (reg)
		{
		case 0: return to_bcd(t[0]);
		case 1: return to_bcd(t[1]);
		case 2: return to_bcd(t[2]);
		case 3: return t[3];
		case 4: return to_bcd(t[4]);
		case 5: return to_bcd(t[5]) | ((t[6] >= 2000) ? 0x80 : 0);
		case 6: return to_bcd(t[6] % 100);
		}
	}
	return m_rtc_regs[reg];
}

void coco3fpga_state::rtc_reg_w(u8 reg, u8 data)
{
	if (reg > 0x12)
		return;
	if (reg <= 6)
	{
		// rebuild the calendar time with one field replaced
		int t[7];
		rtc_now(t);
		switch (reg)
		{
		case 0: t[0] = from_bcd(data & 0x7f); break;
		case 1: t[1] = from_bcd(data & 0x7f); break;
		case 2: t[2] = from_bcd(data & 0x3f); break;
		case 3: return;
		case 4: t[4] = from_bcd(data & 0x3f); break;
		case 5: t[5] = from_bcd(data & 0x1f); t[6] = (t[6] % 100) + (BIT(data, 7) ? 2000 : 1900); break;
		case 6: t[6] = (t[6] / 100) * 100 + from_bcd(data); break;
		}
		// days from civil (proleptic Gregorian)
		const int y = t[6] - (t[5] <= 2);
		const int era = (y >= 0 ? y : y - 399) / 400;
		const int yoe = y - era * 400;
		const int doy = (153 * (t[5] + (t[5] > 2 ? -3 : 9)) + 2) / 5 + t[4] - 1;
		const int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
		const s64 days = s64(era) * 146097 + doe - 719468;
		const s64 wanted = days * 86400 + t[2] * 3600 + t[1] * 60 + t[0];
		system_time st;
		machine().current_datetime(st);
		m_rtc_offset = wanted - st.time;
		return;
	}
	if (reg <= 0x10)
		m_rtc_regs[reg] = data;
}

// register-addressed single-byte transfer; only the DS3231 answers
bool coco3fpga_state::rtc_transaction()
{
	if ((m_ck_device >> 1) != 0x68)
		return false;
	if (BIT(m_ck_device, 0))
		m_ck_data_in = rtc_reg_r(m_ck_reg);
	else
		rtc_reg_w(m_ck_reg, m_ck_data_out);
	return true;
}

TIMER_CALLBACK_MEMBER(coco3fpga_state::rtc_i2c_done)
{
	m_ck_fail = rtc_transaction() ? 0 : 1;
	m_ck_complete = 1;
	if (!m_ck_start)
		m_ck_done = 1;
}


//**************************************************************************
//  DISK INTERFACE, 6809 SIDE (section 13.2)
//**************************************************************************

u8 coco3fpga_state::hdd_r(offs_t reg)
{
	const bool se = !machine().side_effects_disabled();
	const bool addr_rst_n = BIT(m_ctrl, 3);

	switch (reg)
	{
	case 0x0:
		return (m_halt_en << 7) | (BIT(m_drive_sel_ext, 3) << 6) | (m_density << 5)
				| (m_wrt_prec << 4) | (m_motor << 3) | (m_drive_sel_ext & 7);
	case 0x1:
		return (irq_09() ? 0x80 : 0) | (addr_rst_n << 6) | (m_rdfifo_count ? 0x02 : 0) | m_bi_irq_en;
	case 0x2:
		if (se && m_rdfifo_count)
		{
			m_rdfifo_q = m_rdfifo[m_rdfifo_head];
			m_rdfifo_head = (m_rdfifo_head + 1) & 0x1ff;
			m_rdfifo_count--;
			update_interrupts();
		}
		return m_rdfifo_q;
	case 0x3:
		return 0x03;
	case 0x4:
		return m_drive_sel_ext;
	case 0x8:
		return (m_status & 0xfe) | BIT(m_ctrl, 0);
	case 0x9:
		return m_track_reg_r;
	case 0xa:
		return m_sector;
	case 0xb:
	{
		u8 data;
		if (!addr_rst_n)
			data = m_data_reg;
		else
		{
			data = m_rdbuf[m_buff_add & 0x1ff];
			if (se)
			{
				m_buff_add = (m_buff_add + 1) & 0x1ff;
				update_halt();
			}
		}
		return data;
	}
	case 0xd:
		return m_track_ext_r;
	case 0xf:
		return m_data_ext;
	}
	return 0x00;
}

void coco3fpga_state::hdd_w(offs_t reg, u8 data)
{
	switch (reg)
	{
	case 0x0:
		m_drive_sel_ext = (BIT(data, 6) << 3) | (data & 7);
		m_motor = BIT(data, 3);
		m_wrt_prec = BIT(data, 4);
		m_density = BIT(data, 5);
		m_halt_en = BIT(data, 7);
		update_nmi();
		update_interrupts();
		break;
	case 0x1:
		m_irq_09_buf2 = 0;
		m_bi_irq_en = BIT(data, 0);
		update_interrupts();
		break;
	case 0x2:
		if (m_wrfifo_count < 0x200)
		{
			m_wrfifo[(m_wrfifo_head + m_wrfifo_count) & 0x1ff] = data;
			m_wrfifo_count++;
			update_halt();
		}
		break;
	case 0x4:
		m_drive_sel_ext = data;
		break;
	case 0x8:
		m_command = data;
		m_irq_02 = !BIT(m_ctrl, 0);     // signal the 6502 unless it is busy
		if (BIT(m_ctrl, 1))
			m_command = 0;
		update_io_irq();
		update_halt_code();
		break;
	case 0x9:
		m_track_reg_w = data;
		m_track_ext_w = 0;
		break;
	case 0xa:
		m_sector = data;
		break;
	case 0xb:
		m_data_reg = data;
		m_data_ext = 0;
		m_wrbuf[m_buff_add & 0x1ff] = data;
		if (BIT(m_ctrl, 3))
		{
			m_buff_add = (m_buff_add + 1) & 0x1ff;
			update_halt();
		}
		break;
	case 0xd:
		m_track_ext_w = data;
		break;
	case 0xf:
		m_data_ext = data;
		break;
	}
}


//**************************************************************************
//  I/O PROCESSOR (6502, section 13)
//**************************************************************************

// $F400-$F4FF control registers, mirrored by the low five address bits
u8 coco3fpga_state::ioreg_r(offs_t addr)
{
	const bool se = !machine().side_effects_disabled();

	switch (addr & 0x1f)
	{
	case 0x00: return m_dwk;
	case 0x01: return m_sec;
	case 0x02: return m_min;
	case 0x03: return m_hour;
	case 0x04: return m_dmth;
	case 0x05: return m_mnth;
	case 0x06: return m_year;
	case 0x07: return m_cent;
	case 0x08: return m_drive_sel_ext;
	case 0x09: return m_sector;
	case 0x0a: return m_track_reg_w;
	case 0x0b: return m_track_reg_r;
	case 0x0c: return m_track_ext_w;
	case 0x0d: return m_track_ext_r;
	case 0x0e: return m_command;
	case 0x0f: return m_data_ext;
	case 0x10: return m_status;
	case 0x11: return m_ctrl;
	case 0x12:
		return (m_halt_sig ? 0x80 : 0) | (BIT(m_buff_add, 8) << 6)
				| ((m_wf_rdfifo_count >= 0x400) ? 0x10 : 0) | (m_wf_wrfifo_count ? 0x08 : 0)
				| ((m_rdfifo_count >= 0x200) ? 0x04 : 0) | (m_wrfifo_count ? 0x02 : 0) | BIT(m_sec, 5);
	case 0x13: return m_irq_02 ? 0x80 : 0x00;
	case 0x14: return 0x00;
	case 0x15:
		return (m_halt_en << 7) | (m_density << 6) | (m_wrt_prec << 5) | (m_motor << 4) | (m_drive_sel_ext & 0x0f);
	case 0x16: return m_trace;
	case 0x17: return m_track1;
	case 0x18: return m_track2;
	case 0x19: return m_heads;
	case 0x1a:
		if (se && m_wrfifo_count)
		{
			m_wrfifo_q = m_wrfifo[m_wrfifo_head];
			m_wrfifo_head = (m_wrfifo_head + 1) & 0x1ff;
			m_wrfifo_count--;
			update_halt();
		}
		return m_wrfifo_q;
	case 0x1b: return m_i2c_device;
	case 0x1c: return m_i2c_reg;
	case 0x1d: return m_i2c_data_in;
	case 0x1e: return (m_i2c_done << 7) | (m_i2c_fail << 6) | (m_i2c_start << 5);
	case 0x1f:
		if (se && m_wf_wrfifo_count)
		{
			m_wf_wrfifo_q = m_wf_wrfifo[m_wf_wrfifo_head];
			m_wf_wrfifo_head = (m_wf_wrfifo_head + 1) & 0x1ff;
			m_wf_wrfifo_count--;
		}
		return m_wf_wrfifo_q;
	}
	return 0xaa;
}

void coco3fpga_state::ioreg_w(offs_t addr, u8 data)
{
	// clock fields: exact addresses only
	switch (addr)
	{
	case 0xf400: m_dwk = data & 0x07; return;
	case 0xf401: m_sec = data & 0x3f; return;
	case 0xf402: m_min = data & 0x3f; return;
	case 0xf403: m_hour = data & 0x1f; return;
	case 0xf404: m_dmth = data & 0x1f; return;
	case 0xf405: m_mnth = data & 0x0f; return;
	case 0xf406: m_year = data & 0x7f; return;
	case 0xf407: m_cent = data & 0x1f; return;
	}

	switch (addr & 0x1f)
	{
	case 0x0b: m_track_reg_r = data; break;
	case 0x0d: m_track_ext_r = data; break;
	case 0x10: m_status = data; break;
	case 0x11:
		m_ctrl = data;
		if (BIT(data, 0))
			m_irq_02 = 0;               // busy acknowledges the command IRQ
		if (BIT(data, 1))
			m_command = 0;
		if (BIT(data, 4))
			m_irq_09_buf2 = 1;          // set, but not cleared, from here
		if (BIT(data, 5))
			m_halt_en = 0;
		if (!BIT(data, 3))
			m_buff_add = 0;
		update_io_irq();
		update_halt_code();
		update_halt();
		update_nmi();
		update_interrupts();
		break;
	case 0x16: m_trace = data; break;
	case 0x1a:
		if (m_rdfifo_count < 0x200)
		{
			m_rdfifo[(m_rdfifo_head + m_rdfifo_count) & 0x1ff] = data;
			m_rdfifo_count++;
			update_interrupts();
		}
		break;
	case 0x1b: m_i2c_device = data; break;
	case 0x1c: m_i2c_reg = data; break;
	case 0x1d: m_i2c_data_out = data; break;
	case 0x1e:
	{
		const u8 start = BIT(data, 5);
		if (start && !m_i2c_start)
		{
			m_i2c_done = 0;
			m_i2c_fail = 0;
			m_i2c_complete = 0;
			// device, register and data bytes at 390.625 kHz / 4 per bit, roughly
			m_i2c_timer->adjust(attotime::from_usec(300));
		}
		else if (!start && m_i2c_complete)
			m_i2c_done = 1;
		m_i2c_start = start;
		break;
	}
	case 0x1f:
		if (m_wf_rdfifo_count < 0x400)
		{
			m_wf_rdfifo[(m_wf_rdfifo_head + m_wf_rdfifo_count) & 0x3ff] = data;
			m_wf_rdfifo_count++;
			update_interrupts();
		}
		break;
	}
}

TIMER_CALLBACK_MEMBER(coco3fpga_state::i2c_done)
{
	// policy: every device on the codec bus acknowledges; reads return $00
	m_i2c_complete = 1;
	m_i2c_data_in = 0x00;
	if (!m_i2c_start)
		m_i2c_done = 1;
}

// 4.1 memory map, CoCo3IO.v:106-258
u8 coco3fpga_state::io_r(offs_t offset)
{
	const offs_t addr = offset;

	if (addr < 0x0200)
		return m_io_ram[addr];
	if ((addr & 0xf000) == 0x2000)
		return m_fw[0][addr & 0xfff];
	if ((addr & 0xf000) == 0x3000 || addr >= 0xf800)
		return m_fw[1][addr & 0xfff];
	if ((addr & 0xff80) == 0xf500)
		return m_acia->read(addr & 1);
	if ((addr & 0xff80) == 0xf580)
		return m_wifi_acia->read(addr & 1);
	if ((addr & 0xfe00) == 0xf600)
		return m_wrbuf[addr & 0x1ff];
	if ((addr & 0xff00) == 0xf400)
		return ioreg_r(addr);
	return 0xaa;
}

void coco3fpga_state::io_w(offs_t offset, u8 data)
{
	const offs_t addr = offset;

	if (addr < 0x0200)
		m_io_ram[addr] = data;
	else if ((addr & 0xff80) == 0xf500)
		m_acia->write(addr & 1, data);
	else if ((addr & 0xff80) == 0xf580)
		m_wifi_acia->write(addr & 1, data);
	else if ((addr & 0xfe00) == 0xf600)
	{
		m_rdbuf[addr & 0x1ff] = data;
		if (addr == 0xf603) m_track1 = data;
		if (addr == 0xf610) m_heads = data;
		if (addr == 0xf612) m_track2 = data;
	}
	else if ((addr & 0xff00) == 0xf400)
		ioreg_w(addr, data);
	// firmware blocks are read-only to the 6502
}


//**************************************************************************
//  SPI BRIDGE (section 14)
//**************************************************************************

u8 coco3fpga_state::spi_transfer(u8 data)
{
	u8 rx = 0;
	for (int i = 7; i >= 0; i--)
	{
		m_sdcard->spi_mosi_w(BIT(data, i));
		m_sdcard->spi_clock_w(1);
		rx = (rx << 1) | (m_spi_miso & 1);
		m_sdcard->spi_clock_w(0);
	}
	m_sdcard->spi_mosi_w(1);
	return rx;
}

u8 coco3fpga_state::spi_r(offs_t reg)
{
	const bool sw6 = BIT(m_switches->read(), 6);
	if (!reg)
	{
		const bool present = sw6 || m_sdcard->get_card_present();
		const bool wp = !sw6 && m_sdcard->get_card_present() && m_sdcard->get_card_readonly();
		return (m_spi_irq << 7) | (wp << 1) | present;
	}

	// reads return the receive register and start a transfer of $FF
	const u8 data = m_spi_rreg;
	if (!machine().side_effects_disabled() && m_spi_en)
	{
		m_spi_buffer = 0xff;
		m_spi_rreg = spi_transfer(m_spi_buffer);
	}
	return data;
}

void coco3fpga_state::spi_w(offs_t reg, u8 data)
{
	if (!reg)
	{
		m_spi_en = BIT(data, 7);
		m_spi_irq_en = BIT(data, 6);
		const u8 ss = BIT(data, 0) && BIT(data, 7);
		if (ss != m_spi_ss)
		{
			m_spi_ss = ss;
			m_sdcard->spi_ss_w(ss);
		}
		if (!m_spi_en)
			m_spi_rreg = 0;
		if (!m_spi_en || !m_spi_irq_en)
		{
			m_spi_irq = 0;
			update_interrupts();
		}
	}
	else
	{
		m_spi_buffer = data;
		if (m_spi_en)
			m_spi_rreg = spi_transfer(m_spi_buffer);
	}
}


//**************************************************************************
//  AUDIO (section 10.4)
//**************************************************************************

// SOUND_DTOA follows DTOA_CODE while SOUND_EN and SEL=00, and is 0 while
// SOUND_EN and SEL=11 (coco3fpga_top.v:3061-3068)
void coco3fpga_state::update_sound_dtoa()
{
	if (m_sound_en && m_sel == 0)
		m_sound_dtoa = m_dtoa_code;
	else if (m_sound_en && m_sel == 3)
		m_sound_dtoa = 0;
	update_audio();
}

void coco3fpga_state::update_audio()
{
	const u32 sound = (m_sbs << 6) | m_sound_dtoa;
	const u32 left = ((((m_orch[0] << 8) | m_orch[2]) << 1) + (sound << 10));
	const u32 right = ((((m_orch[1] << 8) | m_orch[3]) << 1) + (sound << 10));
	m_dac_l->write(left >> 2);
	m_dac_r->write(right >> 2);
}


//**************************************************************************
//  VIDEO (section 9)
//**************************************************************************

u8 coco3fpga_state::lines_row() const
{
	if (!m_coco1)
	{
		static constexpr u8 table[8] = { 0, 0, 1, 7, 8, 9, 10, 15 };
		return table[m_lpr];
	}
	switch (m_sam_v)
	{
	case 1: case 2: return 2;
	case 3: case 4: return 1;
	case 5: case 6: return 0;
	default:        return 11;
	}
}

u32 coco3fpga_state::screen_off() const
{
	u32 stride;
	if (m_hven)
		stride = 256;
	else if (m_coco1)
		stride = BIT(m_sam_v, 0) ? 16 : 32;
	else if (!m_grmode)
	{
		static constexpr u16 text[8] = { 32, 40, 64, 80, 64, 80, 128, 160 };
		stride = BIT(m_hres, 3) ? 32 : text[(BIT(m_hres, 2) << 2) | (BIT(m_cres, 0) << 1) | BIT(m_hres, 0)];
	}
	else
	{
		static constexpr u16 gfx[16] = { 16, 20, 32, 40, 64, 80, 128, 160, 256, 320, 512, 640, 32, 32, 32, 32 };
		stride = gfx[m_hres];
	}
	return (m_row_add + stride) & 0x7fffff;
}

void coco3fpga_state::reset_row()
{
	m_six_r = 0;
	m_sg_lines = 0;
	m_num_row = lines_row();
	m_underline = 0;
	m_coco1_vlpr = 0;
	if (!m_coco1)
	{
		m_row_add = (u32(m_scrn_hsb) << 16 | u32(m_scrn_msb) << 8 | m_scrn_lsb) << 3;
		if (m_grmode)
			m_vlpr = 0;
		else
			m_vlpr = (lines_row() > m_vert_fin_scrl) ? m_vert_fin_scrl : lines_row();
	}
	else
	{
		m_vlpr = 0;
		m_row_add = (u32(m_scrn_hsb) << 19) | (u32(m_scrn_msb >> 5) << 16) | (u32(m_sam_vert) << 9)
				| (u32(m_scrn_lsb & 0x3f) << 3);
	}
	m_row_add &= 0x7fffff;
}

// coco3vid.v:1942-2240, translated with nonblocking (old value) semantics
void coco3fpga_state::step_row()
{
	if (!(BIT(m_line, 0) || (m_hlpr && !BIT(m_switches->read(), 3))))
		return;

	const u8 lr = lines_row();
	const u8 num = m_num_row;
	const u8 vlpr = m_vlpr;

	m_coco1_vlpr = (m_coco1_vlpr == 11) ? 0 : m_coco1_vlpr + 1;

	auto next_row = [this, lr]() { m_row_add = screen_off(); m_num_row = lr; m_vlpr = 0; };
	auto sg_step = [this](u8 limit) {
		if (m_sg_lines == limit)
		{
			m_six_r ^= 1;
			m_sg_lines = 0;
		}
		else
			m_sg_lines = (m_sg_lines + 1) & 7;
	};

	switch (vlpr)
	{
	case 0x0:
		if (num == 0)
		{
			m_row_add = screen_off();
			sg_step(5);         // SG24
		}
		if (num == 0 || (m_grmode && lr == 15))
		{
			m_num_row = lr;
			m_vlpr = 0;
		}
		else
			m_vlpr = 1;
		m_underline = (num == 1);
		break;
	case 0x1:
		m_underline = 0;
		if (num == 1)
		{
			next_row();
			sg_step(2);         // SG12
		}
		else
			m_vlpr = 2;
		break;
	case 0x2:
		m_underline = 0;
		if (num == 2)
		{
			next_row();
			sg_step(1);         // SG8
		}
		else
			m_vlpr = 3;
		break;
	case 0x5:
		m_vlpr = 6;
		break;
	case 0x6:
		m_vlpr = 7;
		m_underline = (num == 7);
		break;
	case 0x7:
		if (num == 7) next_row(); else m_vlpr = 8;
		m_underline = (num == 8);
		break;
	case 0x8:
		if (num == 8) next_row(); else m_vlpr = 9;
		m_underline = (num == 9);
		break;
	case 0x9:
		if (num == 9) next_row(); else m_vlpr = 10;
		m_underline = (num == 10);
		break;
	case 0xa:
		m_vlpr = 11;
		m_underline = (num == 11);
		break;
	case 0xb: case 0xc: case 0xd: case 0xe: case 0xf:
		m_underline = 0;
		if (num != 15)
			next_row();
		break;
	default:
		m_vlpr = vlpr + 1;
		break;
	}
}

// coco3vid.v:2252-2322, executed at the HSYNC rising edge
void coco3fpga_state::line_advance()
{
	const u8 old_vsync = m_vsync;
	switch (m_line)
	{
	case 383: if (m_lpf == 0 || m_coco1) m_vblank = 1; break;
	case 399: if (m_lpf == 1) m_vblank = 1; break;
	case 419: if (m_lpf == 2) m_vblank = 1; break;
	case 449: m_vblank = 1; break;
	case 467: m_vborder = 0; break;
	case 475: m_vsync = 0; break;
	case 483: m_vsync = 1; break;
	case 505: m_vborder = 1; break;
	}
	if (m_line == 523)
	{
		m_line = 0;
		m_vblank = 0;
	}
	else
		m_line++;

	if (old_vsync != m_vsync)
	{
		vsync_int_edge(!m_vsync);
		// cocokey.v:193-201: SLO_RESET counts falling V_SYNC_N edges
		if (!m_vsync && m_slo_reset < 63)
			m_slo_reset++;
	}

	// hardware clock counts VSYNC edges; 61 per second increment
	if (!old_vsync && m_vsync)
	{
		if (m_click == 60)
		{
			m_click = 0;
			m_sec = (m_sec == 59) ? 0 : ((m_sec + 1) & 0x3f);
		}
		else
			m_click++;
	}
}

u16 coco3fpga_state::border_code() const
{
	if (m_coco1)
	{
		if (!BIT(m_vdg_control, 3))
			return 0x200;           // black
		if (BIT(m_vdg_control, 0))
			return m_css ? 0x0b : 0x09;
		return m_css ? 0x04 : 0x00;
	}
	return 0x10;
}

// 10-bit colour: bit 9 black, bit 8 VDAC[7:0], else PALETTE[4:0]
rgb_t coco3fpga_state::code_to_rgb(u16 code, bool dim) const
{
	if (BIT(code, 9))
		return rgb_t(0, 0, 0);
	const u16 p = BIT(code, 8) ? m_vdac[code & 0xff] : m_palette[code & 0x1f];
	u8 r = (BIT(p, 11) << 3) | (BIT(p, 8) << 2) | (BIT(p, 5) << 1) | BIT(p, 2);
	u8 g = (BIT(p, 10) << 3) | (BIT(p, 7) << 2) | (BIT(p, 4) << 1) | BIT(p, 1);
	u8 b = (BIT(p, 9) << 3) | (BIT(p, 6) << 2) | (BIT(p, 3) << 1) | BIT(p, 0);
	if (dim)
	{
		r >>= 1;
		g >>= 1;
		b >>= 1;
	}
	return rgb_t(r * 17, g * 17, b * 17);
}

// byte 'offset' of the current row: word offset plus HOR_OFFSET_BUF, wrapped
// to 128 words when HVEN (coco3vid.v:256-261)
u8 coco3fpga_state::vram_r(u32 offset) const
{
	const u32 w = ((offset >> 1) + m_hor_offset_buf) & (m_hven ? 0x7f : 0x3ff);
	const u32 phys = ((m_row_add & ~1) + (w << 1) + (offset & 1)) & 0x7fffff;
	return phys_r(phys, 0x00);
}

void coco3fpga_state::gen_active(u16 *out, int width)
{
	if (m_coco1)
	{
		// legacy (VDG-compatible) modes
		static constexpr bool inverse[4][4] = {
			{ true,  true,  false, false },
			{ false, true,  false, false },
			{ false, false, true,  true  },
			{ true,  false, true,  true  } };
		const int bytes = BIT(m_sam_v, 0) ? 16 : 32;
		const int dots_per_byte = 512 / bytes;
		const bool mode6 = BIT(m_vdg_control, 0) && m_sam_v == 0 && BIT(m_switches->read(), 5);
		const bool graphics = BIT(m_vdg_control, 3);
		const int gfx_exp = (m_vdg_control == 0x8) ? 8 : (m_vdg_control == 0xf) ? 2 : 4;

		for (int i = 0; i < bytes; i++)
		{
			const u8 c = vram_r(i);
			for (int xb = 0; xb < dots_per_byte; xb++)
			{
				u16 code;
				if (!graphics)
				{
					if (!BIT(c, 7))
					{
						u16 rom_addr;
						if (BIT(m_vdg_control, 0) && !(c & 0x60))
							rom_addr = 0x600 | ((c & 0x1f) << 4) | m_coco1_vlpr;
						else
							rom_addr = ((!BIT(c, 5)) << 10) | ((c & 0x3f) << 4) | m_coco1_vlpr;
						u8 pat = m_chargen[rom_addr & 0x7ff];
						if (inverse[m_vdg_control & 3][(c >> 5) & 3])
							pat = ~pat;
						const bool on = BIT(pat, 7 - ((xb >> 1) & 7));
						code = m_css ? (on ? 0x0e : 0x0f) : (on ? 0x0c : 0x0d);
					}
					else
					{
						const int cell = (xb >> 3) & 1;
						if (!mode6)
						{
							const bool six = (m_sam_v != 0) ? m_six_r : (m_vlpr >= 6);
							const int bitn = six ? (cell ? 0 : 1) : (cell ? 2 : 3);
							code = BIT(c, bitn) ? ((c >> 4) & 7) : 0x08;
						}
						else
						{
							const int sg6 = m_vlpr >> 2;
							if (sg6 == 3)
								code = 0x08;
							else
							{
								const int bitn = (2 - sg6) * 2 + (cell ? 0 : 1);
								code = BIT(c, bitn) ? ((m_css ? 6 : 2) + BIT(c, 6)) : 0x08;
							}
						}
					}
				}
				else
				{
					const int ppb = BIT(m_vdg_control, 0) ? 8 : 4;
					const int sub = (xb / gfx_exp) % ppb;
					if (BIT(m_vdg_control, 0))
						code = 0x08 + (m_css << 1) + BIT(c, 7 - sub);
					else
						code = (m_css << 2) + ((c >> (6 - 2 * sub)) & 3);
				}
				out[i * dots_per_byte + xb] = code;
			}
		}
		return;
	}

	if (!m_grmode)
	{
		// FPGA text modes
		const bool attr = BIT(m_cres, 0);
		const bool narrow = BIT(m_hres, 2);
		const int cw = narrow ? 8 : 16;
		const int exp = narrow ? 1 : 2;
		const int chars = width / cw;
		const u8 font_line = (m_vlpr + 3) & 0x0f;
		for (int i = 0; i < chars; i++)
		{
			const u8 ch = attr ? vram_r(2 * i) : vram_r(i);
			const u8 at = attr ? vram_r(2 * i + 1) : 0;
			u8 pat = m_chargen[((ch & 0x7f) << 4) | font_line];
			if (attr && BIT(at, 6) && m_underline)
				pat = 0xff;
			if (attr && BIT(at, 7) && m_blink)
				pat = 0x00;
			for (int x = 0; x < cw; x++)
			{
				const bool on = BIT(pat, 7 - (x / exp));
				if (attr)
					out[i * cw + x] = on ? (0x08 | ((at >> 3) & 7)) : (at & 7);
				else
					out[i * cw + x] = on ? 0x0c : 0x0d;
			}
		}
		for (int x = chars * cw; x < width; x++)
			out[x] = attr ? 0 : 0x0d;
		return;
	}

	// FPGA graphics modes
	static constexpr u8 fetch_shift[16] = { 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 5, 5, 5, 5 };
	const int k = fetch_shift[m_hres];
	const int nbytes = std::max(1, (width * 2) >> k);
	const int bpp = 1 << m_cres;
	const int ppb = 8 >> m_cres;

	int exp;
	switch (((m_hres >> 1) << 2) | m_cres)
	{
	case 0x01: exp = 8; break;  // 16/20 bytes, 4 colour
	case 0x04: exp = 2; break;  // 32/40 bytes, 2 colour
	case 0x06: exp = 8; break;  // 32/40 bytes, 16 colour
	case 0x08: exp = 1; break;  // 64/80 bytes, 2 colour
	case 0x09: exp = 2; break;  // 64/80 bytes, 4 colour
	case 0x0b: exp = 8; break;  // 64/80 bytes, 256 colour
	case 0x0d: exp = 1; break;  // 128/160 bytes, 4 colour
	case 0x0e: exp = 2; break;  // 128/160 bytes, 16 colour
	case 0x12: exp = 1; break;  // 256/320 bytes, 16 colour
	case 0x13: exp = 2; break;  // 256/320 bytes, 256 colour
	case 0x17: exp = 1; break;  // 512/640 bytes, 256 colour
	default:   exp = 4; break;  // HDL default expansion
	}

	for (int x = 0; x < width; x++)
	{
		const int p = x / exp;
		const int byte = (p / ppb) % nbytes;
		const int sub = p % ppb;
		const u8 v = vram_r(byte);
		if (m_cres == 3)
			out[x] = 0x100 | v;
		else
			out[x] = (v >> (8 - bpp * (sub + 1))) & ((1 << bpp) - 1);
	}
}

// HBORDER is set from count 800 (512-wide only; 640-wide modes skip it) and
// from count 16, and cleared at 655.  So 512-wide lines get an 80-dot left
// and 64-dot right border, 640-wide active lines get none, and in vertical
// border lines the border also covers the active width (section 9.1).
void coco3fpga_state::render_line(int line)
{
	const int y = (line + VIS_TOP_LINES) % FRAME_LINES;
	if (y >= VIS_HEIGHT)
		return;

	u32 *const dest = &m_bitmap.pix(y);
	const bool dim = BIT(line, 0) && BIT(m_switches->read(), 3);   // H_FLAG lines
	const rgb_t black(0, 0, 0);
	const bool w512 = m_line_mode_256;
	const int width = w512 ? 512 : 640;

	std::fill_n(dest, VIS_WIDTH, u32(black));
	if (!m_vborder)
		return;

	const rgb_t border = code_to_rgb(border_code(), dim);
	if (w512)
	{
		std::fill_n(dest, ACTIVE_X, u32(border));
		std::fill_n(dest + ACTIVE_X + 512, 64, u32(border));
	}

	if (m_vblank)
	{
		std::fill_n(dest + ACTIVE_X, width, u32(border));
		return;
	}

	u16 codes[640];
	gen_active(codes, width);
	for (int x = 0; x < width; x++)
		dest[ACTIVE_X + x] = code_to_rgb(codes[x], dim);
}

TIMER_CALLBACK_MEMBER(coco3fpga_state::line_event)
{
	switch (param)
	{
	case EV_LINE_START:
		m_line_start = machine().time();
		m_line_mode_256 = mode_256();
		update_key();
		update_interrupts();
		m_line_timer->adjust(attotime::from_ticks(flag_dot(), PIXEL_CLOCK), EV_FLAG);
		break;

	case EV_FLAG:
	{
		// count 657: SYNC_FLAG = !LINE[0]; latch the horizontal offset on odd lines
		const u8 old = m_hflag;
		m_hflag = !BIT(m_line, 0);
		if (BIT(m_line, 0))
			m_hor_offset_buf = m_hor_offset;
		// timer clock (!H_SYNC_N | !H_FLAG) falls as H_FLAG rises
		if (!m_timer_ins && !old && m_hflag)
			timer_tick(1);
		update_interrupts();
		m_line_timer->adjust(attotime::from_ticks(hsync_fall_dot() - flag_dot(), PIXEL_CLOCK), EV_HSYNC_FALL);
		break;
	}

	case EV_HSYNC_FALL:
		render_line(m_line);

		// row sequencer runs on the HSYNC falling edge
		if (m_vblank)
			reset_row();
		else
			step_row();

		m_hsync = 0;
		if (m_hflag)
			hsync_int_edge(true);
		update_interrupts();
		m_line_timer->adjust(attotime::from_ticks(hsync_rise_dot() - hsync_fall_dot(), PIXEL_CLOCK), EV_HSYNC_RISE);
		break;

	case EV_HSYNC_RISE:
	{
		m_hsync = 1;
		if (m_hflag)
		{
			hsync_int_edge(false);
			if (!m_timer_ins)
				timer_tick(1);          // timer clock falls as HSYNC ends
		}

		// line counter runs on the HSYNC rising edge
		line_advance();
		if (m_vblank)
			reset_row();

		update_interrupts();
		const attotime next = m_line_start + attotime::from_ticks(LINE_DOTS, PIXEL_CLOCK);
		const attotime now = machine().time();
		m_line_timer->adjust(next > now ? next - now : attotime::zero, EV_LINE_START);
		break;
	}
	}
}

u32 coco3fpga_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	copybitmap(bitmap, m_bitmap, 0, 0, 0, 0, cliprect);
	return 0;
}


//**************************************************************************
//  CLOCKS / RESET
//**************************************************************************

void coco3fpga_state::update_cpu_clock()
{
	u32 clock;
	if (!m_rate)
		clock = 50'000'000 / 56;
	else if (!BIT(m_switches->read(), 0))
		clock = 50'000'000 / 28;
	else
		clock = 50'000'000 / 2;
	m_maincpu->set_unscaled_clock(clock);
}

void coco3fpga_state::update_acia_clocks()
{
	// eight pulses per 27 master clocks, then /8, /4, /2 or /1
	static constexpr int div[4] = { 8, 4, 2, 1 };
	const int sel = (m_switches->read() >> 7) & 3;
	m_acia_clock->set_unscaled_clock(u32(50'000'000.0 * 8.0 / 27.0 / div[sel]));
	m_wifi_clock->set_unscaled_clock(u32(50'000'000.0 * 8.0 / 27.0 / div[m_wf_baud & 3]));
}

INPUT_CHANGED_MEMBER(coco3fpga_state::reset_button)
{
	if (!newval)
		machine().schedule_soft_reset();
}

INPUT_CHANGED_MEMBER(coco3fpga_state::switches_changed)
{
	update_cpu_clock();
	update_acia_clocks();
	update_interrupts();
}

TIMER_CALLBACK_MEMBER(coco3fpga_state::cpu_reset_release)
{
	m_cpu_reset = 0;
	m_maincpu->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
	update_slave_reset();
	update_halt();
}

void coco3fpga_state::periph_reset()
{
	std::fill(std::begin(m_dd_reg), std::end(m_dd_reg), 0);
	std::fill(std::begin(m_ddr), std::end(m_ddr), 0);
	m_key_column = 0;
	m_dtoa_code = m_sound_dtoa = 0;
	m_sbs = m_css = m_vdg_control = m_sel = 0;
	m_hsync_int = m_hsync_pol = m_vsync_int = m_vsync_pol = 0;
	m_cas_mtr = 0;
	m_cart_int = m_cart_pol = m_sound_en = 0;
	m_pdl = 0;

	std::fill(std::begin(m_orch), std::end(m_orch), 0x80);
	std::fill(std::begin(m_orch_ext_buf), std::end(m_orch_ext_buf), 0x80);
	m_gart_write = m_gart_read = 0;
	m_gart_inc = 0;
	m_gart_cnt = 0;
	m_w_prot = 3;
	m_mpi_scs = m_mpi_cts = (m_switches->read() >> 1) & 3;
	m_rom_bank = 0;
	m_wf_irq_en = m_wf_baud = 0;
	m_slave_reset = m_slave_add_hi = m_slave_add_lo = 0;
	m_gpio_dir = m_gpio_out = 0;
	m_ck_start = 0;
	m_ck_data_out = m_ck_device = m_ck_reg = 0;
	m_ck_done = 1;
	m_ck_fail = 0;
	m_ck_complete = 0;
	m_rtc_timer->adjust(attotime::never);
	m_sdram_read = 0;
	m_sdram_din = 0;
	m_sdram_addr = 0;

	m_rom_mode = m_st_scs = m_vec_pag_ram = m_gime_firq = m_gime_irq = m_mmu_en = m_coco1 = 0;
	m_timer_ins = m_mmu_tr = 0;
	m_irq_en = m_firq_en = 0;
	m_tmr_msb = m_tmr_lsb = m_tmr_enable = 0;
	m_grmode = m_hres = m_descen = m_mono = m_lpr = 0;
	m_hlpr = m_lpf = m_cres = 0;
	m_scrn_hsb = m_scrn_msb = m_scrn_lsb = m_vert_fin_scrl = m_hven = m_hor_offset = 0;
	m_sam_ext = 0;
	std::fill(std::begin(m_mmu), std::end(m_mmu), 0);
	for (int i = 0; i < 16; i++)
		m_palette[i] = 0;
	m_sam_v = m_sam_vert = m_rate = m_ram_mode = 0;

	m_hs_stat = m_vs_stat = m_cart1_stat = 0;
	std::fill(std::begin(m_irq_stat), std::end(m_irq_stat), 0);
	std::fill(std::begin(m_firq_stat), std::end(m_firq_stat), 0);
	m_cart_level = 1;
	m_key_level = 1;

	m_timer = 0x1fff;
	m_blink = 1;
	m_timer_last = machine().time();

	m_line = 0;
	m_vblank = 0;
	m_vsync = 1;

	// SPI
	m_spi_en = m_spi_irq_en = 0;
	m_spi_buffer = 0xff;
	m_spi_rreg = 0;
	m_spi_irq = 0;
	if (m_spi_ss)
	{
		m_spi_ss = 0;
		m_sdcard->spi_ss_w(0);
	}

	// disk interface
	m_drive_sel_ext = m_motor = m_wrt_prec = m_density = m_halt_en = 0;
	m_command = m_track_reg_w = m_track_ext_w = m_sector = m_data_reg = m_data_ext = 0;
	m_irq_09_buf2 = 0;
	m_buff_add = 0;
	m_track_reg_r = m_track_ext_r = m_status = m_trace = 0;
	m_track1 = m_track2 = m_heads = 0;
	m_ctrl = 0;
	m_irq_02 = 0;
	m_halt_state6 = 0;
	m_halt_code = 0;
	m_halt_state_timer->adjust(attotime::never);
	m_rdfifo_head = m_rdfifo_count = 0;
	m_wrfifo_head = m_wrfifo_count = 0;
	m_wf_rdfifo_head = m_wf_rdfifo_count = 0;
	m_wf_wrfifo_head = m_wf_wrfifo_count = 0;
	m_i2c_done = 1;
	m_i2c_fail = 0;
	m_i2c_start = 0;
	m_i2c_complete = 0;
	m_i2c_timer->adjust(attotime::never);
	m_dma_timer->adjust(attotime::never);
}

void coco3fpga_state::machine_start()
{
	m_ram = std::make_unique<u8[]>(4 * CHIP_SIZE);
	std::fill_n(m_ram.get(), 4 * CHIP_SIZE, 0);
	m_sdram = std::make_unique<u8[]>(0x800000);
	std::fill_n(m_sdram.get(), 0x800000, 0);
	std::fill(std::begin(m_chip_mask), std::end(m_chip_mask), 0);

	// FPGA memory initialisers: loaded at power on only
	std::copy_n(&m_io_ram_init[0], 0x200, m_io_ram);
	std::copy_n(&m_fw_init[0], 0x1000, m_fw[0]);
	std::copy_n(&m_fw_init[0], 0x1000, m_fw[1]);
	for (int i = 0; i < 256; i++)
		m_vdac[i] = (m_vdac_init[i * 2] | (m_vdac_init[i * 2 + 1] << 8)) & 0xfff;

	std::fill(std::begin(m_rdbuf), std::end(m_rdbuf), 0);
	std::fill(std::begin(m_wrbuf), std::end(m_wrbuf), 0);
	std::fill(std::begin(m_rdfifo), std::end(m_rdfifo), 0);
	std::fill(std::begin(m_wrfifo), std::end(m_wrfifo), 0);
	std::fill(std::begin(m_wf_rdfifo), std::end(m_wf_rdfifo), 0);
	std::fill(std::begin(m_wf_wrfifo), std::end(m_wf_wrfifo), 0);
	m_rdfifo_q = m_wrfifo_q = m_wf_rdfifo_q = m_wf_wrfifo_q = 0;

	// power-on policies for state the HDL does not reset
	m_palette[16] = 0;
	for (int i = 0; i < 8; i++)
		m_bank[i] = i;
	m_bank_size = 0;
	m_cent = m_year = m_mnth = m_dmth = m_dwk = m_hour = m_min = m_sec = m_click = 0;
	m_bi_irq_en = 0;
	m_i2c_device = m_i2c_reg = m_i2c_data_in = m_i2c_data_out = 0;
	m_ck_data_in = 0;
	std::fill(std::begin(m_rtc_regs), std::end(m_rtc_regs), 0);
	m_rtc_regs[0x11] = 0x19;        // 25 degrees C
	m_rtc_offset = 0;
	m_gart_buf = 0;
	m_sdram_dout = 0;
	m_spi_ss = 0;
	m_spi_miso = 1;
	m_ser_irq = 0;
	m_halt_sig = 0;
	m_hsync = 1;
	m_hflag = 0;
	m_vborder = 1;
	m_hor_offset_buf = 0;
	m_vlpr = m_num_row = m_coco1_vlpr = m_underline = m_six_r = m_sg_lines = 0;
	m_row_add = 0;
	m_line_mode_256 = 1;
	m_mugs = 0;
	m_reset_ins = 1;
	m_ps2_clk = m_ps2_data = 1;
	m_ps2_state = 0;
	m_ps2_shift = m_ps2_released = m_ps2_extended = 0;
	m_ps2_frame_start = attotime::zero;
	m_slo_reset = 0;
	cocokey_reset();
	m_cpu_reset = 1;

	m_bitmap.allocate(VIS_WIDTH, VIS_HEIGHT);

	m_line_timer = timer_alloc(FUNC(coco3fpga_state::line_event), this);
	m_gime_timer = timer_alloc(FUNC(coco3fpga_state::gime_timer_expired), this);
	m_reset_timer = timer_alloc(FUNC(coco3fpga_state::cpu_reset_release), this);
	m_halt_state_timer = timer_alloc(FUNC(coco3fpga_state::halt_state_expired), this);
	m_i2c_timer = timer_alloc(FUNC(coco3fpga_state::i2c_done), this);
	m_rtc_timer = timer_alloc(FUNC(coco3fpga_state::rtc_i2c_done), this);
	m_dma_timer = timer_alloc(FUNC(coco3fpga_state::dma_step), this);

	save_pointer(NAME(m_ram), 4 * CHIP_SIZE);
	save_pointer(NAME(m_sdram), 0x800000);
	save_item(NAME(m_chip_mask));
	save_item(NAME(m_io_ram));
	save_item(NAME(m_fw));
	save_item(NAME(m_vdac));
	save_item(NAME(m_rdbuf));
	save_item(NAME(m_wrbuf));
	save_item(NAME(m_rdfifo));
	save_item(NAME(m_rdfifo_head));
	save_item(NAME(m_rdfifo_count));
	save_item(NAME(m_rdfifo_q));
	save_item(NAME(m_wrfifo));
	save_item(NAME(m_wrfifo_head));
	save_item(NAME(m_wrfifo_count));
	save_item(NAME(m_wrfifo_q));
	save_item(NAME(m_wf_rdfifo));
	save_item(NAME(m_wf_rdfifo_head));
	save_item(NAME(m_wf_rdfifo_count));
	save_item(NAME(m_wf_rdfifo_q));
	save_item(NAME(m_wf_wrfifo));
	save_item(NAME(m_wf_wrfifo_head));
	save_item(NAME(m_wf_wrfifo_count));
	save_item(NAME(m_wf_wrfifo_q));
	save_item(NAME(m_dd_reg));
	save_item(NAME(m_ddr));
	save_item(NAME(m_key_column));
	save_item(NAME(m_dtoa_code));
	save_item(NAME(m_sound_dtoa));
	save_item(NAME(m_sbs));
	save_item(NAME(m_css));
	save_item(NAME(m_vdg_control));
	save_item(NAME(m_sel));
	save_item(NAME(m_hsync_int));
	save_item(NAME(m_hsync_pol));
	save_item(NAME(m_vsync_int));
	save_item(NAME(m_vsync_pol));
	save_item(NAME(m_cas_mtr));
	save_item(NAME(m_cart_int));
	save_item(NAME(m_cart_pol));
	save_item(NAME(m_sound_en));
	save_item(NAME(m_pdl));
	save_item(NAME(m_orch));
	save_item(NAME(m_orch_ext_buf));
	save_item(NAME(m_gart_write));
	save_item(NAME(m_gart_read));
	save_item(NAME(m_gart_inc));
	save_item(NAME(m_gart_cnt));
	save_item(NAME(m_gart_buf));
	save_item(NAME(m_w_prot));
	save_item(NAME(m_mpi_scs));
	save_item(NAME(m_mpi_cts));
	save_item(NAME(m_rom_bank));
	save_item(NAME(m_bank));
	save_item(NAME(m_bank_size));
	save_item(NAME(m_wf_irq_en));
	save_item(NAME(m_wf_baud));
	save_item(NAME(m_slave_reset));
	save_item(NAME(m_slave_add_hi));
	save_item(NAME(m_slave_add_lo));
	save_item(NAME(m_gpio_dir));
	save_item(NAME(m_gpio_out));
	save_item(NAME(m_sdram_read));
	save_item(NAME(m_sdram_din));
	save_item(NAME(m_sdram_dout));
	save_item(NAME(m_sdram_addr));
	save_item(NAME(m_ck_start));
	save_item(NAME(m_ck_done));
	save_item(NAME(m_ck_fail));
	save_item(NAME(m_ck_complete));
	save_item(NAME(m_ck_data_in));
	save_item(NAME(m_ck_data_out));
	save_item(NAME(m_ck_device));
	save_item(NAME(m_ck_reg));
	save_item(NAME(m_rtc_regs));
	save_item(NAME(m_rtc_offset));
	save_item(NAME(m_rom_mode));
	save_item(NAME(m_st_scs));
	save_item(NAME(m_vec_pag_ram));
	save_item(NAME(m_gime_firq));
	save_item(NAME(m_gime_irq));
	save_item(NAME(m_mmu_en));
	save_item(NAME(m_coco1));
	save_item(NAME(m_timer_ins));
	save_item(NAME(m_mmu_tr));
	save_item(NAME(m_irq_en));
	save_item(NAME(m_firq_en));
	save_item(NAME(m_tmr_msb));
	save_item(NAME(m_tmr_lsb));
	save_item(NAME(m_tmr_enable));
	save_item(NAME(m_grmode));
	save_item(NAME(m_hres));
	save_item(NAME(m_descen));
	save_item(NAME(m_mono));
	save_item(NAME(m_lpr));
	save_item(NAME(m_hlpr));
	save_item(NAME(m_lpf));
	save_item(NAME(m_cres));
	save_item(NAME(m_palette));
	save_item(NAME(m_scrn_hsb));
	save_item(NAME(m_scrn_msb));
	save_item(NAME(m_scrn_lsb));
	save_item(NAME(m_vert_fin_scrl));
	save_item(NAME(m_hven));
	save_item(NAME(m_hor_offset));
	save_item(NAME(m_hor_offset_buf));
	save_item(NAME(m_sam_ext));
	save_item(NAME(m_mmu));
	save_item(NAME(m_sam_v));
	save_item(NAME(m_sam_vert));
	save_item(NAME(m_rate));
	save_item(NAME(m_ram_mode));
	save_item(NAME(m_hs_stat));
	save_item(NAME(m_vs_stat));
	save_item(NAME(m_cart1_stat));
	save_item(NAME(m_irq_stat));
	save_item(NAME(m_firq_stat));
	save_item(NAME(m_cart_level));
	save_item(NAME(m_key_level));
	save_item(NAME(m_timer));
	save_item(NAME(m_blink));
	save_item(NAME(m_timer_last));
	save_item(NAME(m_line_start));
	save_item(NAME(m_line_mode_256));
	save_item(NAME(m_line));
	save_item(NAME(m_hsync));
	save_item(NAME(m_hflag));
	save_item(NAME(m_vsync));
	save_item(NAME(m_vblank));
	save_item(NAME(m_vborder));
	save_item(NAME(m_vlpr));
	save_item(NAME(m_num_row));
	save_item(NAME(m_coco1_vlpr));
	save_item(NAME(m_underline));
	save_item(NAME(m_six_r));
	save_item(NAME(m_sg_lines));
	save_item(NAME(m_row_add));
	save_item(NAME(m_cent));
	save_item(NAME(m_year));
	save_item(NAME(m_mnth));
	save_item(NAME(m_dmth));
	save_item(NAME(m_dwk));
	save_item(NAME(m_hour));
	save_item(NAME(m_min));
	save_item(NAME(m_sec));
	save_item(NAME(m_click));
	save_item(NAME(m_spi_en));
	save_item(NAME(m_spi_irq_en));
	save_item(NAME(m_spi_ss));
	save_item(NAME(m_spi_rreg));
	save_item(NAME(m_spi_buffer));
	save_item(NAME(m_spi_irq));
	save_item(NAME(m_spi_miso));
	save_item(NAME(m_ser_irq));
	save_item(NAME(m_drive_sel_ext));
	save_item(NAME(m_motor));
	save_item(NAME(m_wrt_prec));
	save_item(NAME(m_density));
	save_item(NAME(m_halt_en));
	save_item(NAME(m_command));
	save_item(NAME(m_track_reg_w));
	save_item(NAME(m_track_ext_w));
	save_item(NAME(m_sector));
	save_item(NAME(m_data_reg));
	save_item(NAME(m_data_ext));
	save_item(NAME(m_bi_irq_en));
	save_item(NAME(m_irq_09_buf2));
	save_item(NAME(m_buff_add));
	save_item(NAME(m_track_reg_r));
	save_item(NAME(m_track_ext_r));
	save_item(NAME(m_status));
	save_item(NAME(m_trace));
	save_item(NAME(m_track1));
	save_item(NAME(m_track2));
	save_item(NAME(m_heads));
	save_item(NAME(m_ctrl));
	save_item(NAME(m_irq_02));
	save_item(NAME(m_halt_state6));
	save_item(NAME(m_halt_code));
	save_item(NAME(m_halt_sig));
	save_item(NAME(m_i2c_device));
	save_item(NAME(m_i2c_reg));
	save_item(NAME(m_i2c_data_in));
	save_item(NAME(m_i2c_data_out));
	save_item(NAME(m_i2c_start));
	save_item(NAME(m_i2c_done));
	save_item(NAME(m_i2c_fail));
	save_item(NAME(m_i2c_complete));
	save_item(NAME(m_cpu_reset));
	save_item(NAME(m_mugs));
	save_item(NAME(m_reset_ins));
	save_item(NAME(m_ps2_clk));
	save_item(NAME(m_ps2_data));
	save_item(NAME(m_ps2_state));
	save_item(NAME(m_ps2_shift));
	save_item(NAME(m_ps2_released));
	save_item(NAME(m_ps2_extended));
	save_item(NAME(m_ps2_frame_start));
	save_item(NAME(m_key));
	save_item(NAME(m_kb_shift));
	save_item(NAME(m_kb_shift_override));
	save_item(NAME(m_kb_reset));
	save_item(NAME(m_shift_arrow_l));
	save_item(NAME(m_shift_arrow_r));
	save_item(NAME(m_slo_reset));
}

void coco3fpga_state::machine_reset()
{
	// chip positions (section 4.1)
	const u8 cfg = m_config->read();
	const u32 de1 = BIT(cfg, 1) ? 0x0fffff : 0x07ffff;
	std::fill(std::begin(m_chip_mask), std::end(m_chip_mask), 0);
	if (BIT(cfg, 0))
	{
		// M5Meg: two 2 MB analog-board SRAMs at 00/01, DE1 SRAM at 10
		m_chip_mask[0] = CHIP_SIZE - 1;
		m_chip_mask[1] = CHIP_SIZE - 1;
		m_chip_mask[2] = de1;
	}
	else
		m_chip_mask[0] = de1;   // M1Meg: DE1 SRAM only

	periph_reset();
	update_cpu_clock();
	update_acia_clocks();
	update_audio();

	// MUGS follows ~RESET_INS while reset is asserted; RESET_INS itself is
	// never reset, so the egg persists until a Ctrl-Alt-Del (section 3)
	m_mugs = !m_reset_ins;

	// cocokey is held in reset until 63 VSYNC edges after RESET_N
	m_slo_reset = 0;
	cocokey_reset();
	m_ps2_state = 0;

	// CPU reset is held for ~10239 ticks of 50 MHz / 1024 (~209.7 ms)
	m_cpu_reset = 1;
	m_maincpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
	update_slave_reset();
	m_reset_timer->adjust(attotime::from_ticks(10239 * 1024, 50'000'000));

	m_line_start = machine().time();
	m_line_mode_256 = mode_256();
	m_hsync = 1;
	m_hflag = 0;
	reset_row();
	m_line_timer->adjust(attotime::zero, EV_LINE_START);
	m_gime_timer->adjust(attotime::never);

	update_io_irq();
	update_nmi();
	update_halt();
	update_interrupts();
}


//**************************************************************************
//  ADDRESS MAPS
//**************************************************************************

void coco3fpga_state::main_map(address_map &map)
{
	map(0x0000, 0xffff).rw(FUNC(coco3fpga_state::main_r), FUNC(coco3fpga_state::main_w));
}

void coco3fpga_state::io_map(address_map &map)
{
	map(0x0000, 0xffff).rw(FUNC(coco3fpga_state::io_r), FUNC(coco3fpga_state::io_w));
}


//**************************************************************************
//  INPUT PORTS
//**************************************************************************

static INPUT_PORTS_START( coco3fpga )
	// 12-bit joystick latches: 0 right X, 1 right Y, 2 left X, 3 left Y.
	// Explicit codes: MAME's default player key bindings (arrows, Ctrl, Alt,
	// A, S, D, F, G, R) collide with the CoCo keyboard matrix.
	PORT_START("joy0")
	PORT_BIT(0xfff, 0x800, IPT_AD_STICK_X) PORT_NAME("Right Joystick X") PORT_MINMAX(0, 0xfff) PORT_SENSITIVITY(100) PORT_KEYDELTA(64)
		PORT_CODE_DEC(KEYCODE_4_PAD) PORT_CODE_INC(KEYCODE_6_PAD) PORT_CODE_DEC(JOYCODE_X_LEFT_SWITCH) PORT_CODE_INC(JOYCODE_X_RIGHT_SWITCH) PORT_PLAYER(1)
	PORT_START("joy1")
	PORT_BIT(0xfff, 0x800, IPT_AD_STICK_Y) PORT_NAME("Right Joystick Y") PORT_MINMAX(0, 0xfff) PORT_SENSITIVITY(100) PORT_KEYDELTA(64)
		PORT_CODE_DEC(KEYCODE_8_PAD) PORT_CODE_INC(KEYCODE_2_PAD) PORT_CODE_DEC(JOYCODE_Y_UP_SWITCH) PORT_CODE_INC(JOYCODE_Y_DOWN_SWITCH) PORT_PLAYER(1)
	PORT_START("joy2")
	PORT_BIT(0xfff, 0x800, IPT_AD_STICK_X) PORT_NAME("Left Joystick X") PORT_MINMAX(0, 0xfff) PORT_SENSITIVITY(100) PORT_KEYDELTA(64)
		PORT_CODE_DEC(KEYCODE_4_PAD) PORT_CODE_INC(KEYCODE_6_PAD) PORT_CODE_DEC(JOYCODE_X_LEFT_SWITCH) PORT_CODE_INC(JOYCODE_X_RIGHT_SWITCH) PORT_PLAYER(2)
	PORT_START("joy3")
	PORT_BIT(0xfff, 0x800, IPT_AD_STICK_Y) PORT_NAME("Left Joystick Y") PORT_MINMAX(0, 0xfff) PORT_SENSITIVITY(100) PORT_KEYDELTA(64)
		PORT_CODE_DEC(KEYCODE_8_PAD) PORT_CODE_INC(KEYCODE_2_PAD) PORT_CODE_DEC(JOYCODE_Y_UP_SWITCH) PORT_CODE_INC(JOYCODE_Y_DOWN_SWITCH) PORT_PLAYER(2)

	PORT_START("joy_buttons")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_BUTTON1) PORT_NAME("Right Button 1") PORT_CODE(KEYCODE_0_PAD) PORT_CODE(JOYCODE_BUTTON1) PORT_PLAYER(1)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_BUTTON1) PORT_NAME("Left Button 1") PORT_CODE(KEYCODE_0_PAD) PORT_CODE(JOYCODE_BUTTON1) PORT_PLAYER(2)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_BUTTON2) PORT_NAME("Left Button 2") PORT_CODE(KEYCODE_DEL_PAD) PORT_CODE(JOYCODE_BUTTON2) PORT_PLAYER(2)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_BUTTON2) PORT_NAME("Right Button 2") PORT_CODE(KEYCODE_DEL_PAD) PORT_CODE(JOYCODE_BUTTON2) PORT_PLAYER(1)

	// DE1 push buttons (active low); SD detect/write protect follow the card image
	PORT_START("board")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("Easter Egg (ALT+CTRL)")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("Reset") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(coco3fpga_state::reset_button), 0)

	PORT_START("switches")
	PORT_DIPNAME(0x001, 0x000, "SW0: Turbo (with RATE=1)") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(coco3fpga_state::switches_changed), 0)
	PORT_DIPSETTING(0x000, DEF_STR(Off))
	PORT_DIPSETTING(0x001, DEF_STR(On))
	PORT_DIPNAME(0x006, 0x006, "SW2-1: Initial MPI slot")
	PORT_DIPSETTING(0x000, "1 (Orchestra-90)")
	PORT_DIPSETTING(0x002, "2 (Alternate disk ROM)")
	PORT_DIPSETTING(0x004, "3 (Cartridge flash)")
	PORT_DIPSETTING(0x006, "4 (Disk)")
	PORT_DIPNAME(0x008, 0x000, "SW3: Dim alternate scanlines")
	PORT_DIPSETTING(0x000, DEF_STR(Off))
	PORT_DIPSETTING(0x008, DEF_STR(On))
	PORT_DIPNAME(0x010, 0x000, "SW4: Disable autostart in slots 1 and 3") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(coco3fpga_state::switches_changed), 0)
	PORT_DIPSETTING(0x000, DEF_STR(Off))
	PORT_DIPSETTING(0x010, DEF_STR(On))
	PORT_DIPNAME(0x020, 0x000, "SW5: SG6 semigraphics")
	PORT_DIPSETTING(0x000, DEF_STR(Off))
	PORT_DIPSETTING(0x020, DEF_STR(On))
	PORT_DIPNAME(0x040, 0x000, "SW6: Ignore SD detect/write protect")
	PORT_DIPSETTING(0x000, DEF_STR(Off))
	PORT_DIPSETTING(0x040, DEF_STR(On))
	PORT_DIPNAME(0x180, 0x000, "SW8-7: DriveWire baud") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(coco3fpga_state::switches_changed), 0)
	PORT_DIPSETTING(0x000, "115200")
	PORT_DIPSETTING(0x080, "230400")
	PORT_DIPSETTING(0x100, "460800")
	PORT_DIPSETTING(0x180, "921600")
	PORT_DIPNAME(0x200, 0x000, "SW9: Swap DriveWire/RS-232 connectors")
	PORT_DIPSETTING(0x000, DEF_STR(Off))
	PORT_DIPSETTING(0x200, DEF_STR(On))

	PORT_START("config")
	PORT_CONFNAME(0x01, 0x01, "Memory build")
	PORT_CONFSETTING(0x00, "M1Meg (DE1 SRAM only)")
	PORT_CONFSETTING(0x01, "M5Meg (DE1 SRAM + analog board 4 MB)")
	PORT_CONFNAME(0x02, 0x00, "DE1 SRAM")
	PORT_CONFSETTING(0x00, "512 KB (stock)")
	PORT_CONFSETTING(0x02, "1 MB (replacement chip)")
INPUT_PORTS_END


//**************************************************************************
//  MACHINE CONFIGURATION
//**************************************************************************

void coco3fpga_state::coco3fpga(machine_config &config)
{
	MC6809E(config, m_maincpu, 50'000'000 / 56);
	m_maincpu->set_addrmap(AS_PROGRAM, &coco3fpga_state::main_map);

	M6502(config, m_iocpu, 50'000'000 / 8);
	m_iocpu->set_addrmap(AS_PROGRAM, &coco3fpga_state::io_map);

	config.set_perfect_quantum(m_maincpu);

	AMD_29F032(config, m_flash);

	// PS/2 keyboard; the FPGA only receives, so the host lines stay released
	PC_KBDC(config, m_kbdc, pc_at_keyboards, STR_KBD_MICROSOFT_NATURAL);
	m_kbdc->out_clock_cb().set(FUNC(coco3fpga_state::ps2_clock_w));
	m_kbdc->out_data_cb().set(FUNC(coco3fpga_state::ps2_data_w));

	SCREEN(config, m_screen);
	m_screen->set_raw(PIXEL_CLOCK, LINE_DOTS, 0, VIS_WIDTH, FRAME_LINES, 0, VIS_HEIGHT);
	m_screen->set_screen_update(FUNC(coco3fpga_state::screen_update));

	// DriveWire 6850 on the I/O processor ($F500)
	ACIA6850(config, m_acia);
	m_acia->txd_handler().set("dwire", FUNC(rs232_port_device::write_txd));
	m_acia->rts_handler().set(m_acia, FUNC(acia6850_device::write_cts));
	m_acia->rts_handler().append(m_acia, FUNC(acia6850_device::write_dcd));

	CLOCK(config, m_acia_clock, 1'851'852);     // 50 MHz * 8 / 27 / 8
	m_acia_clock->signal_handler().set(m_acia, FUNC(acia6850_device::write_txc));
	m_acia_clock->signal_handler().append(m_acia, FUNC(acia6850_device::write_rxc));

	rs232_port_device &dwire(RS232_PORT(config, "dwire", default_rs232_devices, nullptr));
	dwire.rxd_handler().set(m_acia, FUNC(acia6850_device::write_rxd));

	// WiFi (ESP8266) 6850 on the I/O processor ($F580); IRQ not connected
	ACIA6850(config, m_wifi_acia);
	m_wifi_acia->txd_handler().set("wifi", FUNC(rs232_port_device::write_txd));
	m_wifi_acia->rts_handler().set(m_wifi_acia, FUNC(acia6850_device::write_cts));
	m_wifi_acia->rts_handler().append(m_wifi_acia, FUNC(acia6850_device::write_dcd));

	CLOCK(config, m_wifi_clock, 1'851'852);
	m_wifi_clock->signal_handler().set(m_wifi_acia, FUNC(acia6850_device::write_txc));
	m_wifi_clock->signal_handler().append(m_wifi_acia, FUNC(acia6850_device::write_rxc));

	rs232_port_device &wifi(RS232_PORT(config, "wifi", default_rs232_devices, nullptr));
	wifi.rxd_handler().set(m_wifi_acia, FUNC(acia6850_device::write_rxd));

	// RS-232 pak 6551 at $FF68
	MOS6551(config, m_uart, 0);
	m_uart->set_xtal(1'851'852);                // 14.815 MHz / 8
	m_uart->irq_handler().set([this] (int state) { m_ser_irq = state; update_interrupts(); });
	m_uart->txd_handler().set("rs232", FUNC(rs232_port_device::write_txd));
	m_uart->rts_handler().set(m_uart, FUNC(mos6551_device::write_cts));
	m_uart->dtr_handler().set(m_uart, FUNC(mos6551_device::write_dcd));
	m_uart->dtr_handler().append(m_uart, FUNC(mos6551_device::write_dsr));

	rs232_port_device &rs232(RS232_PORT(config, "rs232", default_rs232_devices, nullptr));
	rs232.rxd_handler().set(m_uart, FUNC(mos6551_device::write_rxd));

	SPI_SDCARD(config, m_sdcard, 0);
	m_sdcard->spi_miso_callback().set(FUNC(coco3fpga_state::spi_miso_w));

	SPEAKER(config, "lspeaker").front_left();
	SPEAKER(config, "rspeaker").front_right();
	DAC_16BIT_R2R(config, m_dac_l);
	DAC_16BIT_R2R(config, m_dac_r);
	m_dac_l->add_route(0, "lspeaker", 0.5);
	m_dac_r->add_route(0, "rspeaker", 0.5);
}


//**************************************************************************
//  ROMS
//**************************************************************************

ROM_START( coco3fpga )
	// 4 MB DE1 flash, writable by software (section 4.3); kept as NVRAM.
	// Contents per the 4.1 user guide: unmodified stock ROMs.
	ROM_REGION( 0x400000, "flash", ROMREGION_ERASEFF )
	ROM_LOAD( "coco3.rom",   0x00000, 0x8000, CRC(b4c88d6c) SHA1(e0d82953fb6fd03768604933df1ce8bc51fc427d) )   // CoCo 3 BASIC
	ROM_LOAD( "disk11.rom",  0x08000, 0x2000, CRC(0b9c5415) SHA1(10bdc5aa2d7d7f205f67b47b19003a4bd89defd1) )   // Disk BASIC 1.1, slot 4
	ROM_LOAD( "orchestra 90,1984,26 - 3143,tandy.rom", 0x0a000, 0x2000, CRC(15fb39af) SHA1(6a20fee9c70b36a6435ac8378f31d5b626017df0) ) // Orchestra-90CC, slot 1
	// $3F8000: slot 2 alternate disk ROM and the slot-3 banks are user content

	// FPGA memory initialisers decoded from the MIF files (section 16)
	ROM_REGION( 0x200, "io_ram", 0 )
	ROM_LOAD( "disk02.bin",    0x000, 0x200, CRC(1d79a15c) SHA1(603f42a590fb87ca0f57b730e8cd877532062e8d) )   // disk02.mif, 6502 $0000
	ROM_REGION( 0x1000, "firmware", 0 )
	ROM_LOAD( "disk02_02.bin", 0x000, 0x1000, CRC(82f8fd4b) SHA1(4209355025f9788779ac5617d65c471643b11fb0) ) // disk02_02.mif, blocks A and B
	ROM_REGION( 0x200, "vdac", 0 )
	ROM_LOAD( "vdac.bin",      0x000, 0x200, CRC(2905adc8) SHA1(869508c7cc0325110305301fe18b120dfb298c26) )   // VDAC.mif, 256 x 16 LE
	ROM_REGION( 0x800, "chargen", 0 )
	ROM_LOAD( "coco3gen.bin",  0x000, 0x800, CRC(0c9b040c) SHA1(34528fce9de7a237d7ddb7b97fca783fc88fb160) )   // coco3gen.mif
ROM_END

} // anonymous namespace


//    YEAR  NAME       PARENT  COMPAT  MACHINE    INPUT      CLASS            INIT        COMPANY        FULLNAME           FLAGS
COMP( 2016, coco3fpga, 0,      0,      coco3fpga, coco3fpga, coco3fpga_state, empty_init, "Gary Becker", "CoCo3FPGA 4.1",   MACHINE_NOT_WORKING | MACHINE_IMPERFECT_TIMING )
