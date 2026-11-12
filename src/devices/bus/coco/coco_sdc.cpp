// license:BSD-3-Clause
// copyright-holders:Tim Lindner
/***************************************************************************

    CoCo SDC, revision C (Darren Atkinson, 2013)

    U1: ATmega328P, external 10 MHz clock
    U2: XC9536XL (ROM banking, flash data latch, status latch) and
        XC9572XL (floppy-controller interface); the schematic labels both
        as XC9572XL
    U3: SST39SF010, eight 16 KiB banks

    The MCU implements the FDC command processor.  The CPLD model supplies
    host registers, the MCU register bus and transfer handshakes.  CoCo
    control-latch and HALT/NMI wiring follow coco_fdc.cpp.  Both CPLDs
    ignore A2, so FF44-FF4F mirror FF40-FF4B, and the control latch
    decodes FF40 only.  CPLD equations come from a 2016/2017 programmer
    dump (SDC36XL.jed, SDC72XL.jed) that may predate current boards;
    exact timing remains unverified.

    MCU wiring from CoCoSDC_Schema_RevC.pdf:
      PD0-PD7 = XD0-XD7; PB0 = IRQ; PB1 = INDEX
      PB2 = SD /SS; PB3 = MOSI; PB4 = MISO; PB5 = SCK
      PC0-PC2 = RA0-RA2; PC3 = WE; PC4 = STRB; PC5 = button
    MCU register writes: RA4 status, RA5 track, RA6 sector, RA7 data;
    RA1 controls INTRQ. RA3 publishes a data byte and sets DRQ. Enhanced
    transfers pair RA6 (high byte) with RA3 (low byte). Holding a read
    strobe exposes command/transfer flags; releasing it captures input
    data.

    MCU-FIRM.AVR is a sparse flash image: big-endian 16-bit checksum of
    the remaining bytes, 256 page indices, then 128-byte pages.  Index zero
    leaves a page erased; other indices are one-based payload page numbers.
    The update omits the startup/bootloader at 0x7e00.  A readback from
    real hardware supplied those 512 bytes; all other bytes match the
    expanded update exactly.  The bootloader initializes SP, R0/R1 and
    GPIO directions before entering the application at 0x7400.

    Both WD-compatible byte transfers and SDC-DOS enhanced word transfers
    use the real MCU firmware and its SD/FAT implementation. No separate
    WD177x or host-side FAT parser is involved.

    TODO: index pulse timing

    The DRGN DIP switch is modelled as a separate device. In Dragon mode
    the MCU samples RA0 low, the WD-compatible registers move to FF40-FF43
    and the control register to FF48 (from SETUP.VDK PROGRAMR), and the
    interrupt wiring follows dragon_fdc.cpp: NMI from INTRQ gated by
    control bit 5, DRQ on CART (FIRQ), no HALT. The flash data latch and
    bank register move to FF4A/FF4B with the same programming protocol
    (from the DOSPlus 5.0 DW4/SDC extender v0.25.15 source).  SDC-DOS is
    CoCo-only, so the Dragon device instead has DOSPlus with that extender
    in bank 0 and DragonDOS 1.0 in bank 1.

    Flash programming is inferred from SETUP.DSK FLASHER: FF43 bit 7 arms
    one CTS read as a flash write using the FF42 data latch. Unlock,
    program and erase commands are handled by the SST39SF010 device.

***************************************************************************/

#include "emu.h"
#include "coco_sdc.h"

#define LOG_BUS (1U << 1)
#define LOG_HOST (1U << 2)
#define VERBOSE (0)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(COCO_SDC, coco_sdc_device, "coco_sdc", "CoCo SDC (Rev C)")
DEFINE_DEVICE_TYPE(DRAGON_SDC, dragon_sdc_device, "dragon_sdc", "CoCo SDC (Rev C, Dragon mode)")

ROM_START(coco_sdc)
	ROM_REGION(0x20000, "flash", ROMREGION_ERASEFF)
	// Stock flash: bank 0 SDC-DOS, bank 1 Disk BASIC 1.1; other banks erased.
	ROM_LOAD("disk11.rom", 0x04000, 0x2000, CRC(0b9c5415) SHA1(10bdc5aa2d7d7f205f67b47b19003a4bd89defd1))
	// Extracted from SDC SETUP.DSK, SDC-DOS.ROM (DECB header/trailer removed).
	ROM_LOAD("sdc_dos.rom", 0x00000, 0x4000, CRC(0a16c181) SHA1(16e110743914e0682ec21e5d905476fb1eec174e))

	ROM_REGION(0x8000, "mcu", ROMREGION_ERASEFF)
	ROM_LOAD("atmega328p.bin", 0x0000, 0x8000, CRC(16ed5ab1) SHA1(85b273fc8fb85814eb5a8ff8830e74a2ef0c0d74))
	ROM_REGION(0x400, "eeprom", ROMREGION_ERASEFF)
ROM_END

ROM_START(dragon_sdc)
	// Bank 0 DOSPlus, bank 1 DragonDOS 1.0; SDC-DOS is CoCo-only.
	ROM_REGION(0x20000, "flash", ROMREGION_ERASEFF)
	// DOSPlus 5.0 with Pere Serrat's DW4&SDC extender v0.25.15 (2021-06-10),
	// UPD2515.BIN from "DW42 v0.25.15 SDC-Updater.VDK" (DECB header removed).
	ROM_LOAD("dp50sdc.rom", 0x00000, 0x3f00, CRC(21860147) SHA1(4ad1cda8a0329a0e0f753faa2d027ca1a74967d1))
	ROM_LOAD("ddos10.rom", 0x04000, 0x2000, CRC(b44536f6) SHA1(a8918c71d319237c1e3155bb38620acb114a80bc))

	ROM_REGION(0x8000, "mcu", ROMREGION_ERASEFF)
	// Identical to FIRMWARE.BIN in the Dragon SETUP.VDK, plus bootloader.
	ROM_LOAD("atmega328p.bin", 0x0000, 0x8000, CRC(16ed5ab1) SHA1(85b273fc8fb85814eb5a8ff8830e74a2ef0c0d74))
	ROM_REGION(0x400, "eeprom", ROMREGION_ERASEFF)
ROM_END

static INPUT_PORTS_START(dragon_sdc)
	PORT_START("SWITCHES")
	PORT_CONFNAME(0x07, 0x00, "Startup Flash Bank")
	PORT_CONFSETTING(0x00, "0")
	PORT_CONFSETTING(0x01, "1")
	PORT_CONFSETTING(0x02, "2")
	PORT_CONFSETTING(0x03, "3")
	PORT_CONFSETTING(0x04, "4")
	PORT_CONFSETTING(0x05, "5")
	PORT_CONFSETTING(0x06, "6")
	PORT_CONFSETTING(0x07, "7")
INPUT_PORTS_END

static INPUT_PORTS_START(coco_sdc)
	PORT_START("SWITCHES")
	PORT_CONFNAME(0x07, 0x00, "Startup Flash Bank")
	PORT_CONFSETTING(0x00, "0")
	PORT_CONFSETTING(0x01, "1")
	PORT_CONFSETTING(0x02, "2")
	PORT_CONFSETTING(0x03, "3")
	PORT_CONFSETTING(0x04, "4")
	PORT_CONFSETTING(0x05, "5")
	PORT_CONFSETTING(0x06, "6")
	PORT_CONFSETTING(0x07, "7")

	PORT_START("AUTO")
	PORT_CONFNAME(0x01, 0x00, "Cartridge Auto-Start Jumper")
	PORT_CONFSETTING(0x00, "Not Installed")
	PORT_CONFSETTING(0x01, "AUTO")
INPUT_PORTS_END

coco_sdc_device_base::coco_sdc_device_base(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, bool dragon)
	: device_t(mconfig, type, tag, owner, clock)
	, device_cococart_interface(mconfig, *this)
	, m_mcu(*this, "mcu")
	, m_flash(*this, "flash")
	, m_sdcard(*this, "sdcard")
	, m_switches(*this, "SWITCHES")
	, m_dragon(dragon)
	, m_power_on(true)
	, m_miso(true)
{
}

coco_sdc_device::coco_sdc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: coco_sdc_device_base(mconfig, COCO_SDC, tag, owner, clock, false)
	, m_autostart(*this, "AUTO")
{
}

dragon_sdc_device::dragon_sdc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: coco_sdc_device_base(mconfig, DRAGON_SDC, tag, owner, clock, true)
{
}

void coco_sdc_device_base::program_map(address_map &map)
{
	map(0x0000, 0x7fff).rom();
}

void coco_sdc_device_base::device_add_mconfig(machine_config &config)
{
	ATMEGA328(config, m_mcu, 10_MHz_XTAL);
	m_mcu->set_addrmap(AS_PROGRAM, &coco_sdc_device_base::program_map);
	m_mcu->set_eeprom_tag("eeprom");
	// Inferred BOOTRST/BOOTSZ: reset into the 256-word boot section at 0x7e00.
	// Physical fuse bytes have not been read back.
	m_mcu->set_high_fuses(0xde);
	m_mcu->gpio_out<atmega328_device::GPIOB>().set(FUNC(coco_sdc_device_base::port_b_w));
	m_mcu->gpio_in<atmega328_device::GPIOB>().set([this] () -> u8 { return (m_miso ? 0x10 : 0x00) | (m_command_pending ? 0x01 : 0x00); });
	m_mcu->gpio_out<atmega328_device::GPIOC>().set(FUNC(coco_sdc_device_base::port_c_w));
	// DRGN switch on RA0 (sampled as an input; high = CoCo), button released.
	m_mcu->gpio_in<atmega328_device::GPIOC>().set_constant(m_dragon ? 0x20 : 0x21);
	m_mcu->gpio_in<atmega328_device::GPIOD>().set(FUNC(coco_sdc_device_base::port_d_r));
	m_mcu->gpio_out<atmega328_device::GPIOD>().set(FUNC(coco_sdc_device_base::port_d_w));

	SST_39SF010(config, m_flash);
	SPI_SDCARD(config, m_sdcard, 0);
	m_sdcard->spi_miso_callback().set([this] (int state) { m_miso = bool(state); });
}

void coco_sdc_device_base::port_b_w(u8 data)
{
	// The card device uses an active-high select input; the PCB signal is /SS.
	m_sdcard->spi_ss_w(!BIT(data, 2));
	if (BIT(data, 2))
		m_miso = true; // Deselected cards release MISO.
	m_sdcard->spi_mosi_w(BIT(data, 3));
	m_sdcard->spi_clock_w(BIT(data, 5));
}

void coco_sdc_device_base::port_c_w(u8 data)
{
	if (!BIT(m_port_c, 4) && BIT(data, 4))
	{
		m_strobe_write = BIT(data, 3);
		m_strobe_reg = data & 7;
		if (m_strobe_write && BIT(m_mcu->space(AS_DATA).read_byte(0x27), 4))
			mcu_write(m_strobe_reg, m_port_d);
	}
	else if (BIT(m_port_c, 4) && !BIT(data, 4))
	{
		if (!m_strobe_write)
		{
			// Capture the selected input before requesting another host
			// byte.  Subsequent address changes do not alter this byte.
			m_read_data = mcu_read(m_port_c & 7);
			if ((m_port_c & 7) == 4)
			{
				m_command_pending = false;
				update_mcu_irq();
			}
		}
		if (m_transfer_active && m_strobe_write && m_strobe_reg == 4 && ((m_port_c & 7) == 7))
		{
			m_drq = true;
			update_lines();
		}
	}
	m_port_c = data;
}

void coco_sdc_device_base::port_d_w(u8 data)
{
	// The bootloader holds a RA4 write strobe with XD7-XD3 high, then
	// raises XD0 to restore the DIP-switch bank. RUN @bank,R suppresses
	// this pulse using its retained SRAM flag; RESET is not wired to U2.
	if ((m_port_c & 0x1f) == 0x1c && (data & 0xf8) == 0xf8 && !BIT(m_port_d, 0) && BIT(data, 0)
		&& BIT(m_mcu->space(AS_DATA).read_byte(0x2a), 0))
	{
		m_bank = m_switches->read() & 7;
		m_flash_programming = false;
		cart_base_changed();
	}
	m_port_d = data;
}

u8 coco_sdc_device_base::mcu_read(u8 reg) const
{
	switch (reg)
	{
	case 0:
	case 1:
	case 2:
		return mcu_dskreg();
	case 3:
		return m_registers[3];
	case 4:
		return m_command;
	case 5:
	case 6:
	case 7:
		return m_registers[reg & 3];
	default:
		return 0xff;
	}
}

u8 coco_sdc_device_base::port_d_r()
{
	if (!BIT(m_port_c, 4))
		return m_read_data;
	// While a read strobe is held, the bus reports transfer/command flags.
	// The receive byte remains available after the strobe is released.
	if (!m_strobe_write || !(m_port_c & 4))
		return (mcu_dskreg() & 0x67) | (m_command_pending ? 0x10 : 0) | (m_drq ? 0x80 : 0);
	return mcu_read(m_port_c & 7);
}

void coco_sdc_device_base::mcu_write(u8 reg, u8 data)
{
	LOGMASKED(LOG_BUS, "%s: MCU write RA=%u XD=%02x\n", machine().describe_context(), reg, data);
	if (reg == 1)
	{
		// The low register bank contains interface controls, not mirrors
		// of the WD-compatible registers in the high bank.
		m_intrq = BIT(data, 0);
	}
	else
	{
		m_registers[reg & 3] = data;
		if (!(reg & 3))
		{
			m_transfer_active = BIT(data, 1) && !m_type1;
			m_drq = m_transfer_active;
		}
		else if (reg == 3)
		{
			// Publish a byte, or the low byte of an enhanced-mode word.
			m_drq = true;
		}
	}
	if (m_drq)
		machine().scheduler().perfect_quantum(attotime::from_usec(100));
	update_lines();
}

void coco_sdc_device_base::update_mcu_irq()
{
	m_mcu->pb_w<0>(m_command_pending);
}

void coco_sdc_device::update_lines()
{
	// Same CoCo interrupt/HALT gates as coco_fdc_device_base::update_lines.
	if (m_intrq)
		m_dskreg &= ~0x80;
	set_line_value(line::NMI, m_intrq && BIT(m_dskreg, 5));
	set_line_value(line::HALT, !m_drq && BIT(m_dskreg, 7));
}

void dragon_sdc_device::update_lines()
{
	// Same Dragon interrupt wiring as dragon_fdc_device_base::update_lines.
	set_line_value(line::NMI, m_intrq && BIT(m_dskreg, 5));
	set_line_value(line::CART, m_drq);
}

u8 coco_sdc_device_base::scs_read(offs_t offset)
{
	offset = host_offset(offset);
	// Streaming hosts poll status for READY, then read a whole sector
	// without further handshaking.  Keep the processors interleaved
	// while polling so the first data reads cannot overtake the MCU.
	if (!machine().side_effects_disabled() && (offset & 0x18) == 0x08 && (BIT(m_registers[0], 0) || m_drq))
		machine().scheduler().perfect_quantum(attotime::from_usec(100));

	switch (offset & 0x1b)
	{
	case 2:
		return m_flash_data;
	case 3:
		// The CPLD returns flash data latch bits 3-7 above the bank.
		// The DOSPlus SDC extender (writes 1A, 00) and SDCFLASH (writes
		// 64, 00) presence tests both depend on this.
		return (m_flash_data & 0xf8) | m_bank;
	case 8:
	{
		u8 const result = m_type1
				? m_registers[0]
				: ((m_registers[0] & ~2) | (m_drq ? 2 : 0));
		if (!machine().side_effects_disabled())
		{
			m_intrq = false;
			update_lines();
		}
		return result;
	}
	case 9:
		return m_registers[1];
	case 10:
		return m_registers[2];
	case 11:
	{
		u8 const result = m_registers[3];
		if (!machine().side_effects_disabled())
		{
			m_drq = false;
			update_lines();
		}
		return result;
	}
	default:
		return 0xff;
	}
}

void coco_sdc_device_base::scs_write(offs_t offset, u8 data)
{
	offset = host_offset(offset);
	LOGMASKED(LOG_HOST, "%s: Host write %02x=%02x\n", machine().describe_context(), offset, data);
	switch (offset & 0x1b)
	{
	case 0:
		m_dskreg = data;
		update_lines();
		break;
	case 2:
		m_flash_data = data;
		break;
	case 3:
		m_bank = data & 7;
		m_flash_programming = BIT(data, 7);
		cart_base_changed();
		break;
	case 8:
		// Force Interrupt preserves the previous WD status interpretation.
		if ((data & 0xf0) != 0xd0)
			m_type1 = !BIT(data, 7);
		m_command = data;
		m_command_pending = true;
		m_intrq = false;
		m_drq = false;
		m_transfer_active = false;
		m_registers[0] |= 1;
		update_mcu_irq();
		update_lines();
		break;
	case 9:
	case 10:
		m_registers[offset & 3] = data;
		break;
	case 11:
		m_registers[3] = data;
		m_drq = false;
		update_lines();
		break;
	}
	// The two processors exchange individual bytes/words through hardware
	// latches.  Keep them interleaved while a command is in flight.
	machine().scheduler().perfect_quantum(attotime::from_msec(1));
}

void coco_sdc_device_base::device_start()
{
	save_item(NAME(m_bank));
	save_item(NAME(m_port_c));
	save_item(NAME(m_port_d));
	save_item(NAME(m_miso));
	save_item(NAME(m_registers));
	save_item(NAME(m_command));
	save_item(NAME(m_dskreg));
	save_item(NAME(m_flash_data));
	save_item(NAME(m_flash_programming));
	save_item(NAME(m_command_pending));
	save_item(NAME(m_drq));
	save_item(NAME(m_intrq));
	save_item(NAME(m_strobe_write));
	save_item(NAME(m_strobe_reg));
	save_item(NAME(m_read_data));
	save_item(NAME(m_transfer_active));
	save_item(NAME(m_type1));
	machine().save().register_postload(save_prepost_delegate(FUNC(coco_sdc_device_base::cart_base_changed), this));
}

void coco_sdc_device_base::device_reset()
{
	if (m_power_on)
		m_bank = m_switches->read() & 7;
	std::fill(std::begin(m_registers), std::end(m_registers), 0);
	m_registers[2] = 1;
	m_command = m_dskreg = m_flash_data = 0;
	m_flash_programming = false;
	m_command_pending = m_drq = m_intrq = false;
	m_strobe_write = m_transfer_active = false;
	m_type1 = true;
	m_strobe_reg = 0;
	m_read_data = 0xff;
	m_port_c = m_port_d = 0;
	update_lines();
	cart_base_changed();
}

void coco_sdc_device::device_reset()
{
	coco_sdc_device_base::device_reset();
	set_line_value(line::CART, m_autostart->read() ? line_value::Q : line_value::CLEAR);
}

void coco_sdc_device_base::device_reset_after_children()
{
	// The CoCo RESET pin resets the MCU; it reports EXTRF rather than PORF
	// in MCUSR, so the bootloader can retain RUN @bank,R and mounted disks.
	m_power_on = false;
	port_b_w(0x04); // /SS high, release MISO before firmware starts.
	update_mcu_irq();
}

u8 *coco_sdc_device_base::get_cart_base()
{
	return m_flash->base() + (u32(m_bank) << 14);
}

u32 coco_sdc_device_base::get_cart_size()
{
	return 0x4000;
}

bool coco_sdc_device_base::cts_read_is_dynamic()
{
	return true;
}

offs_t coco_sdc_device_base::host_offset(offs_t offset) const
{
	return offset;
}

u8 coco_sdc_device_base::mcu_dskreg() const
{
	return m_dskreg;
}

offs_t dragon_sdc_device::host_offset(offs_t offset) const
{
	// Dragon mode swaps the halves: WD-compatible registers at FF40-FF43,
	// control register at FF48.
	return offset ^ 0x08;
}

u8 dragon_sdc_device::mcu_dskreg() const
{
	// Dragon single-density (bit 3) is presented where the firmware
	// expects CoCo bit 6, so FF48=0B matches the 43 command-mode code.
	return bitswap<8>(m_dskreg, 7, 3, 5, 4, 6, 2, 1, 0);
}

u8 coco_sdc_device_base::cts_read(offs_t offset)
{
	offs_t const address = (u32(m_bank) << 14) | (offset & 0x3fff);
	// The CPLD substitutes the data latch and pulses /WE on CTS reads
	// when bit 7 of the bank register is set. The enable is consumed by
	// that cycle; subsequent reads verify the flash. CPU writes do not program it.
	if (m_flash_programming)
	{
		if (!machine().side_effects_disabled())
		{
			m_flash->write(address, m_flash_data);
			m_flash_programming = false;
		}
		return 0xff;
	}
	return m_flash->read(address);
}

const tiny_rom_entry *coco_sdc_device::device_rom_region() const
{
	return ROM_NAME(coco_sdc);
}

const tiny_rom_entry *dragon_sdc_device::device_rom_region() const
{
	return ROM_NAME(dragon_sdc);
}

ioport_constructor dragon_sdc_device::device_input_ports() const
{
	// SDC-DOS is CoCo-only; bank 0 holds DOSPlus.
	return INPUT_PORTS_NAME(dragon_sdc);
}

ioport_constructor coco_sdc_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(coco_sdc);
}
