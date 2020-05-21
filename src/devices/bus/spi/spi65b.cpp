// license:GPL-v2
// copyright-holders:Tormod Volden, tim lindner
/*********************************************************************

    spi65b.h: 65SPI/B

    An SPI interface for the 65C02 family of microprocessors.

    http://www.6502.org/users/andre/spi65b/index.html

    This device was created to provide a basic SPI interface for the 65xx family of
    microprocessors. Currently, the only way to provide SPI is to bit-bang it using a
    6522 or equivalent device. That uses a lot of microprocessor time and program space.

    This device takes care of the data loading, shifting, clocking, and control - freeing
    the microprocessor for more important duties. There is interrupt support to allow an
    ISR to handle the SPI interface.   The status register provides signals for polling
    if interrupts are not desired.


*********************************************************************/

#include "emu.h"
#include "spi65b.h"
#include "spi65b_sdcard.h"

#define LOG_SPI65B         (1 << 0 )

#define VERBOSE 0 //(LOG_SPI65B)
#include "logmacro.h"

#define SPI_DATA 0
#define SPI_CTRL 1
#define SPI_STATUS 1
#define SPI_CLK 2
#define SPI_SIE 3

#define SPI_CTRL_TRANMISSION_CONTROL  0x80
#define SPI_CTRL_INTERRUPT_ENABLE  0x40
#define SPI_CTRL_FAST_RECIEVE_MODE 0x10

#define SPI_IRQ_STATUS_0 0x10
#define SPI_IRQ_STATUS_1 0x20
#define SPI_IRQ_STATUS_2 0x40
#define SPI_IRQ_STATUS_3 0x80

/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(SPI65BBUS_SLOT, spi65b_bus_slot_device, "spi65b_slot", "65SPI/B SPI Bus")

//**************************************************************************
//  SPI65B SLOT DEVICE
//**************************************************************************

//-------------------------------------------------
//  spi65b_bus_slot_device - constructor
//-------------------------------------------------
spi65b_bus_slot_device::spi65b_bus_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SPI65BBUS_SLOT, tag, owner, clock)
	, device_slot_interface(mconfig, *this)
	, m_spi65b_bus(*this, finder_base::DUMMY_TAG)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void spi65b_bus_slot_device::device_validity_check(validity_checker &valid) const
{
	device_t *const card(get_card_device());
	if (card && !dynamic_cast<device_spi65b_bus_interface *>(card))
		osd_printf_error("Card device %s (%s) does not implement device_spi65b_bus_interface\n", card->tag(), card->name());
}

void spi65b_bus_slot_device::device_resolve_objects()
{
	device_spi65b_bus_interface *const spi65b_bus_card(dynamic_cast<device_spi65b_bus_interface *>(get_card_device()));
	if (spi65b_bus_card)
		spi65b_bus_card->set_spi65b_bus(m_spi65b_bus, tag());
}

void spi65b_bus_slot_device::device_start()
{
	device_t *const card(get_card_device());
	if (card && !dynamic_cast<device_spi65b_bus_interface *>(card))
		throw emu_fatalerror("spi65b_bus_slot_device: card device %s (%s) does not implement device_spi65b_bus_interface\n", card->tag(), card->name());
}

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(SPI65BBUS, spi65b_bus_device, "spi65b_bus", "65SPI/B Bus")

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  spi65b_bus_device - constructor
//-------------------------------------------------

spi65b_bus_device::spi65b_bus_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: spi65b_bus_device(mconfig, SPI65BBUS, tag, owner, clock)
{
}

spi65b_bus_device::spi65b_bus_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
 	, m_out_irq_cb(*this)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void spi65b_bus_device::device_resolve_objects()
{
	// resolve callbacks
 	m_out_irq_cb.resolve_safe();
}

void spi65b_bus_device::device_start()
{
	// clear slots
	std::fill(std::begin(m_device_list), std::end(m_device_list), nullptr);

    save_item(NAME(m_reg_data_in));
    save_item(NAME(m_reg_data_out));
    save_item(NAME(m_status));
    save_item(NAME(m_clock_divisor));
    save_item(NAME(m_slave_select_interrupt_enable));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void spi65b_bus_device::device_reset()
{
	m_reg_data_in = 0xFF;
	m_reg_data_out = 0;
	m_status = 0;
	m_clock_divisor = 0;
	m_slave_select_interrupt_enable = 0x0F;
}

device_spi65b_bus_interface *spi65b_bus_device::get_spi65b_bus_card(int slot)
{
	if (slot < 0)
	{
		return nullptr;
	}

	if (m_device_list[slot])
	{
		return m_device_list[slot];
	}

	return nullptr;
}

void spi65b_bus_device::add_spi65b_bus_card(int slot, device_spi65b_bus_interface *card)
{
	m_device_list[slot] = card;
}

READ8_MEMBER(spi65b_bus_device::spi65b_bus_read)
{
	uint8_t result = 0;

	switch( offset & 3) {
        case SPI_DATA:
            result = m_reg_data_in;
            m_status &= ~SPI_CTRL_TRANMISSION_CONTROL;
            if (m_status & SPI_CTRL_FAST_RECIEVE_MODE) {

                if (m_device_list[3] && ((m_slave_select_interrupt_enable & 8) == 0)) {
                    m_reg_data_in = m_device_list[3]->spi_transfer(m_reg_data_out, (m_slave_select_interrupt_enable & 8) == 0);
                    set_irq_line(CLEAR_LINE, 3, false);
                }

                if (m_device_list[2] && ((m_slave_select_interrupt_enable & 4) == 0)) {
                    m_reg_data_in = m_device_list[2]->spi_transfer(m_reg_data_out, (m_slave_select_interrupt_enable & 4) == 0);
                    set_irq_line(CLEAR_LINE, 2, false);
                }

                if (m_device_list[1] && ((m_slave_select_interrupt_enable & 2) == 0)) {
                    m_reg_data_in = m_device_list[1]->spi_transfer(m_reg_data_out, (m_slave_select_interrupt_enable & 2) == 0);
                    set_irq_line(CLEAR_LINE, 1, false);
                }

                if (m_device_list[0] && ((m_slave_select_interrupt_enable & 1) == 0)) {
                    m_reg_data_in = m_device_list[0]->spi_transfer(m_reg_data_out, (m_slave_select_interrupt_enable & 1) == 0);
                    set_irq_line(CLEAR_LINE, 0, false);
                }

                update_line();
            }
            break;
        case SPI_STATUS:
            result = m_status;
            m_status |= SPI_CTRL_TRANMISSION_CONTROL; // complete next time
            break;
        case SPI_CLK:
            result = m_clock_divisor;
            break;
        case SPI_SIE:
            result = m_slave_select_interrupt_enable;
            break;
	}

	LOGMASKED(LOG_SPI65B, "spi65b_bus_slot_device::spi65b_bus_read offset %02x, data: %02x\n", offset, result );
    return result;
}

WRITE8_MEMBER(spi65b_bus_device::spi65b_bus_write)
{
	LOGMASKED(LOG_SPI65B, "spi65b_bus_slot_device::spi65b_bus_write offset %02x, data: %02x\n", offset, data );
	switch( offset & 3) {
        case SPI_DATA:
            m_reg_data_out = data;

            if (m_device_list[3] && ((m_slave_select_interrupt_enable & 8) == 0)) {
                m_reg_data_in = m_device_list[3]->spi_transfer(m_reg_data_out, (m_slave_select_interrupt_enable & 8) == 0);
                set_irq_line(CLEAR_LINE, 3, false);
            }

            if (m_device_list[2] && ((m_slave_select_interrupt_enable & 4) == 0)) {
                m_reg_data_in = m_device_list[2]->spi_transfer(m_reg_data_out, (m_slave_select_interrupt_enable & 4) == 0);
                set_irq_line(CLEAR_LINE, 2, false);
            }

            if (m_device_list[1] && ((m_slave_select_interrupt_enable & 2) == 0)) {
                m_reg_data_in = m_device_list[1]->spi_transfer(m_reg_data_out, (m_slave_select_interrupt_enable & 2) == 0);
                set_irq_line(CLEAR_LINE, 1, false);
            }

            if (m_device_list[0] && ((m_slave_select_interrupt_enable & 1) == 0)) {
                m_reg_data_in = m_device_list[0]->spi_transfer(m_reg_data_out, (m_slave_select_interrupt_enable & 1) == 0);
                set_irq_line(CLEAR_LINE, 0, false);
            }

            m_status &= ~SPI_CTRL_TRANMISSION_CONTROL;
            update_line();
            break;
        case SPI_CTRL:
            m_status = (data & ~0xA0) | (m_status & 0xA0);
            update_line();
            break;
        case SPI_CLK:
            m_clock_divisor = (data & 0x0F) | (m_clock_divisor & ~0xF0);
            break;
        case SPI_SIE:
            m_slave_select_interrupt_enable = data;
            update_line();
            break;
    }
}

void spi65b_bus_device::set_irq_line(int state, int slot, bool update)
{
	if (state == CLEAR_LINE)
	{
		m_clock_divisor &= ~(0x10 << slot);
	}
	else if (state == ASSERT_LINE)
	{
		m_clock_divisor |= (0x10 << slot);
	}

	if(update) update_line();
}

void spi65b_bus_device::update_line(void)
{
    int state = CLEAR_LINE;

    if( m_status & SPI_CTRL_TRANMISSION_CONTROL) {
        if( m_status & SPI_CTRL_INTERRUPT_ENABLE ) {
            if( m_clock_divisor & SPI_IRQ_STATUS_0 || m_clock_divisor & SPI_IRQ_STATUS_1 ||
                m_clock_divisor & SPI_IRQ_STATUS_2 || m_clock_divisor & SPI_IRQ_STATUS_3 ) {
                state = ASSERT_LINE;
            }
        }
    }

	m_out_irq_cb(state);
}

// interrupt request from spi65b slave device
WRITE_LINE_MEMBER( spi65b_bus_device::irq_w ) { m_out_irq_cb(state); }

//**************************************************************************
//  DEVICE SPI65B INTERFACE - Implemented by devices that plug into
//  SPI bus
//**************************************************************************

template class device_finder<device_spi65b_bus_interface, false>;
template class device_finder<device_spi65b_bus_interface, true>;

//-------------------------------------------------
//  device_spi65b_bus_interface - constructor
//-------------------------------------------------

device_spi65b_bus_interface::device_spi65b_bus_interface(const machine_config &mconfig, device_t &device)
	: device_slot_card_interface(mconfig, device)
	, m_spi65b_bus_finder(device, finder_base::DUMMY_TAG)
	, m_spi65b_bus(nullptr)
	, m_spi65b_bus_slottag(nullptr)
	, m_slot(-1)
	, m_next(nullptr)
{
}

//-------------------------------------------------
//  ~device_spi65b_bus_interface - destructor
//-------------------------------------------------

device_spi65b_bus_interface::~device_spi65b_bus_interface()
{
}

void device_spi65b_bus_interface::interface_validity_check(validity_checker &valid) const
{
	if (m_spi65b_bus_finder && m_spi65b_bus && (m_spi65b_bus != m_spi65b_bus_finder))
		osd_printf_error("Contradictory buses configured (%s and %s)\n", m_spi65b_bus_finder->tag(), m_spi65b_bus->tag());
}

void device_spi65b_bus_interface::interface_pre_start()
{
	device_slot_card_interface::interface_pre_start();

	if (!m_spi65b_bus)
	{
		m_spi65b_bus = m_spi65b_bus_finder;
		if (!m_spi65b_bus)
			fatalerror("Can't find 65SPI/B device %s\n", m_spi65b_bus_finder.finder_tag());
	}

	if (0 > m_slot)
	{
		if (!m_spi65b_bus->started())
			throw device_missing_dependencies();

		// extract the slot number from the last digit of the slot tag
		size_t const tlen = strlen(m_spi65b_bus_slottag);

		m_slot = (m_spi65b_bus_slottag[tlen - 1] - '0');
		if (m_slot < 0 || m_slot > 3)
			fatalerror("Slot %x out of range for 65SPI/B bus\n", m_slot);

		m_spi65b_bus->add_spi65b_bus_card(m_slot, this);
	}
}

uint8_t device_spi65b_bus_interface::spi_transfer(uint8_t data_out, int slave_select_active)
{
    fatalerror("device_spi65b_bus_interface::spi_transfer should never be called\n");
    return 0;
}
