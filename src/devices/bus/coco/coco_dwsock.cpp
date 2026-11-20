// license:BSD-3-Clause
// copyright-holders:Nathan Woods
/***************************************************************************

    Virtual Becker Port

    A synthetic serial port for talking to a DriveWire server, as used by
    HDB-DOS and NitrOS-9.  It connects through its bitbanger media, for
    example -bitb socket.127.0.0.1:65504.

    Address       Description
    -------       -----------
    FF41          Status (bit 1 set when a byte is waiting)
    FF42          Data

    The registers are decoded through SCS.  In a Multi-Pak, the SCS select
    in $FF7F must point to the slot holding this cartridge.

***************************************************************************/

#include "emu.h"
#include "coco_dwsock.h"

#include "imagedev/bitbngr.h"

//#define VERBOSE 1
#include "logmacro.h"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> beckerport_device

namespace
{
	class beckerport_device :
			public device_t,
			public device_cococart_interface
	{
	public:
		beckerport_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	protected:
		// device_t implementation
		virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
		virtual void device_start() override ATTR_COLD;

		// device_cococart_interface implementation
		virtual u8 scs_read(offs_t offset) override;
		virtual void scs_write(offs_t offset, u8 data) override;

	private:
		required_device<bitbanger_device> m_socket;

		unsigned int m_rx_pending;
		unsigned int m_head;
		u8 m_buf[0x80];
	};
} // anonymous namespace

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  beckerport_device - constructor
//-------------------------------------------------

beckerport_device::beckerport_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, COCO_DWSOCK, tag, owner, clock)
	, device_cococart_interface(mconfig, *this)
	, m_socket(*this, "socket")
	, m_rx_pending(0)
	, m_head(0)
{
}

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

void beckerport_device::device_add_mconfig(machine_config &config)
{
	BITBANGER(config, m_socket, 0);
}

/*-------------------------------------------------
    device_start
-------------------------------------------------*/

void beckerport_device::device_start()
{
	// save state support
	save_item(NAME(m_rx_pending));
	save_item(NAME(m_head));
	save_item(NAME(m_buf));
}

/*-------------------------------------------------
    scs_read
-------------------------------------------------*/

u8 beckerport_device::scs_read(offs_t offset)
{
	u8 data = 0x5a;

	if (offset != 1 && offset != 2)
		return device_cococart_interface::scs_read(offset);

	if (!m_socket->exists())
		return data;

	switch (offset)
	{
		case 1: // status
			if (!m_rx_pending)
			{
				// Try to read from the DriveWire server
				m_rx_pending = m_socket->input(m_buf, sizeof(m_buf));
				m_head = 0;
			}
			LOG("status read. %d bytes remaining.\n", m_rx_pending);
			data = (m_rx_pending > 0) ? 2 : 0;
			break;
		case 2: // data
			if (!m_rx_pending)
			{
				logerror("read buffer underrun\n");
				break;
			}
			data = m_buf[m_head++];
			m_rx_pending--;
			break;
	}

	return data;
}

/*-------------------------------------------------
    scs_write
-------------------------------------------------*/

void beckerport_device::scs_write(offs_t offset, u8 data)
{
	switch (offset)
	{
		case 1: // status
			LOG("error: write (0x%02x) to status register\n", data);
			break;
		case 2: // data
			m_socket->output(data);
			LOG("data write one byte (0x%02x)\n", data);
			break;
	}
}


//**************************************************************************
//  DEVICE DECLARATION
//**************************************************************************

DEFINE_DEVICE_TYPE_PRIVATE(COCO_DWSOCK, device_cococart_interface, beckerport_device, "coco_dwsock", "Virtual Becker Port")
