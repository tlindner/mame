// license:BSD-3-Clause
// copyright-holders:Nathan Woods
/***************************************************************************

    coco_vhd.cpp

    Color Computer Virtual Hard Drives (EMUDSK)

    A synthetic hard disk interface for emulators, used by the EMUDSK
    drivers for HDB-DOS and NitrOS-9.  To use it together with a disk
    controller, put both in a Multi-Pak.

****************************************************************************

    Technical specs on the Virtual Hard Disk interface

    Address       Description
    -------       -----------
    FF80          Logical record number (high byte)
    FF81          Logical record number (middle byte)
    FF82          Logical record number (low byte)
    FF83          Command/status register
    FF84          Buffer address (high byte)
    FF85          Buffer address (low byte)

    Set the other registers, and then issue a command to FF83 as follows:

     0 = read 256-byte sector at LRN
     1 = write 256-byte sector at LRN
     2 = flush write cache (Closes and then opens the image file)

    Error values:

     0 = no error
    -1 = power-on state (before the first command is received)
    -2 = invalid command
     2 = VHD image does not exist
     4 = Unable to open VHD image file
     5 = access denied (may not be able to write to VHD image)

    FF86          Drive select (write only, 0 or 1)

    IMPORTANT: The I/O buffer must NOT cross an 8K MMU bank boundary.

 ***************************************************************************/

#include "emu.h"
#include "coco_vhd.h"

#define VERBOSE 0
#include "logmacro.h"


/***************************************************************************
    CONSTANTS
***************************************************************************/

#define VHDSTATUS_OK                    0x00
#define VHDSTATUS_NO_VHD_ATTACHED       0x02
#define VHDSTATUS_ACCESS_DENIED         0x05
#define VHDSTATUS_UNKNOWN_COMMAND       0xFE
#define VHDSTATUS_POWER_ON_STATE        0xFF

#define VHDCMD_READ     0
#define VHDCMD_WRITE    1
#define VHDCMD_FLUSH    2


/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

// ======================> coco_vhd_image_device

DECLARE_DEVICE_TYPE(COCO_VHD, coco_vhd_image_device)

class coco_vhd_image_device :   public device_t,
								public device_image_interface
{
public:
	// construction/destruction
	coco_vhd_image_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);
	virtual ~coco_vhd_image_device();

	// device_image_interface implementation
	virtual std::pair<std::error_condition, std::string> call_load() override;

	virtual bool is_readable()  const noexcept override { return true; }
	virtual bool is_writeable() const noexcept override { return true; }
	virtual bool is_creatable() const noexcept override { return true; }
	virtual bool is_reset_on_load() const noexcept override { return false; }
	virtual const char *file_extensions() const noexcept override { return "vhd"; }
	virtual const char *image_type_name() const noexcept override { return "harddisk"; }
	virtual const char *image_brief_type_name() const noexcept override { return "hard"; }

	// specific implementation
	uint8_t read(offs_t offset);
	void write(address_space &space, offs_t offset, uint8_t data);

protected:
	// device-level overrides
	virtual void device_start() override ATTR_COLD;

	void coco_vhd_readwrite(address_space &space, uint8_t data);

private:
	uint32_t                    m_logical_record_number = 0;
	uint32_t                    m_buffer_address = 0;
	uint8_t                     m_status = 0;
};


// ======================> coco_emudsk_device

namespace
{
	class coco_emudsk_device :
			public device_t,
			public device_cococart_interface
	{
	public:
		// construction/destruction
		coco_emudsk_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

		// optional information overrides
		virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	protected:
		// device-level overrides
		virtual void device_start() override ATTR_COLD;
		virtual void device_reset() override ATTR_COLD;

	private:
		required_device_array<coco_vhd_image_device, 2> m_vhd;

		coco_vhd_image_device *current_vhd();
		u8 vhd_read(offs_t offset);
		void vhd_write(offs_t offset, u8 data);

		u8 m_select;
	};
} // anonymous namespace


/***************************************************************************
    CORE IMPLEMENTATION
***************************************************************************/

DEFINE_DEVICE_TYPE(COCO_VHD, coco_vhd_image_device, "coco_vhd_image", "CoCo Virtual Hard Disk")

//-------------------------------------------------
//  coco_vhd_image_device - constructor
//-------------------------------------------------

coco_vhd_image_device::coco_vhd_image_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, COCO_VHD, tag, owner, clock)
	, device_image_interface(mconfig, *this)
{
}

//-------------------------------------------------
//  coco_vhd_image_device - destructor
//-------------------------------------------------

coco_vhd_image_device::~coco_vhd_image_device()
{
}



//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void coco_vhd_image_device::device_start()
{
	m_status = VHDSTATUS_NO_VHD_ATTACHED;

	save_item(NAME(m_logical_record_number));
	save_item(NAME(m_buffer_address));
	save_item(NAME(m_status));
}



//-------------------------------------------------
//  call_load
//-------------------------------------------------

std::pair<std::error_condition, std::string> coco_vhd_image_device::call_load()
{
	m_status = VHDSTATUS_POWER_ON_STATE;
	m_logical_record_number = 0;
	m_buffer_address = 0;
	return std::make_pair(std::error_condition(), std::string());
}



//-------------------------------------------------
//  coco_vhd_readwrite
//-------------------------------------------------

void coco_vhd_image_device::coco_vhd_readwrite(address_space &space, uint8_t data)
{
	int result, i;
	uint32_t bytes_to_read;
	uint32_t bytes_to_write;
	uint64_t seek_position;
	uint64_t total_size;
	char buffer[1024];

	/* access the image */
	if (!exists())
	{
		m_status = VHDSTATUS_NO_VHD_ATTACHED;
		return;
	}

	/* perform the seek */
	seek_position = uint64_t(256) * m_logical_record_number;
	total_size = length();
	if (fseek(std::min(seek_position, total_size), SEEK_SET))
	{
		m_status = VHDSTATUS_ACCESS_DENIED;
		return;
	}

	/* expand the disk, if necessary */
	if (data == VHDCMD_WRITE)
	{
		while(total_size < seek_position)
		{
			memset(buffer, 0, sizeof(buffer));

			bytes_to_write = (uint32_t)std::min(seek_position - total_size, (uint64_t) sizeof(buffer));
			result = fwrite(buffer, bytes_to_write);
			if (result != bytes_to_write)
			{
				m_status = VHDSTATUS_ACCESS_DENIED;
				return;
			}

			total_size += bytes_to_write;
		}
	}

	switch(data)
	{
		case VHDCMD_READ: /* Read sector */
			memset(buffer, 0, 256);
			if (total_size > seek_position)
			{
				bytes_to_read = (uint32_t)std::min((uint64_t) 256, total_size - seek_position);
				result = fread(buffer, bytes_to_read);
				if (result != bytes_to_read)
				{
					m_status = VHDSTATUS_ACCESS_DENIED;
					return;
				}
			}

			/* write the bytes to memory */
			for (i = 0; i < 256; i++)
				space.write_byte(m_buffer_address + i, buffer[i]);

			m_status = VHDSTATUS_OK;
			break;

		case VHDCMD_WRITE: /* Write Sector */
			/* read the bytes from memory */
			for (i = 0; i < 256; i++)
				buffer[i] = space.read_byte(m_buffer_address + i);

			/* and write to disk */
			result = fwrite(buffer, 256);
			if (result != 256)
			{
				m_status = VHDSTATUS_ACCESS_DENIED;
				return;
			}

			m_status = VHDSTATUS_OK;
			break;

		case VHDCMD_FLUSH: /* Flush file cache */
			m_status = VHDSTATUS_OK;
			break;

		default:
			m_status = VHDSTATUS_UNKNOWN_COMMAND;
			break;
	}
}



uint8_t coco_vhd_image_device::read(offs_t offset)
{
	uint8_t result = 0;

	switch(offset)
	{
		case 0xff83 - 0xff80:
			LOG("vhd: Status read: %d\n", m_status);
			result = m_status;
			break;
	}
	return result;
}



void coco_vhd_image_device::write(address_space &space, offs_t offset, uint8_t data)
{
	int pos;

	switch(offset)
	{
		case 0xff80 - 0xff80:
		case 0xff81 - 0xff80:
		case 0xff82 - 0xff80:
			pos = ((0xff82 - 0xff80) - offset) * 8;
			m_logical_record_number &= ~(0xFF << pos);
			m_logical_record_number += data << pos;
			LOG("vhd: LRN write: %6.6X\n", m_logical_record_number);
			break;

		case 0xff83 - 0xff80:
			coco_vhd_readwrite(space, data);
			LOG("vhd: Command: %d\n", data);
			break;

		case 0xff84 - 0xff80:
			m_buffer_address &= 0xFFFF00FF;
			m_buffer_address += data << 8;
			LOG("vhd: BA write: %X (%2.2X..)\n", m_buffer_address, data);
			break;

		case 0xff85 - 0xff80:
			m_buffer_address &= 0xFFFFFF00;
			m_buffer_address += data;
			LOG("vhd: BA write: %X (..%2.2X)\n", m_buffer_address, data);
			break;
	}
}



/***************************************************************************
    EMUDSK CARTRIDGE
***************************************************************************/

//-------------------------------------------------
//  coco_emudsk_device - constructor
//-------------------------------------------------

coco_emudsk_device::coco_emudsk_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, COCO_EMUDSK, tag, owner, clock)
	, device_cococart_interface(mconfig, *this)
	, m_vhd(*this, "vhd%u", 0U)
	, m_select(0)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void coco_emudsk_device::device_start()
{
	install_readwrite_handler(0xff80, 0xff86,
			read8sm_delegate(*this, FUNC(coco_emudsk_device::vhd_read)),
			write8sm_delegate(*this, FUNC(coco_emudsk_device::vhd_write)));

	save_item(NAME(m_select));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void coco_emudsk_device::device_reset()
{
	m_select = 0;
}

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

void coco_emudsk_device::device_add_mconfig(machine_config &config)
{
	COCO_VHD(config, m_vhd[0]);
	COCO_VHD(config, m_vhd[1]);
}

//-------------------------------------------------
//  current_vhd
//-------------------------------------------------

coco_vhd_image_device *coco_emudsk_device::current_vhd()
{
	return (m_select < m_vhd.size()) ? m_vhd[m_select].target() : nullptr;
}

//-------------------------------------------------
//  vhd_read
//-------------------------------------------------

u8 coco_emudsk_device::vhd_read(offs_t offset)
{
	if (offset == 0xff86 - 0xff80)
		return 0x00;

	coco_vhd_image_device *const vhd = current_vhd();
	if (!vhd)
		return (offset == 0xff83 - 0xff80) ? VHDSTATUS_NO_VHD_ATTACHED : 0x00;

	return vhd->read(offset);
}

//-------------------------------------------------
//  vhd_write
//-------------------------------------------------

void coco_emudsk_device::vhd_write(offs_t offset, u8 data)
{
	if (offset == 0xff86 - 0xff80)
	{
		// writes to $FF86 will switch the VHD
		m_select = data;
		if (!current_vhd())
			LOG("Unknown virtual hard disk index %u\n", data);
	}
	else if (coco_vhd_image_device *const vhd = current_vhd())
	{
		vhd->write(cpu_space(), offset, data);
	}
}


//**************************************************************************
//  DEVICE DECLARATION
//**************************************************************************

DEFINE_DEVICE_TYPE_PRIVATE(COCO_EMUDSK, device_cococart_interface, coco_emudsk_device, "coco_emudsk", "CoCo EMUDSK Virtual Hard Disk Interface")
