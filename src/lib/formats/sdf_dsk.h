// license:BSD-3-Clause
// copyright-holders:Wilbert Pol, tim lindner
/*********************************************************************

    formats/sdf_dsk.h

    SDF disk images. Format created by Darren Atkinson for use with
    his CoCoSDC floppy disk emulator.

*********************************************************************/
#ifndef MAME_FORMATS_SDF_DSK_H
#define MAME_FORMATS_SDF_DSK_H

#pragma once

#include "flopimg.h"

/**************************************************************************/

class sdf_format : public floppy_image_format_t
{
public:
	sdf_format();

	virtual int identify(io_generic *io, uint32_t form_factor) override;
	virtual bool load(io_generic *io, uint32_t form_factor, floppy_image *image) override;
	virtual bool save(io_generic *io, floppy_image *image) override;

	virtual const char *name() const override;
	virtual const char *description() const override;
	virtual const char *extensions() const override;
	virtual bool supports_save() const override;


protected:
	struct desc_dsk_sector
	{
		uint8_t track, head, sector, size;
		int actual_size;
		int id_pos;
		int data_pos;
		bool deleted;
		bool bad_id_crc;
		bool bad_data_crc;
	};

	void get_geometry_mfm_pc(const uint8_t *bitstream, int track_size, int &sector_count);
	void get_geometry_fm_pc(const uint8_t *bitstream, int track_size, int &sector_count);
	void sdf_count_sectors( std::vector<uint8_t>track_data, int *mfm_sector_count, int *fm_sector_count );

	uint64_t sdf_write_filler(struct io_generic *genio, uint8_t filler, uint64_t offset, size_t length);
	uint64_t sdf_write(struct io_generic *genio, const void *buffer, uint64_t offset, size_t length);
	uint64_t sdf_write_le_uint16(struct io_generic *genio, uint16_t value, uint64_t offset);
	uint64_t sdf_write_uint8(struct io_generic *genio, uint8_t value, uint64_t offset);
	uint64_t sdf_sd_write(struct io_generic *genio, const void *buffer, uint64_t offset, size_t length);

	static constexpr int HEADER_SIZE  = 512;
	static constexpr int TRACK_HEADER_SIZE  = 256;
	static constexpr int TRACK_SIZE  = 6250;
	static constexpr int TRACK_PADDING  = 150;
	static constexpr int TOTAL_TRACK_SIZE  = TRACK_HEADER_SIZE + TRACK_SIZE + TRACK_PADDING;
	static constexpr int SINGLE_DENSITY_FLAG = 0x4000;
	static constexpr int DELETED_DATA_FLAG = 0x4000;
	static constexpr int BAD_CRC_FLAG = 0x8000;
	static constexpr int BLOCKS_COUNT = 200;

	static constexpr int SECTOR_SLOT_COUNT  = 31;
};

extern const floppy_format_type FLOPPY_SDF_FORMAT;

#endif // MAME_FORMATS_SDF_DSK_H
