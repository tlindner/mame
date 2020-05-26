// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/*********************************************************************

    formats/dmk_dsk.h

    DMK disk images

*********************************************************************/
#ifndef MAME_FORMATS_DMK_DSK_H
#define MAME_FORMATS_DMK_DSK_H

#pragma once

#include "flopimg.h"

/**************************************************************************/

class dmk_format : public floppy_image_format_t
{
public:
	dmk_format();

	virtual int identify(io_generic *io, uint32_t form_factor) override;
	virtual bool load(io_generic *io, uint32_t form_factor, floppy_image *image) override;
	virtual bool save(io_generic *io, floppy_image *image) override;

	virtual const char *name() const override;
	virtual const char *description() const override;
	virtual const char *extensions() const override;
	virtual bool supports_save() const override;

protected:
	void get_geometry_mfm_pc(const uint8_t *bitstream, int track_size, int &sector_count);
	void get_geometry_fm_pc(const uint8_t *bitstream, int track_size, int &sector_count);
	void dmk_count_sectors( std::vector<uint8_t>track_data, int *mfm_sector_count, int *fm_sector_count );

	uint64_t dmk_write_filler(struct io_generic *genio, uint8_t filler, uint64_t offset, size_t length);
	uint64_t dmk_write(struct io_generic *genio, const void *buffer, uint64_t offset, size_t length);
	uint64_t dmk_write_le_uint16(struct io_generic *genio, uint16_t value, uint64_t offset);
	uint64_t dmk_write_uint8(struct io_generic *genio, uint8_t value, uint64_t offset);
	uint64_t dmk_sd_write(struct io_generic *genio, const void *buffer, uint64_t offset, size_t length);

	static constexpr int HEAD_FLAG = 0x10;
	static constexpr int DENSITY_FLAG = 0x40;
	static constexpr int SECTOR_DOUBLE_DENSITY_FLAG = 0x80;
	static constexpr int DOUBLE_DENSITY_FLAG = 0x8000;
	static constexpr int TRACK_SIZE  = 6272;
	static constexpr int SECTOR_SLOT_COUNT = 64;
	static constexpr int TRACK_HEADER_SIZE = 128;
	static constexpr int BLOCKS_COUNT = 200;
	static constexpr int TOTAL_TRACK_SIZE  = TRACK_HEADER_SIZE + TRACK_SIZE;
};

extern const floppy_format_type FLOPPY_DMK_FORMAT;

#endif // MAME_FORMATS_DMK_DSK_H
