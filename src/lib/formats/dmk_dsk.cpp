// license:BSD-3-Clause
// copyright-holders:Wilbert Pol, tim lindner
/*********************************************************************

    formats/dmk_dsk.h

    DMK disk images

	http://www.classiccmp.org/cpmarchives/trs80/mirrors/www.discover-net.net/
		~dmkeil/trs80/trstech.htm#Technical-DMK-disks

TODO:
- Add format support.

*********************************************************************/

#include <cassert>

#include "dmk_dsk.h"


dmk_format::dmk_format()
{
}


const char *dmk_format::name() const
{
	return "dmk";
}


const char *dmk_format::description() const
{
	return "DMK disk image";
}


const char *dmk_format::extensions() const
{
	return "dmk";
}


int dmk_format::identify(io_generic *io, uint32_t form_factor)
{
	const int header_size = 16;
	uint8_t header[header_size];

	uint64_t size = io_generic_size(io);

	io_generic_read(io, header, 0, header_size);

	int tracks = header[1];
	int track_size = ( header[3] << 8 ) | header[2];
	int heads = (header[4] & HEAD_FLAG) ? 1 : 2;
	bool single_density_size = (header[4] & (1 << 6)) ? true : false;
	bool ignore_density_flag = (header[4] & (1 << 7)) ? true : false;

	// The first header byte must be 00 or FF
	if (header[0] != 0x00 && header[0] != 0xff)
	{
		return 0;
	}

	// Bytes C-F must be zero
	if (header[0x0c] != 0 || header[0xd] != 0 || header[0xe] != 0 || header[0xf] != 0)
	{
		return 0;
	}

	// Check track size within limits
	if (track_size < 0x80 || track_size > 0x3FFF )
	{
		return 0;
	}

	// Check single density size flag - not supported
	if( single_density_size )
	{
		return 0;
	}

	// Check single density ignore flag - not supported
	if( ignore_density_flag )
	{
		return 0;
	}

	if (size == header_size + heads * tracks * track_size)
	{
		return 70;
	}

	return 0;
}


void dmk_format::dmk_count_sectors( std::vector<uint8_t>track_data, int *mfm_sector_count, int *fm_sector_count )
{
	int offset;

	*mfm_sector_count = 0;
	*fm_sector_count = 0;

	for( int i=0; i<SECTOR_SLOT_COUNT; i++)
	{
		if( (track_data[i * 2] == 0) && (track_data[(i * 2) + 1] == 0) ) break;

		offset = track_data[i * 2] << 8 | track_data[(i * 2) + 1];

		if( offset & SECTOR_DOUBLE_DENSITY_FLAG )
		{
			*mfm_sector_count += 1;
		}
		else
		{
			*fm_sector_count += 1;
		}
	}
}


bool dmk_format::load(io_generic *io, uint32_t form_factor, floppy_image *image)
{
	bool sd_track = false, dd_track = false;
	const int header_size = 16;
	uint8_t header[header_size];

	io_generic_read(io, header, 0, header_size);

	const int tracks = header[1];
	const int track_size = ( header[3] << 8 ) | header[2];
	const int heads = (header[4] & HEAD_FLAG) ? 1 : 2;
// 	const bool is_sd = (header[4] & DENSITY_FLAG) ? true : false;

	for (int track = 0; track < tracks; track++)
	{
		for (int head = 0; head < heads; head++)
		{
			std::vector<uint8_t> track_data(track_size);
			std::vector<uint32_t> raw_track_data;
			int iam_location = -1;
			int idam_location[64];
			int dam_location[64];

			// Read track
			io_generic_read(io, &track_data[0], header_size + ( heads * track + head ) * track_size, track_size);

			int fm_sector_count, mfm_sector_count;
			dmk_count_sectors( track_data, &mfm_sector_count, &fm_sector_count );

			if( mfm_sector_count > 0 && fm_sector_count > 0 )
			{
				osd_printf_error("dmk: reading mixed density tracks not supported. Choosing %s. track: %d, side: %d. Dropping %d sectors.\n",
							mfm_sector_count >= fm_sector_count ? "double density" : "single density", track, head,
							mfm_sector_count >= fm_sector_count ? fm_sector_count : mfm_sector_count);

				if( mfm_sector_count >= fm_sector_count )
				{
					fm_sector_count = 0;
				}
				else
				{
					mfm_sector_count = 0;
				}
			}

			for (int i = 0; i < 64; i++)
			{
				idam_location[i] = -1;
				dam_location[i] = -1;
			}

			if( fm_sector_count > 0 ) // Single density track
			{
				sd_track = true;

				// Find IDAM locations
				uint16_t track_header_offset = 0;
				uint16_t track_offset = ( ( track_data[track_header_offset + 1] << 8 ) | track_data[track_header_offset] ) & 0x3fff;
				track_header_offset += 2;

				while ( track_offset != 0 && track_offset >= 0x83 && track_offset < track_size && track_header_offset < 0x80 )
				{
					// Assume 3 bytes before IDAM pointers are the start of IDAM indicators
					idam_location[(track_header_offset/2) - 1] = track_offset;

					// Scan for DAM location
					for (int i = track_offset + 53; i > track_offset + 10; i -= 2)
					{
						if ((track_data[i] == 0xfb || track_data[i] == 0xf8) && track_data[i-2] == 0x00 && track_data[i-4] == 0x00)
						{
							dam_location[(track_header_offset/2) - 1] = i;
							break;
						}
					}

					track_offset = ( ( track_data[track_header_offset + 1] << 8 ) | track_data[track_header_offset] ) & 0x3fff;
					track_header_offset += 2;
				};

				// Find IAM location
				for(int i = idam_location[0] - 1; i >= 3; i -= 2)
				{
					// It's usually 3 bytes but several dumped tracks seem to contain only 2 bytes
					if (track_data[i] == 0xfc && track_data[i-2] == 0x00 && track_data[i-4] == 0x00)
					{
						iam_location = i;
						break;
					}
				}

				int idam_index = 0;
				int dam_index = 0;
				for (int offset = TRACK_HEADER_SIZE; offset < track_size; offset += 2)
				{
					if (offset == iam_location)
					{
						if( track_data[offset] == 0xfc )
						{
							// Write IAM
							raw_w(raw_track_data, 16, 0xf57a);
							offset += 2;
						}
					}

					if (offset == idam_location[idam_index])
					{
						if( track_data[offset] == 0xfe )
						{
							raw_w(raw_track_data, 16, 0xf57e);
							offset += 2;
							idam_index += 1;
						}
					}

					if (offset == dam_location[dam_index])
					{
						if( track_data[offset] == 0xf8 )
						{
							raw_w(raw_track_data, 16, 0xf56a);
							offset += 2;
							dam_index += 1;
						}
						else if( track_data[offset] == 0xf9 )
						{
							raw_w(raw_track_data, 16, 0xf56b);
							dam_index += 1;
							offset += 2;
						}
						else if( track_data[offset] == 0xfa )
						{
							raw_w(raw_track_data, 16, 0xf56e);
							dam_index += 1;
							offset += 2;
						}
						else if( track_data[offset] == 0xfb )
						{
							raw_w(raw_track_data, 16, 0xf56f);
							dam_index += 1;
							offset += 2;
						}
					}

					fm_w(raw_track_data, 8, track_data[offset]);
				}

				generate_track_from_levels(track, head, raw_track_data, 0, image);
			}
			else // double density track
			{
				// Find IDAM locations
				uint16_t track_header_offset = 0;
				uint16_t track_offset = ( ( track_data[track_header_offset + 1] << 8 ) | track_data[track_header_offset] ) & 0x3fff;
				track_header_offset += 2;

				while ( track_offset != 0 && track_offset >= 0x83 && track_offset < track_size && track_header_offset < 0x80 )
				{
					// Assume 3 bytes before IDAM pointers are the start of IDAM indicators
					idam_location[(track_header_offset/2) - 1] = track_offset - 3;

					// Scan for DAM location
					for (int i = track_offset + 53; i > track_offset + 10; i--)
					{
						if ((track_data[i] == 0xfb || track_data[i] == 0xf8) && track_data[i-1] == 0xa1 && track_data[i-2] == 0xa1)
						{
							dam_location[(track_header_offset/2) - 1] = i - 3;
							break;
						}
					}

					track_offset = ( ( track_data[track_header_offset + 1] << 8 ) | track_data[track_header_offset] ) & 0x3fff;
					track_header_offset += 2;
				};

				// Find IAM location
				for(int i = idam_location[0] - 1; i >= 3; i--)
				{
					// It's usually 3 bytes but several dumped tracks seem to contain only 2 bytes
					if (track_data[i] == 0xfc && track_data[i-1] == 0xc2 && track_data[i-2] == 0xc2)
					{
						iam_location = i - 3;
						break;
					}
				}

				int idam_index = 0;
				int dam_index = 0;
				for (int offset = 0x80; offset < track_size; offset++)
				{
					if (offset == iam_location)
					{
						// Write IAM
						raw_w(raw_track_data, 16, 0x5224);
						raw_w(raw_track_data, 16, 0x5224);
						raw_w(raw_track_data, 16, 0x5224);
						offset += 3;
					}

					if (offset == idam_location[idam_index])
					{
						raw_w(raw_track_data, 16, 0x4489);
						raw_w(raw_track_data, 16, 0x4489);
						raw_w(raw_track_data, 16, 0x4489);
						idam_index += 1;
						offset += 3;
					}

					if (offset == dam_location[dam_index])
					{
						raw_w(raw_track_data, 16, 0x4489);
						raw_w(raw_track_data, 16, 0x4489);
						raw_w(raw_track_data, 16, 0x4489);
						dam_index += 1;
						offset += 3;
					}

					mfm_w(raw_track_data, 8, track_data[offset]);
				}

				generate_track_from_levels(track, head, raw_track_data, 0, image);
			}
		}
	}

	if (heads == 2)
	{
		if( sd_track == true && dd_track == false)
		{
			image->set_variant(floppy_image::DSSD);
		}
		else
		{
			image->set_variant(floppy_image::DSDD);
		}
	}
	else
	{
		if( sd_track == true && dd_track == false)
		{
			image->set_variant(floppy_image::SSSD);
		}
		else
		{
			image->set_variant(floppy_image::SSDD);
		}
	}

	return true;
}


void dmk_format::get_geometry_mfm_pc(const uint8_t *bitstream, int track_size, int &sector_count)
{
	uint8_t sectdata[50000];
	desc_xs sectors[256];
	extract_sectors_from_bitstream_mfm_pc(bitstream, track_size, sectors, sectdata, sizeof(sectdata));
	for(sector_count = 44; sector_count > 0 && !sectors[sector_count].data; sector_count--);
}


void dmk_format::get_geometry_fm_pc(const uint8_t *bitstream, int track_size, int &sector_count)
{
	uint8_t sectdata[50000];
	desc_xs sectors[256];
	extract_sectors_from_bitstream_fm_pc(bitstream, track_size, sectors, sectdata, sizeof(sectdata));
	for(sector_count = 44; sector_count > 0 && !sectors[sector_count].data; sector_count--);
}


uint64_t dmk_format::dmk_write_filler(struct io_generic *genio, uint8_t filler, uint64_t offset, size_t length)
{
	io_generic_write_filler(genio, filler, offset, length);
	return offset + length;
}


uint64_t dmk_format::dmk_write(struct io_generic *genio, const void *buffer, uint64_t offset, size_t length)
{
	io_generic_write( genio, buffer, offset, length );
	return offset + length;
}


uint64_t dmk_format::dmk_sd_write(struct io_generic *genio, const void *buffer, uint64_t offset, size_t length)
{
	uint8_t *pointer = (uint8_t *)buffer;

	for( int i=0; i<length; i++ )
	{
		offset = dmk_write_filler(genio, pointer[i], offset, 2);
	}

	return offset;
}


uint64_t dmk_format::dmk_write_le_uint16(struct io_generic *genio, uint16_t value, uint64_t offset)
{
	value = little_endianize_int16(value);
	io_generic_write( genio, &value, offset, 2 );
	return offset + 2;
}


uint64_t dmk_format::dmk_write_uint8(struct io_generic *genio, uint8_t value, uint64_t offset)
{
	value = little_endianize_int16(value);
	io_generic_write( genio, &value, offset, 1 );
	return offset + 1;
}


bool dmk_format::save(io_generic *io, floppy_image *image)
{
	uint64_t offset = 0;
	int mfm_act_track_size = 0, fm_act_track_size = 0;
	uint8_t mfm_bitstream[500000/8], fm_bitstream[500000/8], track_buffer[TRACK_SIZE];

	int track_count, head_count;
	image->get_maximal_geometry(track_count, head_count);

	if (track_count > 80) track_count = 80;
	if( head_count > 2) head_count = 2;

	// write DMK header
	offset = dmk_write_uint8(io, 0, offset); // no write protected
	offset = dmk_write_uint8(io, track_count, offset);
	offset = dmk_write_le_uint16(io, TRACK_SIZE, offset); // track length
	offset = dmk_write_uint8(io, (head_count == 1 ? HEAD_FLAG : 0), offset); // options
	offset = dmk_write_filler(io, 0, offset, 7); // reserved
	offset = dmk_write_filler(io, 0, offset, 4); // not native disk

	for (int track = 0; track < track_count; track++)
	{
		for ( int head = 0; head < head_count; head++ )
		{
			int mfm_sector_count, fm_sector_count;

 			generate_bitstream_from_track(track, head, 2000, mfm_bitstream, mfm_act_track_size, image);
 			get_geometry_mfm_pc(mfm_bitstream, mfm_act_track_size, mfm_sector_count);

 			generate_bitstream_from_track(track, head, 4000, fm_bitstream, fm_act_track_size, image);
 			get_geometry_fm_pc(fm_bitstream, fm_act_track_size, fm_sector_count);

			if( mfm_sector_count > 0 && fm_sector_count > 0 )
			{
				osd_printf_error("dmk: writing mixed density tracks not supported. Choosing %s. track: %d, side: %d. Dropping %d sectors.\n",
							mfm_sector_count >= fm_sector_count ? "double density" : "single density", track, head,
							mfm_sector_count >= fm_sector_count ? fm_sector_count : mfm_sector_count);

				if( mfm_sector_count >= fm_sector_count )
				{
					fm_sector_count = 0;
				}
				else
				{
					mfm_sector_count = 0;
				}
			}

			if( mfm_sector_count > 0 && fm_sector_count == 0)
			{
				// writing mfm track
				int blocks[BLOCKS_COUNT];
				extract_track_from_bitstream_mfm_pc(mfm_bitstream, mfm_act_track_size, track_buffer, TRACK_SIZE, blocks, BLOCKS_COUNT);
				offset = dmk_write_uint8(io, mfm_sector_count, offset);
				offset = dmk_write_filler(io, 0, offset, 7);

				int i=0, j=0;

				while( i < BLOCKS_COUNT-1 && j < SECTOR_SLOT_COUNT )
				{
					if( blocks[i] == -1 )
					{
						offset = dmk_write_le_uint16(io, 0, offset);
					}
					else
					{
						int pos = blocks[i++];

						if( track_buffer[pos] == 0xfe || track_buffer[pos] == 0xff )
						{
							// we found a idam
							offset = dmk_write_le_uint16(io, (pos + (SECTOR_SLOT_COUNT * 2)) | DOUBLE_DENSITY_FLAG, offset);
						}
					}

					j++;
				}

				for( ; j < SECTOR_SLOT_COUNT; j++ )
				{
					offset = dmk_write_filler(io, 0, offset, 8);
				}

				offset = dmk_write(io, track_buffer, offset, TRACK_SIZE);
			}
			else if(mfm_sector_count == 0 && fm_sector_count > 0)
			{
				// writing fm track
				int blocks[BLOCKS_COUNT];
				extract_track_from_bitstream_fm_pc(fm_bitstream, fm_act_track_size, track_buffer, TRACK_SIZE/2, blocks, BLOCKS_COUNT);
				offset = dmk_write_uint8(io, fm_sector_count, offset);
				offset = dmk_write_filler(io, 0, offset, 7);

				int i=0, j=0;

				while( i < BLOCKS_COUNT-1 && j < SECTOR_SLOT_COUNT )
				{
					if( blocks[i] == -1 )
					{
						offset = dmk_write_le_uint16(io, 0, offset);
					}
					else
					{
						int pos = blocks[i++];

						if( track_buffer[pos] == 0xfe )
						{
							// we found a idam
							offset = dmk_write_le_uint16(io, ((pos * 2) + (SECTOR_SLOT_COUNT * 2)), offset);
						}
					}

					j++;
				}

				for( ; j < SECTOR_SLOT_COUNT; j++ )
				{
					offset = dmk_write_filler(io, 0, offset, 8);
				}

				offset = dmk_sd_write(io, track_buffer, offset, TRACK_SIZE/2);
			}
			else
			{
				// no sectors found
				offset = dmk_write_filler(io, 0, offset, TOTAL_TRACK_SIZE);
			}
		}
	}

	return true;
}


bool dmk_format::supports_save() const
{
	return true;
}


const floppy_format_type FLOPPY_DMK_FORMAT = &floppy_image_format_creator<dmk_format>;
