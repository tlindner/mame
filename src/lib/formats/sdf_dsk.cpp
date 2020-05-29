// license:BSD-3-Clause
// copyright-holders:Wilbert Pol, tim lindner
/*********************************************************************

    formats/sdf_dsk.h

    SDF disk images. Format created by Darren Atkinson for use with
    his CoCoSDC floppy disk emulator.

    http://cocosdc.blogspot.com/p/sd-card-socket-sd-card-socket-is-push.html

*********************************************************************/

#include "sdf_dsk.h"
#include "imageutl.h"
#include <cassert>

sdf_format::sdf_format()
{
}


const char *sdf_format::name() const
{
	return "sdf";
}


const char *sdf_format::description() const
{
	return "SDF disk image";
}


const char *sdf_format::extensions() const
{
	return "sdf";
}


int sdf_format::identify(io_generic *io, uint32_t form_factor)
{
	uint8_t header[HEADER_SIZE];

	uint64_t size = io_generic_size(io);

	if (size < HEADER_SIZE)
	{
		return 0;
	}

	io_generic_read(io, header, 0, HEADER_SIZE);

	int tracks = header[4];
	int heads = header[5];

	// Check magic bytes
	if (header[0] != 'S' || header[1] != 'D' || header[2] != 'F' || header[3] != '1')
	{
		return 0;
	}

	// Check heads
	if (heads != 1 && heads != 2)
	{
		return 0;
	}

	if (size == HEADER_SIZE + heads * tracks * TOTAL_TRACK_SIZE)
	{
		return 100;
	}

	return 0;
}

void sdf_format::sdf_count_sectors( std::vector<uint8_t>track_data, int *mfm_sector_count, int *fm_sector_count )
{
	int count = track_data[0];
	*mfm_sector_count = 0;
	*fm_sector_count = 0;

	for( int i=0; i<count; i++ )
	{
		if( (track_data[(i+1)*8] & SINGLE_DENSITY_FLAG) != 0 )
		{
			*fm_sector_count += 1;

		}
		else
		{
			*mfm_sector_count += 1;
		}
	}
}

bool sdf_format::load(io_generic *io, uint32_t form_factor, floppy_image *image)
{
	bool sd_track = false, dd_track = false;
	uint8_t header[HEADER_SIZE];
	std::vector<uint8_t> track_data(TOTAL_TRACK_SIZE);
	std::vector<uint32_t> raw_track_data;

	io_generic_read(io, header, 0, HEADER_SIZE);

	const int tracks = header[4];
	const int heads = header[5];

	for (int track = 0; track < tracks; track++)
	{
		for (int head = 0; head < heads; head++)
		{
			int iam_location = -1;
			int idam_location[SECTOR_SLOT_COUNT+1];
			int dam_location[SECTOR_SLOT_COUNT+1];
			raw_track_data.clear();

			// Read track
			io_generic_read(io, &track_data[0], HEADER_SIZE + ( heads * track + head ) * TOTAL_TRACK_SIZE, TOTAL_TRACK_SIZE);

			int sector_count = track_data[0];

			if (sector_count > SECTOR_SLOT_COUNT) return false;

			int fm_sector_count, mfm_sector_count;
			sdf_count_sectors( track_data, &mfm_sector_count, &fm_sector_count );

			if( mfm_sector_count > 0 && fm_sector_count > 0 )
			{
				osd_printf_error("sdf: reading mixed density tracks not supported. Choosing %s. track: %d, side: %d. Dropping %d sectors.\n",
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

			if( fm_sector_count > 0 ) // single density track
			{
				sd_track = true;

				// remove double bytes from track buffer
// 				for( int i=0; i<TRACK_SIZE; i += 2)
// 				{
// 					track_data[256+(i/2)] = track_data[256 + i];
// 				}

				// Transfer IDAM and DAM locations to table
				for (int i = 0; i < SECTOR_SLOT_COUNT+1; i++)
				{
					if (i < sector_count )
					{
						idam_location[i] = (((track_data[ 8 * (i+1) + 1] << 8 | track_data[ 8 * (i+1)]) & 0x3FFF));
						dam_location[i] = (((track_data[ 8 * (i+1) + 1 + 2] << 8 | track_data[ 8 * (i+1) + 2]) & 0x3FFF));

// 						idam_location[i] -= 256;
// 						idam_location[i] /= 2;
// 						idam_location[i] += 256;
//
// 						dam_location[i] -= 256;
// 						dam_location[i] /= 2;
// 						dam_location[i] += 256;


						if (idam_location[i] > TOTAL_TRACK_SIZE) return false;
						if (dam_location[i] > TOTAL_TRACK_SIZE) return false;
					}
					else
					{
						idam_location[i] = -1;
						dam_location[i] = -1;
					}
				}

				// Find IAM location
				for (int i = idam_location[0] - 1; i >= (TRACK_HEADER_SIZE) + 3; i -= 2)
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
				for (int offset = TRACK_HEADER_SIZE; offset < TRACK_HEADER_SIZE + TRACK_SIZE; offset += 2)
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
				dd_track = true;

				// Transfer IDAM and DAM locations to table
				for (int i = 0; i < SECTOR_SLOT_COUNT+1; i++)
				{
					if (i < sector_count )
					{
						idam_location[i] = ((track_data[ 8 * (i+1) + 1] << 8 | track_data[ 8 * (i+1)]) & 0x3FFF) - 4;
						dam_location[i] = ((track_data[ 8 * (i+1) + 1 + 2] << 8 | track_data[ 8 * (i+1) + 2]) & 0x3FFF) - 4;

						if (idam_location[i] > TOTAL_TRACK_SIZE) return false;
						if (dam_location[i] > TOTAL_TRACK_SIZE) return false;
					}
					else
					{
						idam_location[i] = -1;
						dam_location[i] = -1;
					}
				}

				// Find IAM location
				for (int i = idam_location[0] - 1; i >= TRACK_HEADER_SIZE + 3; i--)
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
				for (int offset = TRACK_HEADER_SIZE; offset < TRACK_HEADER_SIZE + TRACK_SIZE; offset++)
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

void sdf_format::get_geometry_mfm_pc(const uint8_t *bitstream, int track_size, int &sector_count)
{
	uint8_t sectdata[50000];
	desc_xs sectors[256];
	extract_sectors_from_bitstream_mfm_pc(bitstream, track_size, sectors, sectdata, sizeof(sectdata));
	for(sector_count = 44; sector_count > 0 && !sectors[sector_count].data; sector_count--);
}

void sdf_format::get_geometry_fm_pc(const uint8_t *bitstream, int track_size, int &sector_count)
{
	uint8_t sectdata[50000];
	desc_xs sectors[256];
	extract_sectors_from_bitstream_fm_pc(bitstream, track_size, sectors, sectdata, sizeof(sectdata));
	for(sector_count = 44; sector_count > 0 && !sectors[sector_count].data; sector_count--);
}

uint64_t sdf_format::sdf_write_filler(struct io_generic *genio, uint8_t filler, uint64_t offset, size_t length)
{
	io_generic_write_filler(genio, filler, offset, length);
	return offset + length;
}

uint64_t sdf_format::sdf_write(struct io_generic *genio, const void *buffer, uint64_t offset, size_t length)
{
	io_generic_write( genio, buffer, offset, length );
	return offset + length;
}

uint64_t sdf_format::sdf_sd_write(struct io_generic *genio, const void *buffer, uint64_t offset, size_t length)
{
	uint8_t *pointer = (uint8_t *)buffer;

	for( int i=0; i<length; i++ )
	{
		offset = sdf_write_filler(genio, pointer[i], offset, 2);
	}

	return offset;
}

uint64_t sdf_format::sdf_write_le_uint16(struct io_generic *genio, uint16_t value, uint64_t offset)
{
	value = little_endianize_int16(value);
	io_generic_write( genio, &value, offset, 2 );
	return offset + 2;
}

uint64_t sdf_format::sdf_write_uint8(struct io_generic *genio, uint8_t value, uint64_t offset)
{
	value = little_endianize_int16(value);
	io_generic_write( genio, &value, offset, 1 );
	return offset + 1;
}

unsigned short sdf_format::sdf_ccitt_crc16(unsigned short crc, const unsigned char *buffer, size_t buffer_len)
{
	// same as the utility function but does every other byte
	for( int i=0; i<buffer_len; i += 2 )
	{
		crc = ccitt_crc16_one( crc, buffer[i] );
	}

	return crc;
}

bool sdf_format::save(io_generic *io, floppy_image *image)
{
	uint64_t offset = 0;

	int track_count, head_count;
	image->get_actual_geometry(track_count, head_count);

	if (track_count > 80) track_count = 80;
	if( head_count > 2) head_count = 2;

	// write header
	offset = sdf_write(io, "SDF1", offset, 4);
	offset = sdf_write_uint8(io, track_count, offset);
	offset = sdf_write_uint8(io, head_count, offset);
	offset = sdf_write_uint8(io, 0, offset); // write permission
	offset = sdf_write_uint8(io, 0, offset); // nested sectors
	offset = sdf_write_filler(io, 0, offset, 504); // reserved

	for (int track = 0; track < track_count; track++)
	{
		for ( int head = 0; head < head_count; head++ )
		{
			printf( "doing track: %d, head: %d\n", track, head );
			bool last_sector_mfm = true;
			uint8_t mfm_bitstream[500000/8], fm_bitstream[500000/8], mfm_track_buffer[TRACK_SIZE], fm_track_buffer[TRACK_SIZE], track_buffer[TRACK_SIZE];
			int mfm_act_track_size = 0, fm_act_track_size = 0;
			int mfm_blocks[BLOCKS_COUNT], fm_blocks[BLOCKS_COUNT];
			unsigned int value;

 			generate_bitstream_from_track(track, head, 2000, mfm_bitstream, mfm_act_track_size, image);
 			generate_bitstream_from_track(track, head, 4000, fm_bitstream, fm_act_track_size, image);
			extract_track_from_bitstream_mfm_pc(mfm_bitstream, mfm_act_track_size, mfm_track_buffer, TRACK_SIZE, mfm_blocks, BLOCKS_COUNT);
			extract_track_from_bitstream_fm_pc(fm_bitstream, fm_act_track_size, fm_track_buffer, TRACK_SIZE, fm_blocks, BLOCKS_COUNT);

			int sector_count;
			get_geometry_mfm_pc(mfm_bitstream, mfm_act_track_size, sector_count);
			printf( "mfm sector count: %d\n", sector_count );
			get_geometry_fm_pc(fm_bitstream, fm_act_track_size, sector_count);
			printf( "fm sector count: %d\n", sector_count );

			int mfm_index = 0, fm_index = 0, header_index = 0, track_index = 0;

			if( (fm_blocks[0] == INT_MAX) && (mfm_blocks[0] == INT_MAX) )
			{
				// no sectors found
				offset = sdf_write_filler(io, 0, offset, TOTAL_TRACK_SIZE);
				continue;
			}

			while( header_index < SECTOR_SLOT_COUNT )
			{
				if( (fm_index == BLOCKS_COUNT) && (mfm_index == BLOCKS_COUNT) ) break;

				if( fm_blocks[fm_index] < mfm_blocks[mfm_index] )
				{
					do_fm:
					// write fm sector
					if( fm_index == BLOCKS_COUNT ) continue;

					if( fm_blocks[fm_index] < TRACK_SIZE )
					{
						int pos = fm_blocks[fm_index++];

						if( pos < TRACK_SIZE && fm_track_buffer[pos] == 0xfe )
						{
							printf( "doing sd sector\n" );
							// we found a idam
							last_sector_mfm = false;

							int disk_crc;
							int calc_crc;

							if( pos + 12 < TRACK_SIZE )
							{
								disk_crc = (track_buffer[pos+(5*2)] << 8) | track_buffer[pos+(6*2)];
								calc_crc = sdf_ccitt_crc16(0xffff, &(track_buffer[pos]), (5*2));
							}
							else
							{
								disk_crc = 0;
								calc_crc = 1;
							}

							value = pos + 256 + 1;

							if( value < TRACK_HEADER_SIZE + TRACK_SIZE )
								value |= (disk_crc != calc_crc ? BAD_CRC_FLAG : 0 ) | (SINGLE_DENSITY_FLAG);
							else
								value = 0;

							offset = sdf_write_le_uint16(io, value, offset);

							while( track_index < pos + 12 )
							{
								if( track_index < TRACK_SIZE )
								{
									track_buffer[track_index] = fm_track_buffer[track_index];
									track_index++;
								}
							}

							int data_pos = fm_blocks[fm_index];
							int delta = data_pos - pos;

							if( (delta > 36) && (delta < 71) && (fm_track_buffer[data_pos] == 0xf8 || fm_track_buffer[data_pos] == 0xf9 || fm_track_buffer[data_pos] == 0xfa || fm_track_buffer[data_pos] == 0xfb))
							{
								// we found a dam
								int sector_length = 128 << track_buffer[pos+(4*2)];
								if( sector_length > 1024) sector_length == 1024;

								if( data_pos + sector_length + 4 < TRACK_SIZE )
								{
									disk_crc = (track_buffer[data_pos + sector_length + 1] << 8) | track_buffer[data_pos + sector_length + 2];
									calc_crc = sdf_ccitt_crc16(0xffff, &(fm_track_buffer[data_pos]), (sector_length+1)*2);
								}
								else
								{
									disk_crc = 0;
									calc_crc = 1;
								}

								value = data_pos + 256 + 1;

								if( value < TRACK_HEADER_SIZE + TRACK_SIZE )
									value |= (disk_crc != calc_crc ? BAD_CRC_FLAG : 0 ) | (track_buffer[data_pos] == 0xf8 ? DELETED_DATA_FLAG : 0);
								else
									value = 0;

								offset = sdf_write_le_uint16(io, value, offset);
								offset = sdf_write_uint8(io, track_buffer[pos+1], offset);
								offset = sdf_write_uint8(io, track_buffer[pos+2], offset);
								offset = sdf_write_uint8(io, track_buffer[pos+3], offset);
								offset = sdf_write_uint8(io, track_buffer[pos+4], offset);

								while( track_index < data_pos + (sector_length + 2)*2 )
								{
									if( track_index < TRACK_SIZE )
									{
										track_buffer[track_index] = fm_track_buffer[track_index];
										track_index++;
									}
								}

								fm_index++;
							}
							else
							{
								offset = sdf_write_filler(io, 0, offset, 6);
							}
						}
					}
					else
					{
						fm_index++;
					}
				}
				else
				{
					// write mfm sector
					if( mfm_index == BLOCKS_COUNT ) goto do_fm;

					if( mfm_blocks[mfm_index] < TRACK_SIZE )
					{
						int pos = mfm_blocks[mfm_index++];

						if( pos < TRACK_SIZE && (mfm_track_buffer[pos] == 0xfe || mfm_track_buffer[pos] == 0xff) )
						{
							printf( "doing dd sector\n" );
							// we found a idam
							last_sector_mfm = true;

							int disk_crc, calc_crc;

							if( pos + 6 < TRACK_SIZE )
							{
								disk_crc = (mfm_track_buffer[pos+5] << 8) | mfm_track_buffer[pos+6];
								calc_crc = ccitt_crc16(0xffff, &(mfm_track_buffer[pos-3]), 8);
							}
							else
							{
								disk_crc = 0;
								calc_crc = 1;
							}

							value = pos + 256 + 1;

							if( value < TRACK_HEADER_SIZE + TRACK_SIZE )
								value |= (disk_crc != calc_crc ? BAD_CRC_FLAG : 0 );
							else
								value = 0;

							offset = sdf_write_le_uint16(io, value, offset);

							while( track_index < pos + 6 )
							{
								if( track_index < TRACK_SIZE )
								{
									track_buffer[track_index] = mfm_track_buffer[track_index];
									track_index++;
								}
							}

							int data_pos = mfm_blocks[mfm_index];

							if( data_pos == INT_MAX )
							{
								offset = sdf_write_filler(io, 0, offset, 6);
								mfm_index++;
							}
							else
							{
								int delta = data_pos - pos;

								if( (delta > 36) && (delta < 71) && (mfm_track_buffer[data_pos] == 0xf8 || mfm_track_buffer[data_pos] == 0xf9 || mfm_track_buffer[data_pos] == 0xfa || mfm_track_buffer[data_pos] == 0xfb))
								{
									// we found a dam
									int sector_length = 128 << mfm_track_buffer[pos+4];
									if( sector_length > 1024) sector_length == 1024;

									if( data_pos + sector_length + 2 < TRACK_SIZE )
									{
										disk_crc = (mfm_track_buffer[data_pos + sector_length + 1] << 8) | mfm_track_buffer[data_pos + sector_length + 2];
										calc_crc = ccitt_crc16(0xffff, &(mfm_track_buffer[data_pos-3]), sector_length+4);
									}
									else
									{
										disk_crc = 0;
										calc_crc = 1;
									}

									value = data_pos + 256 + 1;

									if( value < TRACK_HEADER_SIZE + TRACK_SIZE )
										value |= (disk_crc != calc_crc ? BAD_CRC_FLAG : 0 ) | (mfm_track_buffer[data_pos] == 0xf8 ? DELETED_DATA_FLAG : 0);
									else
										value = 0;

									offset = sdf_write_le_uint16(io, value, offset);
									offset = sdf_write_uint8(io, mfm_track_buffer[pos+1], offset);
									offset = sdf_write_uint8(io, mfm_track_buffer[pos+2], offset);
									offset = sdf_write_uint8(io, mfm_track_buffer[pos+3], offset);
									offset = sdf_write_uint8(io, mfm_track_buffer[pos+4], offset);

									while( track_index < data_pos + sector_length + 2 )
									{
										if( track_index < TRACK_SIZE )
										{
											track_buffer[track_index] = mfm_track_buffer[track_index];
											track_index++;
										}
									}

									mfm_index++;
								}
								else
								{
									offset = sdf_write_filler(io, 0, offset, 6);
								}
							}
						}
					}
					else
					{
						mfm_index++;
					}
				}
				header_index++;
			}

			while( header_index < SECTOR_SLOT_COUNT )
			{
				offset = sdf_write_filler(io, 0, offset, 8);
			}

			if( last_sector_mfm )
			{
				while( track_index < TRACK_SIZE )
				{
					track_buffer[track_index] = mfm_track_buffer[track_index];
					track_index++;
				}
			}
			else
			{
				while( track_index < TRACK_SIZE )
				{
					track_buffer[track_index] = fm_track_buffer[track_index];
					track_index++;
				}
			}

			offset = sdf_write(io, track_buffer, offset, TRACK_SIZE);
			offset = sdf_write_filler(io, 0, offset, TRACK_PADDING);
		}
	}

	return true;
}


bool sdf_format::supports_save() const
{
	return true;
}


const floppy_format_type FLOPPY_SDF_FORMAT = &floppy_image_format_creator<sdf_format>;
