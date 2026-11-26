// license:BSD-3-Clause
// copyright-holders:tim lindner
/***************************************************************************

    CoCoMP3 serial MP3 player

    A DY-SV5W MP3/WAV player module that plugs into the Color Computer's
    bit-banger serial port. It plays MP3 and WAV files from its own
    microSD card.

    Serial format is 9600 baud, 8N1. Commands and replies use the
    packet format:

        AA cmd len data... sum

    where sum is the low byte of the sum of all preceding bytes.

    The microSD card is a raw card image mounted as "mp3card": the bytes
    of a real card (read it with dd, balenaEtcher or Win32 Disk Imager),
    with or without an MBR partition table, FAT12/16/32 or exFAT. With no
    image the module behaves as it does with no card inserted. Lad vs Sewer
    Slimes, for example, wants /LAD/00001.MP3 and so on on the card.

    Numbering, path matching, timing and the other behaviour modelled here
    were measured on a real module (hwtest/ in the CoCoMP3 project): within
    each folder its MP3 and WAV files come first in directory-entry order,
    then its subfolders depth-first; paths match the 8.3 short names.

    The real device's transistor interface sends its replies inverted
    compared to standard RS-232, so the CoCo reads PIA1 PB0 low when the
    line is idle and high for a start bit. CoCo software written for it
    (Lad vs Sewer Slimes, Glen Hewlett's CoCoMP3 library) expects this,
    so this device drives its output inverted too. Confirmed on real
    hardware.

    External CD is low while idle and high during playback, as confirmed
    by the hardware designer. The CoCo's receiver inverts it before PIA1
    CA1, so drive the emulated DCD line low during playback and high idle.

    Audio: with nothing plugged into the module's 3.5 mm line-out jack, its
    audio goes, mixed to mono, down the cassette cable into the CoCo's
    cassette input, where software can read it through the cassette
    comparator and hear it with AUDIO ON. With a cable plugged in, the jack
    switch cuts the cassette feed and stereo goes to the line out. The
    "CoCoMP3 Line Out Jack" configuration setting selects which.

    TODO:
    - EQ filters
    - the power-on volume (20 or 30) and the noise heard at volume 0
    - save state support for an MP3 that is playing

***************************************************************************/

#include "emu.h"
#include "cocomp3.h"

#include "multibyte.h"

#include "diserial.h"
#include "speaker.h"

#include "machine/mc14529.h"
#include "sound/mp3_audio.h"


#include "corestr.h"

#include <algorithm>
#include <deque>

#define LOG_COMMAND (1U << 1)
#define LOG_FILE    (1U << 2)

//#define VERBOSE (LOG_COMMAND | LOG_FILE)
#include "logmacro.h"

#define LOGCMD(...)  LOGMASKED(LOG_COMMAND, __VA_ARGS__)
#define LOGFILE(...) LOGMASKED(LOG_FILE, __VA_ARGS__)


namespace {

//**************************************************************************
//  CARD FILESYSTEM (read-only FAT12/16/32 and exFAT)
//**************************************************************************

class card_fs
{
public:
	struct entry
	{
		std::string short_name;     // 8.3 as "NAME.EXT" (FAT); long name on exFAT
		std::string long_name;      // long name, or the 8.3 name when there is none
		bool is_dir = false;
		u32 cluster = 0;
		u64 size = 0;
		bool contiguous = false;    // exFAT NoFatChain
	};

	using reader = std::function<bool (u64 offset, void *buffer, u32 length)>;

	bool mount(reader read);
	void unmount() { m_type = NONE; }
	bool mounted() const { return m_type != NONE; }
	bool is_exfat() const { return m_type == EXFAT; }
	u32 root() const { return m_root; }

	std::vector<entry> list(u32 cluster, u64 size = 0, bool contiguous = false) const;
	bool load(const entry &e, std::vector<u8> &data) const;

private:
	enum type_t { NONE, FAT12, FAT16, FAT32, EXFAT };

	u32 next_cluster(u32 cluster) const;
	u64 cluster_offset(u32 cluster) const { return m_heap + u64(cluster - 2) * m_cluster_size; }
	bool read_chain(u32 cluster, u64 limit, bool contiguous, std::vector<u8> &data) const;

	reader m_read;
	type_t m_type = NONE;
	u64 m_base = 0;             // partition start
	u32 m_bps = 512;
	u32 m_cluster_size = 0;
	u64 m_fat = 0;              // first FAT
	u64 m_heap = 0;             // cluster 2
	u64 m_root_dir = 0;         // FAT12/16 fixed root
	u32 m_root_entries = 0;
	u32 m_root = 0;             // root cluster (0 = fixed FAT12/16 root)
	u32 m_clusters = 0;
};

bool card_fs::mount(reader read)
{
	m_read = std::move(read);
	m_type = NONE;
	u8 sec[512];

	// MBR with a partition table, or a filesystem from sector 0
	m_base = 0;
	if (!m_read(0, sec, 512))
		return false;
	bool const boot0 = (sec[0] == 0xeb || sec[0] == 0xe9) && sec[510] == 0x55 && sec[511] == 0xaa;
	if (!boot0 && sec[510] == 0x55 && sec[511] == 0xaa)
	{
		for (int i = 0; i < 4; i++)
		{
			u8 const *const p = &sec[446 + i * 16];
			u32 const lba = get_u32le(p + 8);
			if (p[4] && lba)
			{
				m_base = u64(lba) * 512;
				break;
			}
		}
		if (!m_read(m_base, sec, 512))
			return false;
	}

	if (!memcmp(&sec[3], "EXFAT   ", 8))
	{
		m_bps = 1U << sec[108];
		m_cluster_size = m_bps << sec[109];
		m_fat = m_base + u64(get_u32le(&sec[80])) * m_bps;
		m_heap = m_base + u64(get_u32le(&sec[88])) * m_bps;
		m_clusters = get_u32le(&sec[92]);
		m_root = get_u32le(&sec[96]);
		m_type = EXFAT;
		return true;
	}

	m_bps = get_u16le(&sec[11]);
	u32 const spc = sec[13];
	u32 const reserved = get_u16le(&sec[14]);
	u32 const fats = sec[16];
	m_root_entries = get_u16le(&sec[17]);
	u32 const total = get_u16le(&sec[19]) ? get_u16le(&sec[19]) : get_u32le(&sec[32]);
	u32 const fat_size = get_u16le(&sec[22]) ? get_u16le(&sec[22]) : get_u32le(&sec[36]);
	if (!m_bps || (m_bps & (m_bps - 1)) || !spc || !fats || !fat_size)
		return false;

	u32 const root_sectors = (m_root_entries * 32 + m_bps - 1) / m_bps;
	u32 const first_data = reserved + fats * fat_size + root_sectors;
	if (total <= first_data)
		return false;
	m_cluster_size = spc * m_bps;
	m_clusters = (total - first_data) / spc;
	m_fat = m_base + u64(reserved) * m_bps;
	m_root_dir = m_base + u64(reserved + fats * fat_size) * m_bps;
	m_heap = m_base + u64(first_data) * m_bps;
	m_type = (m_clusters < 4085) ? FAT12 : (m_clusters < 65525) ? FAT16 : FAT32;
	m_root = (m_type == FAT32) ? get_u32le(&sec[44]) : 0;
	return true;
}

u32 card_fs::next_cluster(u32 cluster) const
{
	u8 b[4];
	switch (m_type)
	{
	case FAT12:
		if (!m_read(m_fat + cluster + cluster / 2, b, 2))
			return 0;
		{
			u16 const v = get_u16le(b);
			u32 const next = (cluster & 1) ? (v >> 4) : (v & 0xfff);
			return (next >= 0xff8 || next < 2) ? 0 : next;
		}
	case FAT16:
		if (!m_read(m_fat + cluster * 2, b, 2))
			return 0;
		{
			u32 const next = get_u16le(b);
			return (next >= 0xfff8 || next < 2) ? 0 : next;
		}
	case FAT32:
		if (!m_read(m_fat + u64(cluster) * 4, b, 4))
			return 0;
		{
			u32 const next = get_u32le(b) & 0x0fffffff;
			return (next >= 0x0ffffff8 || next < 2) ? 0 : next;
		}
	case EXFAT:
		if (!m_read(m_fat + u64(cluster) * 4, b, 4))
			return 0;
		{
			u32 const next = get_u32le(b);
			return (next >= 0xfffffff7 || next < 2) ? 0 : next;
		}
	default:
		return 0;
	}
}

bool card_fs::read_chain(u32 cluster, u64 limit, bool contiguous, std::vector<u8> &data) const
{
	data.clear();
	u32 guard = m_clusters + 2;
	while (cluster >= 2 && data.size() < limit && guard--)
	{
		size_t const pos = data.size();
		u32 const count = u32(std::min<u64>(m_cluster_size, limit - pos));
		data.resize(pos + count);
		if (!m_read(cluster_offset(cluster), &data[pos], count))
			return false;
		cluster = contiguous ? (cluster + 1) : next_cluster(cluster);
	}
	return data.size() >= limit || limit == ~u64(0);
}

bool card_fs::load(const entry &e, std::vector<u8> &data) const
{
	if (!e.size)
	{
		data.clear();
		return true;
	}
	return read_chain(e.cluster, e.size, e.contiguous, data) && data.size() == e.size;
}

static std::string ucs2_to_ascii(const std::u16string &s)
{
	std::string out;
	for (char16_t c : s)
		out += (c && c < 0x80) ? char(c) : '_';
	return out;
}

std::vector<card_fs::entry> card_fs::list(u32 cluster, u64 size, bool contiguous) const
{
	std::vector<entry> result;
	std::vector<u8> dir;

	if (m_type == NONE)
		return result;
	if (m_type != EXFAT && cluster == 0)
	{
		dir.resize(m_root_entries * 32);
		if (!m_read(m_root_dir, dir.data(), dir.size()))
			return result;
	}
	else if (contiguous && size)
	{
		if (!read_chain(cluster, size, true, dir))
			return result;
	}
	else if (!read_chain(cluster, ~u64(0), false, dir))
		return result;

	if (m_type == EXFAT)
	{
		// file entry 85, stream extension C0, file name entries C1
		for (size_t i = 0; i + 32 <= dir.size(); i += 32)
		{
			u8 const *const e = &dir[i];
			if (e[0] == 0x00)
				break;
			if (e[0] != 0x85)
				continue;
			u8 const secondaries = e[1];
			u16 const attr = get_u16le(e + 4);
			if (i + (secondaries + 1) * 32 > dir.size() || dir[i + 32] != 0xc0)
				continue;
			u8 const *const s = &dir[i + 32];
			entry ent;
			ent.is_dir = attr & 0x10;
			ent.contiguous = s[1] & 0x02;
			ent.cluster = get_u32le(s + 20);
			ent.size = get_u64le(s + 24);
			u8 const name_len = s[3];
			std::u16string name;
			for (unsigned n = 2; n <= secondaries && name.size() < name_len; n++)
			{
				u8 const *const f = &dir[i + n * 32];
				if (f[0] != 0xc1)
					break;
				for (int c = 0; c < 15 && name.size() < name_len; c++)
					name += char16_t(get_u16le(f + 2 + c * 2));
			}
			ent.long_name = ucs2_to_ascii(name);
			ent.short_name = ent.long_name;
			result.push_back(std::move(ent));
			i += secondaries * 32;
		}
		return result;
	}

	std::u16string lfn;
	for (size_t i = 0; i + 32 <= dir.size(); i += 32)
	{
		u8 const *const e = &dir[i];
		if (e[0] == 0x00)
			break;
		if (e[0] == 0xe5)
		{
			lfn.clear();
			continue;
		}
		if (e[11] == 0x0f)
		{
			// long name pieces come last part first
			std::u16string part;
			static const int offs[] = { 1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30 };
			for (int o : offs)
			{
				char16_t const c = get_u16le(e + o);
				if (c == 0 || c == 0xffff)
					break;
				part += c;
			}
			if (e[0] & 0x40)
				lfn = part;
			else
				lfn = part + lfn;
			continue;
		}
		if (e[11] & 0x08)
		{
			lfn.clear();
			continue;   // volume label
		}

		std::string name(reinterpret_cast<const char *>(e), 8);
		std::string ext(reinterpret_cast<const char *>(e + 8), 3);
		name.erase(name.find_last_not_of(' ') + 1);
		ext.erase(ext.find_last_not_of(' ') + 1);
		if (!name.empty() && u8(name[0]) == 0x05)
			name[0] = char(0xe5);
		if (name == "." || name == "..")
		{
			lfn.clear();
			continue;
		}

		entry ent;
		ent.short_name = ext.empty() ? name : (name + "." + ext);
		if (!lfn.empty())
			ent.long_name = ucs2_to_ascii(lfn);
		else
		{
			// Windows NT case bits: 0x08 lowercase name, 0x10 lowercase extension
			std::string n = name, x = ext;
			if (e[12] & 0x08)
				std::transform(n.begin(), n.end(), n.begin(), [] (char c) { return char(tolower(u8(c))); });
			if (e[12] & 0x10)
				std::transform(x.begin(), x.end(), x.begin(), [] (char c) { return char(tolower(u8(c))); });
			ent.long_name = x.empty() ? n : (n + "." + x);
		}
		lfn.clear();
		ent.is_dir = e[11] & 0x10;
		ent.cluster = (m_type == FAT32 ? (u32(get_u16le(e + 20)) << 16) : 0) | get_u16le(e + 26);
		ent.size = get_u32le(e + 28);
		result.push_back(std::move(ent));
	}
	return result;
}


//**************************************************************************
//  DEVICE
//**************************************************************************

class serial_cocomp3_device : public device_t,
		public device_serial_interface,
		public device_rs232_port_interface,
		public device_sound_interface,
		public device_image_interface,
		public cocomp3_cassette_interface
{
public:
	serial_cocomp3_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void input_txd(int state) override { device_serial_interface::rx_w(state); }

	// device_image_interface
	virtual std::pair<std::error_condition, std::string> call_load() override;
	virtual void call_unload() override;
	virtual bool is_readable() const noexcept override { return true; }
	virtual bool is_writeable() const noexcept override { return false; }
	virtual bool is_creatable() const noexcept override { return false; }
	virtual bool is_reset_on_load() const noexcept override { return false; }
	virtual const char *file_extensions() const noexcept override { return "img,ima,bin,raw,hd"; }
	virtual const char *image_type_name() const noexcept override { return "mp3card"; }
	virtual const char *image_brief_type_name() const noexcept override { return "mp3c"; }

	// cocomp3_cassette_interface
	virtual double cassette_input() override;

	DECLARE_INPUT_CHANGED_MEMBER(jack_changed);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual void tra_callback() override;
	virtual void tra_complete() override;
	virtual void rcv_complete() override;

	virtual void sound_stream_update(sound_stream &stream) override;

private:
	enum : u8
	{
		STATUS_STOP = 0,
		STATUS_PLAY = 1,
		STATUS_PAUSE = 2
	};

	enum : u8
	{
		MODE_LOOP_ALL = 0,          // full cycle
		MODE_LOOP_ONE = 1,          // single cycle
		MODE_SINGLE_STOP = 2,
		MODE_RANDOM = 3,
		MODE_LOOP_FOLDER = 4,
		MODE_RANDOM_FOLDER = 5,
		MODE_FOLDER_ONCE = 6,
		MODE_ALL_ONCE = 7
	};

	enum : u8
	{
		DRIVE_USB = 0,
		DRIVE_SD = 1,
		DRIVE_FLASH = 2
	};

	struct track
	{
		card_fs::entry entry;
		std::string short_path;     // "/MUSIC/00001.MP3"
		int folder = -1;
	};

	struct folder
	{
		std::string short_path;
		int first = 0;
		int count = 0;
	};

	struct position
	{
		int track = -1;
		size_t file_pos = 0;
	};

	static constexpr int MAX_VOLUME = 30;
	static constexpr attotime INTERBYTE_TIMEOUT = attotime::from_msec(20);
	static constexpr double CASSETTE_GAIN = 0.5;

	// Timings measured on a real CoCoMP3 (hwtest/MP3TEST.BAS, Nov 2026)
	static constexpr double START_DELAY = 0.33;                                // command to first audio
	static constexpr attotime CD_PAUSE_DELAY = attotime::from_msec(170);       // pause to CD idle
	static constexpr attotime CD_STOP_DELAY = attotime::from_msec(310);        // stop to CD idle
	static constexpr attotime HOLD_START = attotime::from_msec(330);           // replies wait until the new track is heard
	static constexpr attotime HOLD_STOP = attotime::from_msec(375);            // replies wait this long after stop
	static constexpr double VOL_DB_STEP = 1.61;                                // measured amplitude step per volume unit
	static constexpr double COMPARATOR_THRESHOLD = 0.00028;                    // cassette comparator offset (volume 1 = 5.8x this)

	TIMER_CALLBACK_MEMBER(send_reply);
	TIMER_CALLBACK_MEMBER(update_sample_rate);
	TIMER_CALLBACK_MEMBER(cd_idle);

	void process_packet();
	void reply_byte(u8 cmd, u8 data);
	void reply_word(u8 cmd, u16 data);

	// track list
	void scan_tracks();
	void scan_card(const card_fs::entry &dir, const std::string &path, int depth);
	int find_path(const std::string &path) const;
	bool read_card(u64 offset, void *buffer, u32 length);

	// playback
	bool load_track(int index, std::vector<u8> &data);
	bool start_track(int index, size_t file_pos = 0);
	bool play_track(int index);
	void stop(attotime cd_delay = attotime::zero);
	void track_finished();
	int folder_of(int index) const { return (index >= 0 && index < int(m_tracks.size())) ? m_tracks[index].folder : -1; }
	int pick_random(int first, int count);
	void set_dcd_playing(bool playing, attotime delay = attotime::zero);
	bool busy() const;
	void send_reply_packet();

	// decoding
	bool fill_window();
	bool decode_frame();
	bool decode_mp3_frame();
	bool decode_wav_frame();
	bool parse_wav();
	size_t play_pos() const;

	required_ioport m_jack;

	sound_stream *m_stream;
	emu_timer *m_reply_timer;
	emu_timer *m_cd_timer;

	// receive packet parser
	std::vector<u8> m_packet;
	attotime m_last_rx;

	std::deque<u8> m_reply;

	// card
	card_fs m_fs;
	std::vector<track> m_tracks;
	std::vector<folder> m_folders;

	// player state
	int m_track;
	u8 m_status;
	u8 m_mode;
	int m_volume;
	u8 m_eq;
	u16 m_cycle_times;          // 0 = unlimited
	u16 m_cycles_done;
	bool m_interlude;
	position m_resume;          // where an interlude returns to
	position m_comb_resume;     // where a combination returns to (track -1: stop)
	attotime m_hold_until;      // replies wait until then
	std::vector<int> m_combination;
	size_t m_combination_pos;
	bool m_jack_plugged;
	float m_cassette_level;
	double m_lead_in;           // seconds of silence before a started track is heard

	// decoder
	std::vector<u8> m_file;
	size_t m_file_pos;
	size_t m_frame_pos;         // file offset of the MP3 frame being heard
	std::array<u8, 0x2000> m_window;
	int m_window_count;
	std::unique_ptr<mp3_audio> m_decoder;
	std::array<s16, 1152 * 2> m_samples;
	int m_sample_count;
	int m_sample_index;
	int m_channels;
	int m_sample_rate;

	// WAV
	bool m_is_wav;
	u16 m_wav_format;           // 1 = PCM, 3 = IEEE float
	u16 m_wav_channels;
	u16 m_wav_bits;
	u16 m_wav_align;
	u32 m_wav_rate;
	size_t m_wav_end;
};


serial_cocomp3_device::serial_cocomp3_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SERIAL_COCOMP3, tag, owner, clock)
	, device_serial_interface(mconfig, *this)
	, device_rs232_port_interface(mconfig, *this)
	, device_sound_interface(mconfig, *this)
	, device_image_interface(mconfig, *this)
	, m_jack(*this, "JACK")
	, m_stream(nullptr)
	, m_reply_timer(nullptr)
	, m_cd_timer(nullptr)
	, m_track(-1)
	, m_status(STATUS_STOP)
	, m_mode(MODE_SINGLE_STOP)
	, m_volume(20)
	, m_eq(0)
	, m_cycle_times(0)
	, m_cycles_done(0)
	, m_interlude(false)
	, m_combination_pos(0)
	, m_jack_plugged(false)
	, m_cassette_level(0)
	, m_lead_in(0)
	, m_file_pos(0)
	, m_frame_pos(0)
	, m_window_count(0)
	, m_sample_count(0)
	, m_sample_index(0)
	, m_channels(2)
	, m_sample_rate(44100)
	, m_is_wav(false)
	, m_wav_format(1)
	, m_wav_channels(2)
	, m_wav_bits(16)
	, m_wav_align(4)
	, m_wav_rate(44100)
	, m_wav_end(0)
{
}

static INPUT_PORTS_START(cocomp3)
	PORT_START("JACK")
	PORT_CONFNAME(0x01, 0x00, "CoCoMP3 Line Out Jack") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(serial_cocomp3_device::jack_changed), 0)
	PORT_CONFSETTING(0x00, "Nothing plugged in (mono to CoCo cassette input)")
	PORT_CONFSETTING(0x01, "Cable plugged in (stereo line out)")
INPUT_PORTS_END

ioport_constructor serial_cocomp3_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(cocomp3);
}

void serial_cocomp3_device::device_add_mconfig(machine_config &config)
{
	// outputs 0/1: stereo line-out jack; output 2: mono cassette cable
	SPEAKER(config, "lineout", 2).front();
	add_route(0, "lineout", 1.0, 0);
	add_route(1, "lineout", 1.0, 1);

	// On a CoCo the cassette cable feeds the MC14529 sound selector's
	// cassette input, so it is heard only when software selects it
	// (AUDIO ON). Elsewhere, let it be heard directly.
	if (dynamic_cast<mc14529_device *>(config.root_device().subdevice("mux")))
		add_route(2, ":mux", CASSETTE_GAIN, mc14529_device::y_sound_input(1));
	else
		add_route(2, "lineout", 0.5);
}

void serial_cocomp3_device::device_start()
{
	m_stream = stream_alloc(0, 3, m_sample_rate);
	m_reply_timer = timer_alloc(FUNC(serial_cocomp3_device::send_reply), this);
	m_cd_timer = timer_alloc(FUNC(serial_cocomp3_device::cd_idle), this);
	m_decoder = std::make_unique<mp3_audio>(m_window.data());

	set_data_frame(1, 8, PARITY_NONE, STOP_BITS_1);
	set_rate(9600);
}

void serial_cocomp3_device::device_reset()
{
	// idle line, inverted (see header comment)
	output_rxd(0);
	output_dcd(1);
	output_dsr(0);
	output_cts(0);
	output_ri(0);

	m_packet.clear();
	m_reply.clear();
	m_last_rx = attotime::zero;

	m_jack_plugged = m_jack->read() & 1;
	m_volume = 20;
	m_mode = MODE_SINGLE_STOP;
	m_eq = 0;
	m_cycle_times = 0;
	m_cycles_done = 0;
	m_interlude = false;
	m_combination.clear();
	m_comb_resume = position{};
	m_hold_until = attotime::zero;
	m_track = -1;
	stop();
	m_cd_timer->adjust(attotime::never);
	output_dcd(1);

	scan_tracks();
	m_track = m_tracks.empty() ? -1 : 0;    // hardware: 0D answers 1 at power-on
}

INPUT_CHANGED_MEMBER(serial_cocomp3_device::jack_changed)
{
	m_stream->update();
	m_jack_plugged = newval & 1;
}


//-------------------------------------------------
//  card image
//-------------------------------------------------

std::pair<std::error_condition, std::string> serial_cocomp3_device::call_load()
{
	if (!m_fs.mount([this] (u64 offset, void *buffer, u32 length) { return read_card(offset, buffer, length); }))
		return std::make_pair(image_error::INVALIDIMAGE, "No FAT12/16/32 or exFAT filesystem found");

	if (started())
	{
		m_stream->update();
		stop();
		scan_tracks();
		m_track = m_tracks.empty() ? -1 : 0;
	}
	return std::make_pair(std::error_condition(), std::string());
}

void serial_cocomp3_device::call_unload()
{
	m_fs.unmount();
	if (started())
	{
		m_stream->update();
		stop();
		scan_tracks();
		m_track = m_tracks.empty() ? -1 : 0;
	}
}

bool serial_cocomp3_device::read_card(u64 offset, void *buffer, u32 length)
{
	if (!is_loaded() || fseek(offset, SEEK_SET))
		return false;
	return fread(buffer, length) == length;
}


//-------------------------------------------------
//  serial
//-------------------------------------------------

void serial_cocomp3_device::tra_callback()
{
	output_rxd(transmit_register_get_data_bit() ? 0 : 1);
}

void serial_cocomp3_device::tra_complete()
{
	if (!m_reply.empty())
		m_reply_timer->adjust(attotime::zero);
}

TIMER_CALLBACK_MEMBER(serial_cocomp3_device::send_reply)
{
	if (!m_reply.empty() && is_transmit_register_empty())
	{
		transmit_register_setup(m_reply.front());
		m_reply.pop_front();
	}
}

void serial_cocomp3_device::rcv_complete()
{
	receive_register_extract();
	u8 const data = get_received_char();

	// drop a partial packet if the line went quiet
	attotime const now = machine().time();
	if (!m_packet.empty() && (now - m_last_rx) > INTERBYTE_TIMEOUT)
		m_packet.clear();
	m_last_rx = now;

	if (m_packet.empty() && data != 0xaa)
		return;

	m_packet.push_back(data);

	if (m_packet.size() >= 3 && m_packet.size() == size_t(m_packet[2]) + 4)
	{
		u8 sum = 0;
		for (size_t i = 0; i < m_packet.size() - 1; i++)
			sum += m_packet[i];

		if (sum == m_packet.back())
			process_packet();
		else
			LOGCMD("checksum error on command %02x\n", m_packet[1]);

		m_packet.clear();
	}
}

void serial_cocomp3_device::reply_byte(u8 cmd, u8 data)
{
	u8 const packet[] = { 0xaa, cmd, 0x01, data };
	u8 sum = 0;
	for (u8 b : packet)
	{
		m_reply.push_back(b);
		sum += b;
	}
	m_reply.push_back(sum);
	send_reply_packet();
}

void serial_cocomp3_device::reply_word(u8 cmd, u16 data)
{
	u8 const packet[] = { 0xaa, cmd, 0x02, u8(data >> 8), u8(data) };
	u8 sum = 0;
	for (u8 b : packet)
	{
		m_reply.push_back(b);
		sum += b;
	}
	m_reply.push_back(sum);
	send_reply_packet();
}

void serial_cocomp3_device::send_reply_packet()
{
	if (busy())
	{
		// hardware: while a WAV of 32 kHz or more plays, replies are cut short
		// (AA, AA 01, AA 01 01 ... were seen)
		size_t const keep = 1 + machine().rand() % 4;
		LOGCMD("busy playing WAV: reply cut to %d bytes\n", int(keep));
		while (m_reply.size() > keep)
			m_reply.pop_back();
	}
	attotime const now = machine().time();
	m_reply_timer->adjust((m_hold_until > now) ? (m_hold_until - now) : attotime::zero);
}

void serial_cocomp3_device::process_packet()
{
	u8 const cmd = m_packet[1];
	u8 const len = m_packet[2];
	u8 const *const data = &m_packet[3];
	int const count = m_tracks.size();

	m_stream->update();

	// hardware: a query sent just after one of these commands is answered
	// only when the new track starts (about 330 ms), or 375 ms after a stop
	switch (cmd)
	{
	case 0x05: case 0x06: case 0x07: case 0x08: case 0x0e: case 0x0f: case 0x16: case 0x17: case 0x1b:
		m_hold_until = machine().time() + HOLD_START;
		break;
	case 0x04:
		m_hold_until = machine().time() + HOLD_STOP;
		break;
	}

	switch (cmd)
	{
	case 0x01: // query play status
		LOGCMD("query status\n");
		reply_byte(cmd, m_status);
		break;

	case 0x02: // play: resume from pause, or start the current track
		LOGCMD("play\n");
		if (m_status == STATUS_PAUSE)
		{
			m_status = STATUS_PLAY;
			set_dcd_playing(true);
		}
		else if (m_status == STATUS_STOP)
			play_track(std::max(m_track, 0));
		else if (!m_combination.empty())
			m_combination.clear();  // hardware: the current file finishes, then single-stop rules
		break;

	case 0x03: // pause
		LOGCMD("pause\n");
		if (m_status == STATUS_PLAY)
		{
			m_status = STATUS_PAUSE;
			m_interlude = false;    // hardware: a paused interlude no longer returns
			set_dcd_playing(false, CD_PAUSE_DELAY);
		}
		break;

	case 0x04: // stop
		LOGCMD("stop\n");
		m_interlude = false;
		m_combination.clear();
		stop(CD_STOP_DELAY);
		break;

	case 0x05: // previous
		LOGCMD("previous\n");
		if (count)
			play_track((m_track <= 0) ? (count - 1) : (m_track - 1));
		break;

	case 0x06: // next
		LOGCMD("next\n");
		if (count)
			play_track((m_track + 1) % count);
		break;

	case 0x07: // play track number
		if (len >= 2)
		{
			int const number = (data[0] << 8) | data[1];
			LOGCMD("play track %d\n", number);
			play_track(number - 1);
		}
		break;

	case 0x08: // play by path
	case 0x17: // interlude by path
		if (len >= 2)
		{
			std::string const path(reinterpret_cast<const char *>(&data[1]), len - 1);
			LOGCMD("%s drive %d path %s\n", (cmd == 0x17) ? "interlude" : "play", data[0], path);
			int const index = find_path(path);
			if (index < 0)
				break;  // hardware: a path that matches nothing changes nothing
			if (cmd == 0x17 && m_status != STATUS_STOP)
			{
				if (!m_interlude)
					m_resume = position{ m_track, play_pos() };
				m_interlude = true;
				start_track(index);
			}
			else
				play_track(index);
		}
		break;

	case 0x09: // query online drives
		LOGCMD("query online drives\n");
		reply_byte(cmd, m_fs.mounted() ? (1 << DRIVE_SD) : 0x00);
		break;

	case 0x0a: // query current drive
		LOGCMD("query current drive\n");
		reply_byte(cmd, DRIVE_SD);
		break;

	case 0x0b: // switch drive: stopped on the first track
		LOGCMD("switch drive %d\n", len ? data[0] : -1);
		stop();
		m_track = count ? 0 : -1;
		break;

	case 0x0c: // query total tracks
		LOGCMD("query total tracks\n");
		reply_word(cmd, count);
		break;

	case 0x0d: // query current track
		LOGCMD("query current track\n");
		reply_word(cmd, m_track + 1);
		break;

	case 0x0e: // previous folder: play its last track
	case 0x0f: // next folder: play its first track
		LOGCMD("%s folder\n", (cmd == 0x0e) ? "previous" : "next");
		if (!m_folders.empty())
		{
			int const nfolders = m_folders.size();
			int f = folder_of(m_track);
			if (f < 0)
				f = (cmd == 0x0e) ? 0 : (nfolders - 1);
			f = (cmd == 0x0e) ? ((f + nfolders - 1) % nfolders) : ((f + 1) % nfolders);
			folder const &fd = m_folders[f];
			play_track((cmd == 0x0e) ? (fd.first + fd.count - 1) : fd.first);
		}
		break;

	case 0x10: // end playing: end an interlude, otherwise stop
		LOGCMD("end playing\n");
		if (m_interlude)
		{
			m_interlude = false;
			if (!start_track(m_resume.track, m_resume.file_pos))
				stop();
		}
		else
			stop();
		break;

	case 0x11: // query first track in current folder
	case 0x12: // query track count in current folder (not subfolders)
		LOGCMD("query folder %02x\n", cmd);
		{
			int const f = folder_of(m_track);
			if (f < 0)
				reply_word(cmd, 0);
			else
				reply_word(cmd, (cmd == 0x11) ? (m_folders[f].first + 1) : m_folders[f].count);
		}
		break;

	case 0x13: // set volume
		if (len >= 1)
		{
			LOGCMD("volume %d\n", data[0]);
			if (data[0] <= MAX_VOLUME)  // hardware: 255 after 0 left it silent, so larger values are ignored
				m_volume = data[0];
		}
		break;

	case 0x14: // volume up
		LOGCMD("volume up\n");
		m_volume = std::min(m_volume + 1, MAX_VOLUME);
		break;

	case 0x15: // volume down
		LOGCMD("volume down\n");
		m_volume = std::max(m_volume - 1, 0);
		break;

	case 0x16: // interlude by track number
		if (len >= 3)
		{
			int const index = ((data[1] << 8) | data[2]) - 1;
			LOGCMD("interlude track %d\n", index + 1);
			if (index < 0 || index >= count)
				break;
			if (m_status == STATUS_STOP)
				play_track(index);
			else
			{
				if (!m_interlude)
					m_resume = position{ m_track, play_pos() };
				m_interlude = true;
				start_track(index);
			}
		}
		break;

	case 0x18: // set loop mode
		if (len >= 1)
		{
			LOGCMD("loop mode %d\n", data[0]);
			m_mode = data[0] & 7;
			m_cycles_done = 0;
		}
		break;

	case 0x19: // set cycle times
		if (len >= 2)
		{
			m_cycle_times = (data[0] << 8) | data[1];
			m_cycles_done = 0;
			LOGCMD("cycle times %d\n", m_cycle_times);
		}
		break;

	case 0x1a: // EQ (not modelled)
		if (len >= 1)
		{
			LOGCMD("EQ %d\n", data[0]);
			m_eq = data[0];
		}
		break;

	// The UART guide says the files live in a folder named DY, but a real
	// module plays them from a folder named ZH (/DY and /XY were ignored).
	// The names are sent as ASCII pairs; binary track numbers play nothing.
	case 0x1b: // combination play: two-character names of files in the ZH folder
		LOGCMD("combination play\n");
		if (m_combination.empty())
			m_comb_resume = (m_status == STATUS_PLAY) ? position{ m_track, play_pos() } : position{};
		m_combination.clear();
		for (int i = 0; i + 1 < len; i += 2)
		{
			std::string const want{ char(data[i]), char(data[i + 1]) };
			for (int t = 0; t < count; t++)
			{
				std::string const &p = m_tracks[t].short_path;
				if (p.size() > 4 && p.compare(0, 4, "/ZH/") == 0 && p.compare(4, 2, want) == 0 && p[6] == '.')
				{
					m_combination.push_back(t);
					break;
				}
			}
		}
		m_combination_pos = 0;
		if (!m_combination.empty())
			start_track(m_combination[0]);
		break;

	case 0x1c: // end combination play, back to what played before (UART guide)
		LOGCMD("end combination play\n");
		if (!m_combination.empty())
		{
			m_combination.clear();
			if (!start_track(m_comb_resume.track, m_comb_resume.file_pos))
				stop();
		}
		break;

	case 0x1f: // select a track without playing it
		if (len >= 2)
		{
			int const index = ((data[0] << 8) | data[1]) - 1;
			LOGCMD("select track %d\n", index + 1);
			stop();
			if (index >= 0 && index < count)
				m_track = index;
		}
		break;

	default:
		LOGCMD("unhandled command %02x length %d\n", cmd, len);
		break;
	}
}


//-------------------------------------------------
//  track list
//-------------------------------------------------

static bool is_audio_file(std::string_view name)
{
	if (name.size() < 4)
		return false;
	std::string_view const ext = name.substr(name.size() - 4);
	return !core_stricmp(ext, ".mp3") || !core_stricmp(ext, ".wav");
}

void serial_cocomp3_device::scan_tracks()
{
	m_tracks.clear();
	m_folders.clear();

	if (m_fs.mounted())
	{
		card_fs::entry root;
		root.cluster = m_fs.root();
		scan_card(root, "", 0);
		LOGFILE("card: %d tracks in %d folders\n", int(m_tracks.size()), int(m_folders.size()));
	}
}

// Numbering measured on hardware: within each folder the MP3 and WAV files
// come first, in directory-entry order (deleted slots skipped, a reused slot
// keeps its place, ._ files counted, other extensions ignored), then each
// subfolder in entry order, depth-first. So the root's files are 1, 2, ...
// and every folder's tracks are one contiguous run. Folders without audio
// files are not folders for 0E/0F.
void serial_cocomp3_device::scan_card(const card_fs::entry &dir, const std::string &path, int depth)
{
	if (depth > 16)
		return;

	std::vector<card_fs::entry> entries = m_fs.list(dir.cluster, dir.size, dir.contiguous);
	int folder_index = -1;
	for (card_fs::entry &e : entries)
	{
		if (e.is_dir || !is_audio_file(e.short_name))
			continue;

		if (folder_index < 0)
		{
			folder_index = m_folders.size();
			m_folders.push_back(folder{ path.empty() ? "/" : path, int(m_tracks.size()), 0 });
		}
		m_folders[folder_index].count++;

		track t;
		t.short_path = path + "/" + e.short_name;
		t.entry = std::move(e);
		t.folder = folder_index;
		m_tracks.push_back(std::move(t));
	}

	for (card_fs::entry &e : entries)
		if (e.is_dir)
			scan_card(e, path + "/" + e.short_name, depth + 1);
}

// One path component against an 8.3 name, by the rules a real module showed:
// the component must be PREFIX*EXT (folders: PREFIX*). PREFIX, compared
// case-sensitively with the start of the name (so commands must be upper
// case), may be empty but not longer than 7 characters: EIGHTCHR and
// LONGFI~1 found nothing. A '.' instead of '*' finds nothing either.
static bool path_match(const std::string &pattern, const std::string &name, bool is_file)
{
	size_t const star = pattern.find('*');
	if (star == std::string::npos || star > 7)
		return false;
	std::string_view const prefix(pattern.data(), star);
	std::string_view const ext(pattern.data() + star + 1, pattern.size() - star - 1);

	size_t const dot = name.find('.');
	std::string_view const base(name.data(), (dot == std::string::npos) ? name.size() : dot);
	std::string_view const name_ext = (dot == std::string::npos) ? std::string_view() : std::string_view(name).substr(dot + 1);

	if (base.substr(0, prefix.size()) != prefix)
		return false;
	if (!is_file)
		return ext.empty();
	return !ext.empty() && ext == name_ext;    // hardware: "NAME*" and "*" alone find nothing
}

static std::vector<std::string> split_path(const std::string &path)
{
	std::vector<std::string> parts;
	size_t start = 0;
	while (start <= path.size())
	{
		size_t end = path.find('/', start);
		if (end == std::string::npos)
			end = path.size();
		if (end > start)
			parts.emplace_back(path.substr(start, end - start));
		start = end + 1;
	}
	return parts;
}

int serial_cocomp3_device::find_path(const std::string &path) const
{
	if (!m_fs.mounted())
		return -1;

	// card: match each component against the 8.3 names, first in entry order
	if (path.empty() || path[0] != '/')
		return -1;  // hardware: the leading slash is required
	std::vector<std::string> const want = split_path(path);
	for (int i = 0; i < int(m_tracks.size()); i++)
	{
		std::vector<std::string> const have = split_path(m_tracks[i].short_path);
		if (have.size() != want.size())
			continue;
		bool ok = true;
		for (size_t c = 0; ok && c < have.size(); c++)
			ok = path_match(want[c], have[c], c + 1 == have.size());
		if (ok)
			return i;
	}
	LOGFILE("no track matches %s\n", path);
	return -1;
}

//-------------------------------------------------
//  playback
//-------------------------------------------------

void serial_cocomp3_device::set_dcd_playing(bool playing, attotime delay)
{
	if (playing)
	{
		m_cd_timer->adjust(attotime::never);
		output_dcd(0);
	}
	else if (delay == attotime::zero)
	{
		m_cd_timer->adjust(attotime::never);
		output_dcd(1);
	}
	else
		m_cd_timer->adjust(delay);
}

TIMER_CALLBACK_MEMBER(serial_cocomp3_device::cd_idle)
{
	if (m_status != STATUS_PLAY)
		output_dcd(1);
}

// a real module garbles its replies while it plays a WAV of 32 kHz or more
// (8 kHz and 22.05 kHz WAVs, mono or stereo, were fine)
bool serial_cocomp3_device::busy() const
{
	return m_status == STATUS_PLAY && m_is_wav && m_wav_rate >= 32000;
}

void serial_cocomp3_device::stop(attotime cd_delay)
{
	m_status = STATUS_STOP;
	m_lead_in = 0;
	set_dcd_playing(false, cd_delay);
	m_file.clear();
	m_file_pos = 0;
	m_window_count = 0;
	m_sample_count = 0;
	m_sample_index = 0;
	m_is_wav = false;
	if (m_decoder)
		m_decoder->clear();
}

bool serial_cocomp3_device::load_track(int index, std::vector<u8> &data)
{
	return m_fs.load(m_tracks[index].entry, data);
}

// Start a track without touching loop, interlude or combination state.
bool serial_cocomp3_device::start_track(int index, size_t file_pos)
{
	stop();
	if (index < 0 || index >= int(m_tracks.size()))
		return false;

	if (!load_track(index, m_file))
	{
		LOGFILE("can't load %s\n", m_tracks[index].short_path);
		m_file.clear();
		return false;
	}

	LOGFILE("playing %d %s\n", index + 1, m_tracks[index].short_path);
	m_track = index;
	m_is_wav = parse_wav();
	if (!m_is_wav)
		m_file_pos = std::min(file_pos, m_file.size());
	else if (file_pos > m_file_pos && m_wav_align)
	{
		// resume on a sample-frame boundary
		size_t const start = m_file_pos;
		m_file_pos = start + (std::min(file_pos, m_wav_end) - start) / m_wav_align * m_wav_align;
	}
	m_status = STATUS_PLAY;
	m_lead_in = START_DELAY;
	set_dcd_playing(true);
	return true;
}

// Start a track chosen by a command: cancels interludes and combinations.
bool serial_cocomp3_device::play_track(int index)
{
	m_interlude = false;
	m_combination.clear();
	m_cycles_done = 0;
	if (!start_track(index))
	{
		stop();
		return false;
	}
	return true;
}

int serial_cocomp3_device::pick_random(int first, int count)
{
	if (count <= 1)
		return first;
	int next;
	do
		next = first + int(machine().rand() % count);
	while (next == m_track);
	return next;
}

// called from the stream update when the current file runs out
void serial_cocomp3_device::track_finished()
{
	int const count = m_tracks.size();

	if (m_interlude)
	{
		m_interlude = false;
		if (!start_track(m_resume.track, m_resume.file_pos))
			stop();
		return;
	}

	if (!m_combination.empty())
	{
		if (++m_combination_pos < m_combination.size() && start_track(m_combination[m_combination_pos]))
			return;
		// hardware: a combination started over a playing track returns to it
		m_combination.clear();
		if (!start_track(m_comb_resume.track, m_comb_resume.file_pos))
			stop();
		return;
	}

	int const f = folder_of(m_track);
	int const ffirst = (f >= 0) ? m_folders[f].first : 0;
	int const fcount = (f >= 0) ? m_folders[f].count : count;
	int next = -1;

	switch (m_mode)
	{
	case MODE_LOOP_ONE:
		if (m_cycle_times && ++m_cycles_done >= m_cycle_times)
			break;
		next = m_track;
		break;

	case MODE_LOOP_ALL:
		if (m_track + 1 >= count && m_cycle_times && ++m_cycles_done >= m_cycle_times)
			break;
		next = (m_track + 1) % count;
		break;

	case MODE_ALL_ONCE:
		if (m_track + 1 < count)
			next = m_track + 1;
		break;

	case MODE_RANDOM:
		next = pick_random(0, count);
		break;

	case MODE_LOOP_FOLDER:
		if (m_track + 1 >= ffirst + fcount && m_cycle_times && ++m_cycles_done >= m_cycle_times)
			break;
		next = (m_track + 1 < ffirst + fcount) ? (m_track + 1) : ffirst;
		break;

	case MODE_RANDOM_FOLDER:
		next = pick_random(ffirst, fcount);
		break;

	case MODE_FOLDER_ONCE:
		if (m_track + 1 < ffirst + fcount)
			next = m_track + 1;
		break;

	default: // MODE_SINGLE_STOP
		break;
	}

	if (next < 0 || !start_track(next))
		stop();
}

bool serial_cocomp3_device::fill_window()
{
	size_t const count = std::min(m_window.size() - m_window_count, m_file.size() - m_file_pos);
	std::copy_n(m_file.begin() + m_file_pos, count, m_window.begin() + m_window_count);
	m_file_pos += count;
	m_window_count += count;
	return count != 0;
}

bool serial_cocomp3_device::decode_frame()
{
	return m_is_wav ? decode_wav_frame() : decode_mp3_frame();
}

bool serial_cocomp3_device::decode_mp3_frame()
{
	// returns false when the file is exhausted
	for (;;)
	{
		fill_window();
		if (m_window_count == 0)
			return false;

		int pos = 0, rate = 0, channels = 0;
		size_t const window_start = m_file_pos - m_window_count;
		bool const ok = m_decoder->decode_buffer(pos, m_window_count, m_samples.data(), m_sample_count, rate, channels);

		if (ok && m_sample_count > 0)
		{
			m_frame_pos = window_start;
			std::copy(m_window.begin() + pos, m_window.begin() + m_window_count, m_window.begin());
			m_window_count -= pos;
			m_sample_index = 0;
			m_channels = channels;
			if (rate != m_sample_rate)
			{
				m_sample_rate = rate;
				machine().scheduler().synchronize(timer_expired_delegate(FUNC(serial_cocomp3_device::update_sample_rate), this));
			}
			return true;
		}

		m_sample_count = 0;
		if (pos > 0)
		{
			// skipped non-audio data (ID3 tags and so on)
			std::copy(m_window.begin() + pos, m_window.begin() + m_window_count, m_window.begin());
			m_window_count -= pos;
		}
		else if (m_file_pos >= m_file.size())
		{
			// nothing decodable left
			return false;
		}
		else
		{
			// no frame sync in a full window: discard a byte and keep looking
			std::copy(m_window.begin() + 1, m_window.begin() + m_window_count, m_window.begin());
			m_window_count--;
		}
	}
}

// RIFF/WAVE: PCM 8/16/24/32-bit or IEEE float, any channel count (the
// first two are played). Leaves m_file_pos at the start of the data.
bool serial_cocomp3_device::parse_wav()
{
	if (m_file.size() < 12 || memcmp(&m_file[0], "RIFF", 4) || memcmp(&m_file[8], "WAVE", 4))
		return false;

	bool have_fmt = false;
	size_t pos = 12;
	while (pos + 8 <= m_file.size())
	{
		u32 const size = get_u32le(&m_file[pos + 4]);
		size_t const body = pos + 8;
		if (!memcmp(&m_file[pos], "fmt ", 4) && size >= 16 && body + 16 <= m_file.size())
		{
			m_wav_format = get_u16le(&m_file[body]);
			m_wav_channels = get_u16le(&m_file[body + 2]);
			m_wav_rate = get_u32le(&m_file[body + 4]);
			m_wav_align = get_u16le(&m_file[body + 12]);
			m_wav_bits = get_u16le(&m_file[body + 14]);
			if (m_wav_format == 0xfffe && size >= 26 && body + 26 <= m_file.size())
				m_wav_format = get_u16le(&m_file[body + 24]);   // extensible: sub-format
			have_fmt = true;
		}
		else if (!memcmp(&m_file[pos], "data", 4) && have_fmt)
		{
			// hardware played 8-bit and 16-bit PCM up to 48 kHz; 24-bit PCM,
			// 32-bit float and 96 kHz were silent (status stop)
			bool const supported = m_wav_channels && m_wav_channels <= 2 && m_wav_rate && m_wav_rate <= 48000
					&& m_wav_align >= m_wav_channels * ((m_wav_bits + 7) / 8)
					&& m_wav_format == 1 && (m_wav_bits == 8 || m_wav_bits == 16);
			if (!supported)
			{
				LOGFILE("unsupported WAV format %d, %d bits, %d channels\n", m_wav_format, m_wav_bits, m_wav_channels);
				m_file_pos = m_file.size();
				m_wav_end = m_file.size();
				return true;    // treat as WAV, plays nothing
			}
			m_file_pos = body;
			m_wav_end = std::min<size_t>(body + size, m_file.size());
			if (int(m_wav_rate) != m_sample_rate)
			{
				m_sample_rate = m_wav_rate;
				machine().scheduler().synchronize(timer_expired_delegate(FUNC(serial_cocomp3_device::update_sample_rate), this));
			}
			return true;
		}
		pos = body + size + (size & 1);
	}
	return false;
}

bool serial_cocomp3_device::decode_wav_frame()
{
	unsigned const bytes = (m_wav_bits + 7) / 8;
	int frames = 0;
	while (frames < 1152 && m_file_pos + m_wav_align <= m_wav_end)
	{
		u8 const *const p = &m_file[m_file_pos];
		for (int ch = 0; ch < 2; ch++)
		{
			u8 const *const s = p + std::min<int>(ch, m_wav_channels - 1) * bytes;
			s16 v;
			if (m_wav_format == 3)
			{
				u32 const bits = get_u32le(s);
				float f;
				memcpy(&f, &bits, sizeof(f));
				v = s16(std::clamp(f, -1.0f, 1.0f) * 32767.0f);
			}
			else
			{
				switch (m_wav_bits)
				{
				case 8:  v = s16((s[0] - 0x80) << 8); break;
				case 16: v = s16(get_u16le(s)); break;
				case 24: v = s16(get_u16le(s + 1)); break;
				default: v = s16(get_u16le(s + 2)); break;
				}
			}
			m_samples[frames * 2 + ch] = v;
		}
		m_file_pos += m_wav_align;
		frames++;
	}
	m_sample_count = frames;
	m_sample_index = 0;
	m_channels = 2;
	return frames != 0;
}

// where an interrupted track resumes: the audio being heard now, not the
// decoder's read-ahead
size_t serial_cocomp3_device::play_pos() const
{
	if (m_is_wav)
		return m_file_pos - size_t(m_sample_count - m_sample_index) * m_wav_align;
	return m_sample_count ? m_frame_pos : m_file_pos;
}

TIMER_CALLBACK_MEMBER(serial_cocomp3_device::update_sample_rate)
{
	m_stream->set_sample_rate(m_sample_rate);
}

void serial_cocomp3_device::sound_stream_update(sound_stream &stream)
{
	// measured: each volume step is 1.61 dB (volume 0 is silent)
	float const gain = m_volume ? float(pow(10.0, VOL_DB_STEP * (m_volume - MAX_VOLUME) / 20.0)) : 0.0f;
	int pos = 0;

	double const period = 1.0 / stream.sample_rate();
	while (pos < stream.samples() && m_status == STATUS_PLAY && m_lead_in > 0)
	{
		m_lead_in -= period;
		stream.put(0, pos, 0);
		stream.put(1, pos, 0);
		stream.put(2, pos, 0);
		m_cassette_level = 0;
		pos++;
	}

	while (pos < stream.samples() && m_status == STATUS_PLAY)
	{
		if (m_sample_index >= m_sample_count)
		{
			if (!decode_frame())
			{
				track_finished();
				continue;
			}

			// don't play this frame at the old rate
			if (m_sample_rate != int(stream.sample_rate()))
				break;
		}

		int const step = std::min(m_channels, 2);
		float const left = gain * m_samples[m_sample_index * step] / 32768.0f;
		float const right = gain * m_samples[m_sample_index * step + step - 1] / 32768.0f;
		if (m_jack_plugged)
		{
			stream.put(0, pos, left);
			stream.put(1, pos, right);
			stream.put(2, pos, 0);
			m_cassette_level = 0;
		}
		else
		{
			m_cassette_level = (left + right) * 0.5f;
			stream.put(0, pos, 0);
			stream.put(1, pos, 0);
			stream.put(2, pos, m_cassette_level);
		}
		m_sample_index++;
		pos++;
	}

	if (pos < stream.samples())
		m_cassette_level = 0;
	for (; pos < stream.samples(); pos++)
	{
		stream.put(0, pos, 0);
		stream.put(1, pos, 0);
		stream.put(2, pos, 0);
	}
}

double serial_cocomp3_device::cassette_input()
{
	m_stream->update();
	// the comparator reads 0 with no signal, and a small offset makes quiet
	// tones read high a little less than half the time
	return m_cassette_level * CASSETTE_GAIN - COMPARATOR_THRESHOLD;
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(SERIAL_COCOMP3, device_rs232_port_interface, serial_cocomp3_device, "serial_cocomp3", "CoCoMP3 Serial MP3 Player")
