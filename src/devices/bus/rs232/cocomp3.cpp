// license:BSD-3-Clause
// copyright-holders:tim lindner
/***************************************************************************

    CoCoMP3 serial MP3 player

    A JQ8400-family MP3 player module that plugs into the Color Computer's
    bit-banger serial port. It plays MP3 files from its own SD card.

    Serial format is 9600 baud, 8N1. Commands and replies use the
    packet format:

        AA cmd len data... sum

    where sum is the low byte of the sum of all preceding bytes.

    The SD card is emulated by a host directory: "cocomp3" inside any
    directory on the sample path. For example, Lad vs Sewer Slimes wants
    samples/cocomp3/LAD/00001.mp3 and so on.

    The CoCo software seen so far (Lad vs Sewer Slimes) expects the receive
    data at PIA1 PB0 to be inverted compared to MAME's RS-232 convention:
    idle low, start bit high. This device drives its output that way.

    TODO:
    - WAV playback
    - EQ, random and folder loop modes
    - save state support for an MP3 that is playing

***************************************************************************/

#include "emu.h"
#include "cocomp3.h"

#include "diserial.h"
#include "speaker.h"

#include "sound/mp3_audio.h"

#include "emuopts.h"
#include "fileio.h"

#include "corestr.h"
#include "osdfile.h"

#include <algorithm>
#include <deque>

#define LOG_COMMAND (1U << 1)
#define LOG_FILE    (1U << 2)

//#define VERBOSE (LOG_COMMAND | LOG_FILE)
#include "logmacro.h"

#define LOGCMD(...)  LOGMASKED(LOG_COMMAND, __VA_ARGS__)
#define LOGFILE(...) LOGMASKED(LOG_FILE, __VA_ARGS__)


namespace {

class serial_cocomp3_device : public device_t,
		public device_serial_interface,
		public device_rs232_port_interface,
		public device_sound_interface
{
public:
	serial_cocomp3_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void input_txd(int state) override { device_serial_interface::rx_w(state); }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

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
		LOOP_ALL = 0,
		LOOP_SINGLE = 1,
		SINGLE_STOP = 2,
		SEQUENTIAL = 7
	};

	enum : u8
	{
		DRIVE_USB = 0,
		DRIVE_SD = 1,
		DRIVE_FLASH = 2
	};

	static constexpr int MAX_VOLUME = 30;
	static constexpr attotime INTERBYTE_TIMEOUT = attotime::from_msec(20);

	TIMER_CALLBACK_MEMBER(send_reply);
	TIMER_CALLBACK_MEMBER(update_sample_rate);

	void process_packet();
	void reply_byte(u8 cmd, u8 data);
	void reply_word(u8 cmd, u16 data);

	std::string find_root() const;
	void scan_tracks();
	std::string resolve_path(const std::string &path) const;
	bool play_file(const std::string &filename);
	bool play_track(int index);
	void stop();
	void track_finished();

	bool fill_window();
	bool decode_frame();

	sound_stream *m_stream;
	emu_timer *m_reply_timer;

	// receive packet parser
	std::vector<u8> m_packet;
	attotime m_last_rx;

	std::deque<u8> m_reply;

	// player state
	std::string m_root;
	std::vector<std::string> m_tracks;
	int m_track;
	u8 m_status;
	u8 m_loop_mode;
	int m_volume;

	// MP3 decoder
	std::vector<u8> m_file;
	size_t m_file_pos;
	std::array<u8, 0x2000> m_window;
	int m_window_count;
	std::unique_ptr<mp3_audio> m_decoder;
	std::array<s16, 1152 * 2> m_samples;
	int m_sample_count;
	int m_sample_index;
	int m_channels;
	int m_sample_rate;
};


serial_cocomp3_device::serial_cocomp3_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SERIAL_COCOMP3, tag, owner, clock)
	, device_serial_interface(mconfig, *this)
	, device_rs232_port_interface(mconfig, *this)
	, device_sound_interface(mconfig, *this)
	, m_stream(nullptr)
	, m_reply_timer(nullptr)
	, m_track(-1)
	, m_status(STATUS_STOP)
	, m_loop_mode(SINGLE_STOP)
	, m_volume(20)
	, m_file_pos(0)
	, m_window_count(0)
	, m_sample_count(0)
	, m_sample_index(0)
	, m_channels(2)
	, m_sample_rate(44100)
{
}

void serial_cocomp3_device::device_add_mconfig(machine_config &config)
{
	SPEAKER(config, "speaker", 2).front();
	add_route(0, "speaker", 1.0, 0);
	add_route(1, "speaker", 1.0, 1);
}

void serial_cocomp3_device::device_start()
{
	m_stream = stream_alloc(0, 2, m_sample_rate);
	m_reply_timer = timer_alloc(FUNC(serial_cocomp3_device::send_reply), this);
	m_decoder = std::make_unique<mp3_audio>(m_window.data());

	set_data_frame(1, 8, PARITY_NONE, STOP_BITS_1);
	set_rate(9600);
}

void serial_cocomp3_device::device_reset()
{
	// idle line, inverted (see header comment)
	output_rxd(0);
	output_dcd(0);
	output_dsr(0);
	output_cts(0);
	output_ri(0);

	m_packet.clear();
	m_reply.clear();
	m_last_rx = attotime::zero;

	m_volume = 20;
	m_loop_mode = SINGLE_STOP;
	stop();

	scan_tracks();
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
	m_reply_timer->adjust(attotime::from_hz(9600));
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
	m_reply_timer->adjust(attotime::from_hz(9600));
}

void serial_cocomp3_device::process_packet()
{
	u8 const cmd = m_packet[1];
	u8 const len = m_packet[2];
	u8 const *const data = &m_packet[3];

	switch (cmd)
	{
	case 0x01: // query play status
		LOGCMD("query status\n");
		reply_byte(cmd, m_status);
		break;

	case 0x02: // play
		LOGCMD("play\n");
		if (m_status == STATUS_PAUSE)
			m_status = STATUS_PLAY;
		else if (m_status == STATUS_STOP)
			play_track(std::max(m_track, 0));
		break;

	case 0x03: // pause
		LOGCMD("pause\n");
		if (m_status == STATUS_PLAY)
		{
			m_stream->update();
			m_status = STATUS_PAUSE;
		}
		break;

	case 0x04: // stop
		LOGCMD("stop\n");
		m_stream->update();
		stop();
		break;

	case 0x05: // previous
		LOGCMD("previous\n");
		if (!m_tracks.empty())
			play_track((m_track <= 0) ? (m_tracks.size() - 1) : (m_track - 1));
		break;

	case 0x06: // next
		LOGCMD("next\n");
		if (!m_tracks.empty())
			play_track((m_track + 1) % m_tracks.size());
		break;

	case 0x07: // play track number
		if (len >= 2)
		{
			int const track = (data[0] << 8) | data[1];
			LOGCMD("play track %d\n", track);
			play_track(track - 1);
		}
		break;

	case 0x08: // play by path
	case 0x17: // insert play by path
		if (len >= 2)
		{
			std::string const path(reinterpret_cast<const char *>(&data[1]), len - 1);
			LOGCMD("play drive %d path %s\n", data[0], path);
			m_stream->update();
			if (!play_file(resolve_path(path)))
				stop();
		}
		break;

	case 0x09: // query online drives
		LOGCMD("query online drives\n");
		reply_byte(cmd, m_root.empty() ? 0x00 : (1 << DRIVE_SD));
		break;

	case 0x0a: // query current drive
		LOGCMD("query current drive\n");
		reply_byte(cmd, DRIVE_SD);
		break;

	case 0x0b: // switch drive
		LOGCMD("switch drive %d\n", len ? data[0] : -1);
		break;

	case 0x0c: // query total tracks
		LOGCMD("query total tracks\n");
		reply_word(cmd, m_tracks.size());
		break;

	case 0x0d: // query current track
		LOGCMD("query current track\n");
		reply_word(cmd, m_track + 1);
		break;

	case 0x11: // query first track in current folder
	case 0x12: // query track count in current folder
		LOGCMD("query folder %02x\n", cmd);
		reply_word(cmd, m_tracks.empty() ? 0 : 1);
		break;

	case 0x13: // set volume
		if (len >= 1)
		{
			LOGCMD("volume %d\n", data[0]);
			m_stream->update();
			m_volume = std::min<int>(data[0], MAX_VOLUME);
		}
		break;

	case 0x14: // volume up
		LOGCMD("volume up\n");
		m_stream->update();
		m_volume = std::min(m_volume + 1, MAX_VOLUME);
		break;

	case 0x15: // volume down
		LOGCMD("volume down\n");
		m_stream->update();
		m_volume = std::max(m_volume - 1, 0);
		break;

	case 0x18: // set loop mode
		if (len >= 1)
		{
			LOGCMD("loop mode %d\n", data[0]);
			m_loop_mode = data[0];
		}
		break;

	default:
		LOGCMD("unhandled command %02x length %d\n", cmd, len);
		break;
	}
}


//-------------------------------------------------
//  SD card (host directory)
//-------------------------------------------------

std::string serial_cocomp3_device::find_root() const
{
	path_iterator iter(machine().options().sample_path());
	std::string dir;
	while (iter.next(dir))
	{
		std::string root = dir + PATH_SEPARATOR "cocomp3";
		if (osd::directory::open(root))
			return root;
	}
	return std::string();
}

static bool is_audio_file(std::string_view name)
{
	return (name.size() >= 4) && !core_stricmp(name.substr(name.size() - 4), ".mp3");
}

void serial_cocomp3_device::scan_tracks()
{
	m_root = find_root();
	m_tracks.clear();
	if (m_root.empty())
	{
		LOGFILE("no cocomp3 directory found on sample path\n");
		return;
	}

	std::vector<std::string> dirs { m_root };
	while (!dirs.empty())
	{
		std::string const dir = dirs.back();
		dirs.pop_back();

		osd::directory::ptr d = osd::directory::open(dir);
		if (!d)
			continue;

		while (const osd::directory::entry *entry = d->read())
		{
			if (entry->name[0] == '.')
				continue;
			std::string const full = dir + PATH_SEPARATOR + entry->name;
			if (entry->type == osd::directory::entry::entry_type::DIR)
				dirs.push_back(full);
			else if (entry->type == osd::directory::entry::entry_type::FILE && is_audio_file(entry->name))
				m_tracks.push_back(full);
		}
	}

	std::sort(m_tracks.begin(), m_tracks.end());
	LOGFILE("%s: %d tracks\n", m_root, int(m_tracks.size()));
}

// case-insensitive match with * and ?
static bool wildcard_match(const char *pattern, const char *name)
{
	while (*pattern)
	{
		if (*pattern == '*')
		{
			while (*pattern == '*')
				pattern++;
			if (!*pattern)
				return true;
			for (const char *n = name; *n; n++)
				if (wildcard_match(pattern, n))
					return true;
			return false;
		}
		if (!*name)
			return false;
		if (*pattern != '?' && tolower(u8(*pattern)) != tolower(u8(*name)))
			return false;
		pattern++;
		name++;
	}
	return !*name;
}

std::string serial_cocomp3_device::resolve_path(const std::string &path) const
{
	if (m_root.empty())
		return std::string();

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

	std::string current = m_root;
	for (size_t i = 0; i < parts.size(); i++)
	{
		bool const last = (i == parts.size() - 1);
		osd::directory::ptr d = osd::directory::open(current);
		if (!d)
			return std::string();

		std::vector<std::string> matches;
		while (const osd::directory::entry *entry = d->read())
		{
			auto const want = last ? osd::directory::entry::entry_type::FILE : osd::directory::entry::entry_type::DIR;
			if (entry->type == want && entry->name[0] != '.' && wildcard_match(parts[i].c_str(), entry->name))
				matches.emplace_back(entry->name);
		}
		if (matches.empty())
		{
			LOGFILE("%s: no match for %s\n", current, parts[i]);
			return std::string();
		}
		std::sort(matches.begin(), matches.end());
		current += PATH_SEPARATOR + matches.front();
	}
	return current;
}


//-------------------------------------------------
//  playback
//-------------------------------------------------

void serial_cocomp3_device::stop()
{
	m_status = STATUS_STOP;
	m_file.clear();
	m_file_pos = 0;
	m_window_count = 0;
	m_sample_count = 0;
	m_sample_index = 0;
	if (m_decoder)
		m_decoder->clear();
}

bool serial_cocomp3_device::play_file(const std::string &filename)
{
	stop();
	if (filename.empty())
		return false;

	if (util::core_file::load(filename, m_file))
	{
		LOGFILE("can't load %s\n", filename);
		m_file.clear();
		return false;
	}

	LOGFILE("playing %s\n", filename);
	auto const found = std::find(m_tracks.begin(), m_tracks.end(), filename);
	m_track = (found != m_tracks.end()) ? (found - m_tracks.begin()) : -1;
	m_status = STATUS_PLAY;
	return true;
}

bool serial_cocomp3_device::play_track(int index)
{
	m_stream->update();
	if (index < 0 || index >= int(m_tracks.size()))
	{
		stop();
		return false;
	}
	if (!play_file(m_tracks[index]))
		return false;
	m_track = index;
	return true;
}

void serial_cocomp3_device::track_finished()
{
	// called from the stream update; restart the same file or move on
	switch (m_loop_mode)
	{
	case LOOP_SINGLE:
		m_file_pos = 0;
		m_window_count = 0;
		m_decoder->clear();
		break;

	case LOOP_ALL:
	case SEQUENTIAL:
		if (m_track >= 0 && !m_tracks.empty() && (m_loop_mode == LOOP_ALL || m_track + 1 < int(m_tracks.size())))
		{
			int const next = (m_track + 1) % m_tracks.size();
			if (play_file(m_tracks[next]))
			{
				m_track = next;
				break;
			}
		}
		stop();
		break;

	default:
		stop();
		break;
	}
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
	// returns false when the file is exhausted
	for (;;)
	{
		fill_window();
		if (m_window_count == 0)
			return false;

		int pos = 0, rate = 0, channels = 0;
		bool const ok = m_decoder->decode_buffer(pos, m_window_count, m_samples.data(), m_sample_count, rate, channels);

		if (ok && m_sample_count > 0)
		{
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

TIMER_CALLBACK_MEMBER(serial_cocomp3_device::update_sample_rate)
{
	m_stream->set_sample_rate(m_sample_rate);
}

void serial_cocomp3_device::sound_stream_update(sound_stream &stream)
{
	float const gain = float(m_volume) / MAX_VOLUME;
	int pos = 0;

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
		s16 const left = m_samples[m_sample_index * step];
		s16 const right = m_samples[m_sample_index * step + step - 1];
		stream.put(0, pos, gain * left / 32768.0f);
		stream.put(1, pos, gain * right / 32768.0f);
		m_sample_index++;
		pos++;
	}

	for (; pos < stream.samples(); pos++)
	{
		stream.put(0, pos, 0);
		stream.put(1, pos, 0);
	}
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(SERIAL_COCOMP3, device_rs232_port_interface, serial_cocomp3_device, "serial_cocomp3", "CoCoMP3 Serial MP3 Player")
