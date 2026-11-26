// license:BSD-3-Clause
// copyright-holders:tim lindner
#ifndef MAME_BUS_RS232_COCOMP3_H
#define MAME_BUS_RS232_COCOMP3_H

#pragma once

#include "rs232.h"


DECLARE_DEVICE_TYPE(SERIAL_COCOMP3, device_rs232_port_interface)


// The CoCoMP3 has a second cable that carries its audio into the CoCo's
// cassette input. A driver whose cassette comparator should see that signal
// can look for this interface on the card plugged into its RS-232 port.
class cocomp3_cassette_interface
{
public:
	virtual ~cocomp3_cassette_interface() = default;

	// instantaneous level fed to the cassette input (silent while the
	// line-out jack has a cable plugged in)
	virtual double cassette_input() = 0;
};

#endif // MAME_BUS_RS232_COCOMP3_H
