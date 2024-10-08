#pragma once

extern "C"
{
	#include "main.h"
}

#include <measurement/encoder_data.hpp>

class BissCrc6
{
private:
	static uint8_t tableCRC6[64];
	CrcFrame dataNoCrc;
public:
	BissCrc6();
	uint8_t calcCrc_raw(uint64_t data);
	uint8_t calcCrc(EncFrame data);
	bool checkCrc(EncFrame data);
};
