#pragma once

#ifdef __cplusplus

extern "C"
{
	#include "main.h"
}

union EncFrame
{
	uint64_t all;
	struct
	{
		uint64_t crc:6;
		uint64_t warn:1;
		uint64_t err:1;
		uint64_t pos:32;
		uint64_t res: 24;
	} bit;
};

union CrcFrame
{
	uint64_t all;
	struct
	{
        uint64_t warn:1;
        uint64_t err:1;
        uint64_t pos:32;
		uint64_t res: 30;
	} bit;
};

#else
	#include "main.h"

	union EncFrame
	{
		uint64_t all;
		struct
		{
			uint64_t crc:6;
			uint64_t warn:1;
			uint64_t err:1;
			uint64_t pos:32;
			uint64_t res: 24;
		} bit;
	};

	union CrcFrame
	{
		uint64_t all;
		struct
		{
			uint64_t warn:1;
			uint64_t err:1;
			uint64_t pos:32;
			uint64_t res: 30;
		} bit;
};
#endif
