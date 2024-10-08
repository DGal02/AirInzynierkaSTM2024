#pragma once

#ifdef __cplusplus

extern "C"
{
	#include "main.h"
}

extern "C" uint8_t calcCrc(uint64_t data);

#else

uint8_t calcCrc(uint64_t data);

#endif
