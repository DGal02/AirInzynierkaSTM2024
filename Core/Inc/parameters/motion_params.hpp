#pragma once

extern "C"
{
	#include "main.h"
}

struct MotionParams
{
	double posMin = 0.06;
	double posMax = 0.24;
	uint32_t deadZoneRange = 4000;
	uint32_t controllerGain = 3000;
	uint32_t maxPrescaler = 9;
};
