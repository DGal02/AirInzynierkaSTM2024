#pragma once

extern "C"
{
	#include "main.h"
}

#include <signal_processing/cordic_32bit.h>

class SinGenerator
{
private:
	float Ts;
	uint32_t counter = 0;
	uint32_t amp = 0;
	float omega = 0;
	uint32_t pos0 = 0;

public:
	SinGenerator(float Ts): Ts(Ts)
	{

	}

	void initialize(uint32_t pos0, uint32_t amp, float freq)
	{
		this->pos0 = pos0;
		this->amp = amp;
		omega = 2 * M_PI * freq;

		counter = 0;
	}

	uint32_t calc()
	{
		float sinVal;

		counter++;
		float t = Ts * counter;
		sinCordic(omega * t, &sinVal);

		uint32_t pos = pos0 + (int32_t)(amp * sinVal);
		return pos;
	}
};
