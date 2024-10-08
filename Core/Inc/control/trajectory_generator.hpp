#pragma once

extern "C"
{
	#include "main.h"
	extern uint32_t desiredPos;
	extern uint8_t trajMode;
	extern uint8_t isTrajInit;
}

#include <control/sin_generator.hpp>

enum class TrajMode
{
	Point = 0,
	Sinus = 1
};

class TrajectoryGenerator
{
private:
	float Ts;
	SinGenerator sinGen = SinGenerator(Ts);
	bool isInitialized = false;

public:
	TrajMode trajectoryMode = TrajMode::Point;
	uint32_t pos = 0;

	TrajectoryGenerator(float Ts): Ts(Ts)
	{

	}

	uint32_t calc()
	{
		trajectoryMode = (TrajMode)trajMode;
		isInitialized = isTrajInit;

		if (!isInitialized)
		{
			switch(trajectoryMode)
			{
			case TrajMode::Point:
				break;
			case TrajMode::Sinus:
				sinGen.initialize(desiredPos, 8000, 0.1);
				break;
			}
			isInitialized = true;
			isTrajInit = 1;
		}

		if (isInitialized)
		{
			switch(trajectoryMode)
			{
			case TrajMode::Point:
				pos = desiredPos;
				break;
			case TrajMode::Sinus:
				pos = sinGen.calc();
				break;
			}
		}
		return pos;
	}
};
