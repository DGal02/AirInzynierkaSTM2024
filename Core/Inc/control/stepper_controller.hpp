#pragma once

extern "C"
{
	#include "main.h"
	extern uint8_t isClocked;
	extern uint8_t frequencyPrescaler;
	extern uint8_t directionToSet;
	extern uint32_t cntFreq;
	extern int posiError;
}

#include <parameters/motion_params.hpp>

enum class Direction
{
	Forward = 0,
	Backward = 1
};

class StepperController
{
private:
	MotionParams motionParams;

	Direction direction = Direction::Forward;
	Direction currentDirection = Direction::Forward;

	bool isSignalGenerated = false;
	uint32_t u_sat;
	uint32_t counter;
	uint32_t prescaler;

public:
	int32_t posError = 0;

	StepperController()
	{

	}

	void calcInput(uint32_t desPos, uint32_t pos)
	{
		double u;
		uint32_t u_floor;

		posError = desPos - pos;
		posiError = posError;

		if ((uint32_t)abs(posError) < motionParams.deadZoneRange)
		{
			isSignalGenerated = false;
			isClocked = 0; //debug
		}
		else
		{
			isSignalGenerated = true;
			isClocked = 1; //debug
		}

		if (posError > 0)
		{
			direction = Direction::Forward;
			directionToSet = 0; //debug
		}
		else
		{
			direction = Direction::Backward;
			directionToSet = 1; //debug
		}

		if (posError == 0)
			u = motionParams.maxPrescaler;
		else
			u = (double)motionParams.controllerGain / abs(posError);

		u_floor = (uint32_t)floor(u);
		u_sat = MIN(u_floor, motionParams.maxPrescaler);

		if (u_sat < prescaler)
			counter = MIN(counter, u_sat);

		prescaler = u_sat;
		frequencyPrescaler = u_sat; //debug

		setDirection();
	}

	void generateSignal()
	{
		if (isSignalGenerated)
		{
			if (counter == prescaler)
			{
				HAL_GPIO_TogglePin(S_CLK_GPIO_Port, S_CLK_Pin);
				counter = 0;
				cntFreq = 0; //debug
			}
			else
			{
				counter++;
				cntFreq = counter; //debug
			}
		}
	}

	void setDirection()
	{
		if (direction != currentDirection)
		{
			HAL_GPIO_TogglePin(S_DIR_GPIO_Port, S_DIR_Pin);
			currentDirection = direction;
		}
	}
};
