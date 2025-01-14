#include <irq_callbacks.hpp>
#include <measurement/encoder_driver.hpp>
#include <control/trajectory_generator.hpp>
#include <control/stepper_controller.hpp>
#include "stm32h7xx_it.h"
#include <cstring>
#include <math.h>

#define SAMPLING_RATE 10000
#define DURATION 10
#define MIN_VALUE 20000000
#define MAX_VALUE 21000000
#define PI 3.14159265358979323846

extern "C"
{
	#include <main.h>

	extern TIM_HandleTypeDef htim4;
	extern TIM_HandleTypeDef htim5;

	extern SPI_HandleTypeDef hspi2;
	extern SPI_HandleTypeDef hspi3;

	extern uint8_t frequencyPrescaler;
	extern uint32_t cntFreq;
	extern uint8_t isClocked;
	extern uint32_t desiredPos;
	extern uint32_t desiredPosB;
	extern double posAngle;
}
int probe = 0;
EncoderDriver encDriver(&hspi3);
EncoderDriver encDriverB(&hspi2);
TrajectoryGenerator trajGen(1e-4);
StepperController stepperController;
StepperController stepperControllerB;
uint8_t probeA = 0;
uint8_t probeB = 0;
uint32_t posA = 0;
uint32_t posB = 0;
uint8_t probeTimer = 0;

double frequency = 1.0 / DURATION;
double amplitude = (MAX_VALUE - MIN_VALUE) / 2.0;
double offset = (MAX_VALUE + MIN_VALUE) / 2.0;
int totalSamples = SAMPLING_RATE * DURATION;
int halfPeriodSamples = SAMPLING_RATE * (DURATION / 2.0);

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
//{
//	if (htim == &htim4) {
//		TIM4_IRQ_Callback();
//	}
//	if (htim->Instance == TIM6) {
//	    HAL_IncTick();
//	}
//}

void TIM5_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim5, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
            TIM5_IRQ_Callback();
        }
    }
}

void TIM4_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
            TIM4_IRQ_Callback();
        }
    }
}

/* Set clock signal to control stepper motor */
void TIM4_IRQ_Callback()
{


}

/* Read Encoder and set control signal */
void TIM5_IRQ_Callback()
{
	if (mode == 1) {
	    uint32_t posSin = amplitude * sin(2.0 * PI * frequency * probe / SAMPLING_RATE) + offset;
        desiredPos = posSin;
        desiredPosB = posSin + 10000000;
	} else if (mode == 2) {
		 if (probe < halfPeriodSamples) {
			desiredPos = MAX_VALUE;
			desiredPosB = MAX_VALUE + 10000000;
		} else {
			desiredPos = MIN_VALUE;
			desiredPosB = MIN_VALUE + 10000000;
		}
	}

	probe++;
	if (probe >= totalSamples) {
		probe = 0;
	}

	probeTimer++;
	if (probeTimer >= 5) {
		encDriver.readRequest();
		encDriverB.readRequestB();
		probeTimer = 0;
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi3)
	{
		SPI3_ReceiveCompleteCallback();
	}

	if (hspi == &hspi2) {
		SPI2_ReceiveCompleteCallback();
	}
}

void SPI3_IRQHandler(void) {
    HAL_SPI_IRQHandler(&hspi3);
}

void SPI2_IRQHandler(void) {
    HAL_SPI_IRQHandler(&hspi2);
}

void SPI3_ReceiveCompleteCallback()
{
	posA = encDriver.readEncoder();
	probeA++;
	if (isFetching && probeA >= 2) {
		push(&dataA, posA);
		pushError(&dataErrorA, desiredPos - posA);
		probeA = 0;
	}

	stepperController.calcInput(desiredPos, posA);
	if (isEngineEnabled) {
		stepperController.generateSignal();
	}
}

void SPI2_ReceiveCompleteCallback()
{
	posB = encDriverB.readEncoderB();
	probeB++;
	if (isFetching && probeB >= 2) {
		push(&dataB, posB);
		pushError(&dataErrorB, desiredPosB - posB);
		probeB = 0;
	}
	stepperControllerB.calcInputB(desiredPosB, posB);
	if (isEngineEnabled) {
		stepperControllerB.generateSignalB();
	}
}
