#include <irq_callbacks.hpp>
#include <measurement/encoder_driver.hpp>
#include <control/trajectory_generator.hpp>
#include <control/stepper_controller.hpp>
#include "stm32h7xx_it.h"
#include <cstring>

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
	extern double posAngle;
}
int probe = 0;
EncoderDriver encDriver(&hspi3);
EncoderDriver encDriverB(&hspi2);
TrajectoryGenerator trajGen(1e-4);
StepperController stepperController;

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
	if (isEngineEnabled) {
		stepperController.generateSignal();
	}
//	if (isClocked)
//	{
//		if (cntFreq == frequencyPrescaler)
//		{
//			HAL_GPIO_TogglePin(S_CLK_GPIO_Port, S_CLK_Pin);
//			cntFreq = 0;
//		}
//		else
//		{
//			cntFreq++;
//		}
//	}
}

/* Read Encoder and set control signal */
void TIM5_IRQ_Callback()
{
//	readRequest();
	encDriver.readRequest();
//	encDriverB.readRequestB();
//	SPI3_ReceiveCompleteCallback();
//	readEncoder();
//	trajectoryGenerator();
//	controller(desiredPos);
//	engineDirectionControl();
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
    if (__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_OVR) != RESET) {
        __HAL_SPI_CLEAR_OVRFLAG(&hspi3);
    }
}

void SPI2_IRQHandler(void) {
    HAL_SPI_IRQHandler(&hspi2);
    if (__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_OVR) != RESET) {
        __HAL_SPI_CLEAR_OVRFLAG(&hspi2);
    }
}

void SPI3_ReceiveCompleteCallback()
{
	uint32_t pos = encDriver.readEncoder();
//	uint32_t desPos = trajGen.calc();
	if (isFetching) {
		push(&dataA, pos);
	}
//	stepperController.calcInput(desPos, pos);
}

void SPI2_ReceiveCompleteCallback()
{
	uint32_t pos = encDriverB.readEncoderB();
//	uint32_t desPos = trajGen.calc();
	if (isFetching) {
		push(&dataB, pos);
	}
//	stepperController.calcInput(desPos, pos);
}
