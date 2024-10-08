#include <irq_callbacks.hpp>
#include <measurement/encoder_driver.hpp>
#include <control/trajectory_generator.hpp>
#include <control/stepper_controller.hpp>

extern "C"
{
	#include <main.h>

	extern TIM_HandleTypeDef htim4;
	extern TIM_HandleTypeDef htim5;

	extern SPI_HandleTypeDef hspi3;

	extern uint8_t frequencyPrescaler;
	extern uint32_t cntFreq;
	extern uint8_t isClocked;
	extern uint32_t desiredPos;
}

EncoderDriver encDriver(&hspi3);
TrajectoryGenerator trajGen(1e-4);
StepperController stepperController;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	if (htim == &htim4) {
		TIM4_IRQ_Callback();
	}
	else if (htim == &htim5) {
		TIM5_IRQ_Callback();
	} else if (htim->Instance == TIM6) {
	    HAL_IncTick();
	}
}

/* Set clock signal to control stepper motor */
void TIM4_IRQ_Callback()
{
	stepperController.generateSignal();
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
}

void SPI3_ReceiveCompleteCallback()
{
	uint32_t pos = encDriver.readEncoder();
	uint32_t desPos = trajGen.calc();
	stepperController.calcInput(desPos, pos);
}
