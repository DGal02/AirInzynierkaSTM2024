#pragma once

#ifdef __cplusplus

extern "C"  void TIM4_IRQ_Callback();
extern "C"  void TIM5_IRQ_Callback();
extern "C"  void SPI3_ReceiveCompleteCallback();
extern "C"  void SPI2_ReceiveCompleteCallback();

#else

void TIM4_IRQ_Callback();
void TIM5_IRQ_Callback();
void SPI3_ReceiveCompleteCallback();
void SPI2_ReceiveCompleteCallback();

#endif
