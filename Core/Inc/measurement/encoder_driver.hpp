#pragma once

extern "C"
{
	#include "main.h"
}

#include <measurement/encoder_data.hpp>
#include <crc/bissCrc6.hpp>
#include <parameters/encoder_params.hpp>

class EncoderDriver
{
public:
	static const uint8_t dataBuffLen = 30;
	static const uint8_t frameBuffLen = 5;
	//	uint8_t * dataBuff;
	uint8_t dataBuff[dataBuffLen];
	uint8_t frameBuff[8];

	SPI_HandleTypeDef * spi;
	EncoderParams encParams;

	BissCrc6 bissCrc;

	bool isAckDetected = 0;
	bool isStartDeteted = 0;

	uint8_t startIndex = 0;
	uint8_t shift = 0;

	bool isTransfer = false;
	EncFrame dataWithCrc;
	bool isCrcOk = 0;

	double posRaw = 0;
	double posCalibrated = 0;

	EncoderDriver(SPI_HandleTypeDef * spi);

	void checkAck();
	void getStartIndex();
	void getShift();
	void assignData();
	void calcPos();
	void readRequest();
//	double readEncoder();
	uint32_t readEncoder();
};
