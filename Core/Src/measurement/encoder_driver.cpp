#include <measurement/encoder_driver.hpp>

#define BUF_POS_LEN 100000

extern "C"
{
	extern int pos[BUF_POS_LEN];
	extern uint32_t cntP;
	extern union EncFrame dataEnc;
	extern uint8_t crcOk;
	extern double posAngle;
	extern uint32_t cntErr;
	extern uint8_t buff[30];
}

uint8_t EncoderDriver::dataBuff[dataBuffLen] = {0};
uint8_t EncoderDriver::frameBuff[8] = {0};

uint8_t EncoderDriver::dataBuffB[dataBuffLen] = {0};
uint8_t EncoderDriver::frameBuffB[8] = {0};

EncoderDriver::EncoderDriver(SPI_HandleTypeDef * spi): spi(spi)
{

}

/* check if ACK is correct in first byte (1110 0000) */
void EncoderDriver::checkAck()
{
	isAckDetected = false;

	for (int i = 0; i < 8; i++)
	{
		uint8_t check = (dataBuff[0] << i) ^ 0x80;
		if (check == 0x00)
		{
			isAckDetected = true;
			break;
		}
	}
//	if (dataBuff[0] == 0xE0)
//		isAckDetected = true;
//	else
//		isAckDetected = false;
}

void EncoderDriver::checkAckB()
{
	isAckDetected = false;

	for (int i = 0; i < 8; i++)
	{
		uint8_t check = (dataBuffB[0] << i) ^ 0x80;
		if (check == 0x00)
		{
			isAckDetected = true;
			break;
		}
	}
}

/* check from which byte we have start (10) */
void EncoderDriver::getStartIndex()
{
	for (int i = 1; i < dataBuffLen; i++)
	{
		if (dataBuff[i] != 0x00)
		{
			startIndex = i;
			break;
		}
	}
}

void EncoderDriver::getStartIndexB()
{
	for (int i = 1; i < dataBuffLen; i++)
	{
		if (dataBuffB[i] != 0x00)
		{
			startIndex = i;
			break;
		}
	}
}

/* get bit shift value and check if start (10) is correct */
void EncoderDriver::getShift()
{
	for (int i = 0; i < 8; i++)
	{
		if (((dataBuff[startIndex] << i) & 0x80) == 0x80)
		{
			 if ((((dataBuff[startIndex] << i) | (dataBuff[startIndex+1] >> (8-i))) & 0xC0) == 0x80)
				 isStartDeteted = 1;

			 shift = i + 2;

			 if (shift > 7)
			 {
				 startIndex++;
				 shift = shift - 8;
			 }

			 break;
		}
	}
}

void EncoderDriver::getShiftB()
{
	for (int i = 0; i < 8; i++)
	{
		if (((dataBuffB[startIndex] << i) & 0x80) == 0x80)
		{
			 if ((((dataBuffB[startIndex] << i) | (dataBuffB[startIndex+1] >> (8-i))) & 0xC0) == 0x80)
				 isStartDeteted = 1;

			 shift = i + 2;

			 if (shift > 7)
			 {
				 startIndex++;
				 shift = shift - 8;
			 }

			 break;
		}
	}
}

/* shift data and assign data to structure (crc warn err pos) */
void EncoderDriver::assignData()
{
	for (int i = 0; i < frameBuffLen; i++)
	{
		int j = startIndex + i;
		frameBuff[frameBuffLen - 1 - i] = (dataBuff[j] << shift) | (dataBuff[j+1] >> (8-shift));
	}

	dataWithCrc = *((union EncFrame*)(frameBuff));
	dataEnc.all = dataWithCrc.all; //debug
}

void EncoderDriver::assignDataB()
{
	for (int i = 0; i < frameBuffLen; i++)
	{
		int j = startIndex + i;
		frameBuffB[frameBuffLen - 1 - i] = (dataBuffB[j] << shift) | (dataBuffB[j+1] >> (8-shift));
	}

	dataWithCrc = *((union EncFrame*)(frameBuffB));
	dataEnc.all = dataWithCrc.all; //debug
}

/* calculate position */
void EncoderDriver::calcPos()
{
	posRaw = encParams.imp2rad * (double)dataWithCrc.bit.pos;
	posCalibrated = posRaw - encParams.mechanicalOffset;

	posAngle = posRaw; //debug
}

void EncoderDriver::readRequest()
{
//	HAL_SPI_Receive(spi, dataBuff, dataBuffLen, HAL_MAX_DELAY);
	if (!isTransfer)
	{
		HAL_SPI_Receive_IT(spi, dataBuff, dataBuffLen);
		isTransfer = true;
	}
	else
	{
		cntErr++;
	}
}

void EncoderDriver::readRequestB()
{
//	HAL_SPI_Receive(spi, dataBuff, dataBuffLen, HAL_MAX_DELAY);
	if (!isTransfer)
	{
		HAL_SPI_Receive_IT(spi, dataBuffB, dataBuffLen);
		isTransfer = true;
	}
	else
	{
		cntErr++;
	}
}

uint32_t EncoderDriver::readEncoder()
{
//	for (int i = 0; i < dataBuffLen; i++)
//	{
//		dataBuff[i] = buff[i];
//	}
	checkAck();
	getStartIndex();
	getShift();
	assignData();
	isCrcOk = bissCrc.checkCrc(dataWithCrc);
	if (isCrcOk)
		calcPos();
	crcOk = (uint8_t)isCrcOk; //debug
	isTransfer = false;
//	pos[cntP] = posError; //debug
//	cntP++; //debug
//	cntP = cntP % BUF_POS_LEN; //debug

//	return posCalibrated;
	return dataWithCrc.bit.pos;
}

uint32_t EncoderDriver::readEncoderB()
{
	checkAckB();
	getStartIndexB();
	getShiftB();
	assignDataB();
	isCrcOk = bissCrc.checkCrc(dataWithCrc);
	if (isCrcOk)
		calcPos();
	crcOk = (uint8_t)isCrcOk; //debug
	isTransfer = false;

	return dataWithCrc.bit.pos;
}
