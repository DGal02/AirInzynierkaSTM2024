#include <config.hpp>
#include <crc/bissCrc6.hpp>
#include <measurement/encoder_driver.hpp>

extern "C"
{

	BissCrc6 bissCrc;

}

uint8_t calcCrc(uint64_t data)
{
	return bissCrc.calcCrc_raw(data);
}
