#include <tug_colin/conversion.h>

namespace utils
{

//--------------------------------------------------------------------------------
void Conversion::uint8Touint16(uint8_t src_value[2], uint16_t & dest_value)
{
  dest_value = ((src_value[1] << 8) | src_value[0]);
}

//--------------------------------------------------------------------------------
void Conversion::uint16Touint8(uint16_t src_value, uint8_t (&dest_value)[2])
{
  dest_value[0] = (0x00FF & src_value);
  dest_value[1] = (0xFF00 & src_value) >> 8;
}

} // namespace

