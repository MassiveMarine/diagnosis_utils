#ifndef KRIKKIT_CONVERSION_H_
#define KRIKKIT_CONVERSION_H_

#include "ros/ros.h"

namespace utils
{
class Conversion
{
public:
  //!-------------------------------------------------------------------------------
  //! uint16Touint8
  //! converts a uint16 into two uint8 (Little-Endian)
  //! @param src_value: the source value of the conversion
  //! @param dest_value: the destination value of the conversion
  static void uint16Touint8(uint16_t src_value, uint8_t (&dest_value)[2]);

  //!-------------------------------------------------------------------------------
  //! uint8Touint16
  //! converts from two uint8 into uint16 (Little-Endian)
  //! @param src_value: the source value of the conversion
  //! @param dest_value: the destination value of the conversion
  static void uint8Touint16(uint8_t src_value[2], uint16_t& dest_value);
};
} // namespace

#endif /* KRIKKIT_CONVERSION_H_ */
