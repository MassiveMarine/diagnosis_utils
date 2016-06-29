#include <tug_bresenham/circle_iterator.h>
#include <cmath>



namespace tug_bresenham
{

CircleIterator::CircleIterator(int radius)
  : dx_(0), dy_(radius), r_squared_(static_cast<long>(radius) * radius)
{
  error_ = r_squared_ - (2 * r_squared_ - 1) * r_squared_;
}

void CircleIterator::operator++()
{
  advance();
}

void CircleIterator::operator++(int)
{
  advance();
}

bool CircleIterator::isFinished() const
{
  return dy_ < 0;
}

void CircleIterator::advance()
{
  if (!isFinished())
  {
    long error_2 = 2 * error_;

    if (error_2 < ((2 * dx_ + 1) * r_squared_))
    {
      dx_++;
      error_ += (2 * dx_ + 1) * r_squared_;
    }

    if (error_2 > -((2 * dy_ - 1) * r_squared_))
    {
      dy_--;
      error_ -= (2 * dy_ - 1) * r_squared_;
    }
  }
}

}
