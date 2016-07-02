#include <tug_bresenham/basic_circle_iterator.h>



namespace tug_bresenham
{

BasicCircleIterator::BasicCircleIterator(int radius)
  : dx_(0), dy_(radius), r_squared_(static_cast<long>(radius) * radius)
{
  error_ = r_squared_ - (2 * r_squared_ - 1) * r_squared_;
}

void BasicCircleIterator::operator++()
{
  advance();
}

void BasicCircleIterator::operator++(int)
{
  advance();
}

bool BasicCircleIterator::isFinished() const
{
  return dy_ < 0;
}

void BasicCircleIterator::advance()
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
