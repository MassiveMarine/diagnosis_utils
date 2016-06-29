#include <tug_bresenham/line_iterator.h>
#include <cmath>



namespace tug_bresenham
{

LineIterator::LineIterator(int start_x, int start_y, int end_x, int end_y)
  : x_(start_x), y_(start_y), end_x_(end_x), end_y_(end_y), beyond_end_(false)
{
  dx_ = std::abs(end_x - start_x);
  dy_ = -std::abs(end_y - start_y);

  sx_ = (start_x < end_x) ? 1 : -1;
  sy_ = (start_y < end_y) ? 1 : -1;

  error_ = dx_ + dy_;
}

void LineIterator::operator++()
{
  advance();
}

void LineIterator::operator++(int)
{
  advance();
}

bool LineIterator::isAtEnd() const
{
  return (x_ == end_x_) && (y_ == end_y_);
}

bool LineIterator::isBeyondEnd() const
{
  return beyond_end_;
}

void LineIterator::advance()
{
  if (isAtEnd())
  {
    beyond_end_ = true;
  }

  int error_2 = 2 * error_;

  if (error_2 > dy_)
  {
    error_ += dy_;
    x_ += sx_;
  }

  if (error_2 < dx_)
  {
    error_ += dx_;
    y_ += sy_;
  }
}

}
