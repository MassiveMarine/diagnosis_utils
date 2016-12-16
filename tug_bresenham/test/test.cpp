#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <tug_bresenham/basic_circle_iterator.h>
#include <tug_bresenham/circle_iterator.h>
#include <tug_bresenham/line_iterator.h>
#include <vector>



typedef std::vector<std::pair<int, int> > CoordsVector;

class CoordsBuilder
{
public:
  CoordsBuilder(int x, int y)
  {
    coords_.push_back(std::make_pair(x, y));
  }

  CoordsBuilder& operator()(int x, int y)
  {
    coords_.push_back(std::make_pair(x, y));
    return *this;
  }

  operator CoordsVector&()
  {
    return coords_;
  }

protected:
  CoordsVector coords_;
};



static void expectCoords(tug_bresenham::BasicCircleIterator& bci, const CoordsVector& expected_coords)
{
  size_t i = 0;
  for (; i < expected_coords.size() && !bci.isFinished(); ++i, ++bci)
  {
    //std::cerr << "{ x: " << bci.getDx() << "; y: " << bci.getDy() << " }" << std::endl;
    EXPECT_EQ(expected_coords[i].first, bci.getDx());
    EXPECT_EQ(expected_coords[i].second, bci.getDy());
    EXPECT_FALSE(bci.isFinished());
  }
  EXPECT_EQ(i, expected_coords.size());
  EXPECT_TRUE(bci.isFinished());
}



static void testRadius(int radius)
{
  tug_bresenham::BasicCircleIterator bci(radius);
  long i = 0;
  for (; !bci.isFinished(); ++i, ++bci)
  {
    //std::cerr << "{ x: " << bci.getDx() << "; y: " << bci.getDy() << " }" << std::endl;
    double actual_radius = std::sqrt(std::pow(bci.getDx(), 2.0) +
                                     std::pow(bci.getDy(), 2.0));
    EXPECT_NEAR(radius, actual_radius, 0.5);
  }
  EXPECT_GT(i, radius); // Iterator must return more than <radius> points
  EXPECT_LE(i, 2L * radius); // Iterator must return less than 2*<radius> points
  EXPECT_TRUE(bci.isFinished());
}



static void testRadius(int radius, long iterations)
{
  tug_bresenham::BasicCircleIterator bci(radius);
  long i = 0;
  for (; i < iterations && !bci.isFinished(); ++i, ++bci)
  {
    //std::cerr << "{ x: " << bci.getDx() << "; y: " << bci.getDy() << " }" << std::endl;
    double actual_radius = std::sqrt(std::pow(bci.getDx(), 2.0) +
                                     std::pow(bci.getDy(), 2.0));
    EXPECT_NEAR(radius, actual_radius, 0.5);
  }
  EXPECT_EQ(i, iterations);
  EXPECT_FALSE(bci.isFinished());
}



static void testExpectedException(int radius)
{
  try
  {
    tug_bresenham::BasicCircleIterator bci(radius);
    ADD_FAILURE() << "Expected exception was not thrown";
  }
  catch (std::invalid_argument&)
  {
    // Expected exception.
  }
  catch (...)
  {
    ADD_FAILURE() << "Caught unexpected exception";
  }
}



TEST(BasicCircleIterator, testCircleNegativeRadius)
{
  testExpectedException(std::numeric_limits<int>::min());
  testExpectedException(-2);
  testExpectedException(-1);
}



TEST(BasicCircleIterator, testCircle0)
{
  testExpectedException(0);
}



TEST(BasicCircleIterator, testCircle1)
{
  tug_bresenham::BasicCircleIterator bci(1);
  expectCoords(bci, CoordsBuilder(0, 1)(1, 0));
}



TEST(BasicCircleIterator, testCircle2)
{
  tug_bresenham::BasicCircleIterator bci(2);
  expectCoords(bci, CoordsBuilder(0, 2)(1, 2)(2, 1)(2, 0));
}



TEST(BasicCircleIterator, testCircle3)
{
  tug_bresenham::BasicCircleIterator bci(3);
  expectCoords(bci, CoordsBuilder(0, 3)(1, 3)(2, 2)(3, 1)(3, 0));
}



TEST(BasicCircleIterator, testCircleRadius1To100)
{
  for (int radius = 1; radius <= 100; ++radius)
  {
    testRadius(radius);
  }
}



/*TEST(BasicCircleIterator, testCircleRadiusIntMax)
{
  testRadius(std::numeric_limits<int>::max() - 2, 100);
}*/



int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
