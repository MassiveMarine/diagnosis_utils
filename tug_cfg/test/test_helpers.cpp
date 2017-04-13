#include "test_helpers.h"
#include <gtest/gtest.h>
#include <limits>

void assertBar1LoadedCorrectly(const tug_cfg::BarConfig& config)
{
  ASSERT_EQ(config.rate, 33);
  ASSERT_EQ(config.duration, 2);
  ASSERT_EQ(config.have_tree, false);
  ASSERT_EQ(config.factor, 20);
  ASSERT_EQ(config.abs_distance, 10);
  ASSERT_EQ(config.rel_distance, 20);
  ASSERT_EQ(config.linear_speed, std::numeric_limits<double>::infinity());
  ASSERT_EQ(config.interpolation_mode, "quadratic");
  ASSERT_EQ(config.baud_rate, 300);
  ASSERT_EQ(config.plugins.size(), 2);
  ASSERT_EQ(config.plugins.at(0), "foo/Bar");
  ASSERT_EQ(config.plugins.at(1), "bla::Blubb");
  ASSERT_EQ(config.filters.size(), 0);
  ASSERT_EQ(config.coefficients.size(), 2);
  ASSERT_EQ(config.coefficients.at(0).size(), 2);
  ASSERT_EQ(config.coefficients.at(0).at(0), 1);
  ASSERT_EQ(config.coefficients.at(0).at(1), 2);
  ASSERT_EQ(config.coefficients.at(1).size(), 2);
  ASSERT_EQ(config.coefficients.at(1).at(0), 3);
  ASSERT_EQ(config.coefficients.at(1).at(1), 4);
  ASSERT_EQ(config.kvargs.size(), 2);
  ASSERT_EQ(config.kvargs.at("a"), "b");
  ASSERT_EQ(config.kvargs.at("c"), "d");
  ASSERT_EQ(config.error_names.size(), 2);
  ASSERT_EQ(config.error_names.at(-1), "Gaah");
  ASSERT_EQ(config.error_names.at(-2), "Panic");
}
