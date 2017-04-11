#include <gtest/gtest.h>
#include <limits>
#include <tug_cfg/BarConfig.h>
#include <tug_cfg/configuration.h>
#include <tug_cfg/yaml_reader.h>

TEST(TestBarConfig, testDefaultConstructor)
{
  tug_cfg::BarConfig config;
}

TEST(TestBarConfig, testConstrainer)
{
  tug_cfg::BarConfig config;
  ASSERT_EQ(config.factor, 10);
  config.factor = 0;
  tug_cfg::constrain(config);
  ASSERT_EQ(config.factor, 0.1);
}

TEST(TestBarConfig, testLoadYaml)
{
  tug_cfg::BarConfig config;
  tug_cfg::YamlReader reader("../../../src/utils/tug_cfg/test/bar1.yaml");
  tug_cfg::load(config, reader);
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
