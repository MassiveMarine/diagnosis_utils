#include <gtest/gtest.h>
#include <limits>
#include <tug_cfg/BarConfig.h>
#include <tug_cfg/configuration.h>
#include <tug_cfg/yaml_reader.h>
#include "test_helpers.h"

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
  assertBar1LoadedCorrectly(config);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
