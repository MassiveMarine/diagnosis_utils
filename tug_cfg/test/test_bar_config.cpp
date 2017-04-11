#include <gtest/gtest.h>
#include <tug_cfg/BarConfig.h>
#include <tug_cfg/configuration.h>

TEST(TestBarConfig, testDefaultConstructor)
{
  tug_cfg::BarConfig bar_config;
}

TEST(TestBarConfig, testConstrainer)
{
  tug_cfg::BarConfig bar_config;
  ASSERT_EQ(bar_config.factor, 10);
  bar_config.factor = 0;
  tug_cfg::constrain(bar_config);
  ASSERT_EQ(bar_config.factor, 0.1);
}

TEST(TestBarConfig, testLoadYaml)
{
  tug_cfg::BarConfig bar_config;
  //tug_cfg::load(bar_config, tug_cfg::YamlReader("test/bar1.yaml"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
