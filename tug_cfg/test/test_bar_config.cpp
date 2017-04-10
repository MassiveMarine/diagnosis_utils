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
  bar_config.factor = 0;
  //tug_cfg::constrain(bar_config);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


