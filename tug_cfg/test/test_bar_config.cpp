#include <gtest/gtest.h>
#include <tug_cfg/BarConfig.h>

TEST(TestBarConfig, testDefaultConstructor)
{
  tug_cfg::BarConfig bar_config;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


