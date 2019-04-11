//
// Created by ruben on 02.11.18.
//

#include <gtest/gtest.h>

#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include <ocs2_core/loopshaping/LoopshapingCost.h>
#include <ocs2_core/loopshaping/LoopshapingConstraint.h>
#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <experimental/filesystem>

using namespace ocs2;

const std::experimental::filesystem::path pathToTest(__FILE__);
const std::string settingsFile_r = std::string(pathToTest.parent_path()) + "/loopshaping_r.conf";
const std::string settingsFile_r_simple = std::string(pathToTest.parent_path()) + "/loopshaping_r_simple.conf";
const std::string settingsFile_s = std::string(pathToTest.parent_path()) + "/loopshaping_s.conf";
const auto inf_ = std::numeric_limits<double>::infinity();

TEST(testLoopshapingDefinition, SISO_Definition) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(settingsFile_r, pt);

  SISOFilterDefinition filter0(pt, "r_filter", "Filter0");
  std::cout << "\nFilter0" << std::endl;
  filter0.print();

  SISOFilterDefinition filter1(pt, "r_filter", "Filter1");
  std::cout << "\nFilter1" << std::endl;
  filter1.print();

  ASSERT_TRUE(true);
}

TEST(testLoopshapingDefinition, MIMO_Definition) {

  MIMOFilterDefinition filter;
  filter.loadSettings(settingsFile_s, "s_inv_filter");
  filter.print();

  ASSERT_TRUE(true);
}

TEST(testLoopshapingDefinition, Loopshaping_Definition_r) {

  LoopshapingDefinition filter;
  filter.loadSettings(settingsFile_r);
  filter.print();

  ASSERT_TRUE(true);
}

TEST(testLoopshapingDefinition, Loopshaping_Definition_r_simple) {

  LoopshapingDefinition filter;
  filter.loadSettings(settingsFile_r_simple);
  filter.print();

  ASSERT_TRUE(true);
}

TEST(testLoopshapingDefinition, Loopshaping_Definition_s) {

  LoopshapingDefinition filter;
  filter.loadSettings(settingsFile_s);
  filter.print();

  ASSERT_TRUE(true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}