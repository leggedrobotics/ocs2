//
// Created by ruben on 02.11.18.
//

#include <gtest/gtest.h>

#include <boost/property_tree/info_parser.hpp>

#include <ocs2_core/loopshaping/LoopshapingPropertyTree.h>
#include <experimental/filesystem>

using namespace ocs2;

const std::experimental::filesystem::path pathToTest(__FILE__);
const std::string settingsFile_r = std::string(pathToTest.parent_path()) + "/loopshaping_r.conf";
const std::string settingsFile_r_simple = std::string(pathToTest.parent_path()) + "/loopshaping_r_simple.conf";
const std::string settingsFile_s = std::string(pathToTest.parent_path()) + "/loopshaping_s.conf";
const std::string settingsFile_s_simple = std::string(pathToTest.parent_path()) + "/loopshaping_s_simple.conf";

TEST(testLoopshapingDefinition, SISO_Definition) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(settingsFile_r, pt);

  auto filter0 = loopshaping_property_tree::readSISOFilter(pt, "r_filter.Filter0");
  std::cout << "\nFilter0" << std::endl;
  filter0.print();

  auto filter1 = loopshaping_property_tree::readSISOFilter(pt, "r_filter.Filter1");
  std::cout << "\nFilter1" << std::endl;
  filter1.print();

  ASSERT_TRUE(true);
}

TEST(testLoopshapingDefinition, MIMO_Definition) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(settingsFile_s, pt);
  auto filter = loopshaping_property_tree::readMIMOFilter(pt, "s_inv_filter", /*invert=*/true);
  filter.print();

  ASSERT_TRUE(true);
}

TEST(testLoopshapingDefinition, Loopshaping_Definition_r) {
  auto filter = loopshaping_property_tree::load(settingsFile_r);
  filter->print();

  ASSERT_TRUE(true);
}

TEST(testLoopshapingDefinition, Loopshaping_Definition_r_simple) {
  auto filter = loopshaping_property_tree::load(settingsFile_r_simple);
  filter->print();

  ASSERT_TRUE(true);
}

TEST(testLoopshapingDefinition, Loopshaping_Definition_s) {
  auto filter = loopshaping_property_tree::load(settingsFile_s);
  filter->print();

  ASSERT_TRUE(true);
}

TEST(testLoopshapingDefinition, Loopshaping_Definition_s_simple) {
  auto filter = loopshaping_property_tree::load(settingsFile_s_simple);
  filter->print();

  ASSERT_TRUE(true);
}
