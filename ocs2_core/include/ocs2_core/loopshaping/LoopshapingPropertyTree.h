#pragma once

#include <boost/property_tree/ptree.hpp>
#include "LoopshapingDefinition.h"
#include "LoopshapingFilter.h"

namespace ocs2 {
namespace loopshaping_property_tree {

Filter readSISOFilter(const boost::property_tree::ptree& pt, std::string filterName, bool invert = false);

Filter readMIMOFilter(const boost::property_tree::ptree& pt, std::string filterName, bool invert = false);

std::shared_ptr<LoopshapingDefinition> load(const std::string& settingsFile);

}  // namespace loopshaping_property_tree
}  // namespace ocs2
