#include "ocs2_pipg/PIPG_Settings.h"

#include <iostream>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
namespace ocs2 {
namespace pipg {
Settings loadSettings(const std::string& filename, const std::string& fieldName, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  Settings settings;

  if (verbose) {
    std::cerr << "\n #### PIPG Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, settings.nThreads, fieldName + ".nThreads", verbose);
  loadData::loadPtreeValue(pt, settings.threadPriority, fieldName + ".threadPriority", verbose);

  loadData::loadPtreeValue(pt, settings.maxNumIterations, fieldName + ".maxNumIterations", verbose);
  loadData::loadPtreeValue(pt, settings.absoluteTolerance, fieldName + ".absoluteTolerance", verbose);
  loadData::loadPtreeValue(pt, settings.relativeTolerance, fieldName + ".relativeTolerance", verbose);

  loadData::loadPtreeValue(pt, settings.numScaling, fieldName + ".numScaling", verbose);
  loadData::loadPtreeValue(pt, settings.lowerBoundH, fieldName + ".lowerBoundH", verbose);

  loadData::loadPtreeValue(pt, settings.checkTerminationInterval, fieldName + ".checkTerminationInterval", verbose);
  loadData::loadPtreeValue(pt, settings.displayShortSummary, fieldName + ".displayShortSummary", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return settings;
}

}  // namespace pipg
}  // namespace ocs2