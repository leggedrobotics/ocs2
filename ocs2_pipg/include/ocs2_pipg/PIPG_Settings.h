#pragma once

#include <string>

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace pipg {

struct Settings {
  /** Number of threads used in the multi-threading scheme. */
  size_t nThreads = 3;
  /** Priority of threads used in the multi-threading scheme. */
  int threadPriority = 99;
  /** Maximum number of iterations of PIPG. */
  size_t maxNumIterations = 3000;
  /** Termination criteria. **/
  scalar_t absoluteTolerance = 1e-3;
  scalar_t relativeTolerance = 1e-2;
  /** Number of iterations between consecutive calculation of termination conditions. **/
  size_t checkTerminationInterval = 1;
  /** Number of pre-conditioning run. **/
  int numScaling = 3;
  /** The static lower bound of the cost hessian H. **/
  scalar_t lowerBoundH = 5e-6;
  /** This value determines to display the a summary log. */
  bool displayShortSummary = false;
};

Settings loadSettings(const std::string& filename, const std::string& fieldName = "pipg", bool verbose = true);

}  // namespace pipg
}  // namespace ocs2
