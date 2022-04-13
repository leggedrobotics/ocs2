#pragma once

#include <ocs2_core/Types.h>

#include <string>

namespace ocs2 {
namespace pipg {

struct Settings {
  Settings() = default;
  Settings(size_t nThreads, int threadPriority, size_t maxNumIterations, scalar_t absoluteTolerance, scalar_t relativeTolerance,
           size_t checkTerminationInterval, bool displayShortSummary)
      : nThreads(nThreads),
        threadPriority(threadPriority),
        maxNumIterations(maxNumIterations),
        absoluteTolerance(absoluteTolerance),
        relativeTolerance(relativeTolerance),
        checkTerminationInterval(checkTerminationInterval),
        displayShortSummary(displayShortSummary){};

  /** Number of threads used in the multi-threading scheme. */
  size_t nThreads = 1;
  /** Priority of threads used in the multi-threading scheme. */
  int threadPriority = 99;
  /** Maximum number of iterations of DDP. */
  size_t maxNumIterations = 15;
  /** Termination criteria. **/
  scalar_t absoluteTolerance = 1e-3;
  scalar_t relativeTolerance = 1e-2;
  /** Number of iterations between consecutive calculation of termination conditions. **/
  size_t checkTerminationInterval = 25;
  /** This value determines to display the a summary log. */
  bool displayShortSummary = false;
};

// Settings loadSettings(const std::string& filename, const std::string& fieldName = "ddp", bool verbose = true);

}  // namespace pipg
}  // namespace ocs2
