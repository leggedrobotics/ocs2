/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iostream>
#include <string>

namespace ocs2 {

/**
 * This structure holds all setting parameters for the MPC class.
 */
class MPC_Settings {
 public:
  /**
   * Default constructor.
   */
  MPC_Settings()

      : runtimeMaxNumIterations_(15),
        initMaxNumIterations_(15),
        runtimeMaxLearningRate_(1.0),
        runtimeMinLearningRate_(1.0),
        initMaxLearningRate_(1.0),
        initMinLearningRate_(1.0),
        debugPrint_(false),
        coldStart_(false),
        recedingHorizon_(true),
        blockwiseMovingHorizon_(false),
        useParallelRiccatiSolver_(false),
        solutionTimeWindow_(-1),    // [s]
        mpcDesiredFrequency_(-1),   // [Hz]
        mrtDesiredFrequency_(100),  // [Hz]
        maxTimeStep_(1e-3) {}

  /**
   * Loads the MPC settings from a given file.
   *
   * @param [in] filename: File name which contains the configuration data.
   * @param [in] verbose: Flag to determine whether to print out the loaded settings or not
   * (The default is true).
   */
  void loadSettings(const std::string& filename, bool verbose = true);

  /**
   * Helper function for loading setting fields
   *
   * @param [in] pt: Property_tree.
   * @param [in] prefix: Field name prefix.
   * @param [in] fieldName: Field name.
   * @param [in] verbose: Print loaded settings if true.
   */
  template <typename T>
  void loadField(const boost::property_tree::ptree& pt, const std::string& prefix, const std::string& fieldName, T& field, bool verbose);

 public:
  /****************
   *** Variables **
   ****************/
  /** Number of iterations which will be used during MPC regular loop.. */
  size_t runtimeMaxNumIterations_;
  /** Number of iterations which will be used during MPC initial run. */
  size_t initMaxNumIterations_;
  /** Maximum learning rate which will be used during MPC regular loop. */
  double runtimeMaxLearningRate_;
  /** Maximum learning rate which will be used during MPC regular loop. */
  double runtimeMinLearningRate_;
  /** Maximum learning rate which will be used during MPC initial run. */
  double initMaxLearningRate_;
  /** Minimum learning rate which will be used during MPC initial run. */
  double initMinLearningRate_;
  /** This value determines to display the log output of MPC. */
  bool debugPrint_;
  /** This value determines to initialize the SLQ with the controller from previous call
   * (warm start) or the given operating trajectories (cold start). */
  bool coldStart_;
  /** Either to use the receding horizon MPC or not.*/
  bool recedingHorizon_;
  /** If true the final time of the MPC will increase by the length of a time partition
   * instead of commonly used scheme where the final time is gradual increased. */
  bool blockwiseMovingHorizon_;
  /** If set true, the parallel Riccati solver will be used from the first iteration of SLQ
   * solver. */
  bool useParallelRiccatiSolver_;
  /** The time window for retrieving the optimized output (controller and trajectory). */
  double solutionTimeWindow_;
  /**
   * MPC loop frequency in Hz. This setting is only used in Dummy_Loop for testing.
   * If set to a positive number, MPC loop of test will be simulated to run by this
   * frequency. Note that this might not be the MPC's realtime frequency.
   */
  double mpcDesiredFrequency_;
  /**
   * MRT loop frequency in Hz. This setting is only used in Dummy_Loop for testing.
   * This should always set to a positive number which can be interpreted as the
   * tracking controller's frequency.
   */
  double mrtDesiredFrequency_;

  double maxTimeStep_;

};  // end of MPC_Settings class

template <typename T>
inline void MPC_Settings::loadField(const boost::property_tree::ptree& pt, const std::string& prefix, const std::string& fieldName,
                                    T& field, bool verbose) {
  std::string comment;
  try {
    field = pt.get<T>(prefix + "." + fieldName);
  } catch (const std::exception& e) {
    comment = "   \t(default)";
  }

  if (verbose) {
    int fill = std::max<int>(0, 36 - static_cast<int>(fieldName.length()));
    std::cerr << " #### Option loader : option '" << fieldName << "' " << std::string(fill, '.') << " " << field << comment << std::endl;
  }
}

inline void MPC_Settings::loadSettings(const std::string& filename, bool verbose /*= true*/) {
  std::string fieldName = "mpc";
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  if (verbose) {
    std::cerr << std::endl << " #### MPC Settings: " << std::endl;
    std::cerr << " #### =============================================================================" << std::endl;
  }

  loadField(pt, fieldName, "runtimeMaxNumIterations", runtimeMaxNumIterations_, verbose);
  loadField(pt, fieldName, "initMaxNumIterations", initMaxNumIterations_, verbose);
  loadField(pt, fieldName, "runtimeMaxLearningRate", runtimeMaxLearningRate_, verbose);
  loadField(pt, fieldName, "runtimeMinLearningRate", runtimeMinLearningRate_, verbose);
  loadField(pt, fieldName, "initMaxLearningRate", initMaxLearningRate_, verbose);
  loadField(pt, fieldName, "initMinLearningRate", initMinLearningRate_, verbose);
  loadField(pt, fieldName, "debugPrint", debugPrint_, verbose);
  loadField(pt, fieldName, "coldStart", coldStart_, verbose);
  loadField(pt, fieldName, "recedingHorizon", recedingHorizon_, verbose);
  loadField(pt, fieldName, "blockwiseMovingHorizon", blockwiseMovingHorizon_, verbose);
  loadField(pt, fieldName, "useParallelRiccatiSolver", useParallelRiccatiSolver_, verbose);
  loadField(pt, fieldName, "solutionTimeWindow", solutionTimeWindow_, verbose);
  loadField(pt, fieldName, "mpcDesiredFrequency", mpcDesiredFrequency_, verbose);
  loadField(pt, fieldName, "mrtDesiredFrequency", mrtDesiredFrequency_, verbose);
  loadField(pt, fieldName, "maxTimeStep", maxTimeStep_, verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }
}

}  // namespace ocs2
