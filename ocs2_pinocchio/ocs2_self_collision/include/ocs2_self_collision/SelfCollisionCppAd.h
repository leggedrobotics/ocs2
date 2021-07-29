/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_self_collision/PinocchioGeometryInterface.h"

namespace ocs2 {

class SelfCollisionCppAd {
 public:
  using vector3_t = Eigen::Matrix<ad_scalar_t, 3, 1>;
  using quaternion_t = Eigen::Quaternion<ad_scalar_t>;

  /**
   * Constructor
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @param [in] pinocchioGeometryInterface: pinocchio geometry interface of the robot model
   * @param [in] minimumDistance: minimum allowed distance between each collision pair
   * @param [in] modelName : name of the generate model library
   * @param [in] modelFolder : folder to save the model library files to
   * @param [in] recompileLibraries : If true, the model library will be newly compiled. If false, an existing library will be loaded if
   *                                  available.
   * @param [in] verbose : print information.
   */
  SelfCollisionCppAd(const PinocchioInterface& pinocchioInterface, PinocchioGeometryInterface pinocchioGeometryInterface,
                     scalar_t minimumDistance, const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2",
                     bool recompileLibraries = true, bool verbose = true);

  /** Default destructor */
  ~SelfCollisionCppAd() = default;

  /** Copy constructor */
  SelfCollisionCppAd(const SelfCollisionCppAd& rhs);

  /** Get the number of collision pairs */
  size_t getNumCollisionPairs() const { return pinocchioGeometryInterface_.getNumCollisionPairs(); }

  /**
   * Evaluate the distance violation
   * Computes the distance results of all collision pairs through PinocchioGeometryInterface
   * and the violation compared with the specified minimum distance.
   *
   * @note Requires updated forwardKinematics() on pinocchioInterface.
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @return: the differences between the distance of each collision pair and the minimum distance
   */
  vector_t getValue(const PinocchioInterface& pinocchioInterface) const;

  /**
   * Evaluate the linear approximation of the distance function
   *
   * @note Requires updated forwardKinematics() on pinocchioInterface.
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @param [in] q: pinocchio coordinates
   * @return: the pair of the distance violation and the first derivative of the distance against q
   */
  std::pair<vector_t, matrix_t> getLinearApproximation(const PinocchioInterface& pinocchioInterface, const vector_t& q) const;

 private:
  /**
   * Sets all the required CppAdCodeGenInterfaces
   */
  void setADInterfaces(PinocchioInterfaceCppAd& pinocchioInterfaceAd, const std::string& modelName, const std::string& modelFolder);

  // From the current state of the robot, and the closest points in world frame, compute the positions of the points in link frame
  // In this case : size of state = stateDim, size of points = 3*2*number of collision pairs + 1 (for sign indicator)
  // Returns a vector that is of length |3*2*number of collision pairs + 1 (for sign indicator)|
  ad_vector_t computeLinkPointsAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, const ad_vector_t& state, const ad_vector_t& points) const;
  // From the current state of the robot, and the closest points in link frames, calculate the distances wrt state
  // In this case : size of state = stateDim, size of points = 3*2*number of collision pairs + 1 (for sign indicator)
  // Returns a vector that is of length |collisionPairs|
  ad_vector_t distanceCalculationAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, const ad_vector_t& state,
                                    const ad_vector_t& points) const;

  // Number of params per result = 3 + 3 + 1 (nearest point 1, nearest point 2, sign indicator)
  const size_t numberOfParamsPerResult_ = 7;

  std::unique_ptr<CppAdInterface> cppAdInterfaceDistanceCalculation_;
  std::unique_ptr<CppAdInterface> cppAdInterfaceLinkPoints_;

  PinocchioGeometryInterface pinocchioGeometryInterface_;
  scalar_t minimumDistance_;
};

} /* namespace ocs2 */
