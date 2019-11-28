/*
 * AnymalWheelsMrtRos.h
 *
 *  Created on: Nov 27, 2019
 *      Author: Marko Bjelonic
 */

#ifndef MRT_ROS_ANYMAL_H_
#define MRT_ROS_ANYMAL_H_

#include <ocs2_quadruped_interface/MRT_ROS_Quadruped.h>

#include "ocs2_anymal_wheels/AnymalWheelsInterface.h"

namespace anymal {

class AnymalWheelsMrtRos : public switched_model::MRT_ROS_Quadruped<16> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<AnymalWheelsMrtRos> Ptr;

  typedef switched_model::MRT_ROS_Quadruped<16> BASE;

  typedef AnymalWheelsInterface ocs2_anymal_interface_t;

  /**
   * @param [in] ocs2QuadrupedInterfacePtr: A shared pointer to the quadruped interface class.
   * @param [in] robotName: The name's of the robot.
   */
  AnymalWheelsMrtRos(const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr, const std::string& pathToConfigFolder)
      : BASE(ocs2QuadrupedInterfacePtr, "anymal") {};

  ~AnymalWheelsMrtRos() = default;
};

}  // end of namespace anymal

#endif /* MRT_ROS_ANYMAL_H_ */
