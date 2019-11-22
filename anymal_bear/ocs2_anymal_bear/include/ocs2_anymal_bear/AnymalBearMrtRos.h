/*
 * AnymalBearMrtRos.h
 *
 *  Created on: Jun 18, 2018
 *      Author: farbod
 */

#ifndef MRT_ROS_ANYMAL_H_
#define MRT_ROS_ANYMAL_H_

#include <ocs2_quadruped_interface/MRT_ROS_Quadruped.h>

#include "ocs2_anymal_bear/AnymalBearInterface.h"

namespace anymal {

class AnymalBearMrtRos : public switched_model::MRT_ROS_Quadruped<12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<AnymalBearMrtRos> Ptr;

  typedef switched_model::MRT_ROS_Quadruped<12> BASE;

  typedef AnymalBearInterface ocs2_anymal_interface_t;

  /**
   * @param [in] ocs2QuadrupedInterfacePtr: A shared pointer to the quadruped interface class.
   * @param [in] robotName: The name's of the robot.
   */
  AnymalBearMrtRos(const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr, const std::string& pathToConfigFolder)
      : BASE(ocs2QuadrupedInterfacePtr, "anymal") {};

  ~AnymalBearMrtRos() = default;
};

}  // end of namespace anymal

#endif /* MRT_ROS_ANYMAL_H_ */
