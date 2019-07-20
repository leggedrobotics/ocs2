#pragma once

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "ocs2_double_slit_example/definitions.h"

namespace ocs2 {
namespace double_slit {

/**
 * This class implements MRT (Model Reference Tracking) communication interface using ROS.
 *
 * @tparam double_slit::STATE_DIM_: Dimension of the state space.
 * @tparam double_slit::INPUT_DIM_: Dimension of the control input space.
 */
using MrtRosDoubleSlit = MRT_ROS_Interface<double_slit::STATE_DIM_, double_slit::INPUT_DIM_>;

}  // namespace double_slit
}  // namespace ocs2
