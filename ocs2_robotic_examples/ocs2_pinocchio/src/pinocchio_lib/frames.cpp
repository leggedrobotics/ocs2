/*
 * Author: Michael Spieler
 * Date:   2020-09-08
 */

#include <ocs2_pinocchio/pinocchio_lib.h>

#include <pinocchio/algorithm/frames.hpp>

namespace pinocchio {

template void updateFramePlacements<scalar_t, 0, JointCollectionDefaultTpl>(const Model& model, Data& data);

}  // namespace pinocchio
