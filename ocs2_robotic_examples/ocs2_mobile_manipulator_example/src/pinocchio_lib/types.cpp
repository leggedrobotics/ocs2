/*
 * Author: Michael Spieler
 * Date:   2020-09-08
 */

#include <ocs2_mobile_manipulator_example/pinocchio_lib.h>

namespace pinocchio {

template struct ModelTpl<double, 0, JointCollectionDefaultTpl>;
template struct DataTpl<double, 0, JointCollectionDefaultTpl>;

}  // namespace pinocchio
