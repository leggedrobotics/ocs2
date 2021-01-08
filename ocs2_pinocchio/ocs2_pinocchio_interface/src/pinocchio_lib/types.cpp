/*
 * Author: Michael Spieler
 * Date:   2020-09-08
 */

#include <ocs2_pinocchio_interface/pinocchio_lib.h>

namespace pinocchio {

template struct ModelTpl<scalar_t, 0, JointCollectionDefaultTpl>;
template struct DataTpl<scalar_t, 0, JointCollectionDefaultTpl>;

}  // namespace pinocchio
