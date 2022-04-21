#pragma once

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace anymal {
namespace tpl {

/**
 * Used to map joint configuration space from OCS2 to Pinocchio. In OCS2, the feet order is {LF, RF, LH, RH}. But in Pinocchio, the feet
 * order depends on the URDF.
 */
template <typename SCALAR_T>
class QuadrupedPinocchioMappingTpl {
 public:
  using joint_coordinate_t = switched_model::joint_coordinate_s_t<SCALAR_T>;

  /**
   * @param feetMap : specify the order in which the feet are ordered in the pinnochio.
   * Assume index i stands for a foot index in OCS2 and thus feetMap[i] is the foot index in Pinocchio.
   *
   * feetmap[0] = left-front foot order in URDF / pinnochio
   * ...
   * feetmap[3] = right-hind foot order in URDF / pinnochio
   */
  explicit QuadrupedPinocchioMappingTpl(const switched_model::feet_array_t<size_t>& feetMap);

  joint_coordinate_t mapJointOcs2ToPinocchio(const joint_coordinate_t& state) const;

  size_t mapFootIdxOcs2ToPinocchio(size_t ocs2Idx) const { return mapFeetOrderOcs2ToPinocchio_[ocs2Idx]; }

 private:
  switched_model::feet_array_t<size_t> mapFeetOrderOcs2ToPinocchio_;
};

}  // namespace tpl

using QuadrupedMapping = tpl::QuadrupedPinocchioMappingTpl<ocs2::scalar_t>;
using QuadrupedMappingAd = tpl::QuadrupedPinocchioMappingTpl<ocs2::ad_scalar_t>;

}  // namespace anymal

extern template class anymal::tpl::QuadrupedPinocchioMappingTpl<ocs2::scalar_t>;
extern template class anymal::tpl::QuadrupedPinocchioMappingTpl<ocs2::ad_scalar_t>;
