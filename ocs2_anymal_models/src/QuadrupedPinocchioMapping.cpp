#include "ocs2_anymal_models/QuadrupedPinocchioMapping.h"

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
QuadrupedPinocchioMappingTpl<SCALAR_T>::QuadrupedPinocchioMappingTpl(switched_model::feet_array_t<std::size_t> feetMap)
    : mapFeetOrderOcs2ToPinocchio_(std::move(feetMap)) {}

template <typename SCALAR_T>
auto QuadrupedPinocchioMappingTpl<SCALAR_T>::mapJointOcs2ToPinocchio(const joint_coordinate_t& jointPositions) const -> joint_coordinate_t {
  joint_coordinate_t pinocchioJointPositions;
  // OCS2 LF
  pinocchioJointPositions.template segment<3>(3 * mapFeetOrderOcs2ToPinocchio_[0]) = jointPositions.template segment<3>(0);
  // OCS2 RF
  pinocchioJointPositions.template segment<3>(3 * mapFeetOrderOcs2ToPinocchio_[1]) = jointPositions.template segment<3>(3);
  // OCS2 LH
  pinocchioJointPositions.template segment<3>(3 * mapFeetOrderOcs2ToPinocchio_[2]) = jointPositions.template segment<3>(6);
  // OCS2 RH
  pinocchioJointPositions.template segment<3>(3 * mapFeetOrderOcs2ToPinocchio_[3]) = jointPositions.template segment<3>(9);

  return pinocchioJointPositions;
}

}  // namespace tpl
}  // namespace anymal

// Explicit instantiation
template class anymal::tpl::QuadrupedPinocchioMappingTpl<ocs2::scalar_t>;
template class anymal::tpl::QuadrupedPinocchioMappingTpl<ocs2::ad_scalar_t>;
