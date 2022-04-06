#pragma once

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace anymal {

namespace tpl {
template <typename SCALAR_T>
class QuadrupedPinocchioMappingTpl final {
 public:
  using vector_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>;
  using matrix_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, Eigen::Dynamic>;
  using joint_coordinate_t = switched_model::joint_coordinate_s_t<SCALAR_T>;

  QuadrupedPinocchioMappingTpl(switched_model::feet_array_t<std::size_t> feetMap);

  joint_coordinate_t mapJointOcs2ToPinocchio(const joint_coordinate_t& state) const;

 private:
  switched_model::feet_array_t<std::size_t> mapFeetOrderOcs2ToPinocchio_;
};

}  // namespace tpl
}  // namespace anymal

extern template class anymal::tpl::QuadrupedPinocchioMappingTpl<ocs2::scalar_t>;
extern template class anymal::tpl::QuadrupedPinocchioMappingTpl<ocs2::ad_scalar_t>;
