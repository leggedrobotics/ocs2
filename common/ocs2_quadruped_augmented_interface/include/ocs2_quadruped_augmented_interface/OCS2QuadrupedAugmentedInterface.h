//
// Created by ruben on 23.07.18.
//

#ifndef OCS2_OCS_QUADRUPEDAUGMENTEDINTERFACE_H
#define OCS2_OCS_QUADRUPEDAUGMENTEDINTERFACE_H

#include <ocs2_core/loopshaping/Loopshaping.h>
#include <ocs2_quadruped_interface/OCS2QuadrupedInterface.h>

namespace switched_model {

template <size_t STATE_DIM, size_t INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM, size_t JOINT_COORD_SIZE>
class OCS2QuadrupedAugmentedInterface : public OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>;
  using kinematic_model_t = typename BASE::kinematic_model_t;
  using com_model_t = typename BASE::com_model_t;
  using state_estimator_t = typename BASE::state_estimator_t;
  using rbd_state_vector_t = typename BASE::rbd_state_vector_t;
  using state_vector_t = typename BASE::state_vector_t;
  using input_vector_t = typename BASE::input_vector_t;

  using typename BASE::rollout_base_t;
  using typename BASE::time_triggered_rollout_t;

  using filter_state_t = Eigen::Matrix<double, FILTER_STATE_DIM, 1>;

  using system_state_matrix_t = Eigen::Matrix<double, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM>;
  using system_state_vector_t = Eigen::Matrix<double, SYSTEM_STATE_DIM, 1>;
  using system_input_matrix_t = Eigen::Matrix<double, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM>;

  OCS2QuadrupedAugmentedInterface(const kinematic_model_t& kinematicModel, const com_model_t& comModel,
                                  const std::string& pathToConfigFolder)
      : BASE(kinematicModel, comModel, pathToConfigFolder) {
    loopshapingDefinition_ = ocs2::loopshaping_property_tree::load(pathToConfigFolder + "/loopshaping.info");
    loopshapingDefinition_->print();

    // cost function components
    Q_system_ = BASE::Q_.template block<SYSTEM_STATE_DIM, SYSTEM_STATE_DIM>(0, 0);
    Q_system_final_ = BASE::QFinal_.template block<SYSTEM_STATE_DIM, SYSTEM_STATE_DIM>(0, 0);
    R_system_ = BASE::R_.template block<SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM>(0, 0);
    x_system_final_ = BASE::xFinal_.template segment<SYSTEM_STATE_DIM>(0);
  };

  virtual ~OCS2QuadrupedAugmentedInterface() = default;

  void computeSwitchedModelState(const rbd_state_vector_t& rbdState, state_vector_t& extended_state) override {
    BASE::computeSwitchedModelState(rbdState, extended_state);
    extended_state.template segment<FILTER_STATE_DIM>(SYSTEM_STATE_DIM).setZero();
  };

  void computeSwitchedModelState(const rbd_state_vector_t& rbdState, const filter_state_t& filterState, state_vector_t& extended_state) {
    BASE::computeSwitchedModelState(rbdState, extended_state);
    extended_state.template segment<FILTER_STATE_DIM>(SYSTEM_STATE_DIM) = filterState;
  };

 protected:
  std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition_;

  system_state_matrix_t Q_system_, Q_system_final_;
  system_input_matrix_t R_system_;
  system_state_vector_t x_system_final_;
};
}  // namespace switched_model

#endif  // OCS2_OCS_QUADRUPEDAUGMENTEDINTERFACE_H
