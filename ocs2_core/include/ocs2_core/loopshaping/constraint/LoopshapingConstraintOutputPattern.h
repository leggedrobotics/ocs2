

#ifndef OCS2_LOOPSHAPINGCONSTRAINTOUTPUTPATTERN_H
#define OCS2_LOOPSHAPINGCONSTRAINTOUTPUTPATTERN_H

namespace ocs2 {
template <size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM>
class LoopshapingConstraintOutputPattern final
    : public LoopshapingConstraint<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE =
      LoopshapingConstraint<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;

  using typename BASE::constraint1_input_matrix_t;
  using typename BASE::constraint1_state_matrix_t;
  using typename BASE::constraint1_vector_t;
  using typename BASE::constraint2_vector_t;
  using typename BASE::input_matrix_array_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_array_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::state_matrix_array_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;

  using typename BASE::SYSTEM_CONSTRAINT;
  using typename BASE::system_constraint1_input_matrix_t;
  using typename BASE::system_constraint1_state_matrix_t;
  using typename BASE::system_constraint2_state_matrix_t;
  using typename BASE::system_contraint1_vector_t;
  using typename BASE::system_contraint2_vector_t;
  using typename BASE::system_input_matrix_array_t;
  using typename BASE::system_input_state_matrix_array_t;
  using typename BASE::system_input_vector_array_t;
  using typename BASE::system_input_vector_t;
  using typename BASE::system_scalar_array_t;
  using typename BASE::system_state_matrix_array_t;
  using typename BASE::system_state_vector_array_t;
  using typename BASE::system_state_vector_t;

  using typename BASE::filter_input_vector_t;
  using typename BASE::filter_state_vector_t;

  explicit LoopshapingConstraintOutputPattern(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(std::move(loopshapingDefinition)){};

  LoopshapingConstraintOutputPattern(const SYSTEM_CONSTRAINT& systemConstraint,
                                     std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemConstraint, std::move(loopshapingDefinition)){};

  virtual ~LoopshapingConstraintOutputPattern() = default;

  LoopshapingConstraintOutputPattern(const LoopshapingConstraintOutputPattern& obj) = default;

  LoopshapingConstraintOutputPattern* clone() const override { return new LoopshapingConstraintOutputPattern(*this); };

  void getInequalityConstraintDerivativesState(state_vector_array_t& dhdx) override {
    this->computeSystemInequalityConstraintDerivatives();
    dhdx.clear();
    if (systemConstraint_) {
      dhdx.resize(system_dhdx.size());
      for (size_t i = 0; i < system_dhdx.size(); i++) {
        dhdx[i].head(system_dhdx[i].size()) = system_dhdx[i];
        dhdx[i].tail(FILTER_STATE_DIM).setZero();
      }
    }
  };

  void getInequalityConstraintDerivativesInput(input_vector_array_t& dhdu) override {
    this->computeSystemInequalityConstraintDerivatives();
    dhdu.clear();
    if (systemConstraint_) {
      dhdu.resize(system_dhdu.size());
      for (size_t i = 0; i < system_dhdu.size(); i++) {
        dhdu[i].head(system_dhdu[i].size()) = system_dhdu[i];
      }
    }
  };

  void getInequalityConstraintSecondDerivativesState(state_matrix_array_t& ddhdxdx) override {
    this->computeSystemInequalityConstraintDerivatives();
    ddhdxdx.clear();
    if (systemConstraint_) {
      ddhdxdx.resize(system_ddhdxdx.size());
      for (size_t i = 0; i < system_ddhdxdx.size(); i++) {
        ddhdxdx[i].block(0, 0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = system_ddhdxdx[i];
        ddhdxdx[i].block(0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM).setZero();
        ddhdxdx[i].block(SYSTEM_STATE_DIM, 0, FILTER_STATE_DIM, SYSTEM_STATE_DIM).setZero();
        ddhdxdx[i].block(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM, FILTER_STATE_DIM).setZero();
      }
    }
  };

  void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t& ddhdudu) override {
    this->computeSystemInequalityConstraintDerivatives();
    ddhdudu.clear();
    if (systemConstraint_) {
      ddhdudu.resize(system_ddhdudu.size());
      for (size_t i = 0; i < system_ddhdudu.size(); i++) {
        ddhdudu[i].block(0, 0, SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM) = system_ddhdudu[i];
      }
    }
  };

  void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t& ddhdudx) override {
    this->computeSystemInequalityConstraintDerivatives();
    ddhdudx.clear();
    if (systemConstraint_) {
      ddhdudx.resize(system_ddhdudx.size());
      for (size_t i = 0; i < system_ddhdudx.size(); i++) {
        ddhdudx[i].block(0, 0, SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM) = system_ddhdudx[i];
        ddhdudx[i].block(0, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM).setZero();
      }
    }
  };

 protected:
  using BASE::loopshapingDefinition_;
  using BASE::systemConstraint_;

  using BASE::t_;
  using BASE::u_filter_;
  using BASE::u_system_;
  using BASE::x_filter_;
  using BASE::x_system_;

  using BASE::system_ddhdudu;
  using BASE::system_ddhdudx;
  using BASE::system_ddhdxdx;
  using BASE::system_dhdu;
  using BASE::system_dhdx;

 private:
};
}  // namespace ocs2

#endif  // OCS2_LOOPSHAPINGCONSTRAINTOUTPUTPATTERN_H
