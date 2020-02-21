

#ifndef OCS2_LOOPSHAPINGCONSTRAINTINPUTPATTERN_H
#define OCS2_LOOPSHAPINGCONSTRAINTINPUTPATTERN_H

namespace ocs2 {
template <size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM>
class LoopshapingConstraintInputPattern final
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

  explicit LoopshapingConstraintInputPattern(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(std::move(loopshapingDefinition)){};

  LoopshapingConstraintInputPattern(const SYSTEM_CONSTRAINT& systemConstraint, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemConstraint, std::move(loopshapingDefinition)){};

  virtual ~LoopshapingConstraintInputPattern() = default;

  LoopshapingConstraintInputPattern(const LoopshapingConstraintInputPattern& obj) = default;

  LoopshapingConstraintInputPattern* clone() const override { return new LoopshapingConstraintInputPattern(*this); };

  void getInequalityConstraintDerivativesState(state_vector_array_t& dhdx) override {
    this->computeSystemInequalityConstraintDerivatives();
    dhdx.clear();
    if (systemConstraint_) {
      dhdx.resize(system_dhdx.size());
      for (size_t i = 0; i < system_dhdx.size(); i++) {
        dhdx[i].head(SYSTEM_STATE_DIM) = system_dhdx[i];
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
        dhdu[i].head(SYSTEM_INPUT_DIM) = system_dhdu[i];
        dhdu[i].tail(FILTER_INPUT_DIM).setZero();
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
        ddhdudu[i].block(0, SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM, FILTER_INPUT_DIM).setZero();
        ddhdudu[i].block(SYSTEM_INPUT_DIM, 0, FILTER_INPUT_DIM, SYSTEM_INPUT_DIM).setZero();
        ddhdudu[i].block(SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM, FILTER_INPUT_DIM, FILTER_INPUT_DIM).setZero();
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
        ddhdudx[i].block(SYSTEM_INPUT_DIM, 0, FILTER_INPUT_DIM, SYSTEM_STATE_DIM).setZero();
        ddhdudx[i].block(SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM, FILTER_INPUT_DIM, FILTER_STATE_DIM).setZero();
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
  size_t addNumStateInputConstraint(size_t numSystemStateInputConstraints) override {
    return numSystemStateInputConstraints + SYSTEM_INPUT_DIM;
  };

  void appendConstraint1(size_t numSystemStateInputConstraints, constraint1_vector_t& e) override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();

    e.template segment<SYSTEM_INPUT_DIM>(numSystemStateInputConstraints) =
        s_filter.getC() * x_filter_ + s_filter.getD() * u_filter_ - u_system_;
  };

  void appendConstraint1DerivativeState(size_t numSystemStateInputConstraints, constraint1_state_matrix_t& C) override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    C.block(numSystemStateInputConstraints, 0, SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM).setZero();
    C.block(numSystemStateInputConstraints, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM) = s_filter.getC();
  };

  void appendConstraint1DerivativeControl(size_t numSystemStateInputConstraints, constraint1_input_matrix_t& D) override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    D.block(numSystemStateInputConstraints, 0, SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM) =
        -Eigen::Matrix<scalar_t, SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM>::Identity();
    D.block(numSystemStateInputConstraints, SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM, FILTER_INPUT_DIM) = s_filter.getD();
  };
};
}  // namespace ocs2

#endif  // OCS2_LOOPSHAPINGCONSTRAINTINPUTPATTERN_H
