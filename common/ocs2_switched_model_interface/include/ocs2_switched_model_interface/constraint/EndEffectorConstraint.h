#pragma once

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_switched_model_interface/constraint/ConstraintTerm.h>
#include <Eigen/Core>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"

namespace switched_model {

struct EndEffectorConstraintSettings {
  // Constraints are described by A() * v_base + b()

 private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::MatrixXd data;

 public:
  Eigen::Ref<Eigen::VectorXd> b() { return data.rightCols<1>(); };
  Eigen::Ref<Eigen::MatrixXd> A() { return data.leftCols(data.cols() - 1); };
  const Eigen::Ref<const Eigen::VectorXd> b() const { return data.rightCols<1>(); };
  const Eigen::Ref<const Eigen::MatrixXd> A() const { return data.leftCols(data.cols() - 1); };
  void resize(size_t rows, size_t cols) { data.resize(rows, cols + 1); };

  EndEffectorConstraintSettings() = default;
  EndEffectorConstraintSettings(size_t rows, size_t cols) { resize(rows, cols); };
};

template <class Derived>
class EndEffectorConstraint : public ocs2::ConstraintTerm<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ocs2::ConstraintTerm<STATE_DIM, INPUT_DIM>;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_t;
  using typename BASE::LinearApproximation_t;
  using typename BASE::QuadraticApproximation_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;

 private:
  static constexpr size_t domain_dim_ = 1 + STATE_DIM + INPUT_DIM;
  static constexpr size_t range_dim_ = 3;

 public:
  using ad_interface_t = ocs2::CppAdInterface<scalar_t>;
  using ad_scalar_t = typename ad_interface_t::ad_scalar_t;
  using ad_dynamic_vector_t = typename ad_interface_t::ad_dynamic_vector_t;
  using dynamic_vector_t = typename ad_interface_t::dynamic_vector_t;

  using constraint_timeStateInput_matrix_t = Eigen::Matrix<scalar_t, -1, domain_dim_>;
  using timeStateInput_matrix_t = Eigen::Matrix<scalar_t, domain_dim_, domain_dim_>;
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  EndEffectorConstraint(ocs2::ConstraintOrder constraintOrder, std::string eeConstraintName, int legNumber,
                        EndEffectorConstraintSettings settings, ad_interface_t& ad_interface, bool generateModels)
      : BASE(constraintOrder),
        legNumber_(legNumber),
        settings_(std::move(settings)),
        libNamePrefix_(eeConstraintName),
        libName_(eeConstraintName + std::to_string(legNumber)),
        libFolder_("/tmp/ocs2"),
        adInterface_(new ad_interface_t(ad_interface)) {
#if NDEBUG
    assert(adInterface_);
#endif
    auto order = static_cast<ad_interface_t::ApproximationOrder>(constraintOrder);
    if (generateModels) {
      adInterface_->createModels(order, true);
    } else {
      adInterface_->loadModelsIfAvailable(order, true);
    }
  }

  EndEffectorConstraint(ocs2::ConstraintOrder constraintOrder, std::string eeConstraintName, int legNumber,
                        EndEffectorConstraintSettings settings, ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel,
                        bool generateModels)
      : BASE(constraintOrder),
        legNumber_(legNumber),
        settings_(std::move(settings)),
        libNamePrefix_(eeConstraintName),
        libName_(eeConstraintName + std::to_string(legNumber)),
        libFolder_("/tmp/ocs2") {
    setAdInterface(adComModel, adKinematicsModel);
#if NDEBUG
    assert(adInterface_);
#endif
    auto order = static_cast<ad_interface_t::ApproximationOrder>(constraintOrder);
    if (generateModels) {
      adInterface_->createModels(order, true);
    } else {
      adInterface_->loadModelsIfAvailable(order, true);
    }
  }

  //! Note: Since the constructors are based on a copy we do not or regenerate/generate the models
  EndEffectorConstraint(const EndEffectorConstraint& rhs)
      : EndEffectorConstraint{rhs.getOrder(), rhs.libNamePrefix_, rhs.legNumber_, rhs.settings_, *rhs.adInterface_, false} {};

  //! Note: Since the constructors are based on a copy we do not or regenerate/generate the models
  EndEffectorConstraint* clone() const override { return new EndEffectorConstraint(*this); }

  const ad_interface_t& getAdInterface() const { return *adInterface_; };

  virtual void configure(const EndEffectorConstraintSettings& settings) { settings_ = settings; };

  size_t getNumConstraints(scalar_t time) const override { return settings_.A().rows(); };

  scalar_array_t getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    // Assemble input
    dynamic_vector_t tapedInput(domain_dim_);
    tapedInput << time, state, input;

    // Compute constraints
    const dynamic_vector_t funcVal = adInterface_->getFunctionValue(tapedInput);

    // Change to std::vector
    scalar_array_t constraintValue;
    Eigen::VectorXd values = settings_.A() * funcVal + settings_.b();
    for (int i = 0; i < settings_.A().rows(); i++) {
      constraintValue.emplace_back(values[i]);
    }
    return constraintValue;
  };

  LinearApproximation_t getLinearApproximation(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    // Assemble input
    dynamic_vector_t tapedInput(domain_dim_);
    tapedInput << time, state, input;

    // Compute end effector velocity and derivatives
    constraint_timeStateInput_matrix_t stateInputJacobian = adInterface_->getJacobian(tapedInput);

    // Collect constraint terms
    auto dhdx = stateInputJacobian.template middleCols<STATE_DIM>(1);
    auto dhdu = stateInputJacobian.template rightCols<INPUT_DIM>();

    // Convert to output format
    LinearApproximation_t linearApproximation;
    linearApproximation.constraintValues = getValue(time, state, input);
    for (int i = 0; i < settings_.A().rows(); i++) {
      linearApproximation.derivativeState.emplace_back(settings_.A().row(i) * dhdx);
      linearApproximation.derivativeInput.emplace_back(settings_.A().row(i) * dhdu);
    }
    return linearApproximation;
  }

  QuadraticApproximation_t getQuadraticApproximation(scalar_t time, const state_vector_t& state,
                                                     const input_vector_t& input) const override {
    // Assemble input
    dynamic_vector_t tapedInput(domain_dim_);
    tapedInput << time, state, input;

    // Convert to output format
    QuadraticApproximation_t quadraticApproximation;
    auto linearApproximation = getLinearApproximation(time, state, input);
    quadraticApproximation.constraintValues = std::move(linearApproximation.constraintValues);
    quadraticApproximation.derivativeState = std::move(linearApproximation.derivativeState);
    quadraticApproximation.derivativeInput = std::move(linearApproximation.derivativeInput);
    for (int i = 0; i < settings_.A().rows(); i++) {
      timeStateInput_matrix_t weightedHessian = adInterface_->getHessian(settings_.A().row(i), tapedInput);

      quadraticApproximation.secondDerivativesState.emplace_back(weightedHessian.block(1, 1, STATE_DIM, STATE_DIM));
      quadraticApproximation.secondDerivativesInput.emplace_back(weightedHessian.block(1 + STATE_DIM, 1 + STATE_DIM, INPUT_DIM, INPUT_DIM));
      quadraticApproximation.derivativesInputState.emplace_back(weightedHessian.block(1 + STATE_DIM, 1, INPUT_DIM, STATE_DIM));
    }
    return quadraticApproximation;
  }

 protected:
  // Function to differentiate
  // template <typename... Ts>
  // virtual void adfunc(Ts... args) = 0;
  // std::function adfunc; = [this, &adComModel, &adKinematicsModel](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) {
  //   this->adFootVelocity(adComModel, adKinematicsModel, x, y);
  EndEffectorConstraintSettings settings_;
  int legNumber_;

 private:
  void setAdInterface(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel) {
    auto adfunc = [this, &adComModel, &adKinematicsModel](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) {
      static_cast<Derived&>(*this).adfunc(adComModel, adKinematicsModel, x, y);
    };
    adInterface_.reset(new ad_interface_t(adfunc, range_dim_, domain_dim_, libName_, libFolder_));
  };

  std::string libName_;
  std::string libNamePrefix_;  // needed for copy constructor
  std::string libFolder_;
  std::unique_ptr<ad_interface_t> adInterface_;
};  // namespace switched_model

}  // namespace switched_model
