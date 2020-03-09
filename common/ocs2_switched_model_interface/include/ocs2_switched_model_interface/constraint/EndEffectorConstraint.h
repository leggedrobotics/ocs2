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

 protected:
  static constexpr char const* libFolderDir = "/tmp/ocs2";
  static constexpr size_t domain_dim_ = 1 + STATE_DIM + INPUT_DIM;
  static constexpr size_t range_dim_ = 3;

 public:
  using ad_interface_t = ocs2::CppAdInterface<scalar_t>;
  using ad_scalar_t = typename ad_interface_t::ad_scalar_t;
  using ad_dynamic_vector_t = typename ad_interface_t::ad_dynamic_vector_t;
  using dynamic_vector_t = typename ad_interface_t::dynamic_vector_t;

  using constraint_timeStateInput_matrix_t = Eigen::Matrix<scalar_t, -1, 1 + STATE_DIM + INPUT_DIM>;
  using timeStateInput_matrix_t = Eigen::Matrix<scalar_t, 1 + STATE_DIM + INPUT_DIM, 1 + STATE_DIM + INPUT_DIM>;
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  /*
   *
   * Children must first call intializeADInterface.
   *
   */
  EndEffectorConstraint(ocs2::ConstraintOrder constraintOrder, std::string eeConstraintName, int legNumber,
                        EndEffectorConstraintSettings settings)
      : BASE{constraintOrder},
        legNumber_{legNumber},
        settings_{std::move(settings)},
        libName_{eeConstraintName + std::to_string(legNumber_)},
        libFolder_{std::string{libFolderDir}, isAdInterfaceIntialized_ = false} {}

  EndEffectorConstraint(const EndEffectorConstraint& rhs)
      : BASE(rhs),
        legNumber_(rhs.legNumber_),
        settings_(rhs.settings_),
        libName_(rhs.libName_),
        libFolder_(rhs.libFolder_),
        adInterface_(new ad_interface_t(*rhs.adInterface_)) {}

  // EndEffectorConstraint* clone() const override { return new EndEffectorConstraint(*this); }

  virtual void configure(const EndEffectorConstraintSettings& settings) { settings_ = settings; };

  void initializeADInterface(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, bool generateModels) {
    setAdInterface(adComModel, adKinematicsModel);
    isAdInterfaceIntialized_ = true;
    switch (this->getOrder()) {
      case ocs2::ConstraintOrder::None:
        assert("__FILE__:__LINE__: Not implemented.");
      case ocs2::ConstraintOrder::Linear:
        if (generateModels) {
          adInterface_->createModels(ad_interface_t::ApproximationOrder::First, true);
        } else {
          adInterface_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::First, true);
        }
        break;
      case ocs2::ConstraintOrder::Quadratic:
        if (generateModels) {
          adInterface_->createModels(ad_interface_t::ApproximationOrder::Second, true);
        } else {
          adInterface_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::Second, true);
        }
        break;
    }
  }
  size_t getNumConstraints(scalar_t time) const override { return settings_.A().rows(); };

  // virtual scalar_array_t getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const = 0;

  virtual LinearApproximation_t getLinearApproximation(scalar_t time, const state_vector_t& state,
                                                       const input_vector_t& input) const override {
    // Assemble input
    dynamic_vector_t tapedInput(1 + STATE_DIM + INPUT_DIM);
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

  virtual QuadraticApproximation_t getQuadraticApproximation(scalar_t time, const state_vector_t& state,
                                                             const input_vector_t& input) const override {
    // Assemble input
    dynamic_vector_t tapedInput(1 + STATE_DIM + INPUT_DIM);
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

 private:
  virtual void setAdInterface(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel){
      /*
       *   // Function to differentiate
       * auto adfunc = [this, &adComModel, &adKinematicsModel](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) {
       *     this->adFootVelocity(adComModel, adKinematicsModel, x, y);
       * };
       *
       *   adInterface_.reset(new ad_interface_t(adfunc, BASE::range_dim_, BASE::domain_dim_, libName_, libFolder_));
       * };
       */
  };

 protected:
  int legNumber_;
  EndEffectorConstraintSettings settings_;
  std::string libName_;
  std::string libFolder_;
  std::unique_ptr<ad_interface_t> adInterface_;

 private:
  bool isAdInterfaceIntialized_;
};

}  // namespace switched_model
