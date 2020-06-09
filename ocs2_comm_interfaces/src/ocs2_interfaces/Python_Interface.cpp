#pragma once

#include <ocs2_comm_interfaces/ocs2_interfaces/Python_Interface.h>
#include <Eigen/SVD>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PythonInterface::PythonInterface(bool async)
    : run_mpc_async_(async), run_mpc_done_(false), run_mpc_requested_(false), shutdown_requested_(false) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PythonInterface::~PythonInterface() {
  if (run_mpc_async_) {
    {
      std::lock_guard<std::mutex> lock(run_mpc_mutex_);
      shutdown_requested_ = true;
    }
    run_mpc_cv_.notify_one();
    run_mpc_worker_.join();
    std::cerr << "Successfully joined MPC worker thread" << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::reset(CostDesiredTrajectories targetTrajectories) {
  targetTrajectories_ = std::move(targetTrajectories);
  mpcMrtInterface_->resetMpcNode(targetTrajectories_);
  cost_->setCostDesiredTrajectoriesPtr(&targetTrajectories_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::init(const RobotInterface& robotInterface, std::unique_ptr<MPC_BASE> mpcPtr) {
  if (!mpcPtr) {
    throw std::runtime_error("[PythonInterface] Mpc pointer must be initialized before passing to the Python interface.");
  }
  mpcPtr_ = std::move(mpcPtr);

  mpcMrtInterface_.reset(new MPC_MRT_Interface(*mpcPtr_));

  dynamics_.reset(robotInterface.getDynamics().clone());
  dynamicsDerivatives_.reset(robotInterface.getDynamicsDerivatives().clone());
  cost_.reset(robotInterface.getCost().clone());
  if (robotInterface.getConstraintPtr()) {
    constraints_.reset(robotInterface.getConstraintPtr()->clone());
  }

  if (run_mpc_async_) {
    run_mpc_worker_ = std::thread{&PythonInterface::runMpcAsync, this};
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::setObservation(double t, Eigen::Ref<const vector_t> x) {
  SystemObservation observation;
  observation.time() = t;
  observation.state() = x;
  mpcMrtInterface_->setCurrentObservation(observation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::setTargetTrajectories(CostDesiredTrajectories targetTrajectories) {
  targetTrajectories_ = std::move(targetTrajectories);
  cost_->setCostDesiredTrajectoriesPtr(&targetTrajectories_);
  mpcMrtInterface_->setTargetTrajectories(targetTrajectories_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::advanceMpc() {
  if (run_mpc_async_) {
    run_mpc_requested_ = true;
    run_mpc_cv_.notify_one();
  } else {
    mpcMrtInterface_->advanceMpc();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::runMpcAsync() {
  std::cerr << "runMpcAsync started in thread " << std::this_thread::get_id() << std::endl;
  while (true) {
    std::unique_lock<std::mutex> lk(run_mpc_mutex_);
    run_mpc_cv_.wait(lk, [this] { return this->run_mpc_requested_ or this->shutdown_requested_; });
    if (shutdown_requested_) {
      return;
    }
    run_mpc_requested_ = false;
    mpcMrtInterface_->advanceMpc();
    run_mpc_done_ = true;
    lk.unlock();
    run_mpc_cv_.notify_one();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::getMpcSolution(scalar_array_t& t, vector_array_t& x, vector_array_t& u) {
  if (run_mpc_async_) {
    // make sure MPC is done before we continue
    std::unique_lock<std::mutex> lk(run_mpc_mutex_);
    run_mpc_cv_.wait(lk, [this] { return this->run_mpc_done_; });
    run_mpc_done_ = false;
    lk.unlock();
  }

  mpcMrtInterface_->updatePolicy();

  t = mpcMrtInterface_->getPolicy().timeTrajectory_;
  x = mpcMrtInterface_->getPolicy().stateTrajectory_;
  u = mpcMrtInterface_->getPolicy().inputTrajectory_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t PythonInterface::getLinearFeedbackGain(scalar_t time) {
  matrix_t K;
  mpcMrtInterface_->getLinearFeedbackGain(time, K);
  return K;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PythonInterface::computeFlowMap(double t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
  vector_t dxdt;
  dynamics_->computeFlowMap(t, x, u, dxdt);
  return dxdt;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::setFlowMapDerivativeStateAndControl(double t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
  dynamicsDerivatives_->setCurrentStateAndControl(t, x, u);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t PythonInterface::computeFlowMapDerivativeState() {
  matrix_t A;
  dynamicsDerivatives_->getFlowMapDerivativeState(A);
  return A;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t PythonInterface::computeFlowMapDerivativeInput() {
  matrix_t B;
  dynamicsDerivatives_->getFlowMapDerivativeInput(B);
  return B;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
double PythonInterface::getIntermediateCost(double t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  scalar_t L;
  cost_->getIntermediateCost(L);

  if (constraints_) {
    constraints_->setCurrentStateAndControl(t, x, u);
    if (constraints_->numInequalityConstraint(t) > 0) {
      scalar_array_t h;
      constraints_->getInequalityConstraint(h);
      L += penalty_->getPenaltyCost(h);
    }
  }

  return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PythonInterface::getIntermediateCostDerivativeState(double t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  vector_t dLdx;
  cost_->getIntermediateCostDerivativeState(dLdx);

  if (constraints_) {
    constraints_->setCurrentStateAndControl(t, x, u);
    if (constraints_->numInequalityConstraint(t) > 0) {
      scalar_array_t h;
      constraints_->getInequalityConstraint(h);
      vector_array_t dhdxFixedSize;
      constraints_->getInequalityConstraintDerivativesState(dhdxFixedSize);
      // TODO: delete this
      vector_array_t dhdx;
      dhdx.reserve(dhdxFixedSize.size());
      for (const auto& v : dhdxFixedSize) {
        dhdx.emplace_back(v);
      }
      dLdx += penalty_->getPenaltyCostDerivativeState(h, dhdx);
    }
  }

  return dLdx;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PythonInterface::getIntermediateCostDerivativeInput(double t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  vector_t dLdu;
  cost_->getIntermediateCostDerivativeInput(dLdu);

  if (constraints_) {
    constraints_->setCurrentStateAndControl(t, x, u);
    if (constraints_->numInequalityConstraint(t) > 0) {
      scalar_array_t h;
      constraints_->getInequalityConstraint(h);
      vector_array_t dhduFixedSize;
      constraints_->getInequalityConstraintDerivativesInput(dhduFixedSize);
      // TODO: delete this
      vector_array_t dhdu;
      dhdu.reserve(dhduFixedSize.size());
      for (const auto& v : dhduFixedSize) {
        dhdu.emplace_back(v);
      }
      dLdu += penalty_->getPenaltyCostDerivativeInput(h, dhdu);
    }
  }

  return dLdu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t PythonInterface::getIntermediateCostSecondDerivativeInput(double t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
  cost_->setCurrentStateAndControl(t, x, u);
  matrix_t ddLduu;
  cost_->getIntermediateCostSecondDerivativeInput(ddLduu);

  if (constraints_) {
    constraints_->setCurrentStateAndControl(t, x, u);
    if (constraints_->numInequalityConstraint(t) > 0) {
      scalar_array_t h;
      constraints_->getInequalityConstraint(h);
      vector_array_t dhduFixedSize;
      constraints_->getInequalityConstraintDerivativesInput(dhduFixedSize);
      input_matrix_array_t ddhduuFixedSize;
      constraints_->getInequalityConstraintSecondDerivativesInput(ddhduuFixedSize);
      // TODO: delete this
      vector_array_t dhdu;
      dhdu.reserve(dhduFixedSize.size());
      dynamic_matrix_array_t ddhduu;
      ddhduu.reserve(dhduFixedSize.size());
      for (int i = 0; i < dhduFixedSize.size(); i++) {
        dhdu.emplace_back(dhduFixedSize[i]);
        ddhduu.emplace_back(ddhduuFixedSize[i]);
      }
      ddLduu += penalty_->getPenaltyCostSecondDerivativeInput(h, dhdu, ddhduu);
    }
  }

  return ddLduu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
double PythonInterface::getValueFunction(double t, Eigen::Ref<const vector_t> x) {
  return mpcMrtInterface_->getValueFunction(t, x);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PythonInterface::getValueFunctionStateDerivative(double t, Eigen::Ref<const vector_t> x) {
  vector_t dVdx;
  mpcMrtInterface_->getValueFunctionStateDerivative(t, x, dVdx);
  return dVdx;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PythonInterface::getStateInputConstraint(double t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
  if (!constraints_) {
    throw std::runtime_error("Cannot getStateInputConstraint if system has no constraints.");
  }
  constraints_->setCurrentStateAndControl(t, x, u);
  vector_t e;
  constraints_->getConstraint1(e);
  return e.head(constraints_->numStateInputConstraint(t));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t PythonInterface::getStateInputConstraintDerivativeControl(double t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
  if (!constraints_) {
    throw std::runtime_error("Cannot getStateInputConstraint if system has no constraints.");
  }
  constraints_->setCurrentStateAndControl(t, x, u);
  matrix_t D;
  constraints_->getConstraint1DerivativesControl(D);
  return D.topRows(constraints_->numStateInputConstraint(t));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PythonInterface::getStateInputConstraintLagrangian(double t, Eigen::Ref<const vector_t> x) {
  vector_t nu;
  //  mpcMrtInterface_->getStateInputConstraintLagrangian(t, x, nu);
  //  return nu;

  vector_t zero_u;
  zero_u.setZero();
  matrix_t Dm = getStateInputConstraintDerivativeControl(t, x, zero_u);

  vector_t c = getStateInputConstraint(t, x, zero_u);

  matrix_t R = getIntermediateCostSecondDerivativeInput(t, x, zero_u);
  vector_t r = getIntermediateCostDerivativeInput(t, x, zero_u);
  matrix_t RinvChol;
  LinearAlgebra::computeInverseMatrixUUT(R, RinvChol);
  matrix_t DmDager, DdaggerT_R_Ddagger_Chol, RmInvConstrainedChol;
  ocs2::LinearAlgebra::computeConstraintProjection(Dm, RinvChol, DmDager, DdaggerT_R_Ddagger_Chol, RmInvConstrainedChol);

  dynamicsDerivatives_->setCurrentStateAndControl(t, x, zero_u);
  matrix_t B;
  dynamicsDerivatives_->getFlowMapDerivativeInput(B);

  vector_t costate = getValueFunctionStateDerivative(t, x);

  nu = DmDager.transpose() * (R * DmDager * c - r - B.transpose() * costate);

  return nu;
}

}  // namespace ocs2
