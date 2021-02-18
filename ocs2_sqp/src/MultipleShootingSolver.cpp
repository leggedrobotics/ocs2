//
// Created by rgrandia on 09.11.20.
//

#include "ocs2_sqp/MultipleShootingSolver.h"
#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <Eigen/QR>
#include <chrono>
#include <iostream>

// scalar_t is double
// matrix_t is Eigen::MatrixXd
// matrix_array_t is std::vector<Eigen::MatrixXd>

namespace ocs2 {

MultipleShootingSolver::MultipleShootingSolver(MultipleShootingSolverSettings settings, const SystemDynamicsBase* systemDynamicsPtr,
                                               const CostFunctionBase* costFunctionPtr, const ConstraintBase* constraintPtr)
    : SolverBase(),
      systemDynamicsPtr_(systemDynamicsPtr->clone()),
      costFunctionPtr_(costFunctionPtr->clone()),
      constraintPtr_(constraintPtr->clone()),
      settings_(std::move(settings)) {
  std::cout << "creating multiple shooting solver\n";
}

void MultipleShootingSolver::reset() {
  // SolverBase::reset();
  // there is no Solve_BASE::reset() function. One can see GaussNewtonDDP.h. The reset function there only clears some variables of the
  // solver itself. additional reset
  std::cout << "resetting\n";
}

void MultipleShootingSolver::runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime,
                                     const scalar_array_t& partitioningTimes) {
  std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
  std::cerr << "\n+++++++++++++ SQP solver is initialized ++++++++++++++";
  std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  // STATE_DIM & INPUT_DIM are fixed, can be retrieved from the Q, R matrices from task.info

  // ignore partitioningTimes

  // Initialize cost
  costFunctionPtr_->setCostDesiredTrajectoriesPtr(&this->getCostDesiredTrajectories());

  // Solve the problem.

  // EQ_CONSTRAINT_DIM is not fixed in different modes, get them
  getInfoFromModeSchedule(initTime, finalTime, *constraintPtr_);

  // setup dimension for the coming OCP
  setupDimension(*constraintPtr_);

  // Initialize the state and input containers
  std::vector<ocs2::vector_t> x(settings_.N_real + 1, ocs2::vector_t(settings_.n_state));
  std::vector<ocs2::vector_t> u(settings_.N_real, ocs2::vector_t(settings_.n_input));

  // need to initialize the u with all zero except contact force along z-axis
  // here 110 corresponds to the weight of ANYmal, this figure should not be hardcoded
  // this is not necessary, since zero input now works
  vector_t stanceInput = vector_t::Zero(settings_.n_input);
  // stanceInput(2) = 110;
  // stanceInput(5) = 110;
  // stanceInput(8) = 110;
  // stanceInput(11) = 110;

  if (!settings_.initPrimalSol) {
    // for the first time, do the following init
    for (int i = 0; i < settings_.N_real; i++) {
      x[i] = initState;
      u[i] = stanceInput;
    }
    x[settings_.N_real] = initState;
    // std::cout << "using given initial state and stancing steady input\n";
  } else {
    // previous N_real may not be the same as the current one, so size mismatch might happen.
    // do linear interpolation here
    for (int i = 0; i < settings_.N_real; i++) {
      x[i] =
          LinearInterpolation::interpolate(settings_.trueEventTimes[i], primalSolution_.timeTrajectory_, primalSolution_.stateTrajectory_);
      u[i] =
          LinearInterpolation::interpolate(settings_.trueEventTimes[i], primalSolution_.timeTrajectory_, primalSolution_.inputTrajectory_);
    }
    x[settings_.N_real] = LinearInterpolation::interpolate(settings_.trueEventTimes[settings_.N_real], primalSolution_.timeTrajectory_,
                                                               primalSolution_.stateTrajectory_);
    // std::cout << "using past primal sol init\n";
  }
  settings_.initPrimalSol = true;

  scalar_t sqpTimeAll = 0.0;
  for (int i = 0; i < settings_.sqpIteration; i++) {
    std::cout << "SQP iteration " << i << ":\n";
    auto startSqpTime = std::chrono::steady_clock::now();
    setupCostDynamicsEqualityConstraint(*systemDynamicsPtr_, *costFunctionPtr_, *constraintPtr_, x, u, initState);

    vector_t delta_x0 = initState - x[0];
    std::vector<ocs2::vector_t> delta_x;
    std::vector<ocs2::vector_t> delta_u;
    std::tie(delta_x, delta_u) = getOCPSolution(delta_x0);

    // step size will be used in future // TODO here
    scalar_t deltaUnorm = 0.0;
    scalar_t deltaXnorm = 0.0;
    for (int i = 0; i < settings_.N_real; i++) {
      deltaXnorm += delta_x[i].norm();
      x[i] += delta_x[i];
      deltaUnorm += delta_u[i].norm();
      u[i] += delta_u[i];
    }
    x[settings_.N_real] += delta_x[settings_.N_real];

    auto endSqpTime = std::chrono::steady_clock::now();
    auto sqpIntervalTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endSqpTime - startSqpTime);
    scalar_t sqpTime = std::chrono::duration<scalar_t, std::milli>(sqpIntervalTime).count();
    sqpTimeAll += sqpTime;
    std::cout << "\tSQP total time in this iter: " << sqpTime << "[ms]." << std::endl;
    if (deltaUnorm < settings_.deltaTol && deltaXnorm < settings_.deltaTol) {
      std::cout << "exiting the loop earlier\n";
      break;
    }
  }
  std::cout << "Summary -- SQP time total: " << sqpTimeAll << "[ms]." << std::endl;

  // Fill PrimalSolution. time, state , input
  scalar_array_t timeTrajectory;
  vector_array_t stateTrajectory;
  vector_array_t inputTrajectory;
  timeTrajectory.resize(settings_.N_real);
  stateTrajectory.resize(settings_.N_real);
  inputTrajectory.resize(settings_.N_real);
  for (int i = 0; i < settings_.N_real; i++) {
    timeTrajectory[i] = settings_.trueEventTimes[i];
    stateTrajectory[i] = x[i];
    inputTrajectory[i] = u[i];
  }
  primalSolution_.timeTrajectory_ = timeTrajectory;
  primalSolution_.stateTrajectory_ = stateTrajectory;
  primalSolution_.inputTrajectory_ = inputTrajectory;
  primalSolution_.modeSchedule_ = this->getModeSchedule();
  primalSolution_.controllerPtr_.reset(new FeedforwardController(primalSolution_.timeTrajectory_, primalSolution_.inputTrajectory_));

  std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
  std::cerr << "\n+++++++++++++ SQP solver has terminated ++++++++++++++";
  std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
}


std::pair<std::vector<ocs2::vector_t>, std::vector<ocs2::vector_t>> MultipleShootingSolver::getOCPSolution(const vector_t& delta_x0) {
  HpipmInterface hpipmInterface(ocpSize);
  bool verbose = settings_.printSolverStatus || settings_.printSolverStatistics;
  std::vector<ocs2::vector_t> deltaXSol;
  std::vector<ocs2::vector_t> deltaUSol;
  
  hpipm_timer timer;
  hpipm_tic(&timer);
  hpipmInterface.solve(delta_x0, dynamics_, cost_, &constraints_, deltaXSol, deltaUSol, verbose);
  scalar_t time_ipm = hpipm_toc(&timer);
  printf("\tSolution time usage: %e [ms]\n", time_ipm * 1e3);
  
  // retrieve deltaX, deltaUTilde
  auto startFillTime = std::chrono::steady_clock::now();
  
  // remap the tilde delta u to real delta u
  if (settings_.constrained && settings_.qr_decomp) {
    for (int i = 0; i < settings_.N_real; i++) {
      deltaUSol[i] =
          Q2_data[i] * deltaUSol[i] - Q1_data[i] * (R1_data[i].transpose()).inverse() * (C_data[i] * deltaXSol[i] + e_data[i]);
    }
  }

  auto endFillTime = std::chrono::steady_clock::now();
  auto fillIntervalTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endFillTime - startFillTime);
  scalar_t fillTime = std::chrono::duration<scalar_t, std::milli>(fillIntervalTime).count();
  std::cout << "\tRetrieve time usage: " << fillTime << "[ms]." << std::endl;

  return {deltaXSol, deltaUSol};
}

void MultipleShootingSolver::setupCostDynamicsEqualityConstraint(SystemDynamicsBase& systemDynamicsObj, CostFunctionBase& costFunctionObj,
                                                                 ConstraintBase& constraintObj, const std::vector<ocs2::vector_t>& x, const std::vector<ocs2::vector_t>& u,
                                                                 const vector_t& initState) {
  // x of shape (n_state, N_real + 1), n = n_state;
  // u of shape (n_input, N_real), m = n_input;
  // Matrix pi of shape (n_state, N), this is not used temporarily
  // N_real is the horizon length
  // Vector initState of shape (n_state, 1)

  auto startSetupTime = std::chrono::steady_clock::now();

  scalar_t operTime, delta_t_;

  // NOTE: This setup will take longer time if in debug mode
  // the most likely reason is that Eigen QR decomposition and later on calculations are not done in the optimal way in debug mode
  // everything works fine when changed to release mode
  dynamics_.resize(settings_.N_real);
  cost_.resize(settings_.N_real + 1);
  constraints_.resize(settings_.N_real + 1);
  for (int i = 0; i < settings_.N_real; i++) {
    operTime = settings_.trueEventTimes[i];
    delta_t_ = settings_.trueEventTimes[i + 1] - settings_.trueEventTimes[i];
    scalar_t delta_t_half_ = delta_t_ / 2.0;
    ocs2::VectorFunctionLinearApproximation k1 = systemDynamicsObj.linearApproximation(operTime, x[i], u[i]);
    ocs2::VectorFunctionLinearApproximation k2 =
        systemDynamicsObj.linearApproximation(operTime + delta_t_half_, x[i] + delta_t_half_ * k1.f, u[i]);
    ocs2::VectorFunctionLinearApproximation k3 =
        systemDynamicsObj.linearApproximation(operTime + delta_t_half_, x[i] + delta_t_half_ * k2.f, u[i]);
    ocs2::VectorFunctionLinearApproximation k4 =
        systemDynamicsObj.linearApproximation(operTime + delta_t_, x[i] + delta_t_ * k3.f, u[i]);
    // dx_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
    // A_{k} = Id + dt * dfdx
    // B_{k} = dt * dfdu
    // b_{k} = x_{n} + dt * f(x_{n},u_{n}) - x_{n+1}
    // TODO here: currently only one-step Euler integration, subject to modification to RK4
    matrix_t dk1dxk = k1.dfdx;
    matrix_t dk2dxk = k2.dfdx * (matrix_t::Identity(settings_.n_state, settings_.n_state) + delta_t_half_ * dk1dxk);
    matrix_t dk3dxk = k3.dfdx * (matrix_t::Identity(settings_.n_state, settings_.n_state) + delta_t_half_ * dk2dxk);
    matrix_t dk4dxk = k4.dfdx * (matrix_t::Identity(settings_.n_state, settings_.n_state) + delta_t_ * dk3dxk);
    dynamics_[i].dfdx = matrix_t::Identity(settings_.n_state, settings_.n_state) + (delta_t_ / 6.0) * (dk1dxk + 2 * dk2dxk + 2 * dk3dxk + dk4dxk);
    matrix_t dk1duk = k1.dfdu;
    matrix_t dk2duk = k2.dfdu + delta_t_half_ * k2.dfdx * dk1duk;
    matrix_t dk3duk = k3.dfdu + delta_t_half_ * k3.dfdx * dk2duk;
    matrix_t dk4duk = k4.dfdu + delta_t_ * k4.dfdx * dk3duk;
    matrix_t trueBMatrix =
        (delta_t_ / 6.0) * (dk1duk + 2 * dk2duk + 2 * dk3duk + dk4duk);              // <-- this should be the correct one, but not working
    dynamics_[i].dfdu = (delta_t_ / 6.0) * (k1.dfdu + 2 * k2.dfdu + 2 * k3.dfdu + k4.dfdu);  // <-- this is working but not the right one
    std::cout << "difference of B and true B " << (trueBMatrix - dynamics_[i].dfdu).norm() << std::endl;
    // dynamics_[i].dfdu = trueBMatrix;
    dynamics_[i].f = x[i] + (delta_t_ / 6.0) * (k1.f + 2 * k2.f + 2 * k3.f + k4.f) - x[i + 1];


    cost_[i] = costFunctionObj.costQuadraticApproximation(operTime, x[i], u[i]);

    if (settings_.constrained) {
      constraints_[i] = constraintObj.stateInputEqualityConstraintLinearApproximation(operTime, x[i], u[i]);
      // C_{k} * dx_{k} + D_{k} * du_{k} + e_{k} = 0
      // C_{k} = constraintApprox.dfdx
      // D_{k} = constraintApprox.dfdu
      // e_{k} = constraintApprox.f
      if (settings_.qr_decomp) {
        // handle equality constraints using QR decomposition
        matrix_t C = constraints_[i].dfdx;  // p x n
        matrix_t D = constraints_[i].dfdu;  // p x m
        vector_t e = constraints_[i].f;     // p x 1
        matrix_t D_transpose = D.transpose();  // m x p
        Eigen::HouseholderQR<matrix_t> qr(D_transpose);
        matrix_t Q = qr.householderQ();                             // m x m
        matrix_t R = qr.matrixQR().triangularView<Eigen::Upper>();  // m x p
        // D_transpose = Q * R
        matrix_t Q1 = Q.leftCols(e.rows());  // m x p
        matrix_t Q2 = Q.rightCols(e.rows());                     // m x (m-p)
        // Q = [Q1, Q2]
        matrix_t R1 = R.topRows(e.rows());  // p x p
                                            // R = [R1;
                                            //      0]

        // store the matrices Ck, Q1k, Q2k, R1k and vector ek for later remapping \tilde{\delta uk} -> \delta uk
        C_data[i] = C;
        e_data[i] = e;
        Q1_data[i] = Q1;
        Q2_data[i] = Q2;
        R1_data[i] = R1;

        // some intermediate variables used multiple times
        matrix_t P_ke = Q1 * (R1.transpose()).inverse();
        matrix_t P_kx = P_ke * C;

        // see the doc/deduction.pdf for more details
        dynamics_[i].dfdx = dynamics_[i].dfdx - dynamics_[i].dfdu * P_kx;
        dynamics_[i].f = dynamics_[i].f - dynamics_[i].dfdu * P_ke * e;
        dynamics_[i].dfdu = dynamics_[i].dfdu * Q2;

        vector_t q_1 = cost_[i].dfdx - P_kx.transpose() * cost_[i].dfdu;
        vector_t q_2 = -cost_[i].dfdux.transpose() * P_ke * e - P_kx.transpose() * cost_[i].dfduu * P_ke * e;
        cost_[i].dfdx = q_1 + q_2;
        vector_t r_1 = Q2.transpose() * cost_[i].dfdu;
        vector_t r_2 = -Q2.transpose() * cost_[i].dfduu * P_ke * e;
        cost_[i].dfdu = r_1 + r_2;
        cost_[i].dfdxx = cost_[i].dfdxx - P_kx.transpose() * cost_[i].dfdux - cost_[i].dfdux.transpose() * P_kx + P_kx.transpose() * cost_[i].dfduu * P_kx;
        cost_[i].dfdux = Q2.transpose() * cost_[i].dfdux - Q2.transpose() * cost_[i].dfduu * P_kx;
        cost_[i].dfduu = Q2.transpose() * cost_[i].dfduu * Q2;
      }
    }
  }

  if (settings_.robotName == "ballbot") {
    // we should use the finalCostQuadraticApproximation defined by Q_final matrix
    // but in the case of ballbot, it is zero, which leads to unpenalized ending states
    // so I temporarily used costQuadraticApproximation with a random linearization point of input u
    cost_[settings_.N_real] = costFunctionObj.costQuadraticApproximation(
        settings_.trueEventTimes[settings_.N_real], x[settings_.N_real], vector_t::Zero(settings_.n_input));
    cost_[settings_.N_real].dfdxx *= 10.0;// manually add larger penalty s.t. the final state converges to the ref state
    cost_[settings_.N_real].dfdx *= 10.0;// 10 is a customized number, subject to adjustment
  } else {
    cost_[settings_.N_real] =
        costFunctionObj.finalCostQuadraticApproximation(settings_.trueEventTimes[settings_.N_real], x[settings_.N_real]);
  }

  auto endSetupTime = std::chrono::steady_clock::now();
  auto setupIntervalTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endSetupTime - startSetupTime);
  scalar_t setupTime = std::chrono::duration<scalar_t, std::milli>(setupIntervalTime).count();
  std::cout << "\tSetup time usage: " << setupTime << "[ms]." << std::endl;
}

void MultipleShootingSolver::getInfoFromModeSchedule(scalar_t initTime, scalar_t finalTime, ConstraintBase& constraintObj) {
  /*
  A simple example here illustrates the mission of this function

  Assume:
    this->getModeSchedule().eventTimes = {3.25, 3.4, 3.88, 4.02, 4.5}
    initTime = 3.0
    finalTime = 4.0
    user_defined delta_t = 0.1

  Then the following variables will be:
    settings_.trueEventTimes = {3.0, 3.1, 3.2, 3.25, 3.35, 3.4, 3.5, 3.6, 3.7, 3.8, 3.88, 3.98, 4.0}
    settings_.N_real = settings_.trueEventTimes.size() - 1 = 13 - 1 = 12

  */
  scalar_array_t tempEventTimes = this->getModeSchedule().eventTimes;
  scalar_t delta_t = (finalTime - initTime) / settings_.N;

  if (settings_.printModeScheduleDebug) {
    std::cout << "event times original \n";
    for (int i = 0; i < tempEventTimes.size(); i++) {
      std::cout << "event time " << i << " is " << tempEventTimes[i] << std::endl;
    }
  }

  tempEventTimes.insert(tempEventTimes.begin(), initTime);
  for (int i = 0; i < tempEventTimes.size(); i++) {
    if (std::abs(tempEventTimes[i] - finalTime) < std::numeric_limits<double>::epsilon() || tempEventTimes[i] > finalTime) {
      tempEventTimes.erase(tempEventTimes.begin() + i, tempEventTimes.end());
      break;
    }
  }
  tempEventTimes.push_back(finalTime);

  if (settings_.printModeScheduleDebug) {
    std::cout << "event times after \n";
    for (int i = 0; i < tempEventTimes.size(); i++) {
      std::cout << "event time " << i << " is " << tempEventTimes[i] << std::endl;
    }
  }

  size_t n_mode = tempEventTimes.size() - 1;
  size_array_t N_distribution;
  N_distribution.resize(n_mode);

  size_t N_distribution_sum = 0;
  for (int i = 0; i < n_mode; i++) {
    scalar_t division_result = (tempEventTimes[i + 1] - tempEventTimes[i]) / delta_t;
    scalar_t fractpart, intpart;
    fractpart = std::modf(division_result, &intpart);
    size_t N_distribution_i = (size_t)(intpart);
    if (std::abs(fractpart) < 1e-4) {
      N_distribution_i -= 1;
    }
    N_distribution[i] = N_distribution_i;
    if (settings_.printModeScheduleDebug) {
      std::cout << "mode " << i << " has distribution " << N_distribution_i << std::endl;
      std::cout << "division result: " << division_result << " intpart: " << intpart << " fracpart:" << fractpart << std::endl;
    }
    N_distribution_sum += N_distribution_i;
  }

  settings_.trueEventTimes.clear();
  for (int i = 0; i < n_mode; i++) {
    for (int j = 0; j < N_distribution[i] + 1; j++) {
      settings_.trueEventTimes.push_back(tempEventTimes[i] + delta_t * j);
    }
  }
  settings_.trueEventTimes.push_back(finalTime);
  settings_.N_real = settings_.trueEventTimes.size() - 1;

  std::cout << "true event times: {";
  for (int i = 0; i < settings_.trueEventTimes.size(); i++) {
    std::cout << settings_.trueEventTimes[i] << ", ";
  }
  std::cout << "}\n";
}

void MultipleShootingSolver::setupDimension(ConstraintBase& constraintObj) {
  // STATE_DIM is always the same
  ocpSize = HpipmInterface::OcpSize(settings_.N_real, settings_.n_state, settings_.n_input);

  // INPUT_DIM and constraint size may differ
  if (settings_.constrained) {
    for (int k = 0; k < settings_.N_real; k++) {
      scalar_t operTime = settings_.trueEventTimes[k];
      // TODO: wastefull constraint evaluation
      ocs2::VectorFunctionLinearApproximation constraintApprox = constraintObj.stateInputEqualityConstraintLinearApproximation(
          operTime, vector_t::Zero(settings_.n_state), vector_t::Zero(settings_.n_input));
      vector_t e = constraintApprox.f;
      if (settings_.qr_decomp) {
        ocpSize.nu[k] = settings_.n_input - e.rows();
      } else {
        ocpSize.ng[k] = e.rows();
      }
    }
  }

  if (settings_.constrained && settings_.qr_decomp) {
    C_data.resize(settings_.N_real);
    e_data.resize(settings_.N_real);
    Q1_data.resize(settings_.N_real);
    Q2_data.resize(settings_.N_real);
    R1_data.resize(settings_.N_real);
  }
}


}  // namespace ocs2