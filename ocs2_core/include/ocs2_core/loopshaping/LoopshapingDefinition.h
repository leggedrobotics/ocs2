//
// Created by ruben on 26.10.18.
//

#ifndef BALLBOT_CTRL_LOOPSHAPINGDEFINITION_H
#define BALLBOT_CTRL_LOOPSHAPINGDEFINITION_H

#include <vector>
#include <Eigen/Dense>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <ocs2_core/dynamics/TransferFunctionBase.h>
#include <ocs2_core/logic/rules/NullLogicRules.h>

namespace ocs2 {

    class SISOFilterDefinition {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SISOFilterDefinition(const boost::property_tree::ptree& pt, std::string filterMIMOName, std::string filterSISOName,
        bool invert = false) {
          std::string filterName = filterMIMOName + "." + filterSISOName;

          // Get Sizes
          size_t numRepeats = pt.get<size_t>(filterName + ".numRepeats");
          size_t numPoles = pt.get<size_t>(filterName + ".numPoles");
          size_t numZeros = pt.get<size_t>(filterName + ".numZeros");
          double DCGain = pt.get<double>(filterName + ".DCGain");
          numStates_ = numRepeats*numPoles;
          numInputs_ = numRepeats;
          numOutputs_ = numRepeats;

          // Setup Filter, convention a0*s^n + a1*s^(n-1) + ... + an
          Eigen::VectorXd numerator(numZeros+1);
          numerator.setZero();
          numerator(0) = 1.0;
          for (size_t z = 0; z<numZeros; z++){
            double zero = pt.get<double>(filterName + ".zeros." + "(" +std::to_string(z) + ")");
            numerator.segment(1, z+1) -= zero*numerator.segment(0, z+1).eval();
          }

          Eigen::VectorXd denominator(numPoles+1);
          denominator.setZero();
          denominator(0) = 1.0;
          for (size_t p = 0; p<numPoles; p++){
            double pole = pt.get<double>(filterName + ".poles." + "(" +std::to_string(p) + ")");
            denominator.segment(1, p+1) -= pole*denominator.segment(0, p+1).eval();
          }

          // Scale
          if (DCGain > 0) {
            double currentDCGain = numerator(numZeros) / denominator(numPoles);
            if (currentDCGain < 1e-6 || currentDCGain > 1e6) {
              throw std::runtime_error("Trouble rescaling transfer function, current DCGain: " + std::to_string(currentDCGain));
            }
            double scaling = DCGain / currentDCGain;
            numerator *= scaling;
          }

          if (invert) {
            Eigen::VectorXd temp;
            temp = numerator;
            numerator = denominator;
            denominator = temp;
          }

          // Convert to state space
          Eigen::MatrixXd a, b, c, d;
          ocs2::tf2ss(numerator, denominator, a, b, c, d);

          A_ = Eigen::MatrixXd::Zero(numStates_, numStates_);
          B_ = Eigen::MatrixXd::Zero(numStates_, numInputs_);
          C_ = Eigen::MatrixXd::Zero(numInputs_, numStates_);
          D_ = Eigen::MatrixXd::Zero(numInputs_, numInputs_);
          size_t statecount = 0;
          for (size_t r = 0; r<numRepeats; r++){
            A_.block(statecount, statecount, numPoles, numPoles) = a;
            B_.block(statecount, r, numPoles, 1) = b;
            C_.block(r, statecount, 1, numPoles) = c;
            D_.block(r, r, 1, 1) = d;
            statecount += numPoles;
          }
        }

        size_t getNumStates() const {return numStates_;};
        size_t getNumInputs() const {return numInputs_;};
        size_t getNumOutputs() const {return numOutputs_;};

        const Eigen::MatrixXd& getA() const {return A_;};
        const Eigen::MatrixXd& getB() const {return B_;};
        const Eigen::MatrixXd& getC() const {return C_;};
        const Eigen::MatrixXd& getD() const {return D_;};

        void print() const {
          std::cout << "numStates: " << numStates_ << std::endl;
          std::cout << "numInputs: " << numInputs_ << std::endl;
          std::cout << "numOutputs: " << numOutputs_ << std::endl;
          std::cout << "A: \n" << A_  << std::endl;
          std::cout << "B: \n" << B_  << std::endl;
          std::cout << "C: \n" << C_  << std::endl;
          std::cout << "D: \n" << D_  << std::endl;
        }

    private:
        Eigen::MatrixXd A_, B_, C_, D_;
        size_t numStates_ = 0;
        size_t numInputs_ = 0;
        size_t numOutputs_ = 0;

    };

    class MIMOFilterDefinition {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        bool loadSettings(std::string settingsFile, std::string filterName, bool invert = false) {
          boost::property_tree::ptree pt;
          boost::property_tree::read_info(settingsFile, pt);
          size_t numFilters = pt.get<size_t>(filterName + ".numFilters");

          if (numFilters == 0){
            numStates_ = 0;
            numInputs_ = 0;
            numOutputs_ = 0;
            A_.resize(0,0);
            B_.resize(0,0);
            C_.resize(0,0);
            D_.resize(0,0);
          } else {

            // Read Individual filters
            std::vector<SISOFilterDefinition> sisoFilters;
            sisoFilters.reserve(numFilters);
            numStates_ = 0;
            numInputs_ = 0;
            numOutputs_ = 0;
            for (size_t i = 0; i < numFilters; i++) {
              sisoFilters.push_back(SISOFilterDefinition(pt, filterName, "Filter" + std::to_string(i), invert));
              numStates_ += sisoFilters.back().getNumStates();
              numInputs_ += sisoFilters.back().getNumInputs();
              numOutputs_ += sisoFilters.back().getNumOutputs();
            }

            // Fill matrices
            A_ = Eigen::MatrixXd::Zero(numStates_, numStates_);
            B_ = Eigen::MatrixXd::Zero(numStates_, numInputs_);
            C_ = Eigen::MatrixXd::Zero(numOutputs_, numStates_);
            D_ = Eigen::MatrixXd::Zero(numOutputs_, numInputs_);
            size_t statecount = 0;
            size_t inputcount = 0;
            size_t outputcount = 0;
            for (const auto &filt : sisoFilters) {
              A_.block(statecount, statecount, filt.getNumStates(), filt.getNumStates()) = filt.getA();
              B_.block(statecount, inputcount, filt.getNumStates(), filt.getNumInputs()) = filt.getB();
              C_.block(outputcount, statecount, filt.getNumOutputs(), filt.getNumStates()) = filt.getC();
              D_.block(outputcount, inputcount, filt.getNumOutputs(), filt.getNumInputs()) = filt.getD();
              statecount += filt.getNumStates();
              inputcount += filt.getNumInputs();
              outputcount += filt.getNumOutputs();
            }
          }
        }

        size_t getNumStates() const {return numStates_;};
        size_t getNumInputs() const {return numInputs_;};
        size_t getNumOutputs() const {return numOutputs_;};

        const Eigen::MatrixXd& getA() const {return A_;};
        const Eigen::MatrixXd& getB() const {return B_;};
        const Eigen::MatrixXd& getC() const {return C_;};
        const Eigen::MatrixXd& getD() const {return D_;};

        void print() const {
          std::cout << "numStates: " << numStates_ << std::endl;
          std::cout << "numInputs: " << numInputs_ << std::endl;
          std::cout << "numOutputs: " << numOutputs_ << std::endl;
          std::cout << "A: \n" << A_  << std::endl;
          std::cout << "B: \n" << B_  << std::endl;
          std::cout << "C: \n" << C_  << std::endl;
          std::cout << "D: \n" << D_  << std::endl;
        }

    private:
        Eigen::MatrixXd A_, B_, C_, D_;
        size_t numStates_ = 0;
        size_t numInputs_ = 0;
        size_t numOutputs_ = 0;
    };



    /*
     *  Class to assemble and store the loopshaping definition
     *  The definition contains three filters
     *  q: filters on state
     *  r: filters on inputs
     *  s: filters on inputs (implemented as inverse)
     */
    class LoopshapingDefinition {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        bool loadSettings(std::string settingsFile) {
          bool success = true;

          success &= q_filter_.loadSettings(settingsFile, "q_filter");
          success &= r_filter_.loadSettings(settingsFile, "r_filter");
          success &= s_filter_.loadSettings(settingsFile, "s_inv_filter", true);

          boost::property_tree::ptree pt;
          boost::property_tree::read_info(settingsFile, pt);
          gamma = pt.get<double>("gamma");
          eliminateInputs = pt.get<bool>("eliminateInputs");
          return success;
        }

        size_t getNumStates() const {return q_filter_.getNumStates()
                                      + r_filter_.getNumStates()
                                      + s_filter_.getNumStates();};
        size_t getNumInputs() const {return s_filter_.getNumInputs();};

        const MIMOFilterDefinition& getStateFilter_q() const {return q_filter_;};
        const MIMOFilterDefinition& getInputFilter_r() const {return r_filter_;};
        const MIMOFilterDefinition& getInputFilter_s() const {return s_filter_;};

        void print() const {
          std::cout << "q_filter: " << std::endl;
          q_filter_.print();
          std::cout << "r_filter: " << std::endl;
          r_filter_.print();
          std::cout << "s_filter: " << std::endl;
          s_filter_.print();
        };

        double gamma = 0.9;
        bool eliminateInputs = false;

        template <typename DerivedStateVector, typename DerivedSystemState>
        void getSystemState(const DerivedStateVector& state, DerivedSystemState& systemState){
          systemState = state.head(systemState.size());
        };

        template <typename DerivedStateVector, typename DerivedInputVector, typename DerivedSystemInput>
        void getSystemInput(const DerivedStateVector& state, const DerivedInputVector& input, DerivedSystemInput& systemInput){
          if (r_filter_.getNumOutputs() > 0){
            systemInput = input.head(systemInput.size());
          }

          if (s_filter_.getNumOutputs() > 0){
            if (eliminateInputs){
              auto& filterState_s = state.tail(s_filter_.getNumStates());
              auto& filterInput_s = input;
              systemInput = getInputFilter_s().getC() * filterState_s + getInputFilter_s().getD() * filterInput_s;
            } else {
              systemInput = input.head(systemInput.size());
            }
          }
        };

        template <typename DerivedStateVector, typename DerivedFilterState>
        void getFilterState(const DerivedStateVector& state, DerivedFilterState& filterState){
          if (r_filter_.getNumOutputs() > 0){
            filterState = state.tail(r_filter_.getNumStates());
          }

          if (s_filter_.getNumOutputs() > 0){
            filterState = state.tail(s_filter_.getNumStates());
          }
        };

        template <typename DerivedStateVector, typename DerivedInputVector, typename DerivedFilterInput>
        void getFilteredInput(const DerivedStateVector& state, const DerivedInputVector& input, DerivedFilterInput& filterInput){
          if (r_filter_.getNumOutputs() > 0){
            filterInput = r_filter_.getC() * state.tail(r_filter_.getNumStates())
                          + r_filter_.getD() * input.head(r_filter_.getNumInputs());
          }

          if (s_filter_.getNumOutputs() > 0){
            filterInput = input.tail(s_filter_.getNumInputs());
          }
        };

        template <typename DerivedSystemVector, typename DerivedFilterState, typename DerivedStateVector>
        void concatenateSystemAndFilterState(const DerivedSystemVector& systemState, const DerivedFilterState& filterState,
             DerivedStateVector& state){
          state.head(systemState.size()) = systemState;

          if (r_filter_.getNumOutputs() > 0){
            state.tail(r_filter_.getNumStates()) = filterState;
          }

          if (s_filter_.getNumOutputs() > 0){
            state.tail(s_filter_.getNumStates()) = filterState;
          }
        };

        template <typename DerivedSystemInput, typename DerivedFilterInput, typename DerivedInput>
        void concatenateSystemAndFilterInput(const DerivedSystemInput& systemInput, const DerivedFilterInput& filterInput,
                                             DerivedInput& input){
          if (r_filter_.getNumInputs() > 0){
            input.head(systemInput.size()) = systemInput;
          }

          if (s_filter_.getNumInputs() > 0){
            if (eliminateInputs){
              input.head(s_filter_.getNumInputs()) = filterInput;
            } else {
              input.head(systemInput.size()) = systemInput;
              input.segment(systemInput.size(), s_filter_.getNumInputs()) = filterInput;
            }
          }
        };

      template<typename DerivedDerivativeSystemState, typename DerivedDerivativeState>
      void functionDerivativeState(const DerivedDerivativeSystemState& system_dfdx,
                                   DerivedDerivativeState& dfdx) {
        if (r_filter_.getNumOutputs() > 0) {
          dfdx.head(system_dfdx.size()) = system_dfdx;
          dfdx.tail(r_filter_.getNumStates()).setZero();
        }
        else if (s_filter_.getNumOutputs() > 0 && eliminateInputs) {
          throw std::runtime_error("[LoopshapingDefinition::functionDerivativeState] Use different functionDerivativeState");
        }
        else if (s_filter_.getNumOutputs() > 0 && !eliminateInputs) {
          dfdx.head(system_dfdx.size()) = system_dfdx;
          dfdx.tail(s_filter_.getNumStates()).setZero();
        }
      };

      template<typename DerivedDerivativeSystemState, typename DerivedDerivativeSystemInput, typename DerivedDerivativeState>
      void functionDerivativeState(const DerivedDerivativeSystemState& system_dfdx,
                                   const DerivedDerivativeSystemInput& system_dfdu,
                                   DerivedDerivativeState& dfdx) {
        // R filter case
        if (r_filter_.getNumOutputs() > 0) {
          functionDerivativeState(system_dfdx, dfdx);
        }
          // S filter + eliminate inputs
        else if (s_filter_.getNumOutputs() > 0 && eliminateInputs) {
          dfdx.head(system_dfdx.size()) = system_dfdx;
          dfdx.tail(s_filter_.getNumStates()) = s_filter_.getC().transpose() * system_dfdu;
        }
          // S filter without eliminate inputs
        else if (s_filter_.getNumOutputs() > 0 && !eliminateInputs) {
          functionDerivativeState(system_dfdx, dfdx);
        }
      };

      template<typename DerivedDerivativeSystemInput, typename DerivedDerivativeInput>
      void functionDerivativeInput(const DerivedDerivativeSystemInput& system_dfdu,
                                   DerivedDerivativeInput& dfdu) {
        if (r_filter_.getNumOutputs() > 0) {
          dfdu.head(system_dfdu.size()) = system_dfdu;
        }
        else if (s_filter_.getNumOutputs() > 0 && eliminateInputs) {
          dfdu.head(s_filter_.getNumInputs()) = s_filter_.getD().transpose() * system_dfdu;
        }
        else if (s_filter_.getNumOutputs() > 0 && !eliminateInputs) {
          dfdu.head(system_dfdu.size()) = system_dfdu;
          dfdu.tail(s_filter_.getNumInputs()).setZero();
        }
      };

      template<typename DerivedSecondDerivativeSystemState, typename DerivedSecondDerivativeState>
      void functionSecondDerivativeState(const DerivedSecondDerivativeSystemState& system_ddfdxdx,
                                         DerivedSecondDerivativeState& ddfdxdx) {
        if (r_filter_.getNumOutputs() > 0) {
          ddfdxdx.block(0, 0, system_ddfdxdx.rows(), system_ddfdxdx.cols()) = system_ddfdxdx;
          ddfdxdx.block(0, system_ddfdxdx.cols(), system_ddfdxdx.rows(), r_filter_.getNumStates()).setZero();
          ddfdxdx.block(system_ddfdxdx.rows(), 0, r_filter_.getNumStates(), system_ddfdxdx.cols()).setZero();
          ddfdxdx.block(system_ddfdxdx.rows(), system_ddfdxdx.cols(), r_filter_.getNumStates(), r_filter_.getNumStates()).setZero();
        }
        else if (s_filter_.getNumOutputs() > 0 && eliminateInputs) {
          throw std::runtime_error("[LoopshapingDefinition::functionSecondDerivativeState] Use different functionSecondDerivativeState");
        }
        else if (s_filter_.getNumOutputs() > 0 && !eliminateInputs) {
          ddfdxdx.block(0, 0, system_ddfdxdx.rows(), system_ddfdxdx.cols()) = system_ddfdxdx;
          ddfdxdx.block(0, system_ddfdxdx.cols(), system_ddfdxdx.rows(), s_filter_.getNumStates()).setZero();
          ddfdxdx.block(system_ddfdxdx.rows(), 0, s_filter_.getNumStates(), system_ddfdxdx.cols()).setZero();
          ddfdxdx.block(system_ddfdxdx.rows(), system_ddfdxdx.cols(), s_filter_.getNumStates(), s_filter_.getNumStates()).setZero();
        }
      };

      template<typename DerivedSecondDerivativeSystemState, typename DerivedDerivativeSystemInputState, typename DerivedSecondDerivativeSystemInput, typename DerivedSecondDerivativeState>
      void functionSecondDerivativeState(const DerivedSecondDerivativeSystemState& system_ddfdxdx,
                                         const DerivedDerivativeSystemInputState& system_ddfdudx,
                                         const DerivedSecondDerivativeSystemInput& system_ddfdudu,
                                         DerivedSecondDerivativeState& ddfdxdx) {
        if (r_filter_.getNumOutputs() > 0) {
          functionSecondDerivativeState(system_ddfdxdx, ddfdxdx);
        }
        else if (s_filter_.getNumOutputs() > 0 && eliminateInputs) {
          ddfdxdx.block(0, 0, system_ddfdxdx.rows(), system_ddfdxdx.cols()) = system_ddfdxdx;
          ddfdxdx.block(0, system_ddfdxdx.cols(), system_ddfdxdx.rows(), s_filter_.getNumStates()) = system_ddfdudx.transpose() * s_filter_.getC();
          ddfdxdx.block(system_ddfdxdx.rows(), 0, s_filter_.getNumStates(), system_ddfdxdx.cols()) = ddfdxdx.block(0, system_ddfdxdx.cols(), system_ddfdxdx.rows(), s_filter_.getNumStates()).transpose();
          ddfdxdx.block(system_ddfdxdx.rows(), system_ddfdxdx.cols(), s_filter_.getNumStates(), s_filter_.getNumStates()) = s_filter_.getC().transpose() * system_ddfdudu * s_filter_.getC();
        }
        else if (s_filter_.getNumOutputs() > 0 && !eliminateInputs) {
          functionSecondDerivativeState(system_ddfdxdx, ddfdxdx);
        }
      };

      template<typename DerivedSecondDerivativeSystemInput, typename DerivedSecondDerivativeInput>
      void functionSecondDerivativeInput(const DerivedSecondDerivativeSystemInput& system_ddfdudu,
                                         DerivedSecondDerivativeInput& ddfdudu) {
        if (r_filter_.getNumOutputs() > 0) {
          ddfdudu.block(0, 0, system_ddfdudu.rows(), system_ddfdudu.cols()) = system_ddfdudu;
        }
        else if (s_filter_.getNumOutputs() > 0 && eliminateInputs) {
          ddfdudu.block(0, 0, s_filter_.getNumInputs(), s_filter_.getNumInputs()) = s_filter_.getD().transpose() * system_ddfdudu * s_filter_.getD();
        }
        else if (s_filter_.getNumOutputs() > 0 && !eliminateInputs) {
          ddfdudu.block(0, 0, system_ddfdudu.rows(), system_ddfdudu.cols()) = system_ddfdudu;
          ddfdudu.block(0, system_ddfdudu.cols(), system_ddfdudu.rows(), s_filter_.getNumInputs()).setZero();
          ddfdudu.block(system_ddfdudu.rows(), 0, s_filter_.getNumInputs(), system_ddfdudu.cols()).setZero();
          ddfdudu.block(system_ddfdudu.rows(), system_ddfdudu.cols(), s_filter_.getNumInputs(), s_filter_.getNumInputs()).setZero();
        }
      };

      template<typename DerivedDerivativeSystemInputState, typename DerivedDerivativeInputState>
      void functionDerivativeInputState(const DerivedDerivativeSystemInputState& system_ddfdudx,
                                         DerivedDerivativeInputState& ddfdudx) {
        if (r_filter_.getNumOutputs() > 0) {
          ddfdudx.block(0, 0, system_ddfdudx.rows(), system_ddfdudx.cols()) = system_ddfdudx;
          ddfdudx.block(0, system_ddfdudx.cols(), system_ddfdudx.rows(), r_filter_.getNumStates()).setZero();
        }
        else if (s_filter_.getNumOutputs() > 0 && eliminateInputs) {
          throw std::runtime_error("[LoopshapingDefinition::functionDerivativeInputState] Use different functionDerivativeInputState");
        }
        else if (s_filter_.getNumOutputs() > 0 && !eliminateInputs) {
          ddfdudx.block(0, 0, system_ddfdudx.rows(), system_ddfdudx.cols()) = system_ddfdudx;
          ddfdudx.block(0, system_ddfdudx.cols(), system_ddfdudx.rows(), s_filter_.getNumStates()).setZero();
          ddfdudx.block(system_ddfdudx.rows(), 0, s_filter_.getNumInputs(), system_ddfdudx.cols()).setZero();
          ddfdudx.block(system_ddfdudx.rows(), system_ddfdudx.cols(), s_filter_.getNumInputs(), s_filter_.getNumStates()).setZero();
        }
      };

      template<typename DerivedDerivativeSystemInputState, typename DerivedSecondDerivativeSystemInput, typename DerivedDerivativeInputState>
      void functionDerivativeInputState(const DerivedDerivativeSystemInputState& system_ddfdudx,
                                        const DerivedSecondDerivativeSystemInput& system_ddfdudu,
                                        DerivedDerivativeInputState& ddfdudx) {
        if (r_filter_.getNumOutputs() > 0) {
          functionDerivativeInputState(system_ddfdudx, ddfdudx);
        }
        else if (s_filter_.getNumOutputs() > 0 && eliminateInputs) {
          ddfdudx.block(0, 0, s_filter_.getNumInputs(), system_ddfdudx.cols()) = s_filter_.getD().transpose() * system_ddfdudx;
          ddfdudx.block(0, system_ddfdudx.cols(), s_filter_.getNumInputs(), s_filter_.getNumStates()) = s_filter_.getD().transpose() * system_ddfdudx * s_filter_.getC();
        }
        else if (s_filter_.getNumOutputs() > 0 && !eliminateInputs) {
          functionDerivativeInputState(system_ddfdudx, ddfdudx);
        }
      };

     private:
        MIMOFilterDefinition q_filter_, r_filter_, s_filter_;
    };

    template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
        size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
        size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
        class LOGIC_RULES_T=NullLogicRules>
    class LoopshapingFilterDynamics {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using Ptr = std::shared_ptr<LoopshapingFilterDynamics>;

        using state_vector_t = Eigen::Matrix<double, FULL_STATE_DIM, 1>;
        using input_vector_t = Eigen::Matrix<double, FULL_INPUT_DIM, 1>;
        using system_state_vector_t = Eigen::Matrix<double, SYSTEM_STATE_DIM, 1>;
        using system_input_vector_t = Eigen::Matrix<double, SYSTEM_INPUT_DIM, 1>;
        using filter_state_vector_t = Eigen::Matrix<double, FILTER_STATE_DIM, 1>;
        using filter_input_vector_t = Eigen::Matrix<double, FILTER_INPUT_DIM, 1>;

        LoopshapingFilterDynamics(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) :
        loopshapingDefinition_(loopshapingDefinition)
        {}

        void initializeEquilibriumState(const system_state_vector_t& system_state, const system_input_vector_t& system_input,
                                   state_vector_t& state)
        {
          filter_state_vector_t filter_state;
          initializeEquilibriumFilterState(system_state, system_input, filter_state);
          state.segment(0, SYSTEM_STATE_DIM) = system_state;
          state.segment(SYSTEM_STATE_DIM, FILTER_STATE_DIM) = filter_state;
        }

        void initializeEquilibriumFilterState(const system_state_vector_t& system_state, const system_input_vector_t& system_input,
                                   filter_state_vector_t& filter_state)
        {
          filter_input_vector_t filter_input;
          filter_input.setZero();
          initializeEquilibriumFilterStateInput(system_state, system_input, filter_state, filter_input);
        }

        void initializeEquilibriumFilterStateInput(const system_state_vector_t& system_state, const system_input_vector_t& system_input,
                                   filter_state_vector_t& filter_state, filter_input_vector_t& filter_input) {
          const auto &q_filter = loopshapingDefinition_->getStateFilter_q();
          const auto &r_filter = loopshapingDefinition_->getInputFilter_r();
          const auto &s_filter = loopshapingDefinition_->getInputFilter_s();

          filter_state_vector_t equilibriumFilterState;
          filter_input_vector_t equilibriumFilterInput;

          if (r_filter.getNumOutputs() > 0) {
            // Solve
            // 0 = A_r*x_r + B_r*u
            filter_state = - r_filter.getA().colPivHouseholderQr().solve(r_filter.getB() * system_input);
          }

          if (s_filter.getNumOutputs() > 0) {
              // Solve
              // [0  =  [  A_s    B_s    [x_s
              //  u]       C_s    D_s  ]  v_s]
              Eigen::MatrixXd ABCD(s_filter.getNumStates() + s_filter.getNumOutputs(),
                                   s_filter.getNumStates() + s_filter.getNumInputs());
              ABCD.block(0, 0, s_filter.getNumStates(), s_filter.getNumStates()) = s_filter.getA();
              ABCD.block(0, s_filter.getNumStates(), s_filter.getNumStates(),
                         s_filter.getNumInputs()) = s_filter.getB();
              ABCD.block(s_filter.getNumStates(), 0, s_filter.getNumOutputs(),
                         s_filter.getNumStates()) = s_filter.getC();
              ABCD.block(s_filter.getNumStates(), s_filter.getNumStates(), s_filter.getNumOutputs(),
                         s_filter.getNumInputs()) =
                  s_filter.getD();

              Eigen::VectorXd x_s_v_s;
              Eigen::VectorXd zero_u(s_filter.getNumStates() + s_filter.getNumOutputs());
              zero_u.segment(0, s_filter.getNumStates()).setZero();
              zero_u.segment(s_filter.getNumStates(), s_filter.getNumOutputs()) = system_input;

              x_s_v_s = ABCD.colPivHouseholderQr().solve(zero_u);

              filter_state = x_s_v_s.segment(0, s_filter.getNumStates());
              filter_input = x_s_v_s.segment(s_filter.getNumStates(), s_filter.getNumInputs());
            }

          filter_state_ = filter_state;
        }

        void advance(double dt, const system_state_vector_t& system_state, const system_input_vector_t& system_input)
        {
          advance(dt, system_state, system_input, filter_input_vector_t::Zero());
        }

        void advance(double dt, const system_state_vector_t& system_state, const system_input_vector_t& system_input,
        const filter_input_vector_t& filter_input) {
          const auto& q_filter = loopshapingDefinition_->getStateFilter_q();
          const auto& r_filter = loopshapingDefinition_->getInputFilter_r();
          const auto& s_filter = loopshapingDefinition_->getInputFilter_s();

          filter_state_vector_t filterstateDerivative;
          if (q_filter.getNumOutputs() > 0){
            filterstateDerivative.segment(0, q_filter.getNumStates()) =
                q_filter.getA() * filter_state_.segment(0, q_filter.getNumStates())
                + q_filter.getB() * system_state;
          }

          if (r_filter.getNumOutputs() > 0){
            filterstateDerivative.segment(q_filter.getNumStates(), r_filter.getNumStates()) =
                r_filter.getA() * filter_state_.segment(q_filter.getNumStates(), r_filter.getNumStates())
                + r_filter.getB() * system_input;
          }

          if (s_filter.getNumOutputs() > 0){
            filterstateDerivative.segment(q_filter.getNumStates() + r_filter.getNumStates(), s_filter.getNumStates()) =
                s_filter.getA() * filter_state_.segment(q_filter.getNumStates() + r_filter.getNumStates(), s_filter.getNumStates())
                + s_filter.getB() * filter_input;
          }

          filter_state_ += filterstateDerivative * dt;
        }

        const filter_state_vector_t& getFilterState() { return filter_state_; };

    private:
        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
        filter_state_vector_t filter_state_;
    };

} // namespace ocs2

#endif //BALLBOT_CTRL_LOOPSHAPINGDEFINITION_H
