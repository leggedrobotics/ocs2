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
#include <ocs2_core/loopshaping/LoopshapingMisc.h>

namespace ocs2 {

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

          boost::property_tree::ptree pt;
          boost::property_tree::read_info(settingsFile, pt);

          q_filter_ = LoopshapingPropertyTree::readMIMOFilter(pt, "q_filter");
          r_filter_ = LoopshapingPropertyTree::readMIMOFilter(pt, "r_filter");
          s_filter_ = LoopshapingPropertyTree::readMIMOFilter(pt, "s_inv_filter", true);
          gamma = pt.get<double>("gamma");
          eliminateInputs = pt.get<bool>("eliminateInputs");
          return success;
        }

        size_t getNumStates() const {return q_filter_.getNumStates()
                                      + r_filter_.getNumStates()
                                      + s_filter_.getNumStates();};
        size_t getNumInputs() const {return s_filter_.getNumInputs();};

        const Filter& getStateFilter_q() const {return q_filter_;};
        const Filter& getInputFilter_r() const {return r_filter_;};
        const Filter& getInputFilter_s() const {return s_filter_;};

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

     private:
        Filter q_filter_, r_filter_, s_filter_;
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
