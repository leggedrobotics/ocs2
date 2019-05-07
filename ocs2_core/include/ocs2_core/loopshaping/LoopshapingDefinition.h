//
// Created by ruben on 26.10.18.
//

#ifndef OCS2_LOOPSHAPINGDEFINITION_H
#define OCS2_LOOPSHAPINGDEFINITION_H

#include <vector>
#include <Eigen/Dense>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <ocs2_core/dynamics/TransferFunctionBase.h>
#include <ocs2_core/logic/rules/NullLogicRules.h>
#include <ocs2_core/loopshaping/LoopshapingMisc.h>

namespace ocs2 {

    enum class LoopshapingType { outputpattern, inputpattern, eliminatepattern };

    /*
     *  Class to assemble and store the loopshaping definition
     *  The definition contains two filters
     *  r: filters on inputs
     *  s: filters on inputs (implemented as inverse)
     */
    class LoopshapingDefinition {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double gamma = 0.9;

        bool loadSettings(std::string settingsFile) {
            bool success = true;

            boost::property_tree::ptree pt;
            boost::property_tree::read_info(settingsFile, pt);

            Filter r_filter_ = LoopshapingPropertyTree::readMIMOFilter(pt, "r_filter");
            Filter s_filter_ = LoopshapingPropertyTree::readMIMOFilter(pt, "s_inv_filter", true);

            gamma = pt.get<double>("gamma");
            bool eliminateInputs = pt.get<bool>("eliminateInputs");

            if (r_filter_.getNumOutputs() > 0 && s_filter_.getNumOutputs() > 0) {
                throw std::runtime_error("[LoopshapingDefinition] using both r and s filter not implemented");
            }

            if (r_filter_.getNumOutputs() > 0) {
                loopshapingType_ = LoopshapingType::outputpattern;
                filter_ = r_filter_;
            }
            if (s_filter_.getNumOutputs() > 0) {
                if (eliminateInputs){
                    loopshapingType_ = LoopshapingType::eliminatepattern;
                } else {
                    loopshapingType_ = LoopshapingType::inputpattern;
                }
                filter_ = s_filter_;
            }

            return success;
        }

        LoopshapingType getType() const {return loopshapingType_;};
        const Filter& getInputFilter() const {return filter_;};

        void print() const { filter_.print(); };

        template <typename DerivedStateVector, typename DerivedSystemState>
        void getSystemState(const DerivedStateVector& state, DerivedSystemState& systemState){
            systemState = state.head(systemState.size());
        };

        template <typename DerivedStateVector, typename DerivedInputVector, typename DerivedSystemInput>
        void getSystemInput(const DerivedStateVector& state, const DerivedInputVector& input, DerivedSystemInput& systemInput){
            switch (loopshapingType_){
                case LoopshapingType::outputpattern :
                case LoopshapingType::inputpattern :
                    systemInput = input.head(systemInput.size());
                    break;
                case LoopshapingType::eliminatepattern :
                    systemInput = filter_.getC() * state.tail(filter_.getNumStates()) + filter_.getD() * input;
                    break;
            }
        };

        template <typename DerivedStateVector, typename DerivedFilterState>
        void getFilterState(const DerivedStateVector& state, DerivedFilterState& filterState) {
            filterState = state.tail(filter_.getNumStates());
        };

        template <typename DerivedStateVector, typename DerivedInputVector, typename DerivedFilterInput>
        void getFilteredInput(const DerivedStateVector& state, const DerivedInputVector& input, DerivedFilterInput& filterInput) {
            switch (loopshapingType_) {
                case LoopshapingType::outputpattern :
                    filterInput = filter_.getC() * state.tail(filter_.getNumStates())
                                  + filter_.getD() * input.head(filter_.getNumInputs());
                    break;
                case LoopshapingType::inputpattern :
                case LoopshapingType::eliminatepattern :
                    filterInput = input.tail(filter_.getNumInputs());
                    break;
            }
        };

        template <typename DerivedSystemVector, typename DerivedFilterState, typename DerivedStateVector>
        void concatenateSystemAndFilterState(const DerivedSystemVector& systemState, const DerivedFilterState& filterState,
                                             DerivedStateVector& state){
            state.head(systemState.size()) = systemState;
            state.tail(filter_.getNumStates()) = filterState;
        };

        template <typename DerivedSystemInput, typename DerivedFilterInput, typename DerivedInput>
        void concatenateSystemAndFilterInput(const DerivedSystemInput& systemInput, const DerivedFilterInput& filterInput,
                                             DerivedInput& input){
            switch (loopshapingType_) {
                case LoopshapingType::outputpattern :
                    input.head(systemInput.size()) = systemInput;
                    break;
                case LoopshapingType::inputpattern :
                    input.head(systemInput.size()) = systemInput;
                    input.segment(systemInput.size(), filter_.getNumInputs()) = filterInput;
                    break;
                case LoopshapingType::eliminatepattern :
                    input.head(filter_.getNumInputs()) = filterInput;
                    break;
            }
        };

    private:
        Filter filter_;
        LoopshapingType loopshapingType_;
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
            const auto &filter = loopshapingDefinition_->getInputFilter();

            filter_state_vector_t equilibriumFilterState;
            filter_input_vector_t equilibriumFilterInput;

            if (loopshapingDefinition_->getType() == LoopshapingType::outputpattern) {
                // Solve
                // 0 = A_r*x_r + B_r*u
                filter_state = - filter.getA().colPivHouseholderQr().solve(filter.getB() * system_input);
            }

            if (loopshapingDefinition_->getType() == LoopshapingType::outputpattern ||
                loopshapingDefinition_->getType() == LoopshapingType::eliminatepattern) {
                // Solve
                // [0  =  [  A_s    B_s    [x_s
                //  u]       C_s    D_s  ]  v_s]
                Eigen::MatrixXd ABCD(filter.getNumStates() + filter.getNumOutputs(),
                                     filter.getNumStates() + filter.getNumInputs());
                ABCD.block(0, 0, filter.getNumStates(), filter.getNumStates()) = filter.getA();
                ABCD.block(0, filter.getNumStates(), filter.getNumStates(),
                           filter.getNumInputs()) = filter.getB();
                ABCD.block(filter.getNumStates(), 0, filter.getNumOutputs(),
                           filter.getNumStates()) = filter.getC();
                ABCD.block(filter.getNumStates(), filter.getNumStates(), filter.getNumOutputs(),
                           filter.getNumInputs()) =
                        filter.getD();

                Eigen::VectorXd x_s_v_s;
                Eigen::VectorXd zero_u(filter.getNumStates() + filter.getNumOutputs());
                zero_u.segment(0, filter.getNumStates()).setZero();
                zero_u.segment(filter.getNumStates(), filter.getNumOutputs()) = system_input;

                x_s_v_s = ABCD.colPivHouseholderQr().solve(zero_u);

                filter_state = x_s_v_s.segment(0, filter.getNumStates());
                filter_input = x_s_v_s.segment(filter.getNumStates(), filter.getNumInputs());
            }

            filter_state_ = filter_state;
        }

        void advance(double dt, const system_state_vector_t& system_state, const system_input_vector_t& system_input)
        {
            advance(dt, system_state, system_input, filter_input_vector_t::Zero());
        }

        void advance(double dt, const system_state_vector_t& system_state, const system_input_vector_t& system_input,
                     const filter_input_vector_t& filter_input) {
            const auto& filter = loopshapingDefinition_->getInputFilter();

            filter_state_vector_t filterstateDerivative;

            if (loopshapingDefinition_->getType() == LoopshapingType::outputpattern) {
                filterstateDerivative.segment(0, filter.getNumStates()) =
                        filter.getA() * filter_state_
                        + filter.getB() * system_input;
            }

            if (loopshapingDefinition_->getType() == LoopshapingType::outputpattern && loopshapingDefinition_->getType() == LoopshapingType::eliminatepattern) {
                filterstateDerivative.segment(0, filter.getNumStates()) =
                        filter.getA() * filter_state_
                        + filter.getB() * filter_input;
            }

            filter_state_ += filterstateDerivative * dt;
        }

        const filter_state_vector_t& getFilterState() { return filter_state_; };

    private:
        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
        filter_state_vector_t filter_state_;
    };

} // namespace ocs2

#endif //OCS2_LOOPSHAPINGDEFINITION_H
