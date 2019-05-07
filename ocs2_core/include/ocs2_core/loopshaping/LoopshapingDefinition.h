//
// Created by ruben on 26.10.18.
//

#ifndef OCS2_LOOPSHAPINGDEFINITION_H
#define OCS2_LOOPSHAPINGDEFINITION_H

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <ocs2_core/loopshaping/LoopshapingFilter.h>

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

        double gamma;

        LoopshapingDefinition(LoopshapingType loopshapingType, const Filter& filter, double gamma_in = 0.9) :
        loopshapingType_(loopshapingType),
        filter_(filter),
        gamma(gamma_in) {}

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
            size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM>
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
                loopshapingDefinition_(std::move(loopshapingDefinition))
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

            switch (loopshapingDefinition_->getType()) {
                case LoopshapingType::outputpattern :
                    // Solve
                    // 0 = A_r*x_r + B_r*u
                    // v = C_r*x_r + D_r*u
                    filter.findEquilibriumForInput(system_input, filter_state, filter_input);
                    break;
                case LoopshapingType::inputpattern :
                case LoopshapingType::eliminatepattern :
                    // Solve
                    // [0  =  [  A_s    B_s    [x_s
                    //  u]       C_s    D_s  ]  v_s]
                    filter.findEquilibriumForOutput(system_input, filter_state, filter_input);
                    break;
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

            switch (loopshapingDefinition_->getType()) {
                case LoopshapingType::outputpattern :
                    filterstateDerivative = filter.getA() * filter_state_ + filter.getB() * system_input;
                    break;
                case LoopshapingType::inputpattern :
                case LoopshapingType::eliminatepattern :
                    filterstateDerivative  = filter.getA() * filter_state_ + filter.getB() * filter_input;
                    break;
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
