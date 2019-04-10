//
// Created by ruben on 24.09.18.
//

#ifndef OCS2_LOOPSHAPINGCOST_H
#define OCS2_LOOPSHAPINGCOST_H

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/cost/QuadraticCostFunction.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"

namespace ocs2 {
    template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
        size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
        size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
        class LOGIC_RULES_T=NullLogicRules>
    class LoopshapingCost final : public CostFunctionBase<FULL_STATE_DIM, FULL_INPUT_DIM, LOGIC_RULES_T>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using Ptr = std::shared_ptr<LoopshapingCost>;
        using ConstPtr = std::shared_ptr<const LoopshapingCost>;

        using BASE = CostFunctionBase<FULL_STATE_DIM, FULL_INPUT_DIM, LOGIC_RULES_T>;
        using scalar_t = typename BASE::scalar_t;
        using state_vector_t = typename BASE::state_vector_t;
        using state_matrix_t = typename BASE::state_matrix_t;
        using input_matrix_t = typename BASE::input_matrix_t;
        using input_vector_t = typename BASE::input_vector_t;
        using dynamic_vector_t = typename BASE::dynamic_vector_t;
        using input_state_matrix_t = typename BASE::input_state_matrix_t;
        using cost_desired_trajectories_t = typename BASE::cost_desired_trajectories_t;
        static constexpr size_t full_state_dim = FULL_STATE_DIM;
        static constexpr size_t full_input_dim = FULL_INPUT_DIM;

        using SYSTEMCOST = CostFunctionBase<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, LOGIC_RULES_T>;
        static constexpr size_t system_state_dim = SYSTEM_STATE_DIM;
        static constexpr size_t system_input_dim = SYSTEM_INPUT_DIM;
        using system_state_vector_t = typename SYSTEMCOST::state_vector_t;
        using system_input_vector_t = typename SYSTEMCOST::input_vector_t;
        using system_state_matrix_t = typename SYSTEMCOST::state_matrix_t;
        using system_input_matrix_t = typename SYSTEMCOST::input_matrix_t;
        using system_input_state_matrix_t = typename SYSTEMCOST::input_state_matrix_t;

        static constexpr size_t filter_state_dim = FILTER_STATE_DIM;
        static constexpr size_t filter_input_dim = FILTER_INPUT_DIM;
        using filter_state_vector_t = Eigen::Matrix<scalar_t, FILTER_STATE_DIM, 1>;
        using filter_input_vector_t = Eigen::Matrix<scalar_t, FILTER_INPUT_DIM, 1>;

        using homogeneous_cost_matrix_t = Eigen::Matrix<scalar_t,
            full_state_dim + full_input_dim + 1,
            full_state_dim + full_input_dim + 1>;
        using homogeneous_vector_t = Eigen::Matrix<scalar_t,
            full_state_dim + full_input_dim + 1,
            1>;
        using homogeneous_system_cost_matrix_t = Eigen::Matrix<scalar_t,
            system_state_dim + system_input_dim + 1,
            system_state_dim + system_input_dim + 1>;
        using homogeneous_system_vector_t = Eigen::Matrix<scalar_t,
            system_state_dim + system_input_dim + 1,
            1>;
        using homogeneous_transform_matrix_t = Eigen::Matrix<scalar_t,
            system_state_dim + system_input_dim + 1,
            full_state_dim + full_input_dim + 1>;

        LoopshapingCost(const SYSTEMCOST& systemCost,
                          std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
        : BASE(),
          systemCost_(systemCost.clone()),
          loopshapingDefinition_(loopshapingDefinition)
        {
        }

        ~LoopshapingCost() override = default;

        LoopshapingCost(const LoopshapingCost& obj) :
            BASE(),
            systemCost_(obj.systemCost_->clone()),
            loopshapingDefinition_(obj.loopshapingDefinition_)
        {
        }

        LoopshapingCost* clone() const override {
          return new LoopshapingCost(*this);
        };

        void initializeModel(
            LogicRulesMachine<LOGIC_RULES_T>& logicRulesMachine,
            const size_t& partitionIndex,
            const char* algorithmName=NULL) override {
          BASE::initializeModel(logicRulesMachine, partitionIndex, algorithmName);
          systemCost_->initializeModel(logicRulesMachine, partitionIndex, algorithmName);
        }

        void setCurrentStateAndControl(
            const scalar_t& t,
            const state_vector_t& x,
            const input_vector_t& u) override {
          auto& r_filter = loopshapingDefinition_->getInputFilter_r();
          auto& s_filter = loopshapingDefinition_->getInputFilter_s();

          dynamic_vector_t xNominal;
          xNominalFunc_.interpolate(t, xNominal);
          dynamic_vector_t uNominal;
          uNominalFunc_.interpolate(t, uNominal);

          loopshapingDefinition_->getSystemState(x, x_system_);
          loopshapingDefinition_->getSystemInput(x, u, u_system_);
          loopshapingDefinition_->getFilterState(x, x_filter_);
          loopshapingDefinition_->getFilteredInput(x, u, u_filter_);

          // Compute cost and approximation around system input
          systemCost_->setCurrentStateAndControl(t, x_system_, u_system_);
          systemCost_->getIntermediateCostSecondDerivativeState(Q_system_);
          systemCost_->getIntermediateCostDerivativeInputState(P_system_);
          systemCost_->getIntermediateCostSecondDerivativeInput(R_system_);
          systemCost_->getIntermediateCostDerivativeState(q_system_);
          systemCost_->getIntermediateCostDerivativeInput(r_system_);
          systemCost_->getIntermediateCost(c_system_);

          // Compute cost and approximation around filter input
          systemCost_->setCurrentStateAndControl(t, x_system_, u_filter_);
          systemCost_->getIntermediateCostSecondDerivativeState(Q_filter_);
          systemCost_->getIntermediateCostDerivativeInputState(P_filter_);
          systemCost_->getIntermediateCostSecondDerivativeInput(R_filter_);
          systemCost_->getIntermediateCostDerivativeState(q_filter_);
          systemCost_->getIntermediateCostDerivativeInput(r_filter_);
          systemCost_->getIntermediateCost(c_filter_);

          // set base class
          BASE::setCurrentStateAndControl(t, x, u);
        }

        void setCostDesiredTrajectories(const cost_desired_trajectories_t& costDesiredTrajectories) override {

          costDesiredTrajectories.getDesiredStateFunc(xNominalFunc_);
          costDesiredTrajectories.getDesiredInputFunc(uNominalFunc_);

          // Desired trajectories are dynamic size -> must resize for future cast to fixed size vectors
          size_t reference_length = costDesiredTrajectories.desiredTimeTrajectory().size();
          systemCostDesiredTrajectories_ = cost_desired_trajectories_t(reference_length);
          systemCostDesiredTrajectories_.desiredTimeTrajectory() = costDesiredTrajectories.desiredTimeTrajectory();
          auto& systemStateTrajectory = systemCostDesiredTrajectories_.desiredStateTrajectory();
          auto& systemInputTrajectory = systemCostDesiredTrajectories_.desiredInputTrajectory();
          auto& stateTrajectory = costDesiredTrajectories.desiredStateTrajectory();
          auto& inputTrajectory = costDesiredTrajectories.desiredInputTrajectory();
          for (int k=0; k<reference_length; k++){
            // For now assume that cost DesiredTrajectory is specified w.r.t original system x, u
            systemStateTrajectory[k] = stateTrajectory[k].segment(0, system_state_dim);
            systemInputTrajectory[k] = inputTrajectory[k].segment(0, system_input_dim);
          }

          systemCost_->setCostDesiredTrajectories(systemCostDesiredTrajectories_);
        }

        void getIntermediateCost(scalar_t& L) override
        {
          auto& gamma = loopshapingDefinition_->gamma;
          auto& r_filter = loopshapingDefinition_->getInputFilter_r();
          auto& s_filter = loopshapingDefinition_->getInputFilter_s();

          L = gamma* c_filter_ + (1.0-gamma)* c_system_;
        };

        void getIntermediateCostDerivativeTime(scalar_t& dLdt)  override
        {
          // TODO
          dLdt = 0;
        }

        void getIntermediateCostDerivativeState(state_vector_t& dLdx) override
        {
          auto& gamma = loopshapingDefinition_->gamma;
          auto& r_filter = loopshapingDefinition_->getInputFilter_r();
          auto& s_filter = loopshapingDefinition_->getInputFilter_s();

          if (s_filter.getNumStates() > 0 ) {
            dLdx.segment(0, system_state_dim) = gamma* q_filter_  + (1.0-gamma)* q_system_;
            dLdx.segment(system_state_dim, filter_state_dim).setZero();
          }

          if (r_filter.getNumStates() > 0 ) {
            dLdx.segment(0, system_state_dim) = gamma* q_filter_ + (1.0-gamma)* q_system_;
            dLdx.segment(system_state_dim, filter_state_dim) = gamma* r_filter.getC().transpose()* r_filter_;
          }

          if (loopshapingDefinition_->eliminateInputs) {
            throw std::runtime_error("Loopshaping setting with eliminate inputs not supported");
          }

        };

        void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) override
        {
          auto& gamma = loopshapingDefinition_->gamma;
          auto& r_filter = loopshapingDefinition_->getInputFilter_r();
          auto& s_filter = loopshapingDefinition_->getInputFilter_s();

          if (s_filter.getNumStates() > 0) {
            dLdxx.setZero();
            dLdxx.block(0, 0, system_state_dim, system_state_dim) = gamma* Q_filter_  + (1.0-gamma)* Q_system_;
          }

          if (r_filter.getNumStates() > 0 ) {
            dLdxx.block(0, 0, system_state_dim, system_state_dim) = gamma* Q_filter_  + (1.0-gamma)* Q_system_;
            dLdxx.block(0, system_state_dim, system_state_dim, filter_state_dim) = gamma* P_filter_.transpose() * r_filter.getC();
            dLdxx.block(system_state_dim, 0, filter_state_dim, system_state_dim) = dLdxx.block(0, system_state_dim, system_state_dim, filter_state_dim).transpose();
            dLdxx.block(system_state_dim, system_state_dim, filter_state_dim, filter_state_dim) = gamma * r_filter.getC().transpose() * R_filter_ * r_filter.getC();
          }

          if (loopshapingDefinition_->eliminateInputs) {
            throw std::runtime_error("Loopshaping setting with eliminate inputs not supported");
          }

        };

        void getIntermediateCostDerivativeInput(input_vector_t& dLdu) override
        {
          auto& gamma = loopshapingDefinition_->gamma;
          auto& r_filter = loopshapingDefinition_->getInputFilter_r();
          auto& s_filter = loopshapingDefinition_->getInputFilter_s();

          if (s_filter.getNumStates() > 0 ){
            dLdu.segment(0, system_input_dim) = (1.0 - gamma) * r_system_;
            dLdu.segment(system_input_dim, filter_input_dim) = gamma * r_filter_;
          }

          if (r_filter.getNumStates() > 0 ){
            dLdu.segment(0, system_input_dim) = gamma* r_filter.getD().transpose()* r_filter_ + (1.0-gamma) * r_system_;
          }

          if (loopshapingDefinition_->eliminateInputs) {
            throw std::runtime_error("Loopshaping setting with eliminate inputs not supported");
          }
        };

        void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) override
        {
          auto& gamma = loopshapingDefinition_->gamma;
          auto& r_filter = loopshapingDefinition_->getInputFilter_r();
          auto& s_filter = loopshapingDefinition_->getInputFilter_s();

          if (s_filter.getNumStates() > 0) {
            dLduu.setZero();
            dLduu.block(0, 0, system_input_dim, system_input_dim) = (1.0 - gamma) * R_system_;
            dLduu.block(system_input_dim, system_input_dim, filter_input_dim, filter_input_dim) = gamma * R_filter_;
          }

          if (r_filter.getNumStates() > 0) {
            dLduu.block(0, 0, system_input_dim, system_input_dim) = gamma* r_filter.getD().transpose() * R_filter_ * r_filter.getD() + (1.0-gamma) * R_system_;
          }

          if (loopshapingDefinition_->eliminateInputs) {
            throw std::runtime_error("Loopshaping setting with eliminate inputs not supported");
          }
        };

        void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdux) override
        {
          auto& gamma = loopshapingDefinition_->gamma;
          auto& r_filter = loopshapingDefinition_->getInputFilter_r();
          auto& s_filter = loopshapingDefinition_->getInputFilter_s();

          if (s_filter.getNumStates() > 0){
            dLdux.block(0, 0, system_input_dim, system_state_dim) = (1.0 - gamma) * P_system_;
            dLdux.block(0, system_state_dim, system_input_dim, filter_state_dim).setZero();
            dLdux.block(system_input_dim, 0, filter_input_dim, system_state_dim) = gamma * P_filter_;
            dLdux.block(system_input_dim, system_state_dim, filter_input_dim, filter_state_dim).setZero();
          }

          if (r_filter.getNumStates() > 0){
            dLdux.block(0, 0, system_input_dim, system_state_dim) = gamma* r_filter.getD().transpose()*P_filter_ + (1.0-gamma)*P_system_;
            dLdux.block(0, system_state_dim, system_input_dim, filter_state_dim) = gamma* r_filter.getD().transpose()*R_filter_*r_filter.getC();
          }

          if (loopshapingDefinition_->eliminateInputs) {
            throw std::runtime_error("Loopshaping setting with eliminate inputs not supported");
          }
        };

        void getTerminalCost(scalar_t& Phi) override
        {
          systemCost_->getTerminalCost(Phi);
        };

        virtual void getTerminalCostDerivativeTime(scalar_t& dPhidt) { dPhidt = 0; }

        void getTerminalCostDerivativeState(state_vector_t& dPhidx) override
        {
          system_state_vector_t dPhidx_system;
          systemCost_->getTerminalCostDerivativeState(dPhidx_system);
          dPhidx.segment(0, system_state_dim) = dPhidx_system;
          dPhidx.segment(system_state_dim, filter_state_dim).setZero();
        };

        void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) override
        {
          system_state_matrix_t dPhidxx_system;
          systemCost_->getTerminalCostSecondDerivativeState(dPhidxx_system);
          dPhidxx.setZero();
          dPhidxx.block(0, 0, system_state_dim, system_state_dim) = dPhidxx_system;
        };

    protected:
        cost_desired_trajectories_t systemCostDesiredTrajectories_;
        using BASE::costDesiredTrajectoriesPtr_;
        using BASE::xNominalFunc_;
        using BASE::uNominalFunc_;

    private:
        filter_state_vector_t x_filter_;
        filter_input_vector_t u_filter_;
        system_state_vector_t x_system_;
        system_input_vector_t u_system_;

        system_state_matrix_t Q_system_;
        system_input_matrix_t R_system_;
        system_input_state_matrix_t P_system_;
        system_state_vector_t q_system_;
        system_input_vector_t r_system_;
        scalar_t c_system_;

        system_state_matrix_t Q_filter_;
        system_input_matrix_t R_filter_;
        system_input_state_matrix_t P_filter_;
        system_state_vector_t q_filter_;
        system_input_vector_t r_filter_;
        scalar_t c_filter_;

        std::unique_ptr<SYSTEMCOST> systemCost_;
        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
    };
}; // namespace ocs2

#endif //OCS2_LOOPSHAPINGCOST_H
