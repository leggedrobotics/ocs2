//
// Created by ruben on 08.11.18.
//

#ifndef OCS2_LOOPSHAPINGOPERATINGPOINT_H
#define OCS2_LOOPSHAPINGOPERATINGPOINT_H

#include "ocs2_core/initialization/SystemOperatingTrajectoriesBase.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"

namespace ocs2 {
    template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
        size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
        size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
        class LOGIC_RULES_T=NullLogicRules>
class LoopshapingOperatingPoint final :
    public SystemOperatingTrajectoriesBase<FULL_STATE_DIM, FULL_INPUT_DIM, LOGIC_RULES_T>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<LoopshapingOperatingPoint> ;

    using BASE = SystemOperatingTrajectoriesBase<FULL_STATE_DIM, FULL_INPUT_DIM, LOGIC_RULES_T>;
    using scalar_t = typename BASE::scalar_t;
    using scalar_array_t = typename BASE::scalar_array_t;
    using size_array_t = typename BASE::size_array_t;
    using state_vector_t = typename BASE::state_vector_t;
    using state_vector_array_t = typename BASE::state_vector_array_t;
    using input_vector_t = typename BASE::input_vector_t;
    using input_vector_array_t = typename BASE::input_vector_array_t;

    using SYSTEMBASE = SystemOperatingTrajectoriesBase<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, LOGIC_RULES_T>;
    static constexpr size_t system_state_dim = SYSTEM_STATE_DIM;
    static constexpr size_t system_input_dim = SYSTEM_INPUT_DIM;
    using system_state_vector_t = typename SYSTEMBASE::state_vector_t;
    using system_state_vector_array_t = typename SYSTEMBASE::state_vector_array_t;
    using system_input_vector_t = typename SYSTEMBASE::input_vector_t;
    using system_input_vector_array_t = typename SYSTEMBASE::input_vector_array_t;

    using filter_dynamics_t = LoopshapingFilterDynamics<
        FULL_STATE_DIM, FULL_INPUT_DIM,
        SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM,
        FILTER_STATE_DIM, FILTER_INPUT_DIM>;
    static constexpr size_t filter_state_dim = FILTER_STATE_DIM;
    static constexpr size_t filter_input_dim = FILTER_INPUT_DIM;
    using filter_state_vector_t = Eigen::Matrix<scalar_t, filter_state_dim, 1>;
    using filter_input_vector_t = Eigen::Matrix<scalar_t, filter_input_dim, 1>;

    LoopshapingOperatingPoint(
        const SYSTEMBASE& systembase,
        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
        : BASE(),
          systembase_(systembase.clone()),
          loopshapingDefinition_(loopshapingDefinition),
          filterDynamics_(new filter_dynamics_t(loopshapingDefinition))
    {}


    virtual ~LoopshapingOperatingPoint() {}

    virtual void initializeModel(
        LogicRulesMachine<LOGIC_RULES_T>& logicRulesMachine,
        const size_t& partitionIndex,
        const char* algorithmName=NULL) override {

      BASE::initializeModel(logicRulesMachine, partitionIndex, algorithmName);
      systembase_->initializeModel(logicRulesMachine, partitionIndex, algorithmName);
    }

    LoopshapingOperatingPoint(const LoopshapingOperatingPoint& obj) :
        BASE(),
        systembase_(obj.systembase_->clone()),
        loopshapingDefinition_(obj.loopshapingDefinition_),
        filterDynamics_(new filter_dynamics_t(obj.loopshapingDefinition_))
    {}

    virtual LoopshapingOperatingPoint* clone() const override {
      return new LoopshapingOperatingPoint(*this);
    }

    virtual void getSystemOperatingTrajectories(
        const state_vector_t& initialState,
        const scalar_t& startTime,
        const scalar_t& finalTime,
        scalar_array_t& timeTrajectory,
        state_vector_array_t& stateTrajectory,
        input_vector_array_t& inputTrajectory,
        bool concatOutput = false) override {

      if (concatOutput==false) {
        timeTrajectory.clear();
        stateTrajectory.clear();
        inputTrajectory.clear();
      }

      system_state_vector_t initialSystemState;
      scalar_array_t systemTimeTrajectory;
      system_state_vector_array_t systemStateTrajectory;
      system_input_vector_array_t systemInputTrajectory;
      loopshapingDefinition_->getSystemState(initialState, initialSystemState);

      // Get system operating point
      systembase_->getSystemOperatingTrajectories(initialSystemState, startTime, finalTime, systemTimeTrajectory,
      systemStateTrajectory,
      systemInputTrajectory,
      false);

      // Filter operating point
      filter_state_vector_t equilibriumFilterState;
      filter_input_vector_t equilibriumFilterInput;
      state_vector_t state;
      input_vector_t input;
      for (int k = 0; k < systemTimeTrajectory.size(); ++k){
        filterDynamics_->initializeEquilibriumFilterStateInput(systemStateTrajectory[k], systemInputTrajectory[k],
                                                               equilibriumFilterState, equilibriumFilterInput);
        loopshapingDefinition_->concatenateSystemAndFilterInput(systemInputTrajectory[k], equilibriumFilterInput, input);
        loopshapingDefinition_->concatenateSystemAndFilterState(systemStateTrajectory[k], equilibriumFilterState, state);
        timeTrajectory.emplace_back(systemTimeTrajectory[k]);
        inputTrajectory.emplace_back(input);
        stateTrajectory.emplace_back(state);
      }

//      std::cout << "============ Operating points ======= " << std::endl;
//      for (int i=0; i<timeTrajectory.size(); i++){
//        std::cout << "t: " << timeTrajectory[i] << std::endl;
//        std::cout << "x: \n" << stateTrajectory[i].transpose() << std::endl;
//        std::cout << "u: \n" << inputTrajectory[i].transpose() << std::endl;
//      }
    }

private:
    std::unique_ptr<SYSTEMBASE> systembase_;
    std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
    std::unique_ptr<filter_dynamics_t> filterDynamics_;
};
} // namespace ocs2

#endif //OCS2_LOOPSHAPINGOPERATINGPOINT_H
