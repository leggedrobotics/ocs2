/*
 * AnymalCost.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/cost/AnymalCost.h"
#include "ocs2_anymal_switched_model/dynamics/AnymalCom.h"

namespace anymal {

AnymalCost::AnymalCost(std::shared_ptr<const logic_rules_t> logicRulesPtr, const state_matrix_t& Q, const input_matrix_t& R,
                       const state_matrix_t& QFinal)

    : Base(AnymalCom(), std::move(logicRulesPtr), Q, R, QFinal) {}

AnymalCost::AnymalCost(const AnymalCost& rhs) : Base(rhs) {}

}  // end of namespace anymal
