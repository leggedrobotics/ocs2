/*!
 * @author    David Hoeller
 * @email     dhoeller@ethz.ch
 * @author    Farbod Farshidian
 * @email     farshidian@ethz.ch
 *
 * Copyright (C) 2018 Robotic Systems Lab, ETH Zurich.
 * All rights reserved.
 * http://www.rsl.ethz.ch/
 */

#ifndef DOUBLE_INTEGRATOR_COST_H
#define DOUBLE_INTEGRATOR_COST_H

#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <ocs2_core/logic/rules/NullLogicRules.h>
#include "ocs2_robotic_examples/examples/double_integrator/definitions.h"

namespace ocs2 {

class DoubleIntegratorCost : public QuadraticCostFunction<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_, NullLogicRules <double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<DoubleIntegratorCost> Ptr;
	typedef std::shared_ptr<const DoubleIntegratorCost> ConstPtr;

typedef NullLogicRules<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_> logic_rules_t;

typedef QuadraticCostFunction<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_, logic_rules_t> BASE;
typedef typename BASE::scalar_t scalar_t;
typedef typename BASE::state_vector_t state_vector_t;
typedef typename BASE::state_matrix_t state_matrix_t;
typedef typename BASE::input_vector_t input_vector_t;
	DoubleIntegratorCost(
			const state_matrix_t& Q,
			const input_matrix_t& R,
			const state_matrix_t& Q_final)
	:QuadraticCostFunction(Q, R, state_vector_t::Zero(), input_vector_t::Zero(), Q_final, state_vector_t::Zero())
	{}

	~DoubleIntegratorCost() = default;

	DoubleIntegratorCost* clone() const {
		return new DoubleIntegratorCost(*this);
	}

	void setCurrentStateAndControl(
			const scalar_t& t,
			const state_vector_t& x,
			const input_vector_t& u) final {

    dynamic_vector_t xNominalDynamic;
    BASE::xNominalFunc_.interpolate(t, xNominalDynamic);
    dynamic_vector_t uNominalDynamic;
    BASE::uNominalFunc_.interpolate(t, uNominalDynamic);

    BASE::setCurrentStateAndControl(t, x, u, xNominalDynamic, uNominalDynamic, xNominalDynamic);

  }

private:

};

} //namespace ocs2

#endif //DOUBLE_INTEGRATOR_COST_H
