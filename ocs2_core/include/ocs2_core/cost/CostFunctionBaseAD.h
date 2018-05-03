/*
 * CostFunctionBaseAD.h
 *
 *  Created on: Apr 25, 2018
 *      Author: farbod
 */

#ifndef COSTFUNCTIONBASEAD_OCS2_H_
#define COSTFUNCTIONBASEAD_OCS2_H_

#include "ocs2_core/cost/CostFunctionBase.h"
#include <ocs2_core/automatic_differentiation/CppAdCodeGenInterface.h>

namespace ocs2{

/**
 * Cost Function Base with Algorithmic Differentiation (i.e. Auto Differentiation).
 *
 * @tparam Derived: Derived class type.
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules<STATE_DIM,INPUT_DIM>>
class CostFunctionBaseAD : public CostFunctionBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		state_dim_ 	= STATE_DIM,
		input_dim_ 	= INPUT_DIM,
		domain_dim_	= 1 + state_dim_ + input_dim_,
	};

	typedef std::shared_ptr<CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;
	typedef std::shared_ptr<const CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > ConstPtr;

	typedef CostFunctionBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> BASE;

	typedef typename BASE::scalar_t       			scalar_t;
	typedef typename BASE::scalar_array_t 			scalar_array_t;
	typedef typename BASE::state_vector_t 			state_vector_t;
	typedef typename BASE::state_vector_array_t 	state_vector_array_t;
	typedef typename BASE::state_matrix_t 			state_matrix_t;
	typedef typename BASE::input_vector_t  			input_vector_t;
	typedef typename BASE::input_vector_array_t 	input_vector_array_t;
	typedef typename BASE::control_matrix_t 		control_matrix_t;
	typedef typename BASE::control_feedback_t 		control_feedback_t;


	/**
	 * Default constructor
	 */
	CostFunctionBaseAD();

	/**
	 * Copy constructor
	 */
	CostFunctionBaseAD(const CostFunctionBaseAD& rhs);

	/**
	 * Default destructor
	 */
	virtual ~CostFunctionBaseAD() = default;

    /**
     * Returns pointer to the base class.
     *
     * @return A raw pointer to the class.
     */
	virtual BASE* clone() const override;

	/**
	 * Interface method to the intermediate cost function. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [in] input: input vector
	 * @param [out] costValue: cost value.
	 */
	template <typename SCALAR_T>
	void intermediateCostFunction(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
			SCALAR_T& costValue);

	/**
	 * Interface method to the terminal cost function. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [out] costValue: cost value.
	 */
	template <typename SCALAR_T>
	void terminalCostFunction(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			SCALAR_T& costValue);

	/**
	 * creates the forward model, the Jacobian model, and the Hessian model.
	 *
	 * @param modelName
	 * @param libraryFolder
	 * @param verbose
	 */
	void createModels(
			const std::string& modelName,
			const std::string& libraryFolder);

	/**
	 * laods the forward model, the Jacobian model, and the Hessian model.
	 *
	 * @param modelName
	 * @param libraryFolder
	 * @param verbose
	 */
	void loadModels(
			const std::string& modelName,
			const std::string& libraryFolder);

	/**
	 * Whether or not the dynamic library is compiled.
	 *
	 * @return true if the dynamic library is compiled
	 */
	bool isDynamicLibraryCompiled() const;

	/**
	 * Gets model name.
	 *
	 * @return model name
	 */
	std::string getModelName() const;

    /**
     * Sets the current time, state, and control input
     *
     * @param [in] t: Current time
     * @param [in] x: Current state vector
     * @param [in] u: Current input vector
     */
	virtual void setCurrentStateAndControl(
			const scalar_t& t,
			const state_vector_t& x,
			const input_vector_t& u) override;

    /**
     * Get the intermediate cost.
     *
     * @param [out] L: The intermediate cost value.
     */
	virtual void getIntermediateCost(scalar_t& L) override;

    /**
     * Get the state derivative of the intermediate cost.
     *
     * @param [out] dLdx: First order derivative of the intermediate cost with respect to state vector.
     */
	virtual void getIntermediateCostDerivativeState(state_vector_t& dLdx) override;

    /**
     * Get state second order derivative of the intermediate cost.
     *
     * @param [out] dLdxx: Second order derivative of the intermediate cost with respect to state vector.
     */
	virtual void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) override;

    /**
     * Get control input derivative of the intermediate cost.
     *
     * @param [out] dLdu: First order derivative of the intermediate cost with respect to input vector.
     */
	virtual void getIntermediateCostDerivativeInput(input_vector_t& dLdu) override;

    /**
     * Get control input second derivative of the intermediate cost.
     *
     * @param [out] dLduu: Second order derivative of the intermediate cost with respect to input vector.
     */
	virtual void getIntermediateCostSecondDerivativeInput(control_matrix_t& dLduu) override;

    /**
     * Get the input-state derivative of the intermediate cost.
     *
     * @param [out] dLdux: Second order derivative of the intermediate cost with respect to input vector and state.
     */
	virtual void getIntermediateCostDerivativeInputState(control_feedback_t& dLdux) override;

    /**
     * Get the terminal cost.
     *
     * @param [out] Phi: The final cost value.
     */
	virtual void getTerminalCost(scalar_t& Phi) override;

    /**
     * Get the terminal cost state derivative.
     *
     * @param [out] dPhidx: First order final cost derivative with respect to state vector.
     */
	virtual void getTerminalCostDerivativeState(state_vector_t& dPhidx) override;

    /**
     * Get the terminal cost state second derivative
     *
     * @param [out] dPhidxx: Second order final cost derivative with respect to state vector.
     */
	virtual void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) override;

protected:
	typedef ocs2::CppAdCodeGenInterface<domain_dim_, 1, scalar_t> ad_interface_t;

	typedef typename ad_interface_t::ad_scalar_t			ad_scalar_t;
	typedef typename ad_interface_t::ad_dynamic_vector_t	ad_dynamic_vector_t;
	typedef typename ad_interface_t::ad_funtion_t			ad_funtion_t;
	typedef typename ad_interface_t::domain_vector_t		domain_vector_t;
	typedef typename ad_interface_t::domain_matrix_t		domain_matrix_t;
	typedef typename ad_interface_t::domain_range_matrix_t 	domain_range_matrix_t;
	typedef typename ad_interface_t::range_domain_matrix_t 	range_domain_matrix_t;

	/**
	 * The intermediate cost function specialized with AD type.
	 *
	 * @param [in] tapedInput: concatenated input vector
	 * @param [out] costValue: cost value
	 */
	void intermediateCostFunctionAD(
			const ad_dynamic_vector_t& tapedInput,
			ad_dynamic_vector_t& costValue);

	/**
	 * The terminal cost function specialized with AD type.
	 *
	 * @param [in] tapedInput: concatenated input vector
	 * @param [out] costValue: cost value
	 */
	void terminalCostFunctionAD(
			const ad_dynamic_vector_t& tapedInput,
			ad_dynamic_vector_t& costValue);

	/**
	 * Create the forward model, the Jacobian model, and the Hessian model.
	 *
	 * @param [in] verbose: display information.
	 */
	void createModels(bool verbose);

	/**
	 * Loads the forward model, the Jacobian model, and the Hessian model.
	 *
	 * @param [in] verbose: display information
	 */
	void loadModels(bool verbose);

private:
	bool dynamicLibraryIsCompiled_;
	std::string modelName_;
	std::string libraryFolder_;

	ad_funtion_t intermediateCostAD_;
	ad_funtion_t terminalCostAD_;

	range_domain_matrix_t intermediateSparsityPattern_;
	range_domain_matrix_t terminalSparsityPattern_;

	std::unique_ptr<ad_interface_t> intermediateADInterfacePtr_;
	std::unique_ptr<ad_interface_t> terminalADInterfacePtr_;

	domain_vector_t 		tapedInput_;

	domain_range_matrix_t 	intermediateJacobian_;
	domain_matrix_t 		intermediateHessian_;

	domain_range_matrix_t 	terminalJacobian_;
	domain_matrix_t 		terminalHessian_;

};

} // namespace ocs2

#include "implementation/CostFunctionBaseAD.h"

#endif /* COSTFUNCTIONBASEAD_OCS2_H_ */
