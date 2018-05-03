/*
 * SystemDynamicsBaseAD.h
 *
 *  Created on: Apr 27, 2018
 *      Author: farbod
 */

#ifndef SYSTEMDYNAMICSBASEAD_OCS2_H_
#define SYSTEMDYNAMICSBASEAD_OCS2_H_

#include "ocs2_core/dynamics/DerivativesBase.h"
#include <ocs2_core/automatic_differentiation/CppAdCodeGenInterface.h>

namespace ocs2{

/**
 * The system dynamics Base with Algorithmic Differentiation (i.e. Auto Differentiation).
 * The linearized system flow map is defined as: \n
 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
 * The linearized system jump map is defined as: \n
 * \f$ x^+ = G \delta x + H \delta u \f$ \n
 *
 * @tparam Derived: Derived class type.
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules<STATE_DIM,INPUT_DIM>>
class SystemDynamicsBaseAD : public DerivativesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		state_dim_ 	= STATE_DIM,
		input_dim_ 	= INPUT_DIM,
		domain_dim_	= 1 + state_dim_ + input_dim_,
	};

	typedef std::shared_ptr<SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;
	typedef std::shared_ptr<const SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > ConstPtr;

	typedef DerivativesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> BASE;

	typedef typename BASE::scalar_t       			scalar_t;
	typedef typename BASE::state_vector_t 			state_vector_t;
	typedef typename BASE::state_matrix_t 			state_matrix_t;
	typedef typename BASE::input_vector_t  			input_vector_t;
	typedef typename BASE::control_gain_matrix_t 	control_gain_matrix_t;

	/**
	 * Default constructor
	 */
	SystemDynamicsBaseAD();

	/**
	 * Copy constructor
	 */
	SystemDynamicsBaseAD(const SystemDynamicsBaseAD& rhs);

	/**
	 * Default destructor
	 */
	virtual ~SystemDynamicsBaseAD() = default;

    /**
     * Returns pointer to the base class.
     *
     * @return A raw pointer to the class.
     */
	virtual BASE* clone() const override;

	/**
	 * Interface method to the state flow map of the hybrid system. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [in] input: input vector
	 * @param [out] stateDerivative: state vector time derivative.
	 */
	template <typename SCALAR_T>
	void systemFlowMap(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
			Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& stateDerivative);

	/**
	 * Interface method to the state jump map of the hybrid system. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [out] jumpedState: jumped state.
	 */
	template <typename SCALAR_T>
	void systemJumpMap(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
			Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& jumpedState);

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
     * Sets the current time, state, and control input.
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
     * Get the flow map value.
     *
     * @param [out] f: The flow map value.
     */
	virtual void getFlowMap(state_vector_t& f);

    /**
     * Get the jump map value.
     *
     * @param [out] g: The jump map value.
     */
	virtual void getJumpMap(state_vector_t& g);

	/**
	 * Partial time derivative of the system flow map.
	 * \f$ \frac{\partial f}{\partial t}  \f$.
	 *
	 * @param [out] df: \f$ \frac{\partial f}{\partial t} \f$ matrix.
	 */
	virtual void getFlowMapDerivativeTime(state_vector_t& df) override;

	/**
	 * The A matrix at a given operating point for the linearized system flow map.
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] A: \f$ A(t) \f$ matrix.
	 */
	virtual void getFlowMapDerivativeState(state_matrix_t& A) override;

	/**
	 * The B matrix at a given operating point for the linearized system flow map.
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] B: \f$ B(t) \f$ matrix.
	 */
	virtual void getFlowMapDerivativeInput(control_gain_matrix_t& B) override;

	/**
	 * Partial time derivative of the system jump map.
	 * \f$ \frac{\partial g}{\partial t}  \f$.
	 *
	 * @param [out] dg: \f$ \frac{\partial g}{\partial t} \f$ matrix.
	 */
	virtual void getJumpMapDerivativeTime(state_vector_t& dg) override;

	/**
	 * The G matrix at a given operating point for the linearized system jump map.
	 * \f$ x^+ = G \delta x + H \delta u \f$.
	 *
	 * @param [out] G: \f$ G \f$ matrix.
	 */
	virtual void getJumpMapDerivativeState(state_matrix_t& G) override;

	/**
	 * The H matrix at a given operating point for the linearized system jump map.
	 * \f$ x^+ = G \delta x + H \delta u \f$.
	 *
	 * @param [out] H: \f$ H \f$ matrix.
	 */
	virtual void getJumpMapDerivativeInput(control_gain_matrix_t& H) override;


protected:
	typedef ocs2::CppAdCodeGenInterface<domain_dim_, state_dim_, scalar_t> ad_interface_t;

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
	 * @param [out] f: The flow map value
	 */
	void systemFlowMapAD(
			const ad_dynamic_vector_t& tapedInput,
			ad_dynamic_vector_t& f);

	/**
	 * The terminal cost function specialized with AD type.
	 *
	 * @param [in] tapedInput: concatenated input vector
	 * @param [out] g: The jump map value
	 */
	void systemJumpMapAD(
			const ad_dynamic_vector_t& tapedInput,
			ad_dynamic_vector_t& g);

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

	ad_funtion_t systemFlowMapAD_;
	ad_funtion_t systemJumpMapAD_;

	range_domain_matrix_t flowSparsityPattern_;
	range_domain_matrix_t jumpSparsityPattern_;

	std::unique_ptr<ad_interface_t> flowMapADInterfacePtr_;
	std::unique_ptr<ad_interface_t> jumpMapADInterfacePtr_;

	domain_vector_t 		tapedInput_;

	domain_range_matrix_t 	flowJacobian_;

	domain_range_matrix_t 	jumpJacobian_;
};

} // namespace ocs2

#include "implementation/SystemDynamicsBaseAD.h"

#endif /* SYSTEMDYNAMICSBASEAD_OCS2_H_ */
