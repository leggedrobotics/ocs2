/*
 * ConstraintBaseAD.h
 *
 *  Created on: May 2, 2018
 *      Author: farbod
 */

#ifndef CONSTRAINTBASEAD_OCS2_H_
#define CONSTRAINTBASEAD_OCS2_H_

#include "ocs2_core/constraint/ConstraintBase.h"
#include <ocs2_core/automatic_differentiation/CppAdCodeGenInterface.h>

namespace ocs2{

/**
 * The system constraint Base with Algorithmic Differentiation (i.e. Auto Differentiation).
 * The linearized state-input constraint is defined as: \n
 * \f$ 0 = C(t) \delta x + D(t) \delta u + e(t) \f$ \n
 * The linearized state-only constraint is defined as: \n
 * \f$ 0 = F(t) \delta x + h(t) \f$ \n
 * The linearized final state-only constraint is defined as: \n
 * \f$ 0 = F_f \delta x + h_f \f$ \n
 *
 * @tparam Derived: Derived class type.
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules<STATE_DIM,INPUT_DIM>>
class ConstraintBaseAD : public ConstraintBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		state_dim_ 	= STATE_DIM,
		input_dim_ 	= INPUT_DIM,
		domain_dim_	= 1 + state_dim_ + input_dim_,
	};

	typedef std::shared_ptr<ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;
	typedef std::shared_ptr<const ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > ConstPtr;

	typedef ConstraintBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> BASE;

	enum {
		MAX_CONSTRAINT_DIM_ = INPUT_DIM
	};

	typedef typename BASE::scalar_t 					scalar_t;
	typedef typename BASE::state_vector_t   			state_vector_t;
	typedef typename BASE::input_vector_t 				input_vector_t;
	typedef typename BASE::state_matrix_t   			state_matrix_t;
	typedef typename BASE::state_input_matrix_t 		state_input_matrix_t;
	typedef typename BASE::constraint1_vector_t 		constraint1_vector_t;
	typedef typename BASE::constraint2_vector_t 		constraint2_vector_t;
	typedef typename BASE::constraint1_state_matrix_t   constraint1_state_matrix_t;
	typedef typename BASE::constraint1_input_matrix_t constraint1_input_matrix_t;
	typedef typename BASE::constraint2_state_matrix_t   constraint2_state_matrix_t;

	/**
	 * Default constructor.
	 *
	 * @param [in] dynamicLibraryIsCompiled: Whether a library is already complied.
	 */
	ConstraintBaseAD(const bool& dynamicLibraryIsCompiled = false);

	/**
	 * Copy constructor
	 */
	ConstraintBaseAD(const ConstraintBaseAD& rhs);

	/**
	 * Default destructor
	 */
	virtual ~ConstraintBaseAD() = default;

	/**
	 * Returns pointer to the base class.
	 *
	 * @return A raw pointer to the class.
	 */
	virtual BASE* clone() const override;

	/**
	 * Interface method to the state-input equality constraints. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [in] input: input vector
	 * @param [out] constraintVector: constraints vector.
	 */
	template <typename SCALAR_T>
	void stateInputConstraint(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
			Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& constraintVector) {}

	/**
	 * Get the number of state-input active equality constraints.
	 *
	 * @param [in] time: time.
	 * @return number of state-input active equality constraints.
	 */
	virtual size_t numStateInputConstraint(const scalar_t& time) override {

		return 0;
	}

	/**
	 * Interface method to the state-only equality constraints. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [out] constraintVector: constraint vector.
	 */
	template <typename SCALAR_T>
	void stateOnlyConstraint(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& constraintVector) {}

	/**
	 * Get the number of state-only active equality constraints.
	 *
	 * @param [in] time: time.
	 * @return number of state-only active equality constraints.
	 */
	virtual size_t numStateOnlyConstraint(const scalar_t& time) override {

		return 0;
	}

	/**
	 * Interface method to the state-only final equality constraints. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [out] constraintVector: constraint vector.
	 */
	template <typename SCALAR_T>
	void stateOnlyFinalConstraint(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& constraintVector) {}

	/**
	 * Get the number of final state-only active equality constraints.
	 *
	 * @param [in] time: time.
	 * @return number of final state-only active equality constraints.
	 */
	virtual size_t numStateOnlyFinalConstraint(const scalar_t& time) override {

		return 0;
	}

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
	const bool& isDynamicLibraryCompiled() const;

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
	 * Computes the state-input equality constraints.
	 *
	 * @param [out] e: The state-input equality constraints value.
	 */
	virtual void getConstraint1(constraint1_vector_t& e) override;

	/**
	 * Compute the state-only equality constraints.
	 *
	 * @param [out] h: The state-only equality constraints value.
	 */
	virtual void getConstraint2(constraint2_vector_t& h) override;

	/**
	 * Compute the final state-only equality constraints.
	 *
	 * @param [out] h_f: The final state-only equality constraints value.
	 */
	virtual void getFinalConstraint2(constraint2_vector_t& h_f) override;

	/**
	 * The C matrix at a given operating point for the linearized state-input constraints,
	 * \f$ C(t) \delta x + D(t) \delta u + e(t) = 0 \f$.
	 *
	 * @param [out] C: \f$ C(t) \f$ matrix.
	 */
	virtual void getConstraint1DerivativesState(constraint1_state_matrix_t& C) override;

	/**
	 * The D matrix at a given operating point for the linearized state-input constraints,
	 * \f$ C(t) \delta x + D(t) \delta u + e(t) = 0 \f$.
	 *
	 * @param [out] D: \f$ D(t) \f$ matrix.
	 */
	virtual void getConstraint1DerivativesControl(constraint1_input_matrix_t& D) override;

	/**
	 * The F matrix at a given operating point for the linearized state-only constraints,
	 * \f$ F(t) \delta x + h(t) = 0 \f$.
	 *
	 * @param [out] F: \f$ F(t) \f$ matrix.
	 */
	virtual void getConstraint2DerivativesState(constraint2_state_matrix_t& F) override;

	/**
	 * The F matrix at a given operating point for the linearized terminal state-only constraints,
	 * \f$ F_f(t) \delta x + h_f(t) = 0 \f$.
	 *
	 * @param [out] F_f: \f$ F_f(t) \f$ matrix.
	 */
	virtual void getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F_f) override;


protected:
	typedef ocs2::CppAdCodeGenInterface<domain_dim_, MAX_CONSTRAINT_DIM_, scalar_t> ad_interface_t;

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
	 * @param [out] g1: The flow map value
	 */
	void stateInputConstraintAD(
			const ad_dynamic_vector_t& tapedInput,
			ad_dynamic_vector_t& g1);

	/**
	 * The terminal cost function specialized with AD type.
	 *
	 * @param [in] tapedInput: concatenated input vector
	 * @param [out] g2: The jump map value
	 */
	void stateOnlyConstraintAD(
			const ad_dynamic_vector_t& tapedInput,
			ad_dynamic_vector_t& g2);

	/**
	 * The terminal cost function specialized with AD type.
	 *
	 * @param [in] tapedInput: concatenated input vector
	 * @param [out] g2Final: The jump map value
	 */
	void stateOnlyConstraintFinalAD(
			const ad_dynamic_vector_t& tapedInput,
			ad_dynamic_vector_t& g2Final);

	/**
	 * Sets all the required CppAdCodeGenInterfaces
	 */
	void setADInterfaces();

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
	 * @return true if it successfully loads the library.
	 */
	bool loadModels(bool verbose);

private:
	bool dynamicLibraryIsCompiled_;
	std::string modelName_;
	std::string libraryFolder_;

	ad_funtion_t stateInputConstraintAD_;
	ad_funtion_t stateOnlyConstraintAD_;
	ad_funtion_t stateOnlyConstraintFinalAD_;

	range_domain_matrix_t stateInputSparsityPattern_;
	range_domain_matrix_t stateOnlySparsityPattern_;
	range_domain_matrix_t stateOnlyFinalSparsityPattern_;

	std::unique_ptr<ad_interface_t> stateInputADInterfacePtr_;
	std::unique_ptr<ad_interface_t> stateOnlyADInterfacePtr_;
	std::unique_ptr<ad_interface_t> stateOnlyFinalADInterfacePtr_;

	domain_vector_t 		tapedInput_;

	domain_range_matrix_t 	stateInputJacobian_;
	domain_range_matrix_t 	stateOnlyJacobian_;
	domain_range_matrix_t 	stateOnlyFinalJacobian_;
};

} // namespace ocs2

#include "implementation/ConstraintBaseAD.h"

#endif /* CONSTRAINTBASEAD_OCS2_H_ */
