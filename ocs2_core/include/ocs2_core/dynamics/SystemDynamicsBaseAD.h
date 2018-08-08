/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef SYSTEMDYNAMICSBASEAD_OCS2_H_
#define SYSTEMDYNAMICSBASEAD_OCS2_H_

#include "ocs2_core/dynamics/SystemDynamicsBase.h"
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
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules<STATE_DIM,INPUT_DIM>, size_t NUM_MODES=1>
class SystemDynamicsBaseAD : public SystemDynamicsBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T, NUM_MODES>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		num_modes_	= NUM_MODES,
		state_dim_ 	= STATE_DIM,
		input_dim_ 	= INPUT_DIM,
		domain_dim_	= 1 + state_dim_ + input_dim_,
	};

	typedef std::shared_ptr<SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, NUM_MODES> > Ptr;
	typedef std::shared_ptr<const SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, NUM_MODES> > ConstPtr;

	typedef SystemDynamicsBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T, NUM_MODES> BASE;

	typedef typename BASE::scalar_t       			scalar_t;
	typedef typename BASE::state_vector_t 			state_vector_t;
	typedef typename BASE::state_matrix_t 			state_matrix_t;
	typedef typename BASE::input_vector_t  			input_vector_t;
	typedef typename BASE::state_input_matrix_t 	state_input_matrix_t;
	typedef typename BASE::dynamic_vector_t			dynamic_vector_t;
	typedef typename BASE::dynamic_state_matrix_t	dynamic_state_matrix_t;
	typedef typename BASE::dynamic_input_matrix_t	dynamic_input_matrix_t;

	/**
	 * Default constructor.
	 *
	 * @param [in] dynamicLibraryIsCompiled: Whether a library is already complied.
	 */
	SystemDynamicsBaseAD(const bool& dynamicLibraryIsCompiled = false);

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
	 * @param [in] input: input vector.
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
	 * @param [in] input: input vector.
	 * @param [out] jumpedState: jumped state.
	 */
	template <typename SCALAR_T>
	void systemJumpMap(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
			Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& jumpedState);

	/**
	 * Interface method to the guard surfaces. This method should be implemented by the derived class.
	 * If there is no guard surfaces return a vector of size 1 with value -1 (or any other negative number).
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state.
	 * @param [in] input: input vector
	 * @param [out] guardSurfacesValue: A vector of guard surfaces values
	 */
	template <typename SCALAR_T>
	void systemGuardSurfaces(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
			Eigen::Matrix<SCALAR_T, NUM_MODES, 1>& guardSurfacesValue);

	/**
	 * Gets the number of the modes in the hybrid system.
	 * The minimum is one.
	 *
	 * @return Number of the modes in the hybrid system.
	 */
	size_t getNumModes() const;

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
	 * Loads the forward model, the Jacobian model, and the Hessian model.
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
	 * Interface method to the state flow map of the hybrid system.
	 *
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [in] input: input vector
	 * @param [out] stateDerivative: state vector time derivative.
	 */
	virtual void computeFlowMap(
			const scalar_t& time,
			const state_vector_t& state,
			const input_vector_t& input,
			state_vector_t& stateDerivative) override;

	/**
	 * Interface method to the state jump map of the hybrid system.
	 *
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [out] jumpedState: jumped state.
	 */
	virtual void computeJumpMap(
			const scalar_t& time,
			const state_vector_t& state,
			state_vector_t& jumpedState) override;

	/**
	 * Interface method to the guard surfaces.
	 *
	 * @param [in] time: transition time
	 * @param [in] state: transition state
	 * @param [out] guardSurfacesValue: A vector of guard surfaces values
	 */
	virtual void computeGuardSurfaces(
			const scalar_t& time,
			const state_vector_t& state,
			dynamic_vector_t& guardSurfacesValue) override;

    /**
     * Sets the current time, state, and control input.
     *
     * @param [in] timet: Current time
     * @param [in] state: Current state vector
     * @param [in] input: Current input vector
     */
	virtual void setCurrentStateAndControl(
			const scalar_t& time,
			const state_vector_t& state,
			const input_vector_t& input) override;

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
	virtual void getFlowMapDerivativeInput(state_input_matrix_t& B) override;

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
	virtual void getJumpMapDerivativeInput(state_input_matrix_t& H) override;

	/**
	 * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
	 *
	 * @param [out] D_t_gamma: Derivative of the guard surfaces w.r.t. time.
	 */
	virtual void getGuardSurfacesDerivativeTime(dynamic_vector_t& D_t_gamma) override;

	/**
	 * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
	 *
	 * @param [out] D_x_gamma: Derivative of the guard surfaces w.r.t. state vector.
	 */
	virtual void getGuardSurfacesDerivativeState(dynamic_state_matrix_t& D_x_gamma) override;

	/**
	 * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
	 *
	 * @param [out] D_u_gamma: Derivative of the guard surfaces w.r.t. input vector.
	 */
	virtual void getGuardSurfacesDerivativeInput(dynamic_input_matrix_t& D_u_gamma) override;


	void getCompileFlags(std::vector<std::string>& cFlags) {
		flowMapADInterfacePtr_->getCompileFlags(cFlags);
	}

	void setCompileFlags(const std::vector<std::string>& cFlags) {
		flowMapADInterfacePtr_->setCompileFlags(cFlags);
		jumpMapADInterfacePtr_->setCompileFlags(cFlags);
		guardSurfacesADInterfacePtr_->setCompileFlags(cFlags);
	}

	void addCompileFlag(const std::vector<std::string>& cFlags) {
		flowMapADInterfacePtr_->addCompileFlags(cFlags);
		jumpMapADInterfacePtr_->addCompileFlags(cFlags);
		guardSurfacesADInterfacePtr_->addCompileFlags(cFlags);
	}


protected:
	typedef CppAdCodeGenInterface<domain_dim_, state_dim_, scalar_t> map_ad_interface_t;
	typedef CppAdCodeGenInterface<domain_dim_, num_modes_, scalar_t> guard_ad_interface_t;

	typedef typename map_ad_interface_t::ad_scalar_t			ad_scalar_t;
	typedef typename map_ad_interface_t::ad_dynamic_vector_t	ad_dynamic_vector_t;
	typedef typename map_ad_interface_t::ad_funtion_t			ad_funtion_t;
	typedef typename map_ad_interface_t::domain_vector_t		ad_domain_vector_t;
	typedef typename map_ad_interface_t::domain_matrix_t		ad_domain_matrix_t;

	typedef typename map_ad_interface_t::range_vector_t 		map_ad_range_vector_t;
	typedef typename map_ad_interface_t::domain_range_matrix_t 	map_ad_domain_range_matrix_t;
	typedef typename map_ad_interface_t::range_domain_matrix_t 	map_ad_range_domain_matrix_t;
	typedef typename guard_ad_interface_t::range_vector_t 		 guard_ad_range_vector_t;
	typedef typename guard_ad_interface_t::domain_range_matrix_t guard_ad_domain_range_matrix_t;
	typedef typename guard_ad_interface_t::range_domain_matrix_t guard_ad_range_domain_matrix_t;

	/**
	 * The flow map specialized with AD type.
	 *
	 * @param [in] tapedInput: concatenated input vector
	 * @param [out] f: The flow map value
	 */
	void systemFlowMapAD(
			const ad_dynamic_vector_t& tapedInput,
			ad_dynamic_vector_t& f);

	/**
	 * The jump map specialized with AD type.
	 *
	 * @param [in] tapedInput: concatenated input vector
	 * @param [out] g: The jump map value
	 */
	void systemJumpMapAD(
			const ad_dynamic_vector_t& tapedInput,
			ad_dynamic_vector_t& g);

	/**
	 * The guard surfaces specialized with AD type.
	 *
	 * @param [in] tapedInput: concatenated input vector
	 * @param [out] gamma: The guard surfaces value
	 */
	void systemGuardSurfacesAD(
			const ad_dynamic_vector_t& tapedInput,
			ad_dynamic_vector_t& gamma);

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

	ad_funtion_t systemFlowMapAD_;
	ad_funtion_t systemJumpMapAD_;
	ad_funtion_t guardSurfacesJumpMapAD_;

	map_ad_range_domain_matrix_t flowSparsityPattern_;
	map_ad_range_domain_matrix_t jumpSparsityPattern_;
	guard_ad_range_domain_matrix_t guardSparsityPattern_;

	std::unique_ptr<map_ad_interface_t> flowMapADInterfacePtr_;
	std::unique_ptr<map_ad_interface_t> jumpMapADInterfacePtr_;
	std::unique_ptr<guard_ad_interface_t> guardSurfacesADInterfacePtr_;

	ad_domain_vector_t 		tapedInput_;

	map_ad_domain_range_matrix_t flowJacobian_;
	map_ad_domain_range_matrix_t jumpJacobian_;
	guard_ad_domain_range_matrix_t guardJacobian_;
};

} // namespace ocs2

#include "implementation/SystemDynamicsBaseAD.h"

#endif /* SYSTEMDYNAMICSBASEAD_OCS2_H_ */
