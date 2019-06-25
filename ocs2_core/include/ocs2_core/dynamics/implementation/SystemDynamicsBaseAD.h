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

namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::SystemDynamicsBaseAD(
		const bool& dynamicLibraryIsCompiled /*= false*/)
	: BASE()
	, dynamicLibraryIsCompiled_(dynamicLibraryIsCompiled)
	, modelName_("")
	, libraryFolder_("")
{
	if (dynamicLibraryIsCompiled) {
		setADInterfaces();
	}
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::SystemDynamicsBaseAD(
		const SystemDynamicsBaseAD& rhs)

	: BASE(rhs)
	, dynamicLibraryIsCompiled_(rhs.dynamicLibraryIsCompiled_)
	, modelName_(rhs.modelName_)
	, libraryFolder_(rhs.libraryFolder_)
	, flowMapADInterfacePtr_(rhs.flowMapADInterfacePtr_->clone())
	, jumpMapADInterfacePtr_(rhs.jumpMapADInterfacePtr_->clone())
	, guardSurfacesADInterfacePtr_(rhs.guardSurfacesADInterfacePtr_->clone())

{
	if (rhs.dynamicLibraryIsCompiled_) {
		loadModels(false);
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
typename SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::BASE*
	SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::clone() const {

		return new Derived(static_cast<Derived const&>(*this));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
template <typename SCALAR_T>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::systemFlowMap(
		const SCALAR_T& time,
		const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
		const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
		Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& stateDerivative) {

	throw std::runtime_error("systemFlowMap() method should be implemented by the derived class.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
template <typename SCALAR_T>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::systemJumpMap(
		const SCALAR_T& time,
		const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
		const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
		Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& jumpedState) {

	jumpedState = state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
template <typename SCALAR_T>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::systemGuardSurfaces(
		const SCALAR_T& time,
		const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
		const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
		Eigen::Matrix<SCALAR_T, NUM_MODES, 1>& guardSurfacesValue) {

	if (NUM_MODES != 1) {
		throw std::runtime_error("systemGuardSurfaces() method should be implemented by the derived class.");
	}

	guardSurfacesValue(0) = -1;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
size_t SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::getNumModes() const {

	return NUM_MODES;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::createModels(
		const std::string& modelName,
		const std::string& libraryFolder) {

	modelName_ = modelName;
	libraryFolder_ = libraryFolder;

	createModels(true);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::loadModels(
		const std::string& modelName,
		const std::string& libraryFolder) {

	modelName_ = modelName;
	libraryFolder_ = libraryFolder;

	if (dynamicLibraryIsCompiled_) {
		bool libraryLoaded = loadModels(false);
		if (!libraryLoaded) {
			throw std::runtime_error("SystemDynamics library is not found!");
	}

	} else {
		throw std::runtime_error("SystemDynamics library has not been compiled!");
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
const bool& SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::isDynamicLibraryCompiled() const {

	return dynamicLibraryIsCompiled_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
std::string SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::getModelName() const {

	return modelName_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::computeFlowMap(
		const scalar_t& time,
		const state_vector_t& state,
		const input_vector_t& input,
		state_vector_t& stateDerivative) {

	ad_domain_vector_t tapedInput;
	tapedInput << time, state, input;

	flowMapADInterfacePtr_->getFunctionValue(tapedInput, stateDerivative);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::computeJumpMap(
		const scalar_t& time,
		const state_vector_t& state,
		state_vector_t& jumpedState) {

	ad_domain_vector_t tapedInput;
	tapedInput << time, state, input_vector_t::Zero();

	jumpMapADInterfacePtr_->getFunctionValue(tapedInput, jumpedState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::computeGuardSurfaces(
		const scalar_t& time,
		const state_vector_t& state,
		dynamic_vector_t& guardSurfacesValue) {

	ad_domain_vector_t tapedInput;
	tapedInput << time, state, input_vector_t::Zero();

	guard_ad_range_vector_t guardSurfacesValueFixed;
	guardSurfacesADInterfacePtr_->getFunctionValue(tapedInput, guardSurfacesValueFixed);
	guardSurfacesValue = guardSurfacesValueFixed;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::setCurrentStateAndControl(
		const scalar_t& time,
		const state_vector_t& state,
		const input_vector_t& input) {

	BASE::setCurrentStateAndControl(time, state, input);

	tapedInput_ << time, state, input;

	flowMapADInterfacePtr_->getJacobian(tapedInput_, flowJacobian_);

	jumpMapADInterfacePtr_->getJacobian(tapedInput_, jumpJacobian_);

	guardSurfacesADInterfacePtr_->getJacobian(tapedInput_, guardJacobian_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::getFlowMapDerivativeTime(
		state_vector_t& df) {

	df = flowJacobian_.template block<1, state_dim_>(0, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::getFlowMapDerivativeState(
		state_matrix_t& A) {

	A = flowJacobian_.template block<state_dim_, state_dim_>(1, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::getFlowMapDerivativeInput(
		state_input_matrix_t& B) {

	B = flowJacobian_.template block<input_dim_, state_dim_>(1 + state_dim_, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::getJumpMapDerivativeTime(
		state_vector_t& dg) {

	dg = jumpJacobian_.template block<1, state_dim_>(0, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::getJumpMapDerivativeState(
		state_matrix_t& G) {

	G = jumpJacobian_.template block<state_dim_, state_dim_>(1, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::getJumpMapDerivativeInput(
		state_input_matrix_t& H) {

	H = jumpJacobian_.template block<input_dim_, state_dim_>(1 + state_dim_, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::getGuardSurfacesDerivativeTime(
		dynamic_vector_t& D_t_gamma) {

	D_t_gamma = guardJacobian_.template block<1, num_modes_>(0, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::getGuardSurfacesDerivativeState(
		dynamic_state_matrix_t& D_x_gamma) {

	D_x_gamma = guardJacobian_.template block<state_dim_, num_modes_>(1, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::getGuardSurfacesDerivativeInput(
		dynamic_input_matrix_t& D_u_gamma) {

	D_u_gamma = guardJacobian_.template block<input_dim_, num_modes_>(1 + state_dim_, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::systemFlowMapAD(
		const ad_dynamic_vector_t& tapedInput,
		ad_dynamic_vector_t& f) {

	auto& t = const_cast<ad_scalar_t&>(tapedInput(0));
	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> x = tapedInput.segment(1, STATE_DIM);
	Eigen::Matrix<ad_scalar_t, INPUT_DIM, 1> u = tapedInput.segment(1+STATE_DIM, INPUT_DIM);

	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> fFixed;
	static_cast<Derived *>(this)->template systemFlowMap<ad_scalar_t>(t, x, u, fFixed);
	f = fFixed;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::systemJumpMapAD(
		const ad_dynamic_vector_t& tapedInput,
		ad_dynamic_vector_t& g) {

	auto& t = const_cast<ad_scalar_t&>(tapedInput(0));
	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> x = tapedInput.segment(1, STATE_DIM);
	Eigen::Matrix<ad_scalar_t, INPUT_DIM, 1> u = tapedInput.segment(1+STATE_DIM, INPUT_DIM);

	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> gFixed;
	static_cast<Derived *>(this)->template systemJumpMap<ad_scalar_t>(t, x, u, gFixed);
	g = gFixed;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::systemGuardSurfacesAD(
		const ad_dynamic_vector_t& tapedInput,
		ad_dynamic_vector_t& gamma) {

	auto& t = const_cast<ad_scalar_t&>(tapedInput(0));
	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> x = tapedInput.segment(1, STATE_DIM);
	Eigen::Matrix<ad_scalar_t, INPUT_DIM, 1> u = tapedInput.segment(1+STATE_DIM, INPUT_DIM);

	Eigen::Matrix<ad_scalar_t, NUM_MODES, 1> gammaFixed;
	static_cast<Derived *>(this)->template systemGuardSurfaces<ad_scalar_t>(t, x, u, gammaFixed);
	gamma = gammaFixed;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::setADInterfaces() {

	systemFlowMapAD_ = [this](
			const ad_dynamic_vector_t& x,
			ad_dynamic_vector_t& y) {
		this->systemFlowMapAD(x, y);
	};

	systemJumpMapAD_ = [this](
			const ad_dynamic_vector_t& x,
			ad_dynamic_vector_t& y) {
		this->systemJumpMapAD(x, y);
	};

	guardSurfacesJumpMapAD_ = [this](
			const ad_dynamic_vector_t& x,
			ad_dynamic_vector_t& y) {
		this->systemGuardSurfacesAD(x, y);
	};

	flowSparsityPattern_.setOnes();

	jumpSparsityPattern_.setOnes();

	guardSparsityPattern_.setOnes();

	flowMapADInterfacePtr_.reset( new map_ad_interface_t(
			systemFlowMapAD_, flowSparsityPattern_) );

	jumpMapADInterfacePtr_.reset( new map_ad_interface_t(
			systemJumpMapAD_, jumpSparsityPattern_) );

	guardSurfacesADInterfacePtr_.reset( new guard_ad_interface_t(
			guardSurfacesJumpMapAD_, guardSparsityPattern_) );

	flowMapADInterfacePtr_->computeForwardModel(true);
	flowMapADInterfacePtr_->computeJacobianModel(true);
	flowMapADInterfacePtr_->computeHessianModel(false);

	jumpMapADInterfacePtr_->computeForwardModel(true);
	jumpMapADInterfacePtr_->computeJacobianModel(true);
	jumpMapADInterfacePtr_->computeHessianModel(false);

	guardSurfacesADInterfacePtr_->computeForwardModel(true);
	guardSurfacesADInterfacePtr_->computeJacobianModel(true);
	guardSurfacesADInterfacePtr_->computeHessianModel(false);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::createModels(
		bool verbose) {

	// sets all the required CppAdCodeGenInterfaces
	setADInterfaces();

	flowMapADInterfacePtr_->createModels(modelName_+"_flow_map", libraryFolder_, verbose);
	jumpMapADInterfacePtr_->createModels(modelName_+"_jump_map", libraryFolder_, verbose);
	guardSurfacesADInterfacePtr_->createModels(modelName_+"_guard_surfaces", libraryFolder_, verbose);

	dynamicLibraryIsCompiled_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
bool SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::loadModels(
		bool verbose) {

	bool flowMapLoaded = flowMapADInterfacePtr_->loadModels(modelName_+"_flow_map", libraryFolder_, verbose);

	bool jumpMapLoaded = jumpMapADInterfacePtr_->loadModels(modelName_+"_jump_map", libraryFolder_, verbose);

	bool guardSurfacesLoaded = guardSurfacesADInterfacePtr_->loadModels(modelName_+"_guard_surfaces", libraryFolder_, verbose);

	return (flowMapLoaded && jumpMapLoaded && guardSurfacesLoaded);
}

} // namespace ocs2
