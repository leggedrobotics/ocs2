/*
 * SystemDynamicsBaseAD.h
 *
 *  Created on: Apr 27, 2018
 *      Author: farbod
 */

namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::SystemDynamicsBaseAD()
	: BASE()
	, dynamicLibraryIsCompiled_(false)
	, modelName_("")
	, libraryFolder_("")
{};

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
	if (rhs.dynamicLibraryIsCompiled_==true)
		loadModels(false);
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

	if (NUM_MODES != 1)
		throw std::runtime_error("systemGuardSurfaces() method should be implemented by the derived class.");

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

	if (dynamicLibraryIsCompiled_==true)
		loadModels(false);
	else
		std::cerr << "WARNING: The dynamicLibraryIsCompiled_ flag is false." << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
bool SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::isDynamicLibraryCompiled() const {

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

	ad_scalar_t& t = const_cast<ad_scalar_t&>(tapedInput(0));
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

	ad_scalar_t& t = const_cast<ad_scalar_t&>(tapedInput(0));
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

	ad_scalar_t& t = const_cast<ad_scalar_t&>(tapedInput(0));
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
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::createModels(
		bool verbose) {

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
	flowMapADInterfacePtr_->createModels(modelName_+"_flow_map", libraryFolder_, verbose);

	jumpMapADInterfacePtr_->computeForwardModel(true);
	jumpMapADInterfacePtr_->computeJacobianModel(true);
	jumpMapADInterfacePtr_->computeHessianModel(false);
	jumpMapADInterfacePtr_->createModels(modelName_+"_jump_map", libraryFolder_, verbose);

	guardSurfacesADInterfacePtr_->computeForwardModel(true);
	guardSurfacesADInterfacePtr_->computeJacobianModel(true);
	guardSurfacesADInterfacePtr_->computeHessianModel(false);
	guardSurfacesADInterfacePtr_->createModels(modelName_+"_guard_surfaces", libraryFolder_, verbose);

	dynamicLibraryIsCompiled_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t, size_t NUM_MODES>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t, NUM_MODES>::loadModels(
		bool verbose) {

	flowMapADInterfacePtr_->loadModels(modelName_+"_flow_map", libraryFolder_, verbose);

	jumpMapADInterfacePtr_->loadModels(modelName_+"_jump_map", libraryFolder_, verbose);

	guardSurfacesADInterfacePtr_->loadModels(modelName_+"_guard_surfaces", libraryFolder_, verbose);
}

} // namespace ocs2
