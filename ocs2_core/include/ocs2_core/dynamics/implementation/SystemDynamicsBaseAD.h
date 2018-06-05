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
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::SystemDynamicsBaseAD()
	: BASE()
	, dynamicLibraryIsCompiled_(false)
	, modelName_("")
	, libraryFolder_("")
{};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::SystemDynamicsBaseAD(
		const SystemDynamicsBaseAD& rhs)

	: BASE(rhs)
	, dynamicLibraryIsCompiled_(rhs.dynamicLibraryIsCompiled_)
	, modelName_(rhs.modelName_)
	, libraryFolder_(rhs.libraryFolder_)
	, flowMapADInterfacePtr_(rhs.flowMapADInterfacePtr_->clone())
	, jumpMapADInterfacePtr_(rhs.jumpMapADInterfacePtr_->clone())

{
	if (rhs.dynamicLibraryIsCompiled_==true)
		loadModels(false);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
typename SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::BASE*
	SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::clone() const {

		return new Derived(static_cast<Derived const&>(*this));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
template <typename SCALAR_T>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::systemFlowMap(
		const SCALAR_T& time,
		const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
		const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
		Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& stateDerivative) {

	static_cast<Derived *>(this)->systemFlowMap<SCALAR_T>(time, state, input, stateDerivative);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
template <typename SCALAR_T>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::systemJumpMap(
		const SCALAR_T& time,
		const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
		const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
		Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& jumpedState) {

	static_cast<Derived *>(this)->systemJumpMap<SCALAR_T>(time, state, input, jumpedState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::createModels(
		const std::string& modelName,
		const std::string& libraryFolder) {

	modelName_ = modelName;
	libraryFolder_ = libraryFolder;

	createModels(true);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::loadModels(
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
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
bool SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::isDynamicLibraryCompiled() const {

	return dynamicLibraryIsCompiled_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
std::string SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getModelName() const {

	return modelName_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::setCurrentStateAndControl(
		const scalar_t& t,
		const state_vector_t& x,
		const input_vector_t& u) {

	BASE::setCurrentStateAndControl(t, x, u);

	tapedInput_ << t, x, u;

	flowMapADInterfacePtr_->getJacobian(tapedInput_, flowJacobian_);

	jumpMapADInterfacePtr_->getJacobian(tapedInput_, jumpJacobian_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getFlowMap(
		state_vector_t& f) {

	flowMapADInterfacePtr_->getFunctionValue(tapedInput_, f);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getJumpMap(
		state_vector_t& g) {

	jumpMapADInterfacePtr_->getFunctionValue(tapedInput_, g);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getFlowMapDerivativeTime(
		state_vector_t& df) {

	df = flowJacobian_.template block<1, state_dim_>(0, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getFlowMapDerivativeState(
		state_matrix_t& A) {

	A = flowJacobian_.template block<state_dim_, state_dim_>(1, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getFlowMapDerivativeInput(
		state_input_matrix_t& B) {

	B = flowJacobian_.template block<input_dim_, state_dim_>(1 + state_dim_, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getJumpMapDerivativeTime(
		state_vector_t& dg) {

	dg = jumpJacobian_.template block<1, state_dim_>(0, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getJumpMapDerivativeState(
		state_matrix_t& G) {

	G = jumpJacobian_.template block<state_dim_, state_dim_>(1, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getJumpMapDerivativeInput(
		state_input_matrix_t& H) {

	H = jumpJacobian_.template block<input_dim_, state_dim_>(1 + state_dim_, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::systemFlowMapAD(
		const ad_dynamic_vector_t& tapedInput,
		ad_dynamic_vector_t& f) {

	ad_scalar_t& t = const_cast<ad_scalar_t&>(tapedInput(0));
	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> x = tapedInput.segment(1, STATE_DIM);
	Eigen::Matrix<ad_scalar_t, INPUT_DIM, 1> u = tapedInput.segment(1+STATE_DIM, INPUT_DIM);

	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> fFixed;
	systemFlowMap<ad_scalar_t>(t, x, u, fFixed);
	f = fFixed;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::systemJumpMapAD(
		const ad_dynamic_vector_t& tapedInput,
		ad_dynamic_vector_t& g) {

	ad_scalar_t& t = const_cast<ad_scalar_t&>(tapedInput(0));
	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> x = tapedInput.segment(1, STATE_DIM);
	Eigen::Matrix<ad_scalar_t, INPUT_DIM, 1> u = tapedInput.segment(1+STATE_DIM, INPUT_DIM);

	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> gFixed;
	systemJumpMap<ad_scalar_t>(t, x, u, gFixed);
	g = gFixed;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::createModels(
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

	flowSparsityPattern_.setOnes();

	jumpSparsityPattern_.setOnes();

	flowMapADInterfacePtr_.reset( new ad_interface_t(
			systemFlowMapAD_, flowSparsityPattern_) );

	jumpMapADInterfacePtr_.reset( new ad_interface_t(
			systemJumpMapAD_, jumpSparsityPattern_) );

	flowMapADInterfacePtr_->computeForwardModel(true);
	flowMapADInterfacePtr_->computeJacobianModel(true);
	flowMapADInterfacePtr_->computeHessianModel(false);
	flowMapADInterfacePtr_->createModels(modelName_+"_flow_map", libraryFolder_, verbose);

	jumpMapADInterfacePtr_->computeForwardModel(true);
	jumpMapADInterfacePtr_->computeJacobianModel(true);
	jumpMapADInterfacePtr_->computeHessianModel(false);
	jumpMapADInterfacePtr_->createModels(modelName_+"_jump_map", libraryFolder_, verbose);

	dynamicLibraryIsCompiled_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void SystemDynamicsBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::loadModels(
		bool verbose) {

	flowMapADInterfacePtr_->loadModels(modelName_+"_flow_map", libraryFolder_, verbose);

	jumpMapADInterfacePtr_->loadModels(modelName_+"_jump_map", libraryFolder_, verbose);
}

} // namespace ocs2
