/*
 * ConstraintBaseAD.h
 *
 *  Created on: May 3, 2018
 *      Author: farbod
 */
namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::ConstraintBaseAD()
	: BASE()
	, dynamicLibraryIsCompiled_(false)
	, modelName_("")
	, libraryFolder_("")
{};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::ConstraintBaseAD(const ConstraintBaseAD& rhs)

	: BASE(rhs)
	, dynamicLibraryIsCompiled_(rhs.dynamicLibraryIsCompiled_)
	, modelName_(rhs.modelName_)
	, libraryFolder_(rhs.libraryFolder_)
	, stateInputADInterfacePtr_(rhs.stateInputADInterfacePtr_->clone())
	, stateOnlyADInterfacePtr_(rhs.stateOnlyADInterfacePtr_->clone())
	, stateOnlyFinalADInterfacePtr_(rhs.stateOnlyFinalADInterfacePtr_->clone())

{
	if (rhs.dynamicLibraryIsCompiled_==true)
		loadModels(false);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
typename ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::BASE*
	ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::clone() const {

		return new Derived(static_cast<Derived const&>(*this));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
template <typename SCALAR_T>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::stateInputConstraint(
		const SCALAR_T& time,
		const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
		const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
		Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& constraintVector) {

	static_cast<Derived *>(this)->stateInputConstraint<SCALAR_T>(time, state, input, constraintVector);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
template <typename SCALAR_T>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::stateOnlyConstraint(
		const SCALAR_T& time,
		const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
		Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& constraintVector) {

	static_cast<Derived *>(this)->stateOnlyConstraint<SCALAR_T>(time, state, constraintVector);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
template <typename SCALAR_T>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::stateOnlyFinalConstraint(
		const SCALAR_T& time,
		const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
		Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& constraintVector) {

	static_cast<Derived *>(this)->stateOnlyFinalConstraint<SCALAR_T>(time, state, constraintVector);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::createModels(
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
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::loadModels(
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
bool ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::isDynamicLibraryCompiled() const {

	return dynamicLibraryIsCompiled_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
std::string ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getModelName() const {

	return modelName_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::setCurrentStateAndControl(
		const scalar_t& t,
		const state_vector_t& x,
		const input_vector_t& u) {

	BASE::setCurrentStateAndControl(t, x, u);

	tapedInput_ << t, x, u;

	stateInputADInterfacePtr_->getJacobian(tapedInput_, stateInputJacobian_);
	stateOnlyADInterfacePtr_->getJacobian(tapedInput_, stateOnlyJacobian_);
	stateOnlyFinalADInterfacePtr_->getJacobian(tapedInput_, stateOnlyFinalJacobian_);

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getConstraint1(
		constraint1_vector_t& e) {

	stateInputADInterfacePtr_->getFunctionValue(tapedInput_, e);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getConstraint2(
		constraint2_vector_t& h) {

	stateOnlyADInterfacePtr_->getFunctionValue(tapedInput_, h);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getFinalConstraint2(
		constraint2_vector_t& h_f) {

	stateOnlyFinalADInterfacePtr_->getFunctionValue(tapedInput_, h_f);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getConstraint1DerivativesState(
		constraint1_state_matrix_t& C) {

	C = stateInputJacobian_.template block<state_dim_, MAX_CONSTRAINT_DIM_>(1, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getConstraint1DerivativesControl(
		constraint1_input_matrix_t& D) {

	D = stateInputJacobian_.template block<input_dim_, MAX_CONSTRAINT_DIM_>(1 + state_dim_, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getConstraint2DerivativesState(
		constraint2_state_matrix_t& F) {

	F = stateOnlyJacobian_.template block<state_dim_, MAX_CONSTRAINT_DIM_>(1, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::getFinalConstraint2DerivativesState(
		constraint2_state_matrix_t& F_f) {

	F_f = stateOnlyFinalJacobian_.template block<state_dim_, MAX_CONSTRAINT_DIM_>(1, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::stateInputConstraintAD(
		const ad_dynamic_vector_t& tapedInput,
		ad_dynamic_vector_t& g1) {

	ad_scalar_t& t = const_cast<ad_scalar_t&>(tapedInput(0));
	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> x = tapedInput.segment(1, STATE_DIM);
	Eigen::Matrix<ad_scalar_t, INPUT_DIM, 1> u = tapedInput.segment(1+STATE_DIM, INPUT_DIM);

	g1.resize(MAX_CONSTRAINT_DIM_);
	stateInputConstraint<ad_scalar_t>(t, x, u, g1);

	if (g1.size()>MAX_CONSTRAINT_DIM_)
		throw std::runtime_error("The max number of constraints is exceeded!");

	g1.conservativeResize(MAX_CONSTRAINT_DIM_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::stateOnlyConstraintAD(
		const ad_dynamic_vector_t& tapedInput,
		ad_dynamic_vector_t& g2) {

	ad_scalar_t& t = const_cast<ad_scalar_t&>(tapedInput(0));
	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> x = tapedInput.segment(1, STATE_DIM);

	g2.resize(MAX_CONSTRAINT_DIM_);
	stateOnlyConstraint<ad_scalar_t>(t, x, g2);

	if (g2.size()>MAX_CONSTRAINT_DIM_)
		throw std::runtime_error("The max number of constraints is exceeded!");

	g2.conservativeResize(MAX_CONSTRAINT_DIM_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::stateOnlyConstraintFinalAD(
		const ad_dynamic_vector_t& tapedInput,
		ad_dynamic_vector_t& g2Final) {

	ad_scalar_t& t = const_cast<ad_scalar_t&>(tapedInput(0));
	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> x = tapedInput.segment(1, STATE_DIM);

	g2Final.resize(MAX_CONSTRAINT_DIM_);
	stateOnlyFinalConstraint<ad_scalar_t>(t, x, g2Final);

	if (g2Final.size()>MAX_CONSTRAINT_DIM_)
		throw std::runtime_error("The max number of constraints is exceeded!");

	g2Final.conservativeResize(MAX_CONSTRAINT_DIM_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::createModels(
		bool verbose) {

	stateInputConstraintAD_ = [this](
			const ad_dynamic_vector_t& x,
			ad_dynamic_vector_t& y) {
		this->stateInputConstraintAD(x, y);
	};

	stateOnlyConstraintAD_ = [this](
			const ad_dynamic_vector_t& x,
			ad_dynamic_vector_t& y) {
		this->stateOnlyConstraintAD(x, y);
	};

	stateOnlyConstraintFinalAD_ = [this](
			const ad_dynamic_vector_t& x,
			ad_dynamic_vector_t& y) {
		this->stateOnlyConstraintFinalAD(x, y);
	};

	stateInputSparsityPattern_.setOnes();
	stateInputSparsityPattern_.col(0).setZero();

	stateOnlySparsityPattern_.setOnes();
	stateOnlySparsityPattern_.col(0).setZero();
	stateOnlySparsityPattern_.template rightCols<input_dim_>().setZero();

	stateOnlyFinalSparsityPattern_.setOnes();
	stateOnlyFinalSparsityPattern_.col(0).setZero();
	stateOnlyFinalSparsityPattern_.template rightCols<input_dim_>().setZero();

	stateInputADInterfacePtr_.reset( new ad_interface_t(
			stateInputConstraintAD_, stateInputSparsityPattern_) );

	stateOnlyADInterfacePtr_.reset( new ad_interface_t(
			stateOnlyConstraintAD_, stateOnlySparsityPattern_) );

	stateOnlyFinalADInterfacePtr_.reset( new ad_interface_t(
			stateOnlyConstraintFinalAD_, stateOnlyFinalSparsityPattern_) );

	stateInputADInterfacePtr_->computeForwardModel(true);
	stateInputADInterfacePtr_->computeJacobianModel(true);
	stateInputADInterfacePtr_->computeHessianModel(false);
	stateInputADInterfacePtr_->createModels(modelName_+"_stateInput", libraryFolder_, verbose);

	stateOnlyADInterfacePtr_->computeForwardModel(true);
	stateOnlyADInterfacePtr_->computeJacobianModel(true);
	stateOnlyADInterfacePtr_->computeHessianModel(false);
	stateOnlyADInterfacePtr_->createModels(modelName_+"_stateOnly", libraryFolder_, verbose);

	stateOnlyFinalADInterfacePtr_->computeForwardModel(true);
	stateOnlyFinalADInterfacePtr_->computeJacobianModel(true);
	stateOnlyFinalADInterfacePtr_->computeHessianModel(false);
	stateOnlyFinalADInterfacePtr_->createModels(modelName_+"_stateOnlyFinal", libraryFolder_, verbose);

	dynamicLibraryIsCompiled_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM, logic_rules_template_t>::loadModels(
		bool verbose) {

	stateInputADInterfacePtr_->loadModels(modelName_+"_stateInput", libraryFolder_, verbose);

	stateOnlyADInterfacePtr_->loadModels(modelName_+"_stateOnly", libraryFolder_, verbose);

	stateOnlyFinalADInterfacePtr_->loadModels(modelName_+"_stateOnlyFinal", libraryFolder_, verbose);
}

} // namespace ocs2

