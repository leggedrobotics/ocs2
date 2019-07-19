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
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::ConstraintBaseAD(
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
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::ConstraintBaseAD(
		const ConstraintBaseAD& rhs)

	: BASE(rhs)
	, dynamicLibraryIsCompiled_(rhs.dynamicLibraryIsCompiled_)
	, modelName_(rhs.modelName_)
	, libraryFolder_(rhs.libraryFolder_)
	, stateInputADInterfacePtr_(rhs.stateInputADInterfacePtr_->clone())
	, stateOnlyADInterfacePtr_(rhs.stateOnlyADInterfacePtr_->clone())
	, stateOnlyFinalADInterfacePtr_(rhs.stateOnlyFinalADInterfacePtr_->clone())

{
	if (static_cast<bool>(rhs.dynamicLibraryIsCompiled_)) {
		loadModels(false);
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
typename ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::BASE*
	ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::clone() const {

		return new Derived(static_cast<Derived const&>(*this));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::createModels(
		const std::string& modelName,
		const std::string& libraryFolder) {

	modelName_ = modelName;
	libraryFolder_ = libraryFolder;

	createModels(true);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::loadModels(
		const std::string& modelName,
		const std::string& libraryFolder) {

	modelName_ = modelName;
	libraryFolder_ = libraryFolder;

	if (dynamicLibraryIsCompiled_) {
		bool libraryLoaded = loadModels(false);
		if (!libraryLoaded) {
			throw std::runtime_error("Constraint library is not found!");
		}

	} else {
		throw std::runtime_error("Constraint library has not been compiled!");
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
const bool& ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::isDynamicLibraryCompiled() const {

	return dynamicLibraryIsCompiled_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
std::string ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::getModelName() const {

	return modelName_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::setCurrentStateAndControl(
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
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::getConstraint1(
		constraint1_vector_t& e) {

	stateInputADInterfacePtr_->getFunctionValue(tapedInput_, e);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::getConstraint2(
		constraint2_vector_t& h) {

	stateOnlyADInterfacePtr_->getFunctionValue(tapedInput_, h);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::getFinalConstraint2(
		constraint2_vector_t& h_f) {

	stateOnlyFinalADInterfacePtr_->getFunctionValue(tapedInput_, h_f);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::getConstraint1DerivativesState(
		constraint1_state_matrix_t& C) {

	C = stateInputJacobian_.template block<state_dim_, MAX_CONSTRAINT_DIM_>(1, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::getConstraint1DerivativesControl(
		constraint1_input_matrix_t& D) {

	D = stateInputJacobian_.template block<input_dim_, MAX_CONSTRAINT_DIM_>(1 + state_dim_, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::getConstraint2DerivativesState(
		constraint2_state_matrix_t& F) {

	F = stateOnlyJacobian_.template block<state_dim_, MAX_CONSTRAINT_DIM_>(1, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::getFinalConstraint2DerivativesState(
		constraint2_state_matrix_t& F_f) {

	F_f = stateOnlyFinalJacobian_.template block<state_dim_, MAX_CONSTRAINT_DIM_>(1, 0).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::stateInputConstraintAD(
		const ad_dynamic_vector_t& tapedInput,
		ad_dynamic_vector_t& g1) {

	auto& t = const_cast<ad_scalar_t&>(tapedInput(0));
	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> x = tapedInput.segment(1, STATE_DIM);
	Eigen::Matrix<ad_scalar_t, INPUT_DIM, 1> u = tapedInput.segment(1+STATE_DIM, INPUT_DIM);

	g1.resize(MAX_CONSTRAINT_DIM_);
	static_cast<Derived *>(this)->template stateInputConstraint<ad_scalar_t>(t, x, u, g1);

	if (g1.size()>MAX_CONSTRAINT_DIM_) {
		throw std::runtime_error("The max number of constraints is exceeded!");
	}

	g1.conservativeResize(MAX_CONSTRAINT_DIM_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::stateOnlyConstraintAD(
		const ad_dynamic_vector_t& tapedInput,
		ad_dynamic_vector_t& g2) {

	auto& t = const_cast<ad_scalar_t&>(tapedInput(0));
	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> x = tapedInput.segment(1, STATE_DIM);

	g2.resize(MAX_CONSTRAINT_DIM_);
	static_cast<Derived *>(this)->template stateOnlyConstraint<ad_scalar_t>(t, x, g2);

	if (g2.size()>MAX_CONSTRAINT_DIM_) {
		throw std::runtime_error("The max number of constraints is exceeded!");
	}

	g2.conservativeResize(MAX_CONSTRAINT_DIM_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::stateOnlyConstraintFinalAD(
		const ad_dynamic_vector_t& tapedInput,
		ad_dynamic_vector_t& g2Final) {

	auto& t = const_cast<ad_scalar_t&>(tapedInput(0));
	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> x = tapedInput.segment(1, STATE_DIM);

	g2Final.resize(MAX_CONSTRAINT_DIM_);
	static_cast<Derived *>(this)->template stateOnlyFinalConstraint<ad_scalar_t>(t, x, g2Final);

	if (g2Final.size()>MAX_CONSTRAINT_DIM_) {
		throw std::runtime_error("The max number of constraints is exceeded!");
	}

	g2Final.conservativeResize(MAX_CONSTRAINT_DIM_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::setADInterfaces() {

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

	stateOnlyADInterfacePtr_->computeForwardModel(true);
	stateOnlyADInterfacePtr_->computeJacobianModel(true);
	stateOnlyADInterfacePtr_->computeHessianModel(false);

	stateOnlyFinalADInterfacePtr_->computeForwardModel(true);
	stateOnlyFinalADInterfacePtr_->computeJacobianModel(true);
	stateOnlyFinalADInterfacePtr_->computeHessianModel(false);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::createModels(
		bool verbose) {

	// sets all the required CppAdCodeGenInterfaces
	setADInterfaces();

	stateInputADInterfacePtr_->createModels(modelName_+"_stateInput", libraryFolder_, verbose);
	stateOnlyADInterfacePtr_->createModels(modelName_+"_stateOnly", libraryFolder_, verbose);
	stateOnlyFinalADInterfacePtr_->createModels(modelName_+"_stateOnlyFinal", libraryFolder_, verbose);

	dynamicLibraryIsCompiled_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
bool ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::loadModels(
		bool verbose) {

	bool stateInputLoaded = stateInputADInterfacePtr_->loadModels(modelName_+"_stateInput", libraryFolder_, verbose);

	bool stateOnlyLoaded = stateOnlyADInterfacePtr_->loadModels(modelName_+"_stateOnly", libraryFolder_, verbose);

	bool stateOnlyFinalLoaded = stateOnlyFinalADInterfacePtr_->loadModels(modelName_+"_stateOnlyFinal", libraryFolder_, verbose);

	return (stateInputLoaded && stateOnlyLoaded && stateOnlyFinalLoaded);
}

} // namespace ocs2

