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
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::CostFunctionBaseAD(
		const bool& dynamicLibraryIsCompiled /*= false*/)
	: BASE()
	, dynamicLibraryIsCompiled_(dynamicLibraryIsCompiled)
	, modelName_("")
	, libraryFolder_("")
{
	if (dynamicLibraryIsCompiled) {
		setADInterfaces();
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::CostFunctionBaseAD(
		const CostFunctionBaseAD& rhs)

	: BASE(rhs)
	, dynamicLibraryIsCompiled_(rhs.dynamicLibraryIsCompiled_)
	, modelName_(rhs.modelName_)
	, libraryFolder_(rhs.libraryFolder_)
	, intermediateADInterfacePtr_(rhs.intermediateADInterfacePtr_->clone())
	, terminalADInterfacePtr_(rhs.terminalADInterfacePtr_->clone())

{
	if (static_cast<bool>(rhs.dynamicLibraryIsCompiled_)) {
		setADInterfaces();
		loadModels(false);
	} else {
		std::cerr << "WARNING: DMPC cost copy constructor called without any compiled library" << std::endl;
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
typename CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::BASE*
	CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::clone() const {

		return new Derived(static_cast<Derived const&>(*this));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
template <typename SCALAR_T>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::intermediateCostFunction(
		const SCALAR_T& time,
		const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
		const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
		const Eigen::Matrix<SCALAR_T, STATE_DESIRED_DIM, 1>& stateDesired,
		const Eigen::Matrix<SCALAR_T, INPUT_DESIRED_DIM, 1>& inputDesired,
		const Eigen::Matrix<SCALAR_T, LOGIC_VARIABLE_DIM, 1>& logicVariable,
		SCALAR_T& costValue) {

	throw std::runtime_error("intermediateCostFunction() method should be implemented by the derived class.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
template <typename SCALAR_T>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::terminalCostFunction(
		const SCALAR_T& time,
		const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
		const Eigen::Matrix<SCALAR_T, STATE_DESIRED_DIM, 1>& stateDesired,
		const Eigen::Matrix<SCALAR_T, LOGIC_VARIABLE_DIM, 1>& logicVariable,
		SCALAR_T& costValue) {

	throw std::runtime_error("terminalCostFunction() method should be implemented by the derived class.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
typename CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::state_desired_vector_t
CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::getDesiredState(
		const scalar_t& t) {

	throw std::runtime_error("getDesiredState() method should be implemented by the derived class.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
typename CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::input_desired_vector_t
CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::getDesiredInput(
		const scalar_t& t) {

	throw std::runtime_error("getDesiredInput() method should be implemented by the derived class.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
typename CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::logic_variable_t
CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::getlogicVariables(
		LogicRulesMachine<LOGIC_RULES_T>& logicRulesMachine,
		const size_t& partitionIndex,
		const char* algorithmName /*= nullptr*/) {

	throw std::runtime_error("getlogicVariables() method should be implemented by the derived class.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::createModels(
		const std::string& modelName,
		const std::string& libraryFolder) {

	modelName_ = modelName;
	libraryFolder_ = libraryFolder;

	createModels(true);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::loadModels(
		const std::string& modelName,
		const std::string& libraryFolder) {

	modelName_ = modelName;
	libraryFolder_ = libraryFolder;

	if (dynamicLibraryIsCompiled_) {
		bool libraryLoaded = loadModels(false);
		if (!libraryLoaded) {
			throw std::runtime_error("CostFunction library is not found!");
		}

	} else {
		throw std::runtime_error("CostFunction library has not been compiled!");
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
const bool& CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::isDynamicLibraryCompiled() const {

	return dynamicLibraryIsCompiled_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
std::string CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::getModelName() const {

	return modelName_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::setCurrentStateAndControl(
		const scalar_t& t,
		const state_vector_t& x,
		const input_vector_t& u) {

	BASE::setCurrentStateAndControl(t, x, u);

	timeVariable(tapedInput_)   = t;
	stateVariables(tapedInput_) = x;
	inputVariables(tapedInput_) = u;
	desiredStateVariables(tapedInput_) = static_cast<Derived *>(this)->template getDesiredState(t);
	desiredInputVariables(tapedInput_) = static_cast<Derived *>(this)->template getDesiredInput(t);

	intermediateADInterfacePtr_->getJacobian(tapedInput_, intermediateJacobian_);
	intermediateADInterfacePtr_->getHessian(tapedInput_, intermediateHessian_, 0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::initializeModel(
		LogicRulesMachine<LOGIC_RULES_T>& logicRulesMachine,
		const size_t& partitionIndex,
		const char* algorithmName /*= nullptr*/) {

	if (std::is_same<LOGIC_RULES_T, NullLogicRules>::value) {
		if (LOGIC_VARIABLE_DIM != 0) {
			throw std::runtime_error("LOGIC_VARIABLE_DIM should be zero!");
		}
	} else {
		logicVariables(tapedInput_) = static_cast<Derived *>(this)->template getlogicVariables(
				logicRulesMachine, partitionIndex, algorithmName);;
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
template <typename Derived_Matrix>
typename Derived_Matrix::Scalar&
CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::timeVariable(
		Eigen::MatrixBase<Derived_Matrix>& tapedInput) {

	return tapedInput(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
template <typename Derived_Matrix>
Eigen::Block<Derived_Matrix, STATE_DIM, 1>
CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::stateVariables(
		Eigen::MatrixBase<Derived_Matrix>& tapedInput) {

	return tapedInput.template segment<STATE_DIM>(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
template <typename Derived_Matrix>
Eigen::Block<Derived_Matrix, INPUT_DIM, 1>
CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::inputVariables(
		Eigen::MatrixBase<Derived_Matrix>& tapedInput) {

	return tapedInput.template segment<INPUT_DIM>(1+STATE_DIM);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
template <typename Derived_Matrix>
Eigen::Block<Derived_Matrix, STATE_DESIRED_DIM, 1>
CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::desiredStateVariables(
		Eigen::MatrixBase<Derived_Matrix>& tapedInput) {

	return tapedInput.template segment<STATE_DESIRED_DIM>(1+STATE_DIM+INPUT_DIM);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
template <typename Derived_Matrix>
Eigen::Block<Derived_Matrix, INPUT_DESIRED_DIM, 1>
CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::desiredInputVariables(
		Eigen::MatrixBase<Derived_Matrix>& tapedInput) {

	return tapedInput.template segment<INPUT_DESIRED_DIM>(1+STATE_DIM+INPUT_DIM+STATE_DESIRED_DIM);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
template <typename Derived_Matrix>
Eigen::Block<Derived_Matrix, LOGIC_VARIABLE_DIM, 1>
CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::logicVariables(
		Eigen::MatrixBase<Derived_Matrix>& tapedInput) {

	return tapedInput.template tail<logic_variable_dim_>();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::getIntermediateCost(
		scalar_t& L) {

	Eigen::Matrix<scalar_t, 1, 1> costValue;
	intermediateADInterfacePtr_->getFunctionValue(tapedInput_, costValue);
	L = costValue(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::getIntermediateCostDerivativeTime(
		scalar_t& dLdt) {

	dLdt = intermediateJacobian_(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::getIntermediateCostDerivativeState(
		state_vector_t& dLdx) {

	dLdx = intermediateJacobian_.template segment<state_dim_>(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::getIntermediateCostSecondDerivativeState(
		state_matrix_t& dLdxx) {

	dLdxx = intermediateHessian_.template block<state_dim_, state_dim_>(1, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::getIntermediateCostDerivativeInput(
		input_vector_t& dLdu) {

	dLdu = intermediateJacobian_.template segment<input_dim_>(1+state_dim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::getIntermediateCostSecondDerivativeInput(
		input_matrix_t& dLduu) {

	dLduu = intermediateHessian_.template block<input_dim_, input_dim_>(1+state_dim_, 1+state_dim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::getIntermediateCostDerivativeInputState(
		input_state_matrix_t& dLdux) {

	dLdux = intermediateHessian_.template block<input_dim_, state_dim_>(1+state_dim_, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::getTerminalCost(
		scalar_t& Phi) {

	Eigen::Matrix<scalar_t, 1, 1> costValue;
	terminalADInterfacePtr_->getFunctionValue(tapedInput_, costValue);
	Phi = costValue(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::getTerminalCostDerivativeTime(
		scalar_t& dPhidt) {

	terminalADInterfacePtr_->getJacobian(tapedInput_, terminalJacobian_);
	dPhidt = terminalJacobian_(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::getTerminalCostDerivativeState(
		state_vector_t& dPhidx) {

	terminalADInterfacePtr_->getJacobian(tapedInput_, terminalJacobian_);
	dPhidx = terminalJacobian_.template segment<state_dim_>(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::getTerminalCostSecondDerivativeState(
		state_matrix_t& dPhidxx) {

	terminalADInterfacePtr_->getHessian(tapedInput_, terminalHessian_, 0);
	dPhidxx = terminalHessian_.template block<state_dim_, state_dim_>(1, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::intermediateCostFunctionAD(
		const ad_dynamic_vector_t& tapedInput,
		ad_dynamic_vector_t& costValue) {

	auto& tapedInputNonConst = const_cast<ad_dynamic_vector_t&>(tapedInput);

	ad_scalar_t& t = timeVariable(tapedInputNonConst);
	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> x = stateVariables(tapedInputNonConst);
	Eigen::Matrix<ad_scalar_t, INPUT_DIM, 1> u = inputVariables(tapedInputNonConst);
	Eigen::Matrix<ad_scalar_t, STATE_DESIRED_DIM, 1> xDesired = desiredStateVariables(tapedInputNonConst);
	Eigen::Matrix<ad_scalar_t, INPUT_DESIRED_DIM, 1> uDesired = desiredInputVariables(tapedInputNonConst);
	Eigen::Matrix<ad_scalar_t, LOGIC_VARIABLE_DIM, 1> logicVar = logicVariables(tapedInputNonConst);

	costValue.resize(1);
	ad_scalar_t& costValueScalar = costValue(0);
	static_cast<Derived *>(this)->template intermediateCostFunction<ad_scalar_t>(
			t, x, u,
			xDesired, uDesired, logicVar,
			costValueScalar);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::terminalCostFunctionAD(
		const ad_dynamic_vector_t& tapedInput,
		ad_dynamic_vector_t& costValue) {

	auto& tapedInputNonConst = const_cast<ad_dynamic_vector_t&>(tapedInput);

	ad_scalar_t& t = timeVariable(tapedInputNonConst);
	Eigen::Matrix<ad_scalar_t, STATE_DIM, 1> x = stateVariables(tapedInputNonConst);
	Eigen::Matrix<ad_scalar_t, STATE_DESIRED_DIM, 1> xDesired = desiredStateVariables(tapedInputNonConst);
	Eigen::Matrix<ad_scalar_t, LOGIC_VARIABLE_DIM, 1> logicVar = logicVariables(tapedInputNonConst);

	costValue.resize(1);
	ad_scalar_t& costValueScalar = costValue(0);
	static_cast<Derived *>(this)->template terminalCostFunction<ad_scalar_t>(
			t, x, xDesired, logicVar,
			costValueScalar);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::setADInterfaces() {

	intermediateCostAD_ = [this](
			const ad_dynamic_vector_t& x,
			ad_dynamic_vector_t& y) {
		this->intermediateCostFunctionAD(x, y);
	};

	terminalCostAD_ = [this](
			const ad_dynamic_vector_t& x,
			ad_dynamic_vector_t& y) {
		this->terminalCostFunctionAD(x, y);
	};

	intermediateSparsityPattern_.setZero();
	intermediateSparsityPattern_.template leftCols<variable_dim_>().setOnes();

	terminalSparsityPattern_.setZero();
	terminalSparsityPattern_.template leftCols<variable_dim_>().setOnes();

	intermediateADInterfacePtr_.reset( new ad_interface_t(
			intermediateCostAD_, intermediateSparsityPattern_) );

	terminalADInterfacePtr_.reset( new ad_interface_t(
			terminalCostAD_, terminalSparsityPattern_) );

	intermediateADInterfacePtr_->computeForwardModel(true);
	intermediateADInterfacePtr_->computeJacobianModel(true);
	intermediateADInterfacePtr_->computeHessianModel(true);

	terminalADInterfacePtr_->computeForwardModel(true);
	terminalADInterfacePtr_->computeJacobianModel(true);
	terminalADInterfacePtr_->computeHessianModel(true);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
void CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::createModels(
		bool verbose) {

	// sets all the required CppAdCodeGenInterfaces
	setADInterfaces();

	intermediateADInterfacePtr_->createModels(modelName_+"_intermediate", libraryFolder_, verbose);
	terminalADInterfacePtr_->createModels(modelName_+"_terminal", libraryFolder_, verbose);

	dynamicLibraryIsCompiled_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T, size_t LOGIC_VARIABLE_DIM, size_t STATE_DESIRED_DIM, size_t INPUT_DESIRED_DIM>
bool CostFunctionBaseAD<Derived, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM, STATE_DESIRED_DIM, INPUT_DESIRED_DIM>::loadModels(
		bool verbose) {

	bool intermediateLoaded = intermediateADInterfacePtr_->loadModels(modelName_+"_intermediate", libraryFolder_, verbose);

	bool terminalLoaded = terminalADInterfacePtr_->loadModels(modelName_+"_terminal", libraryFolder_, verbose);

	return (intermediateLoaded && terminalLoaded);
}

} // namespace ocs2
