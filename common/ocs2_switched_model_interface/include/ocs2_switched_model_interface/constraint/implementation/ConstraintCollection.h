
namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
ConstraintCollection<STATE_DIM, INPUT_DIM>::ConstraintCollection(ConstraintCollection&& rhs) noexcept
    : constraintTermMap_(std::move(rhs.constraintTermMap_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
ConstraintCollection<STATE_DIM, INPUT_DIM>::ConstraintCollection(const ConstraintCollection& rhs) {
  // Loop through all constraints by name and clone into the new object
  constraintTermMap_.clear();
  for (auto& constraintPair : rhs.constraintTermMap_) {
    add(constraintPair.first, std::unique_ptr<constraint_term_t>(constraintPair.second->clone()));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintCollection<STATE_DIM, INPUT_DIM>::add(std::string name, std::unique_ptr<constraint_term_t> constraintTerm) {
  auto info = constraintTermMap_.emplace(std::move(name), std::move(constraintTerm));
  if (!info.second) {
    throw std::runtime_error("[ConstraintCollection::add] Constraint name already exists");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template <typename Derived>
Derived& ConstraintCollection<STATE_DIM, INPUT_DIM>::get(const std::string& name) {
  static_assert(std::is_base_of<constraint_term_t, Derived>::value, "Template argument must derive from ConstraintTerm");
  // if the key does not exist throws an exception
  return dynamic_cast<Derived&>(*constraintTermMap_.at(name));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
std::vector<std::string> ConstraintCollection<STATE_DIM, INPUT_DIM>::constraintNameList() const {
  std::vector<std::string> constraintNames;
  for (const auto& constraintPair : constraintTermMap_) {
    constraintNames.push_back(constraintPair.first);
  }
  return constraintNames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
size_t ConstraintCollection<STATE_DIM, INPUT_DIM>::getNumConstraints(scalar_t time) const {
  size_t numConstraints = 0;

  // accumulate number of constraints for each constraintTerm
  for (auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive()) {
      numConstraints += constraintPair.second->getNumConstraints(time);
    }
  }

  return numConstraints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
auto ConstraintCollection<STATE_DIM, INPUT_DIM>::getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const
    -> scalar_array_t {
  scalar_array_t constraintValues;
  constraintValues.reserve(getNumConstraints(time));

  // append vectors of constraint values from each constraintTerm
  for (auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive()) {
      auto constraintTermValues = constraintPair.second->getValue(time, state, input);
      appendVectorToVectorByMoving(constraintValues, constraintTermValues);
    }
  }

  return constraintValues;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
auto ConstraintCollection<STATE_DIM, INPUT_DIM>::getLinearApproximation(scalar_t time, const state_vector_t& state,
                                                                        const input_vector_t& input) const -> LinearApproximation_t {
  const auto numConstraints = getNumConstraints(time);

  LinearApproximation_t linearApproximation;
  linearApproximation.constraintValues.reserve(numConstraints);
  linearApproximation.derivativeState.reserve(numConstraints);
  linearApproximation.derivativeInput.reserve(numConstraints);

  // append linearApproximation each constraintTerm
  for (auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive()) {
      auto constraintTermApproximation = constraintPair.second->getLinearApproximation(time, state, input);
      appendVectorToVectorByMoving(linearApproximation.constraintValues, constraintTermApproximation.constraintValues);
      appendVectorToVectorByMoving(linearApproximation.derivativeState, constraintTermApproximation.derivativeState);
      appendVectorToVectorByMoving(linearApproximation.derivativeInput, constraintTermApproximation.derivativeInput);
    }
  }

  return linearApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
auto ConstraintCollection<STATE_DIM, INPUT_DIM>::getQuadraticApproximation(scalar_t time, const state_vector_t& state,
                                                                           const input_vector_t& input) const -> QuadraticApproximation_t {
  const auto numConstraints = getNumConstraints(time);

  QuadraticApproximation_t quadraticApproximation;
  quadraticApproximation.constraintValues.reserve(numConstraints);
  quadraticApproximation.derivativeState.reserve(numConstraints);
  quadraticApproximation.derivativeInput.reserve(numConstraints);
  quadraticApproximation.secondDerivativesState.reserve(numConstraints);
  quadraticApproximation.secondDerivativesInput.reserve(numConstraints);
  quadraticApproximation.derivativesInputState.reserve(numConstraints);

  // append linearApproximation each constraintTerm
  for (auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive()) {
      auto constraintTermApproximation = constraintPair.second->getQuadraticApproximation(time, state, input);
      appendVectorToVectorByMoving(quadraticApproximation.constraintValues, constraintTermApproximation.constraintValues);
      appendVectorToVectorByMoving(quadraticApproximation.derivativeState, constraintTermApproximation.derivativeState);
      appendVectorToVectorByMoving(quadraticApproximation.derivativeInput, constraintTermApproximation.derivativeInput);
      appendVectorToVectorByMoving(quadraticApproximation.secondDerivativesState, constraintTermApproximation.secondDerivativesState);
      appendVectorToVectorByMoving(quadraticApproximation.secondDerivativesInput, constraintTermApproximation.secondDerivativesInput);
      appendVectorToVectorByMoving(quadraticApproximation.derivativesInputState, constraintTermApproximation.derivativesInputState);
    }
  }

  return quadraticApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
Eigen::VectorXd ConstraintCollection<STATE_DIM, INPUT_DIM>::getValueAsVector(scalar_t time, const state_vector_t& state,
                                                                             const input_vector_t& input) const {
  Eigen::VectorXd constraintVector(getNumConstraints(time));

  // append vectors of constraint values from each constraintTerm
  int nextFreeIndex = 0;
  for (auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive()) {
      auto constraintTermValues = constraintPair.second->getValue(time, state, input);

      // add constraint to next free index of Eigen::VectorXd
      for (const auto& v : constraintTermValues) {
        constraintVector[nextFreeIndex] = v;
        nextFreeIndex++;
      }
    }
  }

  return constraintVector;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
auto ConstraintCollection<STATE_DIM, INPUT_DIM>::getLinearApproximationAsMatrices(scalar_t time, const state_vector_t& state,
                                                                                  const input_vector_t& input) const
    -> LinearApproximationAsMatrices_t {
  LinearApproximationAsMatrices_t linearApproximationAsMatrices;

  // append linearApproximation each constraintTerm
  int nextFreeIndex = 0;
  for (auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive()) {
      auto constraintTermApproximation = constraintPair.second->getLinearApproximation(time, state, input);

      // add constraint to next free index of eigen vector vector
      for (int i = 0; i < constraintTermApproximation.constraintValues.size(); i++) {
        if (nextFreeIndex >= linearApproximationAsMatrices.constraintValues.rows()) {
          throw std::runtime_error(
              "[ConstraintCollection::getLinearApproximationAsMatrices] More constraints than rows available in the approximation "
              "matrices");
        }

        linearApproximationAsMatrices.constraintValues(nextFreeIndex) = constraintTermApproximation.constraintValues[i];
        linearApproximationAsMatrices.derivativeState.row(nextFreeIndex) = constraintTermApproximation.derivativeState[i];
        linearApproximationAsMatrices.derivativeInput.row(nextFreeIndex) = constraintTermApproximation.derivativeInput[i];
        nextFreeIndex++;
      }
    }
  }

  return linearApproximationAsMatrices;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template <typename T, typename Allocator>
void ConstraintCollection<STATE_DIM, INPUT_DIM>::appendVectorToVectorByMoving(std::vector<T, Allocator>& v1,
                                                                              std::vector<T, Allocator>& v2) const {
  v1.insert(v1.end(), std::make_move_iterator(v2.begin()), std::make_move_iterator(v2.end()));
}

}  // namespace ocs2
