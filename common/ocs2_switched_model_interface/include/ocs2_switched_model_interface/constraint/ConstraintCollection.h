//
// Created by rgrandia on 03.04.19.
//

#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <map>
#include <memory>
#include <vector>
#include "ocs2_core/Dimensions.h"

#include "ocs2_switched_model_interface/constraint/ConstraintCollectionView.h"
#include "ocs2_switched_model_interface/constraint/ConstraintTerm.h"

namespace ocs2 {

/*
 *  Container keeps a vector of unique pointers
 *   - const read is provided through the observerVector
 *   - write access is provided through the modifierVector
 */
template <typename T>
class TermContainer {
 public:
  void add(std::unique_ptr<T> term) {
    termModifierVector_.emplace_back(std::move(term));
    termObserverVector_.push_back(termModifierVector_.back().get());
  }

  const std::vector<std::unique_ptr<T>>& getModifierVector() const { return termModifierVector_; };
  const std::vector<T const*>& getObserverVector() const { return termObserverVector_; };
  size_t size() const { return termModifierVector_.size(); };

 private:
  std::vector<std::unique_ptr<T>> termModifierVector_;
  std::vector<T const*> termObserverVector_;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
class ConstraintCollection {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;

  using ConstraintTerm_t = ConstraintTerm<STATE_DIM, INPUT_DIM>;
  using ConstraintTermContainer_t = TermContainer<ConstraintTerm_t>;
  using ConstraintCollectionView_t = ConstraintCollectionView<STATE_DIM, INPUT_DIM>;

  ConstraintCollection() = default;
  ~ConstraintCollection() = default;
  ConstraintCollection(const ConstraintCollection& rhs) {
    // Loop through all constraints by name and clone into the new object
    for (const auto& nameIndex : rhs.nameIndexMap) {
      const auto& name = nameIndex.first;
      add(std::unique_ptr<ConstraintTerm_t>(rhs.getConstraint(name).clone()), name);
    }
  }

  ConstraintCollection& operator=(const ConstraintCollection&) = delete;

  /**
   * Add a constraint to the collection, and transfer ownership to the collection
   * The provided name must be unique and is later used to access the constraint.
   * @param constraintTerm: Constraint to be added.
   * @param name: Name stored along with the constraint.
   */
  void add(std::unique_ptr<ConstraintTerm_t> constraintTerm, const std::string& name) {
    if (nameIndexMap.find(name) == nameIndexMap.end()) {
      constraintTermContainer.add(std::move(constraintTerm));
      nameIndexMap.insert({name, constraintTermContainer.size() - 1});
    } else {
      throw std::runtime_error("[ConstraintCollection::add] Constraint name already exists");
    }
  };

  /**
   * Get an interface to the constraints to extract their concatenated values and derivatives.
   * @return Collection view on the constraints.
   */
  ConstraintCollectionView_t getConstraints() const { return ConstraintCollectionView_t(constraintTermContainer.getObserverVector()); };

  /**
   * Use to modify a constraint. The returned pointer is not to be stored since the ConstraintCollection contains a unique pointer to the
   * object
   * @tparam Derived: derived class of ConstraintTerm<STATE_DIM, INPUT_DIM> to cast to. Casts to the base class by default
   * @param name: Name of the constraint to modify
   * @return Pointer to the underlying constraint
   */
  template <typename Derived = ConstraintTerm_t>
  Derived* modifyConstraint(std::string name) {
    static_assert(std::is_convertible<Derived*, ConstraintTerm_t*>::value,
                  "Template argument must derive from ConstraintTerm<STATE_DIM, INPUT_DIM>");

    auto derivedPtr = dynamic_cast<Derived*>(constraintTermContainer.getModifierVector()[nameIndexMap.at(name)].get());
    if (derivedPtr) {
      return derivedPtr;
    } else {
      throw std::runtime_error("[ConstraintCollection::modifyConstraint] Could not cast to specified template argument");
    }
  };

  /**
   * Get read access to an indiviual constraint
   * @param name: Name of the constraint
   * @return Reference to the corresponding constraint
   */
  const ConstraintTerm_t& getConstraint(std::string name) const {
    return *constraintTermContainer.getObserverVector()[nameIndexMap.at(name)];
  };

 private:
  ConstraintTermContainer_t constraintTermContainer;
  std::map<std::string, int> nameIndexMap;
};

}  // namespace ocs2