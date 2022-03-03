/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace ocs2 {

/**
 * Implements the common add/get interface for cost and constraint collections.
 *
 * @tparam T : Type of the terms in the collection.
 */
template <typename T>
class Collection {
 public:
  Collection() = default;
  virtual ~Collection() = default;
  virtual Collection* clone() const { return new Collection(*this); }

  /** Checks if the collection has no elements */
  bool empty() const { return terms_.empty(); }

  /** Erases all elements from the Collection. */
  void clear();

  /**
   * Adds a term to the collection, and transfer ownership to the collection
   * The provided name must be unique and is later used to access the cost term.
   * @param name: Name stored along with the term.
   * @param term: Term to be added.
   */
  void add(std::string name, std::unique_ptr<T> term);

  /**
   * Erases a term from the collection.
   *
   * @param name: Name of the term.
   * @return True if the term was in the Collection and false if the term was not found in the Collection.
   */
  bool erase(const std::string& name) { return (extract(name) != nullptr); }

  /**
   * Removes a term from the Collection and returns it as a unique_ptr.
   *
   * @param name: Name of the term.
   * @return A unique pointer to the extracted term. If the term was not found it returns nullptr.
   */
  std::unique_ptr<T> extract(const std::string& name);

  /**
   * Use to modify a term.
   * @tparam Derived: derived class of base type T to cast to. Casts to the base class by default
   * @param name: Name of the term to modify
   * @return A reference to the underlying term
   */
  template <typename Derived = T>
  Derived& get(const std::string& name);

  /**
   * Finds the index of the term in the stored map.
   *
   * @param [in] name: Name of the term.
   * @param [out] index : Term index.
   * @return True if the name found in the collection.
   */
  bool getTermIndex(const std::string& name, size_t& index) const;

 protected:
  /** Copy constructor */
  Collection(const Collection& other);

  //! Contains all terms in the order they were added
  std::vector<std::unique_ptr<T>> terms_;

 private:
  //! Lookup from cost term name to index in the cost term vector
  std::unordered_map<std::string, size_t> termNameMap_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
void Collection<T>::clear() {
  terms_.clear();
  termNameMap_.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
void Collection<T>::add(std::string name, std::unique_ptr<T> term) {
  const size_t nextIndex = terms_.size();
  auto info = termNameMap_.emplace(std::move(name), nextIndex);
  if (info.second) {
    terms_.push_back(std::move(term));
  } else {
    throw std::runtime_error(std::string("[Collection::add] Term with name \"") + info.first->first + "\" already exists");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
std::unique_ptr<T> Collection<T>::extract(const std::string& name) {
  // find the term iterator with the name
  const auto termItr = termNameMap_.find(name);

  // term was not found
  if (termItr == termNameMap_.end()) {
    return nullptr;
  }

  // adjust the index of the terms added after the requested term
  const size_t termInd = termItr->second;
  for (auto itr = termNameMap_.begin(); itr != termNameMap_.end(); ++itr) {
    if (itr->second > termInd) {
      --itr->second;
    }
  }

  // remove term from map
  termNameMap_.erase(termItr);

  // get the term
  auto term = (std::move(terms_[termInd]));
  // remove the term
  terms_.erase(terms_.begin() + termInd);

  return term;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
template <typename Derived>
Derived& Collection<T>::get(const std::string& name) {
  static_assert(std::is_base_of<T, Derived>::value, "Template argument must derive from the base type of this collection");
  // if the key does not exist throws an exception
  const auto index = termNameMap_.at(name);
  return dynamic_cast<Derived&>(*terms_[index]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
Collection<T>::Collection(const Collection& other) : termNameMap_(other.termNameMap_) {
  // Loop through all terms and clone. The name map can be copied directly because the order stays the same.
  terms_.reserve(other.terms_.size());
  for (const auto& term : other.terms_) {
    terms_.emplace_back(term->clone());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
bool Collection<T>::getTermIndex(const std::string& name, size_t& index) const {
  auto itr = termNameMap_.find(name);
  if (itr != termNameMap_.cend()) {
    index = itr->second;
    return true;
  } else {
    index = termNameMap_.size();
    return false;
  }
}

/**
 * Helper function for merging two vectors by moving objects.
 * @param v1 : vector to move objects to
 * @param v2 : vector to move objects from
 */
template <typename T, typename Allocator>
inline void appendVectorToVectorByMoving(std::vector<T, Allocator>& v1, std::vector<T, Allocator>&& v2) {
  v1.insert(v1.end(), std::make_move_iterator(v2.begin()), std::make_move_iterator(v2.end()));
}

}  // namespace ocs2
