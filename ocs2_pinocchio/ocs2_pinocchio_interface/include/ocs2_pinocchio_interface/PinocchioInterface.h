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

#include <Eigen/Dense>
#include <iosfwd>
#include <memory>
#include <string>
#include <type_traits>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <cppad/cg.hpp>

/* Forward declaration of main pinocchio types */
#include <ocs2_pinocchio_interface/pinocchio_forward_declaration.h>

#include <urdf_model/model.h>

namespace ocs2 {

template <typename SCALAR>
class PinocchioInterfaceTpl;

using PinocchioInterface = PinocchioInterfaceTpl<scalar_t>;
using PinocchioInterfaceCppAd = PinocchioInterfaceTpl<ad_scalar_t>;

/**
 * Pinocchio interface class contatining robot model and data.
 * The robot model is shared between interface instances.
 */
template <typename SCALAR>
class PinocchioInterfaceTpl final {
 public:
  using Model = pinocchio::ModelTpl<SCALAR, 0, pinocchio::JointCollectionDefaultTpl>;
  using Data = pinocchio::DataTpl<SCALAR, 0, pinocchio::JointCollectionDefaultTpl>;
  using JointModel = pinocchio::JointModelTpl<scalar_t, 0, pinocchio::JointCollectionDefaultTpl>;

  // Template for conditional compilation using SFINAE
  template <typename T>
  using EnableIfScalar_t = typename std::enable_if<std::is_same<T, scalar_t>::value, bool>::type;

  /**
   * Construct from given pinocchio model
   * @param[in] model pinocchio model
   */
  explicit PinocchioInterfaceTpl(const Model& model, const std::shared_ptr<const ::urdf::ModelInterface> urdfModelPtr =
                                                         std::shared_ptr<const ::urdf::ModelInterface>());

  /** Destructor */
  ~PinocchioInterfaceTpl();

  /** Copy constructor */
  PinocchioInterfaceTpl(const PinocchioInterfaceTpl& rhs);

  /** Move constructor */
  PinocchioInterfaceTpl(PinocchioInterfaceTpl&& rhs);

  /** Copy assignment operator */
  PinocchioInterfaceTpl& operator=(const PinocchioInterfaceTpl& rhs);

  /** Move assignment */
  PinocchioInterfaceTpl<SCALAR>& operator=(PinocchioInterfaceTpl&& rhs);

  /** Get the pinocchio model */
  const Model& getModel() const { return *robotModelPtr_; }

  /** Get the pinocchio data */
  Data& getData() { return *robotDataPtr_; }
  const Data& getData() const { return *robotDataPtr_; }

  /** Get the urdf model */
  const std::shared_ptr<const ::urdf::ModelInterface>& getUrdfModelPtr() const { return urdfModelPtr_; }

  /** Cast pinocchio interface to CppAD scalar type. */
  template <typename T = SCALAR, EnableIfScalar_t<T> = true>
  PinocchioInterfaceCppAd toCppAd() const;

  friend std::ostream& operator<<(std::ostream& os, const PinocchioInterfaceTpl<scalar_t>& p);

 private:
  std::shared_ptr<const Model> robotModelPtr_;
  std::unique_ptr<Data> robotDataPtr_;
  std::shared_ptr<const ::urdf::ModelInterface> urdfModelPtr_;
};

/** Print PinocchioInterfaceTpl info to stream */
std::ostream& operator<<(std::ostream& os, const PinocchioInterface& p);

/* Explicit template instantiation for scalar_t and ad_scalar_t */
extern template class PinocchioInterfaceTpl<scalar_t>;
extern template class PinocchioInterfaceTpl<ad_scalar_t>;

}  // namespace ocs2
