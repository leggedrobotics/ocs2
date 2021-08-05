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

namespace ocs2 {
namespace centroidal_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived, typename SCALAR>
Eigen::Block<Derived, 3, 1> getContactForces(Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                             const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
  const size_t contactForceIndex = 3 * contactIndex;
  const size_t contactWrenchIndex = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts);
  const size_t startRow = (contactIndex < info.numThreeDofContacts) ? contactForceIndex : contactWrenchIndex;
  return Eigen::Block<Derived, 3, 1>(input.derived(), startRow, 0);
}

template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, 3, 1> getContactForces(const Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                                         const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
  const size_t contactForceIndex = 3 * contactIndex;
  const size_t contactWrenchIndex = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts);
  const size_t startRow = (contactIndex < info.numThreeDofContacts) ? contactForceIndex : contactWrenchIndex;
  return Eigen::Block<const Derived, 3, 1>(input.derived(), startRow, 0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived, typename SCALAR>
Eigen::Block<Derived, 3, 1> getContactTorques(Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                              const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
  assert(contactIndex >= info.numThreeDofContacts);
  const size_t startRow = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts) + 3;
  return Eigen::Block<Derived, 3, 1>(input.derived(), startRow, 0);
}

template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, 3, 1> getContactTorques(const Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                                          const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
  assert(contactIndex >= info.numThreeDofContacts);
  const size_t startRow = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts) + 3;
  return Eigen::Block<const Derived, 3, 1>(input.derived(), startRow, 0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived, typename SCALAR>
Eigen::Block<Derived, -1, 1> getJointVelocities(Eigen::MatrixBase<Derived>& input, const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  const size_t startRow = 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;
  return Eigen::Block<Derived, -1, 1>(input.derived(), startRow, 0, info.actuatedDofNum, 1);
}

template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, -1, 1> getJointVelocities(const Eigen::MatrixBase<Derived>& input,
                                                            const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  const size_t startRow = 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;
  return Eigen::Block<const Derived, -1, 1>(input.derived(), startRow, 0, info.actuatedDofNum, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived, typename SCALAR>
Eigen::Block<Derived, 6, 1> getNormalizedMomentum(Eigen::MatrixBase<Derived>& state, const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  return Eigen::Block<Derived, 6, 1>(state.derived(), 0, 0);
}

template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, 6, 1> getNormalizedMomentum(const Eigen::MatrixBase<Derived>& state,
                                                              const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  return Eigen::Block<const Derived, 6, 1>(state.derived(), 0, 0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived, typename SCALAR>
Eigen::Block<Derived, 6, 1> getBasePose(Eigen::MatrixBase<Derived>& state, const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  return Eigen::Block<Derived, 6, 1>(state.derived(), 6, 0);
}

template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, 6, 1> getBasePose(const Eigen::MatrixBase<Derived>& state, const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  return Eigen::Block<const Derived, 6, 1>(state.derived(), 6, 0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived, typename SCALAR>
Eigen::Block<Derived, -1, 1> getJointAngles(Eigen::MatrixBase<Derived>& state, const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  return Eigen::Block<Derived, -1, 1>(state.derived(), 12, 0, info.actuatedDofNum, 1);
}

template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, -1, 1> getJointAngles(const Eigen::MatrixBase<Derived>& state,
                                                        const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  return Eigen::Block<const Derived, -1, 1>(state.derived(), 12, 0, info.actuatedDofNum, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived, typename SCALAR>
Eigen::Block<Derived, -1, 1> getGeneralizedCoordinates(Eigen::MatrixBase<Derived>& state, const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  return Eigen::Block<Derived, -1, 1>(state.derived(), 6, 0, info.generalizedCoordinatesNum, 1);
}

template <typename Derived, typename SCALAR>
const Eigen::Block<const Derived, -1, 1> getGeneralizedCoordinates(const Eigen::MatrixBase<Derived>& state,
                                                                   const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  return Eigen::Block<const Derived, -1, 1>(state.derived(), 6, 0, info.generalizedCoordinatesNum, 1);
}

}  // namespace centroidal_model
}  // namespace ocs2
