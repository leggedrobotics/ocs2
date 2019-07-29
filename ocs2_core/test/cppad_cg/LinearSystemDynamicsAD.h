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

#ifndef LINEARSYSTEMDYNAMICSAD_OCS2_H_
#define LINEARSYSTEMDYNAMICSAD_OCS2_H_

#include "ocs2_core/dynamics/SystemDynamicsBaseAD.h"

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
class LinearSystemDynamicsAD : public
SystemDynamicsBaseAD<LinearSystemDynamicsAD<STATE_DIM, INPUT_DIM>, STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<LinearSystemDynamicsAD<STATE_DIM, INPUT_DIM> > Ptr;
	typedef std::shared_ptr<const LinearSystemDynamicsAD<STATE_DIM, INPUT_DIM> > ConstPtr;

	typedef SystemDynamicsBaseAD<LinearSystemDynamicsAD<STATE_DIM, INPUT_DIM>, STATE_DIM, INPUT_DIM> BASE;
	typedef typename BASE::scalar_t             scalar_t;
	typedef typename BASE::state_vector_t       state_vector_t;
	typedef typename BASE::state_matrix_t       state_matrix_t;
	typedef typename BASE::input_vector_t       input_vector_t;
	typedef typename BASE::state_input_matrix_t state_input_matrix_t;

	LinearSystemDynamicsAD(
			const state_matrix_t& A,
			const state_input_matrix_t& B,
			const state_matrix_t& G,
			const state_input_matrix_t& H)
	: A_(A)
	, B_(B)
	, G_(G)
	, H_(H)
	{}

	~LinearSystemDynamicsAD() = default;

	/**
	 * Interface method to the state flow map of the hybrid system. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [in] input: input vector
	 * @param [out] stateDerivative: state vector time derivative.
	 */
	template <typename SCALAR_T>
	void systemFlowMap(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
			Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& stateDerivative) {

		stateDerivative =
				A_.template cast<SCALAR_T>() * state +
				B_.template cast<SCALAR_T>() * input;
	}

	/**
	 * Interface method to the state jump map of the hybrid system. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [out] jumpedState: jumped state.
	 */
	template <typename SCALAR_T>
	void systemJumpMap(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
			Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& jumpedState) {

		jumpedState =
				G_.template cast<SCALAR_T>() * state +
				H_.template cast<SCALAR_T>() * input;
	}

private:
	state_matrix_t 			A_;
	state_input_matrix_t 	B_;
	state_matrix_t 			G_;
	state_input_matrix_t 	H_;
};

} // namespace ocs2

#endif /* LINEARSYSTEMDYNAMICSAD_OCS2_H_ */
