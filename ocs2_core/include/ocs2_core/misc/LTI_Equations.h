/*
 * LTI_Equations.h
 *
 *  Created on: Jun 27, 2017
 *      Author: farbod
 */

#ifndef OCS2_LTI_EQUATIONS_H_
#define OCS2_LTI_EQUATIONS_H_

#include <vector>
#include "ocs2_core/integration/ODE_Base.h"

namespace ocs2{

/**
 * LTI system class for linear defined as \f$ dx/dt = G_m*x(t) + G_v \f$.
 * with \f$ x \in R^{DIM1 \times DIM2} \f$.
 *
 * @tparam DIM1: First dimension of the state space.
 * @tparam DIM2: Second dimension of the state space.
 * @tparam SCALAR_T: data type
 */
template <int DIM1, int DIM2=1, typename SCALAR_T=double>
class LTI_Equations : public ODE_Base<DIM1*DIM2>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum {
		LTI_DIM_ = DIM1*DIM2
	};

	typedef Eigen::Matrix<SCALAR_T, DIM1, DIM2> state_t;
	typedef Eigen::Matrix<SCALAR_T, LTI_DIM_, 1> vectorized_state_t;
	typedef std::vector<state_t, Eigen::aligned_allocator<state_t>> state_array_t;
	typedef std::vector<vectorized_state_t, Eigen::aligned_allocator<vectorized_state_t>> vectorized_state_array_t;

	LTI_Equations() = default;

	~LTI_Equations() = default;

	/**
	 * Transcribes the stacked vector into its matrix form.
	 *
	 * @param [in] vector: a single vector constructed by concatenating columns of matrix.
	 * @param [out] matrix: the original matrix
	 */
	void static convert2Matrix(
			const Eigen::Matrix<SCALAR_T, LTI_DIM_, 1>& vector,
			Eigen::Matrix<SCALAR_T, DIM1, DIM2>& matrix) {

		matrix = Eigen::Map<const Eigen::Matrix<SCALAR_T, DIM1, DIM2>>(vector.data());
	}

	/**
	 * Transcribes the state matrix type to a stacked vector.
	 *
	 * @param [in] matrix: the original matrix
	 * @param [out] vector: a single vector constructed by concatenating columns of matrix.
	 */
	void static convert2Vector(
			const Eigen::Matrix<SCALAR_T, DIM1, DIM2>& matrix,
			Eigen::Matrix<SCALAR_T, LTI_DIM_, 1>& vector) {

		vector = Eigen::Map<const Eigen::Matrix<SCALAR_T, LTI_DIM_, 1>>(matrix.data());
	}

	/**
	 * Set the LTI system coefficients, \f$ dx/dt = G_m*x(t) + G_v \f$.
	 * @param [in] Gm: G_m matrix.
	 * @param [in] Gv: G_v vector.
	 */
	void setData(
			const Eigen::Matrix<SCALAR_T, DIM1, DIM1>* GmPtr,
			const Eigen::Matrix<SCALAR_T, DIM1, DIM2>* GvPtr) {

		GmPtr_ = GmPtr;
		GvPtr_ = GvPtr;
	}

	/**
	 * Computes the LTI system dynamics.
	 *
	 * @param [in] t: Current time.
	 * @param [in] x: Current state.
	 * @param [out] dxdt: Current state time derivative
	 */
	void computeFlowMap(
			const SCALAR_T& t,
			const vectorized_state_t& x,
			vectorized_state_t& dxdt) override {

		Eigen::Map<Eigen::Matrix<SCALAR_T, DIM1, DIM2>> dxdt_Matrix(dxdt.data(), DIM1, DIM2);

		dxdt_Matrix = (*GmPtr_) * Eigen::Map<const Eigen::Matrix<SCALAR_T, DIM1, DIM2>>(x.data(), DIM1, DIM2) + (*GvPtr_);
	}


private:
	// members required in computeFlowMap
	const Eigen::Matrix<SCALAR_T, DIM1, DIM1>* GmPtr_;
	const Eigen::Matrix<SCALAR_T, DIM1, DIM2>* GvPtr_;
};

} // namespace ocs2


#endif /* OCS2_LTI_EQUATIONS_H_ */
