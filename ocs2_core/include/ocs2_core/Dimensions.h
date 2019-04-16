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

#ifndef DIMENSIONS_OCS2_H_
#define DIMENSIONS_OCS2_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

#include "ocs2_core/misc/LinearFunction.h"

namespace ocs2{

/**
 * This class defines the types which are used throughout this package.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template<int STATE_DIM, int INPUT_DIM>
class Dimensions {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 * Enum for Dimensions
	 */
	enum DIMS
	{
		/** The State space dimension. */
		STATE_DIM_  = STATE_DIM,
		/** The control input space dimension. */
		INPUT_DIM_  = INPUT_DIM,
		/** The maximum permitted number of state-input constraints. */
		MAX_CONSTRAINT1_DIM_ = INPUT_DIM,
		/** The maximum permitted number of state-only constraints. */
		MAX_CONSTRAINT2_DIM_ = INPUT_DIM
	};

	/**
	 * Riccati integrator type.
	 */
	enum RICCATI_INTEGRATOR_TYPE
	{
		/** ode45 type. */
		ODE45 = 1,
		/** adams-bashforth type. */
		ADAMS_BASHFORTH = 2,
		/** bulirch-store type. */
		BULIRSCH_STOER = 3
	};


    /** Unsigned integer type */
    // size_t is already defined
    typedef std::vector<size_t> size_array_t;

    /** Scalar type. */
	typedef double scalar_t;
	/** Scalar trajectory type. */
	typedef std::vector<scalar_t> scalar_array_t;
    /** Array of scalar trajectory type. */
	typedef std::vector<scalar_array_t> scalar_array2_t;
    /** Array of arrays of scalar trajectory type. */
    typedef std::vector<scalar_array2_t> scalar_array3_t;

	/** Eigen scalar type. */
	typedef Eigen::Matrix<scalar_t, 1, 1> eigen_scalar_t;
	/** Eigen scalar trajectory type. */
	typedef std::vector<eigen_scalar_t, Eigen::aligned_allocator<eigen_scalar_t> > eigen_scalar_array_t;
	/** Array of eigen scalar trajectory type. */
	typedef std::vector<eigen_scalar_array_t, Eigen::aligned_allocator<eigen_scalar_array_t> > eigen_scalar_array2_t;

	/** Fixed-size state vector type with size \f$ n_x \f$ . */
	typedef Eigen::Matrix<scalar_t, STATE_DIM, 1> state_vector_t;
	/** State vector trajectory type. */
	typedef std::vector<state_vector_t, Eigen::aligned_allocator<state_vector_t> > state_vector_array_t;
	/** Array of state vector trajectory type. */
	typedef std::vector<state_vector_array_t, Eigen::aligned_allocator<state_vector_array_t> > state_vector_array2_t;
    /** Array of arrays of state vector trajectory type. */
    typedef std::vector<state_vector_array2_t, Eigen::aligned_allocator<state_vector_array2_t> > state_vector_array3_t;

	/** Fixed-size state matrix type with size \f$ n_x * n_x \f$ . */
	typedef Eigen::Matrix<scalar_t, STATE_DIM, STATE_DIM> state_matrix_t;
	/** State matrix trajectory type. */
	typedef std::vector<state_matrix_t, Eigen::aligned_allocator<state_matrix_t> > state_matrix_array_t;
	/** Array of state matrix trajectory type. */
	typedef std::vector<state_matrix_array_t, Eigen::aligned_allocator<state_matrix_array_t> > state_matrix_array2_t;
    /** Array of arrays of state matrix trajectory type. */
    typedef std::vector<state_matrix_array2_t, Eigen::aligned_allocator<state_matrix_array2_t> > state_matrix_array3_t;

	/** Fixed-size state-input matrix type with size \f$ n_x * n_u \f$ . */
	typedef Eigen::Matrix<scalar_t, STATE_DIM, INPUT_DIM> state_input_matrix_t;
	/** State-input matrix trajectory type. */
	typedef std::vector<state_input_matrix_t, Eigen::aligned_allocator<state_input_matrix_t> > state_input_matrix_array_t;
	/** Array of state-input matrix trajectory type. */
	typedef std::vector<state_input_matrix_array_t, Eigen::aligned_allocator<state_input_matrix_array_t> > state_input_matrix_array2_t;
    /** Array of arrays of state-input matrix trajectory type. */
    typedef std::vector<state_input_matrix_array2_t, Eigen::aligned_allocator<state_input_matrix_array2_t> > state_input_matrix_array3_t;

	/** Fixed-size input_state matrix type with size \f$ n_u * n_x \f$ . */
	typedef Eigen::Matrix<scalar_t, INPUT_DIM, STATE_DIM> input_state_matrix_t;
	/** Input_state matrix trajectory type. */
	typedef std::vector<input_state_matrix_t, Eigen::aligned_allocator<input_state_matrix_t> > input_state_matrix_array_t;
	/** Array of input_state matrix trajectory type. */
	typedef std::vector<input_state_matrix_array_t, Eigen::aligned_allocator<input_state_matrix_array_t> > input_state_matrix_array2_t;
    /** Array of arrays of input_state matrix trajectory type. */
    typedef std::vector<input_state_matrix_array2_t, Eigen::aligned_allocator<input_state_matrix_array2_t> > input_state_matrix_array3_t;

	/** Fixed-size control input vector type with size \f$ n_u \f$ . */
	typedef Eigen::Matrix<scalar_t, INPUT_DIM, 1> input_vector_t;
	/** Control input vector trajectory type. */
	typedef std::vector<input_vector_t, Eigen::aligned_allocator<input_vector_t> > input_vector_array_t;
	/** Array of control input vector trajectory type. */
	typedef std::vector<input_vector_array_t, Eigen::aligned_allocator<input_vector_array_t> > input_vector_array2_t;
    /** Array of arrays of control input vector trajectory type. */
    typedef std::vector<input_vector_array2_t, Eigen::aligned_allocator<input_vector_array2_t> > input_vector_array3_t;

	/** Fixed-size control input matrix type with size \f$ n_u \f$ . */
	typedef Eigen::Matrix<scalar_t, INPUT_DIM, INPUT_DIM> input_matrix_t;
	/** Control input matrix trajectory type. */
	typedef std::vector<input_matrix_t, Eigen::aligned_allocator<input_matrix_t> > input_matrix_array_t;
	/** Array of control input matrix trajectory type. */
	typedef std::vector<input_matrix_array_t, Eigen::aligned_allocator<input_matrix_array_t> > input_matrix_array2_t;
    /** Array of arrays of control input matrix trajectory type. */
    typedef std::vector<input_matrix_array2_t, Eigen::aligned_allocator<input_matrix_array2_t> > input_matrix_array3_t;

	/** Dynamic-size vector type. */
	typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, 1> dynamic_vector_t;
	/** Dynamic vector's trajectory type. */
	typedef std::vector<dynamic_vector_t, Eigen::aligned_allocator<dynamic_vector_t> > dynamic_vector_array_t;
	/** Array of dynamic vector's trajectory type. */
	typedef std::vector<dynamic_vector_array_t, Eigen::aligned_allocator<dynamic_vector_array_t> > dynamic_vector_array2_t;

	/** Dynamic-size matrix type. */
	typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> dynamic_matrix_t;
	/** Dynamic matrix's trajectory type. */
	typedef std::vector<dynamic_matrix_t, Eigen::aligned_allocator<dynamic_matrix_t> > dynamic_matrix_array_t;
	/** Array of dynamic matrix's trajectory type. */
	typedef std::vector<dynamic_matrix_array_t, Eigen::aligned_allocator<dynamic_matrix_array_t> > dynamic_matrix_array2_t;

	/** Dynamic-size by n_x matrix type. */
	typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, STATE_DIM> dynamic_state_matrix_t;
	/** Dynamic-state matrix trajectory type. */
	typedef std::vector<dynamic_state_matrix_t, Eigen::aligned_allocator<dynamic_state_matrix_t> > dynamic_state_matrix_array_t;
	/** Array of dynamic-state matrix trajectory type. */
	typedef std::vector<dynamic_state_matrix_array_t, Eigen::aligned_allocator<dynamic_state_matrix_array_t> > dynamic_state_matrix_array2_t;

	/** n_x by dynamic-size matrix type. */
	typedef Eigen::Matrix<scalar_t, STATE_DIM, Eigen::Dynamic> state_dynamic_matrix_t;
	/** state_dynamic matrix trajectory type. */
	typedef std::vector<state_dynamic_matrix_t, Eigen::aligned_allocator<state_dynamic_matrix_t> > state_dynamic_matrix_array_t;
	/** Array of state_dynamic matrix trajectory type. */
	typedef std::vector<state_dynamic_matrix_array_t, Eigen::aligned_allocator<state_dynamic_matrix_array_t> > state_dynamic_matrix_array2_t;

	/** Dynamic-size by n_u matrix type. */
	typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, INPUT_DIM> dynamic_input_matrix_t;
	/** Dynamic-input matrix trajectory type. */
	typedef std::vector<dynamic_input_matrix_t, Eigen::aligned_allocator<dynamic_input_matrix_t> > dynamic_input_matrix_array_t;
	/** Array of dynamic-input matrix trajectory type. */
	typedef std::vector<dynamic_input_matrix_array_t, Eigen::aligned_allocator<dynamic_input_matrix_array_t> > dynamic_input_matrix_array2_t;

	/** n_u by dynamic-size matrix type. */
	typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, INPUT_DIM> input_dynamic_matrix_t;
	/** input_dynamic matrix trajectory type. */
	typedef std::vector<input_dynamic_matrix_t, Eigen::aligned_allocator<input_dynamic_matrix_t> > input_dynamic_matrix_array_t;
	/** Array of input_dynamic matrix trajectory type. */
	typedef std::vector<input_dynamic_matrix_array_t, Eigen::aligned_allocator<input_dynamic_matrix_array_t> > input_dynamic_matrix_array2_t;

	/** Fixed-size vector type with size \f$ n_x^2 \f$ . */
    typedef Eigen::Matrix<scalar_t, STATE_DIM*STATE_DIM , 1 > state_matrix_vectorized_t;

    /** Fixed-size vector of state-input constraints (i.e. type-1 constraint) with size \f$ {n_c}_1 \f$ . */
    typedef Eigen::Matrix<scalar_t, MAX_CONSTRAINT1_DIM_, 1> constraint1_vector_t;
    /** Type-1 constraint (state-input constraint) vector trajectory type. */
    typedef std::vector<constraint1_vector_t, Eigen::aligned_allocator<constraint1_vector_t> > constraint1_vector_array_t;
    /** Array of type-1 constraint (state-input constraint) vector trajectory type. */
    typedef std::vector<constraint1_vector_array_t, Eigen::aligned_allocator<constraint1_vector_array_t> > constraint1_vector_array2_t;

    /** Fixed-size Matrix of state-input constraints (i.e. type-1 constraint) with size \f$ {n_c}_1 * {n_c}_1 \f$ . */
    typedef Eigen::Matrix<scalar_t, MAX_CONSTRAINT1_DIM_, 1> constraint1_matrix_t;
    /** Type-1 constraint (state-input constraint) matrix trajectory type. */
    typedef std::vector<constraint1_matrix_t, Eigen::aligned_allocator<constraint1_matrix_t> > constraint1_matrix_array_t;
    /** Array of type-1 constraint (state-input constraint) matrix trajectory type. */
    typedef std::vector<constraint1_matrix_array_t, Eigen::aligned_allocator<constraint1_matrix_array_t> > constraint1_matrix_array2_t;

    /** Fixed-size type-1 constraint by state matrix type with size \f$ {n_c}_1 * n_x \f$ . */
    typedef Eigen::Matrix<scalar_t, MAX_CONSTRAINT1_DIM_, STATE_DIM> constraint1_state_matrix_t;
    /** Constraint1_state matrix trajectory type. */
    typedef std::vector<constraint1_state_matrix_t, Eigen::aligned_allocator<constraint1_state_matrix_t> > constraint1_state_matrix_array_t;
    /** Array of constraint1_state matrix trajectory type. */
    typedef std::vector<constraint1_state_matrix_array_t, Eigen::aligned_allocator<constraint1_state_matrix_array_t> > constraint1_state_matrix_array2_t;

    /** Fixed-size type-1 constraint by control input matrix type with size \f$ {n_c}_1 * n_u \f$ . */
    typedef Eigen::Matrix<scalar_t, MAX_CONSTRAINT1_DIM_, INPUT_DIM> constraint1_input_matrix_t;
    /** constraint1_input matrix trajectory type. */
    typedef std::vector<constraint1_input_matrix_t, Eigen::aligned_allocator<constraint1_input_matrix_t> > constraint1_input_matrix_array_t;
    /** Array of constraint1_input matrix trajectory type. */
    typedef std::vector<constraint1_input_matrix_array_t, Eigen::aligned_allocator<constraint1_input_matrix_array_t> > constraint1_input_matrix_array2_t;

    /** Fixed-size control by type-1 constraint type with size \f$ n_u * {n_c}_1 \f$ . */
    typedef Eigen::Matrix<scalar_t, INPUT_DIM, MAX_CONSTRAINT1_DIM_> control_constraint1_matrix_t;
    /** Input-constraint1 matrix trajectory type. */
    typedef std::vector<control_constraint1_matrix_t, Eigen::aligned_allocator<control_constraint1_matrix_t> > control_constraint1_matrix_array_t;
    /** Array of Input-constraint1 matrix trajectory type. */
    typedef std::vector<control_constraint1_matrix_array_t, Eigen::aligned_allocator<control_constraint1_matrix_array_t> > control_constraint1_matrix_array2_t;

    /** Fixed-size vector of state-only constraints (i.e. type-2 constraint) with size \f$ {n_c}_2 \f$ . */
    typedef Eigen::Matrix<scalar_t, MAX_CONSTRAINT2_DIM_, 1> constraint2_vector_t;
    /** Type-2 constraint (state-only constraint) vector trajectory type. */
    typedef std::vector<constraint2_vector_t, Eigen::aligned_allocator<constraint2_vector_t> > constraint2_vector_array_t;
    /** Array of type-2 constraint (state-only constraint) vector trajectory type. */
    typedef std::vector<constraint2_vector_array_t, Eigen::aligned_allocator<constraint2_vector_array_t> > constraint2_vector_array2_t;

    /** Fixed-size type-2 constraint by state matrix type with size \f$ {n_c}_2 * n_x \f$ . */
    typedef Eigen::Matrix<scalar_t, MAX_CONSTRAINT2_DIM_, STATE_DIM> constraint2_state_matrix_t;
    /** Constraint2_state matrix trajectory type. */
    typedef std::vector<constraint2_state_matrix_t, Eigen::aligned_allocator<constraint2_state_matrix_t> > constraint2_state_matrix_array_t;
    /** Array of constraint2_state matrix trajectory type. */
    typedef std::vector<constraint2_state_matrix_array_t, Eigen::aligned_allocator<constraint2_state_matrix_array_t> > constraint2_state_matrix_array2_t;

	/**
	 * Affine function class in the form \f$ u_{ff}(t) + K(t) x \f$ where \f$ u_{ff} \f$ is a vector of size \f$ d_1 \f$ and
	 * \f$ K \f$ is a matrix of size \f$ d_1 * n_x \f$.
	 *
	 * @tparam DIM1: \f$ d_1 \f$.
	 */
	template <int DIM1, int DIM2=1>
	using LinearFunction_t = LinearFunction<STATE_DIM, DIM1, DIM2, scalar_t>;
	/** Array of LinearFunction_t */
	template <int DIM1, int DIM2=1>
	using linearFunction_array_t = std::vector<LinearFunction_t<DIM1,DIM2>, Eigen::aligned_allocator<LinearFunction_t<DIM1,DIM2>> >;
	/** 2D array of LinearFunction_t */
	template <int DIM1, int DIM2=1>
	using linearFunction_array2_t = std::vector<linearFunction_array_t<DIM1,DIM2>, Eigen::aligned_allocator<linearFunction_array_t<DIM1,DIM2>> >;

	/** Linear control policy in the form \f$ u_{ff}(t) + K(t) \f$. */
	typedef LinearFunction_t<INPUT_DIM,1>		controller_t;
	/** Array of linear control policy. */
	typedef linearFunction_array_t<INPUT_DIM,1> controller_array_t;
	/** 2D array of linear control policy. */
	typedef linearFunction_array2_t<INPUT_DIM,1> controller_array2_t;

	/** Linear function for lagrange multiplier */
	typedef LinearFunction_t<MAX_CONSTRAINT1_DIM_,1> lagrange_t;
	/** Array of lagrange multiplier */
	typedef linearFunction_array_t<MAX_CONSTRAINT1_DIM_,1> lagrange_array_t;

};

} // namespace ocs2

#endif /* DIMENSIONS_H_ */
