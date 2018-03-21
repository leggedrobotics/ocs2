/*
 * Dimensions.h
 *
 *  Created on: Jan 3, 2016
 *      Author: farbod
 */

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
 * @tparam OUTPUT_DIM: Dimension of the output space.
 */
template<size_t STATE_DIM, size_t INPUT_DIM, size_t OUTPUT_DIM = STATE_DIM>
class Dimensions {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 * Enum for Dimensions
	 */
	enum DIMS {
		/** The State space dimension. */
		STATE_DIM_  = STATE_DIM,
		/** The control input space dimension. */
		INPUT_DIM_  = INPUT_DIM,
		/** The output space dimension. */
		OUTPUT_DIM_ = OUTPUT_DIM,
		/** The maximum permitted number of state-input constraints. */
		MAX_CONSTRAINT1_DIM_ = INPUT_DIM,
		/** The maximum permitted number of state-only constraints. */
		MAX_CONSTRAINT2_DIM_ = INPUT_DIM
	};

	/** Fixed-size state vector type with size \f$ n_x \f$ . */
	typedef Eigen::Matrix<double, STATE_DIM, 1> state_vector_t;
	/** State vector trajectory type. */
	typedef std::vector<state_vector_t, Eigen::aligned_allocator<state_vector_t> > state_vector_array_t;
	/** Array of state vector trajectory type. */
	typedef std::vector<state_vector_array_t, Eigen::aligned_allocator<state_vector_array_t> > state_vector_array2_t;

	/** Fixed-size output vector type with size \f$ n_y \f$ . */
	typedef Eigen::Matrix<double, OUTPUT_DIM, 1> output_vector_t;
	/** Output vector trajectory type. */
	typedef std::vector<output_vector_t, Eigen::aligned_allocator<output_vector_t> > output_vector_array_t;
	/** Array of output vector trajectory type. */
	typedef std::vector<output_vector_array_t, Eigen::aligned_allocator<output_vector_array_t> > output_vector_array2_t;

	/** Fixed-size state matrix type with size \f$ n_x * n_x \f$ . */
	typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> state_matrix_t;
	/** State matrix trajectory type. */
	typedef std::vector<state_matrix_t, Eigen::aligned_allocator<state_matrix_t> > state_matrix_array_t;
	/** Array of state matrix trajectory type. */
	typedef std::vector<state_matrix_array_t, Eigen::aligned_allocator<state_matrix_array_t> > state_matrix_array2_t;

	/** Fixed-size control-gain matrix type with size \f$ n_x * n_u \f$ . */
	typedef Eigen::Matrix<double, STATE_DIM, INPUT_DIM> control_gain_matrix_t;
	/** Control-gain matrix trajectory type. */
	typedef std::vector<control_gain_matrix_t, Eigen::aligned_allocator<control_gain_matrix_t> > control_gain_matrix_array_t;
	/** Array of control-gain matrix trajectory type. */
	typedef std::vector<control_gain_matrix_array_t, Eigen::aligned_allocator<control_gain_matrix_array_t> > control_gain_matrix_array2_t;

	/** Fixed-size control-feedback matrix type with size \f$ n_u * n_x \f$ . */
	typedef Eigen::Matrix<double, INPUT_DIM, STATE_DIM> control_feedback_t;
	/** Control-feedback matrix trajectory type. */
	typedef std::vector<control_feedback_t, Eigen::aligned_allocator<control_feedback_t> > control_feedback_array_t;
	/** Array of control-feedback matrix trajectory type. */
	typedef std::vector<control_feedback_array_t, Eigen::aligned_allocator<control_feedback_array_t> > control_feedback_array2_t;

	/** Fixed-size control input vector type with size \f$ n_u \f$ . */
	typedef Eigen::Matrix<double, INPUT_DIM, 1> control_vector_t;
	/** Control input vector trajectory type. */
	typedef std::vector<control_vector_t, Eigen::aligned_allocator<control_vector_t> > control_vector_array_t;
	/** Array of control input vector trajectory type. */
	typedef std::vector<control_vector_array_t, Eigen::aligned_allocator<control_vector_array_t> > control_vector_array2_t;

	/** Fixed-size control matrix type with size \f$ n_u \f$ . */
	typedef Eigen::Matrix<double, INPUT_DIM, INPUT_DIM> control_matrix_t;
	/** Control input matrix trajectory type. */
	typedef std::vector<control_matrix_t, Eigen::aligned_allocator<control_matrix_t> > control_matrix_array_t;
	/** Array of control input matrix trajectory type. */
	typedef std::vector<control_matrix_array_t, Eigen::aligned_allocator<control_matrix_array_t> > control_matrix_array2_t;

	/** Fixed-size vector type with size \f$ n_x^2 \f$ . */
    typedef Eigen::Matrix<double, STATE_DIM*STATE_DIM , 1 > state_matrix_vectorized_t;

    /** Fixed-size vector of state-input constraints (i.e. type-1 constraint) with size \f$ {n_c}_1 \f$ . */
    typedef Eigen::Matrix<double, MAX_CONSTRAINT1_DIM_, 1> constraint1_vector_t;
    /** Type-1 constraint (state-input constraint) vector trajectory type. */
    typedef std::vector<constraint1_vector_t, Eigen::aligned_allocator<constraint1_vector_t> > constraint1_vector_array_t;
    /** Array of type-1 constraint (state-input constraint) vector trajectory type. */
    typedef std::vector<constraint1_vector_array_t, Eigen::aligned_allocator<constraint1_vector_array_t> > constraint1_vector_array2_t;

    /** Fixed-size Matrix of state-input constraints (i.e. type-1 constraint) with size \f$ {n_c}_1 * {n_c}_1 \f$ . */
    typedef Eigen::Matrix<double, MAX_CONSTRAINT1_DIM_, 1> constraint1_matrix_t;
    /** Type-1 constraint (state-input constraint) matrix trajectory type. */
    typedef std::vector<constraint1_matrix_t, Eigen::aligned_allocator<constraint1_matrix_t> > constraint1_matrix_array_t;
    /** Array of type-1 constraint (state-input constraint) matrix trajectory type. */
    typedef std::vector<constraint1_matrix_array_t, Eigen::aligned_allocator<constraint1_matrix_array_t> > constraint1_matrix_array2_t;

    /** Fixed-size type-1 constraint by state matrix type with size \f$ {n_c}_1 * n_x \f$ . */
    typedef Eigen::Matrix<double, MAX_CONSTRAINT1_DIM_, STATE_DIM> constraint1_state_matrix_t;
    /** Constraint1_state matrix trajectory type. */
    typedef std::vector<constraint1_state_matrix_t, Eigen::aligned_allocator<constraint1_state_matrix_t> > constraint1_state_matrix_array_t;
    /** Array of constraint1_state matrix trajectory type. */
    typedef std::vector<constraint1_state_matrix_array_t, Eigen::aligned_allocator<constraint1_state_matrix_array_t> > constraint1_state_matrix_array2_t;

    /** Fixed-size type-1 constraint by control input matrix type with size \f$ {n_c}_1 * n_u \f$ . */
    typedef Eigen::Matrix<double, MAX_CONSTRAINT1_DIM_, INPUT_DIM> constraint1_control_matrix_t;
    /** constraint1_input matrix trajectory type. */
    typedef std::vector<constraint1_control_matrix_t, Eigen::aligned_allocator<constraint1_control_matrix_t> > constraint1_control_matrix_array_t;
    /** Array of constraint1_input matrix trajectory type. */
    typedef std::vector<constraint1_control_matrix_array_t, Eigen::aligned_allocator<constraint1_control_matrix_array_t> > constraint1_control_matrix_array2_t;

    /** Fixed-size control by type-1 constraint type with size \f$ n_u * {n_c}_1 \f$ . */
    typedef Eigen::Matrix<double, INPUT_DIM, MAX_CONSTRAINT1_DIM_> control_constraint1_matrix_t;
    /** Input-constraint1 matrix trajectory type. */
    typedef std::vector<control_constraint1_matrix_t, Eigen::aligned_allocator<control_constraint1_matrix_t> > control_constraint1_matrix_array_t;
    /** Array of Input-constraint1 matrix trajectory type. */
    typedef std::vector<control_constraint1_matrix_array_t, Eigen::aligned_allocator<control_constraint1_matrix_array_t> > control_constraint1_matrix_array2_t;

    /** Fixed-size vector of state-only constraints (i.e. type-2 constraint) with size \f$ {n_c}_2 \f$ . */
    typedef Eigen::Matrix<double, MAX_CONSTRAINT2_DIM_, 1> constraint2_vector_t;
    /** Type-2 constraint (state-only constraint) vector trajectory type. */
    typedef std::vector<constraint2_vector_t, Eigen::aligned_allocator<constraint2_vector_t> > constraint2_vector_array_t;
    /** Array of type-2 constraint (state-only constraint) vector trajectory type. */
    typedef std::vector<constraint2_vector_array_t, Eigen::aligned_allocator<constraint2_vector_array_t> > constraint2_vector_array2_t;

    /** Fixed-size type-2 constraint by state matrix type with size \f$ {n_c}_2 * n_x \f$ . */
    typedef Eigen::Matrix<double, MAX_CONSTRAINT2_DIM_, STATE_DIM> constraint2_state_matrix_t;
    /** Constraint2_state matrix trajectory type. */
    typedef std::vector<constraint2_state_matrix_t, Eigen::aligned_allocator<constraint2_state_matrix_t> > constraint2_state_matrix_array_t;
    /** Array of constraint2_state matrix trajectory type. */
    typedef std::vector<constraint2_state_matrix_array_t, Eigen::aligned_allocator<constraint2_state_matrix_array_t> > constraint2_state_matrix_array2_t;

    /** Unsigned integer type */
    // size_t is already defined
    typedef std::vector<size_t> size_array_t;

    /** Scalar type. */
	typedef double scalar_t;
	/** Scalar trajectory type. */
	typedef std::vector<scalar_t> scalar_array_t;

	/** Eigen scalar type. */
	typedef Eigen::Matrix<double, 1, 1> eigen_scalar_t;
	/** Eigen scalar trajectory type. */
	typedef std::vector<eigen_scalar_t, Eigen::aligned_allocator<eigen_scalar_t> > eigen_scalar_array_t;
	/** Array of eigen scalar trajectory type. */
	typedef std::vector<eigen_scalar_array_t, Eigen::aligned_allocator<eigen_scalar_array_t> > eigen_scalar_array2_t;


	/**
	 * Affine function class in the form \f$ u_{ff}(t) + K(t) \f$ where \f$ u_{ff} \f$ is a vector of size \f$ d_1 \f$ and
	 * \f$ K \f$ is a matrix of size \f$ d_1 * n_x \f$.
	 *
	 * @tparam DIM1: \f$ d_1 \f$.
	 */
	template <int DIM1, int DIM2=1>
	using LinearFunction_t = LinearFunction<STATE_DIM, DIM1, DIM2, scalar_t>;
	/** Array of LinearFunction_t */
	template <int DIM1, int DIM2=1>
	using linearFunction_array_t = std::vector<LinearFunction_t<DIM1,DIM2>, Eigen::aligned_allocator<LinearFunction_t<DIM1,DIM2>> >;

	/** Linear control policy in the form \f$ u_{ff}(t) + K(t) \f$. */
	typedef LinearFunction_t<INPUT_DIM,1>		controller_t;
	/** Array of linear control policy. */
	typedef linearFunction_array_t<INPUT_DIM,1> controller_array_t;

	/** Linear funtion for lagrange multiplier */
	typedef LinearFunction_t<Eigen::Dynamic,1> lagrange_t;
	/** Array of lagrange multiplier */
	typedef linearFunction_array_t<Eigen::Dynamic,1> lagrange_array_t;

	/**
	 * Riccati integrator type.
	 */
	enum RICCATI_INTEGRATOR_TYPE{
		/** ode45 type. */
		ODE45 = 1,
		/** adams-bashforth type. */
		ADAMS_BASHFORTH = 2,
		/** bulirch-store type. */
		BULIRSCH_STOER = 3
	};

};

} // namespace ocs2

#endif /* DIMENSIONS_H_ */
