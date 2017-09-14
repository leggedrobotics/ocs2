/*
 * Dimensions.h
 *
 *  Created on: Jan 3, 2016
 *      Author: farbod
 */

#ifndef DIMENSIONS_OCS2_H_
#define DIMENSIONS_OCS2_H_

#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>


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
	 * Affine function class in the form \f$ u_{ff}(t) + K(t) \f$ where \f$ u_{ff} \f$ is a matrix of size \f$ d_1 * d_2 \f$ and
	 * \f$ K \f$ is a matrix of size \f$ d_1 * n_x \f$.
	 *
	 * @tparam DIM1: \f$ d_1 \f$.
	 * @tparam DIM2: \f$ d_2 \f$.
	 */
	template<int DIM1, int DIM2 = 1>
	struct LinearFunction_t {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		scalar_array_t time_;
		std::vector<Eigen::Matrix<double, DIM1, DIM2>, Eigen::aligned_allocator<Eigen::Matrix<double, DIM1, DIM2>> > uff_;
		std::vector<Eigen::Matrix<double, DIM1, DIM2>, Eigen::aligned_allocator<Eigen::Matrix<double, DIM1, DIM2>> > deltaUff_;
		std::vector<Eigen::Matrix<double, DIM1, STATE_DIM>, Eigen::aligned_allocator<Eigen::Matrix<double, DIM1, STATE_DIM>> > k_;

		/**
		 * Updates the internal variables with the given input
		 * @param [in] arg
		 */
		void swap(LinearFunction_t &arg) {
			time_.swap(arg.time_);
			uff_.swap(arg.uff_);
			deltaUff_.swap(arg.deltaUff_);
			k_.swap(arg.k_);
		}

		/**
		 * Sets all the data containers to zero
		 */
		void setZero() {
			std::fill(uff_.begin(), uff_.end(), Eigen::Matrix<double, DIM1, DIM2>::Zero());
			std::fill(deltaUff_.begin(), deltaUff_.end(), Eigen::Matrix<double, DIM1, DIM2>::Zero());
			std::fill(k_.begin(), k_.end(), Eigen::Matrix<double, DIM1, STATE_DIM>::Zero());
		}

		/**
		 * Clears the internal variables
		 */
		void clear() {
			time_.clear();
			uff_.clear();
			deltaUff_.clear();
			k_.clear();
		}
	};

	/**
	 * Affine function class in the form \f$ u_{ff}(t) + K(t) \f$ where \f$ u_{ff} \f$ is a vector of size \f$ d_1 \f$ and
	 * \f$ K \f$ is a matrix of size \f$ d_1 * n_x \f$.
	 *
	 * @tparam DIM1: \f$ d_1 \f$.
	 */
	template <int DIM1, int DIM2=1>
	using linearFunction_array_t = std::vector<LinearFunction_t<DIM1,DIM2>, Eigen::aligned_allocator<LinearFunction_t<DIM1,DIM2>> >;

	/**
	 * Linear control policy in the form \f$ u_{ff}(t) + K(t) \f$.
	 */
	typedef LinearFunction_t<INPUT_DIM> 	  controller_t;
	/**
	 * Array of linear control policy.
	 */
	typedef linearFunction_array_t<INPUT_DIM> controller_array_t;


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

	/**
	 * This structure contains the settings for the SLQ algorithm.
	 */
	struct Options {
	public:
		Options() :
			maxIterationGSLQP_(15),
			minLearningRateGSLQP_(0.05),
			maxLearningRateGSLQP_(1.0),
			lineSearchContractionRate_(0.5),
			minRelCostGSLQP_(1e-3),
			stateConstraintPenaltyCoeff_(0.0),
			stateConstraintPenaltyBase_(1.0),
			meritFunctionRho_(1.0),
			constraintStepSize_(1.0),
			dispayGSLQP_(false),
			displayShortSummary_(false),
			warmStartGSLQP_(false),
			useLQForDerivatives_(false),

			AbsTolODE_(1e-9),
			RelTolODE_(1e-6),
			maxNumStepsPerSecond_(5000),
			simulationIsConstrained_(false),
			minSimulationTimeDuration_(1e-3),
			minAbsConstraint1ISE_(1e-3),
			minRelConstraint1ISE_(1e-3),

			displayGradientDescent_(true),
			tolGradientDescent_(1e-2),
			acceptableTolGradientDescent_(1e-1),
			maxIterationGradientDescent_(20),
			minLearningRateNLP_(0.05),
		    maxLearningRateNLP_(1.0),
		    useAscendingLineSearchNLP_(true),
			minAcceptedSwitchingTimeDifference_(0.0),

			RiccatiIntegratorType_(ODE45),
			adams_integrator_dt_(0.001),

			useMultiThreading_(false),
			nThreads_(4),
			debugPrintMP_(false),
			lsStepsizeGreedy_(true),
			checkNumericalStability_(true),
			useRiccatiSolver_(true)
		{}

		/** Maximum number of iterations of SLQ. */
		size_t maxIterationGSLQP_;
		/** Minimum number of iterations of SLQ. */
		double minLearningRateGSLQP_;
		/** Maximum learning rate of line-search scheme in SLQ. */
		double maxLearningRateGSLQP_;
		/** Line-search scheme contraction rate. */
		double lineSearchContractionRate_;
		/** This value determines the termination condition based on the minimum relative changes of the cost. */
		double minRelCostGSLQP_;
		/** The penalty function coefficient, \f$\alpha\f$, for state-only constraints. \f$ p(i) = \alpha a^i \f$ */
		double stateConstraintPenaltyCoeff_;
		/** The penalty function base, \f$ a \f$, for state-only constraints. \f$ p(i) = \alpha a^i \f$ */
		double stateConstraintPenaltyBase_;
		/** merit function coefficient. */
		double meritFunctionRho_;
		/** Constant step size for type-1 constraints. */
		double constraintStepSize_;
		/** This value determines to display the log output of SLQ. */
		bool dispayGSLQP_;
		/** This value determines to display the a summary log of SLQ. */
		bool displayShortSummary_;
		/** This value determines to use a warm strating schem for calculating cost gradients w.r.t. switching times. */
		bool warmStartGSLQP_;
		/** This value determines to use LQ-based method or sweeping method for calculating cost gradients w.r.t. switching times. */
		bool useLQForDerivatives_;

		/** This value determines the absolute tolerance error for ode solvers. */
		double AbsTolODE_;
		/** This value determines the relative tolerance error for ode solvers. */
		double RelTolODE_;
		/** This value determines the maximum number of integration points per a second for ode solvers. */
		size_t maxNumStepsPerSecond_;
		bool simulationIsConstrained_;
		double minSimulationTimeDuration_;
		/** This value determines the maximum permitted absolute ISE (Integral of Square Error) for constrained type-1.*/
		double minAbsConstraint1ISE_;
		/** This value determines the maximum permitted relative ISE (Integral of Square Error) for constrained type-1.*/
		double minRelConstraint1ISE_;

		/** This value determines to display the log output of GSLQ. */
		bool displayGradientDescent_;
		double tolGradientDescent_;
		/** This value determines the termination condition for OCS2 based on the minimum relative changes of the cost.*/
		double acceptableTolGradientDescent_;
		/** This value determines the maximum number of iterations in OCS2 algorithm.*/
		size_t maxIterationGradientDescent_;
		/** This value determines the minimum step size for the line search scheme in OCS2 algorithm.*/
		double minLearningRateNLP_;
		/** This value determines the maximum step size for the line search scheme in OCS2 algorithm.*/
		double maxLearningRateNLP_;
		/**
		 * This value determines the the line search scheme to be used in OCS2. \n
		 * - \b Ascending: The step size eventually increases from the minimum value to the maximum. \n
		 * - \b Descending: The step size eventually decreases from the minimum value to the maximum.
		 * */
		bool useAscendingLineSearchNLP_;
		/** This value determines the minimum allowable difference between to consecutive switching times.*/
		double minAcceptedSwitchingTimeDifference_;

		/** Check the numerical stability of the algorithms for debugging purpose. */
		bool checkNumericalStability_;
		/** Riccati integrator type. */
		size_t RiccatiIntegratorType_;
		/** Adams integrator dt. */
		double adams_integrator_dt_;

		/** Use multi threading for the algorithms. */
		bool useMultiThreading_;
		/** Number of threads used in the multi threading scheme. */
		size_t nThreads_;
		/** Special debugging output for multi threading scheme. */
		bool debugPrintMP_;
		/**
		 * line search options in multi threading scheme.
		 * - True: The largest acceptable step-size will be chosen. The solution is equivalent to single core lineSearch.
		 * - False: The first acceptable step-size will be chosen.
		 * */
		bool lsStepsizeGreedy_;

		/** If true, SLQ uses ode solver to solve the Riccati equations. Otherwise it uses matrix exponential to solve it. */
		bool useRiccatiSolver_;
	};
};

} // namespace ocs2

#endif /* DIMENSIONS_H_ */
