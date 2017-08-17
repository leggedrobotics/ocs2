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
 * Dimensions class
 * @tparam STATE_DIM
 * @tparam INPUT_DIM
 * @tparam OUTPUT_DIM
 */
template<size_t STATE_DIM, size_t INPUT_DIM, size_t OUTPUT_DIM = STATE_DIM>
class Dimensions {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 * Enum for Dimensions
	 */
	enum DIMS {
		STATE_DIM_  = STATE_DIM,
		INPUT_DIM_  = INPUT_DIM,
		OUTPUT_DIM_ = OUTPUT_DIM,
		MAX_CONSTRAINT1_DIM_ = INPUT_DIM,
		MAX_CONSTRAINT2_DIM_ = INPUT_DIM
	};

	typedef Eigen::Matrix<double, STATE_DIM, 1> state_vector_t;
	typedef std::vector<state_vector_t, Eigen::aligned_allocator<state_vector_t> > state_vector_array_t;
	typedef std::vector<state_vector_array_t, Eigen::aligned_allocator<state_vector_array_t> > state_vector_array2_t;

	typedef Eigen::Matrix<double, OUTPUT_DIM, 1> output_vector_t;
	typedef std::vector<output_vector_t, Eigen::aligned_allocator<output_vector_t> > output_vector_array_t;
	typedef std::vector<output_vector_array_t, Eigen::aligned_allocator<output_vector_array_t> > output_vector_array2_t;

	typedef Eigen::Matrix<double, OUTPUT_DIM, OUTPUT_DIM> state_matrix_t;
	typedef std::vector<state_matrix_t, Eigen::aligned_allocator<state_matrix_t> > state_matrix_array_t;
	typedef std::vector<state_matrix_array_t, Eigen::aligned_allocator<state_matrix_array_t> > state_matrix_array2_t;

	typedef Eigen::Matrix<double, OUTPUT_DIM, INPUT_DIM> control_gain_matrix_t;
	typedef std::vector<control_gain_matrix_t, Eigen::aligned_allocator<control_gain_matrix_t> > control_gain_matrix_array_t;
	typedef std::vector<control_gain_matrix_array_t, Eigen::aligned_allocator<control_gain_matrix_array_t> > control_gain_matrix_array2_t;

	typedef Eigen::Matrix<double, INPUT_DIM, OUTPUT_DIM> control_feedback_t;
	typedef std::vector<control_feedback_t, Eigen::aligned_allocator<control_feedback_t> > control_feedback_array_t;
	typedef std::vector<control_feedback_array_t, Eigen::aligned_allocator<control_feedback_array_t> > control_feedback_array2_t;

	typedef Eigen::Matrix<double, INPUT_DIM, 1> control_vector_t;
	typedef std::vector<control_vector_t, Eigen::aligned_allocator<control_vector_t> > control_vector_array_t;
	typedef std::vector<control_vector_array_t, Eigen::aligned_allocator<control_vector_array_t> > control_vector_array2_t;

	typedef Eigen::Matrix<double, INPUT_DIM, INPUT_DIM> control_matrix_t;
	typedef std::vector<control_matrix_t, Eigen::aligned_allocator<control_matrix_t> > control_matrix_array_t;
	typedef std::vector<control_matrix_array_t, Eigen::aligned_allocator<control_matrix_array_t> > control_matrix_array2_t;

    typedef Eigen::Matrix<double, OUTPUT_DIM*OUTPUT_DIM , 1 > state_matrix_vectorized_t;

    typedef Eigen::Matrix<double, MAX_CONSTRAINT1_DIM_, 1> constraint1_vector_t;
    typedef std::vector<constraint1_vector_t, Eigen::aligned_allocator<constraint1_vector_t> > constraint1_vector_array_t;
    typedef std::vector<constraint1_vector_array_t, Eigen::aligned_allocator<constraint1_vector_array_t> > constraint1_vector_array2_t;

    typedef Eigen::Matrix<double, MAX_CONSTRAINT1_DIM_, 1> constraint1_matrix_t;
    typedef std::vector<constraint1_matrix_t, Eigen::aligned_allocator<constraint1_matrix_t> > constraint1_matrix_array_t;
    typedef std::vector<constraint1_matrix_array_t, Eigen::aligned_allocator<constraint1_matrix_array_t> > constraint1_matrix_array2_t;

    typedef Eigen::Matrix<double, MAX_CONSTRAINT1_DIM_, OUTPUT_DIM> constraint1_state_matrix_t;
    typedef std::vector<constraint1_state_matrix_t, Eigen::aligned_allocator<constraint1_state_matrix_t> > constraint1_state_matrix_array_t;
    typedef std::vector<constraint1_state_matrix_array_t, Eigen::aligned_allocator<constraint1_state_matrix_array_t> > constraint1_state_matrix_array2_t;

    typedef Eigen::Matrix<double, MAX_CONSTRAINT1_DIM_, INPUT_DIM> constraint1_control_matrix_t;
    typedef std::vector<constraint1_control_matrix_t, Eigen::aligned_allocator<constraint1_control_matrix_t> > constraint1_control_matrix_array_t;
    typedef std::vector<constraint1_control_matrix_array_t, Eigen::aligned_allocator<constraint1_control_matrix_array_t> > constraint1_control_matrix_array2_t;

    typedef Eigen::Matrix<double, MAX_CONSTRAINT1_DIM_, INPUT_DIM> control_constraint1_matrix_t;
    typedef std::vector<control_constraint1_matrix_t, Eigen::aligned_allocator<control_constraint1_matrix_t> > control_constraint1_matrix_array_t;
    typedef std::vector<control_constraint1_matrix_array_t, Eigen::aligned_allocator<control_constraint1_matrix_array_t> > control_constraint1_matrix_array2_t;

    typedef Eigen::Matrix<double, MAX_CONSTRAINT2_DIM_, 1> constraint2_vector_t;
    typedef std::vector<constraint2_vector_t, Eigen::aligned_allocator<constraint2_vector_t> > constraint2_vector_array_t;
    typedef std::vector<constraint2_vector_array_t, Eigen::aligned_allocator<constraint2_vector_array_t> > constraint2_vector_array2_t;

    typedef Eigen::Matrix<double, MAX_CONSTRAINT2_DIM_, OUTPUT_DIM> constraint2_state_matrix_t;
    typedef std::vector<constraint2_state_matrix_t, Eigen::aligned_allocator<constraint2_state_matrix_t> > constraint2_state_matrix_array_t;
    typedef std::vector<constraint2_state_matrix_array_t, Eigen::aligned_allocator<constraint2_state_matrix_array_t> > constraint2_state_matrix_array2_t;

	typedef double scalar_t;
	typedef std::vector<scalar_t> scalar_array_t;

	typedef Eigen::Matrix<double, 1, 1> eigen_scalar_t;
	typedef std::vector<eigen_scalar_t, Eigen::aligned_allocator<eigen_scalar_t> > eigen_scalar_array_t;
	typedef std::vector<eigen_scalar_array_t, Eigen::aligned_allocator<eigen_scalar_array_t> > eigen_scalar_array2_t;

	/**
	 * Description of the struct
	 * @tparam DIM1
	 * @tparam DIM2
	 */
	 template<int DIM1, int DIM2 = 1>
	struct LinearFunction_t {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		scalar_array_t time_;
		std::vector<Eigen::Matrix<double, DIM1, DIM2>, Eigen::aligned_allocator<Eigen::Matrix<double, DIM1, DIM2>> > uff_;
		std::vector<Eigen::Matrix<double, DIM1, DIM2>, Eigen::aligned_allocator<Eigen::Matrix<double, DIM1, DIM2>> > deltaUff_;
		std::vector<Eigen::Matrix<double, DIM1, OUTPUT_DIM>, Eigen::aligned_allocator<Eigen::Matrix<double, DIM1, OUTPUT_DIM>> > k_;

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
			std::fill(k_.begin(), k_.end(), Eigen::Matrix<double, DIM1, OUTPUT_DIM>::Zero());
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
	template <int DIM1, int DIM2=1>
	using linearFunction_array_t = std::vector<LinearFunction_t<DIM1,DIM2>, Eigen::aligned_allocator<LinearFunction_t<DIM1,DIM2>> >;

	typedef LinearFunction_t<INPUT_DIM> 	  controller_t;
	typedef linearFunction_array_t<INPUT_DIM> controller_array_t;

	enum RICCATI_INTEGRATOR_TYPE{
		ODE45 = 1,
		ADAMS_BASHFORTH = 2,
		BULIRSCH_STOER = 3
	};

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

		size_t maxIterationGSLQP_;
		double minLearningRateGSLQP_;
		double maxLearningRateGSLQP_;
		double lineSearchContractionRate_;
		double minRelCostGSLQP_;
		double stateConstraintPenaltyCoeff_;
		double stateConstraintPenaltyBase_;
		double meritFunctionRho_;
		double constraintStepSize_;
		bool dispayGSLQP_;
		bool displayShortSummary_;
		bool warmStartGSLQP_;
		bool useLQForDerivatives_;

		double AbsTolODE_;
		double RelTolODE_;
		size_t maxNumStepsPerSecond_;
		bool simulationIsConstrained_;
		double minSimulationTimeDuration_;
		double minAbsConstraint1ISE_;
		double minRelConstraint1ISE_;

		bool displayGradientDescent_;
		double tolGradientDescent_;
		double acceptableTolGradientDescent_;
		size_t maxIterationGradientDescent_;
		double minLearningRateNLP_;
		double maxLearningRateNLP_;
		bool useAscendingLineSearchNLP_;
		double minAcceptedSwitchingTimeDifference_;
		bool checkNumericalStability_;

		size_t RiccatiIntegratorType_;
		double adams_integrator_dt_;

		bool useMultiThreading_;
		size_t nThreads_;
		bool debugPrintMP_;
		//mp line search options
		bool lsStepsizeGreedy_;	// otherwise it's merit-greedy

		bool useRiccatiSolver_;
	};
};

using HyQDimensions = Dimensions<36,12>;

} // namespace ocs2

#endif /* DIMENSIONS_H_ */
