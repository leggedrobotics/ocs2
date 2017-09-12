/*
 * GradientDescent.cpp
 *
 *  Created on: Jul 26, 2016
 *      Author: farbod
 */

#include "ocs2_frank_wolfe/GradientDescent.h"

namespace nlp {


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GradientDescent::adjustOptions() {
	numLineSearch_ = floor(log2(nlpOptions_.maxLearningRate_/nlpOptions_.minLearningRate_)) + 1;
	nlpOptions_.minLearningRate_ = (nlpOptions_.maxLearningRate_- 2*std::numeric_limits<double>::epsilon()) / pow(2,numLineSearch_-1);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool GradientDescent::calculateNumericalGradient(const size_t& id, const Eigen::VectorXd& parameters, Eigen::VectorXd& gradient) {
	bool status;
	double cost;
	status = calculateCost(id, parameters, cost);
	numFuntionCall_++;
	gradient.resize(parameters.rows());
	for (size_t i=0; i<parameters.rows(); i++)  {
		double h = sqrt(std::numeric_limits<double>::epsilon()) * std::max(parameters(i), 1.0);
		Eigen::VectorXd parametersPlus = parameters;
		parametersPlus(i) += h;
		double costPlus;
		bool statusPlus = calculateCost(id, parametersPlus, costPlus);
		numFuntionCall_++;
		gradient(i) = (costPlus-cost)/h;
		status = status && statusPlus;
	}  // end of i loop

	return status;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GradientDescent::ascendingLineSearch(const Eigen::VectorXd& gradient, double& learningRateStar) {

	learningRateStar = 0.0;
	id_ = 0;
	const Eigen::VectorXd initParameters = parameters_;
	double learningRate = nlpOptions_.minLearningRate_;
	size_t id = 1;

	while (learningRate<=nlpOptions_.maxLearningRate_)  {

		// lineSerach parameter
		Eigen::VectorXd lsParameters = initParameters - learningRate*gradient;

		// display
		if (nlpOptions_.displayGradientDescent_) {
			double lsMaxConstriant = 0.0;
			if (linearEqualityConstraintBv_.rows()>0)
				lsMaxConstriant = (linearEqualityConstraintAm_*lsParameters + linearEqualityConstraintBv_).maxCoeff() - nlpOptions_.minDisToBoundary_;
			std::cerr << "\t [" << id <<"] learningRate: " << learningRate << "\t max NLP constraint: " << lsMaxConstriant << std::endl;
			std::cerr << "\t     parameters:   " << lsParameters.transpose().format(CleanFmtDisplay_) << std::endl;
		}

		// calculate cost function
		double lsCost;
		bool status = calculateCost(id, lsParameters, lsCost);

		// increment the number of function calls
		numFuntionCall_++;

		// skip it if status is not OK
		if (status==false) {
			if (nlpOptions_.displayGradientDescent_)  std::cerr << "\t     cost:         " << lsCost << " (rejected)" << std::endl;
			learningRate *= 2.0;
			id++;
			continue;
		}

		// display
		if (nlpOptions_.displayGradientDescent_)  std::cerr << "\t     cost:         " << lsCost << std::endl;

		if (lsCost>cost_*(1.0-learningRate*1e-3))  break;

		cost_ = lsCost;
		parameters_ = lsParameters;
		id_ = id;
		learningRateStar = learningRate;

		learningRate *= 2.0;
		id++;

	}  // end of while loop

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GradientDescent::decreasingLineSearch(const Eigen::VectorXd& gradient, double& learningRateStar) {

	learningRateStar = 0.0;
	id_ = 0;
	const Eigen::VectorXd initParameters = parameters_;
	double learningRate = nlpOptions_.maxLearningRate_;

	size_t id = numLineSearch_;

	while (learningRate>=nlpOptions_.minLearningRate_)  {

		// lineSerach parameter
		Eigen::VectorXd lsParameters = initParameters - learningRate*gradient;

		// display
		if (nlpOptions_.displayGradientDescent_) {
			double lsMaxConstriant = 0.0;
			if (linearEqualityConstraintBv_.rows()>0)
				lsMaxConstriant = (linearEqualityConstraintAm_*lsParameters + linearEqualityConstraintBv_).maxCoeff() - nlpOptions_.minDisToBoundary_;
			std::cerr << "\t [" << id <<"] learningRate: " << learningRate << "\t maxNlpConstraint: " << lsMaxConstriant << std::endl;
			std::cerr << "\t     parameters:   " << lsParameters.transpose().format(CleanFmtDisplay_) << std::endl;
		}

		// calculate cost function
		double lsCost;
		bool status = calculateCost(id, lsParameters, lsCost);

		// increment the number of function calls
		numFuntionCall_++;

		// skip it if status is not OK
		if (status==false) {
			if (nlpOptions_.displayGradientDescent_)  std::cerr << "\t     cost:         " << lsCost << " (rejected)" << std::endl;
			learningRate /= 2.0;
			id--;
			continue;
		}

		if (nlpOptions_.displayGradientDescent_)  std::cerr << "\t     cost:         " << lsCost << std::endl;

		// display
		if (lsCost<cost_*(1.0-learningRate*1e-3))  {
			cost_ = lsCost;
			parameters_ = lsParameters;
			id_ = id;
			learningRateStar = learningRate;
			break;
		}

		learningRate /= 2.0;
		id--;

	}  // end of while loop

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GradientDescent::setupLP() {

	if (numConstraints_==0) return;

	// create a LP object
	lpPtr_ = glp_create_prob();
	glp_set_prob_name(lpPtr_, "FrankWolfe");

	// set it as a Minimization problem
	glp_set_obj_dir(lpPtr_, GLP_MIN);

	// set parameters limits
	glp_add_cols(lpPtr_, numParameters_);
	for (size_t i=0; i<numParameters_; i++)
		glp_set_col_bnds(lpPtr_, i+1, GLP_FR, 0.0, 0.0);

	// set the constraint limits
	glp_add_rows(lpPtr_, numConstraints_);
	for (size_t i=0; i<numConstraints_; i++)
		glp_set_row_bnds(lpPtr_, i+1, GLP_UP, 0.0, -linearEqualityConstraintBv_(i));

	// set the constraint coefficients
	std::vector<double> values;
	std::vector<int> xIndices, yIndices;
	values.push_back(0.0);   // the 0 index is not used
	xIndices.push_back(-1);  // the 0 index is not used
	yIndices.push_back(-1);  // the 0 index is not used
	for (size_t i=0; i<linearEqualityConstraintAm_.rows(); i++)
		for (size_t j=0; j<linearEqualityConstraintAm_.cols(); j++)
			if (fabs(linearEqualityConstraintAm_(i,j)) > Eigen::NumTraits<double>::epsilon()) {
				values.push_back(linearEqualityConstraintAm_(i,j));
				xIndices.push_back(i+1);
				yIndices.push_back(j+1);
			}
	size_t numNonZeroCoeff = values.size()-1;

	glp_load_matrix(lpPtr_, numNonZeroCoeff, xIndices.data(), yIndices.data(), values.data());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GradientDescent::frankWolfeGradient(const size_t& id, const Eigen::VectorXd& parameters, Eigen::VectorXd& gradient) {

	// calculate gradient
	calculateGradient(id, parameters, gradient);
	if (gradient_.rows() != numParameters_)  throw std::runtime_error("size of the gradient vector is not equal to the size of parameters vector.");

	if (numConstraints_==0)  return;

	// set the LP of Frank-Wolfe algorithm cost function
	for (size_t i=0; i<numParameters_; i++)
		glp_set_obj_coef(lpPtr_, i+1, gradient(i));

	// set LP options
	glp_smcp lpOptions;
	glp_init_smcp(&lpOptions);
	if (nlpOptions_.displayGradientDescent_==false)
		lpOptions.msg_lev = GLP_MSG_ERR;

	// solve LP
	glp_simplex(lpPtr_, &lpOptions);

	// get the solution
	Eigen::VectorXd parametersLp(numParameters_);
	for (size_t i=0; i<numParameters_; i++)
		parametersLp(i) = glp_get_col_prim(lpPtr_, i+1);

	// Frank-Wolfe gradient
	if (gradient.dot(parameters - parametersLp) <= 0)  throw std::runtime_error("The Frank-Wolfe gradient is not in the descent direction.");
	gradient = parameters - parametersLp;

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GradientDescent::run(const Eigen::VectorXd& initParameters)  {

	parameters_ = initParameters;
	numParameters_ = initParameters.rows();
	iterationCost_.clear();

	// display
	if (nlpOptions_.displayGradientDescent_) std::cerr << std::endl << "### Initial iteration " << std::endl;

	// initial cost
	id_ = 0;
	calculateCost(id_, parameters_, cost_);
	numFuntionCall_++;
	std::cerr << "\t     parameters:   " << parameters_.transpose().format(CleanFmtDisplay_) << std::endl;
	if (nlpOptions_.displayGradientDescent_) std::cerr << "\t cost:             " << cost_ << std::endl;

	iterationCost_.push_back( (Eigen::VectorXd(1) << cost_).finished() );

	// call user getSolution function
	getSolution(0);

	// linear equality constraint
	calculateLinearEqualityConstraint(linearEqualityConstraintAm_, linearEqualityConstraintBv_);
	if (linearEqualityConstraintAm_.rows() != linearEqualityConstraintBv_.rows())
		throw std::runtime_error("Number of rows in linearEqualityConstraintAm and linearEqualityConstraintBm are not equal.");
	if (linearEqualityConstraintAm_.cols() != numParameters_)
		throw std::runtime_error("Number of columns in linearEqualityConstraintAm is not equal to the size of parameters vector.");

	// number of constraints
	numConstraints_ = linearEqualityConstraintAm_.rows();

	// softening the constraint
	if (numConstraints_>0)
		linearEqualityConstraintBv_ += nlpOptions_.minDisToBoundary_*Eigen::VectorXd::Ones(numConstraints_);

	// setup LP solver
	setupLP();

	size_t iteration = 0;
	bool isCostFunctionConverged = false;
	double relCost = std::numeric_limits<double>::max();
	double learningRateStar;

	while (iteration<nlpOptions_.maxIterations_ && isCostFunctionConverged==false)  {

		// display
		if (nlpOptions_.displayGradientDescent_) std::cerr << std::endl << "### Iteration " << iteration+1 << std::endl;

		double cashedCost = cost_;

		// gradient projection
		frankWolfeGradient(id_, parameters_, gradient_);
		std::cerr << "gradient: " << gradient_.transpose().format(CleanFmtDisplay_) << std::endl;

		// line search
		if (nlpOptions_.useAscendingLineSearchNLP_==true)
			ascendingLineSearch(gradient_, learningRateStar);
		else
			decreasingLineSearch(gradient_, learningRateStar);

		// call user getSolution function
		if (learningRateStar>0)  getSolution(id_);

		// loop variables
		relCost = fabs(cost_-cashedCost);
		isCostFunctionConverged = (learningRateStar==0) || gradient_.isZero() || (relCost<nlpOptions_.minRelCost_);
		iteration++;

		iterationCost_.push_back( (Eigen::VectorXd(1) << cost_).finished() );

		// display
		if (nlpOptions_.displayGradientDescent_) {
			std::cerr << "cost[" << id_ << "]:           " << cost_ << std::endl;
			std::cerr << "learningRateStar:  " << learningRateStar << std::endl;
			std::cerr << "parameters:        " << parameters_.transpose().format(CleanFmtDisplay_) << std::endl;
		}

	}  // end of while loop

	// delete the glpk object
	glp_delete_prob(lpPtr_);

	// display
	if (nlpOptions_.displayGradientDescent_) {
		std::cerr << "\n++++++++++++++++++++++++++++++++++++" << std::endl;
		std::cerr <<   "++++++ NLP Solver is terminated ++++" << std::endl;
		std::cerr <<   "++++++++++++++++++++++++++++++++++++" << std::endl;
		if (isCostFunctionConverged) {
			if (learningRateStar==0)
				std::cerr << "NLP successfully terminates as learningRate reduced to zero." << std::endl;
			else if (gradient_.isZero())
				std::cerr << "NLP successfully terminates as gradient reduced to zero." << std::endl;
			else
				std::cerr << "NLP successfully terminates as cost relative change (relCost=" << relCost <<") reached to the minimum value." << std::endl;
		} else
			std::cerr << "Maximum number of iterations has reached." << std::endl;

		std::cout << "number of function calls:\t" << numFuntionCall_ << std::endl;
	}

}

}  // end of nlp namespace

