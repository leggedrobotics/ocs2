/*
 * Implementation of GradientDescent.h
 *
 *  Created on: Jul 26, 2016
 *      Author: farbod
 */


namespace ocs2 {
namespace nlp {


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
GradientDescent<SCALAR_T>::GradientDescent() {

	adjustOptions();
	CleanFmtDisplay_ = Eigen::IOFormat(3, 0, ", ", "\n", "[", "]");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void GradientDescent<SCALAR_T>::adjustOptions() {

	numLineSearch_ =
			floor(log2(nlpSettings_.maxLearningRate_/nlpSettings_.minLearningRate_)) + 1;
	nlpSettings_.minLearningRate_ =
			(nlpSettings_.maxLearningRate_- 2*std::numeric_limits<SCALAR_T>::epsilon()) / pow(2,numLineSearch_-1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void GradientDescent<SCALAR_T>::getCost(SCALAR_T& cost) {

	cost = cost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
template <typename Derived>
void GradientDescent<SCALAR_T>::getParameters(
		Eigen::MatrixBase<Derived>& parameters) const {

	parameters = parameters_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void GradientDescent<SCALAR_T>::calculateLinearEqualityConstraint(
		dynamic_matrix_t& Am,
		dynamic_vector_t& Bv)  {

	Am.resize(0,numParameters_);
	Bv.resize(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void GradientDescent<SCALAR_T>::getIterationsLog(
		eigen_scalar_array_t& iterationCost) const {

	iterationCost = iterationCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
bool GradientDescent<SCALAR_T>::calculateNumericalGradient(
		const size_t& id,
		const dynamic_vector_t& parameters,
		dynamic_vector_t& gradient) {

	bool status;
	SCALAR_T cost;
	status = calculateCost(id, parameters, cost);
	numFuntionCall_++;
	gradient.resize(parameters.rows());
	for (size_t i=0; i<parameters.rows(); i++)  {
		SCALAR_T h = sqrt(std::numeric_limits<SCALAR_T>::epsilon()) * std::max(parameters(i), 1.0);
		dynamic_vector_t parametersPlus = parameters;
		parametersPlus(i) += h;
		SCALAR_T costPlus;
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
template <typename SCALAR_T>
void GradientDescent<SCALAR_T>::ascendingLineSearch(
		const dynamic_vector_t& gradient,
		SCALAR_T& learningRateStar) {

	learningRateStar = 0.0;
	id_ = 0;
	const dynamic_vector_t initParameters = parameters_;
	SCALAR_T learningRate = nlpSettings_.minLearningRate_;
	size_t id = 1;

	while (learningRate<=nlpSettings_.maxLearningRate_)  {

		// lineSerach parameter
		dynamic_vector_t lsParameters = initParameters - learningRate*gradient;

		// display
		if (nlpSettings_.displayGradientDescent_) {
			SCALAR_T lsMaxConstriant = 0.0;
			if (linearEqualityConstraintBv_.rows()>0)
				lsMaxConstriant = (linearEqualityConstraintAm_*lsParameters + linearEqualityConstraintBv_).maxCoeff() - nlpSettings_.minDisToBoundary_;
			std::cerr << "\t [" << id <<"] learningRate: " << learningRate << "\t max NLP constraint: " << lsMaxConstriant << std::endl;
			std::cerr << "\t     parameters:   " << lsParameters.transpose().format(CleanFmtDisplay_) << std::endl;
		}

		// calculate cost function
		SCALAR_T lsCost;
		bool status = calculateCost(id, lsParameters, lsCost);

		// increment the number of function calls
		numFuntionCall_++;

		// skip it if status is not OK
		if (status==false) {
			if (nlpSettings_.displayGradientDescent_)  std::cerr << "\t     cost:         " << lsCost << " (rejected)" << std::endl;
			learningRate *= 2.0;
			id++;
			continue;
		}

		// display
		if (nlpSettings_.displayGradientDescent_)  std::cerr << "\t     cost:         " << lsCost << std::endl;

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
template <typename SCALAR_T>
void GradientDescent<SCALAR_T>::decreasingLineSearch(
		const dynamic_vector_t& gradient,
		SCALAR_T& learningRateStar) {

	learningRateStar = 0.0;
	id_ = 0;
	const dynamic_vector_t initParameters = parameters_;
	SCALAR_T learningRate = nlpSettings_.maxLearningRate_;

	size_t id = numLineSearch_;

	while (learningRate>=nlpSettings_.minLearningRate_)  {

		// lineSerach parameter
		dynamic_vector_t lsParameters = initParameters - learningRate*gradient;

		// display
		if (nlpSettings_.displayGradientDescent_) {
			SCALAR_T lsMaxConstriant = 0.0;
			if (linearEqualityConstraintBv_.rows()>0)
				lsMaxConstriant = (linearEqualityConstraintAm_*lsParameters + linearEqualityConstraintBv_).maxCoeff() - nlpSettings_.minDisToBoundary_;
			std::cerr << "\t [" << id <<"] learningRate: " << learningRate << "\t maxNlpConstraint: " << lsMaxConstriant << std::endl;
			std::cerr << "\t     parameters:   " << lsParameters.transpose().format(CleanFmtDisplay_) << std::endl;
		}

		// calculate cost function
		SCALAR_T lsCost;
		bool status = calculateCost(id, lsParameters, lsCost);

		// increment the number of function calls
		numFuntionCall_++;

		// skip it if status is not OK
		if (status==false) {
			if (nlpSettings_.displayGradientDescent_)  std::cerr << "\t     cost:         " << lsCost << " (rejected)" << std::endl;
			learningRate /= 2.0;
			id--;
			continue;
		}

		if (nlpSettings_.displayGradientDescent_)  std::cerr << "\t     cost:         " << lsCost << std::endl;

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
template <typename SCALAR_T>
void GradientDescent<SCALAR_T>::setupLP() {

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
	std::vector<SCALAR_T> values;
	std::vector<int> xIndices, yIndices;
	values.push_back(0.0);   // the 0 index is not used
	xIndices.push_back(-1);  // the 0 index is not used
	yIndices.push_back(-1);  // the 0 index is not used
	for (size_t i=0; i<linearEqualityConstraintAm_.rows(); i++)
		for (size_t j=0; j<linearEqualityConstraintAm_.cols(); j++)
			if (fabs(linearEqualityConstraintAm_(i,j)) > Eigen::NumTraits<SCALAR_T>::epsilon()) {
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
template <typename SCALAR_T>
void GradientDescent<SCALAR_T>::frankWolfeGradient(
		const size_t& id,
		const dynamic_vector_t& parameters,
		dynamic_vector_t& gradient) {

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
	if (nlpSettings_.displayGradientDescent_==false)
		lpOptions.msg_lev = GLP_MSG_ERR;

	// solve LP
	glp_simplex(lpPtr_, &lpOptions);

	// get the solution
	dynamic_vector_t parametersLp(numParameters_);
	for (size_t i=0; i<numParameters_; i++)
		parametersLp(i) = glp_get_col_prim(lpPtr_, i+1);

	// Frank-Wolfe gradient
	if (gradient.dot(parameters - parametersLp) <= 0)  throw std::runtime_error("The Frank-Wolfe gradient is not in the descent direction.");
	gradient = parameters - parametersLp;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
bool GradientDescent<SCALAR_T>::calculateGradient(
		const size_t& id,
		const dynamic_vector_t& parameters,
		dynamic_vector_t& gradient) {

	return calculateNumericalGradient(id, parameters, gradient);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void GradientDescent<SCALAR_T>::run(const dynamic_vector_t& initParameters)  {

	parameters_ = initParameters;
	numParameters_ = initParameters.rows();
	iterationCost_.clear();

	// display
	if (nlpSettings_.displayGradientDescent_) std::cerr << std::endl << "### Initial iteration " << std::endl;

	// initial cost
	id_ = 0;
	calculateCost(id_, parameters_, cost_);
	numFuntionCall_++;
	std::cerr << "\t     parameters:   " << parameters_.transpose().format(CleanFmtDisplay_) << std::endl;
	if (nlpSettings_.displayGradientDescent_) std::cerr << "\t cost:             " << cost_ << std::endl;

	iterationCost_.push_back( (dynamic_vector_t(1) << cost_).finished() );

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
		linearEqualityConstraintBv_ += nlpSettings_.minDisToBoundary_*dynamic_vector_t::Ones(numConstraints_);

	// setup LP solver
	setupLP();

	size_t iteration = 0;
	bool isCostFunctionConverged = false;
	SCALAR_T relCost = std::numeric_limits<SCALAR_T>::max();
	SCALAR_T learningRateStar;

	while (iteration<nlpSettings_.maxIterations_ && isCostFunctionConverged==false)  {

		// display
		if (nlpSettings_.displayGradientDescent_) std::cerr << std::endl << "### Iteration " << iteration+1 << std::endl;

		SCALAR_T cashedCost = cost_;

		// gradient projection
		frankWolfeGradient(id_, parameters_, gradient_);
		std::cerr << "gradient: " << gradient_.transpose().format(CleanFmtDisplay_) << std::endl;

		// line search
		if (nlpSettings_.useAscendingLineSearchNLP_==true)
			ascendingLineSearch(gradient_, learningRateStar);
		else
			decreasingLineSearch(gradient_, learningRateStar);

		// call user getSolution function
		if (learningRateStar>0)  getSolution(id_);

		// loop variables
		relCost = fabs(cost_-cashedCost);
		isCostFunctionConverged = (learningRateStar==0) || gradient_.isZero() || (relCost<nlpSettings_.minRelCost_);
		iteration++;

		iterationCost_.push_back( (dynamic_vector_t(1) << cost_).finished() );

		// display
		if (nlpSettings_.displayGradientDescent_) {
			std::cerr << "cost[" << id_ << "]:           " << cost_ << std::endl;
			std::cerr << "learningRateStar:  " << learningRateStar << std::endl;
			std::cerr << "parameters:        " << parameters_.transpose().format(CleanFmtDisplay_) << std::endl;
		}

	}  // end of while loop

	// delete the glpk object
	glp_delete_prob(lpPtr_);

	// display
	if (nlpSettings_.displayGradientDescent_) {
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

		std::cerr << "number of function calls:\t" << numFuntionCall_ << std::endl;
	}
}

}  // end of nlp namespace
}  // end of ocs2 namespace

