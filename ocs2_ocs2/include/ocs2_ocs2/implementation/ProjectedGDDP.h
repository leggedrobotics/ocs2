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

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
ProjectedGDDP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::ProjectedGDDP(
		const SLQ_Settings& settings /*= SLQ_Settings()*/)
	: BASE(settings)
{
	// create a LP object
//	lpPtr_ = glp_create_prob();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ProjectedGDDP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::eventTimesConstraint(
		const scalar_array_t& eventTimes,
		const size_t& activeEventTimeBeginIndex,
		const size_t& activeEventTimeEndIndex,
		const scalar_t& startTime,
		const scalar_t& finalTime,
		dynamic_matrix_t& Cm,
		dynamic_vector_t& Dv) const {

	const size_t numActiveEventTimes = activeEventTimeEndIndex-activeEventTimeBeginIndex;

	// return if there is no event times
	if (numActiveEventTimes==0) return;

	Cm = dynamic_matrix_t::Zero(numActiveEventTimes+1, eventTimes.size());
	for (size_t i=activeEventTimeBeginIndex; i<activeEventTimeEndIndex+1; i++) {

		if (i < activeEventTimeEndIndex)
			Cm(i-activeEventTimeBeginIndex,i)  = -1.0;

		if (i > activeEventTimeBeginIndex)
			Cm(i-activeEventTimeBeginIndex,i-1) = 1.0;
	}

	Dv = dynamic_vector_t::Zero(numActiveEventTimes+1);
	Dv(0) = startTime;
	Dv(numActiveEventTimes) = -finalTime;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ProjectedGDDP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setupGLPK() {

	// create the solver
	lpPtr_.reset(glp_create_prob());

	// name
	glp_set_prob_name(lpPtr_.get(), "FrankWolfe");

	// set it as a minimization problem
	glp_set_obj_dir(lpPtr_.get(), GLP_MIN);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ProjectedGDDP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setupLP(
		const scalar_array_t& eventTimes,
		const dynamic_vector_t& gradient,
		const size_t& activeEventTimeBeginIndex,
		const size_t& activeEventTimeEndIndex,
		const dynamic_matrix_t& Cm,
		const dynamic_vector_t& Dv) {

	const size_t numEventTimes = Cm.cols();
	const size_t numActiveConstraints = Cm.rows();

	// return if there is no event times
	if (numEventTimes==0) return;

	// setup GLPK
	setupGLPK();

	// set parameters limits
	glp_add_cols(lpPtr_.get(), numEventTimes);
	for (size_t i=0; i<numEventTimes; i++) {
		if (activeEventTimeBeginIndex<=i && i<activeEventTimeEndIndex) {
			if (gradient(i)>0)
				glp_set_col_bnds(lpPtr_.get(), i+1, GLP_LO, eventTimes[i]-gradient(i), 0.0);
			else
				glp_set_col_bnds(lpPtr_.get(), i+1, GLP_UP, 0.0, eventTimes[i]-gradient(i));
		} else {
			glp_set_col_bnds(lpPtr_.get(), i+1, GLP_FX, eventTimes[i], eventTimes[i]);
		}
	}  // end of i loop

	// set the constraint limits (the other half is set later )
	glp_add_rows(lpPtr_.get(), numActiveConstraints);
	for (size_t i=0; i<numActiveConstraints; i++)
		glp_set_row_bnds(lpPtr_.get(), i+1, GLP_UP, 0.0, -Dv(i));

	// set the constraint coefficients
	scalar_array_t values;
	std::vector<int> xIndices, yIndices;
	values.push_back(0.0);   // the 0 index is not used
	xIndices.push_back(-1);  // the 0 index is not used
	yIndices.push_back(-1);  // the 0 index is not used
	for (size_t i=0; i<numActiveConstraints; i++)
		for (size_t j=0; j<numEventTimes; j++)
			if (fabs(Cm(i,j)) > Eigen::NumTraits<scalar_t>::epsilon()) {
				values.push_back(Cm(i,j));
				xIndices.push_back(i+1);
				yIndices.push_back(j+1);
			}
	size_t numNonZeroCoeff = values.size()-1;

	glp_load_matrix(lpPtr_.get(), numNonZeroCoeff, xIndices.data(), yIndices.data(), values.data());
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ProjectedGDDP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::frankWolfeProblem(
		const dynamic_vector_t& gradient,
		scalar_array_t& eventTimesOptimized) {

	// return if no constraints are defined
	if (gradient.size()==0)  return;

	// set the LP of Frank-Wolfe algorithm cost function
	for (size_t i=0; i<gradient.size(); i++)
		glp_set_obj_coef(lpPtr_.get(), i+1, gradient(i));

	// set LP options
	glp_smcp lpOptions;
	glp_init_smcp(&lpOptions);
	if (BASE::settings_.displayGradientDescent_ == false)
		lpOptions.msg_lev = GLP_MSG_ERR;

	// solve LP
	glp_simplex(lpPtr_.get(), &lpOptions);

	// get the solution
	eventTimesOptimized.resize(gradient.size());
	for (size_t i=0; i<gradient.size(); i++)
		eventTimesOptimized[i] = glp_get_col_prim(lpPtr_.get(), i+1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ProjectedGDDP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::run(
		scalar_array_t eventTimes,
		const slq_data_collector_t* dcPtr,
		scalar_array_t& eventTimesOptimized,
		const scalar_t& maxStepSize /*= 1.0*/)  {

	// run the base method which will calculate the raw gradient
	BASE::run(eventTimes, dcPtr);

	dynamic_vector_t gradient = maxStepSize * BASE::nominalCostFuntionDerivative_.normalized();

	// Event time constraints
	dynamic_matrix_t Cm;
	dynamic_vector_t Dv;
	eventTimesConstraint(eventTimes,
			BASE::activeEventTimeBeginIndex_, BASE::activeEventTimeEndIndex_,
			dcPtr->initTime_, dcPtr->finalTime_,
			Cm, Dv);

	// setup LP problem
	setupLP(eventTimes, gradient,
			BASE::activeEventTimeBeginIndex_, BASE::activeEventTimeEndIndex_,
			Cm, Dv);

	// Frank-Wolfe solution
	frankWolfeProblem(gradient, eventTimesOptimized);
}

} // namespace ocs2

