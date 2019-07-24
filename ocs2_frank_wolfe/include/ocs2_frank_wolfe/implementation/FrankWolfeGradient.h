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
FrankWolfeGradient::FrankWolfeGradient()
	: lpPtr_(glp_create_prob(), glp_delete_prob)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FrankWolfeGradient::instantiateGLPK() {

	// erase the solver
	glp_erase_prob(lpPtr_.get());

	// name
	glp_set_prob_name(lpPtr_.get(), "FrankWolfe");

	// set it as a minimization problem
	glp_set_obj_dir(lpPtr_.get(), GLP_MIN);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FrankWolfeGradient::setupLP(
		const scalar_array_t& parameter,
		const dynamic_vector_t& gradient) {

	const size_t parameterDim = parameter.size();

	// return if there is no parameter
	if (parameterDim == 0)
		return;

	if (gradient.size() != parameterDim)
		throw std::runtime_error("The gradient vector size is in compatible with parameter size.");

	// instantiate GLPK
	instantiateGLPK();

	// set parameters limits
	glp_add_cols(lpPtr_.get(), parameterDim);
	for (size_t i=0; i<parameterDim; i++) {
		if (std::fabs(gradient(i)) > Eigen::NumTraits<scalar_t>::epsilon()) {
			if (gradient(i)>0)
				glp_set_col_bnds(lpPtr_.get(), i+1, GLP_LO, parameter[i]-gradient(i), 0.0);
			else
				glp_set_col_bnds(lpPtr_.get(), i+1, GLP_UP, 0.0, parameter[i]-gradient(i));
		} else {
			glp_set_col_bnds(lpPtr_.get(), i+1, GLP_FX, parameter[i], parameter[i]);
		}
	}  // end of i loop

	//

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
		for (size_t j=0; j<parameterDim; j++)
			if (fabs(Cm(i,j)) > Eigen::NumTraits<scalar_t>::epsilon()) {
				values.push_back(Cm(i,j));
				xIndices.push_back(i+1);
				yIndices.push_back(j+1);
			}
	size_t numNonZeroCoeff = values.size()-1;

	glp_load_matrix(lpPtr_.get(), numNonZeroCoeff, xIndices.data(), yIndices.data(), values.data());

}


} // namespace ocs2
