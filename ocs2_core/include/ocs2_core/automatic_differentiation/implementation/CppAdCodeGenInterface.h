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

namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T, int VARIABLE_DIM>
CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM>::CppAdCodeGenInterface(
		const ad_funtion_t& adFunction,
		const range_domain_matrix_t& sparsityPattern)

    : CppAdCodeGenInterface(DOMAIN_DIM, RANGE_DIM, VARIABLE_DIM, adFunction, sparsityPattern)
{
//	CppadParallelSetting::initParallel(2 + 1);

	if (BASE::domain_dim_<0) {
		throw std::runtime_error("Use the overloaded method which takes the domain and range dimensions "
				"since the domain dimension is not defined by the template.");
	}

	if (BASE::range_dim_<0) {
		throw std::runtime_error("Use the overloaded method which takes the domain and range dimensions "
				"since the range dimension is not defined by the template.");
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T, int VARIABLE_DIM>
CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM>::CppAdCodeGenInterface(
		int domainDim,
		int rangeDim,
		int variableDim,
		const ad_funtion_t& adFunction,
		const range_domain_matrix_t& sparsityPattern)

    : BASE(domainDim, rangeDim, variableDim)
    , adFunction_(adFunction)
    , sparsityPattern_(sparsityPattern)
    , modelFullDerivatives_((sparsityPattern.array()>0).all())
    , computeForwardModel_(false)
    , computeJacobianModel_(false)
    , computeHessianModel_(false)
    , rowsJacobian_(0)
    , colsJacobian_(0)
    , rowsHessian_(0)
    , colsHessian_(0)
    , zeroCountHessian_(domainDim)
    , relatedDependent_(0)
    , compiler_()
    , hessianWeight_(rangeDim)
{
//	CppadParallelSetting::initParallel(2 + 1);

    // Hessian
    domain_vector_t sparsityPatternSum = sparsityPattern.colwise().maxCoeff().transpose();
    size_t zeroCount = 0;
    for (size_t i=0; i<domainDim; i++) {
    	// zero count
    	if (sparsityPatternSum(i)<1e-3) {  ++zeroCount;
}
    	zeroCountHessian_[i] = zeroCount;
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T, int VARIABLE_DIM>
CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM>::CppAdCodeGenInterface(
		const CppAdCodeGenInterface& rhs)

    : CppAdCodeGenInterface(rhs.domain_dim_, rhs.range_dim_, rhs.variable_dim_, rhs.adFunction_, rhs.sparsityPattern_)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T, int VARIABLE_DIM>
CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM>* CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM>::clone() const {

	return new CppAdCodeGenInterface(adFunction_, sparsityPattern_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T, int VARIABLE_DIM>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM>::computeForwardModel(
		bool computeForwardModel) {

	computeForwardModel_ = computeForwardModel;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T, int VARIABLE_DIM>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM>::computeJacobianModel(
		bool computeJacobianModel) {

	computeJacobianModel_ = computeJacobianModel;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T, int VARIABLE_DIM>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM>::computeHessianModel(
		bool computeHessianModel) {

	computeHessianModel_ = computeHessianModel;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T, int VARIABLE_DIM>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM>::createModels(
		const std::string& modelName,
		const std::string& libraryFolder /* = "" */,
		const bool verbose /* = true */,
		bool cgJIT /*= true*/) {

	if (BASE::domain_dim_<0) {
		throw std::runtime_error("Use the overloaded method which takes the domain and range dimensions "
				"since the domain dimension is not defined by the template.");
}

	if (BASE::range_dim_<0) {
		throw std::runtime_error("Use the overloaded method which takes the domain and range dimensions "
				"since the range dimension is not defined by the template.");
}

	createModels(DOMAIN_DIM, RANGE_DIM, modelName, libraryFolder, verbose, cgJIT);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T, int VARIABLE_DIM>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM>::createModels(
		const int& domainDim,
		const int& rangeDim,
		const std::string& modelName,
		const std::string& libraryFolder /* = "" */,
		bool verbose /* = true */,
		bool cgJIT /*= true*/) {

  BASE::domain_dim_ = domainDim;
	BASE::range_dim_  = rangeDim;

	if (BASE::domain_dim_<0) {
		throw std::runtime_error("Use the overloaded method which takes the domain and range dimensions "
				"since the domain dimension is not defined by the template.");
}

	if (BASE::range_dim_<0) {
		throw std::runtime_error("Use the overloaded method which takes the domain and range dimensions "
				"since the range dimension is not defined by the template.");
}

    //***************************************************************************
    //                               create folder
    //***************************************************************************
	if (!libraryFolder.empty()) {
		CppAD::cg::system::createFolder(libraryFolder);
}

	if (!libraryFolder.empty()) {
		CppAD::cg::system::createFolder(libraryFolder + "/" + modelName);
	} else {
		CppAD::cg::system::createFolder(modelName);
	}

    //***************************************************************************
    //                               the model
    //***************************************************************************
    // set and declare independent variables and start tape recording
	ad_dynamic_vector_t x(BASE::domain_dim_);
    x.setZero();
    CppAD::Independent(x);
    // dependent variable vector
    ad_dynamic_vector_t y(BASE::range_dim_);
    // the model equation
    adFunction_(x, y);
    // create f: x -> y and stop tape recording
    ad_fun_t fun(x, y);
    // Optimize the operation sequence
    fun.optimize();

    //**************************************************************************
    //                              Sparsity Pattern
    //**************************************************************************
    BASE::extractSparsityPattern(
    		sparsityPattern_,
    		rowsJacobian_, colsJacobian_,
    		rowsHessian_, colsHessian_);

    if (VARIABLE_DIM != DOMAIN_DIM) {
    	if (zeroCountHessian_.back()+VARIABLE_DIM != DOMAIN_DIM) {
    		throw std::runtime_error("VARIABLE_DIM is incorrect. It should be either " + std::to_string(DOMAIN_DIM) +
    				" or " + std::to_string(DOMAIN_DIM-zeroCountHessian_.back()) + ".");
}
}

    if (verbose) { std::cerr << "Sparsity pattern of " << modelName <<
    		" model: \n" << sparsityPattern_ << std::endl;
}
    //*************************************************************************
    //                      Create the dynamic library
    //                 (generates and compiles source code)
    //*************************************************************************
    // generates source code
    CppAD::cg::ModelCSourceGen<SCALAR_T> cgen(fun, modelName);

    // set the options
    cgen.setCreateForwardZero(computeForwardModel_);

    if (modelFullDerivatives_) {
    	cgen.setCreateJacobian(computeJacobianModel_);

    } else {
    	cgen.setCreateSparseJacobian(computeJacobianModel_);
    	cgen.setCustomSparseJacobianElements(rowsJacobian_, colsJacobian_);
    	cgen.setRelatedDependents(relatedDependent_);
    }

    if (modelFullDerivatives_) {
    	cgen.setCreateHessian(computeHessianModel_);

    } else {
    	cgen.setCreateSparseHessian(computeHessianModel_);
    	cgen.setCustomSparseHessianElements(rowsHessian_, colsHessian_);
    	cgen.setRelatedDependents(relatedDependent_);
    }
    //
    CppAD::cg::ModelLibraryCSourceGen<SCALAR_T> libcgen(cgen);

    // save to files (not really required)
    CppAD::cg::SaveFilesModelLibraryProcessor<SCALAR_T> p2(libcgen);
    if (!libraryFolder.empty()) {
    	p2.saveSourcesTo(libraryFolder + "/" + modelName + "/cppad_generated");
    } else {
    	p2.saveSourcesTo(modelName + "/cppad_generated");
    }
    if (cgJIT) {
    	// compile source code
    	std::string libraryName;
    	if (!libraryFolder.empty()) {
    		libraryName = libraryFolder + "/" + modelName + "/cppad_generated/" + modelName + "_lib";
    	} else {
    		libraryName = modelName + "_lib";
}
    	CppAD::cg::DynamicModelLibraryProcessor<SCALAR_T> p(libcgen, libraryName);
    	if (verbose) {
    		std::cerr << "Compiled Shared Library: " << p.getLibraryName() +
    				CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION << std::endl;
    	}

    	// Add random string to tmp folder to avoid race condition on the temporary objects
    	int randomFromClock = std::chrono::high_resolution_clock::now().time_since_epoch().count() % 1000;
    	std::string tmpFolder = std::string("cppadcg_tmp") + std::to_string(randomFromClock) + std::to_string(getpid());
    	compiler_.setTemporaryFolder(tmpFolder);

    	dynamicLib_.reset(p.createDynamicLibrary(compiler_));
    	// get model
    	model_.reset(dynamicLib_->model(modelName));

    } else {
    	// TODO:
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T, int VARIABLE_DIM>
bool CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM>::loadModels(
		const std::string& modelName,
		const std::string& libraryFolder /* = "" */,
		bool verbose /* = true */) {

	// load dynamic library
	std::string libraryName;
	if (!libraryFolder.empty()) {
		libraryName = libraryFolder + "/" + modelName + "/cppad_generated/" + modelName + "_lib";
	} else {
		libraryName = modelName + "_lib";
}

	dynamicLib_.reset( new CppAD::cg::LinuxDynamicLib<SCALAR_T>(
			libraryName + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION) );

	// get model
	if (dynamicLib_) {
		model_.reset(dynamicLib_->model(modelName));
		return true;
	} else {
		return false;
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T, int VARIABLE_DIM>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM>::getSparsityPattern(
		range_domain_matrix_t& sparsityPattern) const {

	sparsityPattern = sparsityPattern_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T, int VARIABLE_DIM>
bool CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM>::getFunctionValue(
		const domain_vector_t& x,
		range_vector_t& funcValue) {

	dynamic_vector_map_t xDynamic(const_cast<SCALAR_T*>(x.data()), BASE::domain_dim_);
	funcValue.resize(BASE::range_dim_);
	dynamic_vector_map_t funcValueDynamic(funcValue.data(), BASE::range_dim_);

	if (model_->isForwardZeroAvailable()) {

		model_->ForwardZero(xDynamic, funcValueDynamic);

		return true;

	} else {
		return false;
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T, int VARIABLE_DIM>
bool CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM>::getJacobian(
		const domain_vector_t& x,
		domain_range_matrix_t& jacobian) {

	dynamic_vector_map_t xDynamic(const_cast<SCALAR_T*>(x.data()), BASE::domain_dim_);
	jacobian.resize(BASE::domain_dim_, BASE::range_dim_);
	dynamic_vector_map_t jacobianDynamic(jacobian.data(), BASE::domain_dim_*BASE::range_dim_);

	if (modelFullDerivatives_) {
		if(model_->isJacobianAvailable()) {
			model_->Jacobian(xDynamic, jacobianDynamic);
			return true;
		}

	} else {
		if(model_->isSparseJacobianAvailable()) {
			model_->SparseJacobian(xDynamic, jacobianDynamic);
			return true;
		}
	}

	return false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T, int VARIABLE_DIM>
bool CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM>::getHessian(
		const domain_vector_t& x,
		variable_matrix_t& hessian,
		size_t outputIndex /*=0*/) {

	hessianWeight_ = std::vector<SCALAR_T>(BASE::range_dim_, 0.0);
	hessianWeight_[outputIndex] = 1.0;

	hessian.resize(BASE::variable_dim_, BASE::variable_dim_);

	if (modelFullDerivatives_) {
		if (model_->isHessianAvailable()) {

			dynamic_vector_map_t xDynamic(const_cast<SCALAR_T*>(x.data()), BASE::domain_dim_);
			dynamic_vector_map_t weightDynamic(hessianWeight_.data(), BASE::range_dim_);
			dynamic_vector_map_t hessianDynamic(hessian.data(), hessian.size());

			model_->Hessian(xDynamic, weightDynamic, hessianDynamic);
			return true;
		}

	} else {
		if (model_->isSparseHessianAvailable()) {
//			dynamic_vector_map_t xDynamic(const_cast<SCALAR_T*>(x.data()), BASE::domain_dim_);
//			dynamic_vector_map_t weightDynamic(hessianWeight_.data(), BASE::range_dim_);
//			dynamic_vector_map_t hessianDynamic(hessian.data(), hessian.size());
//			model_->SparseHessian(xDynamic.data(), xDynamic.size(), weightDynamic.data(), weightDynamic.size(),
//					hessianDynamic.data(), hessianDynamic.size());
//			hessian.template triangularView<Eigen::StrictlyUpper>() = hessian.template triangularView<Eigen::StrictlyLower>().transpose();

			std::vector<SCALAR_T> xVec(const_cast<SCALAR_T*>(x.data()), const_cast<SCALAR_T*>(x.data())+BASE::domain_dim_);
			std::vector<SCALAR_T> hessianVec;

			std::vector<size_t> row;
			std::vector<size_t> col;
			model_->SparseHessian(
					const_cast<SCALAR_T*>(x.data()), BASE::domain_dim_,
					hessianWeight_.data(), BASE::range_dim_,
					hessianVec, row, col);

			hessian.setZero();
			for (size_t i=0; i<hessianVec.size(); i++) {

				size_t rowIndex = row[i]-zeroCountHessian_[row[i]];
				size_t colIndex = col[i]-zeroCountHessian_[col[i]];

				// use compact format
				if (zeroCountHessian_.back()+VARIABLE_DIM == DOMAIN_DIM) {
					rowIndex = row[i]-zeroCountHessian_[row[i]];
					colIndex = col[i]-zeroCountHessian_[col[i]];
				} else {
					rowIndex = row[i];
					colIndex = col[i];
				}

				hessian(rowIndex, colIndex) = hessianVec[i];
			}
			hessian.template triangularView<Eigen::StrictlyLower>() = hessian.template triangularView<Eigen::StrictlyUpper>().transpose();

			return true;
		}
	}

	return false;
}

} // namespace ocs2
