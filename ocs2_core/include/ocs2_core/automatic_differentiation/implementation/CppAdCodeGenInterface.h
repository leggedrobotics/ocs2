/*
 * CppAdCodeGenInterface.h
 *
 *  Created on: Apr 23, 2018
 *      Author: farbod
 */

namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T>
CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::CppAdCodeGenInterface(
		const ad_funtion_t& adFunction,
		const range_domain_matrix_t& sparsityPattern)

    : CppAdCodeGenInterface(DOMAIN_DIM, RANGE_DIM, adFunction, sparsityPattern)
{
//	CppadParallelSetting::initParallel(2 + 1);

	if (BASE::domain_dim_<0)
		throw std::runtime_error("Use the overloaded method which takes the domain and range dimensions "
				"since the domain dimension is not defined by the template.");

	if (BASE::range_dim_<0)
		throw std::runtime_error("Use the overloaded method which takes the domain and range dimensions "
				"since the range dimension is not defined by the template.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T>
CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::CppAdCodeGenInterface(
		const int& domainDim,
		const int& rangeDim,
		const ad_funtion_t& adFunction,
		const range_domain_matrix_t& sparsityPattern)

    : BASE(domainDim, rangeDim)
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
    , relatedDependent_(0)
    , hessianWeight_(range_vector_t::Zero(BASE::range_dim_))
{
//	CppadParallelSetting::initParallel(2 + 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T>
CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::CppAdCodeGenInterface(
		const CppAdCodeGenInterface& rhs)

    : CppAdCodeGenInterface(rhs.domain_dim_, rhs.range_dim_, rhs.adFunction_, rhs.sparsityPattern_)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T>
CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>* CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::clone() const {

	return new CppAdCodeGenInterface(adFunction_, sparsityPattern_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::computeForwardModel(
		bool computeForwardModel) {

	computeForwardModel_ = computeForwardModel;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::computeJacobianModel(
		bool computeJacobianModel) {

	computeJacobianModel_ = computeJacobianModel;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::computeHessianModel(
		bool computeHessianModel) {

	computeHessianModel_ = computeHessianModel;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::createModels(
		const std::string& modelName,
		const std::string& libraryFolder /* = "" */,
		const bool verbose /* = true */) {

	if (BASE::domain_dim_<0)
		throw std::runtime_error("Use the overloaded method which takes the domain and range dimensions "
				"since the domain dimension is not defined by the template.");

	if (BASE::range_dim_<0)
		throw std::runtime_error("Use the overloaded method which takes the domain and range dimensions "
				"since the range dimension is not defined by the template.");

	createModels(DOMAIN_DIM, RANGE_DIM, modelName, libraryFolder, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::createModels(
		const int& domainDim,
		const int& rangeDim,
		const std::string& modelName,
		const std::string& libraryFolder /* = "" */,
		const bool verbose /* = true */) {

	BASE::domain_dim_ = domainDim;
	BASE::range_dim_  = rangeDim;

	if (BASE::domain_dim_<0)
		throw std::runtime_error("Use the overloaded method which takes the domain and range dimensions "
				"since the domain dimension is not defined by the template.");

	if (BASE::range_dim_<0)
		throw std::runtime_error("Use the overloaded method which takes the domain and range dimensions "
				"since the range dimension is not defined by the template.");

    //***************************************************************************
    //                               create folder
    //***************************************************************************
	if (libraryFolder.empty()==false)
		CppAD::cg::system::createFolder(libraryFolder);

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
    BASE::extractSparsityPattern(sparsityPattern_,
    		rowsJacobian_, colsJacobian_,
    		rowsHessian_, colsHessian_);

    if (verbose) std::cerr << "Sparsity pattern of " << modelName <<
    		" model: \n" << sparsityPattern_ << std::endl;

    //*************************************************************************
    //                      Create the dynamic library
    //                 (generates and compiles source code)
    //*************************************************************************
    // generates source code
    CppAD::cg::ModelCSourceGen<SCALAR_T> cgen(fun, modelName);

    // set the options
    cgen.setCreateForwardZero(computeForwardModel_);

    if (modelFullDerivatives_==true) {
    	cgen.setCreateJacobian(computeJacobianModel_);

    } else {
    	cgen.setCreateSparseJacobian(computeJacobianModel_);
    	cgen.setCustomSparseJacobianElements(rowsJacobian_, colsJacobian_);
    	cgen.setRelatedDependents(relatedDependent_);
    }

    if (modelFullDerivatives_==true) {
    	cgen.setCreateHessian(computeHessianModel_);

    } else {
    	cgen.setCreateSparseHessian(computeHessianModel_);
    	cgen.setCustomSparseHessianElements(rowsHessian_, colsHessian_);
    	cgen.setRelatedDependents(relatedDependent_);
    }

//    domain_vector_t xtemp = domain_vector_t::Zero();
//    cgen.setTypicalIndependentValues(std::vector<SCALAR_T>(xtemp.data(), xtemp.data()+BASE::domain_dim_));

//    cgen.setCreateReverseOne(true);
//    cgen.setMultiThreading(true);
//    if (cgen.isJacobianMultiThreadingEnabled())
//    	std::cout << ">>>>>>>>>>>>>>>>>>> isJacobianMultiThreadingEnabled \n";
//    if (cgen.isHessianMultiThreadingEnabled())
//        	std::cout << ">>>>>>>>>>>>>>>>>>> isHessianMultiThreadingEnabled \n";

    //
    CppAD::cg::ModelLibraryCSourceGen<SCALAR_T> libcgen(cgen);

    // compile source code
    std::string libraryName;
    if (libraryFolder.empty()==false)
    	libraryName = libraryFolder + "/" + modelName + "_lib";
    else
    	libraryName = modelName + "_lib";

    CppAD::cg::DynamicModelLibraryProcessor<SCALAR_T> p(libcgen, libraryName);
    if (verbose) {
    	std::cerr << "Compiled Shared Library: " << p.getLibraryName() +
    			CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION << std::endl;
    }

    // save to files (not really required)
    CppAD::cg::SaveFilesModelLibraryProcessor<SCALAR_T> p2(libcgen);
    if (libraryFolder.empty()==false)
    	p2.saveSourcesTo(libraryFolder + "/" + modelName + "_sources");
    else
    	p2.saveSourcesTo(modelName + "_sources");

    // compile and load dynamic liberary
    CppAD::cg::GccCompiler<SCALAR_T> compiler;
    dynamicLib_.reset(p.createDynamicLibrary(compiler));

    // get model
    model_.reset(dynamicLib_->model(modelName));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::loadModels(
		const std::string& modelName,
		const std::string& libraryFolder /* = "" */,
		const bool verbose /* = true */) {

	// load dynamic liberary
	std::string libraryName;
	if (libraryFolder.empty()==false)
		libraryName = libraryFolder + "/" + modelName + "_lib";
	else
		libraryName = modelName + "_lib";

	dynamicLib_.reset( new CppAD::cg::LinuxDynamicLib<SCALAR_T>(
			libraryName + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION) );

	// get model
	model_.reset(dynamicLib_->model(modelName));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::getSparsityPattern(
		range_domain_matrix_t& sparsityPattern) const {

	sparsityPattern = sparsityPattern_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T>
bool CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::getFunctionValue(
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
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T>
bool CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::getJacobian(
		const domain_vector_t& x,
		domain_range_matrix_t& jacobian) {

	dynamic_vector_map_t xDynamic(const_cast<SCALAR_T*>(x.data()), BASE::domain_dim_);
	jacobian.resize(BASE::domain_dim_, BASE::range_dim_);
	dynamic_vector_map_t jacobianDynamic(jacobian.data(), BASE::domain_dim_*BASE::range_dim_);

	if (modelFullDerivatives_==true) {
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
template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T>
bool CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::getHessian(
		const domain_vector_t& x,
		domain_matrix_t& hessian,
		size_t outputIndex /*=0*/) {

	dynamic_vector_map_t xDynamic(const_cast<SCALAR_T*>(x.data()), BASE::domain_dim_);
	hessian.resize(BASE::domain_dim_, BASE::domain_dim_);
	dynamic_vector_map_t hessianDynamic(hessian.data(), BASE::domain_dim_*BASE::domain_dim_);

	hessianWeight_.setZero();
	hessianWeight_(outputIndex) = 1.0;
	dynamic_vector_map_t weightDynamic(hessianWeight_.data(), RANGE_DIM);

	if (modelFullDerivatives_==true) {
		if (model_->isHessianAvailable()) {
			model_->Hessian(xDynamic, weightDynamic, hessianDynamic);
			return true;
		}

	} else {
		if (model_->isSparseHessianAvailable()) {
			model_->SparseHessian(xDynamic, weightDynamic, hessianDynamic);
			hessian.template triangularView<Eigen::StrictlyUpper>() = hessian.template triangularView<Eigen::StrictlyLower>().transpose();
			return true;
		}
	}

	return false;
}

} // namespace ocs2
