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
template <size_t DOMAIN_DIM, size_t RANGE_DIM, typename SCALAR_T>
CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::CppAdCodeGenInterface(
			const ad_funtion_t& adFunction,
			const range_doamin_matrix_t& sparsityPattern)

    : BASE()
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
    , hessianWeight_(range_vector_t::Zero())
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t DOMAIN_DIM, size_t RANGE_DIM, typename SCALAR_T>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::computeForwardModel(
		bool computeForwardModel) {

	computeForwardModel_ = computeForwardModel;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t DOMAIN_DIM, size_t RANGE_DIM, typename SCALAR_T>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::computeJacobianModel(
		bool computeJacobianModel) {

	computeJacobianModel_ = computeJacobianModel;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t DOMAIN_DIM, size_t RANGE_DIM, typename SCALAR_T>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::computeHessianModel(
		bool computeHessianModel) {

	computeHessianModel_ = computeHessianModel;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t DOMAIN_DIM, size_t RANGE_DIM, typename SCALAR_T>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::createModels(
		const std::string& modelName,
		const std::string& libraryFolder /* = "" */,
		const bool verbose /* = true */) {

    //***************************************************************************
    //                               create folder
    //***************************************************************************
	if (libraryFolder.empty()==false)
		CppAD::cg::system::createFolder(libraryFolder);

    //***************************************************************************
    //                               the model
    //***************************************************************************

    // set and declare independent variables and start tape recording
	ad_dynamic_vector_t x(DOMAIN_DIM);
    x.setZero();
    CppAD::Independent(x);

    // dependent variable vector
    ad_dynamic_vector_t y(RANGE_DIM);

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

    if (verbose) std::cerr << "sparsityPattern: \n" << sparsityPattern_ << std::endl;

    //*************************************************************************
    //                      Create the dynamic library
    //                 (generates and compiles source code)
    //*************************************************************************
    // generates source code
    CppAD::cg::ModelCSourceGen<SCALAR_T> cgen(fun, "modelName");

    // set the options
    cgen.setCreateForwardZero(computeForwardModel_);

    if (modelFullDerivatives_==true) {
    	cgen.setCreateJacobian(computeJacobianModel_);

    } else {
    	cgen.setCreateSparseJacobian(computeJacobianModel_);
    	cgen.setCustomSparseJacobianElements(rowsJacobian_, colsJacobian_);
    }

    if (modelFullDerivatives_==true) {
    	cgen.setCreateHessian(computeHessianModel_);

    } else {
    	cgen.setCreateSparseHessian(computeHessianModel_);
    	cgen.setCustomSparseHessianElements(rowsHessian_, colsHessian_);
    }

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
    model_.reset(dynamicLib_->model("modelName"));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t DOMAIN_DIM, size_t RANGE_DIM, typename SCALAR_T>
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
	model_.reset(dynamicLib_->model("modelName"));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t DOMAIN_DIM, size_t RANGE_DIM, typename SCALAR_T>
void CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::getSparsityPattern(
		range_doamin_matrix_t& sparsityPattern) const {

	sparsityPattern = sparsityPattern_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t DOMAIN_DIM, size_t RANGE_DIM, typename SCALAR_T>
bool CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::getFunctionValue(
		const domain_vector_t& x,
		range_vector_t& funcValue) {

	dynamic_vector_map_t xDynamic(const_cast<SCALAR_T*>(x.data()), DOMAIN_DIM);
	dynamic_vector_map_t funcValueDynamic(funcValue.data(), RANGE_DIM);

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
template <size_t DOMAIN_DIM, size_t RANGE_DIM, typename SCALAR_T>
bool CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::getJacobian(
		const domain_vector_t& x,
		doamin_range_matrix_t& jacobian) {

	dynamic_vector_map_t xDynamic(const_cast<SCALAR_T*>(x.data()), DOMAIN_DIM);
	dynamic_vector_map_t jacobianDynamic(jacobian.data(), DOMAIN_DIM*RANGE_DIM);

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
template <size_t DOMAIN_DIM, size_t RANGE_DIM, typename SCALAR_T>
bool CppAdCodeGenInterface<DOMAIN_DIM, RANGE_DIM, SCALAR_T>::getHessian(
		const domain_vector_t& x,
		domain_matrix_t& hessian,
		size_t outputIndex /*=0*/) {

	dynamic_vector_map_t xDynamic(const_cast<SCALAR_T*>(x.data()), DOMAIN_DIM);
	dynamic_vector_map_t hessianDynamic(hessian.data(), DOMAIN_DIM*DOMAIN_DIM);

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
