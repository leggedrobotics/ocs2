/*
 * BVPEquations.h
 *
 *  Created on: Jun 18, 2016
 *      Author: farbod
 */

#ifndef BVPEQUATIONS_OCS2_H_
#define BVPEQUATIONS_OCS2_H_

#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <ocs2_core/integration/ODE_Base.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2{

/**
 * This class contains the general BVP equations.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class BVPEquations : public ODE_Base<STATE_DIM*STATE_DIM+STATE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum {
		/** If STATE_DIM=n, Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv and +1 one from a scalar */
		Full_ODE_VECTOR_DIM = STATE_DIM*STATE_DIM+STATE_DIM
	};

	typedef Eigen::Matrix<double, Full_ODE_VECTOR_DIM, 1> full_ode_vector_t;
	typedef std::vector<full_ode_vector_t, Eigen::aligned_allocator<full_ode_vector_t> > full_ode_vector_array_t;

	typedef Eigen::Matrix<double, 1, 1> eigen_scalar_t;
	typedef Eigen::Matrix<double, STATE_DIM, 1> state_vector_t;
	typedef Eigen::Matrix<double, INPUT_DIM, 1> input_vector_t;
	typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> state_state_matrix_t;
	typedef Eigen::Matrix<double, INPUT_DIM, INPUT_DIM> input_input_matrix_t;
	typedef Eigen::Matrix<double, INPUT_DIM, STATE_DIM> input_state_matrix_t;
	typedef Eigen::Matrix<double, STATE_DIM, INPUT_DIM> state_input_matrix_t;

	typedef std::vector<double> double_array_t;
	typedef std::vector<eigen_scalar_t, Eigen::aligned_allocator<eigen_scalar_t> > eigen_scalar_array_t;
	typedef std::vector<state_vector_t, Eigen::aligned_allocator<state_vector_t> > state_vector_array_t;
	typedef std::vector<input_vector_t, Eigen::aligned_allocator<input_vector_t> > input_vector_array_t;
	typedef std::vector<state_state_matrix_t, Eigen::aligned_allocator<state_state_matrix_t> > state_state_matrix_array_t;
	typedef std::vector<input_input_matrix_t, Eigen::aligned_allocator<input_input_matrix_t> > input_input_matrix_array_t;
	typedef std::vector<input_state_matrix_t, Eigen::aligned_allocator<input_state_matrix_t> > input_state_matrix_array_t;
	typedef std::vector<state_input_matrix_t, Eigen::aligned_allocator<state_input_matrix_t> > state_input_matrix_array_t;

	/**
	 * Default constructor.
	 */
	BVPEquations(const bool& useMakePSD)
	: useMakePSD_(useMakePSD)
	{}

	/**
	 * Default destructor.
	 */
	~BVPEquations() = default;

	/**
	 * Transcribe symmetric matrix Mm and vector Sv into a single vector.
	 *
     * @param [in] Mm: \f$ M_m \f$
     * @param [in] Sv: \f$ S_v \f$
     * @param [out] MSv: Single vector constructed by concatenating Mm and Sv.
     */
	static void convert2Vector(const state_state_matrix_t& Mm, const state_vector_t& Sv, full_ode_vector_t& MSv)  {

		MSv << Eigen::Map<const Eigen::VectorXd>(Mm.data(),STATE_DIM*STATE_DIM),
				Eigen::Map<const Eigen::VectorXd>(Sv.data(),STATE_DIM);
	}

	/**
	 * Transcribes the stacked vector allSs into a symmetric matrix, Mm and a vector, Sv.
	 *
     * @param [in] MSv: Single vector constructed by concatenating Mm and Sv.
     * @param [out] Mm: \f$ M_m \f$
     * @param [out] Sv: \f$ S_v \f$
     */
	static void convert2Matrix(const full_ode_vector_t& MSv, state_state_matrix_t& Mm, state_vector_t& Sv)  {

		Mm = Eigen::Map<const state_state_matrix_t>(MSv.data(),STATE_DIM,STATE_DIM);
		Sv = Eigen::Map<const state_vector_t>(MSv.data()+STATE_DIM*STATE_DIM, STATE_DIM);
	}

    /**
     * Sets coefficients of the model.
     *
 	 * @param [in] timeStampPtr: A pointer to the time stamp trajectory.
	 * @param [in] AmPtr: A pointer to the trajectory of \f$ A_m(t) \f$ .
     * @param [in] OmPtr: A pointer to the trajectory of \f$ O_m(t) \f$ .
	 * @param [in] BmPtr: A pointer to the trajectory of \f$ B_m(t) \f$ .
     * @param [in] GvPtr: A pointer to the trajectory of \f$ G_v(t) \f$ .
 	 * @param [in] QvPtr: A pointer to the trajectory of \f$ Q_v(t) \f$ .
	 * @param [in] QmPtr: A pointer to the trajectory of \f$ Q_m(t) \f$ .
	 * @param [in] PmPtr: A pointer to the trajectory of \f$ P_m(t) \f$ .
	 * @param [in] RvPtr: A pointer to the trajectory of \f$ R_v(t) \f$ .
 	 * @param [in] RmPtr: A pointer to the trajectory of \f$ R_m(t) \f$ .
     * @param [in] RmInversePtr: A pointer to the trajectory of \f$ R_m^{-1}(t) \f$ .
     */
	void setData(const double_array_t* timeStampPtr,
			const state_state_matrix_array_t* AmPtr, const state_state_matrix_array_t* OmPtr, const state_input_matrix_array_t* BmPtr, const state_vector_array_t* GvPtr,
			const state_vector_array_t* QvPtr, const state_state_matrix_array_t* QmPtr, const input_state_matrix_array_t* PmPtr,
			const input_vector_array_t* RvPtr, const input_input_matrix_array_t* RmPtr, const input_input_matrix_array_t* RmInversePtr)  {
		AmFunc_.setTimeStamp(timeStampPtr);
		OmFunc_.setTimeStamp(timeStampPtr);
		BmFunc_.setTimeStamp(timeStampPtr);
		GvFunc_.setTimeStamp(timeStampPtr);

		QvFunc_.setTimeStamp(timeStampPtr);
		QmFunc_.setTimeStamp(timeStampPtr);
		PmFunc_.setTimeStamp(timeStampPtr);

		RvFunc_.setTimeStamp(timeStampPtr);
		RmFunc_.setTimeStamp(timeStampPtr);
		RmInverseFunc_.setTimeStamp(timeStampPtr);

		if (AmPtr) AmFunc_.setData(AmPtr); else  throw std::runtime_error("Am is not set.");
		if (OmPtr) OmFunc_.setData(OmPtr); else  OmFunc_.setZero();
		if (BmPtr) BmFunc_.setData(BmPtr); else  throw std::runtime_error("Bm is not set.");
		if (GvPtr) GvFunc_.setData(GvPtr); else  GvFunc_.setZero();

		if (QvPtr) QvFunc_.setData(QvPtr); else  QvFunc_.setZero();
		if (QmPtr) QmFunc_.setData(QmPtr); else  QmFunc_.setZero();
		if (PmPtr) PmFunc_.setData(PmPtr); else  PmFunc_.setZero();

		if (RvPtr) RvFunc_.setData(RvPtr); else  RvFunc_.setZero();
		if (RmPtr) RmFunc_.setData(RmPtr); else  throw std::runtime_error("Rm is not set.");
		if (RmInversePtr) RmInverseFunc_.setData(RmInversePtr); else  throw std::runtime_error("RmInverse is not set.");

		startTime_ = timeStampPtr->front();
		finalTime_ = timeStampPtr->back();
	}

    /**
     * Computes derivative
     * @param [in] z: Normalized time.
     * @param [in] MSv: Single vector constructed by concatenating Mm, Sv.
     * @param [out] derivatives: derivatives: d(MSv)/dz.
     */
	void computeFlowMap(const double& z, const full_ode_vector_t& MSv, full_ode_vector_t& derivatives) override {

		// denormalized time
		if (z>1 || z<0)  throw std::runtime_error("The normalized time should be between zero and one.");

		double t = finalTime_ - (finalTime_-startTime_)*z;

		state_state_matrix_t Mm;
		state_vector_t Sv;
		convert2Matrix(MSv, Mm, Sv);

		// numerical consideration
		if (useMakePSD_==true)
			bool hasNegativeEigenValue = makePSD(Mm);
		else
			Mm += state_state_matrix_t::Identity()*(1e-5);

		state_state_matrix_t Am;
		AmFunc_.interpolate(t, Am);
		size_t greatestLessTimeStampIndex = AmFunc_.getGreatestLessTimeStampIndex();
		state_state_matrix_t Om;
		OmFunc_.interpolate(t, Om, greatestLessTimeStampIndex);
		state_input_matrix_t Bm;
		BmFunc_.interpolate(t, Bm, greatestLessTimeStampIndex);
		state_vector_t Gv;
		GvFunc_.interpolate(t, Gv, greatestLessTimeStampIndex);

		state_vector_t Qv;
		QvFunc_.interpolate(t, Qv, greatestLessTimeStampIndex);
		state_state_matrix_t Qm;
		QmFunc_.interpolate(t, Qm, greatestLessTimeStampIndex);
		input_state_matrix_t Pm;
		PmFunc_.interpolate(t, Pm, greatestLessTimeStampIndex);
		input_vector_t Rv;
		RvFunc_.interpolate(t, Rv, greatestLessTimeStampIndex);
		input_input_matrix_t Rm;
		RmFunc_.interpolate(t, Rm, greatestLessTimeStampIndex);
		input_input_matrix_t RmInverse;
		RmInverseFunc_.interpolate(t, RmInverse, greatestLessTimeStampIndex);

		state_state_matrix_t dMmdt, dMmdz;
		state_vector_t dSvdt, dSvdz;

		// Uv = -Lv - Km*x
		input_vector_t       Lv = RmInverse*(Rv+Bm.transpose()*Sv);
		input_state_matrix_t Km = RmInverse*(Pm+Bm.transpose()*Mm);

		// Riccati equations for the original system
		dMmdt = Qm + Am.transpose()*Mm + Mm.transpose()*Am + Mm.transpose()*Om*Mm - Km.transpose()*Rm*Km;
		dSvdt = (Qv+Mm*Gv) + Am.transpose()*Sv + Mm.transpose()*Om*Sv - Km.transpose()*Rm*Lv;

		// Riccati equations for the equivalent system
		dMmdz = (finalTime_-startTime_)*dMmdt;
		dSvdz = (finalTime_-startTime_)*dSvdt;

		convert2Vector(dMmdz, dSvdz, derivatives);
	}

	/**
	 * Makes the matrix PSD.
	 * @tparam Derived type.
	 * @param [out] squareMatrix: The matrix to become PSD.
	 * @return boolean
	 */
	template <typename Derived>
	static bool makePSD(Eigen::MatrixBase<Derived>& squareMatrix) {

		if (squareMatrix.rows() != squareMatrix.cols())  throw std::runtime_error("Not a square matrix: makePSD() method is for square matrix.");

		Eigen::SelfAdjointEigenSolver<Derived> eig(squareMatrix);
		Eigen::VectorXd lambda = eig.eigenvalues();

		bool hasNegativeEigenValue = false;
		for (size_t j=0; j<lambda.size() ; j++)
			if (lambda(j) < 0.0) {
				hasNegativeEigenValue = true;
				lambda(j) = 1e-6;
			}

		if (hasNegativeEigenValue)
			squareMatrix = eig.eigenvectors() * lambda.asDiagonal() * eig.eigenvectors().inverse();
		else
			squareMatrix = 0.5*(squareMatrix+squareMatrix.transpose()).eval();

		return hasNegativeEigenValue;
	}

private:
	bool useMakePSD_;
	double startTime_;
	double finalTime_;

	LinearInterpolation<state_state_matrix_t, Eigen::aligned_allocator<state_state_matrix_t> > AmFunc_;
	LinearInterpolation<state_state_matrix_t, Eigen::aligned_allocator<state_state_matrix_t> > OmFunc_;
	LinearInterpolation<state_input_matrix_t, Eigen::aligned_allocator<state_input_matrix_t> > BmFunc_;
	LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> >             GvFunc_;

	LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> > QvFunc_;
	LinearInterpolation<input_vector_t, Eigen::aligned_allocator<input_vector_t> > RvFunc_;

	LinearInterpolation<state_state_matrix_t, Eigen::aligned_allocator<state_state_matrix_t> > QmFunc_;
	LinearInterpolation<input_state_matrix_t, Eigen::aligned_allocator<input_state_matrix_t> > PmFunc_;
	LinearInterpolation<input_input_matrix_t, Eigen::aligned_allocator<input_input_matrix_t> > RmFunc_;
	LinearInterpolation<input_input_matrix_t, Eigen::aligned_allocator<input_input_matrix_t> > RmInverseFunc_;

};

}  // end of OCS2 name space



#endif /* BVPEQUATIONS_H_OCS2_ */
