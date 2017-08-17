/*
 * SensitivitySequentialRiccatiEquations.h
 *
 *  Created on: Jun 21, 2016
 *      Author: farbod
 */

#ifndef SENSITIVITYSEQUENTIALRICCATIEQUATIONS_OCS2_H_
#define SENSITIVITYSEQUENTIALRICCATIEQUATIONS_OCS2_H_

#include <array>
#include <Eigen/Dense>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/dynamics/SystemBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2{

template <size_t STATE_DIM, size_t INPUT_DIM>
class SensitivitySequentialRiccatiEquations : public SystemBase<Eigen::Dynamic>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum { S_DIM_ = STATE_DIM*STATE_DIM+STATE_DIM+1 };

	typedef Eigen::Matrix<double,S_DIM_,1> s_vector_t;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::controller_t controller_t;
	typedef typename DIMENSIONS::scalar_t 		scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_t       eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t eigen_scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t 	  state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::control_vector_t 		control_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::control_feedback_t 	  control_feedback_t;
	typedef typename DIMENSIONS::control_feedback_array_t control_feedback_array_t;
	typedef typename DIMENSIONS::state_matrix_t 	  state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::control_matrix_t 		control_matrix_t;
	typedef typename DIMENSIONS::control_matrix_array_t control_matrix_array_t;
	typedef typename DIMENSIONS::control_gain_matrix_t 		 control_gain_matrix_t;
	typedef typename DIMENSIONS::control_gain_matrix_array_t control_gain_matrix_array_t;

	typedef Eigen::Matrix<double,STATE_DIM,Eigen::Dynamic> nabla_state_matrix_t;
	typedef Eigen::Matrix<double,INPUT_DIM,Eigen::Dynamic> nabla_input_matrix_t;
	typedef Eigen::Matrix<double,1,Eigen::Dynamic> 		   nabla_scalar_rowvector_t;
	typedef std::vector<nabla_state_matrix_t>     nabla_state_matrix_array_t;
	typedef std::vector<nabla_input_matrix_t>     nabla_input_matrix_array_t;
	typedef std::vector<nabla_scalar_rowvector_t> nabla_scalar_rowvector_array_t;

	SensitivitySequentialRiccatiEquations() {}
	~SensitivitySequentialRiccatiEquations() {}

	static void convert2Vector(const size_t& numSubsystems, const state_matrix_array_t& nabla_Sm, const state_vector_array_t& nabla_Sv, const eigen_scalar_array_t& nabla_s,
			Eigen::VectorXd& allSs)  {

		allSs.resize((numSubsystems-1)*S_DIM_);

		for (size_t j=0; j<numSubsystems-1; j++)
			allSs.segment(j*S_DIM_, S_DIM_) << Eigen::Map<const Eigen::VectorXd>(nabla_Sm[j].data(),STATE_DIM*STATE_DIM),
					Eigen::Map<const Eigen::VectorXd>(nabla_Sv[j].data(),STATE_DIM),
					nabla_s[j];
	}

	static void convert2Matrix(const size_t& numSubsystems, const Eigen::VectorXd& allSs,
			state_matrix_array_t& nabla_Sm, state_vector_array_t& nabla_Sv, eigen_scalar_array_t& nabla_s)  {

		nabla_Sm.resize(numSubsystems-1);
		nabla_Sv.resize(numSubsystems-1);
		nabla_s.resize(numSubsystems-1);

		for (size_t j=0; j<numSubsystems-1; j++) {
			nabla_Sm.at(j) = Eigen::Map<const Eigen::MatrixXd>(allSs.data()+j*S_DIM_,STATE_DIM, STATE_DIM);
			nabla_Sv.at(j) = Eigen::Map<const Eigen::VectorXd>(allSs.data()+j*S_DIM_+STATE_DIM*STATE_DIM, STATE_DIM);
			nabla_s.at(j)  = Eigen::Map<const Eigen::VectorXd>(allSs.data()+j*S_DIM_+STATE_DIM*STATE_DIM+STATE_DIM, 1);
		}
	}

	void setData(const size_t& numSubsystems, const scalar_t& learningRate,
			const size_t& activeSubsystem, const scalar_t& switchingTimeStart, const scalar_t& switchingTimeFinal,
			const scalar_array_t* SsTimePtr, const state_matrix_array_t* SmPtr, const state_vector_array_t* SvPtr,
			const scalar_array_t* timeStampPtr,
			const state_matrix_array_t* AmPtr, const control_gain_matrix_array_t* BmPtr,
			const eigen_scalar_array_t* qPtr, const state_vector_array_t* QvPtr, const state_matrix_array_t* QmPtr,
			const control_vector_array_t* RvPtr, const control_matrix_array_t* RmInversePtr, const control_matrix_array_t* RmPtr,
			const control_feedback_array_t* PmPtr,
			const scalar_array_t* sensitivityTimeStampPtr, const nabla_scalar_rowvector_array_t* nablaqPtr,
			const nabla_state_matrix_array_t* nablaQvPtr, const nabla_input_matrix_array_t* nablaRvPtr)  {

		numSubsystems_ = numSubsystems;
		alpha_ = learningRate;

		activeSubsystem_ = activeSubsystem;
		switchingTimeStart_ = switchingTimeStart;
		switchingTimeFinal_ = switchingTimeFinal;

		SvFunc_.setTimeStamp(SsTimePtr);
		SvFunc_.setData(SvPtr);
		SmFunc_.setTimeStamp(SsTimePtr);
		SmFunc_.setData(SmPtr);

		AmFunc_.setTimeStamp(timeStampPtr);
		AmFunc_.setData(AmPtr);
		BmFunc_.setTimeStamp(timeStampPtr);
		BmFunc_.setData(BmPtr);

		qFunc_.setTimeStamp(timeStampPtr);
		qFunc_.setData(qPtr);
		QvFunc_.setTimeStamp(timeStampPtr);
		QvFunc_.setData(QvPtr);
		QmFunc_.setTimeStamp(timeStampPtr);
		QmFunc_.setData(QmPtr);
		RvFunc_.setTimeStamp(timeStampPtr);
		RvFunc_.setData(RvPtr);
		RmInverseFunc_.setTimeStamp(timeStampPtr);
		RmInverseFunc_.setData(RmInversePtr);
		RmFunc_.setTimeStamp(timeStampPtr);
		RmFunc_.setData(RmPtr);
		PmFunc_.setTimeStamp(timeStampPtr);
		PmFunc_.setData(PmPtr);

		nablaqFunc_.setTimeStamp(sensitivityTimeStampPtr);
		nablaqFunc_.setData(nablaqPtr);
		nablaQvFunc_.setTimeStamp(sensitivityTimeStampPtr);
		nablaQvFunc_.setData(nablaQvPtr);
		nablaRvFunc_.setTimeStamp(sensitivityTimeStampPtr);
		nablaRvFunc_.setData(nablaRvPtr);
	}

	void computeDerivative(const scalar_t& z, const Eigen::VectorXd& allSs, Eigen::VectorXd& derivatives)  {

		// denormalized time
		scalar_t t = switchingTimeFinal_ - (switchingTimeFinal_-switchingTimeStart_)*z;

		convert2Matrix(numSubsystems_, allSs, nabla_Sm_, nabla_Sv_, nabla_s_);

		size_t greatestLessTimeStampIndex;

		SvFunc_.interpolate(t, Sv_);
		greatestLessTimeStampIndex = SvFunc_.getGreatestLessTimeStampIndex();
		SmFunc_.interpolate(t, Sm_, greatestLessTimeStampIndex);
		AmFunc_.interpolate(t, Am_);
		greatestLessTimeStampIndex = AmFunc_.getGreatestLessTimeStampIndex();
		BmFunc_.interpolate(t, Bm_, greatestLessTimeStampIndex);
		qFunc_.interpolate(t, q_);
		greatestLessTimeStampIndex = qFunc_.getGreatestLessTimeStampIndex();
		QvFunc_.interpolate(t, Qv_, greatestLessTimeStampIndex);
		QmFunc_.interpolate(t, Qm_, greatestLessTimeStampIndex);
		RvFunc_.interpolate(t, Rv_, greatestLessTimeStampIndex);
		RmInverseFunc_.interpolate(t, invRm_, greatestLessTimeStampIndex);
		RmFunc_.interpolate(t, Rm_, greatestLessTimeStampIndex);
		PmFunc_.interpolate(t, Pm_, greatestLessTimeStampIndex);

		nablaqFunc_.interpolate(t, nablaq_);
		greatestLessTimeStampIndex = nablaqFunc_.getGreatestLessTimeStampIndex();
		nablaQvFunc_.interpolate(t, nablaQv_, greatestLessTimeStampIndex);
		nablaRvFunc_.interpolate(t, nablaRv_, greatestLessTimeStampIndex);

		// Riccati equations for the original system
		Lm_ = invRm_ * (Pm_+Bm_.transpose()*Sm_);
		Lv_ = invRm_ * (Rv_+Bm_.transpose()*Sv_);
		dSmdt_ = Qm_ + Am_.transpose()*Sm_ + Sm_.transpose()*Am_ - Lm_.transpose()*Rm_*Lm_;
		dSmdt_ = 0.5*(dSmdt_+dSmdt_.transpose()).eval();
		dSvdt_ = Qv_ + Am_.transpose()*Sv_ - Lm_.transpose()*Rm_*Lv_;
		dsdt_  = q_ - 0.5*alpha_*(2.0-alpha_)*Lv_.transpose()*Rm_*Lv_;

		// derivatives of Riccati equations
		for (size_t j=0; j<numSubsystems_-1; j++) {

			// switching time gradient for the original system
			nabla_dSmdt_ = Am_.transpose()*nabla_Sm_[j] + nabla_Sm_[j].transpose()*Am_ - nabla_Sm_[j].transpose()*Bm_*invRm_*Rm_*Lm_
					- Lm_.transpose()*Rm_*invRm_*Bm_.transpose()*nabla_Sm_[j];
			nabla_dSmdt_ = 0.5*(nabla_dSmdt_+nabla_dSmdt_.transpose()).eval();
			nabla_dSvdt_ = nablaQv_.col(j) + Am_.transpose()*nabla_Sv_[j] - nabla_Sm_[j].transpose()*Bm_*invRm_*Rm_*Lv_
					- Lm_.transpose()*Rm_*invRm_*(nablaRv_.col(j) + Bm_.transpose()*nabla_Sv_[j]);
			nabla_dsdt_  = nablaq_.col(j) - 0.5*alpha_*(2-alpha_)*(nablaRv_.col(j) + Bm_.transpose()*nabla_Sv_[j]).transpose()*invRm_*Rm_*Lv_
					- 0.5*alpha_*(2-alpha_)*Lv_.transpose()*Rm_*invRm_*(nablaRv_.col(j) + Bm_.transpose()*nabla_Sv_[j]);

			// switching time gradient for the equivalent system
			nabla_dSmdz_[j] = (switchingTimeFinal_-switchingTimeStart_)*nabla_dSmdt_;
			nabla_dSvdz_[j] = (switchingTimeFinal_-switchingTimeStart_)*nabla_dSvdt_;
			nabla_dsdz_[j]  = (switchingTimeFinal_-switchingTimeStart_)*nabla_dsdt_;

			if (j==activeSubsystem_)  {
				nabla_dSmdz_[j] += dSmdt_;
				nabla_dSvdz_[j] += dSvdt_;
				nabla_dsdz_[j]  += dsdt_;
			}
			if (j==activeSubsystem_-1) {
				nabla_dSmdz_[j] -= dSmdt_;
				nabla_dSvdz_[j] -= dSvdt_;
				nabla_dsdz_[j]  -= dsdt_;
			}

		}  // end of j loop

		convert2Vector(numSubsystems_, nabla_dSmdz_, nabla_dSvdz_, nabla_dsdz_, derivatives);
	}


private:
	size_t numSubsystems_;
	scalar_t alpha_;

	size_t activeSubsystem_;
	scalar_t switchingTimeStart_;
	scalar_t switchingTimeFinal_;

	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > SvFunc_;
	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > SmFunc_;

	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > AmFunc_;
	LinearInterpolation<control_gain_matrix_t,Eigen::aligned_allocator<control_gain_matrix_t> > BmFunc_;

	LinearInterpolation<eigen_scalar_t,Eigen::aligned_allocator<eigen_scalar_t> > qFunc_;
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > QvFunc_;
	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > QmFunc_;
	LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> > RvFunc_;
	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> > RmInverseFunc_;
	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> > RmFunc_;
	LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> > PmFunc_;

	LinearInterpolation<nabla_scalar_rowvector_t> 	nablaqFunc_;
	LinearInterpolation<nabla_state_matrix_t> 		nablaQvFunc_;
	LinearInterpolation<nabla_input_matrix_t> 		nablaRvFunc_;

	state_matrix_array_t nabla_Sm_;
	state_vector_array_t nabla_Sv_;
	eigen_scalar_array_t nabla_s_;
	state_vector_t Sv_;
	state_matrix_t Sm_;
	state_matrix_t Am_;
	control_gain_matrix_t Bm_;
	eigen_scalar_t q_;
	state_vector_t Qv_;
	state_matrix_t Qm_;
	control_vector_t Rv_;
	control_matrix_t invRm_;
	control_matrix_t Rm_;
	control_feedback_t Pm_;
	nabla_scalar_rowvector_t nablaq_;
	nabla_state_matrix_t nablaQv_;
	nabla_input_matrix_t nablaRv_;
	control_feedback_t Lm_;
	control_vector_t Lv_;
	state_matrix_t dSmdt_;
	state_vector_t dSvdt_;
	eigen_scalar_t dsdt_;

	// derivatives of Riccati equations
	state_matrix_array_t nabla_dSmdz_;
	state_vector_array_t nabla_dSvdz_;
	eigen_scalar_array_t nabla_dsdz_;

	// switching time gradient for the original system
	state_matrix_t nabla_dSmdt_;
	state_vector_t nabla_dSvdt_;
	eigen_scalar_t nabla_dsdt_;

};

} // namespace ocs2



#endif /* SENSITIVITYSEQUENTIALRICCATIEQUATIONS_OCS2_H_ */
