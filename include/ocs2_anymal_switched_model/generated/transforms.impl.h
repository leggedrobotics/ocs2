
// Constructors
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::MotionTransforms
    ()
     :
    fr_base_X_fr_LF_HIP(),
    fr_LF_HIP_X_fr_base(),
    fr_base_X_fr_LF_THIGH(),
    fr_LF_THIGH_X_fr_base(),
    fr_base_X_fr_LF_SHANK(),
    fr_LF_SHANK_X_fr_base(),
    fr_base_X_fr_RF_HIP(),
    fr_RF_HIP_X_fr_base(),
    fr_base_X_fr_RF_THIGH(),
    fr_RF_THIGH_X_fr_base(),
    fr_base_X_fr_RF_SHANK(),
    fr_RF_SHANK_X_fr_base(),
    fr_base_X_fr_LH_HIP(),
    fr_LH_HIP_X_fr_base(),
    fr_base_X_fr_LH_THIGH(),
    fr_LH_THIGH_X_fr_base(),
    fr_base_X_fr_LH_SHANK(),
    fr_LH_SHANK_X_fr_base(),
    fr_base_X_fr_RH_HIP(),
    fr_RH_HIP_X_fr_base(),
    fr_base_X_fr_RH_THIGH(),
    fr_RH_THIGH_X_fr_base(),
    fr_base_X_fr_RH_SHANK(),
    fr_RH_SHANK_X_fr_base(),
    fr_base_X_fr_LF_ADAPTER(),
    fr_LF_ADAPTER_X_fr_base(),
    fr_base_X_fr_LF_FOOT(),
    fr_LF_FOOT_X_fr_base(),
    fr_base_X_fr_LF_HIP_COM(),
    fr_LF_HIP_COM_X_fr_base(),
    fr_base_X_fr_LF_SHANK_COM(),
    fr_LF_SHANK_COM_X_fr_base(),
    fr_base_X_fr_LF_THIGH_COM(),
    fr_LF_THIGH_COM_X_fr_base(),
    fr_base_X_fr_LH_ADAPTER(),
    fr_LH_ADAPTER_X_fr_base(),
    fr_base_X_fr_LH_FOOT(),
    fr_LH_FOOT_X_fr_base(),
    fr_base_X_fr_LH_HIP_COM(),
    fr_LH_HIP_COM_X_fr_base(),
    fr_base_X_fr_LH_SHANK_COM(),
    fr_LH_SHANK_COM_X_fr_base(),
    fr_base_X_fr_LH_THIGH_COM(),
    fr_LH_THIGH_COM_X_fr_base(),
    fr_base_X_fr_RF_ADAPTER(),
    fr_RF_ADAPTER_X_fr_base(),
    fr_base_X_fr_RF_FOOT(),
    fr_RF_FOOT_X_fr_base(),
    fr_base_X_fr_RF_HIP_COM(),
    fr_RF_HIP_COM_X_fr_base(),
    fr_base_X_fr_RF_SHANK_COM(),
    fr_RF_SHANK_COM_X_fr_base(),
    fr_base_X_fr_RF_THIGH_COM(),
    fr_RF_THIGH_COM_X_fr_base(),
    fr_base_X_fr_RH_ADAPTER(),
    fr_RH_ADAPTER_X_fr_base(),
    fr_base_X_fr_RH_FOOT(),
    fr_RH_FOOT_X_fr_base(),
    fr_base_X_fr_RH_HIP_COM(),
    fr_RH_HIP_COM_X_fr_base(),
    fr_base_X_fr_RH_SHANK_COM(),
    fr_RH_SHANK_COM_X_fr_base(),
    fr_base_X_fr_RH_THIGH_COM(),
    fr_RH_THIGH_COM_X_fr_base(),
    fr_base_X_fr_base_COM(),
    fr_base_COM_X_fr_base(),
    fr_base_X_fr_base_inertia(),
    fr_base_inertia_X_fr_base(),
    fr_base_X_fr_imu_link(),
    fr_imu_link_X_fr_base(),
    fr_base_X_fr_LF_HAA(),
    fr_base_X_fr_LF_HFE(),
    fr_base_X_fr_LF_KFE(),
    fr_base_X_fr_RF_HAA(),
    fr_base_X_fr_RF_HFE(),
    fr_base_X_fr_RF_KFE(),
    fr_base_X_fr_LH_HAA(),
    fr_base_X_fr_LH_HFE(),
    fr_base_X_fr_LH_KFE(),
    fr_base_X_fr_RH_HAA(),
    fr_base_X_fr_RH_HFE(),
    fr_base_X_fr_RH_KFE(),
    fr_LF_THIGH_X_fr_LF_HIP(),
    fr_LF_HIP_X_fr_LF_THIGH(),
    fr_LF_SHANK_X_fr_LF_THIGH(),
    fr_LF_THIGH_X_fr_LF_SHANK(),
    fr_RF_THIGH_X_fr_RF_HIP(),
    fr_RF_HIP_X_fr_RF_THIGH(),
    fr_RF_SHANK_X_fr_RF_THIGH(),
    fr_RF_THIGH_X_fr_RF_SHANK(),
    fr_LH_THIGH_X_fr_LH_HIP(),
    fr_LH_HIP_X_fr_LH_THIGH(),
    fr_LH_SHANK_X_fr_LH_THIGH(),
    fr_LH_THIGH_X_fr_LH_SHANK(),
    fr_RH_THIGH_X_fr_RH_HIP(),
    fr_RH_HIP_X_fr_RH_THIGH(),
    fr_RH_SHANK_X_fr_RH_THIGH(),
    fr_RH_THIGH_X_fr_RH_SHANK()
{
    updateParameters();
}
template <typename TRAIT>
void iit::ANYmal::tpl::MotionTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::ForceTransforms
    ()
     :
    fr_base_X_fr_LF_HIP(),
    fr_LF_HIP_X_fr_base(),
    fr_base_X_fr_LF_THIGH(),
    fr_LF_THIGH_X_fr_base(),
    fr_base_X_fr_LF_SHANK(),
    fr_LF_SHANK_X_fr_base(),
    fr_base_X_fr_RF_HIP(),
    fr_RF_HIP_X_fr_base(),
    fr_base_X_fr_RF_THIGH(),
    fr_RF_THIGH_X_fr_base(),
    fr_base_X_fr_RF_SHANK(),
    fr_RF_SHANK_X_fr_base(),
    fr_base_X_fr_LH_HIP(),
    fr_LH_HIP_X_fr_base(),
    fr_base_X_fr_LH_THIGH(),
    fr_LH_THIGH_X_fr_base(),
    fr_base_X_fr_LH_SHANK(),
    fr_LH_SHANK_X_fr_base(),
    fr_base_X_fr_RH_HIP(),
    fr_RH_HIP_X_fr_base(),
    fr_base_X_fr_RH_THIGH(),
    fr_RH_THIGH_X_fr_base(),
    fr_base_X_fr_RH_SHANK(),
    fr_RH_SHANK_X_fr_base(),
    fr_base_X_fr_LF_ADAPTER(),
    fr_LF_ADAPTER_X_fr_base(),
    fr_base_X_fr_LF_FOOT(),
    fr_LF_FOOT_X_fr_base(),
    fr_base_X_fr_LF_HIP_COM(),
    fr_LF_HIP_COM_X_fr_base(),
    fr_base_X_fr_LF_SHANK_COM(),
    fr_LF_SHANK_COM_X_fr_base(),
    fr_base_X_fr_LF_THIGH_COM(),
    fr_LF_THIGH_COM_X_fr_base(),
    fr_base_X_fr_LH_ADAPTER(),
    fr_LH_ADAPTER_X_fr_base(),
    fr_base_X_fr_LH_FOOT(),
    fr_LH_FOOT_X_fr_base(),
    fr_base_X_fr_LH_HIP_COM(),
    fr_LH_HIP_COM_X_fr_base(),
    fr_base_X_fr_LH_SHANK_COM(),
    fr_LH_SHANK_COM_X_fr_base(),
    fr_base_X_fr_LH_THIGH_COM(),
    fr_LH_THIGH_COM_X_fr_base(),
    fr_base_X_fr_RF_ADAPTER(),
    fr_RF_ADAPTER_X_fr_base(),
    fr_base_X_fr_RF_FOOT(),
    fr_RF_FOOT_X_fr_base(),
    fr_base_X_fr_RF_HIP_COM(),
    fr_RF_HIP_COM_X_fr_base(),
    fr_base_X_fr_RF_SHANK_COM(),
    fr_RF_SHANK_COM_X_fr_base(),
    fr_base_X_fr_RF_THIGH_COM(),
    fr_RF_THIGH_COM_X_fr_base(),
    fr_base_X_fr_RH_ADAPTER(),
    fr_RH_ADAPTER_X_fr_base(),
    fr_base_X_fr_RH_FOOT(),
    fr_RH_FOOT_X_fr_base(),
    fr_base_X_fr_RH_HIP_COM(),
    fr_RH_HIP_COM_X_fr_base(),
    fr_base_X_fr_RH_SHANK_COM(),
    fr_RH_SHANK_COM_X_fr_base(),
    fr_base_X_fr_RH_THIGH_COM(),
    fr_RH_THIGH_COM_X_fr_base(),
    fr_base_X_fr_base_COM(),
    fr_base_COM_X_fr_base(),
    fr_base_X_fr_base_inertia(),
    fr_base_inertia_X_fr_base(),
    fr_base_X_fr_imu_link(),
    fr_imu_link_X_fr_base(),
    fr_base_X_fr_LF_HAA(),
    fr_base_X_fr_LF_HFE(),
    fr_base_X_fr_LF_KFE(),
    fr_base_X_fr_RF_HAA(),
    fr_base_X_fr_RF_HFE(),
    fr_base_X_fr_RF_KFE(),
    fr_base_X_fr_LH_HAA(),
    fr_base_X_fr_LH_HFE(),
    fr_base_X_fr_LH_KFE(),
    fr_base_X_fr_RH_HAA(),
    fr_base_X_fr_RH_HFE(),
    fr_base_X_fr_RH_KFE(),
    fr_LF_THIGH_X_fr_LF_HIP(),
    fr_LF_HIP_X_fr_LF_THIGH(),
    fr_LF_SHANK_X_fr_LF_THIGH(),
    fr_LF_THIGH_X_fr_LF_SHANK(),
    fr_RF_THIGH_X_fr_RF_HIP(),
    fr_RF_HIP_X_fr_RF_THIGH(),
    fr_RF_SHANK_X_fr_RF_THIGH(),
    fr_RF_THIGH_X_fr_RF_SHANK(),
    fr_LH_THIGH_X_fr_LH_HIP(),
    fr_LH_HIP_X_fr_LH_THIGH(),
    fr_LH_SHANK_X_fr_LH_THIGH(),
    fr_LH_THIGH_X_fr_LH_SHANK(),
    fr_RH_THIGH_X_fr_RH_HIP(),
    fr_RH_HIP_X_fr_RH_THIGH(),
    fr_RH_SHANK_X_fr_RH_THIGH(),
    fr_RH_THIGH_X_fr_RH_SHANK()
{
    updateParameters();
}
template <typename TRAIT>
void iit::ANYmal::tpl::ForceTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::HomogeneousTransforms
    ()
     :
    fr_base_X_fr_LF_HIP(),
    fr_LF_HIP_X_fr_base(),
    fr_base_X_fr_LF_THIGH(),
    fr_LF_THIGH_X_fr_base(),
    fr_base_X_fr_LF_SHANK(),
    fr_LF_SHANK_X_fr_base(),
    fr_base_X_fr_RF_HIP(),
    fr_RF_HIP_X_fr_base(),
    fr_base_X_fr_RF_THIGH(),
    fr_RF_THIGH_X_fr_base(),
    fr_base_X_fr_RF_SHANK(),
    fr_RF_SHANK_X_fr_base(),
    fr_base_X_fr_LH_HIP(),
    fr_LH_HIP_X_fr_base(),
    fr_base_X_fr_LH_THIGH(),
    fr_LH_THIGH_X_fr_base(),
    fr_base_X_fr_LH_SHANK(),
    fr_LH_SHANK_X_fr_base(),
    fr_base_X_fr_RH_HIP(),
    fr_RH_HIP_X_fr_base(),
    fr_base_X_fr_RH_THIGH(),
    fr_RH_THIGH_X_fr_base(),
    fr_base_X_fr_RH_SHANK(),
    fr_RH_SHANK_X_fr_base(),
    fr_base_X_fr_LF_ADAPTER(),
    fr_LF_ADAPTER_X_fr_base(),
    fr_base_X_fr_LF_FOOT(),
    fr_LF_FOOT_X_fr_base(),
    fr_base_X_fr_LF_HIP_COM(),
    fr_LF_HIP_COM_X_fr_base(),
    fr_base_X_fr_LF_SHANK_COM(),
    fr_LF_SHANK_COM_X_fr_base(),
    fr_base_X_fr_LF_THIGH_COM(),
    fr_LF_THIGH_COM_X_fr_base(),
    fr_base_X_fr_LH_ADAPTER(),
    fr_LH_ADAPTER_X_fr_base(),
    fr_base_X_fr_LH_FOOT(),
    fr_LH_FOOT_X_fr_base(),
    fr_base_X_fr_LH_HIP_COM(),
    fr_LH_HIP_COM_X_fr_base(),
    fr_base_X_fr_LH_SHANK_COM(),
    fr_LH_SHANK_COM_X_fr_base(),
    fr_base_X_fr_LH_THIGH_COM(),
    fr_LH_THIGH_COM_X_fr_base(),
    fr_base_X_fr_RF_ADAPTER(),
    fr_RF_ADAPTER_X_fr_base(),
    fr_base_X_fr_RF_FOOT(),
    fr_RF_FOOT_X_fr_base(),
    fr_base_X_fr_RF_HIP_COM(),
    fr_RF_HIP_COM_X_fr_base(),
    fr_base_X_fr_RF_SHANK_COM(),
    fr_RF_SHANK_COM_X_fr_base(),
    fr_base_X_fr_RF_THIGH_COM(),
    fr_RF_THIGH_COM_X_fr_base(),
    fr_base_X_fr_RH_ADAPTER(),
    fr_RH_ADAPTER_X_fr_base(),
    fr_base_X_fr_RH_FOOT(),
    fr_RH_FOOT_X_fr_base(),
    fr_base_X_fr_RH_HIP_COM(),
    fr_RH_HIP_COM_X_fr_base(),
    fr_base_X_fr_RH_SHANK_COM(),
    fr_RH_SHANK_COM_X_fr_base(),
    fr_base_X_fr_RH_THIGH_COM(),
    fr_RH_THIGH_COM_X_fr_base(),
    fr_base_X_fr_base_COM(),
    fr_base_COM_X_fr_base(),
    fr_base_X_fr_base_inertia(),
    fr_base_inertia_X_fr_base(),
    fr_base_X_fr_imu_link(),
    fr_imu_link_X_fr_base(),
    fr_base_X_fr_LF_HAA(),
    fr_base_X_fr_LF_HFE(),
    fr_base_X_fr_LF_KFE(),
    fr_base_X_fr_RF_HAA(),
    fr_base_X_fr_RF_HFE(),
    fr_base_X_fr_RF_KFE(),
    fr_base_X_fr_LH_HAA(),
    fr_base_X_fr_LH_HFE(),
    fr_base_X_fr_LH_KFE(),
    fr_base_X_fr_RH_HAA(),
    fr_base_X_fr_RH_HFE(),
    fr_base_X_fr_RH_KFE(),
    fr_LF_THIGH_X_fr_LF_HIP(),
    fr_LF_HIP_X_fr_LF_THIGH(),
    fr_LF_SHANK_X_fr_LF_THIGH(),
    fr_LF_THIGH_X_fr_LF_SHANK(),
    fr_RF_THIGH_X_fr_RF_HIP(),
    fr_RF_HIP_X_fr_RF_THIGH(),
    fr_RF_SHANK_X_fr_RF_THIGH(),
    fr_RF_THIGH_X_fr_RF_SHANK(),
    fr_LH_THIGH_X_fr_LH_HIP(),
    fr_LH_HIP_X_fr_LH_THIGH(),
    fr_LH_SHANK_X_fr_LH_THIGH(),
    fr_LH_THIGH_X_fr_LH_SHANK(),
    fr_RH_THIGH_X_fr_RH_HIP(),
    fr_RH_HIP_X_fr_RH_THIGH(),
    fr_RH_SHANK_X_fr_RH_THIGH(),
    fr_RH_THIGH_X_fr_RH_SHANK()
{
    updateParameters();
}
template <typename TRAIT>
void iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP::Type_fr_base_X_fr_LF_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = - 0.116;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,0) =  s_q_LF_HAA_;
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(2,0) = - c_q_LF_HAA_;
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(3,0) = (- 0.116 *  c_q_LF_HAA_);
    (*this)(3,1) = ( 0.116 *  s_q_LF_HAA_);
    (*this)(4,0) = ( 0.277 *  c_q_LF_HAA_);
    (*this)(4,1) = (- 0.277 *  s_q_LF_HAA_);
    (*this)(4,3) =  s_q_LF_HAA_;
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(5,0) = ( 0.277 *  s_q_LF_HAA_);
    (*this)(5,1) = ( 0.277 *  c_q_LF_HAA_);
    (*this)(5,3) = - c_q_LF_HAA_;
    (*this)(5,4) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base::Type_fr_LF_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,3) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.116;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,1) =  s_q_LF_HAA_;
    (*this)(0,2) = - c_q_LF_HAA_;
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(3,0) = (- 0.116 *  c_q_LF_HAA_);
    (*this)(3,1) = ( 0.277 *  c_q_LF_HAA_);
    (*this)(3,2) = ( 0.277 *  s_q_LF_HAA_);
    (*this)(3,4) =  s_q_LF_HAA_;
    (*this)(3,5) = - c_q_LF_HAA_;
    (*this)(4,0) = ( 0.116 *  s_q_LF_HAA_);
    (*this)(4,1) = (- 0.277 *  s_q_LF_HAA_);
    (*this)(4,2) = ( 0.277 *  c_q_LF_HAA_);
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH::Type_fr_base_X_fr_LF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) =  c_q_LF_HFE_;
    (*this)(0,1) = - s_q_LF_HFE_;
    (*this)(1,0) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(1,1) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(2,0) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(2,1) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(3,0) = (((- 0.116 *  c_q_LF_HAA_) -  0.041) *  s_q_LF_HFE_);
    (*this)(3,1) = (((- 0.116 *  c_q_LF_HAA_) -  0.041) *  c_q_LF_HFE_);
    (*this)(3,2) = ( 0.116 *  s_q_LF_HAA_);
    (*this)(3,3) =  c_q_LF_HFE_;
    (*this)(3,4) = - s_q_LF_HFE_;
    (*this)(4,0) = ((( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.041 *  s_q_LF_HAA_) *  c_q_LF_HFE_));
    (*this)(4,1) = ((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.041 *  s_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(4,2) = (- 0.3405 *  s_q_LF_HAA_);
    (*this)(4,3) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(4,4) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(4,5) =  c_q_LF_HAA_;
    (*this)(5,0) = ((( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((- 0.116 - ( 0.041 *  c_q_LF_HAA_)) *  c_q_LF_HFE_));
    (*this)(5,1) = ((( 0.116 + ( 0.041 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) + (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_));
    (*this)(5,2) = ( 0.3405 *  c_q_LF_HAA_);
    (*this)(5,3) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(5,4) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_base::Type_fr_LF_THIGH_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) =  c_q_LF_HFE_;
    (*this)(0,1) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(0,2) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(1,0) = - s_q_LF_HFE_;
    (*this)(1,1) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,2) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,1) =  c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(3,0) = (((- 0.116 *  c_q_LF_HAA_) -  0.041) *  s_q_LF_HFE_);
    (*this)(3,1) = ((( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.041 *  s_q_LF_HAA_) *  c_q_LF_HFE_));
    (*this)(3,2) = ((( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((- 0.116 - ( 0.041 *  c_q_LF_HAA_)) *  c_q_LF_HFE_));
    (*this)(3,3) =  c_q_LF_HFE_;
    (*this)(3,4) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(3,5) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(4,0) = (((- 0.116 *  c_q_LF_HAA_) -  0.041) *  c_q_LF_HFE_);
    (*this)(4,1) = ((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.041 *  s_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(4,2) = ((( 0.116 + ( 0.041 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) + (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_));
    (*this)(4,3) = - s_q_LF_HFE_;
    (*this)(4,4) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(4,5) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(5,0) = ( 0.116 *  s_q_LF_HAA_);
    (*this)(5,1) = (- 0.3405 *  s_q_LF_HAA_);
    (*this)(5,2) = ( 0.3405 *  c_q_LF_HAA_);
    (*this)(5,4) =  c_q_LF_HAA_;
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK::Type_fr_base_X_fr_LF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,1) = ((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(1,0) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(2,0) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) = ((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(3,0) = ((((- 0.15 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.15 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,1) = (((( 0.15 + ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.15 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,2) = (( 0.25 *  c_q_LF_HFE_) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,4) = ((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(4,0) = ((((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.15 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  c_q_LF_HAA_) + (( 0.15 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(4,1) = ((((( 0.25 *  c_q_LF_HAA_) - (( 0.15 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.15 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(4,2) = ((( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(4,3) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,4) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(4,5) =  c_q_LF_HAA_;
    (*this)(5,0) = ((((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.15 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  s_q_LF_HAA_) + (((- 0.15 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(5,1) = ((((( 0.25 *  s_q_LF_HAA_) + ((( 0.15 *  c_q_LF_HAA_) +  0.116) *  c_q_LF_HFE_)) - (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.15 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(5,2) = (( 0.3405 *  c_q_LF_HAA_) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(5,3) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,4) = ((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_base::Type_fr_LF_SHANK_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,2) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,0) = ((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(1,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(1,2) = ((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) =  c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(3,0) = ((((- 0.15 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.15 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,1) = ((((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.15 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  c_q_LF_HAA_) + (( 0.15 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(3,2) = ((((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.15 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  s_q_LF_HAA_) + (((- 0.15 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,4) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,5) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,0) = (((( 0.15 + ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.15 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,1) = ((((( 0.25 *  c_q_LF_HAA_) - (( 0.15 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.15 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(4,2) = ((((( 0.25 *  s_q_LF_HAA_) + ((( 0.15 *  c_q_LF_HAA_) +  0.116) *  c_q_LF_HFE_)) - (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.15 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(4,3) = ((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(4,4) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(4,5) = ((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,0) = (( 0.25 *  c_q_LF_HFE_) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(5,1) = ((( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(5,2) = (( 0.3405 *  c_q_LF_HAA_) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(5,4) =  c_q_LF_HAA_;
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP::Type_fr_base_X_fr_RF_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0.116;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,0) =  s_q_RF_HAA_;
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(2,0) = - c_q_RF_HAA_;
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(3,0) = ( 0.116 *  c_q_RF_HAA_);
    (*this)(3,1) = (- 0.116 *  s_q_RF_HAA_);
    (*this)(4,0) = ( 0.277 *  c_q_RF_HAA_);
    (*this)(4,1) = (- 0.277 *  s_q_RF_HAA_);
    (*this)(4,3) =  s_q_RF_HAA_;
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(5,0) = ( 0.277 *  s_q_RF_HAA_);
    (*this)(5,1) = ( 0.277 *  c_q_RF_HAA_);
    (*this)(5,3) = - c_q_RF_HAA_;
    (*this)(5,4) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base::Type_fr_RF_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,3) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0.116;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,1) =  s_q_RF_HAA_;
    (*this)(0,2) = - c_q_RF_HAA_;
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(3,0) = ( 0.116 *  c_q_RF_HAA_);
    (*this)(3,1) = ( 0.277 *  c_q_RF_HAA_);
    (*this)(3,2) = ( 0.277 *  s_q_RF_HAA_);
    (*this)(3,4) =  s_q_RF_HAA_;
    (*this)(3,5) = - c_q_RF_HAA_;
    (*this)(4,0) = (- 0.116 *  s_q_RF_HAA_);
    (*this)(4,1) = (- 0.277 *  s_q_RF_HAA_);
    (*this)(4,2) = ( 0.277 *  c_q_RF_HAA_);
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH::Type_fr_base_X_fr_RF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) =  c_q_RF_HFE_;
    (*this)(0,1) = - s_q_RF_HFE_;
    (*this)(1,0) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(1,1) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(2,0) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(2,1) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(3,0) = ((( 0.116 *  c_q_RF_HAA_) +  0.041) *  s_q_RF_HFE_);
    (*this)(3,1) = ((( 0.116 *  c_q_RF_HAA_) +  0.041) *  c_q_RF_HFE_);
    (*this)(3,2) = (- 0.116 *  s_q_RF_HAA_);
    (*this)(3,3) =  c_q_RF_HFE_;
    (*this)(3,4) = - s_q_RF_HFE_;
    (*this)(4,0) = ((( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.041 *  s_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(4,1) = ((( 0.041 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(4,2) = (- 0.3405 *  s_q_RF_HAA_);
    (*this)(4,3) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(4,4) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(4,5) =  c_q_RF_HAA_;
    (*this)(5,0) = ((( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.116 + ( 0.041 *  c_q_RF_HAA_)) *  c_q_RF_HFE_));
    (*this)(5,1) = (((- 0.116 - ( 0.041 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) + (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(5,2) = ( 0.3405 *  c_q_RF_HAA_);
    (*this)(5,3) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(5,4) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_base::Type_fr_RF_THIGH_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) =  c_q_RF_HFE_;
    (*this)(0,1) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(0,2) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(1,0) = - s_q_RF_HFE_;
    (*this)(1,1) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,2) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,1) =  c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(3,0) = ((( 0.116 *  c_q_RF_HAA_) +  0.041) *  s_q_RF_HFE_);
    (*this)(3,1) = ((( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.041 *  s_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(3,2) = ((( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.116 + ( 0.041 *  c_q_RF_HAA_)) *  c_q_RF_HFE_));
    (*this)(3,3) =  c_q_RF_HFE_;
    (*this)(3,4) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(3,5) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(4,0) = ((( 0.116 *  c_q_RF_HAA_) +  0.041) *  c_q_RF_HFE_);
    (*this)(4,1) = ((( 0.041 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(4,2) = (((- 0.116 - ( 0.041 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) + (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(4,3) = - s_q_RF_HFE_;
    (*this)(4,4) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(4,5) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(5,0) = (- 0.116 *  s_q_RF_HAA_);
    (*this)(5,1) = (- 0.3405 *  s_q_RF_HAA_);
    (*this)(5,2) = ( 0.3405 *  c_q_RF_HAA_);
    (*this)(5,4) =  c_q_RF_HAA_;
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK::Type_fr_base_X_fr_RF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,1) = ((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(1,0) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(2,0) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) = ((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(3,0) = (((( 0.15 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.15 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,1) = ((((- 0.15 - ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.15 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,2) = (( 0.25 *  c_q_RF_HFE_) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,4) = ((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(4,0) = ((((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.15 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  c_q_RF_HAA_) - (( 0.15 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(4,1) = ((((( 0.25 *  c_q_RF_HAA_) + (( 0.15 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.15 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(4,2) = ((( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(4,3) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,4) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(4,5) =  c_q_RF_HAA_;
    (*this)(5,0) = ((((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.15 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  s_q_RF_HAA_) + ((( 0.15 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(5,1) = ((((( 0.25 *  s_q_RF_HAA_) + (((- 0.15 *  c_q_RF_HAA_) -  0.116) *  c_q_RF_HFE_)) - (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.15 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(5,2) = (( 0.3405 *  c_q_RF_HAA_) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(5,3) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,4) = ((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_base::Type_fr_RF_SHANK_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,2) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,0) = ((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(1,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(1,2) = ((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) =  c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(3,0) = (((( 0.15 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.15 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,1) = ((((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.15 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  c_q_RF_HAA_) - (( 0.15 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(3,2) = ((((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.15 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  s_q_RF_HAA_) + ((( 0.15 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,4) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,5) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,0) = ((((- 0.15 - ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.15 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,1) = ((((( 0.25 *  c_q_RF_HAA_) + (( 0.15 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.15 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(4,2) = ((((( 0.25 *  s_q_RF_HAA_) + (((- 0.15 *  c_q_RF_HAA_) -  0.116) *  c_q_RF_HFE_)) - (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.15 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(4,3) = ((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(4,4) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(4,5) = ((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,0) = (( 0.25 *  c_q_RF_HFE_) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(5,1) = ((( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(5,2) = (( 0.3405 *  c_q_RF_HAA_) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(5,4) =  c_q_RF_HAA_;
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP::Type_fr_base_X_fr_LH_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = - 0.116;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,0) =  s_q_LH_HAA_;
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(2,0) = - c_q_LH_HAA_;
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(3,0) = (- 0.116 *  c_q_LH_HAA_);
    (*this)(3,1) = ( 0.116 *  s_q_LH_HAA_);
    (*this)(4,0) = (- 0.277 *  c_q_LH_HAA_);
    (*this)(4,1) = ( 0.277 *  s_q_LH_HAA_);
    (*this)(4,3) =  s_q_LH_HAA_;
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(5,0) = (- 0.277 *  s_q_LH_HAA_);
    (*this)(5,1) = (- 0.277 *  c_q_LH_HAA_);
    (*this)(5,3) = - c_q_LH_HAA_;
    (*this)(5,4) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base::Type_fr_LH_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,3) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.116;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,1) =  s_q_LH_HAA_;
    (*this)(0,2) = - c_q_LH_HAA_;
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(3,0) = (- 0.116 *  c_q_LH_HAA_);
    (*this)(3,1) = (- 0.277 *  c_q_LH_HAA_);
    (*this)(3,2) = (- 0.277 *  s_q_LH_HAA_);
    (*this)(3,4) =  s_q_LH_HAA_;
    (*this)(3,5) = - c_q_LH_HAA_;
    (*this)(4,0) = ( 0.116 *  s_q_LH_HAA_);
    (*this)(4,1) = ( 0.277 *  s_q_LH_HAA_);
    (*this)(4,2) = (- 0.277 *  c_q_LH_HAA_);
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH::Type_fr_base_X_fr_LH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) =  c_q_LH_HFE_;
    (*this)(0,1) = - s_q_LH_HFE_;
    (*this)(1,0) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(1,1) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(2,0) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(2,1) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(3,0) = (((- 0.116 *  c_q_LH_HAA_) -  0.041) *  s_q_LH_HFE_);
    (*this)(3,1) = (((- 0.116 *  c_q_LH_HAA_) -  0.041) *  c_q_LH_HFE_);
    (*this)(3,2) = ( 0.116 *  s_q_LH_HAA_);
    (*this)(3,3) =  c_q_LH_HFE_;
    (*this)(3,4) = - s_q_LH_HFE_;
    (*this)(4,0) = ((( 0.041 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(4,1) = (((- 0.041 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(4,2) = ( 0.3405 *  s_q_LH_HAA_);
    (*this)(4,3) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(4,4) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(4,5) =  c_q_LH_HAA_;
    (*this)(5,0) = (((- 0.116 - ( 0.041 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(5,1) = ((( 0.116 + ( 0.041 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(5,2) = (- 0.3405 *  c_q_LH_HAA_);
    (*this)(5,3) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(5,4) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_base::Type_fr_LH_THIGH_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) =  c_q_LH_HFE_;
    (*this)(0,1) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(0,2) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(1,0) = - s_q_LH_HFE_;
    (*this)(1,1) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,2) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,1) =  c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(3,0) = (((- 0.116 *  c_q_LH_HAA_) -  0.041) *  s_q_LH_HFE_);
    (*this)(3,1) = ((( 0.041 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(3,2) = (((- 0.116 - ( 0.041 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(3,3) =  c_q_LH_HFE_;
    (*this)(3,4) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(3,5) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(4,0) = (((- 0.116 *  c_q_LH_HAA_) -  0.041) *  c_q_LH_HFE_);
    (*this)(4,1) = (((- 0.041 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(4,2) = ((( 0.116 + ( 0.041 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(4,3) = - s_q_LH_HFE_;
    (*this)(4,4) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(4,5) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(5,0) = ( 0.116 *  s_q_LH_HAA_);
    (*this)(5,1) = ( 0.3405 *  s_q_LH_HAA_);
    (*this)(5,2) = (- 0.3405 *  c_q_LH_HAA_);
    (*this)(5,4) =  c_q_LH_HAA_;
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK::Type_fr_base_X_fr_LH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,1) = ((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(1,0) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(2,0) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) = ((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(3,0) = ((((- 0.15 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.15 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,1) = (((( 0.15 + ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.15 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,2) = (( 0.25 *  c_q_LH_HFE_) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,4) = ((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(4,0) = (((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.15 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  c_q_LH_HAA_) + (( 0.15 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(4,1) = ((((( 0.25 *  c_q_LH_HAA_) - (( 0.15 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.15 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(4,2) = ((( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(4,3) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,4) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(4,5) =  c_q_LH_HAA_;
    (*this)(5,0) = (((((( 0.15 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  s_q_LH_HAA_) + (((- 0.15 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(5,1) = ((((( 0.25 *  s_q_LH_HAA_) + ((( 0.15 *  c_q_LH_HAA_) +  0.116) *  c_q_LH_HFE_)) + (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((( 0.15 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(5,2) = (((- 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(5,3) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,4) = ((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_base::Type_fr_LH_SHANK_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,2) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,0) = ((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(1,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(1,2) = ((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) =  c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(3,0) = ((((- 0.15 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.15 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,1) = (((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.15 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  c_q_LH_HAA_) + (( 0.15 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(3,2) = (((((( 0.15 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  s_q_LH_HAA_) + (((- 0.15 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,4) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,5) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,0) = (((( 0.15 + ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.15 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,1) = ((((( 0.25 *  c_q_LH_HAA_) - (( 0.15 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.15 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(4,2) = ((((( 0.25 *  s_q_LH_HAA_) + ((( 0.15 *  c_q_LH_HAA_) +  0.116) *  c_q_LH_HFE_)) + (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((( 0.15 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(4,3) = ((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(4,4) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(4,5) = ((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,0) = (( 0.25 *  c_q_LH_HFE_) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(5,1) = ((( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(5,2) = (((- 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(5,4) =  c_q_LH_HAA_;
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP::Type_fr_base_X_fr_RH_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0.116;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,0) =  s_q_RH_HAA_;
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(2,0) = - c_q_RH_HAA_;
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(3,0) = ( 0.116 *  c_q_RH_HAA_);
    (*this)(3,1) = (- 0.116 *  s_q_RH_HAA_);
    (*this)(4,0) = (- 0.277 *  c_q_RH_HAA_);
    (*this)(4,1) = ( 0.277 *  s_q_RH_HAA_);
    (*this)(4,3) =  s_q_RH_HAA_;
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(5,0) = (- 0.277 *  s_q_RH_HAA_);
    (*this)(5,1) = (- 0.277 *  c_q_RH_HAA_);
    (*this)(5,3) = - c_q_RH_HAA_;
    (*this)(5,4) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base::Type_fr_RH_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,3) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0.116;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,1) =  s_q_RH_HAA_;
    (*this)(0,2) = - c_q_RH_HAA_;
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(3,0) = ( 0.116 *  c_q_RH_HAA_);
    (*this)(3,1) = (- 0.277 *  c_q_RH_HAA_);
    (*this)(3,2) = (- 0.277 *  s_q_RH_HAA_);
    (*this)(3,4) =  s_q_RH_HAA_;
    (*this)(3,5) = - c_q_RH_HAA_;
    (*this)(4,0) = (- 0.116 *  s_q_RH_HAA_);
    (*this)(4,1) = ( 0.277 *  s_q_RH_HAA_);
    (*this)(4,2) = (- 0.277 *  c_q_RH_HAA_);
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH::Type_fr_base_X_fr_RH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) =  c_q_RH_HFE_;
    (*this)(0,1) = - s_q_RH_HFE_;
    (*this)(1,0) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(1,1) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(2,0) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(2,1) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(3,0) = ((( 0.116 *  c_q_RH_HAA_) +  0.041) *  s_q_RH_HFE_);
    (*this)(3,1) = ((( 0.116 *  c_q_RH_HAA_) +  0.041) *  c_q_RH_HFE_);
    (*this)(3,2) = (- 0.116 *  s_q_RH_HAA_);
    (*this)(3,3) =  c_q_RH_HFE_;
    (*this)(3,4) = - s_q_RH_HFE_;
    (*this)(4,0) = (((- 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.041 *  s_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(4,1) = ((( 0.041 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(4,2) = ( 0.3405 *  s_q_RH_HAA_);
    (*this)(4,3) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(4,4) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(4,5) =  c_q_RH_HAA_;
    (*this)(5,0) = ((( 0.116 + ( 0.041 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_));
    (*this)(5,1) = (((- 0.116 - ( 0.041 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(5,2) = (- 0.3405 *  c_q_RH_HAA_);
    (*this)(5,3) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(5,4) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_base::Type_fr_RH_THIGH_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) =  c_q_RH_HFE_;
    (*this)(0,1) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(0,2) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(1,0) = - s_q_RH_HFE_;
    (*this)(1,1) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,2) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,1) =  c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(3,0) = ((( 0.116 *  c_q_RH_HAA_) +  0.041) *  s_q_RH_HFE_);
    (*this)(3,1) = (((- 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.041 *  s_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(3,2) = ((( 0.116 + ( 0.041 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_));
    (*this)(3,3) =  c_q_RH_HFE_;
    (*this)(3,4) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(3,5) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(4,0) = ((( 0.116 *  c_q_RH_HAA_) +  0.041) *  c_q_RH_HFE_);
    (*this)(4,1) = ((( 0.041 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(4,2) = (((- 0.116 - ( 0.041 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(4,3) = - s_q_RH_HFE_;
    (*this)(4,4) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(4,5) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(5,0) = (- 0.116 *  s_q_RH_HAA_);
    (*this)(5,1) = ( 0.3405 *  s_q_RH_HAA_);
    (*this)(5,2) = (- 0.3405 *  c_q_RH_HAA_);
    (*this)(5,4) =  c_q_RH_HAA_;
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK::Type_fr_base_X_fr_RH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,1) = ((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(1,0) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(2,0) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) = ((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(3,0) = (((( 0.15 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.15 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,1) = ((((- 0.15 - ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.15 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,2) = (( 0.25 *  c_q_RH_HFE_) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,4) = ((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(4,0) = ((((( 0.15 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  c_q_RH_HAA_) - (( 0.15 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(4,1) = ((((( 0.25 *  c_q_RH_HAA_) + (( 0.15 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.15 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(4,2) = ((( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(4,3) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,4) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(4,5) =  c_q_RH_HAA_;
    (*this)(5,0) = ((((((- 0.15 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  s_q_RH_HAA_) + ((( 0.15 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(5,1) = ((((( 0.25 *  s_q_RH_HAA_) + (((- 0.15 *  c_q_RH_HAA_) -  0.116) *  c_q_RH_HFE_)) + (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.15 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(5,2) = (((- 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(5,3) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,4) = ((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_base::Type_fr_RH_SHANK_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,2) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,0) = ((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(1,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(1,2) = ((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) =  c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(3,0) = (((( 0.15 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.15 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,1) = ((((( 0.15 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  c_q_RH_HAA_) - (( 0.15 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(3,2) = ((((((- 0.15 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  s_q_RH_HAA_) + ((( 0.15 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,4) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,5) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,0) = ((((- 0.15 - ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.15 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,1) = ((((( 0.25 *  c_q_RH_HAA_) + (( 0.15 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.15 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(4,2) = ((((( 0.25 *  s_q_RH_HAA_) + (((- 0.15 *  c_q_RH_HAA_) -  0.116) *  c_q_RH_HFE_)) + (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.15 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(4,3) = ((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(4,4) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(4,5) = ((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,0) = (( 0.25 *  c_q_RH_HFE_) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(5,1) = ((( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(5,2) = (((- 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(5,4) =  c_q_RH_HAA_;
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_ADAPTER::Type_fr_base_X_fr_LF_ADAPTER()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_ADAPTER& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_ADAPTER::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,2) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(1,0) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,0) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(3,0) = ((((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,1) = ((((( 0.1 *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( 0.1 *  s_q_LF_HFE_) *  c_q_LF_KFE_)) + ( 0.25 *  c_q_LF_HFE_)) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(3,2) = ((((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.13 + ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,5) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(4,0) = ((((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.13 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  c_q_LF_HAA_) + (( 0.13 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(4,1) = (((((( 0.1 *  s_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) - ((( 0.1 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(4,2) = ((((((- 0.25 *  c_q_LF_HAA_) + (( 0.13 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.13 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.1 *  c_q_LF_HAA_));
    (*this)(4,3) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,0) = ((((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.13 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(5,1) = ((((((- 0.1 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.1 *  c_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.3405 *  c_q_LF_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.13 *  c_q_LF_HAA_) -  0.116) *  s_q_LF_HFE_) - (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.1 *  s_q_LF_HAA_));
    (*this)(5,3) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,4) =  s_q_LF_HAA_;
    (*this)(5,5) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_ADAPTER_X_fr_base::Type_fr_LF_ADAPTER_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(4,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_ADAPTER_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_ADAPTER_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,2) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(2,0) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(2,1) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(3,0) = ((((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,1) = ((((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.13 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  c_q_LF_HAA_) + (( 0.13 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(3,2) = ((((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.13 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,4) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,5) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,0) = ((((( 0.1 *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( 0.1 *  s_q_LF_HFE_) *  c_q_LF_KFE_)) + ( 0.25 *  c_q_LF_HFE_)) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(4,1) = (((((( 0.1 *  s_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) - ((( 0.1 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(4,2) = ((((((- 0.1 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.1 *  c_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.3405 *  c_q_LF_HAA_));
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
    (*this)(5,0) = ((((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.13 + ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,1) = ((((((- 0.25 *  c_q_LF_HAA_) + (( 0.13 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.13 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.1 *  c_q_LF_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.13 *  c_q_LF_HAA_) -  0.116) *  s_q_LF_HFE_) - (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.1 *  s_q_LF_HAA_));
    (*this)(5,3) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(5,4) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,5) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT::Type_fr_base_X_fr_LF_FOOT()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,2) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(1,0) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,0) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(3,0) = ((((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,1) = (((((( 0.1 *  c_q_LF_HFE_) - ( 0.32125 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.32125 *  c_q_LF_HFE_) + ( 0.1 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.25 *  c_q_LF_HFE_)) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(3,2) = ((((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.13 + ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,5) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(4,0) = (((((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.13 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  c_q_LF_HAA_) + (( 0.13 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.32125 *  c_q_LF_HAA_));
    (*this)(4,1) = ((((((( 0.32125 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.1 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.32125 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.1 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(4,2) = ((((((- 0.25 *  c_q_LF_HAA_) + (( 0.13 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.13 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.1 *  c_q_LF_HAA_));
    (*this)(4,3) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,0) = (((((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.13 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.32125 *  s_q_LF_HAA_));
    (*this)(5,1) = (((((((- 0.32125 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.1 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.1 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.32125 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.3405 *  c_q_LF_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.13 *  c_q_LF_HAA_) -  0.116) *  s_q_LF_HFE_) - (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.1 *  s_q_LF_HAA_));
    (*this)(5,3) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,4) =  s_q_LF_HAA_;
    (*this)(5,5) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base::Type_fr_LF_FOOT_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(4,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,2) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(2,0) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(2,1) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(3,0) = ((((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,1) = (((((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.13 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  c_q_LF_HAA_) + (( 0.13 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.32125 *  c_q_LF_HAA_));
    (*this)(3,2) = (((((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.13 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.32125 *  s_q_LF_HAA_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,4) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,5) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,0) = (((((( 0.1 *  c_q_LF_HFE_) - ( 0.32125 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.32125 *  c_q_LF_HFE_) + ( 0.1 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.25 *  c_q_LF_HFE_)) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(4,1) = ((((((( 0.32125 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.1 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.32125 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.1 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(4,2) = (((((((- 0.32125 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.1 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.1 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.32125 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.3405 *  c_q_LF_HAA_));
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
    (*this)(5,0) = ((((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.13 + ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,1) = ((((((- 0.25 *  c_q_LF_HAA_) + (( 0.13 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.13 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.1 *  c_q_LF_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.13 *  c_q_LF_HAA_) -  0.116) *  s_q_LF_HFE_) - (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.1 *  s_q_LF_HAA_));
    (*this)(5,3) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(5,4) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,5) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP_COM::Type_fr_base_X_fr_LF_HIP_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,3) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP_COM& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP_COM::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = - s_q_LF_HAA_;
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) =  c_q_LF_HAA_;
    (*this)(3,1) = (( 0.116 *  s_q_LF_HAA_) +  1.5E-4);
    (*this)(3,2) = (( 0.116 *  c_q_LF_HAA_) -  0.00379);
    (*this)(4,0) = ((- 0.00379 *  s_q_LF_HAA_) - ( 1.5E-4 *  c_q_LF_HAA_));
    (*this)(4,1) = (- 0.34152 *  s_q_LF_HAA_);
    (*this)(4,2) = (- 0.34152 *  c_q_LF_HAA_);
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) = - s_q_LF_HAA_;
    (*this)(5,0) = (((- 1.5E-4 *  s_q_LF_HAA_) + ( 0.00379 *  c_q_LF_HAA_)) -  0.116);
    (*this)(5,1) = ( 0.34152 *  c_q_LF_HAA_);
    (*this)(5,2) = (- 0.34152 *  s_q_LF_HAA_);
    (*this)(5,4) =  s_q_LF_HAA_;
    (*this)(5,5) =  c_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_HIP_COM_X_fr_base::Type_fr_LF_HIP_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,3) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_HIP_COM_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_HIP_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(2,1) = - s_q_LF_HAA_;
    (*this)(2,2) =  c_q_LF_HAA_;
    (*this)(3,1) = ((- 0.00379 *  s_q_LF_HAA_) - ( 1.5E-4 *  c_q_LF_HAA_));
    (*this)(3,2) = (((- 1.5E-4 *  s_q_LF_HAA_) + ( 0.00379 *  c_q_LF_HAA_)) -  0.116);
    (*this)(4,0) = (( 0.116 *  s_q_LF_HAA_) +  1.5E-4);
    (*this)(4,1) = (- 0.34152 *  s_q_LF_HAA_);
    (*this)(4,2) = ( 0.34152 *  c_q_LF_HAA_);
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
    (*this)(5,0) = (( 0.116 *  c_q_LF_HAA_) -  0.00379);
    (*this)(5,1) = (- 0.34152 *  c_q_LF_HAA_);
    (*this)(5,2) = (- 0.34152 *  s_q_LF_HAA_);
    (*this)(5,4) = - s_q_LF_HAA_;
    (*this)(5,5) =  c_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK_COM::Type_fr_base_X_fr_LF_SHANK_COM()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK_COM& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK_COM::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,2) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(1,0) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,0) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(3,0) = ((((- 0.13918 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.13918 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,1) = (((((( 0.05873 *  c_q_LF_HFE_) - ( 0.09806 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.09806 *  c_q_LF_HFE_) + ( 0.05873 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.25 *  c_q_LF_HFE_)) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(3,2) = ((((- 0.13918 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.13918 + ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,5) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(4,0) = (((((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.13918 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  c_q_LF_HAA_) + (( 0.13918 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.09806 *  c_q_LF_HAA_));
    (*this)(4,1) = ((((((( 0.09806 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.05873 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.09806 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.05873 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(4,2) = ((((((- 0.25 *  c_q_LF_HAA_) + (( 0.13918 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.13918 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.05873 *  c_q_LF_HAA_));
    (*this)(4,3) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,0) = (((((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.13918 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13918 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.09806 *  s_q_LF_HAA_));
    (*this)(5,1) = (((((((- 0.09806 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.05873 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.05873 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.09806 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.3405 *  c_q_LF_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13918 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.13918 *  c_q_LF_HAA_) -  0.116) *  s_q_LF_HFE_) - (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.05873 *  s_q_LF_HAA_));
    (*this)(5,3) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,4) =  s_q_LF_HAA_;
    (*this)(5,5) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_SHANK_COM_X_fr_base::Type_fr_LF_SHANK_COM_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(4,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_SHANK_COM_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_SHANK_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,2) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(2,0) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(2,1) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(3,0) = ((((- 0.13918 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.13918 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,1) = (((((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.13918 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  c_q_LF_HAA_) + (( 0.13918 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.09806 *  c_q_LF_HAA_));
    (*this)(3,2) = (((((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.13918 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13918 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.09806 *  s_q_LF_HAA_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,4) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,5) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,0) = (((((( 0.05873 *  c_q_LF_HFE_) - ( 0.09806 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.09806 *  c_q_LF_HFE_) + ( 0.05873 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.25 *  c_q_LF_HFE_)) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(4,1) = ((((((( 0.09806 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.05873 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.09806 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.05873 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(4,2) = (((((((- 0.09806 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.05873 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.05873 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.09806 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.3405 *  c_q_LF_HAA_));
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
    (*this)(5,0) = ((((- 0.13918 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.13918 + ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,1) = ((((((- 0.25 *  c_q_LF_HAA_) + (( 0.13918 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.13918 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.05873 *  c_q_LF_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13918 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.13918 *  c_q_LF_HAA_) -  0.116) *  s_q_LF_HFE_) - (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.05873 *  s_q_LF_HAA_));
    (*this)(5,3) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(5,4) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,5) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH_COM::Type_fr_base_X_fr_LF_THIGH_COM()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH_COM& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH_COM::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) =  c_q_LF_HFE_;
    (*this)(0,2) =  s_q_LF_HFE_;
    (*this)(1,0) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = (- s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,0) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) = ( c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(3,0) = (((- 0.116 *  c_q_LF_HAA_) -  0.09523) *  s_q_LF_HFE_);
    (*this)(3,1) = (((- 0.0039 *  s_q_LF_HFE_) + ( 0.21458 *  c_q_LF_HFE_)) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(3,2) = ((( 0.116 *  c_q_LF_HAA_) +  0.09523) *  c_q_LF_HFE_);
    (*this)(3,3) =  c_q_LF_HFE_;
    (*this)(3,5) =  s_q_LF_HFE_;
    (*this)(4,0) = (((( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.09523 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.21458 *  c_q_LF_HAA_));
    (*this)(4,1) = (((( 0.21458 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.0039 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(4,2) = (((( 0.09523 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.0039 *  c_q_LF_HAA_));
    (*this)(4,3) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) = (- s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(5,0) = (((( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((- 0.116 - ( 0.09523 *  c_q_LF_HAA_)) *  c_q_LF_HFE_)) - ( 0.21458 *  s_q_LF_HAA_));
    (*this)(5,1) = ((((- 0.21458 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.0039 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.3405 *  c_q_LF_HAA_));
    (*this)(5,2) = ((((- 0.116 - ( 0.09523 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) - (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.0039 *  s_q_LF_HAA_));
    (*this)(5,3) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(5,4) =  s_q_LF_HAA_;
    (*this)(5,5) = ( c_q_LF_HAA_ *  c_q_LF_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_COM_X_fr_base::Type_fr_LF_THIGH_COM_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(4,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_COM_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) =  c_q_LF_HFE_;
    (*this)(0,1) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(0,2) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(2,0) =  s_q_LF_HFE_;
    (*this)(2,1) = (- s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,2) = ( c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(3,0) = (((- 0.116 *  c_q_LF_HAA_) -  0.09523) *  s_q_LF_HFE_);
    (*this)(3,1) = (((( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.09523 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.21458 *  c_q_LF_HAA_));
    (*this)(3,2) = (((( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((- 0.116 - ( 0.09523 *  c_q_LF_HAA_)) *  c_q_LF_HFE_)) - ( 0.21458 *  s_q_LF_HAA_));
    (*this)(3,3) =  c_q_LF_HFE_;
    (*this)(3,4) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(3,5) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(4,0) = (((- 0.0039 *  s_q_LF_HFE_) + ( 0.21458 *  c_q_LF_HFE_)) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(4,1) = (((( 0.21458 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.0039 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(4,2) = ((((- 0.21458 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.0039 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.3405 *  c_q_LF_HAA_));
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
    (*this)(5,0) = ((( 0.116 *  c_q_LF_HAA_) +  0.09523) *  c_q_LF_HFE_);
    (*this)(5,1) = (((( 0.09523 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.0039 *  c_q_LF_HAA_));
    (*this)(5,2) = ((((- 0.116 - ( 0.09523 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) - (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.0039 *  s_q_LF_HAA_));
    (*this)(5,3) =  s_q_LF_HFE_;
    (*this)(5,4) = (- s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(5,5) = ( c_q_LF_HAA_ *  c_q_LF_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_ADAPTER::Type_fr_base_X_fr_LH_ADAPTER()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_ADAPTER& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_ADAPTER::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,2) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(1,0) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,0) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(3,0) = ((((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,1) = (((((- 0.1 *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( 0.1 *  s_q_LH_HFE_) *  c_q_LH_KFE_)) + ( 0.25 *  c_q_LH_HFE_)) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(3,2) = ((((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.13 + ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,5) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(4,0) = (((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.13 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  c_q_LH_HAA_) + (( 0.13 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(4,1) = ((((((- 0.1 *  s_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.1 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(4,2) = ((((((- 0.25 *  c_q_LH_HAA_) + (( 0.13 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.13 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.1 *  c_q_LH_HAA_));
    (*this)(4,3) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,0) = (((((( 0.13 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(5,1) = (((((( 0.1 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) - ((( 0.1 *  c_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) - (( 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.1 *  s_q_LH_HAA_));
    (*this)(5,3) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,4) =  s_q_LH_HAA_;
    (*this)(5,5) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_ADAPTER_X_fr_base::Type_fr_LH_ADAPTER_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(4,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_ADAPTER_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_ADAPTER_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,2) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(2,0) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(2,1) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(3,0) = ((((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,1) = (((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.13 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  c_q_LH_HAA_) + (( 0.13 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(3,2) = (((((( 0.13 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,4) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,5) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,0) = (((((- 0.1 *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( 0.1 *  s_q_LH_HFE_) *  c_q_LH_KFE_)) + ( 0.25 *  c_q_LH_HFE_)) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(4,1) = ((((((- 0.1 *  s_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.1 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(4,2) = (((((( 0.1 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) - ((( 0.1 *  c_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) - (( 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
    (*this)(5,0) = ((((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.13 + ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,1) = ((((((- 0.25 *  c_q_LH_HAA_) + (( 0.13 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.13 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.1 *  c_q_LH_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.1 *  s_q_LH_HAA_));
    (*this)(5,3) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(5,4) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,5) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT::Type_fr_base_X_fr_LH_FOOT()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,2) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(1,0) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,0) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(3,0) = ((((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,1) = ((((((- 0.1 *  c_q_LH_HFE_) - ( 0.32125 *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.32125 *  c_q_LH_HFE_) - ( 0.1 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.25 *  c_q_LH_HFE_)) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(3,2) = ((((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.13 + ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,5) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(4,0) = ((((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.13 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  c_q_LH_HAA_) + (( 0.13 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.32125 *  c_q_LH_HAA_));
    (*this)(4,1) = ((((((( 0.32125 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.1 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.1 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.32125 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(4,2) = ((((((- 0.25 *  c_q_LH_HAA_) + (( 0.13 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.13 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.1 *  c_q_LH_HAA_));
    (*this)(4,3) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,0) = ((((((( 0.13 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.32125 *  s_q_LH_HAA_));
    (*this)(5,1) = ((((((( 0.1 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.32125 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.1 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.32125 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.1 *  s_q_LH_HAA_));
    (*this)(5,3) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,4) =  s_q_LH_HAA_;
    (*this)(5,5) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base::Type_fr_LH_FOOT_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(4,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,2) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(2,0) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(2,1) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(3,0) = ((((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,1) = ((((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.13 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  c_q_LH_HAA_) + (( 0.13 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.32125 *  c_q_LH_HAA_));
    (*this)(3,2) = ((((((( 0.13 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.32125 *  s_q_LH_HAA_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,4) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,5) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,0) = ((((((- 0.1 *  c_q_LH_HFE_) - ( 0.32125 *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.32125 *  c_q_LH_HFE_) - ( 0.1 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.25 *  c_q_LH_HFE_)) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(4,1) = ((((((( 0.32125 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.1 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.1 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.32125 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(4,2) = ((((((( 0.1 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.32125 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.1 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.32125 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
    (*this)(5,0) = ((((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.13 + ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,1) = ((((((- 0.25 *  c_q_LH_HAA_) + (( 0.13 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.13 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.1 *  c_q_LH_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.1 *  s_q_LH_HAA_));
    (*this)(5,3) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(5,4) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,5) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP_COM::Type_fr_base_X_fr_LH_HIP_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,3) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP_COM& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP_COM::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = - s_q_LH_HAA_;
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) =  c_q_LH_HAA_;
    (*this)(3,1) = (( 0.116 *  s_q_LH_HAA_) +  1.5E-4);
    (*this)(3,2) = (( 0.116 *  c_q_LH_HAA_) -  0.00379);
    (*this)(4,0) = ((- 0.00379 *  s_q_LH_HAA_) - ( 1.5E-4 *  c_q_LH_HAA_));
    (*this)(4,1) = ( 0.34152 *  s_q_LH_HAA_);
    (*this)(4,2) = ( 0.34152 *  c_q_LH_HAA_);
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) = - s_q_LH_HAA_;
    (*this)(5,0) = (((- 1.5E-4 *  s_q_LH_HAA_) + ( 0.00379 *  c_q_LH_HAA_)) -  0.116);
    (*this)(5,1) = (- 0.34152 *  c_q_LH_HAA_);
    (*this)(5,2) = ( 0.34152 *  s_q_LH_HAA_);
    (*this)(5,4) =  s_q_LH_HAA_;
    (*this)(5,5) =  c_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_HIP_COM_X_fr_base::Type_fr_LH_HIP_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,3) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_HIP_COM_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_HIP_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(2,1) = - s_q_LH_HAA_;
    (*this)(2,2) =  c_q_LH_HAA_;
    (*this)(3,1) = ((- 0.00379 *  s_q_LH_HAA_) - ( 1.5E-4 *  c_q_LH_HAA_));
    (*this)(3,2) = (((- 1.5E-4 *  s_q_LH_HAA_) + ( 0.00379 *  c_q_LH_HAA_)) -  0.116);
    (*this)(4,0) = (( 0.116 *  s_q_LH_HAA_) +  1.5E-4);
    (*this)(4,1) = ( 0.34152 *  s_q_LH_HAA_);
    (*this)(4,2) = (- 0.34152 *  c_q_LH_HAA_);
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
    (*this)(5,0) = (( 0.116 *  c_q_LH_HAA_) -  0.00379);
    (*this)(5,1) = ( 0.34152 *  c_q_LH_HAA_);
    (*this)(5,2) = ( 0.34152 *  s_q_LH_HAA_);
    (*this)(5,4) = - s_q_LH_HAA_;
    (*this)(5,5) =  c_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK_COM::Type_fr_base_X_fr_LH_SHANK_COM()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK_COM& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK_COM::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,2) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(1,0) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,0) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(3,0) = ((((- 0.13918 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.13918 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,1) = ((((((- 0.05873 *  c_q_LH_HFE_) - ( 0.09806 *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.09806 *  c_q_LH_HFE_) - ( 0.05873 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.25 *  c_q_LH_HFE_)) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(3,2) = ((((- 0.13918 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.13918 + ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,5) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(4,0) = ((((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.13918 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  c_q_LH_HAA_) + (( 0.13918 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.09806 *  c_q_LH_HAA_));
    (*this)(4,1) = ((((((( 0.09806 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.05873 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.05873 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.09806 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(4,2) = ((((((- 0.25 *  c_q_LH_HAA_) + (( 0.13918 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.13918 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.05873 *  c_q_LH_HAA_));
    (*this)(4,3) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,0) = ((((((( 0.13918 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13918 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.09806 *  s_q_LH_HAA_));
    (*this)(5,1) = ((((((( 0.05873 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.09806 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.05873 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.09806 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13918 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (((- 0.13918 *  c_q_LH_HAA_) -  0.116) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.05873 *  s_q_LH_HAA_));
    (*this)(5,3) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,4) =  s_q_LH_HAA_;
    (*this)(5,5) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_SHANK_COM_X_fr_base::Type_fr_LH_SHANK_COM_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(4,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_SHANK_COM_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_SHANK_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,2) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(2,0) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(2,1) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(3,0) = ((((- 0.13918 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.13918 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,1) = ((((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.13918 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  c_q_LH_HAA_) + (( 0.13918 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.09806 *  c_q_LH_HAA_));
    (*this)(3,2) = ((((((( 0.13918 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13918 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.09806 *  s_q_LH_HAA_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,4) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,5) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,0) = ((((((- 0.05873 *  c_q_LH_HFE_) - ( 0.09806 *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.09806 *  c_q_LH_HFE_) - ( 0.05873 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.25 *  c_q_LH_HFE_)) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(4,1) = ((((((( 0.09806 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.05873 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.05873 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.09806 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(4,2) = ((((((( 0.05873 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.09806 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.05873 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.09806 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
    (*this)(5,0) = ((((- 0.13918 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.13918 + ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,1) = ((((((- 0.25 *  c_q_LH_HAA_) + (( 0.13918 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.13918 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.05873 *  c_q_LH_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13918 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (((- 0.13918 *  c_q_LH_HAA_) -  0.116) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.05873 *  s_q_LH_HAA_));
    (*this)(5,3) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(5,4) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,5) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH_COM::Type_fr_base_X_fr_LH_THIGH_COM()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH_COM& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH_COM::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) =  c_q_LH_HFE_;
    (*this)(0,2) =  s_q_LH_HFE_;
    (*this)(1,0) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = (- s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,0) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) = ( c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(3,0) = (((- 0.116 *  c_q_LH_HAA_) -  0.09523) *  s_q_LH_HFE_);
    (*this)(3,1) = ((( 0.0039 *  s_q_LH_HFE_) + ( 0.21458 *  c_q_LH_HFE_)) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(3,2) = ((( 0.116 *  c_q_LH_HAA_) +  0.09523) *  c_q_LH_HFE_);
    (*this)(3,3) =  c_q_LH_HFE_;
    (*this)(3,5) =  s_q_LH_HFE_;
    (*this)(4,0) = ((((- 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.09523 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.21458 *  c_q_LH_HAA_));
    (*this)(4,1) = (((( 0.21458 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.0039 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(4,2) = (((( 0.09523 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.0039 *  c_q_LH_HAA_));
    (*this)(4,3) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) = (- s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(5,0) = ((((- 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((- 0.116 - ( 0.09523 *  c_q_LH_HAA_)) *  c_q_LH_HFE_)) - ( 0.21458 *  s_q_LH_HAA_));
    (*this)(5,1) = ((((- 0.21458 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.0039 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(5,2) = ((((- 0.116 - ( 0.09523 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) + (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.0039 *  s_q_LH_HAA_));
    (*this)(5,3) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(5,4) =  s_q_LH_HAA_;
    (*this)(5,5) = ( c_q_LH_HAA_ *  c_q_LH_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_COM_X_fr_base::Type_fr_LH_THIGH_COM_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(4,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_COM_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) =  c_q_LH_HFE_;
    (*this)(0,1) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(0,2) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(2,0) =  s_q_LH_HFE_;
    (*this)(2,1) = (- s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,2) = ( c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(3,0) = (((- 0.116 *  c_q_LH_HAA_) -  0.09523) *  s_q_LH_HFE_);
    (*this)(3,1) = ((((- 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.09523 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.21458 *  c_q_LH_HAA_));
    (*this)(3,2) = ((((- 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((- 0.116 - ( 0.09523 *  c_q_LH_HAA_)) *  c_q_LH_HFE_)) - ( 0.21458 *  s_q_LH_HAA_));
    (*this)(3,3) =  c_q_LH_HFE_;
    (*this)(3,4) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(3,5) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(4,0) = ((( 0.0039 *  s_q_LH_HFE_) + ( 0.21458 *  c_q_LH_HFE_)) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(4,1) = (((( 0.21458 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.0039 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(4,2) = ((((- 0.21458 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.0039 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
    (*this)(5,0) = ((( 0.116 *  c_q_LH_HAA_) +  0.09523) *  c_q_LH_HFE_);
    (*this)(5,1) = (((( 0.09523 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.0039 *  c_q_LH_HAA_));
    (*this)(5,2) = ((((- 0.116 - ( 0.09523 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) + (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.0039 *  s_q_LH_HAA_));
    (*this)(5,3) =  s_q_LH_HFE_;
    (*this)(5,4) = (- s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(5,5) = ( c_q_LH_HAA_ *  c_q_LH_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_ADAPTER::Type_fr_base_X_fr_RF_ADAPTER()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_ADAPTER& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_ADAPTER::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,2) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(1,0) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,0) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(3,0) = (((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,1) = ((((( 0.1 *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( 0.1 *  s_q_RF_HFE_) *  c_q_RF_KFE_)) + ( 0.25 *  c_q_RF_HFE_)) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(3,2) = (((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((- 0.13 - ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,5) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(4,0) = ((((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.13 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  c_q_RF_HAA_) - (( 0.13 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(4,1) = (((((( 0.1 *  s_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) - ((( 0.1 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(4,2) = ((((((- 0.25 *  c_q_RF_HAA_) - (( 0.13 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.13 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.1 *  c_q_RF_HAA_));
    (*this)(4,3) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,0) = ((((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.13 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(5,1) = ((((((- 0.1 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.1 *  c_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.3405 *  c_q_RF_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.13 *  c_q_RF_HAA_) +  0.116) *  s_q_RF_HFE_) - (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.1 *  s_q_RF_HAA_));
    (*this)(5,3) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,4) =  s_q_RF_HAA_;
    (*this)(5,5) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_ADAPTER_X_fr_base::Type_fr_RF_ADAPTER_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(4,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_ADAPTER_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_ADAPTER_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,2) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(2,0) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(2,1) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(3,0) = (((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,1) = ((((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.13 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  c_q_RF_HAA_) - (( 0.13 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(3,2) = ((((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.13 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,4) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,5) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,0) = ((((( 0.1 *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( 0.1 *  s_q_RF_HFE_) *  c_q_RF_KFE_)) + ( 0.25 *  c_q_RF_HFE_)) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(4,1) = (((((( 0.1 *  s_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) - ((( 0.1 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(4,2) = ((((((- 0.1 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.1 *  c_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.3405 *  c_q_RF_HAA_));
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
    (*this)(5,0) = (((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((- 0.13 - ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,1) = ((((((- 0.25 *  c_q_RF_HAA_) - (( 0.13 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.13 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.1 *  c_q_RF_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.13 *  c_q_RF_HAA_) +  0.116) *  s_q_RF_HFE_) - (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.1 *  s_q_RF_HAA_));
    (*this)(5,3) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(5,4) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,5) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT::Type_fr_base_X_fr_RF_FOOT()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,2) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(1,0) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,0) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(3,0) = (((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,1) = (((((( 0.1 *  c_q_RF_HFE_) - ( 0.32125 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.32125 *  c_q_RF_HFE_) + ( 0.1 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.25 *  c_q_RF_HFE_)) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(3,2) = (((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((- 0.13 - ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,5) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(4,0) = (((((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.13 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  c_q_RF_HAA_) - (( 0.13 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.32125 *  c_q_RF_HAA_));
    (*this)(4,1) = ((((((( 0.32125 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.1 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.32125 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.1 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(4,2) = ((((((- 0.25 *  c_q_RF_HAA_) - (( 0.13 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.13 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.1 *  c_q_RF_HAA_));
    (*this)(4,3) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,0) = (((((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.13 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.32125 *  s_q_RF_HAA_));
    (*this)(5,1) = (((((((- 0.32125 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.1 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.1 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.32125 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.3405 *  c_q_RF_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.13 *  c_q_RF_HAA_) +  0.116) *  s_q_RF_HFE_) - (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.1 *  s_q_RF_HAA_));
    (*this)(5,3) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,4) =  s_q_RF_HAA_;
    (*this)(5,5) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base::Type_fr_RF_FOOT_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(4,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,2) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(2,0) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(2,1) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(3,0) = (((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,1) = (((((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.13 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  c_q_RF_HAA_) - (( 0.13 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.32125 *  c_q_RF_HAA_));
    (*this)(3,2) = (((((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.13 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.32125 *  s_q_RF_HAA_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,4) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,5) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,0) = (((((( 0.1 *  c_q_RF_HFE_) - ( 0.32125 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.32125 *  c_q_RF_HFE_) + ( 0.1 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.25 *  c_q_RF_HFE_)) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(4,1) = ((((((( 0.32125 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.1 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.32125 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.1 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(4,2) = (((((((- 0.32125 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.1 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.1 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.32125 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.3405 *  c_q_RF_HAA_));
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
    (*this)(5,0) = (((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((- 0.13 - ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,1) = ((((((- 0.25 *  c_q_RF_HAA_) - (( 0.13 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.13 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.1 *  c_q_RF_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.13 *  c_q_RF_HAA_) +  0.116) *  s_q_RF_HFE_) - (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.1 *  s_q_RF_HAA_));
    (*this)(5,3) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(5,4) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,5) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP_COM::Type_fr_base_X_fr_RF_HIP_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,3) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP_COM& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP_COM::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = - s_q_RF_HAA_;
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) =  c_q_RF_HAA_;
    (*this)(3,1) = ( 1.5E-4 - ( 0.116 *  s_q_RF_HAA_));
    (*this)(3,2) = ( 0.00379 - ( 0.116 *  c_q_RF_HAA_));
    (*this)(4,0) = (( 0.00379 *  s_q_RF_HAA_) - ( 1.5E-4 *  c_q_RF_HAA_));
    (*this)(4,1) = (- 0.34152 *  s_q_RF_HAA_);
    (*this)(4,2) = (- 0.34152 *  c_q_RF_HAA_);
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) = - s_q_RF_HAA_;
    (*this)(5,0) = (((- 1.5E-4 *  s_q_RF_HAA_) - ( 0.00379 *  c_q_RF_HAA_)) +  0.116);
    (*this)(5,1) = ( 0.34152 *  c_q_RF_HAA_);
    (*this)(5,2) = (- 0.34152 *  s_q_RF_HAA_);
    (*this)(5,4) =  s_q_RF_HAA_;
    (*this)(5,5) =  c_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_HIP_COM_X_fr_base::Type_fr_RF_HIP_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,3) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_HIP_COM_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_HIP_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(2,1) = - s_q_RF_HAA_;
    (*this)(2,2) =  c_q_RF_HAA_;
    (*this)(3,1) = (( 0.00379 *  s_q_RF_HAA_) - ( 1.5E-4 *  c_q_RF_HAA_));
    (*this)(3,2) = (((- 1.5E-4 *  s_q_RF_HAA_) - ( 0.00379 *  c_q_RF_HAA_)) +  0.116);
    (*this)(4,0) = ( 1.5E-4 - ( 0.116 *  s_q_RF_HAA_));
    (*this)(4,1) = (- 0.34152 *  s_q_RF_HAA_);
    (*this)(4,2) = ( 0.34152 *  c_q_RF_HAA_);
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
    (*this)(5,0) = ( 0.00379 - ( 0.116 *  c_q_RF_HAA_));
    (*this)(5,1) = (- 0.34152 *  c_q_RF_HAA_);
    (*this)(5,2) = (- 0.34152 *  s_q_RF_HAA_);
    (*this)(5,4) = - s_q_RF_HAA_;
    (*this)(5,5) =  c_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK_COM::Type_fr_base_X_fr_RF_SHANK_COM()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK_COM& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK_COM::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,2) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(1,0) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,0) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(3,0) = (((( 0.13918 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.13918 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,1) = (((((( 0.05873 *  c_q_RF_HFE_) - ( 0.09806 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.09806 *  c_q_RF_HFE_) + ( 0.05873 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.25 *  c_q_RF_HFE_)) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(3,2) = (((( 0.13918 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((- 0.13918 - ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,5) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(4,0) = (((((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.13918 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  c_q_RF_HAA_) - (( 0.13918 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.09806 *  c_q_RF_HAA_));
    (*this)(4,1) = ((((((( 0.09806 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.05873 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.09806 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.05873 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(4,2) = ((((((- 0.25 *  c_q_RF_HAA_) - (( 0.13918 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.13918 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.05873 *  c_q_RF_HAA_));
    (*this)(4,3) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,0) = (((((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.13918 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13918 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.09806 *  s_q_RF_HAA_));
    (*this)(5,1) = (((((((- 0.09806 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.05873 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.05873 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.09806 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.3405 *  c_q_RF_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13918 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.13918 *  c_q_RF_HAA_) +  0.116) *  s_q_RF_HFE_) - (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.05873 *  s_q_RF_HAA_));
    (*this)(5,3) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,4) =  s_q_RF_HAA_;
    (*this)(5,5) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_SHANK_COM_X_fr_base::Type_fr_RF_SHANK_COM_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(4,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_SHANK_COM_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_SHANK_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,2) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(2,0) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(2,1) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(3,0) = (((( 0.13918 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.13918 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,1) = (((((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.13918 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  c_q_RF_HAA_) - (( 0.13918 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.09806 *  c_q_RF_HAA_));
    (*this)(3,2) = (((((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.13918 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13918 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.09806 *  s_q_RF_HAA_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,4) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,5) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,0) = (((((( 0.05873 *  c_q_RF_HFE_) - ( 0.09806 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.09806 *  c_q_RF_HFE_) + ( 0.05873 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.25 *  c_q_RF_HFE_)) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(4,1) = ((((((( 0.09806 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.05873 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.09806 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.05873 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(4,2) = (((((((- 0.09806 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.05873 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.05873 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.09806 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.3405 *  c_q_RF_HAA_));
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
    (*this)(5,0) = (((( 0.13918 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((- 0.13918 - ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,1) = ((((((- 0.25 *  c_q_RF_HAA_) - (( 0.13918 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.13918 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.05873 *  c_q_RF_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13918 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.13918 *  c_q_RF_HAA_) +  0.116) *  s_q_RF_HFE_) - (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.05873 *  s_q_RF_HAA_));
    (*this)(5,3) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(5,4) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,5) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH_COM::Type_fr_base_X_fr_RF_THIGH_COM()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH_COM& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH_COM::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) =  c_q_RF_HFE_;
    (*this)(0,2) =  s_q_RF_HFE_;
    (*this)(1,0) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = (- s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,0) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) = ( c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(3,0) = ((( 0.116 *  c_q_RF_HAA_) +  0.09523) *  s_q_RF_HFE_);
    (*this)(3,1) = (((- 0.0039 *  s_q_RF_HFE_) + ( 0.21458 *  c_q_RF_HFE_)) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(3,2) = (((- 0.116 *  c_q_RF_HAA_) -  0.09523) *  c_q_RF_HFE_);
    (*this)(3,3) =  c_q_RF_HFE_;
    (*this)(3,5) =  s_q_RF_HFE_;
    (*this)(4,0) = (((( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.09523 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.21458 *  c_q_RF_HAA_));
    (*this)(4,1) = (((( 0.21458 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.0039 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(4,2) = ((((- 0.09523 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.0039 *  c_q_RF_HAA_));
    (*this)(4,3) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) = (- s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(5,0) = (((( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.116 + ( 0.09523 *  c_q_RF_HAA_)) *  c_q_RF_HFE_)) - ( 0.21458 *  s_q_RF_HAA_));
    (*this)(5,1) = ((((- 0.21458 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.0039 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.3405 *  c_q_RF_HAA_));
    (*this)(5,2) = (((( 0.116 + ( 0.09523 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) - (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.0039 *  s_q_RF_HAA_));
    (*this)(5,3) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(5,4) =  s_q_RF_HAA_;
    (*this)(5,5) = ( c_q_RF_HAA_ *  c_q_RF_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_COM_X_fr_base::Type_fr_RF_THIGH_COM_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(4,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_COM_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) =  c_q_RF_HFE_;
    (*this)(0,1) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(0,2) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(2,0) =  s_q_RF_HFE_;
    (*this)(2,1) = (- s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,2) = ( c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(3,0) = ((( 0.116 *  c_q_RF_HAA_) +  0.09523) *  s_q_RF_HFE_);
    (*this)(3,1) = (((( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.09523 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.21458 *  c_q_RF_HAA_));
    (*this)(3,2) = (((( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.116 + ( 0.09523 *  c_q_RF_HAA_)) *  c_q_RF_HFE_)) - ( 0.21458 *  s_q_RF_HAA_));
    (*this)(3,3) =  c_q_RF_HFE_;
    (*this)(3,4) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(3,5) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(4,0) = (((- 0.0039 *  s_q_RF_HFE_) + ( 0.21458 *  c_q_RF_HFE_)) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(4,1) = (((( 0.21458 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.0039 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(4,2) = ((((- 0.21458 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.0039 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.3405 *  c_q_RF_HAA_));
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
    (*this)(5,0) = (((- 0.116 *  c_q_RF_HAA_) -  0.09523) *  c_q_RF_HFE_);
    (*this)(5,1) = ((((- 0.09523 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.0039 *  c_q_RF_HAA_));
    (*this)(5,2) = (((( 0.116 + ( 0.09523 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) - (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.0039 *  s_q_RF_HAA_));
    (*this)(5,3) =  s_q_RF_HFE_;
    (*this)(5,4) = (- s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(5,5) = ( c_q_RF_HAA_ *  c_q_RF_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_ADAPTER::Type_fr_base_X_fr_RH_ADAPTER()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_ADAPTER& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_ADAPTER::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,2) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(1,0) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,0) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(3,0) = (((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,1) = (((((- 0.1 *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( 0.1 *  s_q_RH_HFE_) *  c_q_RH_KFE_)) + ( 0.25 *  c_q_RH_HFE_)) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(3,2) = (((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((- 0.13 - ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,5) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(4,0) = ((((( 0.13 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  c_q_RH_HAA_) - (( 0.13 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(4,1) = ((((((- 0.1 *  s_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.1 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(4,2) = ((((((- 0.25 *  c_q_RH_HAA_) - (( 0.13 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.13 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.1 *  c_q_RH_HAA_));
    (*this)(4,3) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,0) = ((((((- 0.13 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(5,1) = (((((( 0.1 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) - ((( 0.1 *  c_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) - (( 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.1 *  s_q_RH_HAA_));
    (*this)(5,3) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,4) =  s_q_RH_HAA_;
    (*this)(5,5) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_ADAPTER_X_fr_base::Type_fr_RH_ADAPTER_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(4,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_ADAPTER_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_ADAPTER_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,2) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(2,0) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(2,1) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(3,0) = (((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,1) = ((((( 0.13 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  c_q_RH_HAA_) - (( 0.13 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(3,2) = ((((((- 0.13 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,4) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,5) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,0) = (((((- 0.1 *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( 0.1 *  s_q_RH_HFE_) *  c_q_RH_KFE_)) + ( 0.25 *  c_q_RH_HFE_)) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(4,1) = ((((((- 0.1 *  s_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.1 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(4,2) = (((((( 0.1 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) - ((( 0.1 *  c_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) - (( 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
    (*this)(5,0) = (((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((- 0.13 - ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,1) = ((((((- 0.25 *  c_q_RH_HAA_) - (( 0.13 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.13 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.1 *  c_q_RH_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.1 *  s_q_RH_HAA_));
    (*this)(5,3) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(5,4) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,5) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT::Type_fr_base_X_fr_RH_FOOT()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,2) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(1,0) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,0) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(3,0) = (((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,1) = ((((((- 0.1 *  c_q_RH_HFE_) - ( 0.32125 *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.32125 *  c_q_RH_HFE_) - ( 0.1 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.25 *  c_q_RH_HFE_)) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(3,2) = (((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((- 0.13 - ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,5) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(4,0) = (((((( 0.13 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  c_q_RH_HAA_) - (( 0.13 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.32125 *  c_q_RH_HAA_));
    (*this)(4,1) = ((((((( 0.32125 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.1 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.1 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + (( 0.32125 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(4,2) = ((((((- 0.25 *  c_q_RH_HAA_) - (( 0.13 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.13 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.1 *  c_q_RH_HAA_));
    (*this)(4,3) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,0) = (((((((- 0.13 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.32125 *  s_q_RH_HAA_));
    (*this)(5,1) = ((((((( 0.1 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.32125 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.1 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.32125 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.1 *  s_q_RH_HAA_));
    (*this)(5,3) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,4) =  s_q_RH_HAA_;
    (*this)(5,5) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base::Type_fr_RH_FOOT_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(4,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,2) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(2,0) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(2,1) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(3,0) = (((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,1) = (((((( 0.13 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  c_q_RH_HAA_) - (( 0.13 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.32125 *  c_q_RH_HAA_));
    (*this)(3,2) = (((((((- 0.13 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.32125 *  s_q_RH_HAA_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,4) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,5) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,0) = ((((((- 0.1 *  c_q_RH_HFE_) - ( 0.32125 *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.32125 *  c_q_RH_HFE_) - ( 0.1 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.25 *  c_q_RH_HFE_)) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(4,1) = ((((((( 0.32125 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.1 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.1 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + (( 0.32125 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(4,2) = ((((((( 0.1 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.32125 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.1 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.32125 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
    (*this)(5,0) = (((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((- 0.13 - ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,1) = ((((((- 0.25 *  c_q_RH_HAA_) - (( 0.13 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.13 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.1 *  c_q_RH_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.1 *  s_q_RH_HAA_));
    (*this)(5,3) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(5,4) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,5) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP_COM::Type_fr_base_X_fr_RH_HIP_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,3) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP_COM& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP_COM::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = - s_q_RH_HAA_;
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) =  c_q_RH_HAA_;
    (*this)(3,1) = ( 1.5E-4 - ( 0.116 *  s_q_RH_HAA_));
    (*this)(3,2) = ( 0.00379 - ( 0.116 *  c_q_RH_HAA_));
    (*this)(4,0) = (( 0.00379 *  s_q_RH_HAA_) - ( 1.5E-4 *  c_q_RH_HAA_));
    (*this)(4,1) = ( 0.34152 *  s_q_RH_HAA_);
    (*this)(4,2) = ( 0.34152 *  c_q_RH_HAA_);
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) = - s_q_RH_HAA_;
    (*this)(5,0) = (((- 1.5E-4 *  s_q_RH_HAA_) - ( 0.00379 *  c_q_RH_HAA_)) +  0.116);
    (*this)(5,1) = (- 0.34152 *  c_q_RH_HAA_);
    (*this)(5,2) = ( 0.34152 *  s_q_RH_HAA_);
    (*this)(5,4) =  s_q_RH_HAA_;
    (*this)(5,5) =  c_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_HIP_COM_X_fr_base::Type_fr_RH_HIP_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,3) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_HIP_COM_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_HIP_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(2,1) = - s_q_RH_HAA_;
    (*this)(2,2) =  c_q_RH_HAA_;
    (*this)(3,1) = (( 0.00379 *  s_q_RH_HAA_) - ( 1.5E-4 *  c_q_RH_HAA_));
    (*this)(3,2) = (((- 1.5E-4 *  s_q_RH_HAA_) - ( 0.00379 *  c_q_RH_HAA_)) +  0.116);
    (*this)(4,0) = ( 1.5E-4 - ( 0.116 *  s_q_RH_HAA_));
    (*this)(4,1) = ( 0.34152 *  s_q_RH_HAA_);
    (*this)(4,2) = (- 0.34152 *  c_q_RH_HAA_);
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
    (*this)(5,0) = ( 0.00379 - ( 0.116 *  c_q_RH_HAA_));
    (*this)(5,1) = ( 0.34152 *  c_q_RH_HAA_);
    (*this)(5,2) = ( 0.34152 *  s_q_RH_HAA_);
    (*this)(5,4) = - s_q_RH_HAA_;
    (*this)(5,5) =  c_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK_COM::Type_fr_base_X_fr_RH_SHANK_COM()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK_COM& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK_COM::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,2) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(1,0) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,0) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(3,0) = (((( 0.13918 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.13918 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,1) = ((((((- 0.05873 *  c_q_RH_HFE_) - ( 0.09806 *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.09806 *  c_q_RH_HFE_) - ( 0.05873 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.25 *  c_q_RH_HFE_)) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(3,2) = (((( 0.13918 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((- 0.13918 - ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,5) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(4,0) = (((((( 0.13918 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  c_q_RH_HAA_) - (( 0.13918 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.09806 *  c_q_RH_HAA_));
    (*this)(4,1) = ((((((( 0.09806 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.05873 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.05873 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + (( 0.09806 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(4,2) = ((((((- 0.25 *  c_q_RH_HAA_) - (( 0.13918 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.13918 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.05873 *  c_q_RH_HAA_));
    (*this)(4,3) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,0) = (((((((- 0.13918 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13918 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.09806 *  s_q_RH_HAA_));
    (*this)(5,1) = ((((((( 0.05873 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.09806 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.05873 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.09806 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13918 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + ((( 0.13918 *  c_q_RH_HAA_) +  0.116) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.05873 *  s_q_RH_HAA_));
    (*this)(5,3) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,4) =  s_q_RH_HAA_;
    (*this)(5,5) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_SHANK_COM_X_fr_base::Type_fr_RH_SHANK_COM_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(4,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_SHANK_COM_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_SHANK_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,2) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(2,0) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(2,1) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(3,0) = (((( 0.13918 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.13918 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,1) = (((((( 0.13918 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  c_q_RH_HAA_) - (( 0.13918 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.09806 *  c_q_RH_HAA_));
    (*this)(3,2) = (((((((- 0.13918 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13918 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.09806 *  s_q_RH_HAA_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,4) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,5) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,0) = ((((((- 0.05873 *  c_q_RH_HFE_) - ( 0.09806 *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.09806 *  c_q_RH_HFE_) - ( 0.05873 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.25 *  c_q_RH_HFE_)) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(4,1) = ((((((( 0.09806 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.05873 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.05873 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + (( 0.09806 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(4,2) = ((((((( 0.05873 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.09806 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.05873 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.09806 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
    (*this)(5,0) = (((( 0.13918 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((- 0.13918 - ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,1) = ((((((- 0.25 *  c_q_RH_HAA_) - (( 0.13918 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.13918 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.05873 *  c_q_RH_HAA_));
    (*this)(5,2) = ((((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13918 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + ((( 0.13918 *  c_q_RH_HAA_) +  0.116) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.05873 *  s_q_RH_HAA_));
    (*this)(5,3) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(5,4) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,5) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH_COM::Type_fr_base_X_fr_RH_THIGH_COM()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH_COM& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH_COM::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) =  c_q_RH_HFE_;
    (*this)(0,2) =  s_q_RH_HFE_;
    (*this)(1,0) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = (- s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,0) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) = ( c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(3,0) = ((( 0.116 *  c_q_RH_HAA_) +  0.09523) *  s_q_RH_HFE_);
    (*this)(3,1) = ((( 0.0039 *  s_q_RH_HFE_) + ( 0.21458 *  c_q_RH_HFE_)) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(3,2) = (((- 0.116 *  c_q_RH_HAA_) -  0.09523) *  c_q_RH_HFE_);
    (*this)(3,3) =  c_q_RH_HFE_;
    (*this)(3,5) =  s_q_RH_HFE_;
    (*this)(4,0) = ((((- 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.09523 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.21458 *  c_q_RH_HAA_));
    (*this)(4,1) = (((( 0.21458 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.0039 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(4,2) = ((((- 0.09523 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.0039 *  c_q_RH_HAA_));
    (*this)(4,3) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) = (- s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(5,0) = ((((- 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.116 + ( 0.09523 *  c_q_RH_HAA_)) *  c_q_RH_HFE_)) - ( 0.21458 *  s_q_RH_HAA_));
    (*this)(5,1) = ((((- 0.21458 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.0039 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(5,2) = (((( 0.116 + ( 0.09523 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) + (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.0039 *  s_q_RH_HAA_));
    (*this)(5,3) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(5,4) =  s_q_RH_HAA_;
    (*this)(5,5) = ( c_q_RH_HAA_ *  c_q_RH_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_COM_X_fr_base::Type_fr_RH_THIGH_COM_X_fr_base()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(4,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_COM_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) =  c_q_RH_HFE_;
    (*this)(0,1) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(0,2) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(2,0) =  s_q_RH_HFE_;
    (*this)(2,1) = (- s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,2) = ( c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(3,0) = ((( 0.116 *  c_q_RH_HAA_) +  0.09523) *  s_q_RH_HFE_);
    (*this)(3,1) = ((((- 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.09523 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.21458 *  c_q_RH_HAA_));
    (*this)(3,2) = ((((- 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.116 + ( 0.09523 *  c_q_RH_HAA_)) *  c_q_RH_HFE_)) - ( 0.21458 *  s_q_RH_HAA_));
    (*this)(3,3) =  c_q_RH_HFE_;
    (*this)(3,4) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(3,5) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(4,0) = ((( 0.0039 *  s_q_RH_HFE_) + ( 0.21458 *  c_q_RH_HFE_)) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(4,1) = (((( 0.21458 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.0039 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(4,2) = ((((- 0.21458 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.0039 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
    (*this)(5,0) = (((- 0.116 *  c_q_RH_HAA_) -  0.09523) *  c_q_RH_HFE_);
    (*this)(5,1) = ((((- 0.09523 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.0039 *  c_q_RH_HAA_));
    (*this)(5,2) = (((( 0.116 + ( 0.09523 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) + (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.0039 *  s_q_RH_HAA_));
    (*this)(5,3) =  s_q_RH_HFE_;
    (*this)(5,4) = (- s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(5,5) = ( c_q_RH_HAA_ *  c_q_RH_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_base_COM::Type_fr_base_X_fr_base_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = - 0.05056;
    (*this)(3,2) = 0.00324;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0.05056;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0.00831;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = - 0.00324;
    (*this)(5,1) = - 0.00831;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_base_COM& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_base_COM::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_COM_X_fr_base::Type_fr_base_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0.05056;
    (*this)(3,2) = - 0.00324;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = - 0.05056;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.00831;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0.00324;
    (*this)(5,1) = 0.00831;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_COM_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_COM_X_fr_base::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_base_inertia::Type_fr_base_X_fr_base_inertia()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_base_inertia& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_base_inertia::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_inertia_X_fr_base::Type_fr_base_inertia_X_fr_base()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_inertia_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_inertia_X_fr_base::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_imu_link::Type_fr_base_X_fr_imu_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = - 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = - 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0.1837;
    (*this)(3,2) = - 0.05755;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0.1837;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0.062;
    (*this)(4,3) = 0;
    (*this)(4,4) = - 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = - 0.05755;
    (*this)(5,1) = - 0.062;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = - 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_imu_link& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_imu_link::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_imu_link_X_fr_base::Type_fr_imu_link_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = - 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = - 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0.1837;
    (*this)(3,2) = - 0.05755;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0.1837;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.062;
    (*this)(4,3) = 0;
    (*this)(4,4) = - 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = - 0.05755;
    (*this)(5,1) = 0.062;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = - 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_imu_link_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_imu_link_X_fr_base::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA::Type_fr_base_X_fr_LF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = - 0.116;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.277;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0.277;
    (*this)(5,2) = - 0.116;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE::Type_fr_base_X_fr_LF_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,3) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,1) =  s_q_LF_HAA_;
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(2,1) = - c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(3,1) = ((- 0.116 *  c_q_LF_HAA_) -  0.041);
    (*this)(3,2) = ( 0.116 *  s_q_LF_HAA_);
    (*this)(4,0) = ( 0.041 *  s_q_LF_HAA_);
    (*this)(4,1) = ( 0.3405 *  c_q_LF_HAA_);
    (*this)(4,2) = (- 0.3405 *  s_q_LF_HAA_);
    (*this)(4,4) =  s_q_LF_HAA_;
    (*this)(4,5) =  c_q_LF_HAA_;
    (*this)(5,0) = ((- 0.041 *  c_q_LF_HAA_) -  0.116);
    (*this)(5,1) = ( 0.3405 *  s_q_LF_HAA_);
    (*this)(5,2) = ( 0.3405 *  c_q_LF_HAA_);
    (*this)(5,4) = - c_q_LF_HAA_;
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE::Type_fr_base_X_fr_LF_KFE()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) =  c_q_LF_HFE_;
    (*this)(0,1) = - s_q_LF_HFE_;
    (*this)(1,0) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(1,1) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(2,0) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(2,1) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(3,0) = (((- 0.116 *  c_q_LF_HAA_) -  0.15) *  s_q_LF_HFE_);
    (*this)(3,1) = (((- 0.116 *  c_q_LF_HAA_) -  0.15) *  c_q_LF_HFE_);
    (*this)(3,2) = (( 0.25 *  c_q_LF_HFE_) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(3,3) =  c_q_LF_HFE_;
    (*this)(3,4) = - s_q_LF_HFE_;
    (*this)(4,0) = (((( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.15 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.25 *  c_q_LF_HAA_));
    (*this)(4,1) = ((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.15 *  s_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(4,2) = ((( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(4,3) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(4,4) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(4,5) =  c_q_LF_HAA_;
    (*this)(5,0) = (((( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((- 0.116 - ( 0.15 *  c_q_LF_HAA_)) *  c_q_LF_HFE_)) - ( 0.25 *  s_q_LF_HAA_));
    (*this)(5,1) = ((( 0.116 + ( 0.15 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) + (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_));
    (*this)(5,2) = (( 0.3405 *  c_q_LF_HAA_) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(5,3) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(5,4) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA::Type_fr_base_X_fr_RF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0.116;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.277;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0.277;
    (*this)(5,2) = 0.116;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE::Type_fr_base_X_fr_RF_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,3) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,1) =  s_q_RF_HAA_;
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(2,1) = - c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(3,1) = (( 0.116 *  c_q_RF_HAA_) +  0.041);
    (*this)(3,2) = (- 0.116 *  s_q_RF_HAA_);
    (*this)(4,0) = (- 0.041 *  s_q_RF_HAA_);
    (*this)(4,1) = ( 0.3405 *  c_q_RF_HAA_);
    (*this)(4,2) = (- 0.3405 *  s_q_RF_HAA_);
    (*this)(4,4) =  s_q_RF_HAA_;
    (*this)(4,5) =  c_q_RF_HAA_;
    (*this)(5,0) = (( 0.041 *  c_q_RF_HAA_) +  0.116);
    (*this)(5,1) = ( 0.3405 *  s_q_RF_HAA_);
    (*this)(5,2) = ( 0.3405 *  c_q_RF_HAA_);
    (*this)(5,4) = - c_q_RF_HAA_;
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE::Type_fr_base_X_fr_RF_KFE()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) =  c_q_RF_HFE_;
    (*this)(0,1) = - s_q_RF_HFE_;
    (*this)(1,0) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(1,1) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(2,0) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(2,1) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(3,0) = ((( 0.116 *  c_q_RF_HAA_) +  0.15) *  s_q_RF_HFE_);
    (*this)(3,1) = ((( 0.116 *  c_q_RF_HAA_) +  0.15) *  c_q_RF_HFE_);
    (*this)(3,2) = (( 0.25 *  c_q_RF_HFE_) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(3,3) =  c_q_RF_HFE_;
    (*this)(3,4) = - s_q_RF_HFE_;
    (*this)(4,0) = (((( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.15 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.25 *  c_q_RF_HAA_));
    (*this)(4,1) = ((( 0.15 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(4,2) = ((( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(4,3) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(4,4) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(4,5) =  c_q_RF_HAA_;
    (*this)(5,0) = (((( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.116 + ( 0.15 *  c_q_RF_HAA_)) *  c_q_RF_HFE_)) - ( 0.25 *  s_q_RF_HAA_));
    (*this)(5,1) = (((- 0.116 - ( 0.15 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) + (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(5,2) = (( 0.3405 *  c_q_RF_HAA_) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(5,3) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(5,4) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA::Type_fr_base_X_fr_LH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = - 0.116;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = - 0.277;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.277;
    (*this)(5,2) = - 0.116;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE::Type_fr_base_X_fr_LH_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,3) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,1) =  s_q_LH_HAA_;
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(2,1) = - c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(3,1) = ((- 0.116 *  c_q_LH_HAA_) -  0.041);
    (*this)(3,2) = ( 0.116 *  s_q_LH_HAA_);
    (*this)(4,0) = ( 0.041 *  s_q_LH_HAA_);
    (*this)(4,1) = (- 0.3405 *  c_q_LH_HAA_);
    (*this)(4,2) = ( 0.3405 *  s_q_LH_HAA_);
    (*this)(4,4) =  s_q_LH_HAA_;
    (*this)(4,5) =  c_q_LH_HAA_;
    (*this)(5,0) = ((- 0.041 *  c_q_LH_HAA_) -  0.116);
    (*this)(5,1) = (- 0.3405 *  s_q_LH_HAA_);
    (*this)(5,2) = (- 0.3405 *  c_q_LH_HAA_);
    (*this)(5,4) = - c_q_LH_HAA_;
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE::Type_fr_base_X_fr_LH_KFE()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) =  c_q_LH_HFE_;
    (*this)(0,1) = - s_q_LH_HFE_;
    (*this)(1,0) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(1,1) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(2,0) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(2,1) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(3,0) = (((- 0.116 *  c_q_LH_HAA_) -  0.15) *  s_q_LH_HFE_);
    (*this)(3,1) = (((- 0.116 *  c_q_LH_HAA_) -  0.15) *  c_q_LH_HFE_);
    (*this)(3,2) = (( 0.25 *  c_q_LH_HFE_) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(3,3) =  c_q_LH_HFE_;
    (*this)(3,4) = - s_q_LH_HFE_;
    (*this)(4,0) = ((((- 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.15 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.25 *  c_q_LH_HAA_));
    (*this)(4,1) = (((- 0.15 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(4,2) = ((( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(4,3) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(4,4) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(4,5) =  c_q_LH_HAA_;
    (*this)(5,0) = ((((- 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((- 0.116 - ( 0.15 *  c_q_LH_HAA_)) *  c_q_LH_HFE_)) - ( 0.25 *  s_q_LH_HAA_));
    (*this)(5,1) = ((( 0.116 + ( 0.15 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(5,2) = (((- 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(5,3) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(5,4) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA::Type_fr_base_X_fr_RH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0.116;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = - 0.277;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.277;
    (*this)(5,2) = 0.116;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE::Type_fr_base_X_fr_RH_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,3) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,1) =  s_q_RH_HAA_;
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(2,1) = - c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(3,1) = (( 0.116 *  c_q_RH_HAA_) +  0.041);
    (*this)(3,2) = (- 0.116 *  s_q_RH_HAA_);
    (*this)(4,0) = (- 0.041 *  s_q_RH_HAA_);
    (*this)(4,1) = (- 0.3405 *  c_q_RH_HAA_);
    (*this)(4,2) = ( 0.3405 *  s_q_RH_HAA_);
    (*this)(4,4) =  s_q_RH_HAA_;
    (*this)(4,5) =  c_q_RH_HAA_;
    (*this)(5,0) = (( 0.041 *  c_q_RH_HAA_) +  0.116);
    (*this)(5,1) = (- 0.3405 *  s_q_RH_HAA_);
    (*this)(5,2) = (- 0.3405 *  c_q_RH_HAA_);
    (*this)(5,4) = - c_q_RH_HAA_;
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE::Type_fr_base_X_fr_RH_KFE()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) =  c_q_RH_HFE_;
    (*this)(0,1) = - s_q_RH_HFE_;
    (*this)(1,0) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(1,1) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(2,0) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(2,1) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(3,0) = ((( 0.116 *  c_q_RH_HAA_) +  0.15) *  s_q_RH_HFE_);
    (*this)(3,1) = ((( 0.116 *  c_q_RH_HAA_) +  0.15) *  c_q_RH_HFE_);
    (*this)(3,2) = (( 0.25 *  c_q_RH_HFE_) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(3,3) =  c_q_RH_HFE_;
    (*this)(3,4) = - s_q_RH_HFE_;
    (*this)(4,0) = ((((- 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.15 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.25 *  c_q_RH_HAA_));
    (*this)(4,1) = ((( 0.15 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(4,2) = ((( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(4,3) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(4,4) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(4,5) =  c_q_RH_HAA_;
    (*this)(5,0) = ((((- 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.116 + ( 0.15 *  c_q_RH_HAA_)) *  c_q_RH_HFE_)) - ( 0.25 *  s_q_RH_HAA_));
    (*this)(5,1) = (((- 0.116 - ( 0.15 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(5,2) = (((- 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(5,3) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(5,4) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP::Type_fr_LF_THIGH_X_fr_LF_HIP()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = - 0.0635;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  s_q_LF_HFE_;
    (*this)(0,2) =  c_q_LF_HFE_;
    (*this)(1,0) =  c_q_LF_HFE_;
    (*this)(1,2) = - s_q_LF_HFE_;
    (*this)(3,0) = ( 0.041 *  c_q_LF_HFE_);
    (*this)(3,1) = ( 0.0635 *  s_q_LF_HFE_);
    (*this)(3,2) = (- 0.041 *  s_q_LF_HFE_);
    (*this)(3,3) =  s_q_LF_HFE_;
    (*this)(3,5) =  c_q_LF_HFE_;
    (*this)(4,0) = (- 0.041 *  s_q_LF_HFE_);
    (*this)(4,1) = ( 0.0635 *  c_q_LF_HFE_);
    (*this)(4,2) = (- 0.041 *  c_q_LF_HFE_);
    (*this)(4,3) =  c_q_LF_HFE_;
    (*this)(4,5) = - s_q_LF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH::Type_fr_LF_HIP_X_fr_LF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = - 0.0635;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  s_q_LF_HFE_;
    (*this)(0,1) =  c_q_LF_HFE_;
    (*this)(2,0) =  c_q_LF_HFE_;
    (*this)(2,1) = - s_q_LF_HFE_;
    (*this)(3,0) = ( 0.041 *  c_q_LF_HFE_);
    (*this)(3,1) = (- 0.041 *  s_q_LF_HFE_);
    (*this)(3,3) =  s_q_LF_HFE_;
    (*this)(3,4) =  c_q_LF_HFE_;
    (*this)(4,0) = ( 0.0635 *  s_q_LF_HFE_);
    (*this)(4,1) = ( 0.0635 *  c_q_LF_HFE_);
    (*this)(5,0) = (- 0.041 *  s_q_LF_HFE_);
    (*this)(5,1) = (- 0.041 *  c_q_LF_HFE_);
    (*this)(5,3) =  c_q_LF_HFE_;
    (*this)(5,4) = - s_q_LF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH::Type_fr_LF_SHANK_X_fr_LF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0.25;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar c_q_LF_KFE_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  c_q_LF_KFE_;
    (*this)(0,1) =  s_q_LF_KFE_;
    (*this)(1,0) = - s_q_LF_KFE_;
    (*this)(1,1) =  c_q_LF_KFE_;
    (*this)(3,0) = (- 0.109 *  s_q_LF_KFE_);
    (*this)(3,1) = ( 0.109 *  c_q_LF_KFE_);
    (*this)(3,2) = (- 0.25 *  c_q_LF_KFE_);
    (*this)(3,3) =  c_q_LF_KFE_;
    (*this)(3,4) =  s_q_LF_KFE_;
    (*this)(4,0) = (- 0.109 *  c_q_LF_KFE_);
    (*this)(4,1) = (- 0.109 *  s_q_LF_KFE_);
    (*this)(4,2) = ( 0.25 *  s_q_LF_KFE_);
    (*this)(4,3) = - s_q_LF_KFE_;
    (*this)(4,4) =  c_q_LF_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK::Type_fr_LF_THIGH_X_fr_LF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0.25;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar c_q_LF_KFE_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  c_q_LF_KFE_;
    (*this)(0,1) = - s_q_LF_KFE_;
    (*this)(1,0) =  s_q_LF_KFE_;
    (*this)(1,1) =  c_q_LF_KFE_;
    (*this)(3,0) = (- 0.109 *  s_q_LF_KFE_);
    (*this)(3,1) = (- 0.109 *  c_q_LF_KFE_);
    (*this)(3,3) =  c_q_LF_KFE_;
    (*this)(3,4) = - s_q_LF_KFE_;
    (*this)(4,0) = ( 0.109 *  c_q_LF_KFE_);
    (*this)(4,1) = (- 0.109 *  s_q_LF_KFE_);
    (*this)(4,3) =  s_q_LF_KFE_;
    (*this)(4,4) =  c_q_LF_KFE_;
    (*this)(5,0) = (- 0.25 *  c_q_LF_KFE_);
    (*this)(5,1) = ( 0.25 *  s_q_LF_KFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP::Type_fr_RF_THIGH_X_fr_RF_HIP()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = - 0.0635;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  s_q_RF_HFE_;
    (*this)(0,2) =  c_q_RF_HFE_;
    (*this)(1,0) =  c_q_RF_HFE_;
    (*this)(1,2) = - s_q_RF_HFE_;
    (*this)(3,0) = (- 0.041 *  c_q_RF_HFE_);
    (*this)(3,1) = ( 0.0635 *  s_q_RF_HFE_);
    (*this)(3,2) = ( 0.041 *  s_q_RF_HFE_);
    (*this)(3,3) =  s_q_RF_HFE_;
    (*this)(3,5) =  c_q_RF_HFE_;
    (*this)(4,0) = ( 0.041 *  s_q_RF_HFE_);
    (*this)(4,1) = ( 0.0635 *  c_q_RF_HFE_);
    (*this)(4,2) = ( 0.041 *  c_q_RF_HFE_);
    (*this)(4,3) =  c_q_RF_HFE_;
    (*this)(4,5) = - s_q_RF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH::Type_fr_RF_HIP_X_fr_RF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = - 0.0635;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  s_q_RF_HFE_;
    (*this)(0,1) =  c_q_RF_HFE_;
    (*this)(2,0) =  c_q_RF_HFE_;
    (*this)(2,1) = - s_q_RF_HFE_;
    (*this)(3,0) = (- 0.041 *  c_q_RF_HFE_);
    (*this)(3,1) = ( 0.041 *  s_q_RF_HFE_);
    (*this)(3,3) =  s_q_RF_HFE_;
    (*this)(3,4) =  c_q_RF_HFE_;
    (*this)(4,0) = ( 0.0635 *  s_q_RF_HFE_);
    (*this)(4,1) = ( 0.0635 *  c_q_RF_HFE_);
    (*this)(5,0) = ( 0.041 *  s_q_RF_HFE_);
    (*this)(5,1) = ( 0.041 *  c_q_RF_HFE_);
    (*this)(5,3) =  c_q_RF_HFE_;
    (*this)(5,4) = - s_q_RF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH::Type_fr_RF_SHANK_X_fr_RF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0.25;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar c_q_RF_KFE_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  c_q_RF_KFE_;
    (*this)(0,1) =  s_q_RF_KFE_;
    (*this)(1,0) = - s_q_RF_KFE_;
    (*this)(1,1) =  c_q_RF_KFE_;
    (*this)(3,0) = ( 0.109 *  s_q_RF_KFE_);
    (*this)(3,1) = (- 0.109 *  c_q_RF_KFE_);
    (*this)(3,2) = (- 0.25 *  c_q_RF_KFE_);
    (*this)(3,3) =  c_q_RF_KFE_;
    (*this)(3,4) =  s_q_RF_KFE_;
    (*this)(4,0) = ( 0.109 *  c_q_RF_KFE_);
    (*this)(4,1) = ( 0.109 *  s_q_RF_KFE_);
    (*this)(4,2) = ( 0.25 *  s_q_RF_KFE_);
    (*this)(4,3) = - s_q_RF_KFE_;
    (*this)(4,4) =  c_q_RF_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK::Type_fr_RF_THIGH_X_fr_RF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0.25;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar c_q_RF_KFE_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  c_q_RF_KFE_;
    (*this)(0,1) = - s_q_RF_KFE_;
    (*this)(1,0) =  s_q_RF_KFE_;
    (*this)(1,1) =  c_q_RF_KFE_;
    (*this)(3,0) = ( 0.109 *  s_q_RF_KFE_);
    (*this)(3,1) = ( 0.109 *  c_q_RF_KFE_);
    (*this)(3,3) =  c_q_RF_KFE_;
    (*this)(3,4) = - s_q_RF_KFE_;
    (*this)(4,0) = (- 0.109 *  c_q_RF_KFE_);
    (*this)(4,1) = ( 0.109 *  s_q_RF_KFE_);
    (*this)(4,3) =  s_q_RF_KFE_;
    (*this)(4,4) =  c_q_RF_KFE_;
    (*this)(5,0) = (- 0.25 *  c_q_RF_KFE_);
    (*this)(5,1) = ( 0.25 *  s_q_RF_KFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP::Type_fr_LH_THIGH_X_fr_LH_HIP()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0.0635;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  s_q_LH_HFE_;
    (*this)(0,2) =  c_q_LH_HFE_;
    (*this)(1,0) =  c_q_LH_HFE_;
    (*this)(1,2) = - s_q_LH_HFE_;
    (*this)(3,0) = ( 0.041 *  c_q_LH_HFE_);
    (*this)(3,1) = (- 0.0635 *  s_q_LH_HFE_);
    (*this)(3,2) = (- 0.041 *  s_q_LH_HFE_);
    (*this)(3,3) =  s_q_LH_HFE_;
    (*this)(3,5) =  c_q_LH_HFE_;
    (*this)(4,0) = (- 0.041 *  s_q_LH_HFE_);
    (*this)(4,1) = (- 0.0635 *  c_q_LH_HFE_);
    (*this)(4,2) = (- 0.041 *  c_q_LH_HFE_);
    (*this)(4,3) =  c_q_LH_HFE_;
    (*this)(4,5) = - s_q_LH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH::Type_fr_LH_HIP_X_fr_LH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0.0635;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  s_q_LH_HFE_;
    (*this)(0,1) =  c_q_LH_HFE_;
    (*this)(2,0) =  c_q_LH_HFE_;
    (*this)(2,1) = - s_q_LH_HFE_;
    (*this)(3,0) = ( 0.041 *  c_q_LH_HFE_);
    (*this)(3,1) = (- 0.041 *  s_q_LH_HFE_);
    (*this)(3,3) =  s_q_LH_HFE_;
    (*this)(3,4) =  c_q_LH_HFE_;
    (*this)(4,0) = (- 0.0635 *  s_q_LH_HFE_);
    (*this)(4,1) = (- 0.0635 *  c_q_LH_HFE_);
    (*this)(5,0) = (- 0.041 *  s_q_LH_HFE_);
    (*this)(5,1) = (- 0.041 *  c_q_LH_HFE_);
    (*this)(5,3) =  c_q_LH_HFE_;
    (*this)(5,4) = - s_q_LH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH::Type_fr_LH_SHANK_X_fr_LH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0.25;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar c_q_LH_KFE_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  c_q_LH_KFE_;
    (*this)(0,1) =  s_q_LH_KFE_;
    (*this)(1,0) = - s_q_LH_KFE_;
    (*this)(1,1) =  c_q_LH_KFE_;
    (*this)(3,0) = (- 0.109 *  s_q_LH_KFE_);
    (*this)(3,1) = ( 0.109 *  c_q_LH_KFE_);
    (*this)(3,2) = (- 0.25 *  c_q_LH_KFE_);
    (*this)(3,3) =  c_q_LH_KFE_;
    (*this)(3,4) =  s_q_LH_KFE_;
    (*this)(4,0) = (- 0.109 *  c_q_LH_KFE_);
    (*this)(4,1) = (- 0.109 *  s_q_LH_KFE_);
    (*this)(4,2) = ( 0.25 *  s_q_LH_KFE_);
    (*this)(4,3) = - s_q_LH_KFE_;
    (*this)(4,4) =  c_q_LH_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK::Type_fr_LH_THIGH_X_fr_LH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0.25;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar c_q_LH_KFE_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  c_q_LH_KFE_;
    (*this)(0,1) = - s_q_LH_KFE_;
    (*this)(1,0) =  s_q_LH_KFE_;
    (*this)(1,1) =  c_q_LH_KFE_;
    (*this)(3,0) = (- 0.109 *  s_q_LH_KFE_);
    (*this)(3,1) = (- 0.109 *  c_q_LH_KFE_);
    (*this)(3,3) =  c_q_LH_KFE_;
    (*this)(3,4) = - s_q_LH_KFE_;
    (*this)(4,0) = ( 0.109 *  c_q_LH_KFE_);
    (*this)(4,1) = (- 0.109 *  s_q_LH_KFE_);
    (*this)(4,3) =  s_q_LH_KFE_;
    (*this)(4,4) =  c_q_LH_KFE_;
    (*this)(5,0) = (- 0.25 *  c_q_LH_KFE_);
    (*this)(5,1) = ( 0.25 *  s_q_LH_KFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP::Type_fr_RH_THIGH_X_fr_RH_HIP()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,4) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0.0635;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  s_q_RH_HFE_;
    (*this)(0,2) =  c_q_RH_HFE_;
    (*this)(1,0) =  c_q_RH_HFE_;
    (*this)(1,2) = - s_q_RH_HFE_;
    (*this)(3,0) = (- 0.041 *  c_q_RH_HFE_);
    (*this)(3,1) = (- 0.0635 *  s_q_RH_HFE_);
    (*this)(3,2) = ( 0.041 *  s_q_RH_HFE_);
    (*this)(3,3) =  s_q_RH_HFE_;
    (*this)(3,5) =  c_q_RH_HFE_;
    (*this)(4,0) = ( 0.041 *  s_q_RH_HFE_);
    (*this)(4,1) = (- 0.0635 *  c_q_RH_HFE_);
    (*this)(4,2) = ( 0.041 *  c_q_RH_HFE_);
    (*this)(4,3) =  c_q_RH_HFE_;
    (*this)(4,5) = - s_q_RH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH::Type_fr_RH_HIP_X_fr_RH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0.0635;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  s_q_RH_HFE_;
    (*this)(0,1) =  c_q_RH_HFE_;
    (*this)(2,0) =  c_q_RH_HFE_;
    (*this)(2,1) = - s_q_RH_HFE_;
    (*this)(3,0) = (- 0.041 *  c_q_RH_HFE_);
    (*this)(3,1) = ( 0.041 *  s_q_RH_HFE_);
    (*this)(3,3) =  s_q_RH_HFE_;
    (*this)(3,4) =  c_q_RH_HFE_;
    (*this)(4,0) = (- 0.0635 *  s_q_RH_HFE_);
    (*this)(4,1) = (- 0.0635 *  c_q_RH_HFE_);
    (*this)(5,0) = ( 0.041 *  s_q_RH_HFE_);
    (*this)(5,1) = ( 0.041 *  c_q_RH_HFE_);
    (*this)(5,3) =  c_q_RH_HFE_;
    (*this)(5,4) = - s_q_RH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH::Type_fr_RH_SHANK_X_fr_RH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0.25;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar c_q_RH_KFE_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  c_q_RH_KFE_;
    (*this)(0,1) =  s_q_RH_KFE_;
    (*this)(1,0) = - s_q_RH_KFE_;
    (*this)(1,1) =  c_q_RH_KFE_;
    (*this)(3,0) = ( 0.109 *  s_q_RH_KFE_);
    (*this)(3,1) = (- 0.109 *  c_q_RH_KFE_);
    (*this)(3,2) = (- 0.25 *  c_q_RH_KFE_);
    (*this)(3,3) =  c_q_RH_KFE_;
    (*this)(3,4) =  s_q_RH_KFE_;
    (*this)(4,0) = ( 0.109 *  c_q_RH_KFE_);
    (*this)(4,1) = ( 0.109 *  s_q_RH_KFE_);
    (*this)(4,2) = ( 0.25 *  s_q_RH_KFE_);
    (*this)(4,3) = - s_q_RH_KFE_;
    (*this)(4,4) =  c_q_RH_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK::Type_fr_RH_THIGH_X_fr_RH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0.25;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar c_q_RH_KFE_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  c_q_RH_KFE_;
    (*this)(0,1) = - s_q_RH_KFE_;
    (*this)(1,0) =  s_q_RH_KFE_;
    (*this)(1,1) =  c_q_RH_KFE_;
    (*this)(3,0) = ( 0.109 *  s_q_RH_KFE_);
    (*this)(3,1) = ( 0.109 *  c_q_RH_KFE_);
    (*this)(3,3) =  c_q_RH_KFE_;
    (*this)(3,4) = - s_q_RH_KFE_;
    (*this)(4,0) = (- 0.109 *  c_q_RH_KFE_);
    (*this)(4,1) = ( 0.109 *  s_q_RH_KFE_);
    (*this)(4,3) =  s_q_RH_KFE_;
    (*this)(4,4) =  c_q_RH_KFE_;
    (*this)(5,0) = (- 0.25 *  c_q_RH_KFE_);
    (*this)(5,1) = ( 0.25 *  s_q_RH_KFE_);
    return *this;
}

template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP::Type_fr_base_X_fr_LF_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = - 0.116;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,3) = (- 0.116 *  c_q_LF_HAA_);
    (*this)(0,4) = ( 0.116 *  s_q_LF_HAA_);
    (*this)(1,0) =  s_q_LF_HAA_;
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,3) = ( 0.277 *  c_q_LF_HAA_);
    (*this)(1,4) = (- 0.277 *  s_q_LF_HAA_);
    (*this)(2,0) = - c_q_LF_HAA_;
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,3) = ( 0.277 *  s_q_LF_HAA_);
    (*this)(2,4) = ( 0.277 *  c_q_LF_HAA_);
    (*this)(4,3) =  s_q_LF_HAA_;
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(5,3) = - c_q_LF_HAA_;
    (*this)(5,4) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base::Type_fr_LF_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.116;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,1) =  s_q_LF_HAA_;
    (*this)(0,2) = - c_q_LF_HAA_;
    (*this)(0,3) = (- 0.116 *  c_q_LF_HAA_);
    (*this)(0,4) = ( 0.277 *  c_q_LF_HAA_);
    (*this)(0,5) = ( 0.277 *  s_q_LF_HAA_);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = ( 0.116 *  s_q_LF_HAA_);
    (*this)(1,4) = (- 0.277 *  s_q_LF_HAA_);
    (*this)(1,5) = ( 0.277 *  c_q_LF_HAA_);
    (*this)(3,4) =  s_q_LF_HAA_;
    (*this)(3,5) = - c_q_LF_HAA_;
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH::Type_fr_base_X_fr_LF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) =  c_q_LF_HFE_;
    (*this)(0,1) = - s_q_LF_HFE_;
    (*this)(0,3) = (((- 0.116 *  c_q_LF_HAA_) -  0.041) *  s_q_LF_HFE_);
    (*this)(0,4) = (((- 0.116 *  c_q_LF_HAA_) -  0.041) *  c_q_LF_HFE_);
    (*this)(0,5) = ( 0.116 *  s_q_LF_HAA_);
    (*this)(1,0) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(1,1) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = ((( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.041 *  s_q_LF_HAA_) *  c_q_LF_HFE_));
    (*this)(1,4) = ((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.041 *  s_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(1,5) = (- 0.3405 *  s_q_LF_HAA_);
    (*this)(2,0) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(2,1) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = ((( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((- 0.116 - ( 0.041 *  c_q_LF_HAA_)) *  c_q_LF_HFE_));
    (*this)(2,4) = ((( 0.116 + ( 0.041 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) + (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_));
    (*this)(2,5) = ( 0.3405 *  c_q_LF_HAA_);
    (*this)(3,3) =  c_q_LF_HFE_;
    (*this)(3,4) = - s_q_LF_HFE_;
    (*this)(4,3) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(4,4) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(4,5) =  c_q_LF_HAA_;
    (*this)(5,3) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(5,4) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_base::Type_fr_LF_THIGH_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) =  c_q_LF_HFE_;
    (*this)(0,1) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(0,2) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(0,3) = (((- 0.116 *  c_q_LF_HAA_) -  0.041) *  s_q_LF_HFE_);
    (*this)(0,4) = ((( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.041 *  s_q_LF_HAA_) *  c_q_LF_HFE_));
    (*this)(0,5) = ((( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((- 0.116 - ( 0.041 *  c_q_LF_HAA_)) *  c_q_LF_HFE_));
    (*this)(1,0) = - s_q_LF_HFE_;
    (*this)(1,1) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,2) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,3) = (((- 0.116 *  c_q_LF_HAA_) -  0.041) *  c_q_LF_HFE_);
    (*this)(1,4) = ((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.041 *  s_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(1,5) = ((( 0.116 + ( 0.041 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) + (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_));
    (*this)(2,1) =  c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = ( 0.116 *  s_q_LF_HAA_);
    (*this)(2,4) = (- 0.3405 *  s_q_LF_HAA_);
    (*this)(2,5) = ( 0.3405 *  c_q_LF_HAA_);
    (*this)(3,3) =  c_q_LF_HFE_;
    (*this)(3,4) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(3,5) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(4,3) = - s_q_LF_HFE_;
    (*this)(4,4) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(4,5) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(5,4) =  c_q_LF_HAA_;
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK::Type_fr_base_X_fr_LF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,1) = ((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(0,3) = ((((- 0.15 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.15 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,4) = (((( 0.15 + ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.15 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,5) = (( 0.25 *  c_q_LF_HFE_) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(1,0) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = ((((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.15 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  c_q_LF_HAA_) + (( 0.15 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(1,4) = ((((( 0.25 *  c_q_LF_HAA_) - (( 0.15 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.15 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(1,5) = ((( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(2,0) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) = ((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = ((((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.15 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  s_q_LF_HAA_) + (((- 0.15 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(2,4) = ((((( 0.25 *  s_q_LF_HAA_) + ((( 0.15 *  c_q_LF_HAA_) +  0.116) *  c_q_LF_HFE_)) - (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.15 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(2,5) = (( 0.3405 *  c_q_LF_HAA_) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,4) = ((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(4,3) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,4) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(4,5) =  c_q_LF_HAA_;
    (*this)(5,3) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,4) = ((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_base::Type_fr_LF_SHANK_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,2) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,3) = ((((- 0.15 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.15 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,4) = ((((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.15 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  c_q_LF_HAA_) + (( 0.15 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(0,5) = ((((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.15 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  s_q_LF_HAA_) + (((- 0.15 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(1,0) = ((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(1,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(1,2) = ((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,3) = (((( 0.15 + ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.15 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,4) = ((((( 0.25 *  c_q_LF_HAA_) - (( 0.15 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.15 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(1,5) = ((((( 0.25 *  s_q_LF_HAA_) + ((( 0.15 *  c_q_LF_HAA_) +  0.116) *  c_q_LF_HFE_)) - (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.15 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(2,1) =  c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = (( 0.25 *  c_q_LF_HFE_) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(2,4) = ((( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(2,5) = (( 0.3405 *  c_q_LF_HAA_) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,4) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,5) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,3) = ((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(4,4) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(4,5) = ((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,4) =  c_q_LF_HAA_;
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP::Type_fr_base_X_fr_RF_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0.116;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,3) = ( 0.116 *  c_q_RF_HAA_);
    (*this)(0,4) = (- 0.116 *  s_q_RF_HAA_);
    (*this)(1,0) =  s_q_RF_HAA_;
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,3) = ( 0.277 *  c_q_RF_HAA_);
    (*this)(1,4) = (- 0.277 *  s_q_RF_HAA_);
    (*this)(2,0) = - c_q_RF_HAA_;
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,3) = ( 0.277 *  s_q_RF_HAA_);
    (*this)(2,4) = ( 0.277 *  c_q_RF_HAA_);
    (*this)(4,3) =  s_q_RF_HAA_;
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(5,3) = - c_q_RF_HAA_;
    (*this)(5,4) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base::Type_fr_RF_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0.116;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,1) =  s_q_RF_HAA_;
    (*this)(0,2) = - c_q_RF_HAA_;
    (*this)(0,3) = ( 0.116 *  c_q_RF_HAA_);
    (*this)(0,4) = ( 0.277 *  c_q_RF_HAA_);
    (*this)(0,5) = ( 0.277 *  s_q_RF_HAA_);
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = (- 0.116 *  s_q_RF_HAA_);
    (*this)(1,4) = (- 0.277 *  s_q_RF_HAA_);
    (*this)(1,5) = ( 0.277 *  c_q_RF_HAA_);
    (*this)(3,4) =  s_q_RF_HAA_;
    (*this)(3,5) = - c_q_RF_HAA_;
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH::Type_fr_base_X_fr_RF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) =  c_q_RF_HFE_;
    (*this)(0,1) = - s_q_RF_HFE_;
    (*this)(0,3) = ((( 0.116 *  c_q_RF_HAA_) +  0.041) *  s_q_RF_HFE_);
    (*this)(0,4) = ((( 0.116 *  c_q_RF_HAA_) +  0.041) *  c_q_RF_HFE_);
    (*this)(0,5) = (- 0.116 *  s_q_RF_HAA_);
    (*this)(1,0) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(1,1) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = ((( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.041 *  s_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(1,4) = ((( 0.041 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(1,5) = (- 0.3405 *  s_q_RF_HAA_);
    (*this)(2,0) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(2,1) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = ((( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.116 + ( 0.041 *  c_q_RF_HAA_)) *  c_q_RF_HFE_));
    (*this)(2,4) = (((- 0.116 - ( 0.041 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) + (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(2,5) = ( 0.3405 *  c_q_RF_HAA_);
    (*this)(3,3) =  c_q_RF_HFE_;
    (*this)(3,4) = - s_q_RF_HFE_;
    (*this)(4,3) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(4,4) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(4,5) =  c_q_RF_HAA_;
    (*this)(5,3) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(5,4) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_base::Type_fr_RF_THIGH_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) =  c_q_RF_HFE_;
    (*this)(0,1) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(0,2) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(0,3) = ((( 0.116 *  c_q_RF_HAA_) +  0.041) *  s_q_RF_HFE_);
    (*this)(0,4) = ((( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.041 *  s_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(0,5) = ((( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.116 + ( 0.041 *  c_q_RF_HAA_)) *  c_q_RF_HFE_));
    (*this)(1,0) = - s_q_RF_HFE_;
    (*this)(1,1) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,2) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,3) = ((( 0.116 *  c_q_RF_HAA_) +  0.041) *  c_q_RF_HFE_);
    (*this)(1,4) = ((( 0.041 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(1,5) = (((- 0.116 - ( 0.041 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) + (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(2,1) =  c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (- 0.116 *  s_q_RF_HAA_);
    (*this)(2,4) = (- 0.3405 *  s_q_RF_HAA_);
    (*this)(2,5) = ( 0.3405 *  c_q_RF_HAA_);
    (*this)(3,3) =  c_q_RF_HFE_;
    (*this)(3,4) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(3,5) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(4,3) = - s_q_RF_HFE_;
    (*this)(4,4) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(4,5) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(5,4) =  c_q_RF_HAA_;
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK::Type_fr_base_X_fr_RF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,1) = ((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(0,3) = (((( 0.15 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.15 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,4) = ((((- 0.15 - ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.15 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,5) = (( 0.25 *  c_q_RF_HFE_) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(1,0) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = ((((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.15 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  c_q_RF_HAA_) - (( 0.15 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(1,4) = ((((( 0.25 *  c_q_RF_HAA_) + (( 0.15 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.15 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(1,5) = ((( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(2,0) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) = ((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = ((((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.15 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  s_q_RF_HAA_) + ((( 0.15 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(2,4) = ((((( 0.25 *  s_q_RF_HAA_) + (((- 0.15 *  c_q_RF_HAA_) -  0.116) *  c_q_RF_HFE_)) - (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.15 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(2,5) = (( 0.3405 *  c_q_RF_HAA_) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,4) = ((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(4,3) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,4) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(4,5) =  c_q_RF_HAA_;
    (*this)(5,3) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,4) = ((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_base::Type_fr_RF_SHANK_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,2) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,3) = (((( 0.15 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.15 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,4) = ((((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.15 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  c_q_RF_HAA_) - (( 0.15 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(0,5) = ((((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.15 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  s_q_RF_HAA_) + ((( 0.15 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(1,0) = ((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(1,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(1,2) = ((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,3) = ((((- 0.15 - ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.15 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,4) = ((((( 0.25 *  c_q_RF_HAA_) + (( 0.15 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.15 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(1,5) = ((((( 0.25 *  s_q_RF_HAA_) + (((- 0.15 *  c_q_RF_HAA_) -  0.116) *  c_q_RF_HFE_)) - (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.15 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(2,1) =  c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (( 0.25 *  c_q_RF_HFE_) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(2,4) = ((( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(2,5) = (( 0.3405 *  c_q_RF_HAA_) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,4) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,5) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,3) = ((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(4,4) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(4,5) = ((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,4) =  c_q_RF_HAA_;
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP::Type_fr_base_X_fr_LH_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = - 0.116;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,3) = (- 0.116 *  c_q_LH_HAA_);
    (*this)(0,4) = ( 0.116 *  s_q_LH_HAA_);
    (*this)(1,0) =  s_q_LH_HAA_;
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,3) = (- 0.277 *  c_q_LH_HAA_);
    (*this)(1,4) = ( 0.277 *  s_q_LH_HAA_);
    (*this)(2,0) = - c_q_LH_HAA_;
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,3) = (- 0.277 *  s_q_LH_HAA_);
    (*this)(2,4) = (- 0.277 *  c_q_LH_HAA_);
    (*this)(4,3) =  s_q_LH_HAA_;
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(5,3) = - c_q_LH_HAA_;
    (*this)(5,4) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base::Type_fr_LH_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.116;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,1) =  s_q_LH_HAA_;
    (*this)(0,2) = - c_q_LH_HAA_;
    (*this)(0,3) = (- 0.116 *  c_q_LH_HAA_);
    (*this)(0,4) = (- 0.277 *  c_q_LH_HAA_);
    (*this)(0,5) = (- 0.277 *  s_q_LH_HAA_);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = ( 0.116 *  s_q_LH_HAA_);
    (*this)(1,4) = ( 0.277 *  s_q_LH_HAA_);
    (*this)(1,5) = (- 0.277 *  c_q_LH_HAA_);
    (*this)(3,4) =  s_q_LH_HAA_;
    (*this)(3,5) = - c_q_LH_HAA_;
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH::Type_fr_base_X_fr_LH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) =  c_q_LH_HFE_;
    (*this)(0,1) = - s_q_LH_HFE_;
    (*this)(0,3) = (((- 0.116 *  c_q_LH_HAA_) -  0.041) *  s_q_LH_HFE_);
    (*this)(0,4) = (((- 0.116 *  c_q_LH_HAA_) -  0.041) *  c_q_LH_HFE_);
    (*this)(0,5) = ( 0.116 *  s_q_LH_HAA_);
    (*this)(1,0) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(1,1) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = ((( 0.041 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(1,4) = (((- 0.041 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(1,5) = ( 0.3405 *  s_q_LH_HAA_);
    (*this)(2,0) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(2,1) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = (((- 0.116 - ( 0.041 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(2,4) = ((( 0.116 + ( 0.041 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(2,5) = (- 0.3405 *  c_q_LH_HAA_);
    (*this)(3,3) =  c_q_LH_HFE_;
    (*this)(3,4) = - s_q_LH_HFE_;
    (*this)(4,3) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(4,4) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(4,5) =  c_q_LH_HAA_;
    (*this)(5,3) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(5,4) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_base::Type_fr_LH_THIGH_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) =  c_q_LH_HFE_;
    (*this)(0,1) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(0,2) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(0,3) = (((- 0.116 *  c_q_LH_HAA_) -  0.041) *  s_q_LH_HFE_);
    (*this)(0,4) = ((( 0.041 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(0,5) = (((- 0.116 - ( 0.041 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(1,0) = - s_q_LH_HFE_;
    (*this)(1,1) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,2) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,3) = (((- 0.116 *  c_q_LH_HAA_) -  0.041) *  c_q_LH_HFE_);
    (*this)(1,4) = (((- 0.041 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(1,5) = ((( 0.116 + ( 0.041 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(2,1) =  c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ( 0.116 *  s_q_LH_HAA_);
    (*this)(2,4) = ( 0.3405 *  s_q_LH_HAA_);
    (*this)(2,5) = (- 0.3405 *  c_q_LH_HAA_);
    (*this)(3,3) =  c_q_LH_HFE_;
    (*this)(3,4) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(3,5) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(4,3) = - s_q_LH_HFE_;
    (*this)(4,4) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(4,5) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(5,4) =  c_q_LH_HAA_;
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK::Type_fr_base_X_fr_LH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,1) = ((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(0,3) = ((((- 0.15 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.15 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,4) = (((( 0.15 + ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.15 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,5) = (( 0.25 *  c_q_LH_HFE_) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(1,0) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = (((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.15 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  c_q_LH_HAA_) + (( 0.15 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(1,4) = ((((( 0.25 *  c_q_LH_HAA_) - (( 0.15 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.15 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(1,5) = ((( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(2,0) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) = ((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = (((((( 0.15 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  s_q_LH_HAA_) + (((- 0.15 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(2,4) = ((((( 0.25 *  s_q_LH_HAA_) + ((( 0.15 *  c_q_LH_HAA_) +  0.116) *  c_q_LH_HFE_)) + (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((( 0.15 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(2,5) = (((- 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,4) = ((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(4,3) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,4) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(4,5) =  c_q_LH_HAA_;
    (*this)(5,3) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,4) = ((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_base::Type_fr_LH_SHANK_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,2) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,3) = ((((- 0.15 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.15 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,4) = (((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.15 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  c_q_LH_HAA_) + (( 0.15 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(0,5) = (((((( 0.15 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  s_q_LH_HAA_) + (((- 0.15 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(1,0) = ((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(1,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(1,2) = ((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,3) = (((( 0.15 + ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.15 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,4) = ((((( 0.25 *  c_q_LH_HAA_) - (( 0.15 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.15 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(1,5) = ((((( 0.25 *  s_q_LH_HAA_) + ((( 0.15 *  c_q_LH_HAA_) +  0.116) *  c_q_LH_HFE_)) + (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((( 0.15 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(2,1) =  c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = (( 0.25 *  c_q_LH_HFE_) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(2,4) = ((( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(2,5) = (((- 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,4) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,5) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,3) = ((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(4,4) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(4,5) = ((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,4) =  c_q_LH_HAA_;
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP::Type_fr_base_X_fr_RH_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0.116;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,3) = ( 0.116 *  c_q_RH_HAA_);
    (*this)(0,4) = (- 0.116 *  s_q_RH_HAA_);
    (*this)(1,0) =  s_q_RH_HAA_;
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,3) = (- 0.277 *  c_q_RH_HAA_);
    (*this)(1,4) = ( 0.277 *  s_q_RH_HAA_);
    (*this)(2,0) = - c_q_RH_HAA_;
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,3) = (- 0.277 *  s_q_RH_HAA_);
    (*this)(2,4) = (- 0.277 *  c_q_RH_HAA_);
    (*this)(4,3) =  s_q_RH_HAA_;
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(5,3) = - c_q_RH_HAA_;
    (*this)(5,4) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base::Type_fr_RH_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0.116;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,1) =  s_q_RH_HAA_;
    (*this)(0,2) = - c_q_RH_HAA_;
    (*this)(0,3) = ( 0.116 *  c_q_RH_HAA_);
    (*this)(0,4) = (- 0.277 *  c_q_RH_HAA_);
    (*this)(0,5) = (- 0.277 *  s_q_RH_HAA_);
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = (- 0.116 *  s_q_RH_HAA_);
    (*this)(1,4) = ( 0.277 *  s_q_RH_HAA_);
    (*this)(1,5) = (- 0.277 *  c_q_RH_HAA_);
    (*this)(3,4) =  s_q_RH_HAA_;
    (*this)(3,5) = - c_q_RH_HAA_;
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH::Type_fr_base_X_fr_RH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) =  c_q_RH_HFE_;
    (*this)(0,1) = - s_q_RH_HFE_;
    (*this)(0,3) = ((( 0.116 *  c_q_RH_HAA_) +  0.041) *  s_q_RH_HFE_);
    (*this)(0,4) = ((( 0.116 *  c_q_RH_HAA_) +  0.041) *  c_q_RH_HFE_);
    (*this)(0,5) = (- 0.116 *  s_q_RH_HAA_);
    (*this)(1,0) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(1,1) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = (((- 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.041 *  s_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(1,4) = ((( 0.041 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(1,5) = ( 0.3405 *  s_q_RH_HAA_);
    (*this)(2,0) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(2,1) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = ((( 0.116 + ( 0.041 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_));
    (*this)(2,4) = (((- 0.116 - ( 0.041 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(2,5) = (- 0.3405 *  c_q_RH_HAA_);
    (*this)(3,3) =  c_q_RH_HFE_;
    (*this)(3,4) = - s_q_RH_HFE_;
    (*this)(4,3) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(4,4) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(4,5) =  c_q_RH_HAA_;
    (*this)(5,3) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(5,4) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_base::Type_fr_RH_THIGH_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) =  c_q_RH_HFE_;
    (*this)(0,1) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(0,2) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(0,3) = ((( 0.116 *  c_q_RH_HAA_) +  0.041) *  s_q_RH_HFE_);
    (*this)(0,4) = (((- 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.041 *  s_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(0,5) = ((( 0.116 + ( 0.041 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_));
    (*this)(1,0) = - s_q_RH_HFE_;
    (*this)(1,1) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,2) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,3) = ((( 0.116 *  c_q_RH_HAA_) +  0.041) *  c_q_RH_HFE_);
    (*this)(1,4) = ((( 0.041 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(1,5) = (((- 0.116 - ( 0.041 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(2,1) =  c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (- 0.116 *  s_q_RH_HAA_);
    (*this)(2,4) = ( 0.3405 *  s_q_RH_HAA_);
    (*this)(2,5) = (- 0.3405 *  c_q_RH_HAA_);
    (*this)(3,3) =  c_q_RH_HFE_;
    (*this)(3,4) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(3,5) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(4,3) = - s_q_RH_HFE_;
    (*this)(4,4) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(4,5) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(5,4) =  c_q_RH_HAA_;
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK::Type_fr_base_X_fr_RH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,1) = ((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(0,3) = (((( 0.15 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.15 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,4) = ((((- 0.15 - ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.15 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,5) = (( 0.25 *  c_q_RH_HFE_) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(1,0) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = ((((( 0.15 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  c_q_RH_HAA_) - (( 0.15 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(1,4) = ((((( 0.25 *  c_q_RH_HAA_) + (( 0.15 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.15 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(1,5) = ((( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(2,0) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) = ((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = ((((((- 0.15 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  s_q_RH_HAA_) + ((( 0.15 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(2,4) = ((((( 0.25 *  s_q_RH_HAA_) + (((- 0.15 *  c_q_RH_HAA_) -  0.116) *  c_q_RH_HFE_)) + (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.15 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(2,5) = (((- 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,4) = ((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(4,3) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,4) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(4,5) =  c_q_RH_HAA_;
    (*this)(5,3) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,4) = ((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_base::Type_fr_RH_SHANK_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,2) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,3) = (((( 0.15 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.15 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,4) = ((((( 0.15 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  c_q_RH_HAA_) - (( 0.15 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(0,5) = ((((((- 0.15 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  s_q_RH_HAA_) + ((( 0.15 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(1,0) = ((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(1,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(1,2) = ((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,3) = ((((- 0.15 - ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.15 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,4) = ((((( 0.25 *  c_q_RH_HAA_) + (( 0.15 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.15 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(1,5) = ((((( 0.25 *  s_q_RH_HAA_) + (((- 0.15 *  c_q_RH_HAA_) -  0.116) *  c_q_RH_HFE_)) + (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.15 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(2,1) =  c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (( 0.25 *  c_q_RH_HFE_) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(2,4) = ((( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(2,5) = (((- 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,4) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,5) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,3) = ((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(4,4) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(4,5) = ((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,4) =  c_q_RH_HAA_;
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_ADAPTER::Type_fr_base_X_fr_LF_ADAPTER()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_ADAPTER& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_ADAPTER::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,2) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(0,3) = ((((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,4) = ((((( 0.1 *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( 0.1 *  s_q_LF_HFE_) *  c_q_LF_KFE_)) + ( 0.25 *  c_q_LF_HFE_)) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(0,5) = ((((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.13 + ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,0) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,3) = ((((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.13 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  c_q_LF_HAA_) + (( 0.13 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(1,4) = (((((( 0.1 *  s_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) - ((( 0.1 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(1,5) = ((((((- 0.25 *  c_q_LF_HAA_) + (( 0.13 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.13 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.1 *  c_q_LF_HAA_));
    (*this)(2,0) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(2,3) = ((((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.13 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(2,4) = ((((((- 0.1 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.1 *  c_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.3405 *  c_q_LF_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.13 *  c_q_LF_HAA_) -  0.116) *  s_q_LF_HFE_) - (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.1 *  s_q_LF_HAA_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,5) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(4,3) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,3) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,4) =  s_q_LF_HAA_;
    (*this)(5,5) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_ADAPTER_X_fr_base::Type_fr_LF_ADAPTER_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_ADAPTER_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_ADAPTER_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,2) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,3) = ((((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,4) = ((((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.13 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  c_q_LF_HAA_) + (( 0.13 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(0,5) = ((((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.13 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = ((((( 0.1 *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( 0.1 *  s_q_LF_HFE_) *  c_q_LF_KFE_)) + ( 0.25 *  c_q_LF_HFE_)) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(1,4) = (((((( 0.1 *  s_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) - ((( 0.1 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(1,5) = ((((((- 0.1 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.1 *  c_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.3405 *  c_q_LF_HAA_));
    (*this)(2,0) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(2,1) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(2,3) = ((((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.13 + ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,4) = ((((((- 0.25 *  c_q_LF_HAA_) + (( 0.13 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.13 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.1 *  c_q_LF_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.13 *  c_q_LF_HAA_) -  0.116) *  s_q_LF_HFE_) - (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.1 *  s_q_LF_HAA_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,4) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,5) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
    (*this)(5,3) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(5,4) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,5) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT::Type_fr_base_X_fr_LF_FOOT()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,2) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(0,3) = ((((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,4) = (((((( 0.1 *  c_q_LF_HFE_) - ( 0.32125 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.32125 *  c_q_LF_HFE_) + ( 0.1 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.25 *  c_q_LF_HFE_)) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(0,5) = ((((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.13 + ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,0) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,3) = (((((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.13 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  c_q_LF_HAA_) + (( 0.13 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.32125 *  c_q_LF_HAA_));
    (*this)(1,4) = ((((((( 0.32125 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.1 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.32125 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.1 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(1,5) = ((((((- 0.25 *  c_q_LF_HAA_) + (( 0.13 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.13 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.1 *  c_q_LF_HAA_));
    (*this)(2,0) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(2,3) = (((((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.13 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.32125 *  s_q_LF_HAA_));
    (*this)(2,4) = (((((((- 0.32125 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.1 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.1 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.32125 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.3405 *  c_q_LF_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.13 *  c_q_LF_HAA_) -  0.116) *  s_q_LF_HFE_) - (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.1 *  s_q_LF_HAA_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,5) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(4,3) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,3) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,4) =  s_q_LF_HAA_;
    (*this)(5,5) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base::Type_fr_LF_FOOT_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,2) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,3) = ((((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,4) = (((((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.13 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  c_q_LF_HAA_) + (( 0.13 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.32125 *  c_q_LF_HAA_));
    (*this)(0,5) = (((((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.13 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.32125 *  s_q_LF_HAA_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = (((((( 0.1 *  c_q_LF_HFE_) - ( 0.32125 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.32125 *  c_q_LF_HFE_) + ( 0.1 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.25 *  c_q_LF_HFE_)) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(1,4) = ((((((( 0.32125 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.1 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.32125 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.1 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(1,5) = (((((((- 0.32125 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.1 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.1 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.32125 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.3405 *  c_q_LF_HAA_));
    (*this)(2,0) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(2,1) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(2,3) = ((((- 0.13 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.13 + ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,4) = ((((((- 0.25 *  c_q_LF_HAA_) + (( 0.13 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.13 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.1 *  c_q_LF_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.13 *  c_q_LF_HAA_) -  0.116) *  s_q_LF_HFE_) - (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.1 *  s_q_LF_HAA_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,4) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,5) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
    (*this)(5,3) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(5,4) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,5) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP_COM::Type_fr_base_X_fr_LF_HIP_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP_COM& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP_COM::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,4) = (( 0.116 *  s_q_LF_HAA_) +  1.5E-4);
    (*this)(0,5) = (( 0.116 *  c_q_LF_HAA_) -  0.00379);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = - s_q_LF_HAA_;
    (*this)(1,3) = ((- 0.00379 *  s_q_LF_HAA_) - ( 1.5E-4 *  c_q_LF_HAA_));
    (*this)(1,4) = (- 0.34152 *  s_q_LF_HAA_);
    (*this)(1,5) = (- 0.34152 *  c_q_LF_HAA_);
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) =  c_q_LF_HAA_;
    (*this)(2,3) = (((- 1.5E-4 *  s_q_LF_HAA_) + ( 0.00379 *  c_q_LF_HAA_)) -  0.116);
    (*this)(2,4) = ( 0.34152 *  c_q_LF_HAA_);
    (*this)(2,5) = (- 0.34152 *  s_q_LF_HAA_);
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) = - s_q_LF_HAA_;
    (*this)(5,4) =  s_q_LF_HAA_;
    (*this)(5,5) =  c_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_COM_X_fr_base::Type_fr_LF_HIP_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_COM_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,4) = ((- 0.00379 *  s_q_LF_HAA_) - ( 1.5E-4 *  c_q_LF_HAA_));
    (*this)(0,5) = (((- 1.5E-4 *  s_q_LF_HAA_) + ( 0.00379 *  c_q_LF_HAA_)) -  0.116);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = (( 0.116 *  s_q_LF_HAA_) +  1.5E-4);
    (*this)(1,4) = (- 0.34152 *  s_q_LF_HAA_);
    (*this)(1,5) = ( 0.34152 *  c_q_LF_HAA_);
    (*this)(2,1) = - s_q_LF_HAA_;
    (*this)(2,2) =  c_q_LF_HAA_;
    (*this)(2,3) = (( 0.116 *  c_q_LF_HAA_) -  0.00379);
    (*this)(2,4) = (- 0.34152 *  c_q_LF_HAA_);
    (*this)(2,5) = (- 0.34152 *  s_q_LF_HAA_);
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
    (*this)(5,4) = - s_q_LF_HAA_;
    (*this)(5,5) =  c_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK_COM::Type_fr_base_X_fr_LF_SHANK_COM()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK_COM& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK_COM::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,2) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(0,3) = ((((- 0.13918 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.13918 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,4) = (((((( 0.05873 *  c_q_LF_HFE_) - ( 0.09806 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.09806 *  c_q_LF_HFE_) + ( 0.05873 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.25 *  c_q_LF_HFE_)) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(0,5) = ((((- 0.13918 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.13918 + ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,0) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,3) = (((((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.13918 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  c_q_LF_HAA_) + (( 0.13918 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.09806 *  c_q_LF_HAA_));
    (*this)(1,4) = ((((((( 0.09806 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.05873 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.09806 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.05873 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(1,5) = ((((((- 0.25 *  c_q_LF_HAA_) + (( 0.13918 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.13918 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.05873 *  c_q_LF_HAA_));
    (*this)(2,0) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(2,3) = (((((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.13918 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13918 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.09806 *  s_q_LF_HAA_));
    (*this)(2,4) = (((((((- 0.09806 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.05873 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.05873 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.09806 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.3405 *  c_q_LF_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13918 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.13918 *  c_q_LF_HAA_) -  0.116) *  s_q_LF_HFE_) - (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.05873 *  s_q_LF_HAA_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,5) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(4,3) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,3) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,4) =  s_q_LF_HAA_;
    (*this)(5,5) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_SHANK_COM_X_fr_base::Type_fr_LF_SHANK_COM_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_SHANK_COM_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_SHANK_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,2) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,3) = ((((- 0.13918 - ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((- 0.13918 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,4) = (((((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.13918 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  c_q_LF_HAA_) + (( 0.13918 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.09806 *  c_q_LF_HAA_));
    (*this)(0,5) = (((((( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ((( 0.13918 *  c_q_LF_HAA_) +  0.116) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13918 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.09806 *  s_q_LF_HAA_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = (((((( 0.05873 *  c_q_LF_HFE_) - ( 0.09806 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.09806 *  c_q_LF_HFE_) + ( 0.05873 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.25 *  c_q_LF_HFE_)) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(1,4) = ((((((( 0.09806 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.05873 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.09806 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.05873 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(1,5) = (((((((- 0.09806 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.05873 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.05873 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.09806 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.3405 *  c_q_LF_HAA_));
    (*this)(2,0) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(2,1) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(2,3) = ((((- 0.13918 - ( 0.116 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.13918 + ( 0.116 *  c_q_LF_HAA_)) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,4) = ((((((- 0.25 *  c_q_LF_HAA_) + (( 0.13918 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + (( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.13918 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.05873 *  c_q_LF_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_LF_HAA_) + (((- 0.13918 *  c_q_LF_HAA_) -  0.116) *  c_q_LF_HFE_)) + (( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.13918 *  c_q_LF_HAA_) -  0.116) *  s_q_LF_HFE_) - (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.05873 *  s_q_LF_HAA_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,4) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,5) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
    (*this)(5,3) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(5,4) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,5) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH_COM::Type_fr_base_X_fr_LF_THIGH_COM()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH_COM& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH_COM::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) =  c_q_LF_HFE_;
    (*this)(0,2) =  s_q_LF_HFE_;
    (*this)(0,3) = (((- 0.116 *  c_q_LF_HAA_) -  0.09523) *  s_q_LF_HFE_);
    (*this)(0,4) = (((- 0.0039 *  s_q_LF_HFE_) + ( 0.21458 *  c_q_LF_HFE_)) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(0,5) = ((( 0.116 *  c_q_LF_HAA_) +  0.09523) *  c_q_LF_HFE_);
    (*this)(1,0) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = (- s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,3) = (((( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.09523 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.21458 *  c_q_LF_HAA_));
    (*this)(1,4) = (((( 0.21458 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.0039 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(1,5) = (((( 0.09523 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.0039 *  c_q_LF_HAA_));
    (*this)(2,0) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) = ( c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,3) = (((( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((- 0.116 - ( 0.09523 *  c_q_LF_HAA_)) *  c_q_LF_HFE_)) - ( 0.21458 *  s_q_LF_HAA_));
    (*this)(2,4) = ((((- 0.21458 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.0039 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.3405 *  c_q_LF_HAA_));
    (*this)(2,5) = ((((- 0.116 - ( 0.09523 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) - (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.0039 *  s_q_LF_HAA_));
    (*this)(3,3) =  c_q_LF_HFE_;
    (*this)(3,5) =  s_q_LF_HFE_;
    (*this)(4,3) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) = (- s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(5,3) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(5,4) =  s_q_LF_HAA_;
    (*this)(5,5) = ( c_q_LF_HAA_ *  c_q_LF_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_COM_X_fr_base::Type_fr_LF_THIGH_COM_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_COM_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) =  c_q_LF_HFE_;
    (*this)(0,1) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(0,2) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(0,3) = (((- 0.116 *  c_q_LF_HAA_) -  0.09523) *  s_q_LF_HFE_);
    (*this)(0,4) = (((( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.09523 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.21458 *  c_q_LF_HAA_));
    (*this)(0,5) = (((( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((- 0.116 - ( 0.09523 *  c_q_LF_HAA_)) *  c_q_LF_HFE_)) - ( 0.21458 *  s_q_LF_HAA_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = (((- 0.0039 *  s_q_LF_HFE_) + ( 0.21458 *  c_q_LF_HFE_)) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(1,4) = (((( 0.21458 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.0039 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(1,5) = ((((- 0.21458 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.0039 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.3405 *  c_q_LF_HAA_));
    (*this)(2,0) =  s_q_LF_HFE_;
    (*this)(2,1) = (- s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,2) = ( c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,3) = ((( 0.116 *  c_q_LF_HAA_) +  0.09523) *  c_q_LF_HFE_);
    (*this)(2,4) = (((( 0.09523 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.0039 *  c_q_LF_HAA_));
    (*this)(2,5) = ((((- 0.116 - ( 0.09523 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) - (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.0039 *  s_q_LF_HAA_));
    (*this)(3,3) =  c_q_LF_HFE_;
    (*this)(3,4) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(3,5) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
    (*this)(5,3) =  s_q_LF_HFE_;
    (*this)(5,4) = (- s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(5,5) = ( c_q_LF_HAA_ *  c_q_LF_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_ADAPTER::Type_fr_base_X_fr_LH_ADAPTER()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_ADAPTER& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_ADAPTER::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,2) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(0,3) = ((((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,4) = (((((- 0.1 *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( 0.1 *  s_q_LH_HFE_) *  c_q_LH_KFE_)) + ( 0.25 *  c_q_LH_HFE_)) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(0,5) = ((((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.13 + ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,0) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,3) = (((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.13 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  c_q_LH_HAA_) + (( 0.13 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(1,4) = ((((((- 0.1 *  s_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.1 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(1,5) = ((((((- 0.25 *  c_q_LH_HAA_) + (( 0.13 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.13 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.1 *  c_q_LH_HAA_));
    (*this)(2,0) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(2,3) = (((((( 0.13 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(2,4) = (((((( 0.1 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) - ((( 0.1 *  c_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) - (( 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.1 *  s_q_LH_HAA_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,5) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(4,3) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,3) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,4) =  s_q_LH_HAA_;
    (*this)(5,5) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_ADAPTER_X_fr_base::Type_fr_LH_ADAPTER_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_ADAPTER_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_ADAPTER_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,2) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,3) = ((((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,4) = (((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.13 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  c_q_LH_HAA_) + (( 0.13 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(0,5) = (((((( 0.13 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = (((((- 0.1 *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( 0.1 *  s_q_LH_HFE_) *  c_q_LH_KFE_)) + ( 0.25 *  c_q_LH_HFE_)) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(1,4) = ((((((- 0.1 *  s_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.1 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(1,5) = (((((( 0.1 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) - ((( 0.1 *  c_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) - (( 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(2,0) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(2,1) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(2,3) = ((((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.13 + ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,4) = ((((((- 0.25 *  c_q_LH_HAA_) + (( 0.13 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.13 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.1 *  c_q_LH_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.1 *  s_q_LH_HAA_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,4) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,5) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
    (*this)(5,3) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(5,4) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,5) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT::Type_fr_base_X_fr_LH_FOOT()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,2) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(0,3) = ((((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,4) = ((((((- 0.1 *  c_q_LH_HFE_) - ( 0.32125 *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.32125 *  c_q_LH_HFE_) - ( 0.1 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.25 *  c_q_LH_HFE_)) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(0,5) = ((((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.13 + ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,0) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,3) = ((((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.13 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  c_q_LH_HAA_) + (( 0.13 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.32125 *  c_q_LH_HAA_));
    (*this)(1,4) = ((((((( 0.32125 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.1 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.1 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.32125 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(1,5) = ((((((- 0.25 *  c_q_LH_HAA_) + (( 0.13 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.13 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.1 *  c_q_LH_HAA_));
    (*this)(2,0) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(2,3) = ((((((( 0.13 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.32125 *  s_q_LH_HAA_));
    (*this)(2,4) = ((((((( 0.1 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.32125 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.1 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.32125 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.1 *  s_q_LH_HAA_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,5) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(4,3) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,3) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,4) =  s_q_LH_HAA_;
    (*this)(5,5) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base::Type_fr_LH_FOOT_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,2) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,3) = ((((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,4) = ((((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.13 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  c_q_LH_HAA_) + (( 0.13 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.32125 *  c_q_LH_HAA_));
    (*this)(0,5) = ((((((( 0.13 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.32125 *  s_q_LH_HAA_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = ((((((- 0.1 *  c_q_LH_HFE_) - ( 0.32125 *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.32125 *  c_q_LH_HFE_) - ( 0.1 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.25 *  c_q_LH_HFE_)) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(1,4) = ((((((( 0.32125 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.1 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.1 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.32125 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(1,5) = ((((((( 0.1 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.32125 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.1 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.32125 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(2,0) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(2,1) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(2,3) = ((((- 0.13 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.13 + ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,4) = ((((((- 0.25 *  c_q_LH_HAA_) + (( 0.13 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.13 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.1 *  c_q_LH_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (((- 0.13 *  c_q_LH_HAA_) -  0.116) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.1 *  s_q_LH_HAA_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,4) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,5) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
    (*this)(5,3) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(5,4) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,5) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP_COM::Type_fr_base_X_fr_LH_HIP_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP_COM& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP_COM::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,4) = (( 0.116 *  s_q_LH_HAA_) +  1.5E-4);
    (*this)(0,5) = (( 0.116 *  c_q_LH_HAA_) -  0.00379);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = - s_q_LH_HAA_;
    (*this)(1,3) = ((- 0.00379 *  s_q_LH_HAA_) - ( 1.5E-4 *  c_q_LH_HAA_));
    (*this)(1,4) = ( 0.34152 *  s_q_LH_HAA_);
    (*this)(1,5) = ( 0.34152 *  c_q_LH_HAA_);
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) =  c_q_LH_HAA_;
    (*this)(2,3) = (((- 1.5E-4 *  s_q_LH_HAA_) + ( 0.00379 *  c_q_LH_HAA_)) -  0.116);
    (*this)(2,4) = (- 0.34152 *  c_q_LH_HAA_);
    (*this)(2,5) = ( 0.34152 *  s_q_LH_HAA_);
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) = - s_q_LH_HAA_;
    (*this)(5,4) =  s_q_LH_HAA_;
    (*this)(5,5) =  c_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_COM_X_fr_base::Type_fr_LH_HIP_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_COM_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,4) = ((- 0.00379 *  s_q_LH_HAA_) - ( 1.5E-4 *  c_q_LH_HAA_));
    (*this)(0,5) = (((- 1.5E-4 *  s_q_LH_HAA_) + ( 0.00379 *  c_q_LH_HAA_)) -  0.116);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = (( 0.116 *  s_q_LH_HAA_) +  1.5E-4);
    (*this)(1,4) = ( 0.34152 *  s_q_LH_HAA_);
    (*this)(1,5) = (- 0.34152 *  c_q_LH_HAA_);
    (*this)(2,1) = - s_q_LH_HAA_;
    (*this)(2,2) =  c_q_LH_HAA_;
    (*this)(2,3) = (( 0.116 *  c_q_LH_HAA_) -  0.00379);
    (*this)(2,4) = ( 0.34152 *  c_q_LH_HAA_);
    (*this)(2,5) = ( 0.34152 *  s_q_LH_HAA_);
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
    (*this)(5,4) = - s_q_LH_HAA_;
    (*this)(5,5) =  c_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK_COM::Type_fr_base_X_fr_LH_SHANK_COM()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK_COM& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK_COM::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,2) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(0,3) = ((((- 0.13918 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.13918 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,4) = ((((((- 0.05873 *  c_q_LH_HFE_) - ( 0.09806 *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.09806 *  c_q_LH_HFE_) - ( 0.05873 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.25 *  c_q_LH_HFE_)) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(0,5) = ((((- 0.13918 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.13918 + ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,0) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,3) = ((((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.13918 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  c_q_LH_HAA_) + (( 0.13918 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.09806 *  c_q_LH_HAA_));
    (*this)(1,4) = ((((((( 0.09806 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.05873 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.05873 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.09806 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(1,5) = ((((((- 0.25 *  c_q_LH_HAA_) + (( 0.13918 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.13918 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.05873 *  c_q_LH_HAA_));
    (*this)(2,0) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(2,3) = ((((((( 0.13918 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13918 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.09806 *  s_q_LH_HAA_));
    (*this)(2,4) = ((((((( 0.05873 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.09806 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.05873 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.09806 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13918 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (((- 0.13918 *  c_q_LH_HAA_) -  0.116) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.05873 *  s_q_LH_HAA_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,5) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(4,3) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,3) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,4) =  s_q_LH_HAA_;
    (*this)(5,5) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_SHANK_COM_X_fr_base::Type_fr_LH_SHANK_COM_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_SHANK_COM_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_SHANK_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,2) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,3) = ((((- 0.13918 - ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((- 0.13918 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,4) = ((((((- 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.13918 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  c_q_LH_HAA_) + (( 0.13918 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.09806 *  c_q_LH_HAA_));
    (*this)(0,5) = ((((((( 0.13918 *  c_q_LH_HAA_) +  0.116) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13918 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.09806 *  s_q_LH_HAA_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = ((((((- 0.05873 *  c_q_LH_HFE_) - ( 0.09806 *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.09806 *  c_q_LH_HFE_) - ( 0.05873 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.25 *  c_q_LH_HFE_)) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(1,4) = ((((((( 0.09806 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.05873 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.05873 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.09806 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(1,5) = ((((((( 0.05873 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.09806 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.05873 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.09806 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(2,0) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(2,1) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(2,3) = ((((- 0.13918 - ( 0.116 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.13918 + ( 0.116 *  c_q_LH_HAA_)) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,4) = ((((((- 0.25 *  c_q_LH_HAA_) + (( 0.13918 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - (( 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.13918 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.05873 *  c_q_LH_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_LH_HAA_) + (((- 0.13918 *  c_q_LH_HAA_) -  0.116) *  c_q_LH_HFE_)) - (( 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + (((- 0.13918 *  c_q_LH_HAA_) -  0.116) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.05873 *  s_q_LH_HAA_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,4) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,5) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
    (*this)(5,3) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(5,4) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,5) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH_COM::Type_fr_base_X_fr_LH_THIGH_COM()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH_COM& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH_COM::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) =  c_q_LH_HFE_;
    (*this)(0,2) =  s_q_LH_HFE_;
    (*this)(0,3) = (((- 0.116 *  c_q_LH_HAA_) -  0.09523) *  s_q_LH_HFE_);
    (*this)(0,4) = ((( 0.0039 *  s_q_LH_HFE_) + ( 0.21458 *  c_q_LH_HFE_)) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(0,5) = ((( 0.116 *  c_q_LH_HAA_) +  0.09523) *  c_q_LH_HFE_);
    (*this)(1,0) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = (- s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,3) = ((((- 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.09523 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.21458 *  c_q_LH_HAA_));
    (*this)(1,4) = (((( 0.21458 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.0039 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(1,5) = (((( 0.09523 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.0039 *  c_q_LH_HAA_));
    (*this)(2,0) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) = ( c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,3) = ((((- 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((- 0.116 - ( 0.09523 *  c_q_LH_HAA_)) *  c_q_LH_HFE_)) - ( 0.21458 *  s_q_LH_HAA_));
    (*this)(2,4) = ((((- 0.21458 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.0039 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(2,5) = ((((- 0.116 - ( 0.09523 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) + (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.0039 *  s_q_LH_HAA_));
    (*this)(3,3) =  c_q_LH_HFE_;
    (*this)(3,5) =  s_q_LH_HFE_;
    (*this)(4,3) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) = (- s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(5,3) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(5,4) =  s_q_LH_HAA_;
    (*this)(5,5) = ( c_q_LH_HAA_ *  c_q_LH_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_COM_X_fr_base::Type_fr_LH_THIGH_COM_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_COM_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) =  c_q_LH_HFE_;
    (*this)(0,1) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(0,2) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(0,3) = (((- 0.116 *  c_q_LH_HAA_) -  0.09523) *  s_q_LH_HFE_);
    (*this)(0,4) = ((((- 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.09523 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.21458 *  c_q_LH_HAA_));
    (*this)(0,5) = ((((- 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((- 0.116 - ( 0.09523 *  c_q_LH_HAA_)) *  c_q_LH_HFE_)) - ( 0.21458 *  s_q_LH_HAA_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = ((( 0.0039 *  s_q_LH_HFE_) + ( 0.21458 *  c_q_LH_HFE_)) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(1,4) = (((( 0.21458 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.0039 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(1,5) = ((((- 0.21458 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.0039 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(2,0) =  s_q_LH_HFE_;
    (*this)(2,1) = (- s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,2) = ( c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,3) = ((( 0.116 *  c_q_LH_HAA_) +  0.09523) *  c_q_LH_HFE_);
    (*this)(2,4) = (((( 0.09523 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.0039 *  c_q_LH_HAA_));
    (*this)(2,5) = ((((- 0.116 - ( 0.09523 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) + (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.0039 *  s_q_LH_HAA_));
    (*this)(3,3) =  c_q_LH_HFE_;
    (*this)(3,4) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(3,5) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
    (*this)(5,3) =  s_q_LH_HFE_;
    (*this)(5,4) = (- s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(5,5) = ( c_q_LH_HAA_ *  c_q_LH_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_ADAPTER::Type_fr_base_X_fr_RF_ADAPTER()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_ADAPTER& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_ADAPTER::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,2) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(0,3) = (((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,4) = ((((( 0.1 *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( 0.1 *  s_q_RF_HFE_) *  c_q_RF_KFE_)) + ( 0.25 *  c_q_RF_HFE_)) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(0,5) = (((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((- 0.13 - ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,0) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,3) = ((((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.13 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  c_q_RF_HAA_) - (( 0.13 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(1,4) = (((((( 0.1 *  s_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) - ((( 0.1 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(1,5) = ((((((- 0.25 *  c_q_RF_HAA_) - (( 0.13 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.13 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.1 *  c_q_RF_HAA_));
    (*this)(2,0) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(2,3) = ((((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.13 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(2,4) = ((((((- 0.1 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.1 *  c_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.3405 *  c_q_RF_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.13 *  c_q_RF_HAA_) +  0.116) *  s_q_RF_HFE_) - (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.1 *  s_q_RF_HAA_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,5) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(4,3) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,3) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,4) =  s_q_RF_HAA_;
    (*this)(5,5) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_ADAPTER_X_fr_base::Type_fr_RF_ADAPTER_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_ADAPTER_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_ADAPTER_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,2) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,3) = (((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,4) = ((((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.13 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  c_q_RF_HAA_) - (( 0.13 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(0,5) = ((((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.13 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = ((((( 0.1 *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( 0.1 *  s_q_RF_HFE_) *  c_q_RF_KFE_)) + ( 0.25 *  c_q_RF_HFE_)) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(1,4) = (((((( 0.1 *  s_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) - ((( 0.1 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(1,5) = ((((((- 0.1 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.1 *  c_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.3405 *  c_q_RF_HAA_));
    (*this)(2,0) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(2,1) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(2,3) = (((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((- 0.13 - ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,4) = ((((((- 0.25 *  c_q_RF_HAA_) - (( 0.13 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.13 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.1 *  c_q_RF_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.13 *  c_q_RF_HAA_) +  0.116) *  s_q_RF_HFE_) - (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.1 *  s_q_RF_HAA_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,4) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,5) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
    (*this)(5,3) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(5,4) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,5) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT::Type_fr_base_X_fr_RF_FOOT()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,2) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(0,3) = (((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,4) = (((((( 0.1 *  c_q_RF_HFE_) - ( 0.32125 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.32125 *  c_q_RF_HFE_) + ( 0.1 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.25 *  c_q_RF_HFE_)) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(0,5) = (((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((- 0.13 - ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,0) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,3) = (((((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.13 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  c_q_RF_HAA_) - (( 0.13 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.32125 *  c_q_RF_HAA_));
    (*this)(1,4) = ((((((( 0.32125 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.1 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.32125 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.1 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(1,5) = ((((((- 0.25 *  c_q_RF_HAA_) - (( 0.13 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.13 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.1 *  c_q_RF_HAA_));
    (*this)(2,0) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(2,3) = (((((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.13 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.32125 *  s_q_RF_HAA_));
    (*this)(2,4) = (((((((- 0.32125 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.1 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.1 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.32125 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.3405 *  c_q_RF_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.13 *  c_q_RF_HAA_) +  0.116) *  s_q_RF_HFE_) - (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.1 *  s_q_RF_HAA_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,5) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(4,3) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,3) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,4) =  s_q_RF_HAA_;
    (*this)(5,5) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base::Type_fr_RF_FOOT_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,2) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,3) = (((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,4) = (((((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.13 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  c_q_RF_HAA_) - (( 0.13 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.32125 *  c_q_RF_HAA_));
    (*this)(0,5) = (((((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.13 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.32125 *  s_q_RF_HAA_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = (((((( 0.1 *  c_q_RF_HFE_) - ( 0.32125 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.32125 *  c_q_RF_HFE_) + ( 0.1 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.25 *  c_q_RF_HFE_)) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(1,4) = ((((((( 0.32125 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.1 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.32125 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.1 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(1,5) = (((((((- 0.32125 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.1 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.1 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.32125 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.3405 *  c_q_RF_HAA_));
    (*this)(2,0) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(2,1) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(2,3) = (((( 0.13 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((- 0.13 - ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,4) = ((((((- 0.25 *  c_q_RF_HAA_) - (( 0.13 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.13 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.1 *  c_q_RF_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.13 *  c_q_RF_HAA_) +  0.116) *  s_q_RF_HFE_) - (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.1 *  s_q_RF_HAA_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,4) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,5) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
    (*this)(5,3) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(5,4) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,5) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP_COM::Type_fr_base_X_fr_RF_HIP_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP_COM& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP_COM::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,4) = ( 1.5E-4 - ( 0.116 *  s_q_RF_HAA_));
    (*this)(0,5) = ( 0.00379 - ( 0.116 *  c_q_RF_HAA_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = - s_q_RF_HAA_;
    (*this)(1,3) = (( 0.00379 *  s_q_RF_HAA_) - ( 1.5E-4 *  c_q_RF_HAA_));
    (*this)(1,4) = (- 0.34152 *  s_q_RF_HAA_);
    (*this)(1,5) = (- 0.34152 *  c_q_RF_HAA_);
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) =  c_q_RF_HAA_;
    (*this)(2,3) = (((- 1.5E-4 *  s_q_RF_HAA_) - ( 0.00379 *  c_q_RF_HAA_)) +  0.116);
    (*this)(2,4) = ( 0.34152 *  c_q_RF_HAA_);
    (*this)(2,5) = (- 0.34152 *  s_q_RF_HAA_);
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) = - s_q_RF_HAA_;
    (*this)(5,4) =  s_q_RF_HAA_;
    (*this)(5,5) =  c_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_COM_X_fr_base::Type_fr_RF_HIP_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_COM_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,4) = (( 0.00379 *  s_q_RF_HAA_) - ( 1.5E-4 *  c_q_RF_HAA_));
    (*this)(0,5) = (((- 1.5E-4 *  s_q_RF_HAA_) - ( 0.00379 *  c_q_RF_HAA_)) +  0.116);
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = ( 1.5E-4 - ( 0.116 *  s_q_RF_HAA_));
    (*this)(1,4) = (- 0.34152 *  s_q_RF_HAA_);
    (*this)(1,5) = ( 0.34152 *  c_q_RF_HAA_);
    (*this)(2,1) = - s_q_RF_HAA_;
    (*this)(2,2) =  c_q_RF_HAA_;
    (*this)(2,3) = ( 0.00379 - ( 0.116 *  c_q_RF_HAA_));
    (*this)(2,4) = (- 0.34152 *  c_q_RF_HAA_);
    (*this)(2,5) = (- 0.34152 *  s_q_RF_HAA_);
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
    (*this)(5,4) = - s_q_RF_HAA_;
    (*this)(5,5) =  c_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK_COM::Type_fr_base_X_fr_RF_SHANK_COM()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK_COM& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK_COM::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,2) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(0,3) = (((( 0.13918 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.13918 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,4) = (((((( 0.05873 *  c_q_RF_HFE_) - ( 0.09806 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.09806 *  c_q_RF_HFE_) + ( 0.05873 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.25 *  c_q_RF_HFE_)) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(0,5) = (((( 0.13918 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((- 0.13918 - ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,0) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,3) = (((((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.13918 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  c_q_RF_HAA_) - (( 0.13918 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.09806 *  c_q_RF_HAA_));
    (*this)(1,4) = ((((((( 0.09806 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.05873 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.09806 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.05873 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(1,5) = ((((((- 0.25 *  c_q_RF_HAA_) - (( 0.13918 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.13918 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.05873 *  c_q_RF_HAA_));
    (*this)(2,0) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(2,3) = (((((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.13918 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13918 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.09806 *  s_q_RF_HAA_));
    (*this)(2,4) = (((((((- 0.09806 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.05873 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.05873 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.09806 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.3405 *  c_q_RF_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13918 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.13918 *  c_q_RF_HAA_) +  0.116) *  s_q_RF_HFE_) - (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.05873 *  s_q_RF_HAA_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,5) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(4,3) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,3) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,4) =  s_q_RF_HAA_;
    (*this)(5,5) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_SHANK_COM_X_fr_base::Type_fr_RF_SHANK_COM_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_SHANK_COM_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_SHANK_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,2) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,3) = (((( 0.13918 + ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.13918 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,4) = (((((( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.13918 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  c_q_RF_HAA_) - (( 0.13918 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.09806 *  c_q_RF_HAA_));
    (*this)(0,5) = (((((( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (((- 0.13918 *  c_q_RF_HAA_) -  0.116) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13918 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.09806 *  s_q_RF_HAA_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = (((((( 0.05873 *  c_q_RF_HFE_) - ( 0.09806 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.09806 *  c_q_RF_HFE_) + ( 0.05873 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.25 *  c_q_RF_HFE_)) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(1,4) = ((((((( 0.09806 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.05873 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.09806 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.05873 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(1,5) = (((((((- 0.09806 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.05873 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.05873 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.09806 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.3405 *  c_q_RF_HAA_));
    (*this)(2,0) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(2,1) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(2,3) = (((( 0.13918 + ( 0.116 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((- 0.13918 - ( 0.116 *  c_q_RF_HAA_)) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,4) = ((((((- 0.25 *  c_q_RF_HAA_) - (( 0.13918 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + (( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.13918 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.05873 *  c_q_RF_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_RF_HAA_) + ((( 0.13918 *  c_q_RF_HAA_) +  0.116) *  c_q_RF_HFE_)) + (( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.13918 *  c_q_RF_HAA_) +  0.116) *  s_q_RF_HFE_) - (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.05873 *  s_q_RF_HAA_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,4) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,5) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
    (*this)(5,3) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(5,4) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,5) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH_COM::Type_fr_base_X_fr_RF_THIGH_COM()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH_COM& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH_COM::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) =  c_q_RF_HFE_;
    (*this)(0,2) =  s_q_RF_HFE_;
    (*this)(0,3) = ((( 0.116 *  c_q_RF_HAA_) +  0.09523) *  s_q_RF_HFE_);
    (*this)(0,4) = (((- 0.0039 *  s_q_RF_HFE_) + ( 0.21458 *  c_q_RF_HFE_)) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(0,5) = (((- 0.116 *  c_q_RF_HAA_) -  0.09523) *  c_q_RF_HFE_);
    (*this)(1,0) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = (- s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,3) = (((( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.09523 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.21458 *  c_q_RF_HAA_));
    (*this)(1,4) = (((( 0.21458 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.0039 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(1,5) = ((((- 0.09523 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.0039 *  c_q_RF_HAA_));
    (*this)(2,0) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) = ( c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,3) = (((( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.116 + ( 0.09523 *  c_q_RF_HAA_)) *  c_q_RF_HFE_)) - ( 0.21458 *  s_q_RF_HAA_));
    (*this)(2,4) = ((((- 0.21458 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.0039 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.3405 *  c_q_RF_HAA_));
    (*this)(2,5) = (((( 0.116 + ( 0.09523 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) - (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.0039 *  s_q_RF_HAA_));
    (*this)(3,3) =  c_q_RF_HFE_;
    (*this)(3,5) =  s_q_RF_HFE_;
    (*this)(4,3) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) = (- s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(5,3) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(5,4) =  s_q_RF_HAA_;
    (*this)(5,5) = ( c_q_RF_HAA_ *  c_q_RF_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_COM_X_fr_base::Type_fr_RF_THIGH_COM_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_COM_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) =  c_q_RF_HFE_;
    (*this)(0,1) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(0,2) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(0,3) = ((( 0.116 *  c_q_RF_HAA_) +  0.09523) *  s_q_RF_HFE_);
    (*this)(0,4) = (((( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.09523 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.21458 *  c_q_RF_HAA_));
    (*this)(0,5) = (((( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.116 + ( 0.09523 *  c_q_RF_HAA_)) *  c_q_RF_HFE_)) - ( 0.21458 *  s_q_RF_HAA_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = (((- 0.0039 *  s_q_RF_HFE_) + ( 0.21458 *  c_q_RF_HFE_)) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(1,4) = (((( 0.21458 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.0039 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(1,5) = ((((- 0.21458 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.0039 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.3405 *  c_q_RF_HAA_));
    (*this)(2,0) =  s_q_RF_HFE_;
    (*this)(2,1) = (- s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,2) = ( c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,3) = (((- 0.116 *  c_q_RF_HAA_) -  0.09523) *  c_q_RF_HFE_);
    (*this)(2,4) = ((((- 0.09523 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.0039 *  c_q_RF_HAA_));
    (*this)(2,5) = (((( 0.116 + ( 0.09523 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) - (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.0039 *  s_q_RF_HAA_));
    (*this)(3,3) =  c_q_RF_HFE_;
    (*this)(3,4) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(3,5) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
    (*this)(5,3) =  s_q_RF_HFE_;
    (*this)(5,4) = (- s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(5,5) = ( c_q_RF_HAA_ *  c_q_RF_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_ADAPTER::Type_fr_base_X_fr_RH_ADAPTER()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_ADAPTER& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_ADAPTER::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,2) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(0,3) = (((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,4) = (((((- 0.1 *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( 0.1 *  s_q_RH_HFE_) *  c_q_RH_KFE_)) + ( 0.25 *  c_q_RH_HFE_)) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(0,5) = (((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((- 0.13 - ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,0) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,3) = ((((( 0.13 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  c_q_RH_HAA_) - (( 0.13 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(1,4) = ((((((- 0.1 *  s_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.1 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(1,5) = ((((((- 0.25 *  c_q_RH_HAA_) - (( 0.13 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.13 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.1 *  c_q_RH_HAA_));
    (*this)(2,0) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(2,3) = ((((((- 0.13 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(2,4) = (((((( 0.1 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) - ((( 0.1 *  c_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) - (( 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.1 *  s_q_RH_HAA_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,5) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(4,3) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,3) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,4) =  s_q_RH_HAA_;
    (*this)(5,5) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_ADAPTER_X_fr_base::Type_fr_RH_ADAPTER_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_ADAPTER_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_ADAPTER_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,2) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,3) = (((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,4) = ((((( 0.13 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  c_q_RH_HAA_) - (( 0.13 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(0,5) = ((((((- 0.13 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = (((((- 0.1 *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( 0.1 *  s_q_RH_HFE_) *  c_q_RH_KFE_)) + ( 0.25 *  c_q_RH_HFE_)) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(1,4) = ((((((- 0.1 *  s_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.1 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(1,5) = (((((( 0.1 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) - ((( 0.1 *  c_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) - (( 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(2,0) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(2,1) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(2,3) = (((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((- 0.13 - ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,4) = ((((((- 0.25 *  c_q_RH_HAA_) - (( 0.13 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.13 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.1 *  c_q_RH_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.1 *  s_q_RH_HAA_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,4) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,5) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
    (*this)(5,3) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(5,4) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,5) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT::Type_fr_base_X_fr_RH_FOOT()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,2) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(0,3) = (((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,4) = ((((((- 0.1 *  c_q_RH_HFE_) - ( 0.32125 *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.32125 *  c_q_RH_HFE_) - ( 0.1 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.25 *  c_q_RH_HFE_)) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(0,5) = (((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((- 0.13 - ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,0) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,3) = (((((( 0.13 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  c_q_RH_HAA_) - (( 0.13 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.32125 *  c_q_RH_HAA_));
    (*this)(1,4) = ((((((( 0.32125 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.1 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.1 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + (( 0.32125 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(1,5) = ((((((- 0.25 *  c_q_RH_HAA_) - (( 0.13 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.13 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.1 *  c_q_RH_HAA_));
    (*this)(2,0) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(2,3) = (((((((- 0.13 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.32125 *  s_q_RH_HAA_));
    (*this)(2,4) = ((((((( 0.1 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.32125 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.1 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.32125 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.1 *  s_q_RH_HAA_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,5) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(4,3) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,3) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,4) =  s_q_RH_HAA_;
    (*this)(5,5) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base::Type_fr_RH_FOOT_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,2) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,3) = (((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,4) = (((((( 0.13 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  c_q_RH_HAA_) - (( 0.13 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.32125 *  c_q_RH_HAA_));
    (*this)(0,5) = (((((((- 0.13 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.32125 *  s_q_RH_HAA_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = ((((((- 0.1 *  c_q_RH_HFE_) - ( 0.32125 *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.32125 *  c_q_RH_HFE_) - ( 0.1 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.25 *  c_q_RH_HFE_)) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(1,4) = ((((((( 0.32125 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.1 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.1 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + (( 0.32125 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(1,5) = ((((((( 0.1 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.32125 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.1 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.32125 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(2,0) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(2,1) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(2,3) = (((( 0.13 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((- 0.13 - ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,4) = ((((((- 0.25 *  c_q_RH_HAA_) - (( 0.13 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.13 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.1 *  c_q_RH_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + ((( 0.13 *  c_q_RH_HAA_) +  0.116) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.1 *  s_q_RH_HAA_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,4) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,5) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
    (*this)(5,3) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(5,4) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,5) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP_COM::Type_fr_base_X_fr_RH_HIP_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP_COM& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP_COM::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,4) = ( 1.5E-4 - ( 0.116 *  s_q_RH_HAA_));
    (*this)(0,5) = ( 0.00379 - ( 0.116 *  c_q_RH_HAA_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = - s_q_RH_HAA_;
    (*this)(1,3) = (( 0.00379 *  s_q_RH_HAA_) - ( 1.5E-4 *  c_q_RH_HAA_));
    (*this)(1,4) = ( 0.34152 *  s_q_RH_HAA_);
    (*this)(1,5) = ( 0.34152 *  c_q_RH_HAA_);
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) =  c_q_RH_HAA_;
    (*this)(2,3) = (((- 1.5E-4 *  s_q_RH_HAA_) - ( 0.00379 *  c_q_RH_HAA_)) +  0.116);
    (*this)(2,4) = (- 0.34152 *  c_q_RH_HAA_);
    (*this)(2,5) = ( 0.34152 *  s_q_RH_HAA_);
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) = - s_q_RH_HAA_;
    (*this)(5,4) =  s_q_RH_HAA_;
    (*this)(5,5) =  c_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_COM_X_fr_base::Type_fr_RH_HIP_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_COM_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,4) = (( 0.00379 *  s_q_RH_HAA_) - ( 1.5E-4 *  c_q_RH_HAA_));
    (*this)(0,5) = (((- 1.5E-4 *  s_q_RH_HAA_) - ( 0.00379 *  c_q_RH_HAA_)) +  0.116);
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = ( 1.5E-4 - ( 0.116 *  s_q_RH_HAA_));
    (*this)(1,4) = ( 0.34152 *  s_q_RH_HAA_);
    (*this)(1,5) = (- 0.34152 *  c_q_RH_HAA_);
    (*this)(2,1) = - s_q_RH_HAA_;
    (*this)(2,2) =  c_q_RH_HAA_;
    (*this)(2,3) = ( 0.00379 - ( 0.116 *  c_q_RH_HAA_));
    (*this)(2,4) = ( 0.34152 *  c_q_RH_HAA_);
    (*this)(2,5) = ( 0.34152 *  s_q_RH_HAA_);
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
    (*this)(5,4) = - s_q_RH_HAA_;
    (*this)(5,5) =  c_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK_COM::Type_fr_base_X_fr_RH_SHANK_COM()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK_COM& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK_COM::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,2) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(0,3) = (((( 0.13918 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.13918 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,4) = ((((((- 0.05873 *  c_q_RH_HFE_) - ( 0.09806 *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.09806 *  c_q_RH_HFE_) - ( 0.05873 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.25 *  c_q_RH_HFE_)) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(0,5) = (((( 0.13918 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((- 0.13918 - ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,0) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,3) = (((((( 0.13918 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  c_q_RH_HAA_) - (( 0.13918 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.09806 *  c_q_RH_HAA_));
    (*this)(1,4) = ((((((( 0.09806 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.05873 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.05873 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + (( 0.09806 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(1,5) = ((((((- 0.25 *  c_q_RH_HAA_) - (( 0.13918 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.13918 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.05873 *  c_q_RH_HAA_));
    (*this)(2,0) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(2,3) = (((((((- 0.13918 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13918 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.09806 *  s_q_RH_HAA_));
    (*this)(2,4) = ((((((( 0.05873 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.09806 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.05873 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.09806 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13918 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + ((( 0.13918 *  c_q_RH_HAA_) +  0.116) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.05873 *  s_q_RH_HAA_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,5) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(4,3) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,3) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,4) =  s_q_RH_HAA_;
    (*this)(5,5) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_SHANK_COM_X_fr_base::Type_fr_RH_SHANK_COM_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_SHANK_COM_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_SHANK_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,2) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,3) = (((( 0.13918 + ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.13918 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,4) = (((((( 0.13918 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  c_q_RH_HAA_) - (( 0.13918 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.09806 *  c_q_RH_HAA_));
    (*this)(0,5) = (((((((- 0.13918 *  c_q_RH_HAA_) -  0.116) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13918 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.09806 *  s_q_RH_HAA_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = ((((((- 0.05873 *  c_q_RH_HFE_) - ( 0.09806 *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.09806 *  c_q_RH_HFE_) - ( 0.05873 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.25 *  c_q_RH_HFE_)) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(1,4) = ((((((( 0.09806 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.05873 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.05873 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + (( 0.09806 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(1,5) = ((((((( 0.05873 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.09806 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.05873 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.09806 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(2,0) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(2,1) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(2,3) = (((( 0.13918 + ( 0.116 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((- 0.13918 - ( 0.116 *  c_q_RH_HAA_)) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,4) = ((((((- 0.25 *  c_q_RH_HAA_) - (( 0.13918 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - (( 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.13918 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.05873 *  c_q_RH_HAA_));
    (*this)(2,5) = ((((((- 0.25 *  s_q_RH_HAA_) + ((( 0.13918 *  c_q_RH_HAA_) +  0.116) *  c_q_RH_HFE_)) - (( 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_) + ((( 0.13918 *  c_q_RH_HAA_) +  0.116) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.05873 *  s_q_RH_HAA_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,4) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,5) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
    (*this)(5,3) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(5,4) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,5) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH_COM::Type_fr_base_X_fr_RH_THIGH_COM()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH_COM& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH_COM::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) =  c_q_RH_HFE_;
    (*this)(0,2) =  s_q_RH_HFE_;
    (*this)(0,3) = ((( 0.116 *  c_q_RH_HAA_) +  0.09523) *  s_q_RH_HFE_);
    (*this)(0,4) = ((( 0.0039 *  s_q_RH_HFE_) + ( 0.21458 *  c_q_RH_HFE_)) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(0,5) = (((- 0.116 *  c_q_RH_HAA_) -  0.09523) *  c_q_RH_HFE_);
    (*this)(1,0) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = (- s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,3) = ((((- 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.09523 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.21458 *  c_q_RH_HAA_));
    (*this)(1,4) = (((( 0.21458 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.0039 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(1,5) = ((((- 0.09523 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.0039 *  c_q_RH_HAA_));
    (*this)(2,0) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) = ( c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,3) = ((((- 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.116 + ( 0.09523 *  c_q_RH_HAA_)) *  c_q_RH_HFE_)) - ( 0.21458 *  s_q_RH_HAA_));
    (*this)(2,4) = ((((- 0.21458 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.0039 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(2,5) = (((( 0.116 + ( 0.09523 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) + (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.0039 *  s_q_RH_HAA_));
    (*this)(3,3) =  c_q_RH_HFE_;
    (*this)(3,5) =  s_q_RH_HFE_;
    (*this)(4,3) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) = (- s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(5,3) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(5,4) =  s_q_RH_HAA_;
    (*this)(5,5) = ( c_q_RH_HAA_ *  c_q_RH_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_COM_X_fr_base::Type_fr_RH_THIGH_COM_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_COM_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) =  c_q_RH_HFE_;
    (*this)(0,1) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(0,2) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(0,3) = ((( 0.116 *  c_q_RH_HAA_) +  0.09523) *  s_q_RH_HFE_);
    (*this)(0,4) = ((((- 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.09523 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.21458 *  c_q_RH_HAA_));
    (*this)(0,5) = ((((- 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.116 + ( 0.09523 *  c_q_RH_HAA_)) *  c_q_RH_HFE_)) - ( 0.21458 *  s_q_RH_HAA_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = ((( 0.0039 *  s_q_RH_HFE_) + ( 0.21458 *  c_q_RH_HFE_)) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(1,4) = (((( 0.21458 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.0039 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(1,5) = ((((- 0.21458 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.0039 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(2,0) =  s_q_RH_HFE_;
    (*this)(2,1) = (- s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,2) = ( c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,3) = (((- 0.116 *  c_q_RH_HAA_) -  0.09523) *  c_q_RH_HFE_);
    (*this)(2,4) = ((((- 0.09523 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.0039 *  c_q_RH_HAA_));
    (*this)(2,5) = (((( 0.116 + ( 0.09523 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) + (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.0039 *  s_q_RH_HAA_));
    (*this)(3,3) =  c_q_RH_HFE_;
    (*this)(3,4) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(3,5) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
    (*this)(5,3) =  s_q_RH_HFE_;
    (*this)(5,4) = (- s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(5,5) = ( c_q_RH_HAA_ *  c_q_RH_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_base_COM::Type_fr_base_X_fr_base_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = - 0.05056;
    (*this)(0,5) = 0.00324;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.05056;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0.00831;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.00324;
    (*this)(2,4) = - 0.00831;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_base_COM& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_base_COM::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_COM_X_fr_base::Type_fr_base_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0.05056;
    (*this)(0,5) = - 0.00324;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.05056;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.00831;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.00324;
    (*this)(2,4) = 0.00831;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_COM_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_COM_X_fr_base::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_base_inertia::Type_fr_base_X_fr_base_inertia()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_base_inertia& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_base_inertia::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_inertia_X_fr_base::Type_fr_base_inertia_X_fr_base()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_inertia_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_inertia_X_fr_base::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_imu_link::Type_fr_base_X_fr_imu_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0.1837;
    (*this)(0,5) = - 0.05755;
    (*this)(1,0) = 0;
    (*this)(1,1) = - 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.1837;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0.062;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = - 1.0;
    (*this)(2,3) = - 0.05755;
    (*this)(2,4) = - 0.062;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = - 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = - 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_imu_link& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_imu_link::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_imu_link_X_fr_base::Type_fr_imu_link_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0.1837;
    (*this)(0,5) = - 0.05755;
    (*this)(1,0) = 0;
    (*this)(1,1) = - 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.1837;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.062;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = - 1.0;
    (*this)(2,3) = - 0.05755;
    (*this)(2,4) = 0.062;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = - 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = - 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_imu_link_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_imu_link_X_fr_base::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA::Type_fr_base_X_fr_LF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.116;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.277;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0.277;
    (*this)(2,5) = - 0.116;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE::Type_fr_base_X_fr_LF_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,4) = ((- 0.116 *  c_q_LF_HAA_) -  0.041);
    (*this)(0,5) = ( 0.116 *  s_q_LF_HAA_);
    (*this)(1,1) =  s_q_LF_HAA_;
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = ( 0.041 *  s_q_LF_HAA_);
    (*this)(1,4) = ( 0.3405 *  c_q_LF_HAA_);
    (*this)(1,5) = (- 0.3405 *  s_q_LF_HAA_);
    (*this)(2,1) = - c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = ((- 0.041 *  c_q_LF_HAA_) -  0.116);
    (*this)(2,4) = ( 0.3405 *  s_q_LF_HAA_);
    (*this)(2,5) = ( 0.3405 *  c_q_LF_HAA_);
    (*this)(4,4) =  s_q_LF_HAA_;
    (*this)(4,5) =  c_q_LF_HAA_;
    (*this)(5,4) = - c_q_LF_HAA_;
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE::Type_fr_base_X_fr_LF_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) =  c_q_LF_HFE_;
    (*this)(0,1) = - s_q_LF_HFE_;
    (*this)(0,3) = (((- 0.116 *  c_q_LF_HAA_) -  0.15) *  s_q_LF_HFE_);
    (*this)(0,4) = (((- 0.116 *  c_q_LF_HAA_) -  0.15) *  c_q_LF_HFE_);
    (*this)(0,5) = (( 0.25 *  c_q_LF_HFE_) + ( 0.116 *  s_q_LF_HAA_));
    (*this)(1,0) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(1,1) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = (((( 0.3405 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.15 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.25 *  c_q_LF_HAA_));
    (*this)(1,4) = ((( 0.3405 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.15 *  s_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(1,5) = ((( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - ( 0.3405 *  s_q_LF_HAA_));
    (*this)(2,0) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(2,1) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = (((( 0.3405 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((- 0.116 - ( 0.15 *  c_q_LF_HAA_)) *  c_q_LF_HFE_)) - ( 0.25 *  s_q_LF_HAA_));
    (*this)(2,4) = ((( 0.116 + ( 0.15 *  c_q_LF_HAA_)) *  s_q_LF_HFE_) + (( 0.3405 *  s_q_LF_HAA_) *  c_q_LF_HFE_));
    (*this)(2,5) = (( 0.3405 *  c_q_LF_HAA_) - (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(3,3) =  c_q_LF_HFE_;
    (*this)(3,4) = - s_q_LF_HFE_;
    (*this)(4,3) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(4,4) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(4,5) =  c_q_LF_HAA_;
    (*this)(5,3) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(5,4) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA::Type_fr_base_X_fr_RF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.116;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.277;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0.277;
    (*this)(2,5) = 0.116;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE::Type_fr_base_X_fr_RF_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,4) = (( 0.116 *  c_q_RF_HAA_) +  0.041);
    (*this)(0,5) = (- 0.116 *  s_q_RF_HAA_);
    (*this)(1,1) =  s_q_RF_HAA_;
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = (- 0.041 *  s_q_RF_HAA_);
    (*this)(1,4) = ( 0.3405 *  c_q_RF_HAA_);
    (*this)(1,5) = (- 0.3405 *  s_q_RF_HAA_);
    (*this)(2,1) = - c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (( 0.041 *  c_q_RF_HAA_) +  0.116);
    (*this)(2,4) = ( 0.3405 *  s_q_RF_HAA_);
    (*this)(2,5) = ( 0.3405 *  c_q_RF_HAA_);
    (*this)(4,4) =  s_q_RF_HAA_;
    (*this)(4,5) =  c_q_RF_HAA_;
    (*this)(5,4) = - c_q_RF_HAA_;
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE::Type_fr_base_X_fr_RF_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) =  c_q_RF_HFE_;
    (*this)(0,1) = - s_q_RF_HFE_;
    (*this)(0,3) = ((( 0.116 *  c_q_RF_HAA_) +  0.15) *  s_q_RF_HFE_);
    (*this)(0,4) = ((( 0.116 *  c_q_RF_HAA_) +  0.15) *  c_q_RF_HFE_);
    (*this)(0,5) = (( 0.25 *  c_q_RF_HFE_) - ( 0.116 *  s_q_RF_HAA_));
    (*this)(1,0) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(1,1) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = (((( 0.3405 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.15 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.25 *  c_q_RF_HAA_));
    (*this)(1,4) = ((( 0.15 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.3405 *  c_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(1,5) = ((( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3405 *  s_q_RF_HAA_));
    (*this)(2,0) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(2,1) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (((( 0.3405 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.116 + ( 0.15 *  c_q_RF_HAA_)) *  c_q_RF_HFE_)) - ( 0.25 *  s_q_RF_HAA_));
    (*this)(2,4) = (((- 0.116 - ( 0.15 *  c_q_RF_HAA_)) *  s_q_RF_HFE_) + (( 0.3405 *  s_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(2,5) = (( 0.3405 *  c_q_RF_HAA_) - (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(3,3) =  c_q_RF_HFE_;
    (*this)(3,4) = - s_q_RF_HFE_;
    (*this)(4,3) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(4,4) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(4,5) =  c_q_RF_HAA_;
    (*this)(5,3) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(5,4) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA::Type_fr_base_X_fr_LH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.116;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.277;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.277;
    (*this)(2,5) = - 0.116;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE::Type_fr_base_X_fr_LH_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,4) = ((- 0.116 *  c_q_LH_HAA_) -  0.041);
    (*this)(0,5) = ( 0.116 *  s_q_LH_HAA_);
    (*this)(1,1) =  s_q_LH_HAA_;
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = ( 0.041 *  s_q_LH_HAA_);
    (*this)(1,4) = (- 0.3405 *  c_q_LH_HAA_);
    (*this)(1,5) = ( 0.3405 *  s_q_LH_HAA_);
    (*this)(2,1) = - c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ((- 0.041 *  c_q_LH_HAA_) -  0.116);
    (*this)(2,4) = (- 0.3405 *  s_q_LH_HAA_);
    (*this)(2,5) = (- 0.3405 *  c_q_LH_HAA_);
    (*this)(4,4) =  s_q_LH_HAA_;
    (*this)(4,5) =  c_q_LH_HAA_;
    (*this)(5,4) = - c_q_LH_HAA_;
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE::Type_fr_base_X_fr_LH_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) =  c_q_LH_HFE_;
    (*this)(0,1) = - s_q_LH_HFE_;
    (*this)(0,3) = (((- 0.116 *  c_q_LH_HAA_) -  0.15) *  s_q_LH_HFE_);
    (*this)(0,4) = (((- 0.116 *  c_q_LH_HAA_) -  0.15) *  c_q_LH_HFE_);
    (*this)(0,5) = (( 0.25 *  c_q_LH_HFE_) + ( 0.116 *  s_q_LH_HAA_));
    (*this)(1,0) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(1,1) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = ((((- 0.3405 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.15 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.25 *  c_q_LH_HAA_));
    (*this)(1,4) = (((- 0.15 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.3405 *  c_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(1,5) = ((( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ( 0.3405 *  s_q_LH_HAA_));
    (*this)(2,0) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(2,1) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ((((- 0.3405 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((- 0.116 - ( 0.15 *  c_q_LH_HAA_)) *  c_q_LH_HFE_)) - ( 0.25 *  s_q_LH_HAA_));
    (*this)(2,4) = ((( 0.116 + ( 0.15 *  c_q_LH_HAA_)) *  s_q_LH_HFE_) - (( 0.3405 *  s_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(2,5) = (((- 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - ( 0.3405 *  c_q_LH_HAA_));
    (*this)(3,3) =  c_q_LH_HFE_;
    (*this)(3,4) = - s_q_LH_HFE_;
    (*this)(4,3) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(4,4) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(4,5) =  c_q_LH_HAA_;
    (*this)(5,3) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(5,4) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA::Type_fr_base_X_fr_RH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.116;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.277;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.277;
    (*this)(2,5) = 0.116;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE::Type_fr_base_X_fr_RH_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,4) = (( 0.116 *  c_q_RH_HAA_) +  0.041);
    (*this)(0,5) = (- 0.116 *  s_q_RH_HAA_);
    (*this)(1,1) =  s_q_RH_HAA_;
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = (- 0.041 *  s_q_RH_HAA_);
    (*this)(1,4) = (- 0.3405 *  c_q_RH_HAA_);
    (*this)(1,5) = ( 0.3405 *  s_q_RH_HAA_);
    (*this)(2,1) = - c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (( 0.041 *  c_q_RH_HAA_) +  0.116);
    (*this)(2,4) = (- 0.3405 *  s_q_RH_HAA_);
    (*this)(2,5) = (- 0.3405 *  c_q_RH_HAA_);
    (*this)(4,4) =  s_q_RH_HAA_;
    (*this)(4,5) =  c_q_RH_HAA_;
    (*this)(5,4) = - c_q_RH_HAA_;
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE::Type_fr_base_X_fr_RH_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) =  c_q_RH_HFE_;
    (*this)(0,1) = - s_q_RH_HFE_;
    (*this)(0,3) = ((( 0.116 *  c_q_RH_HAA_) +  0.15) *  s_q_RH_HFE_);
    (*this)(0,4) = ((( 0.116 *  c_q_RH_HAA_) +  0.15) *  c_q_RH_HFE_);
    (*this)(0,5) = (( 0.25 *  c_q_RH_HFE_) - ( 0.116 *  s_q_RH_HAA_));
    (*this)(1,0) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(1,1) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = ((((- 0.3405 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.15 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.25 *  c_q_RH_HAA_));
    (*this)(1,4) = ((( 0.15 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3405 *  c_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(1,5) = ((( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ( 0.3405 *  s_q_RH_HAA_));
    (*this)(2,0) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(2,1) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = ((((- 0.3405 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.116 + ( 0.15 *  c_q_RH_HAA_)) *  c_q_RH_HFE_)) - ( 0.25 *  s_q_RH_HAA_));
    (*this)(2,4) = (((- 0.116 - ( 0.15 *  c_q_RH_HAA_)) *  s_q_RH_HFE_) - (( 0.3405 *  s_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(2,5) = (((- 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - ( 0.3405 *  c_q_RH_HAA_));
    (*this)(3,3) =  c_q_RH_HFE_;
    (*this)(3,4) = - s_q_RH_HFE_;
    (*this)(4,3) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(4,4) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(4,5) =  c_q_RH_HAA_;
    (*this)(5,3) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(5,4) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP::Type_fr_LF_THIGH_X_fr_LF_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.0635;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  s_q_LF_HFE_;
    (*this)(0,2) =  c_q_LF_HFE_;
    (*this)(0,3) = ( 0.041 *  c_q_LF_HFE_);
    (*this)(0,4) = ( 0.0635 *  s_q_LF_HFE_);
    (*this)(0,5) = (- 0.041 *  s_q_LF_HFE_);
    (*this)(1,0) =  c_q_LF_HFE_;
    (*this)(1,2) = - s_q_LF_HFE_;
    (*this)(1,3) = (- 0.041 *  s_q_LF_HFE_);
    (*this)(1,4) = ( 0.0635 *  c_q_LF_HFE_);
    (*this)(1,5) = (- 0.041 *  c_q_LF_HFE_);
    (*this)(3,3) =  s_q_LF_HFE_;
    (*this)(3,5) =  c_q_LF_HFE_;
    (*this)(4,3) =  c_q_LF_HFE_;
    (*this)(4,5) = - s_q_LF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH::Type_fr_LF_HIP_X_fr_LF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = - 0.0635;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  s_q_LF_HFE_;
    (*this)(0,1) =  c_q_LF_HFE_;
    (*this)(0,3) = ( 0.041 *  c_q_LF_HFE_);
    (*this)(0,4) = (- 0.041 *  s_q_LF_HFE_);
    (*this)(1,3) = ( 0.0635 *  s_q_LF_HFE_);
    (*this)(1,4) = ( 0.0635 *  c_q_LF_HFE_);
    (*this)(2,0) =  c_q_LF_HFE_;
    (*this)(2,1) = - s_q_LF_HFE_;
    (*this)(2,3) = (- 0.041 *  s_q_LF_HFE_);
    (*this)(2,4) = (- 0.041 *  c_q_LF_HFE_);
    (*this)(3,3) =  s_q_LF_HFE_;
    (*this)(3,4) =  c_q_LF_HFE_;
    (*this)(5,3) =  c_q_LF_HFE_;
    (*this)(5,4) = - s_q_LF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH::Type_fr_LF_SHANK_X_fr_LF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.25;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar c_q_LF_KFE_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  c_q_LF_KFE_;
    (*this)(0,1) =  s_q_LF_KFE_;
    (*this)(0,3) = (- 0.109 *  s_q_LF_KFE_);
    (*this)(0,4) = ( 0.109 *  c_q_LF_KFE_);
    (*this)(0,5) = (- 0.25 *  c_q_LF_KFE_);
    (*this)(1,0) = - s_q_LF_KFE_;
    (*this)(1,1) =  c_q_LF_KFE_;
    (*this)(1,3) = (- 0.109 *  c_q_LF_KFE_);
    (*this)(1,4) = (- 0.109 *  s_q_LF_KFE_);
    (*this)(1,5) = ( 0.25 *  s_q_LF_KFE_);
    (*this)(3,3) =  c_q_LF_KFE_;
    (*this)(3,4) =  s_q_LF_KFE_;
    (*this)(4,3) = - s_q_LF_KFE_;
    (*this)(4,4) =  c_q_LF_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK::Type_fr_LF_THIGH_X_fr_LF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0.25;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar c_q_LF_KFE_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  c_q_LF_KFE_;
    (*this)(0,1) = - s_q_LF_KFE_;
    (*this)(0,3) = (- 0.109 *  s_q_LF_KFE_);
    (*this)(0,4) = (- 0.109 *  c_q_LF_KFE_);
    (*this)(1,0) =  s_q_LF_KFE_;
    (*this)(1,1) =  c_q_LF_KFE_;
    (*this)(1,3) = ( 0.109 *  c_q_LF_KFE_);
    (*this)(1,4) = (- 0.109 *  s_q_LF_KFE_);
    (*this)(2,3) = (- 0.25 *  c_q_LF_KFE_);
    (*this)(2,4) = ( 0.25 *  s_q_LF_KFE_);
    (*this)(3,3) =  c_q_LF_KFE_;
    (*this)(3,4) = - s_q_LF_KFE_;
    (*this)(4,3) =  s_q_LF_KFE_;
    (*this)(4,4) =  c_q_LF_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP::Type_fr_RF_THIGH_X_fr_RF_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.0635;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  s_q_RF_HFE_;
    (*this)(0,2) =  c_q_RF_HFE_;
    (*this)(0,3) = (- 0.041 *  c_q_RF_HFE_);
    (*this)(0,4) = ( 0.0635 *  s_q_RF_HFE_);
    (*this)(0,5) = ( 0.041 *  s_q_RF_HFE_);
    (*this)(1,0) =  c_q_RF_HFE_;
    (*this)(1,2) = - s_q_RF_HFE_;
    (*this)(1,3) = ( 0.041 *  s_q_RF_HFE_);
    (*this)(1,4) = ( 0.0635 *  c_q_RF_HFE_);
    (*this)(1,5) = ( 0.041 *  c_q_RF_HFE_);
    (*this)(3,3) =  s_q_RF_HFE_;
    (*this)(3,5) =  c_q_RF_HFE_;
    (*this)(4,3) =  c_q_RF_HFE_;
    (*this)(4,5) = - s_q_RF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH::Type_fr_RF_HIP_X_fr_RF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = - 0.0635;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  s_q_RF_HFE_;
    (*this)(0,1) =  c_q_RF_HFE_;
    (*this)(0,3) = (- 0.041 *  c_q_RF_HFE_);
    (*this)(0,4) = ( 0.041 *  s_q_RF_HFE_);
    (*this)(1,3) = ( 0.0635 *  s_q_RF_HFE_);
    (*this)(1,4) = ( 0.0635 *  c_q_RF_HFE_);
    (*this)(2,0) =  c_q_RF_HFE_;
    (*this)(2,1) = - s_q_RF_HFE_;
    (*this)(2,3) = ( 0.041 *  s_q_RF_HFE_);
    (*this)(2,4) = ( 0.041 *  c_q_RF_HFE_);
    (*this)(3,3) =  s_q_RF_HFE_;
    (*this)(3,4) =  c_q_RF_HFE_;
    (*this)(5,3) =  c_q_RF_HFE_;
    (*this)(5,4) = - s_q_RF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH::Type_fr_RF_SHANK_X_fr_RF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.25;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar c_q_RF_KFE_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  c_q_RF_KFE_;
    (*this)(0,1) =  s_q_RF_KFE_;
    (*this)(0,3) = ( 0.109 *  s_q_RF_KFE_);
    (*this)(0,4) = (- 0.109 *  c_q_RF_KFE_);
    (*this)(0,5) = (- 0.25 *  c_q_RF_KFE_);
    (*this)(1,0) = - s_q_RF_KFE_;
    (*this)(1,1) =  c_q_RF_KFE_;
    (*this)(1,3) = ( 0.109 *  c_q_RF_KFE_);
    (*this)(1,4) = ( 0.109 *  s_q_RF_KFE_);
    (*this)(1,5) = ( 0.25 *  s_q_RF_KFE_);
    (*this)(3,3) =  c_q_RF_KFE_;
    (*this)(3,4) =  s_q_RF_KFE_;
    (*this)(4,3) = - s_q_RF_KFE_;
    (*this)(4,4) =  c_q_RF_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK::Type_fr_RF_THIGH_X_fr_RF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0.25;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar c_q_RF_KFE_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  c_q_RF_KFE_;
    (*this)(0,1) = - s_q_RF_KFE_;
    (*this)(0,3) = ( 0.109 *  s_q_RF_KFE_);
    (*this)(0,4) = ( 0.109 *  c_q_RF_KFE_);
    (*this)(1,0) =  s_q_RF_KFE_;
    (*this)(1,1) =  c_q_RF_KFE_;
    (*this)(1,3) = (- 0.109 *  c_q_RF_KFE_);
    (*this)(1,4) = ( 0.109 *  s_q_RF_KFE_);
    (*this)(2,3) = (- 0.25 *  c_q_RF_KFE_);
    (*this)(2,4) = ( 0.25 *  s_q_RF_KFE_);
    (*this)(3,3) =  c_q_RF_KFE_;
    (*this)(3,4) = - s_q_RF_KFE_;
    (*this)(4,3) =  s_q_RF_KFE_;
    (*this)(4,4) =  c_q_RF_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP::Type_fr_LH_THIGH_X_fr_LH_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.0635;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  s_q_LH_HFE_;
    (*this)(0,2) =  c_q_LH_HFE_;
    (*this)(0,3) = ( 0.041 *  c_q_LH_HFE_);
    (*this)(0,4) = (- 0.0635 *  s_q_LH_HFE_);
    (*this)(0,5) = (- 0.041 *  s_q_LH_HFE_);
    (*this)(1,0) =  c_q_LH_HFE_;
    (*this)(1,2) = - s_q_LH_HFE_;
    (*this)(1,3) = (- 0.041 *  s_q_LH_HFE_);
    (*this)(1,4) = (- 0.0635 *  c_q_LH_HFE_);
    (*this)(1,5) = (- 0.041 *  c_q_LH_HFE_);
    (*this)(3,3) =  s_q_LH_HFE_;
    (*this)(3,5) =  c_q_LH_HFE_;
    (*this)(4,3) =  c_q_LH_HFE_;
    (*this)(4,5) = - s_q_LH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH::Type_fr_LH_HIP_X_fr_LH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0.0635;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  s_q_LH_HFE_;
    (*this)(0,1) =  c_q_LH_HFE_;
    (*this)(0,3) = ( 0.041 *  c_q_LH_HFE_);
    (*this)(0,4) = (- 0.041 *  s_q_LH_HFE_);
    (*this)(1,3) = (- 0.0635 *  s_q_LH_HFE_);
    (*this)(1,4) = (- 0.0635 *  c_q_LH_HFE_);
    (*this)(2,0) =  c_q_LH_HFE_;
    (*this)(2,1) = - s_q_LH_HFE_;
    (*this)(2,3) = (- 0.041 *  s_q_LH_HFE_);
    (*this)(2,4) = (- 0.041 *  c_q_LH_HFE_);
    (*this)(3,3) =  s_q_LH_HFE_;
    (*this)(3,4) =  c_q_LH_HFE_;
    (*this)(5,3) =  c_q_LH_HFE_;
    (*this)(5,4) = - s_q_LH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH::Type_fr_LH_SHANK_X_fr_LH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.25;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar c_q_LH_KFE_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  c_q_LH_KFE_;
    (*this)(0,1) =  s_q_LH_KFE_;
    (*this)(0,3) = (- 0.109 *  s_q_LH_KFE_);
    (*this)(0,4) = ( 0.109 *  c_q_LH_KFE_);
    (*this)(0,5) = (- 0.25 *  c_q_LH_KFE_);
    (*this)(1,0) = - s_q_LH_KFE_;
    (*this)(1,1) =  c_q_LH_KFE_;
    (*this)(1,3) = (- 0.109 *  c_q_LH_KFE_);
    (*this)(1,4) = (- 0.109 *  s_q_LH_KFE_);
    (*this)(1,5) = ( 0.25 *  s_q_LH_KFE_);
    (*this)(3,3) =  c_q_LH_KFE_;
    (*this)(3,4) =  s_q_LH_KFE_;
    (*this)(4,3) = - s_q_LH_KFE_;
    (*this)(4,4) =  c_q_LH_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK::Type_fr_LH_THIGH_X_fr_LH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0.25;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar c_q_LH_KFE_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  c_q_LH_KFE_;
    (*this)(0,1) = - s_q_LH_KFE_;
    (*this)(0,3) = (- 0.109 *  s_q_LH_KFE_);
    (*this)(0,4) = (- 0.109 *  c_q_LH_KFE_);
    (*this)(1,0) =  s_q_LH_KFE_;
    (*this)(1,1) =  c_q_LH_KFE_;
    (*this)(1,3) = ( 0.109 *  c_q_LH_KFE_);
    (*this)(1,4) = (- 0.109 *  s_q_LH_KFE_);
    (*this)(2,3) = (- 0.25 *  c_q_LH_KFE_);
    (*this)(2,4) = ( 0.25 *  s_q_LH_KFE_);
    (*this)(3,3) =  c_q_LH_KFE_;
    (*this)(3,4) = - s_q_LH_KFE_;
    (*this)(4,3) =  s_q_LH_KFE_;
    (*this)(4,4) =  c_q_LH_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP::Type_fr_RH_THIGH_X_fr_RH_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.0635;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  s_q_RH_HFE_;
    (*this)(0,2) =  c_q_RH_HFE_;
    (*this)(0,3) = (- 0.041 *  c_q_RH_HFE_);
    (*this)(0,4) = (- 0.0635 *  s_q_RH_HFE_);
    (*this)(0,5) = ( 0.041 *  s_q_RH_HFE_);
    (*this)(1,0) =  c_q_RH_HFE_;
    (*this)(1,2) = - s_q_RH_HFE_;
    (*this)(1,3) = ( 0.041 *  s_q_RH_HFE_);
    (*this)(1,4) = (- 0.0635 *  c_q_RH_HFE_);
    (*this)(1,5) = ( 0.041 *  c_q_RH_HFE_);
    (*this)(3,3) =  s_q_RH_HFE_;
    (*this)(3,5) =  c_q_RH_HFE_;
    (*this)(4,3) =  c_q_RH_HFE_;
    (*this)(4,5) = - s_q_RH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH::Type_fr_RH_HIP_X_fr_RH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0.0635;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  s_q_RH_HFE_;
    (*this)(0,1) =  c_q_RH_HFE_;
    (*this)(0,3) = (- 0.041 *  c_q_RH_HFE_);
    (*this)(0,4) = ( 0.041 *  s_q_RH_HFE_);
    (*this)(1,3) = (- 0.0635 *  s_q_RH_HFE_);
    (*this)(1,4) = (- 0.0635 *  c_q_RH_HFE_);
    (*this)(2,0) =  c_q_RH_HFE_;
    (*this)(2,1) = - s_q_RH_HFE_;
    (*this)(2,3) = ( 0.041 *  s_q_RH_HFE_);
    (*this)(2,4) = ( 0.041 *  c_q_RH_HFE_);
    (*this)(3,3) =  s_q_RH_HFE_;
    (*this)(3,4) =  c_q_RH_HFE_;
    (*this)(5,3) =  c_q_RH_HFE_;
    (*this)(5,4) = - s_q_RH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH::Type_fr_RH_SHANK_X_fr_RH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.25;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar c_q_RH_KFE_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  c_q_RH_KFE_;
    (*this)(0,1) =  s_q_RH_KFE_;
    (*this)(0,3) = ( 0.109 *  s_q_RH_KFE_);
    (*this)(0,4) = (- 0.109 *  c_q_RH_KFE_);
    (*this)(0,5) = (- 0.25 *  c_q_RH_KFE_);
    (*this)(1,0) = - s_q_RH_KFE_;
    (*this)(1,1) =  c_q_RH_KFE_;
    (*this)(1,3) = ( 0.109 *  c_q_RH_KFE_);
    (*this)(1,4) = ( 0.109 *  s_q_RH_KFE_);
    (*this)(1,5) = ( 0.25 *  s_q_RH_KFE_);
    (*this)(3,3) =  c_q_RH_KFE_;
    (*this)(3,4) =  s_q_RH_KFE_;
    (*this)(4,3) = - s_q_RH_KFE_;
    (*this)(4,4) =  c_q_RH_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK::Type_fr_RH_THIGH_X_fr_RH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0.25;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar c_q_RH_KFE_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  c_q_RH_KFE_;
    (*this)(0,1) = - s_q_RH_KFE_;
    (*this)(0,3) = ( 0.109 *  s_q_RH_KFE_);
    (*this)(0,4) = ( 0.109 *  c_q_RH_KFE_);
    (*this)(1,0) =  s_q_RH_KFE_;
    (*this)(1,1) =  c_q_RH_KFE_;
    (*this)(1,3) = (- 0.109 *  c_q_RH_KFE_);
    (*this)(1,4) = ( 0.109 *  s_q_RH_KFE_);
    (*this)(2,3) = (- 0.25 *  c_q_RH_KFE_);
    (*this)(2,4) = ( 0.25 *  s_q_RH_KFE_);
    (*this)(3,3) =  c_q_RH_KFE_;
    (*this)(3,4) = - s_q_RH_KFE_;
    (*this)(4,3) =  s_q_RH_KFE_;
    (*this)(4,4) =  c_q_RH_KFE_;
    return *this;
}

template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP::Type_fr_base_X_fr_LF_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.277;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.116;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,0) =  s_q_LF_HAA_;
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(2,0) = - c_q_LF_HAA_;
    (*this)(2,1) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base::Type_fr_LF_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.277;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,1) =  s_q_LF_HAA_;
    (*this)(0,2) = - c_q_LF_HAA_;
    (*this)(0,3) = (- 0.116 *  s_q_LF_HAA_);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = (- 0.116 *  c_q_LF_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH::Type_fr_base_X_fr_LF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.3405;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) =  c_q_LF_HFE_;
    (*this)(0,1) = - s_q_LF_HFE_;
    (*this)(1,0) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(1,1) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = (( 0.041 *  c_q_LF_HAA_) +  0.116);
    (*this)(2,0) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(2,1) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = ( 0.041 *  s_q_LF_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_base::Type_fr_LF_THIGH_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) =  c_q_LF_HFE_;
    (*this)(0,1) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(0,2) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(0,3) = (((- 0.116 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - ( 0.3405 *  c_q_LF_HFE_));
    (*this)(1,0) = - s_q_LF_HFE_;
    (*this)(1,1) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,2) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,3) = (( 0.3405 *  s_q_LF_HFE_) - (( 0.116 *  s_q_LF_HAA_) *  c_q_LF_HFE_));
    (*this)(2,1) =  c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = ((- 0.116 *  c_q_LF_HAA_) -  0.041);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK::Type_fr_base_X_fr_LF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,1) = ((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(0,3) = ( 0.3405 - ( 0.25 *  s_q_LF_HFE_));
    (*this)(1,0) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = (((( 0.25 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ( 0.15 *  c_q_LF_HAA_)) +  0.116);
    (*this)(2,0) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) = ((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = (( 0.15 *  s_q_LF_HAA_) - (( 0.25 *  c_q_LF_HAA_) *  c_q_LF_HFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_base::Type_fr_LF_SHANK_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,2) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,3) = ((((- 0.25 - (( 0.116 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.3405 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((- 0.3405 *  c_q_LF_HFE_) - (( 0.116 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(1,0) = ((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(1,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(1,2) = ((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,3) = (((( 0.3405 *  c_q_LF_HFE_) + (( 0.116 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((- 0.25 - (( 0.116 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.3405 *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(2,1) =  c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = ((- 0.116 *  c_q_LF_HAA_) -  0.15);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP::Type_fr_base_X_fr_RF_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.277;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.116;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,0) =  s_q_RF_HAA_;
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(2,0) = - c_q_RF_HAA_;
    (*this)(2,1) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base::Type_fr_RF_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.277;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,1) =  s_q_RF_HAA_;
    (*this)(0,2) = - c_q_RF_HAA_;
    (*this)(0,3) = ( 0.116 *  s_q_RF_HAA_);
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = ( 0.116 *  c_q_RF_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH::Type_fr_base_X_fr_RF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.3405;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) =  c_q_RF_HFE_;
    (*this)(0,1) = - s_q_RF_HFE_;
    (*this)(1,0) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(1,1) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = ((- 0.041 *  c_q_RF_HAA_) -  0.116);
    (*this)(2,0) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(2,1) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (- 0.041 *  s_q_RF_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_base::Type_fr_RF_THIGH_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) =  c_q_RF_HFE_;
    (*this)(0,1) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(0,2) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(0,3) = ((( 0.116 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3405 *  c_q_RF_HFE_));
    (*this)(1,0) = - s_q_RF_HFE_;
    (*this)(1,1) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,2) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,3) = (( 0.3405 *  s_q_RF_HFE_) + (( 0.116 *  s_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(2,1) =  c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (( 0.116 *  c_q_RF_HAA_) +  0.041);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK::Type_fr_base_X_fr_RF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,1) = ((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(0,3) = ( 0.3405 - ( 0.25 *  s_q_RF_HFE_));
    (*this)(1,0) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = (((( 0.25 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - ( 0.15 *  c_q_RF_HAA_)) -  0.116);
    (*this)(2,0) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) = ((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (((- 0.25 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - ( 0.15 *  s_q_RF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_base::Type_fr_RF_SHANK_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,2) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,3) = ((((- 0.25 + (( 0.116 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.3405 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.116 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3405 *  c_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(1,0) = ((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(1,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(1,2) = ((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,3) = (((( 0.3405 *  c_q_RF_HFE_) - (( 0.116 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((- 0.25 + (( 0.116 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.3405 *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(2,1) =  c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (( 0.116 *  c_q_RF_HAA_) +  0.15);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP::Type_fr_base_X_fr_LH_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.277;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.116;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,0) =  s_q_LH_HAA_;
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(2,0) = - c_q_LH_HAA_;
    (*this)(2,1) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base::Type_fr_LH_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.277;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,1) =  s_q_LH_HAA_;
    (*this)(0,2) = - c_q_LH_HAA_;
    (*this)(0,3) = (- 0.116 *  s_q_LH_HAA_);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = (- 0.116 *  c_q_LH_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH::Type_fr_base_X_fr_LH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.3405;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) =  c_q_LH_HFE_;
    (*this)(0,1) = - s_q_LH_HFE_;
    (*this)(1,0) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(1,1) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = (( 0.041 *  c_q_LH_HAA_) +  0.116);
    (*this)(2,0) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(2,1) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ( 0.041 *  s_q_LH_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_base::Type_fr_LH_THIGH_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) =  c_q_LH_HFE_;
    (*this)(0,1) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(0,2) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(0,3) = (( 0.3405 *  c_q_LH_HFE_) - (( 0.116 *  s_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(1,0) = - s_q_LH_HFE_;
    (*this)(1,1) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,2) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,3) = ((- 0.3405 *  s_q_LH_HFE_) - (( 0.116 *  s_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(2,1) =  c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ((- 0.116 *  c_q_LH_HAA_) -  0.041);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK::Type_fr_base_X_fr_LH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,1) = ((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(0,3) = ((- 0.25 *  s_q_LH_HFE_) -  0.3405);
    (*this)(1,0) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = (((( 0.25 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + ( 0.15 *  c_q_LH_HAA_)) +  0.116);
    (*this)(2,0) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) = ((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = (( 0.15 *  s_q_LH_HAA_) - (( 0.25 *  c_q_LH_HAA_) *  c_q_LH_HFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_base::Type_fr_LH_SHANK_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,2) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,3) = ((((- 0.25 - (( 0.116 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.3405 *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.3405 *  c_q_LH_HFE_) - (( 0.116 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(1,0) = ((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(1,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(1,2) = ((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,3) = ((((( 0.116 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - ( 0.3405 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((- 0.25 - (( 0.116 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.3405 *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(2,1) =  c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ((- 0.116 *  c_q_LH_HAA_) -  0.15);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP::Type_fr_base_X_fr_RH_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.277;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.116;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,0) =  s_q_RH_HAA_;
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(2,0) = - c_q_RH_HAA_;
    (*this)(2,1) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base::Type_fr_RH_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.277;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,1) =  s_q_RH_HAA_;
    (*this)(0,2) = - c_q_RH_HAA_;
    (*this)(0,3) = ( 0.116 *  s_q_RH_HAA_);
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = ( 0.116 *  c_q_RH_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH::Type_fr_base_X_fr_RH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.3405;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) =  c_q_RH_HFE_;
    (*this)(0,1) = - s_q_RH_HFE_;
    (*this)(1,0) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(1,1) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = ((- 0.041 *  c_q_RH_HAA_) -  0.116);
    (*this)(2,0) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(2,1) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (- 0.041 *  s_q_RH_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_base::Type_fr_RH_THIGH_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) =  c_q_RH_HFE_;
    (*this)(0,1) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(0,2) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(0,3) = ((( 0.116 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ( 0.3405 *  c_q_RH_HFE_));
    (*this)(1,0) = - s_q_RH_HFE_;
    (*this)(1,1) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,2) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,3) = ((( 0.116 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - ( 0.3405 *  s_q_RH_HFE_));
    (*this)(2,1) =  c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (( 0.116 *  c_q_RH_HAA_) +  0.041);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK::Type_fr_base_X_fr_RH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,1) = ((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(0,3) = ((- 0.25 *  s_q_RH_HFE_) -  0.3405);
    (*this)(1,0) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = (((( 0.25 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - ( 0.15 *  c_q_RH_HAA_)) -  0.116);
    (*this)(2,0) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) = ((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (((- 0.25 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - ( 0.15 *  s_q_RH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_base::Type_fr_RH_SHANK_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,2) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,3) = ((((- 0.25 + (( 0.116 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.3405 *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.3405 *  c_q_RH_HFE_) + (( 0.116 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(1,0) = ((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(1,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(1,2) = ((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,3) = ((((- 0.3405 *  c_q_RH_HFE_) - (( 0.116 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((- 0.25 + (( 0.116 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.3405 *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(2,1) =  c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (( 0.116 *  c_q_RH_HAA_) +  0.15);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_ADAPTER::Type_fr_base_X_fr_LF_ADAPTER()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_ADAPTER& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_ADAPTER::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,2) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(0,3) = (((((- 0.1 *  s_q_LF_HFE_) *  s_q_LF_KFE_) + (( 0.1 *  c_q_LF_HFE_) *  c_q_LF_KFE_)) - ( 0.25 *  s_q_LF_HFE_)) +  0.3405);
    (*this)(1,0) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,3) = ((((((( 0.1 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.1 *  s_q_LF_HAA_) *  s_q_LF_HFE_) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.13 *  c_q_LF_HAA_)) +  0.116);
    (*this)(2,0) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(2,3) = ((((((- 0.1 *  c_q_LF_HAA_) *  c_q_LF_HFE_) *  s_q_LF_KFE_) - ((( 0.1 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  c_q_LF_KFE_)) - (( 0.25 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.13 *  s_q_LF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_ADAPTER_X_fr_base::Type_fr_LF_ADAPTER_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_ADAPTER_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_ADAPTER_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,2) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,3) = (((((- 0.25 - (( 0.116 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.3405 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((- 0.3405 *  c_q_LF_HFE_) - (( 0.116 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) -  0.1);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = ((- 0.116 *  c_q_LF_HAA_) -  0.13);
    (*this)(2,0) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(2,1) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(2,3) = ((((- 0.3405 *  c_q_LF_HFE_) - (( 0.116 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.25 + (( 0.116 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT::Type_fr_base_X_fr_LF_FOOT()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,2) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(0,3) = ((((((- 0.32125 *  c_q_LF_HFE_) - ( 0.1 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.1 *  c_q_LF_HFE_) - ( 0.32125 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.25 *  s_q_LF_HFE_)) +  0.3405);
    (*this)(1,0) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,3) = (((((((( 0.1 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.32125 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.32125 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.1 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.13 *  c_q_LF_HAA_)) +  0.116);
    (*this)(2,0) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(2,3) = ((((((( 0.32125 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.1 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.32125 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.1 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.25 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.13 *  s_q_LF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base::Type_fr_LF_FOOT_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,2) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,3) = (((((- 0.25 - (( 0.116 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.3405 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((- 0.3405 *  c_q_LF_HFE_) - (( 0.116 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) -  0.1);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = ((- 0.116 *  c_q_LF_HAA_) -  0.13);
    (*this)(2,0) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(2,1) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(2,3) = (((((- 0.3405 *  c_q_LF_HFE_) - (( 0.116 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.25 + (( 0.116 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) +  0.32125);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP_COM::Type_fr_base_X_fr_LF_HIP_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.34152;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP_COM& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP_COM::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = - s_q_LF_HAA_;
    (*this)(1,3) = ((( 1.5E-4 *  s_q_LF_HAA_) - ( 0.00379 *  c_q_LF_HAA_)) +  0.116);
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) =  c_q_LF_HAA_;
    (*this)(2,3) = ((- 0.00379 *  s_q_LF_HAA_) - ( 1.5E-4 *  c_q_LF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_HIP_COM_X_fr_base::Type_fr_LF_HIP_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.34152;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_HIP_COM_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_HIP_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = ( 0.00379 - ( 0.116 *  c_q_LF_HAA_));
    (*this)(2,1) = - s_q_LF_HAA_;
    (*this)(2,2) =  c_q_LF_HAA_;
    (*this)(2,3) = (( 0.116 *  s_q_LF_HAA_) +  1.5E-4);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK_COM::Type_fr_base_X_fr_LF_SHANK_COM()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK_COM& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_SHANK_COM::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,2) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(0,3) = ((((((- 0.09806 *  c_q_LF_HFE_) - ( 0.05873 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.05873 *  c_q_LF_HFE_) - ( 0.09806 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.25 *  s_q_LF_HFE_)) +  0.3405);
    (*this)(1,0) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,3) = (((((((( 0.05873 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.09806 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.09806 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.05873 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.13918 *  c_q_LF_HAA_)) +  0.116);
    (*this)(2,0) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(2,3) = ((((((( 0.09806 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.05873 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.09806 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.05873 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.25 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.13918 *  s_q_LF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_SHANK_COM_X_fr_base::Type_fr_LF_SHANK_COM_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_SHANK_COM_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_SHANK_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(0,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,2) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,3) = (((((- 0.25 - (( 0.116 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.3405 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((- 0.3405 *  c_q_LF_HFE_) - (( 0.116 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) -  0.05873);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = ((- 0.116 *  c_q_LF_HAA_) -  0.13918);
    (*this)(2,0) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(2,1) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(2,3) = (((((- 0.3405 *  c_q_LF_HFE_) - (( 0.116 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.25 + (( 0.116 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.3405 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) +  0.09806);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH_COM::Type_fr_base_X_fr_LF_THIGH_COM()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH_COM& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_THIGH_COM::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) =  c_q_LF_HFE_;
    (*this)(0,2) =  s_q_LF_HFE_;
    (*this)(0,3) = (((- 0.21458 *  s_q_LF_HFE_) - ( 0.0039 *  c_q_LF_HFE_)) +  0.3405);
    (*this)(1,0) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = (- s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,3) = (((((- 0.0039 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.21458 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.09523 *  c_q_LF_HAA_)) +  0.116);
    (*this)(2,0) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) = ( c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,3) = (((( 0.0039 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.21458 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.09523 *  s_q_LF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_COM_X_fr_base::Type_fr_LF_THIGH_COM_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_COM_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) =  c_q_LF_HFE_;
    (*this)(0,1) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(0,2) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(0,3) = ((((- 0.116 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - ( 0.3405 *  c_q_LF_HFE_)) +  0.0039);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = ((- 0.116 *  c_q_LF_HAA_) -  0.09523);
    (*this)(2,0) =  s_q_LF_HFE_;
    (*this)(2,1) = (- s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,2) = ( c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,3) = (((- 0.3405 *  s_q_LF_HFE_) + (( 0.116 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) +  0.21458);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_ADAPTER::Type_fr_base_X_fr_LH_ADAPTER()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_ADAPTER& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_ADAPTER::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,2) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(0,3) = ((((( 0.1 *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( 0.1 *  c_q_LH_HFE_) *  c_q_LH_KFE_)) - ( 0.25 *  s_q_LH_HFE_)) -  0.3405);
    (*this)(1,0) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,3) = (((((((- 0.1 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  s_q_LH_KFE_) - ((( 0.1 *  s_q_LH_HAA_) *  s_q_LH_HFE_) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.13 *  c_q_LH_HAA_)) +  0.116);
    (*this)(2,0) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(2,3) = (((((( 0.1 *  c_q_LH_HAA_) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.1 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  c_q_LH_KFE_)) - (( 0.25 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.13 *  s_q_LH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_ADAPTER_X_fr_base::Type_fr_LH_ADAPTER_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_ADAPTER_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_ADAPTER_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,2) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,3) = (((((- 0.25 - (( 0.116 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.3405 *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.3405 *  c_q_LH_HFE_) - (( 0.116 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) +  0.1);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = ((- 0.116 *  c_q_LH_HAA_) -  0.13);
    (*this)(2,0) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(2,1) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(2,3) = (((( 0.3405 *  c_q_LH_HFE_) - (( 0.116 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.25 + (( 0.116 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT::Type_fr_base_X_fr_LH_FOOT()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,2) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(0,3) = (((((( 0.1 *  s_q_LH_HFE_) - ( 0.32125 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((- 0.1 *  c_q_LH_HFE_) - ( 0.32125 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.25 *  s_q_LH_HFE_)) -  0.3405);
    (*this)(1,0) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,3) = ((((((((- 0.1 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.32125 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.32125 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.1 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.13 *  c_q_LH_HAA_)) +  0.116);
    (*this)(2,0) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(2,3) = ((((((( 0.1 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.32125 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.1 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.32125 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.25 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.13 *  s_q_LH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base::Type_fr_LH_FOOT_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,2) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,3) = (((((- 0.25 - (( 0.116 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.3405 *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.3405 *  c_q_LH_HFE_) - (( 0.116 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) +  0.1);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = ((- 0.116 *  c_q_LH_HAA_) -  0.13);
    (*this)(2,0) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(2,1) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(2,3) = ((((( 0.3405 *  c_q_LH_HFE_) - (( 0.116 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.25 + (( 0.116 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) +  0.32125);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP_COM::Type_fr_base_X_fr_LH_HIP_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.34152;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP_COM& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP_COM::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = - s_q_LH_HAA_;
    (*this)(1,3) = ((( 1.5E-4 *  s_q_LH_HAA_) - ( 0.00379 *  c_q_LH_HAA_)) +  0.116);
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) =  c_q_LH_HAA_;
    (*this)(2,3) = ((- 0.00379 *  s_q_LH_HAA_) - ( 1.5E-4 *  c_q_LH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_HIP_COM_X_fr_base::Type_fr_LH_HIP_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.34152;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_HIP_COM_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_HIP_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = ( 0.00379 - ( 0.116 *  c_q_LH_HAA_));
    (*this)(2,1) = - s_q_LH_HAA_;
    (*this)(2,2) =  c_q_LH_HAA_;
    (*this)(2,3) = (( 0.116 *  s_q_LH_HAA_) +  1.5E-4);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK_COM::Type_fr_base_X_fr_LH_SHANK_COM()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK_COM& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_SHANK_COM::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,2) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(0,3) = (((((( 0.05873 *  s_q_LH_HFE_) - ( 0.09806 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((- 0.05873 *  c_q_LH_HFE_) - ( 0.09806 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.25 *  s_q_LH_HFE_)) -  0.3405);
    (*this)(1,0) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,3) = ((((((((- 0.05873 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.09806 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.09806 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.05873 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.13918 *  c_q_LH_HAA_)) +  0.116);
    (*this)(2,0) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(2,3) = ((((((( 0.05873 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.09806 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.05873 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.09806 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.25 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.13918 *  s_q_LH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_SHANK_COM_X_fr_base::Type_fr_LH_SHANK_COM_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_SHANK_COM_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_SHANK_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(0,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,2) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,3) = (((((- 0.25 - (( 0.116 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.3405 *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.3405 *  c_q_LH_HFE_) - (( 0.116 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) +  0.05873);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = ((- 0.116 *  c_q_LH_HAA_) -  0.13918);
    (*this)(2,0) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(2,1) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(2,3) = ((((( 0.3405 *  c_q_LH_HFE_) - (( 0.116 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.25 + (( 0.116 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.3405 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) +  0.09806);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH_COM::Type_fr_base_X_fr_LH_THIGH_COM()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH_COM& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_THIGH_COM::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) =  c_q_LH_HFE_;
    (*this)(0,2) =  s_q_LH_HFE_;
    (*this)(0,3) = (((- 0.21458 *  s_q_LH_HFE_) + ( 0.0039 *  c_q_LH_HFE_)) -  0.3405);
    (*this)(1,0) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = (- s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,3) = ((((( 0.0039 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.21458 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.09523 *  c_q_LH_HAA_)) +  0.116);
    (*this)(2,0) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) = ( c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,3) = ((((- 0.0039 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.21458 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.09523 *  s_q_LH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_COM_X_fr_base::Type_fr_LH_THIGH_COM_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_COM_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) =  c_q_LH_HFE_;
    (*this)(0,1) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(0,2) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(0,3) = ((((- 0.116 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ( 0.3405 *  c_q_LH_HFE_)) -  0.0039);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = ((- 0.116 *  c_q_LH_HAA_) -  0.09523);
    (*this)(2,0) =  s_q_LH_HFE_;
    (*this)(2,1) = (- s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,2) = ( c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,3) = ((( 0.3405 *  s_q_LH_HFE_) + (( 0.116 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) +  0.21458);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_ADAPTER::Type_fr_base_X_fr_RF_ADAPTER()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_ADAPTER& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_ADAPTER::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,2) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(0,3) = (((((- 0.1 *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (( 0.1 *  c_q_RF_HFE_) *  c_q_RF_KFE_)) - ( 0.25 *  s_q_RF_HFE_)) +  0.3405);
    (*this)(1,0) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,3) = ((((((( 0.1 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.1 *  s_q_RF_HAA_) *  s_q_RF_HFE_) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.13 *  c_q_RF_HAA_)) -  0.116);
    (*this)(2,0) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(2,3) = ((((((- 0.1 *  c_q_RF_HAA_) *  c_q_RF_HFE_) *  s_q_RF_KFE_) - ((( 0.1 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  c_q_RF_KFE_)) - (( 0.25 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.13 *  s_q_RF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_ADAPTER_X_fr_base::Type_fr_RF_ADAPTER_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_ADAPTER_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_ADAPTER_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,2) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,3) = (((((- 0.25 + (( 0.116 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.3405 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.116 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3405 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) -  0.1);
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = (( 0.116 *  c_q_RF_HAA_) +  0.13);
    (*this)(2,0) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(2,1) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(2,3) = ((((( 0.116 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3405 *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.25 - (( 0.116 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT::Type_fr_base_X_fr_RF_FOOT()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,2) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(0,3) = ((((((- 0.32125 *  c_q_RF_HFE_) - ( 0.1 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.1 *  c_q_RF_HFE_) - ( 0.32125 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.25 *  s_q_RF_HFE_)) +  0.3405);
    (*this)(1,0) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,3) = (((((((( 0.1 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.32125 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.32125 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.1 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.13 *  c_q_RF_HAA_)) -  0.116);
    (*this)(2,0) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(2,3) = ((((((( 0.32125 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.1 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.32125 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.1 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.25 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.13 *  s_q_RF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base::Type_fr_RF_FOOT_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,2) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,3) = (((((- 0.25 + (( 0.116 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.3405 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.116 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3405 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) -  0.1);
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = (( 0.116 *  c_q_RF_HAA_) +  0.13);
    (*this)(2,0) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(2,1) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(2,3) = (((((( 0.116 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3405 *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.25 - (( 0.116 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) +  0.32125);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP_COM::Type_fr_base_X_fr_RF_HIP_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.34152;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP_COM& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP_COM::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = - s_q_RF_HAA_;
    (*this)(1,3) = ((( 1.5E-4 *  s_q_RF_HAA_) + ( 0.00379 *  c_q_RF_HAA_)) -  0.116);
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) =  c_q_RF_HAA_;
    (*this)(2,3) = (( 0.00379 *  s_q_RF_HAA_) - ( 1.5E-4 *  c_q_RF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_HIP_COM_X_fr_base::Type_fr_RF_HIP_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.34152;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_HIP_COM_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_HIP_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = (( 0.116 *  c_q_RF_HAA_) -  0.00379);
    (*this)(2,1) = - s_q_RF_HAA_;
    (*this)(2,2) =  c_q_RF_HAA_;
    (*this)(2,3) = ( 1.5E-4 - ( 0.116 *  s_q_RF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK_COM::Type_fr_base_X_fr_RF_SHANK_COM()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK_COM& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_SHANK_COM::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,2) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(0,3) = ((((((- 0.09806 *  c_q_RF_HFE_) - ( 0.05873 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.05873 *  c_q_RF_HFE_) - ( 0.09806 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.25 *  s_q_RF_HFE_)) +  0.3405);
    (*this)(1,0) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,3) = (((((((( 0.05873 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.09806 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.09806 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.05873 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.13918 *  c_q_RF_HAA_)) -  0.116);
    (*this)(2,0) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(2,3) = ((((((( 0.09806 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.05873 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.09806 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.05873 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.25 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.13918 *  s_q_RF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_SHANK_COM_X_fr_base::Type_fr_RF_SHANK_COM_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_SHANK_COM_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_SHANK_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(0,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,2) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,3) = (((((- 0.25 + (( 0.116 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.3405 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.116 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3405 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) -  0.05873);
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = (( 0.116 *  c_q_RF_HAA_) +  0.13918);
    (*this)(2,0) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(2,1) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(2,3) = (((((( 0.116 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3405 *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.25 - (( 0.116 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.3405 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) +  0.09806);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH_COM::Type_fr_base_X_fr_RF_THIGH_COM()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH_COM& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_THIGH_COM::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) =  c_q_RF_HFE_;
    (*this)(0,2) =  s_q_RF_HFE_;
    (*this)(0,3) = (((- 0.21458 *  s_q_RF_HFE_) - ( 0.0039 *  c_q_RF_HFE_)) +  0.3405);
    (*this)(1,0) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = (- s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,3) = (((((- 0.0039 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.21458 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.09523 *  c_q_RF_HAA_)) -  0.116);
    (*this)(2,0) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) = ( c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,3) = (((( 0.0039 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.21458 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.09523 *  s_q_RF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_COM_X_fr_base::Type_fr_RF_THIGH_COM_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_COM_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) =  c_q_RF_HFE_;
    (*this)(0,1) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(0,2) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(0,3) = (((( 0.116 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3405 *  c_q_RF_HFE_)) +  0.0039);
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = (( 0.116 *  c_q_RF_HAA_) +  0.09523);
    (*this)(2,0) =  s_q_RF_HFE_;
    (*this)(2,1) = (- s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,2) = ( c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,3) = (((- 0.3405 *  s_q_RF_HFE_) - (( 0.116 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) +  0.21458);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_ADAPTER::Type_fr_base_X_fr_RH_ADAPTER()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_ADAPTER& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_ADAPTER::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,2) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(0,3) = ((((( 0.1 *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( 0.1 *  c_q_RH_HFE_) *  c_q_RH_KFE_)) - ( 0.25 *  s_q_RH_HFE_)) -  0.3405);
    (*this)(1,0) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,3) = (((((((- 0.1 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  s_q_RH_KFE_) - ((( 0.1 *  s_q_RH_HAA_) *  s_q_RH_HFE_) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.13 *  c_q_RH_HAA_)) -  0.116);
    (*this)(2,0) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(2,3) = (((((( 0.1 *  c_q_RH_HAA_) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.1 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  c_q_RH_KFE_)) - (( 0.25 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.13 *  s_q_RH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_ADAPTER_X_fr_base::Type_fr_RH_ADAPTER_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_ADAPTER_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_ADAPTER_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,2) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,3) = (((((- 0.25 + (( 0.116 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.3405 *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.3405 *  c_q_RH_HFE_) + (( 0.116 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) +  0.1);
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = (( 0.116 *  c_q_RH_HAA_) +  0.13);
    (*this)(2,0) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(2,1) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(2,3) = (((( 0.3405 *  c_q_RH_HFE_) + (( 0.116 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.25 - (( 0.116 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT::Type_fr_base_X_fr_RH_FOOT()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,2) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(0,3) = (((((( 0.1 *  s_q_RH_HFE_) - ( 0.32125 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((- 0.1 *  c_q_RH_HFE_) - ( 0.32125 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.25 *  s_q_RH_HFE_)) -  0.3405);
    (*this)(1,0) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,3) = ((((((((- 0.1 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.32125 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.32125 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.1 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.13 *  c_q_RH_HAA_)) -  0.116);
    (*this)(2,0) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(2,3) = ((((((( 0.1 *  c_q_RH_HAA_) *  c_q_RH_HFE_) + (( 0.32125 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.1 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.32125 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.25 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.13 *  s_q_RH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base::Type_fr_RH_FOOT_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,2) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,3) = (((((- 0.25 + (( 0.116 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.3405 *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.3405 *  c_q_RH_HFE_) + (( 0.116 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) +  0.1);
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = (( 0.116 *  c_q_RH_HAA_) +  0.13);
    (*this)(2,0) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(2,1) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(2,3) = ((((( 0.3405 *  c_q_RH_HFE_) + (( 0.116 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.25 - (( 0.116 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) +  0.32125);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP_COM::Type_fr_base_X_fr_RH_HIP_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.34152;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP_COM& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP_COM::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = - s_q_RH_HAA_;
    (*this)(1,3) = ((( 1.5E-4 *  s_q_RH_HAA_) + ( 0.00379 *  c_q_RH_HAA_)) -  0.116);
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) =  c_q_RH_HAA_;
    (*this)(2,3) = (( 0.00379 *  s_q_RH_HAA_) - ( 1.5E-4 *  c_q_RH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_HIP_COM_X_fr_base::Type_fr_RH_HIP_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.34152;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_HIP_COM_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_HIP_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = (( 0.116 *  c_q_RH_HAA_) -  0.00379);
    (*this)(2,1) = - s_q_RH_HAA_;
    (*this)(2,2) =  c_q_RH_HAA_;
    (*this)(2,3) = ( 1.5E-4 - ( 0.116 *  s_q_RH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK_COM::Type_fr_base_X_fr_RH_SHANK_COM()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK_COM& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_SHANK_COM::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,2) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(0,3) = (((((( 0.05873 *  s_q_RH_HFE_) - ( 0.09806 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((- 0.05873 *  c_q_RH_HFE_) - ( 0.09806 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.25 *  s_q_RH_HFE_)) -  0.3405);
    (*this)(1,0) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,3) = ((((((((- 0.05873 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.09806 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.09806 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.05873 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.13918 *  c_q_RH_HAA_)) -  0.116);
    (*this)(2,0) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(2,3) = ((((((( 0.05873 *  c_q_RH_HAA_) *  c_q_RH_HFE_) + (( 0.09806 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.05873 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.09806 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.25 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.13918 *  s_q_RH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_SHANK_COM_X_fr_base::Type_fr_RH_SHANK_COM_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_SHANK_COM_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_SHANK_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(0,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,2) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,3) = (((((- 0.25 + (( 0.116 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.3405 *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.3405 *  c_q_RH_HFE_) + (( 0.116 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) +  0.05873);
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = (( 0.116 *  c_q_RH_HAA_) +  0.13918);
    (*this)(2,0) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(2,1) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(2,3) = ((((( 0.3405 *  c_q_RH_HFE_) + (( 0.116 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.25 - (( 0.116 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.3405 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) +  0.09806);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH_COM::Type_fr_base_X_fr_RH_THIGH_COM()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH_COM& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_THIGH_COM::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) =  c_q_RH_HFE_;
    (*this)(0,2) =  s_q_RH_HFE_;
    (*this)(0,3) = (((- 0.21458 *  s_q_RH_HFE_) + ( 0.0039 *  c_q_RH_HFE_)) -  0.3405);
    (*this)(1,0) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = (- s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,3) = ((((( 0.0039 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.21458 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.09523 *  c_q_RH_HAA_)) -  0.116);
    (*this)(2,0) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) = ( c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,3) = ((((- 0.0039 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.21458 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.09523 *  s_q_RH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_COM_X_fr_base::Type_fr_RH_THIGH_COM_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_COM_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_COM_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) =  c_q_RH_HFE_;
    (*this)(0,1) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(0,2) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(0,3) = (((( 0.116 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ( 0.3405 *  c_q_RH_HFE_)) -  0.0039);
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = (( 0.116 *  c_q_RH_HAA_) +  0.09523);
    (*this)(2,0) =  s_q_RH_HFE_;
    (*this)(2,1) = (- s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,2) = ( c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,3) = ((( 0.3405 *  s_q_RH_HFE_) - (( 0.116 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) +  0.21458);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_base_COM::Type_fr_base_X_fr_base_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.00831;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.00324;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.05056;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_base_COM& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_base_COM::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_COM_X_fr_base::Type_fr_base_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.00831;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.00324;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.05056;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_COM_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_COM_X_fr_base::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_base_inertia::Type_fr_base_X_fr_base_inertia()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_base_inertia& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_base_inertia::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_inertia_X_fr_base::Type_fr_base_inertia_X_fr_base()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_inertia_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_inertia_X_fr_base::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_imu_link::Type_fr_base_X_fr_imu_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.062;
    (*this)(1,0) = 0;
    (*this)(1,1) = - 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.05755;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = - 1.0;
    (*this)(2,3) = 0.1837;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_imu_link& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_imu_link::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_imu_link_X_fr_base::Type_fr_imu_link_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.062;
    (*this)(1,0) = 0;
    (*this)(1,1) = - 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.05755;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = - 1.0;
    (*this)(2,3) = 0.1837;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_imu_link_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_imu_link_X_fr_base::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA::Type_fr_base_X_fr_LF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.277;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.116;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE::Type_fr_base_X_fr_LF_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.3405;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,1) =  s_q_LF_HAA_;
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = (( 0.041 *  c_q_LF_HAA_) +  0.116);
    (*this)(2,1) = - c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = ( 0.041 *  s_q_LF_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE::Type_fr_base_X_fr_LF_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) =  c_q_LF_HFE_;
    (*this)(0,1) = - s_q_LF_HFE_;
    (*this)(0,3) = ( 0.3405 - ( 0.25 *  s_q_LF_HFE_));
    (*this)(1,0) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(1,1) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = (((( 0.25 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ( 0.15 *  c_q_LF_HAA_)) +  0.116);
    (*this)(2,0) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(2,1) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = (( 0.15 *  s_q_LF_HAA_) - (( 0.25 *  c_q_LF_HAA_) *  c_q_LF_HFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA::Type_fr_base_X_fr_RF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.277;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.116;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE::Type_fr_base_X_fr_RF_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.3405;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,1) =  s_q_RF_HAA_;
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = ((- 0.041 *  c_q_RF_HAA_) -  0.116);
    (*this)(2,1) = - c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (- 0.041 *  s_q_RF_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE::Type_fr_base_X_fr_RF_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) =  c_q_RF_HFE_;
    (*this)(0,1) = - s_q_RF_HFE_;
    (*this)(0,3) = ( 0.3405 - ( 0.25 *  s_q_RF_HFE_));
    (*this)(1,0) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(1,1) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = (((( 0.25 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - ( 0.15 *  c_q_RF_HAA_)) -  0.116);
    (*this)(2,0) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(2,1) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (((- 0.25 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - ( 0.15 *  s_q_RF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA::Type_fr_base_X_fr_LH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.277;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.116;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE::Type_fr_base_X_fr_LH_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.3405;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,1) =  s_q_LH_HAA_;
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = (( 0.041 *  c_q_LH_HAA_) +  0.116);
    (*this)(2,1) = - c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ( 0.041 *  s_q_LH_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE::Type_fr_base_X_fr_LH_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) =  c_q_LH_HFE_;
    (*this)(0,1) = - s_q_LH_HFE_;
    (*this)(0,3) = ((- 0.25 *  s_q_LH_HFE_) -  0.3405);
    (*this)(1,0) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(1,1) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = (((( 0.25 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + ( 0.15 *  c_q_LH_HAA_)) +  0.116);
    (*this)(2,0) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(2,1) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = (( 0.15 *  s_q_LH_HAA_) - (( 0.25 *  c_q_LH_HAA_) *  c_q_LH_HFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA::Type_fr_base_X_fr_RH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.277;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.116;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE::Type_fr_base_X_fr_RH_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.3405;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,1) =  s_q_RH_HAA_;
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = ((- 0.041 *  c_q_RH_HAA_) -  0.116);
    (*this)(2,1) = - c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (- 0.041 *  s_q_RH_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE::Type_fr_base_X_fr_RH_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) =  c_q_RH_HFE_;
    (*this)(0,1) = - s_q_RH_HFE_;
    (*this)(0,3) = ((- 0.25 *  s_q_RH_HFE_) -  0.3405);
    (*this)(1,0) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(1,1) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = (((( 0.25 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - ( 0.15 *  c_q_RH_HAA_)) -  0.116);
    (*this)(2,0) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(2,1) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (((- 0.25 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - ( 0.15 *  s_q_RH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP::Type_fr_LF_THIGH_X_fr_LF_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.041;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  s_q_LF_HFE_;
    (*this)(0,2) =  c_q_LF_HFE_;
    (*this)(0,3) = (- 0.0635 *  c_q_LF_HFE_);
    (*this)(1,0) =  c_q_LF_HFE_;
    (*this)(1,2) = - s_q_LF_HFE_;
    (*this)(1,3) = ( 0.0635 *  s_q_LF_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH::Type_fr_LF_HIP_X_fr_LF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.041;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.0635;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  s_q_LF_HFE_;
    (*this)(0,1) =  c_q_LF_HFE_;
    (*this)(2,0) =  c_q_LF_HFE_;
    (*this)(2,1) = - s_q_LF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH::Type_fr_LF_SHANK_X_fr_LF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.109;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar c_q_LF_KFE_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  c_q_LF_KFE_;
    (*this)(0,1) =  s_q_LF_KFE_;
    (*this)(0,3) = (- 0.25 *  s_q_LF_KFE_);
    (*this)(1,0) = - s_q_LF_KFE_;
    (*this)(1,1) =  c_q_LF_KFE_;
    (*this)(1,3) = (- 0.25 *  c_q_LF_KFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK::Type_fr_LF_THIGH_X_fr_LF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.25;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.109;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar c_q_LF_KFE_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  c_q_LF_KFE_;
    (*this)(0,1) = - s_q_LF_KFE_;
    (*this)(1,0) =  s_q_LF_KFE_;
    (*this)(1,1) =  c_q_LF_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP::Type_fr_RF_THIGH_X_fr_RF_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.041;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  s_q_RF_HFE_;
    (*this)(0,2) =  c_q_RF_HFE_;
    (*this)(0,3) = (- 0.0635 *  c_q_RF_HFE_);
    (*this)(1,0) =  c_q_RF_HFE_;
    (*this)(1,2) = - s_q_RF_HFE_;
    (*this)(1,3) = ( 0.0635 *  s_q_RF_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH::Type_fr_RF_HIP_X_fr_RF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = - 0.041;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.0635;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  s_q_RF_HFE_;
    (*this)(0,1) =  c_q_RF_HFE_;
    (*this)(2,0) =  c_q_RF_HFE_;
    (*this)(2,1) = - s_q_RF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH::Type_fr_RF_SHANK_X_fr_RF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.109;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar c_q_RF_KFE_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  c_q_RF_KFE_;
    (*this)(0,1) =  s_q_RF_KFE_;
    (*this)(0,3) = (- 0.25 *  s_q_RF_KFE_);
    (*this)(1,0) = - s_q_RF_KFE_;
    (*this)(1,1) =  c_q_RF_KFE_;
    (*this)(1,3) = (- 0.25 *  c_q_RF_KFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK::Type_fr_RF_THIGH_X_fr_RF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.25;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.109;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar c_q_RF_KFE_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  c_q_RF_KFE_;
    (*this)(0,1) = - s_q_RF_KFE_;
    (*this)(1,0) =  s_q_RF_KFE_;
    (*this)(1,1) =  c_q_RF_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP::Type_fr_LH_THIGH_X_fr_LH_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.041;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  s_q_LH_HFE_;
    (*this)(0,2) =  c_q_LH_HFE_;
    (*this)(0,3) = ( 0.0635 *  c_q_LH_HFE_);
    (*this)(1,0) =  c_q_LH_HFE_;
    (*this)(1,2) = - s_q_LH_HFE_;
    (*this)(1,3) = (- 0.0635 *  s_q_LH_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH::Type_fr_LH_HIP_X_fr_LH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.041;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.0635;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  s_q_LH_HFE_;
    (*this)(0,1) =  c_q_LH_HFE_;
    (*this)(2,0) =  c_q_LH_HFE_;
    (*this)(2,1) = - s_q_LH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH::Type_fr_LH_SHANK_X_fr_LH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.109;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar c_q_LH_KFE_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  c_q_LH_KFE_;
    (*this)(0,1) =  s_q_LH_KFE_;
    (*this)(0,3) = (- 0.25 *  s_q_LH_KFE_);
    (*this)(1,0) = - s_q_LH_KFE_;
    (*this)(1,1) =  c_q_LH_KFE_;
    (*this)(1,3) = (- 0.25 *  c_q_LH_KFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK::Type_fr_LH_THIGH_X_fr_LH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.25;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.109;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar c_q_LH_KFE_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  c_q_LH_KFE_;
    (*this)(0,1) = - s_q_LH_KFE_;
    (*this)(1,0) =  s_q_LH_KFE_;
    (*this)(1,1) =  c_q_LH_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP::Type_fr_RH_THIGH_X_fr_RH_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.041;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  s_q_RH_HFE_;
    (*this)(0,2) =  c_q_RH_HFE_;
    (*this)(0,3) = ( 0.0635 *  c_q_RH_HFE_);
    (*this)(1,0) =  c_q_RH_HFE_;
    (*this)(1,2) = - s_q_RH_HFE_;
    (*this)(1,3) = (- 0.0635 *  s_q_RH_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH::Type_fr_RH_HIP_X_fr_RH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = - 0.041;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.0635;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  s_q_RH_HFE_;
    (*this)(0,1) =  c_q_RH_HFE_;
    (*this)(2,0) =  c_q_RH_HFE_;
    (*this)(2,1) = - s_q_RH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH::Type_fr_RH_SHANK_X_fr_RH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.109;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar c_q_RH_KFE_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  c_q_RH_KFE_;
    (*this)(0,1) =  s_q_RH_KFE_;
    (*this)(0,3) = (- 0.25 *  s_q_RH_KFE_);
    (*this)(1,0) = - s_q_RH_KFE_;
    (*this)(1,1) =  c_q_RH_KFE_;
    (*this)(1,3) = (- 0.25 *  c_q_RH_KFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK::Type_fr_RH_THIGH_X_fr_RH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.25;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.109;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar c_q_RH_KFE_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  c_q_RH_KFE_;
    (*this)(0,1) = - s_q_RH_KFE_;
    (*this)(1,0) =  s_q_RH_KFE_;
    (*this)(1,1) =  c_q_RH_KFE_;
    return *this;
}

