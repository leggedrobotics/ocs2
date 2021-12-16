
// Constructors
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::MotionTransforms
    ()
     :
    fr_base_X_fr_LF_FOOT(),
    fr_LF_FOOT_X_fr_base(),
    fr_base_X_fr_LH_FOOT(),
    fr_LH_FOOT_X_fr_base(),
    fr_base_X_fr_RF_FOOT(),
    fr_RF_FOOT_X_fr_base(),
    fr_base_X_fr_RH_FOOT(),
    fr_RH_FOOT_X_fr_base(),
    fr_base_X_fr_LF_HAA(),
    fr_base_X_fr_LF_HFE(),
    fr_base_X_fr_LF_KFE(),
    fr_base_X_fr_LH_HAA(),
    fr_base_X_fr_LH_HFE(),
    fr_base_X_fr_LH_KFE(),
    fr_base_X_fr_RF_HAA(),
    fr_base_X_fr_RF_HFE(),
    fr_base_X_fr_RF_KFE(),
    fr_base_X_fr_RH_HAA(),
    fr_base_X_fr_RH_HFE(),
    fr_base_X_fr_RH_KFE(),
    fr_LF_HIP_X_fr_base(),
    fr_base_X_fr_LF_HIP(),
    fr_LF_THIGH_X_fr_LF_HIP(),
    fr_LF_HIP_X_fr_LF_THIGH(),
    fr_LF_SHANK_X_fr_LF_THIGH(),
    fr_LF_THIGH_X_fr_LF_SHANK(),
    fr_RF_HIP_X_fr_base(),
    fr_base_X_fr_RF_HIP(),
    fr_RF_THIGH_X_fr_RF_HIP(),
    fr_RF_HIP_X_fr_RF_THIGH(),
    fr_RF_SHANK_X_fr_RF_THIGH(),
    fr_RF_THIGH_X_fr_RF_SHANK(),
    fr_LH_HIP_X_fr_base(),
    fr_base_X_fr_LH_HIP(),
    fr_LH_THIGH_X_fr_LH_HIP(),
    fr_LH_HIP_X_fr_LH_THIGH(),
    fr_LH_SHANK_X_fr_LH_THIGH(),
    fr_LH_THIGH_X_fr_LH_SHANK(),
    fr_RH_HIP_X_fr_base(),
    fr_base_X_fr_RH_HIP(),
    fr_RH_THIGH_X_fr_RH_HIP(),
    fr_RH_HIP_X_fr_RH_THIGH(),
    fr_RH_SHANK_X_fr_RH_THIGH(),
    fr_RH_THIGH_X_fr_RH_SHANK()
{
    updateParameters();
}
template <typename TRAIT>
void iit::camel::tpl::MotionTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::ForceTransforms
    ()
     :
    fr_base_X_fr_LF_FOOT(),
    fr_LF_FOOT_X_fr_base(),
    fr_base_X_fr_LH_FOOT(),
    fr_LH_FOOT_X_fr_base(),
    fr_base_X_fr_RF_FOOT(),
    fr_RF_FOOT_X_fr_base(),
    fr_base_X_fr_RH_FOOT(),
    fr_RH_FOOT_X_fr_base(),
    fr_base_X_fr_LF_HAA(),
    fr_base_X_fr_LF_HFE(),
    fr_base_X_fr_LF_KFE(),
    fr_base_X_fr_LH_HAA(),
    fr_base_X_fr_LH_HFE(),
    fr_base_X_fr_LH_KFE(),
    fr_base_X_fr_RF_HAA(),
    fr_base_X_fr_RF_HFE(),
    fr_base_X_fr_RF_KFE(),
    fr_base_X_fr_RH_HAA(),
    fr_base_X_fr_RH_HFE(),
    fr_base_X_fr_RH_KFE(),
    fr_LF_HIP_X_fr_base(),
    fr_base_X_fr_LF_HIP(),
    fr_LF_THIGH_X_fr_LF_HIP(),
    fr_LF_HIP_X_fr_LF_THIGH(),
    fr_LF_SHANK_X_fr_LF_THIGH(),
    fr_LF_THIGH_X_fr_LF_SHANK(),
    fr_RF_HIP_X_fr_base(),
    fr_base_X_fr_RF_HIP(),
    fr_RF_THIGH_X_fr_RF_HIP(),
    fr_RF_HIP_X_fr_RF_THIGH(),
    fr_RF_SHANK_X_fr_RF_THIGH(),
    fr_RF_THIGH_X_fr_RF_SHANK(),
    fr_LH_HIP_X_fr_base(),
    fr_base_X_fr_LH_HIP(),
    fr_LH_THIGH_X_fr_LH_HIP(),
    fr_LH_HIP_X_fr_LH_THIGH(),
    fr_LH_SHANK_X_fr_LH_THIGH(),
    fr_LH_THIGH_X_fr_LH_SHANK(),
    fr_RH_HIP_X_fr_base(),
    fr_base_X_fr_RH_HIP(),
    fr_RH_THIGH_X_fr_RH_HIP(),
    fr_RH_HIP_X_fr_RH_THIGH(),
    fr_RH_SHANK_X_fr_RH_THIGH(),
    fr_RH_THIGH_X_fr_RH_SHANK()
{
    updateParameters();
}
template <typename TRAIT>
void iit::camel::tpl::ForceTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::HomogeneousTransforms
    ()
     :
    fr_base_X_fr_LF_FOOT(),
    fr_LF_FOOT_X_fr_base(),
    fr_base_X_fr_LH_FOOT(),
    fr_LH_FOOT_X_fr_base(),
    fr_base_X_fr_RF_FOOT(),
    fr_RF_FOOT_X_fr_base(),
    fr_base_X_fr_RH_FOOT(),
    fr_RH_FOOT_X_fr_base(),
    fr_base_X_fr_LF_HAA(),
    fr_base_X_fr_LF_HFE(),
    fr_base_X_fr_LF_KFE(),
    fr_base_X_fr_LH_HAA(),
    fr_base_X_fr_LH_HFE(),
    fr_base_X_fr_LH_KFE(),
    fr_base_X_fr_RF_HAA(),
    fr_base_X_fr_RF_HFE(),
    fr_base_X_fr_RF_KFE(),
    fr_base_X_fr_RH_HAA(),
    fr_base_X_fr_RH_HFE(),
    fr_base_X_fr_RH_KFE(),
    fr_LF_HIP_X_fr_base(),
    fr_base_X_fr_LF_HIP(),
    fr_LF_THIGH_X_fr_LF_HIP(),
    fr_LF_HIP_X_fr_LF_THIGH(),
    fr_LF_SHANK_X_fr_LF_THIGH(),
    fr_LF_THIGH_X_fr_LF_SHANK(),
    fr_RF_HIP_X_fr_base(),
    fr_base_X_fr_RF_HIP(),
    fr_RF_THIGH_X_fr_RF_HIP(),
    fr_RF_HIP_X_fr_RF_THIGH(),
    fr_RF_SHANK_X_fr_RF_THIGH(),
    fr_RF_THIGH_X_fr_RF_SHANK(),
    fr_LH_HIP_X_fr_base(),
    fr_base_X_fr_LH_HIP(),
    fr_LH_THIGH_X_fr_LH_HIP(),
    fr_LH_HIP_X_fr_LH_THIGH(),
    fr_LH_SHANK_X_fr_LH_THIGH(),
    fr_LH_THIGH_X_fr_LH_SHANK(),
    fr_RH_HIP_X_fr_base(),
    fr_base_X_fr_RH_HIP(),
    fr_RH_THIGH_X_fr_RH_HIP(),
    fr_RH_HIP_X_fr_RH_THIGH(),
    fr_RH_SHANK_X_fr_RH_THIGH(),
    fr_RH_THIGH_X_fr_RH_SHANK()
{
    updateParameters();
}
template <typename TRAIT>
void iit::camel::tpl::HomogeneousTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT::Type_fr_base_X_fr_LF_FOOT()
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
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT::update(const JState& q) {
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
    (*this)(3,0) = (((((- 0.104 *  c_q_LF_HAA_) -  0.19716) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.19716) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,1) = (((((( 0.08795 *  c_q_LF_HFE_) - ( 0.33797 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.08795 *  s_q_LF_HFE_) + ( 0.33797 *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.285 *  c_q_LF_HFE_)) + ( 0.104 *  s_q_LF_HAA_));
    (*this)(3,2) = (((((- 0.104 *  c_q_LF_HAA_) -  0.19716) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + (((( 0.104 *  c_q_LF_HAA_) +  0.19716) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,5) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(4,0) = (((((( 0.3598 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.19716 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.3598 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.19716 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.285 *  c_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.33797 *  c_q_LF_HAA_));
    (*this)(4,1) = ((((((( 0.08795 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.33797 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.33797 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08795 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.285 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.3598 *  s_q_LF_HAA_));
    (*this)(4,2) = ((((((( 0.3598 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.19716 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.285 *  c_q_LF_HAA_)) *  s_q_LF_KFE_) + (((( 0.19716 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3598 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.08795 *  c_q_LF_HAA_));
    (*this)(4,3) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,0) = ((((((( 0.19716 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.3598 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.3598 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.19716 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.285 *  s_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.33797 *  s_q_LF_HAA_));
    (*this)(5,1) = (((((((- 0.08795 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.33797 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08795 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.33797 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.285 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.3598 *  c_q_LF_HAA_));
    (*this)(5,2) = ((((((( 0.3598 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.19716 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.285 *  s_q_LF_HAA_)) *  s_q_LF_KFE_) + (((((- 0.19716 *  c_q_LF_HAA_) -  0.104) *  s_q_LF_HFE_) - (( 0.3598 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.08795 *  s_q_LF_HAA_));
    (*this)(5,3) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,4) =  s_q_LF_HAA_;
    (*this)(5,5) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base::Type_fr_LF_FOOT_X_fr_base()
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
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base::update(const JState& q) {
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
    (*this)(3,0) = (((((- 0.104 *  c_q_LF_HAA_) -  0.19716) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.19716) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,1) = (((((( 0.3598 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.19716 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.3598 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.19716 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.285 *  c_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.33797 *  c_q_LF_HAA_));
    (*this)(3,2) = ((((((( 0.19716 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.3598 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.3598 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.19716 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.285 *  s_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.33797 *  s_q_LF_HAA_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,4) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,5) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,0) = (((((( 0.08795 *  c_q_LF_HFE_) - ( 0.33797 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.08795 *  s_q_LF_HFE_) + ( 0.33797 *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.285 *  c_q_LF_HFE_)) + ( 0.104 *  s_q_LF_HAA_));
    (*this)(4,1) = ((((((( 0.08795 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.33797 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.33797 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08795 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.285 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.3598 *  s_q_LF_HAA_));
    (*this)(4,2) = (((((((- 0.08795 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.33797 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08795 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.33797 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.285 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.3598 *  c_q_LF_HAA_));
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
    (*this)(5,0) = (((((- 0.104 *  c_q_LF_HAA_) -  0.19716) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + (((( 0.104 *  c_q_LF_HAA_) +  0.19716) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,1) = ((((((( 0.3598 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.19716 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.285 *  c_q_LF_HAA_)) *  s_q_LF_KFE_) + (((( 0.19716 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3598 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.08795 *  c_q_LF_HAA_));
    (*this)(5,2) = ((((((( 0.3598 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.19716 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.285 *  s_q_LF_HAA_)) *  s_q_LF_KFE_) + (((((- 0.19716 *  c_q_LF_HAA_) -  0.104) *  s_q_LF_HFE_) - (( 0.3598 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.08795 *  s_q_LF_HAA_));
    (*this)(5,3) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(5,4) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,5) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT::Type_fr_base_X_fr_LH_FOOT()
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
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT::update(const JState& q) {
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
    (*this)(3,0) = (((((- 0.104 *  c_q_LH_HAA_) -  0.19716) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.19716) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,1) = ((((((- 0.33797 *  s_q_LH_HFE_) - ( 0.08795 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.33797 *  c_q_LH_HFE_) - ( 0.08795 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.285 *  c_q_LH_HFE_)) + ( 0.104 *  s_q_LH_HAA_));
    (*this)(3,2) = (((((- 0.104 *  c_q_LH_HAA_) -  0.19716) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + (((( 0.104 *  c_q_LH_HAA_) +  0.19716) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,5) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(4,0) = ((((((- 0.19716 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.3598 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.3598 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.19716 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.285 *  c_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.33797 *  c_q_LH_HAA_));
    (*this)(4,1) = ((((((( 0.33797 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08795 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.33797 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08795 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.285 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.3598 *  s_q_LH_HAA_));
    (*this)(4,2) = (((((((- 0.3598 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.19716 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.285 *  c_q_LH_HAA_)) *  s_q_LH_KFE_) + (((( 0.19716 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.3598 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.08795 *  c_q_LH_HAA_));
    (*this)(4,3) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,0) = ((((((( 0.19716 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.3598 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.3598 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.19716 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.285 *  s_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.33797 *  s_q_LH_HAA_));
    (*this)(5,1) = ((((((( 0.08795 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.33797 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.33797 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08795 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.285 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.3598 *  c_q_LH_HAA_));
    (*this)(5,2) = (((((((- 0.3598 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.19716 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.285 *  s_q_LH_HAA_)) *  s_q_LH_KFE_) + (((((- 0.19716 *  c_q_LH_HAA_) -  0.104) *  s_q_LH_HFE_) + (( 0.3598 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.08795 *  s_q_LH_HAA_));
    (*this)(5,3) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,4) =  s_q_LH_HAA_;
    (*this)(5,5) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base::Type_fr_LH_FOOT_X_fr_base()
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
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base::update(const JState& q) {
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
    (*this)(3,0) = (((((- 0.104 *  c_q_LH_HAA_) -  0.19716) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.19716) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,1) = ((((((- 0.19716 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.3598 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.3598 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.19716 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.285 *  c_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.33797 *  c_q_LH_HAA_));
    (*this)(3,2) = ((((((( 0.19716 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.3598 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.3598 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.19716 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.285 *  s_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.33797 *  s_q_LH_HAA_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,4) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,5) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,0) = ((((((- 0.33797 *  s_q_LH_HFE_) - ( 0.08795 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.33797 *  c_q_LH_HFE_) - ( 0.08795 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.285 *  c_q_LH_HFE_)) + ( 0.104 *  s_q_LH_HAA_));
    (*this)(4,1) = ((((((( 0.33797 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08795 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.33797 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08795 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.285 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.3598 *  s_q_LH_HAA_));
    (*this)(4,2) = ((((((( 0.08795 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.33797 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.33797 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08795 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.285 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.3598 *  c_q_LH_HAA_));
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
    (*this)(5,0) = (((((- 0.104 *  c_q_LH_HAA_) -  0.19716) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + (((( 0.104 *  c_q_LH_HAA_) +  0.19716) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,1) = (((((((- 0.3598 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.19716 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.285 *  c_q_LH_HAA_)) *  s_q_LH_KFE_) + (((( 0.19716 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.3598 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.08795 *  c_q_LH_HAA_));
    (*this)(5,2) = (((((((- 0.3598 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.19716 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.285 *  s_q_LH_HAA_)) *  s_q_LH_KFE_) + (((((- 0.19716 *  c_q_LH_HAA_) -  0.104) *  s_q_LH_HFE_) + (( 0.3598 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.08795 *  s_q_LH_HAA_));
    (*this)(5,3) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(5,4) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,5) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT::Type_fr_base_X_fr_RF_FOOT()
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
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT::update(const JState& q) {
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
    (*this)(3,0) = ((((( 0.104 *  c_q_RF_HAA_) +  0.19716) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.19716) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,1) = (((((( 0.08795 *  c_q_RF_HFE_) - ( 0.33797 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.08795 *  s_q_RF_HFE_) + ( 0.33797 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.285 *  c_q_RF_HFE_)) - ( 0.104 *  s_q_RF_HAA_));
    (*this)(3,2) = ((((( 0.104 *  c_q_RF_HAA_) +  0.19716) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + ((((- 0.104 *  c_q_RF_HAA_) -  0.19716) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,5) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(4,0) = (((((( 0.19716 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.3598 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.3598 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.19716 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.285 *  c_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.33797 *  c_q_RF_HAA_));
    (*this)(4,1) = ((((((( 0.08795 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.33797 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.33797 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08795 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.285 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.3598 *  s_q_RF_HAA_));
    (*this)(4,2) = ((((((( 0.3598 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.19716 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.285 *  c_q_RF_HAA_)) *  s_q_RF_KFE_) + ((((- 0.19716 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.3598 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.08795 *  c_q_RF_HAA_));
    (*this)(4,3) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,0) = (((((((- 0.19716 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.3598 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.3598 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.19716 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.285 *  s_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.33797 *  s_q_RF_HAA_));
    (*this)(5,1) = (((((((- 0.08795 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.33797 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08795 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.33797 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.285 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.3598 *  c_q_RF_HAA_));
    (*this)(5,2) = ((((((( 0.3598 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.19716 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.285 *  s_q_RF_HAA_)) *  s_q_RF_KFE_) + ((((( 0.19716 *  c_q_RF_HAA_) +  0.104) *  s_q_RF_HFE_) - (( 0.3598 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.08795 *  s_q_RF_HAA_));
    (*this)(5,3) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,4) =  s_q_RF_HAA_;
    (*this)(5,5) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base::Type_fr_RF_FOOT_X_fr_base()
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
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base::update(const JState& q) {
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
    (*this)(3,0) = ((((( 0.104 *  c_q_RF_HAA_) +  0.19716) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.19716) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,1) = (((((( 0.19716 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.3598 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.3598 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.19716 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.285 *  c_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.33797 *  c_q_RF_HAA_));
    (*this)(3,2) = (((((((- 0.19716 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.3598 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.3598 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.19716 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.285 *  s_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.33797 *  s_q_RF_HAA_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,4) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,5) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,0) = (((((( 0.08795 *  c_q_RF_HFE_) - ( 0.33797 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.08795 *  s_q_RF_HFE_) + ( 0.33797 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.285 *  c_q_RF_HFE_)) - ( 0.104 *  s_q_RF_HAA_));
    (*this)(4,1) = ((((((( 0.08795 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.33797 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.33797 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08795 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.285 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.3598 *  s_q_RF_HAA_));
    (*this)(4,2) = (((((((- 0.08795 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.33797 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08795 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.33797 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.285 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.3598 *  c_q_RF_HAA_));
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
    (*this)(5,0) = ((((( 0.104 *  c_q_RF_HAA_) +  0.19716) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + ((((- 0.104 *  c_q_RF_HAA_) -  0.19716) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,1) = ((((((( 0.3598 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.19716 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.285 *  c_q_RF_HAA_)) *  s_q_RF_KFE_) + ((((- 0.19716 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.3598 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.08795 *  c_q_RF_HAA_));
    (*this)(5,2) = ((((((( 0.3598 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.19716 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.285 *  s_q_RF_HAA_)) *  s_q_RF_KFE_) + ((((( 0.19716 *  c_q_RF_HAA_) +  0.104) *  s_q_RF_HFE_) - (( 0.3598 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.08795 *  s_q_RF_HAA_));
    (*this)(5,3) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(5,4) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,5) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT::Type_fr_base_X_fr_RH_FOOT()
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
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT::update(const JState& q) {
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
    (*this)(3,0) = ((((( 0.104 *  c_q_RH_HAA_) +  0.19716) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.19716) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,1) = ((((((- 0.33797 *  s_q_RH_HFE_) - ( 0.08795 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.33797 *  c_q_RH_HFE_) - ( 0.08795 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.285 *  c_q_RH_HFE_)) - ( 0.104 *  s_q_RH_HAA_));
    (*this)(3,2) = ((((( 0.104 *  c_q_RH_HAA_) +  0.19716) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + ((((- 0.104 *  c_q_RH_HAA_) -  0.19716) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,5) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(4,0) = (((((( 0.19716 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3598 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.3598 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.19716 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.285 *  c_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.33797 *  c_q_RH_HAA_));
    (*this)(4,1) = ((((((( 0.33797 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08795 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.33797 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08795 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.285 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.3598 *  s_q_RH_HAA_));
    (*this)(4,2) = (((((((- 0.3598 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.19716 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.285 *  c_q_RH_HAA_)) *  s_q_RH_KFE_) + (((( 0.3598 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.19716 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.08795 *  c_q_RH_HAA_));
    (*this)(4,3) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,0) = (((((((- 0.19716 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.3598 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.3598 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.19716 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.285 *  s_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.33797 *  s_q_RH_HAA_));
    (*this)(5,1) = ((((((( 0.08795 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.33797 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.33797 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08795 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.285 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.3598 *  c_q_RH_HAA_));
    (*this)(5,2) = (((((((- 0.3598 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.19716 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.285 *  s_q_RH_HAA_)) *  s_q_RH_KFE_) + ((((( 0.19716 *  c_q_RH_HAA_) +  0.104) *  s_q_RH_HFE_) + (( 0.3598 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.08795 *  s_q_RH_HAA_));
    (*this)(5,3) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,4) =  s_q_RH_HAA_;
    (*this)(5,5) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base::Type_fr_RH_FOOT_X_fr_base()
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
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base::update(const JState& q) {
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
    (*this)(3,0) = ((((( 0.104 *  c_q_RH_HAA_) +  0.19716) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.19716) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,1) = (((((( 0.19716 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3598 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.3598 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.19716 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.285 *  c_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.33797 *  c_q_RH_HAA_));
    (*this)(3,2) = (((((((- 0.19716 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.3598 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.3598 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.19716 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.285 *  s_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.33797 *  s_q_RH_HAA_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,4) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,5) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,0) = ((((((- 0.33797 *  s_q_RH_HFE_) - ( 0.08795 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.33797 *  c_q_RH_HFE_) - ( 0.08795 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.285 *  c_q_RH_HFE_)) - ( 0.104 *  s_q_RH_HAA_));
    (*this)(4,1) = ((((((( 0.33797 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08795 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.33797 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08795 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.285 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.3598 *  s_q_RH_HAA_));
    (*this)(4,2) = ((((((( 0.08795 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.33797 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.33797 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08795 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.285 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.3598 *  c_q_RH_HAA_));
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
    (*this)(5,0) = ((((( 0.104 *  c_q_RH_HAA_) +  0.19716) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + ((((- 0.104 *  c_q_RH_HAA_) -  0.19716) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,1) = (((((((- 0.3598 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.19716 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.285 *  c_q_RH_HAA_)) *  s_q_RH_KFE_) + (((( 0.3598 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.19716 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.08795 *  c_q_RH_HAA_));
    (*this)(5,2) = (((((((- 0.3598 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.19716 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.285 *  s_q_RH_HAA_)) *  s_q_RH_KFE_) + ((((( 0.19716 *  c_q_RH_HAA_) +  0.104) *  s_q_RH_HFE_) + (( 0.3598 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.08795 *  s_q_RH_HAA_));
    (*this)(5,3) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(5,4) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,5) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA::Type_fr_base_X_fr_LF_HAA()
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
    (*this)(3,0) = - 0.104;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.2999;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0.2999;
    (*this)(5,2) = - 0.104;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE::Type_fr_base_X_fr_LF_HFE()
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
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,1) =  s_q_LF_HAA_;
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(2,1) = - c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(3,1) = ((- 0.104 *  c_q_LF_HAA_) -  0.08381);
    (*this)(3,2) = ( 0.104 *  s_q_LF_HAA_);
    (*this)(4,0) = ( 0.08381 *  s_q_LF_HAA_);
    (*this)(4,1) = ( 0.3598 *  c_q_LF_HAA_);
    (*this)(4,2) = (- 0.3598 *  s_q_LF_HAA_);
    (*this)(4,4) =  s_q_LF_HAA_;
    (*this)(4,5) =  c_q_LF_HAA_;
    (*this)(5,0) = ((- 0.08381 *  c_q_LF_HAA_) -  0.104);
    (*this)(5,1) = ( 0.3598 *  s_q_LF_HAA_);
    (*this)(5,2) = ( 0.3598 *  c_q_LF_HAA_);
    (*this)(5,4) = - c_q_LF_HAA_;
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE::Type_fr_base_X_fr_LF_KFE()
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
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE::update(const JState& q) {
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
    (*this)(3,0) = (((- 0.104 *  c_q_LF_HAA_) -  0.18411) *  s_q_LF_HFE_);
    (*this)(3,1) = (((- 0.104 *  c_q_LF_HAA_) -  0.18411) *  c_q_LF_HFE_);
    (*this)(3,2) = (( 0.285 *  c_q_LF_HFE_) + ( 0.104 *  s_q_LF_HAA_));
    (*this)(3,3) =  c_q_LF_HFE_;
    (*this)(3,4) = - s_q_LF_HFE_;
    (*this)(4,0) = (((( 0.3598 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.18411 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.285 *  c_q_LF_HAA_));
    (*this)(4,1) = ((( 0.3598 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18411 *  s_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(4,2) = ((( 0.285 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - ( 0.3598 *  s_q_LF_HAA_));
    (*this)(4,3) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(4,4) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(4,5) =  c_q_LF_HAA_;
    (*this)(5,0) = (((( 0.3598 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.18411 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.285 *  s_q_LF_HAA_));
    (*this)(5,1) = (((( 0.18411 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.3598 *  s_q_LF_HAA_) *  c_q_LF_HFE_));
    (*this)(5,2) = (( 0.3598 *  c_q_LF_HAA_) - (( 0.285 *  c_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(5,3) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(5,4) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA::Type_fr_base_X_fr_LH_HAA()
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
    (*this)(3,0) = - 0.104;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = - 0.2999;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.2999;
    (*this)(5,2) = - 0.104;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE::Type_fr_base_X_fr_LH_HFE()
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
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,1) =  s_q_LH_HAA_;
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(2,1) = - c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(3,1) = ((- 0.104 *  c_q_LH_HAA_) -  0.08381);
    (*this)(3,2) = ( 0.104 *  s_q_LH_HAA_);
    (*this)(4,0) = ( 0.08381 *  s_q_LH_HAA_);
    (*this)(4,1) = (- 0.3598 *  c_q_LH_HAA_);
    (*this)(4,2) = ( 0.3598 *  s_q_LH_HAA_);
    (*this)(4,4) =  s_q_LH_HAA_;
    (*this)(4,5) =  c_q_LH_HAA_;
    (*this)(5,0) = ((- 0.08381 *  c_q_LH_HAA_) -  0.104);
    (*this)(5,1) = (- 0.3598 *  s_q_LH_HAA_);
    (*this)(5,2) = (- 0.3598 *  c_q_LH_HAA_);
    (*this)(5,4) = - c_q_LH_HAA_;
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE::Type_fr_base_X_fr_LH_KFE()
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
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE::update(const JState& q) {
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
    (*this)(3,0) = (((- 0.104 *  c_q_LH_HAA_) -  0.18411) *  s_q_LH_HFE_);
    (*this)(3,1) = (((- 0.104 *  c_q_LH_HAA_) -  0.18411) *  c_q_LH_HFE_);
    (*this)(3,2) = (( 0.285 *  c_q_LH_HFE_) + ( 0.104 *  s_q_LH_HAA_));
    (*this)(3,3) =  c_q_LH_HFE_;
    (*this)(3,4) = - s_q_LH_HFE_;
    (*this)(4,0) = ((((- 0.3598 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.18411 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.285 *  c_q_LH_HAA_));
    (*this)(4,1) = (((- 0.18411 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.3598 *  c_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(4,2) = ((( 0.285 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ( 0.3598 *  s_q_LH_HAA_));
    (*this)(4,3) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(4,4) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(4,5) =  c_q_LH_HAA_;
    (*this)(5,0) = ((((- 0.3598 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.18411 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.285 *  s_q_LH_HAA_));
    (*this)(5,1) = (((( 0.18411 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.3598 *  s_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(5,2) = (((- 0.285 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - ( 0.3598 *  c_q_LH_HAA_));
    (*this)(5,3) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(5,4) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA::Type_fr_base_X_fr_RF_HAA()
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
    (*this)(3,0) = 0.104;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.2999;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0.2999;
    (*this)(5,2) = 0.104;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE::Type_fr_base_X_fr_RF_HFE()
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
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,1) =  s_q_RF_HAA_;
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(2,1) = - c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(3,1) = (( 0.104 *  c_q_RF_HAA_) +  0.08381);
    (*this)(3,2) = (- 0.104 *  s_q_RF_HAA_);
    (*this)(4,0) = (- 0.08381 *  s_q_RF_HAA_);
    (*this)(4,1) = ( 0.3598 *  c_q_RF_HAA_);
    (*this)(4,2) = (- 0.3598 *  s_q_RF_HAA_);
    (*this)(4,4) =  s_q_RF_HAA_;
    (*this)(4,5) =  c_q_RF_HAA_;
    (*this)(5,0) = (( 0.08381 *  c_q_RF_HAA_) +  0.104);
    (*this)(5,1) = ( 0.3598 *  s_q_RF_HAA_);
    (*this)(5,2) = ( 0.3598 *  c_q_RF_HAA_);
    (*this)(5,4) = - c_q_RF_HAA_;
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE::Type_fr_base_X_fr_RF_KFE()
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
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE::update(const JState& q) {
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
    (*this)(3,0) = ((( 0.104 *  c_q_RF_HAA_) +  0.18411) *  s_q_RF_HFE_);
    (*this)(3,1) = ((( 0.104 *  c_q_RF_HAA_) +  0.18411) *  c_q_RF_HFE_);
    (*this)(3,2) = (( 0.285 *  c_q_RF_HFE_) - ( 0.104 *  s_q_RF_HAA_));
    (*this)(3,3) =  c_q_RF_HFE_;
    (*this)(3,4) = - s_q_RF_HFE_;
    (*this)(4,0) = (((( 0.3598 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.18411 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.285 *  c_q_RF_HAA_));
    (*this)(4,1) = ((( 0.18411 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.3598 *  c_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(4,2) = ((( 0.285 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3598 *  s_q_RF_HAA_));
    (*this)(4,3) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(4,4) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(4,5) =  c_q_RF_HAA_;
    (*this)(5,0) = (((( 0.3598 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.18411 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.285 *  s_q_RF_HAA_));
    (*this)(5,1) = ((((- 0.18411 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.3598 *  s_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(5,2) = (( 0.3598 *  c_q_RF_HAA_) - (( 0.285 *  c_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(5,3) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(5,4) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA::Type_fr_base_X_fr_RH_HAA()
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
    (*this)(3,0) = 0.104;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = - 0.2999;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.2999;
    (*this)(5,2) = 0.104;
    (*this)(5,3) = - 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE::Type_fr_base_X_fr_RH_HFE()
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
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,1) =  s_q_RH_HAA_;
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(2,1) = - c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(3,1) = (( 0.104 *  c_q_RH_HAA_) +  0.08381);
    (*this)(3,2) = (- 0.104 *  s_q_RH_HAA_);
    (*this)(4,0) = (- 0.08381 *  s_q_RH_HAA_);
    (*this)(4,1) = (- 0.3598 *  c_q_RH_HAA_);
    (*this)(4,2) = ( 0.3598 *  s_q_RH_HAA_);
    (*this)(4,4) =  s_q_RH_HAA_;
    (*this)(4,5) =  c_q_RH_HAA_;
    (*this)(5,0) = (( 0.08381 *  c_q_RH_HAA_) +  0.104);
    (*this)(5,1) = (- 0.3598 *  s_q_RH_HAA_);
    (*this)(5,2) = (- 0.3598 *  c_q_RH_HAA_);
    (*this)(5,4) = - c_q_RH_HAA_;
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE::Type_fr_base_X_fr_RH_KFE()
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
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE::update(const JState& q) {
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
    (*this)(3,0) = ((( 0.104 *  c_q_RH_HAA_) +  0.18411) *  s_q_RH_HFE_);
    (*this)(3,1) = ((( 0.104 *  c_q_RH_HAA_) +  0.18411) *  c_q_RH_HFE_);
    (*this)(3,2) = (( 0.285 *  c_q_RH_HFE_) - ( 0.104 *  s_q_RH_HAA_));
    (*this)(3,3) =  c_q_RH_HFE_;
    (*this)(3,4) = - s_q_RH_HFE_;
    (*this)(4,0) = ((((- 0.3598 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.18411 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.285 *  c_q_RH_HAA_));
    (*this)(4,1) = ((( 0.18411 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3598 *  c_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(4,2) = ((( 0.285 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ( 0.3598 *  s_q_RH_HAA_));
    (*this)(4,3) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(4,4) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(4,5) =  c_q_RH_HAA_;
    (*this)(5,0) = ((((- 0.3598 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.18411 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.285 *  s_q_RH_HAA_));
    (*this)(5,1) = ((((- 0.18411 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.3598 *  s_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(5,2) = (((- 0.285 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - ( 0.3598 *  c_q_RH_HAA_));
    (*this)(5,3) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(5,4) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base::Type_fr_LF_HIP_X_fr_base()
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
    (*this)(5,2) = - 0.104;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,1) =  s_q_LF_HAA_;
    (*this)(0,2) = - c_q_LF_HAA_;
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(3,0) = (- 0.104 *  c_q_LF_HAA_);
    (*this)(3,1) = ( 0.2999 *  c_q_LF_HAA_);
    (*this)(3,2) = ( 0.2999 *  s_q_LF_HAA_);
    (*this)(3,4) =  s_q_LF_HAA_;
    (*this)(3,5) = - c_q_LF_HAA_;
    (*this)(4,0) = ( 0.104 *  s_q_LF_HAA_);
    (*this)(4,1) = (- 0.2999 *  s_q_LF_HAA_);
    (*this)(4,2) = ( 0.2999 *  c_q_LF_HAA_);
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP::Type_fr_base_X_fr_LF_HIP()
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
    (*this)(5,2) = - 0.104;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,0) =  s_q_LF_HAA_;
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(2,0) = - c_q_LF_HAA_;
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(3,0) = (- 0.104 *  c_q_LF_HAA_);
    (*this)(3,1) = ( 0.104 *  s_q_LF_HAA_);
    (*this)(4,0) = ( 0.2999 *  c_q_LF_HAA_);
    (*this)(4,1) = (- 0.2999 *  s_q_LF_HAA_);
    (*this)(4,3) =  s_q_LF_HAA_;
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(5,0) = ( 0.2999 *  s_q_LF_HAA_);
    (*this)(5,1) = ( 0.2999 *  c_q_LF_HAA_);
    (*this)(5,3) = - c_q_LF_HAA_;
    (*this)(5,4) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP::Type_fr_LF_THIGH_X_fr_LF_HIP()
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
    (*this)(5,0) = - 0.0599;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  s_q_LF_HFE_;
    (*this)(0,2) =  c_q_LF_HFE_;
    (*this)(1,0) =  c_q_LF_HFE_;
    (*this)(1,2) = - s_q_LF_HFE_;
    (*this)(3,0) = ( 0.08381 *  c_q_LF_HFE_);
    (*this)(3,1) = ( 0.0599 *  s_q_LF_HFE_);
    (*this)(3,2) = (- 0.08381 *  s_q_LF_HFE_);
    (*this)(3,3) =  s_q_LF_HFE_;
    (*this)(3,5) =  c_q_LF_HFE_;
    (*this)(4,0) = (- 0.08381 *  s_q_LF_HFE_);
    (*this)(4,1) = ( 0.0599 *  c_q_LF_HFE_);
    (*this)(4,2) = (- 0.08381 *  c_q_LF_HFE_);
    (*this)(4,3) =  c_q_LF_HFE_;
    (*this)(4,5) = - s_q_LF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH::Type_fr_LF_HIP_X_fr_LF_THIGH()
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
    (*this)(3,2) = - 0.0599;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  s_q_LF_HFE_;
    (*this)(0,1) =  c_q_LF_HFE_;
    (*this)(2,0) =  c_q_LF_HFE_;
    (*this)(2,1) = - s_q_LF_HFE_;
    (*this)(3,0) = ( 0.08381 *  c_q_LF_HFE_);
    (*this)(3,1) = (- 0.08381 *  s_q_LF_HFE_);
    (*this)(3,3) =  s_q_LF_HFE_;
    (*this)(3,4) =  c_q_LF_HFE_;
    (*this)(4,0) = ( 0.0599 *  s_q_LF_HFE_);
    (*this)(4,1) = ( 0.0599 *  c_q_LF_HFE_);
    (*this)(5,0) = (- 0.08381 *  s_q_LF_HFE_);
    (*this)(5,1) = (- 0.08381 *  c_q_LF_HFE_);
    (*this)(5,3) =  c_q_LF_HFE_;
    (*this)(5,4) = - s_q_LF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH::Type_fr_LF_SHANK_X_fr_LF_THIGH()
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
    (*this)(5,0) = 0.285;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar c_q_LF_KFE_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  c_q_LF_KFE_;
    (*this)(0,1) =  s_q_LF_KFE_;
    (*this)(1,0) = - s_q_LF_KFE_;
    (*this)(1,1) =  c_q_LF_KFE_;
    (*this)(3,0) = (- 0.1003 *  s_q_LF_KFE_);
    (*this)(3,1) = ( 0.1003 *  c_q_LF_KFE_);
    (*this)(3,2) = (- 0.285 *  c_q_LF_KFE_);
    (*this)(3,3) =  c_q_LF_KFE_;
    (*this)(3,4) =  s_q_LF_KFE_;
    (*this)(4,0) = (- 0.1003 *  c_q_LF_KFE_);
    (*this)(4,1) = (- 0.1003 *  s_q_LF_KFE_);
    (*this)(4,2) = ( 0.285 *  s_q_LF_KFE_);
    (*this)(4,3) = - s_q_LF_KFE_;
    (*this)(4,4) =  c_q_LF_KFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK::Type_fr_LF_THIGH_X_fr_LF_SHANK()
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
    (*this)(3,2) = 0.285;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar c_q_LF_KFE_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  c_q_LF_KFE_;
    (*this)(0,1) = - s_q_LF_KFE_;
    (*this)(1,0) =  s_q_LF_KFE_;
    (*this)(1,1) =  c_q_LF_KFE_;
    (*this)(3,0) = (- 0.1003 *  s_q_LF_KFE_);
    (*this)(3,1) = (- 0.1003 *  c_q_LF_KFE_);
    (*this)(3,3) =  c_q_LF_KFE_;
    (*this)(3,4) = - s_q_LF_KFE_;
    (*this)(4,0) = ( 0.1003 *  c_q_LF_KFE_);
    (*this)(4,1) = (- 0.1003 *  s_q_LF_KFE_);
    (*this)(4,3) =  s_q_LF_KFE_;
    (*this)(4,4) =  c_q_LF_KFE_;
    (*this)(5,0) = (- 0.285 *  c_q_LF_KFE_);
    (*this)(5,1) = ( 0.285 *  s_q_LF_KFE_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base::Type_fr_RF_HIP_X_fr_base()
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
    (*this)(5,2) = 0.104;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,1) =  s_q_RF_HAA_;
    (*this)(0,2) = - c_q_RF_HAA_;
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(3,0) = ( 0.104 *  c_q_RF_HAA_);
    (*this)(3,1) = ( 0.2999 *  c_q_RF_HAA_);
    (*this)(3,2) = ( 0.2999 *  s_q_RF_HAA_);
    (*this)(3,4) =  s_q_RF_HAA_;
    (*this)(3,5) = - c_q_RF_HAA_;
    (*this)(4,0) = (- 0.104 *  s_q_RF_HAA_);
    (*this)(4,1) = (- 0.2999 *  s_q_RF_HAA_);
    (*this)(4,2) = ( 0.2999 *  c_q_RF_HAA_);
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP::Type_fr_base_X_fr_RF_HIP()
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
    (*this)(5,2) = 0.104;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,0) =  s_q_RF_HAA_;
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(2,0) = - c_q_RF_HAA_;
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(3,0) = ( 0.104 *  c_q_RF_HAA_);
    (*this)(3,1) = (- 0.104 *  s_q_RF_HAA_);
    (*this)(4,0) = ( 0.2999 *  c_q_RF_HAA_);
    (*this)(4,1) = (- 0.2999 *  s_q_RF_HAA_);
    (*this)(4,3) =  s_q_RF_HAA_;
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(5,0) = ( 0.2999 *  s_q_RF_HAA_);
    (*this)(5,1) = ( 0.2999 *  c_q_RF_HAA_);
    (*this)(5,3) = - c_q_RF_HAA_;
    (*this)(5,4) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP::Type_fr_RF_THIGH_X_fr_RF_HIP()
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
    (*this)(5,0) = - 0.0599;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  s_q_RF_HFE_;
    (*this)(0,2) =  c_q_RF_HFE_;
    (*this)(1,0) =  c_q_RF_HFE_;
    (*this)(1,2) = - s_q_RF_HFE_;
    (*this)(3,0) = (- 0.08381 *  c_q_RF_HFE_);
    (*this)(3,1) = ( 0.0599 *  s_q_RF_HFE_);
    (*this)(3,2) = ( 0.08381 *  s_q_RF_HFE_);
    (*this)(3,3) =  s_q_RF_HFE_;
    (*this)(3,5) =  c_q_RF_HFE_;
    (*this)(4,0) = ( 0.08381 *  s_q_RF_HFE_);
    (*this)(4,1) = ( 0.0599 *  c_q_RF_HFE_);
    (*this)(4,2) = ( 0.08381 *  c_q_RF_HFE_);
    (*this)(4,3) =  c_q_RF_HFE_;
    (*this)(4,5) = - s_q_RF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH::Type_fr_RF_HIP_X_fr_RF_THIGH()
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
    (*this)(3,2) = - 0.0599;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  s_q_RF_HFE_;
    (*this)(0,1) =  c_q_RF_HFE_;
    (*this)(2,0) =  c_q_RF_HFE_;
    (*this)(2,1) = - s_q_RF_HFE_;
    (*this)(3,0) = (- 0.08381 *  c_q_RF_HFE_);
    (*this)(3,1) = ( 0.08381 *  s_q_RF_HFE_);
    (*this)(3,3) =  s_q_RF_HFE_;
    (*this)(3,4) =  c_q_RF_HFE_;
    (*this)(4,0) = ( 0.0599 *  s_q_RF_HFE_);
    (*this)(4,1) = ( 0.0599 *  c_q_RF_HFE_);
    (*this)(5,0) = ( 0.08381 *  s_q_RF_HFE_);
    (*this)(5,1) = ( 0.08381 *  c_q_RF_HFE_);
    (*this)(5,3) =  c_q_RF_HFE_;
    (*this)(5,4) = - s_q_RF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH::Type_fr_RF_SHANK_X_fr_RF_THIGH()
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
    (*this)(5,0) = 0.285;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar c_q_RF_KFE_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  c_q_RF_KFE_;
    (*this)(0,1) =  s_q_RF_KFE_;
    (*this)(1,0) = - s_q_RF_KFE_;
    (*this)(1,1) =  c_q_RF_KFE_;
    (*this)(3,0) = ( 0.1003 *  s_q_RF_KFE_);
    (*this)(3,1) = (- 0.1003 *  c_q_RF_KFE_);
    (*this)(3,2) = (- 0.285 *  c_q_RF_KFE_);
    (*this)(3,3) =  c_q_RF_KFE_;
    (*this)(3,4) =  s_q_RF_KFE_;
    (*this)(4,0) = ( 0.1003 *  c_q_RF_KFE_);
    (*this)(4,1) = ( 0.1003 *  s_q_RF_KFE_);
    (*this)(4,2) = ( 0.285 *  s_q_RF_KFE_);
    (*this)(4,3) = - s_q_RF_KFE_;
    (*this)(4,4) =  c_q_RF_KFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK::Type_fr_RF_THIGH_X_fr_RF_SHANK()
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
    (*this)(3,2) = 0.285;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar c_q_RF_KFE_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  c_q_RF_KFE_;
    (*this)(0,1) = - s_q_RF_KFE_;
    (*this)(1,0) =  s_q_RF_KFE_;
    (*this)(1,1) =  c_q_RF_KFE_;
    (*this)(3,0) = ( 0.1003 *  s_q_RF_KFE_);
    (*this)(3,1) = ( 0.1003 *  c_q_RF_KFE_);
    (*this)(3,3) =  c_q_RF_KFE_;
    (*this)(3,4) = - s_q_RF_KFE_;
    (*this)(4,0) = (- 0.1003 *  c_q_RF_KFE_);
    (*this)(4,1) = ( 0.1003 *  s_q_RF_KFE_);
    (*this)(4,3) =  s_q_RF_KFE_;
    (*this)(4,4) =  c_q_RF_KFE_;
    (*this)(5,0) = (- 0.285 *  c_q_RF_KFE_);
    (*this)(5,1) = ( 0.285 *  s_q_RF_KFE_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base::Type_fr_LH_HIP_X_fr_base()
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
    (*this)(5,2) = - 0.104;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,1) =  s_q_LH_HAA_;
    (*this)(0,2) = - c_q_LH_HAA_;
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(3,0) = (- 0.104 *  c_q_LH_HAA_);
    (*this)(3,1) = (- 0.2999 *  c_q_LH_HAA_);
    (*this)(3,2) = (- 0.2999 *  s_q_LH_HAA_);
    (*this)(3,4) =  s_q_LH_HAA_;
    (*this)(3,5) = - c_q_LH_HAA_;
    (*this)(4,0) = ( 0.104 *  s_q_LH_HAA_);
    (*this)(4,1) = ( 0.2999 *  s_q_LH_HAA_);
    (*this)(4,2) = (- 0.2999 *  c_q_LH_HAA_);
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP::Type_fr_base_X_fr_LH_HIP()
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
    (*this)(5,2) = - 0.104;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,0) =  s_q_LH_HAA_;
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(2,0) = - c_q_LH_HAA_;
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(3,0) = (- 0.104 *  c_q_LH_HAA_);
    (*this)(3,1) = ( 0.104 *  s_q_LH_HAA_);
    (*this)(4,0) = (- 0.2999 *  c_q_LH_HAA_);
    (*this)(4,1) = ( 0.2999 *  s_q_LH_HAA_);
    (*this)(4,3) =  s_q_LH_HAA_;
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(5,0) = (- 0.2999 *  s_q_LH_HAA_);
    (*this)(5,1) = (- 0.2999 *  c_q_LH_HAA_);
    (*this)(5,3) = - c_q_LH_HAA_;
    (*this)(5,4) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP::Type_fr_LH_THIGH_X_fr_LH_HIP()
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
    (*this)(5,0) = 0.0599;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  s_q_LH_HFE_;
    (*this)(0,2) =  c_q_LH_HFE_;
    (*this)(1,0) =  c_q_LH_HFE_;
    (*this)(1,2) = - s_q_LH_HFE_;
    (*this)(3,0) = ( 0.08381 *  c_q_LH_HFE_);
    (*this)(3,1) = (- 0.0599 *  s_q_LH_HFE_);
    (*this)(3,2) = (- 0.08381 *  s_q_LH_HFE_);
    (*this)(3,3) =  s_q_LH_HFE_;
    (*this)(3,5) =  c_q_LH_HFE_;
    (*this)(4,0) = (- 0.08381 *  s_q_LH_HFE_);
    (*this)(4,1) = (- 0.0599 *  c_q_LH_HFE_);
    (*this)(4,2) = (- 0.08381 *  c_q_LH_HFE_);
    (*this)(4,3) =  c_q_LH_HFE_;
    (*this)(4,5) = - s_q_LH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH::Type_fr_LH_HIP_X_fr_LH_THIGH()
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
    (*this)(3,2) = 0.0599;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  s_q_LH_HFE_;
    (*this)(0,1) =  c_q_LH_HFE_;
    (*this)(2,0) =  c_q_LH_HFE_;
    (*this)(2,1) = - s_q_LH_HFE_;
    (*this)(3,0) = ( 0.08381 *  c_q_LH_HFE_);
    (*this)(3,1) = (- 0.08381 *  s_q_LH_HFE_);
    (*this)(3,3) =  s_q_LH_HFE_;
    (*this)(3,4) =  c_q_LH_HFE_;
    (*this)(4,0) = (- 0.0599 *  s_q_LH_HFE_);
    (*this)(4,1) = (- 0.0599 *  c_q_LH_HFE_);
    (*this)(5,0) = (- 0.08381 *  s_q_LH_HFE_);
    (*this)(5,1) = (- 0.08381 *  c_q_LH_HFE_);
    (*this)(5,3) =  c_q_LH_HFE_;
    (*this)(5,4) = - s_q_LH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH::Type_fr_LH_SHANK_X_fr_LH_THIGH()
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
    (*this)(5,0) = 0.285;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar c_q_LH_KFE_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  c_q_LH_KFE_;
    (*this)(0,1) =  s_q_LH_KFE_;
    (*this)(1,0) = - s_q_LH_KFE_;
    (*this)(1,1) =  c_q_LH_KFE_;
    (*this)(3,0) = (- 0.1003 *  s_q_LH_KFE_);
    (*this)(3,1) = ( 0.1003 *  c_q_LH_KFE_);
    (*this)(3,2) = (- 0.285 *  c_q_LH_KFE_);
    (*this)(3,3) =  c_q_LH_KFE_;
    (*this)(3,4) =  s_q_LH_KFE_;
    (*this)(4,0) = (- 0.1003 *  c_q_LH_KFE_);
    (*this)(4,1) = (- 0.1003 *  s_q_LH_KFE_);
    (*this)(4,2) = ( 0.285 *  s_q_LH_KFE_);
    (*this)(4,3) = - s_q_LH_KFE_;
    (*this)(4,4) =  c_q_LH_KFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK::Type_fr_LH_THIGH_X_fr_LH_SHANK()
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
    (*this)(3,2) = 0.285;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar c_q_LH_KFE_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  c_q_LH_KFE_;
    (*this)(0,1) = - s_q_LH_KFE_;
    (*this)(1,0) =  s_q_LH_KFE_;
    (*this)(1,1) =  c_q_LH_KFE_;
    (*this)(3,0) = (- 0.1003 *  s_q_LH_KFE_);
    (*this)(3,1) = (- 0.1003 *  c_q_LH_KFE_);
    (*this)(3,3) =  c_q_LH_KFE_;
    (*this)(3,4) = - s_q_LH_KFE_;
    (*this)(4,0) = ( 0.1003 *  c_q_LH_KFE_);
    (*this)(4,1) = (- 0.1003 *  s_q_LH_KFE_);
    (*this)(4,3) =  s_q_LH_KFE_;
    (*this)(4,4) =  c_q_LH_KFE_;
    (*this)(5,0) = (- 0.285 *  c_q_LH_KFE_);
    (*this)(5,1) = ( 0.285 *  s_q_LH_KFE_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base::Type_fr_RH_HIP_X_fr_base()
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
    (*this)(5,2) = 0.104;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,1) =  s_q_RH_HAA_;
    (*this)(0,2) = - c_q_RH_HAA_;
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(3,0) = ( 0.104 *  c_q_RH_HAA_);
    (*this)(3,1) = (- 0.2999 *  c_q_RH_HAA_);
    (*this)(3,2) = (- 0.2999 *  s_q_RH_HAA_);
    (*this)(3,4) =  s_q_RH_HAA_;
    (*this)(3,5) = - c_q_RH_HAA_;
    (*this)(4,0) = (- 0.104 *  s_q_RH_HAA_);
    (*this)(4,1) = ( 0.2999 *  s_q_RH_HAA_);
    (*this)(4,2) = (- 0.2999 *  c_q_RH_HAA_);
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP::Type_fr_base_X_fr_RH_HIP()
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
    (*this)(5,2) = 0.104;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,0) =  s_q_RH_HAA_;
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(2,0) = - c_q_RH_HAA_;
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(3,0) = ( 0.104 *  c_q_RH_HAA_);
    (*this)(3,1) = (- 0.104 *  s_q_RH_HAA_);
    (*this)(4,0) = (- 0.2999 *  c_q_RH_HAA_);
    (*this)(4,1) = ( 0.2999 *  s_q_RH_HAA_);
    (*this)(4,3) =  s_q_RH_HAA_;
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(5,0) = (- 0.2999 *  s_q_RH_HAA_);
    (*this)(5,1) = (- 0.2999 *  c_q_RH_HAA_);
    (*this)(5,3) = - c_q_RH_HAA_;
    (*this)(5,4) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP::Type_fr_RH_THIGH_X_fr_RH_HIP()
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
    (*this)(5,0) = 0.0599;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  s_q_RH_HFE_;
    (*this)(0,2) =  c_q_RH_HFE_;
    (*this)(1,0) =  c_q_RH_HFE_;
    (*this)(1,2) = - s_q_RH_HFE_;
    (*this)(3,0) = (- 0.08381 *  c_q_RH_HFE_);
    (*this)(3,1) = (- 0.0599 *  s_q_RH_HFE_);
    (*this)(3,2) = ( 0.08381 *  s_q_RH_HFE_);
    (*this)(3,3) =  s_q_RH_HFE_;
    (*this)(3,5) =  c_q_RH_HFE_;
    (*this)(4,0) = ( 0.08381 *  s_q_RH_HFE_);
    (*this)(4,1) = (- 0.0599 *  c_q_RH_HFE_);
    (*this)(4,2) = ( 0.08381 *  c_q_RH_HFE_);
    (*this)(4,3) =  c_q_RH_HFE_;
    (*this)(4,5) = - s_q_RH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH::Type_fr_RH_HIP_X_fr_RH_THIGH()
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
    (*this)(3,2) = 0.0599;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  s_q_RH_HFE_;
    (*this)(0,1) =  c_q_RH_HFE_;
    (*this)(2,0) =  c_q_RH_HFE_;
    (*this)(2,1) = - s_q_RH_HFE_;
    (*this)(3,0) = (- 0.08381 *  c_q_RH_HFE_);
    (*this)(3,1) = ( 0.08381 *  s_q_RH_HFE_);
    (*this)(3,3) =  s_q_RH_HFE_;
    (*this)(3,4) =  c_q_RH_HFE_;
    (*this)(4,0) = (- 0.0599 *  s_q_RH_HFE_);
    (*this)(4,1) = (- 0.0599 *  c_q_RH_HFE_);
    (*this)(5,0) = ( 0.08381 *  s_q_RH_HFE_);
    (*this)(5,1) = ( 0.08381 *  c_q_RH_HFE_);
    (*this)(5,3) =  c_q_RH_HFE_;
    (*this)(5,4) = - s_q_RH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH::Type_fr_RH_SHANK_X_fr_RH_THIGH()
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
    (*this)(5,0) = 0.285;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar c_q_RH_KFE_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  c_q_RH_KFE_;
    (*this)(0,1) =  s_q_RH_KFE_;
    (*this)(1,0) = - s_q_RH_KFE_;
    (*this)(1,1) =  c_q_RH_KFE_;
    (*this)(3,0) = ( 0.1003 *  s_q_RH_KFE_);
    (*this)(3,1) = (- 0.1003 *  c_q_RH_KFE_);
    (*this)(3,2) = (- 0.285 *  c_q_RH_KFE_);
    (*this)(3,3) =  c_q_RH_KFE_;
    (*this)(3,4) =  s_q_RH_KFE_;
    (*this)(4,0) = ( 0.1003 *  c_q_RH_KFE_);
    (*this)(4,1) = ( 0.1003 *  s_q_RH_KFE_);
    (*this)(4,2) = ( 0.285 *  s_q_RH_KFE_);
    (*this)(4,3) = - s_q_RH_KFE_;
    (*this)(4,4) =  c_q_RH_KFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK::Type_fr_RH_THIGH_X_fr_RH_SHANK()
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
    (*this)(3,2) = 0.285;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK& iit::camel::tpl::MotionTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar c_q_RH_KFE_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  c_q_RH_KFE_;
    (*this)(0,1) = - s_q_RH_KFE_;
    (*this)(1,0) =  s_q_RH_KFE_;
    (*this)(1,1) =  c_q_RH_KFE_;
    (*this)(3,0) = ( 0.1003 *  s_q_RH_KFE_);
    (*this)(3,1) = ( 0.1003 *  c_q_RH_KFE_);
    (*this)(3,3) =  c_q_RH_KFE_;
    (*this)(3,4) = - s_q_RH_KFE_;
    (*this)(4,0) = (- 0.1003 *  c_q_RH_KFE_);
    (*this)(4,1) = ( 0.1003 *  s_q_RH_KFE_);
    (*this)(4,3) =  s_q_RH_KFE_;
    (*this)(4,4) =  c_q_RH_KFE_;
    (*this)(5,0) = (- 0.285 *  c_q_RH_KFE_);
    (*this)(5,1) = ( 0.285 *  s_q_RH_KFE_);
    return *this;
}

template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT::Type_fr_base_X_fr_LF_FOOT()
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT::update(const JState& q) {
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
    (*this)(0,3) = (((((- 0.104 *  c_q_LF_HAA_) -  0.19716) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.19716) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,4) = (((((( 0.08795 *  c_q_LF_HFE_) - ( 0.33797 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.08795 *  s_q_LF_HFE_) + ( 0.33797 *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.285 *  c_q_LF_HFE_)) + ( 0.104 *  s_q_LF_HAA_));
    (*this)(0,5) = (((((- 0.104 *  c_q_LF_HAA_) -  0.19716) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + (((( 0.104 *  c_q_LF_HAA_) +  0.19716) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,0) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,3) = (((((( 0.3598 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.19716 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.3598 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.19716 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.285 *  c_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.33797 *  c_q_LF_HAA_));
    (*this)(1,4) = ((((((( 0.08795 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.33797 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.33797 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08795 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.285 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.3598 *  s_q_LF_HAA_));
    (*this)(1,5) = ((((((( 0.3598 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.19716 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.285 *  c_q_LF_HAA_)) *  s_q_LF_KFE_) + (((( 0.19716 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3598 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.08795 *  c_q_LF_HAA_));
    (*this)(2,0) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(2,3) = ((((((( 0.19716 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.3598 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.3598 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.19716 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.285 *  s_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.33797 *  s_q_LF_HAA_));
    (*this)(2,4) = (((((((- 0.08795 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.33797 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08795 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.33797 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.285 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.3598 *  c_q_LF_HAA_));
    (*this)(2,5) = ((((((( 0.3598 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.19716 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.285 *  s_q_LF_HAA_)) *  s_q_LF_KFE_) + (((((- 0.19716 *  c_q_LF_HAA_) -  0.104) *  s_q_LF_HFE_) - (( 0.3598 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.08795 *  s_q_LF_HAA_));
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
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base::Type_fr_LF_FOOT_X_fr_base()
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base::update(const JState& q) {
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
    (*this)(0,3) = (((((- 0.104 *  c_q_LF_HAA_) -  0.19716) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.19716) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,4) = (((((( 0.3598 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.19716 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.3598 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.19716 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.285 *  c_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.33797 *  c_q_LF_HAA_));
    (*this)(0,5) = ((((((( 0.19716 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.3598 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.3598 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.19716 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.285 *  s_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.33797 *  s_q_LF_HAA_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = (((((( 0.08795 *  c_q_LF_HFE_) - ( 0.33797 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.08795 *  s_q_LF_HFE_) + ( 0.33797 *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.285 *  c_q_LF_HFE_)) + ( 0.104 *  s_q_LF_HAA_));
    (*this)(1,4) = ((((((( 0.08795 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.33797 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.33797 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08795 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.285 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.3598 *  s_q_LF_HAA_));
    (*this)(1,5) = (((((((- 0.08795 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.33797 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08795 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.33797 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.285 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.3598 *  c_q_LF_HAA_));
    (*this)(2,0) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(2,1) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(2,3) = (((((- 0.104 *  c_q_LF_HAA_) -  0.19716) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + (((( 0.104 *  c_q_LF_HAA_) +  0.19716) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,4) = ((((((( 0.3598 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.19716 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.285 *  c_q_LF_HAA_)) *  s_q_LF_KFE_) + (((( 0.19716 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.3598 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.08795 *  c_q_LF_HAA_));
    (*this)(2,5) = ((((((( 0.3598 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.19716 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.285 *  s_q_LF_HAA_)) *  s_q_LF_KFE_) + (((((- 0.19716 *  c_q_LF_HAA_) -  0.104) *  s_q_LF_HFE_) - (( 0.3598 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.08795 *  s_q_LF_HAA_));
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
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT::Type_fr_base_X_fr_LH_FOOT()
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT::update(const JState& q) {
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
    (*this)(0,3) = (((((- 0.104 *  c_q_LH_HAA_) -  0.19716) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.19716) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,4) = ((((((- 0.33797 *  s_q_LH_HFE_) - ( 0.08795 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.33797 *  c_q_LH_HFE_) - ( 0.08795 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.285 *  c_q_LH_HFE_)) + ( 0.104 *  s_q_LH_HAA_));
    (*this)(0,5) = (((((- 0.104 *  c_q_LH_HAA_) -  0.19716) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + (((( 0.104 *  c_q_LH_HAA_) +  0.19716) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,0) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,3) = ((((((- 0.19716 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.3598 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.3598 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.19716 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.285 *  c_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.33797 *  c_q_LH_HAA_));
    (*this)(1,4) = ((((((( 0.33797 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08795 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.33797 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08795 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.285 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.3598 *  s_q_LH_HAA_));
    (*this)(1,5) = (((((((- 0.3598 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.19716 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.285 *  c_q_LH_HAA_)) *  s_q_LH_KFE_) + (((( 0.19716 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.3598 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.08795 *  c_q_LH_HAA_));
    (*this)(2,0) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(2,3) = ((((((( 0.19716 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.3598 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.3598 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.19716 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.285 *  s_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.33797 *  s_q_LH_HAA_));
    (*this)(2,4) = ((((((( 0.08795 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.33797 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.33797 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08795 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.285 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.3598 *  c_q_LH_HAA_));
    (*this)(2,5) = (((((((- 0.3598 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.19716 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.285 *  s_q_LH_HAA_)) *  s_q_LH_KFE_) + (((((- 0.19716 *  c_q_LH_HAA_) -  0.104) *  s_q_LH_HFE_) + (( 0.3598 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.08795 *  s_q_LH_HAA_));
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
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base::Type_fr_LH_FOOT_X_fr_base()
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base::update(const JState& q) {
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
    (*this)(0,3) = (((((- 0.104 *  c_q_LH_HAA_) -  0.19716) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.19716) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,4) = ((((((- 0.19716 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.3598 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.3598 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.19716 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.285 *  c_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.33797 *  c_q_LH_HAA_));
    (*this)(0,5) = ((((((( 0.19716 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.3598 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.3598 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.19716 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.285 *  s_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.33797 *  s_q_LH_HAA_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = ((((((- 0.33797 *  s_q_LH_HFE_) - ( 0.08795 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.33797 *  c_q_LH_HFE_) - ( 0.08795 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.285 *  c_q_LH_HFE_)) + ( 0.104 *  s_q_LH_HAA_));
    (*this)(1,4) = ((((((( 0.33797 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08795 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.33797 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08795 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.285 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.3598 *  s_q_LH_HAA_));
    (*this)(1,5) = ((((((( 0.08795 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.33797 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.33797 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08795 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.285 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.3598 *  c_q_LH_HAA_));
    (*this)(2,0) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(2,1) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(2,3) = (((((- 0.104 *  c_q_LH_HAA_) -  0.19716) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + (((( 0.104 *  c_q_LH_HAA_) +  0.19716) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,4) = (((((((- 0.3598 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.19716 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.285 *  c_q_LH_HAA_)) *  s_q_LH_KFE_) + (((( 0.19716 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.3598 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.08795 *  c_q_LH_HAA_));
    (*this)(2,5) = (((((((- 0.3598 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.19716 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.285 *  s_q_LH_HAA_)) *  s_q_LH_KFE_) + (((((- 0.19716 *  c_q_LH_HAA_) -  0.104) *  s_q_LH_HFE_) + (( 0.3598 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.08795 *  s_q_LH_HAA_));
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
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT::Type_fr_base_X_fr_RF_FOOT()
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT::update(const JState& q) {
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
    (*this)(0,3) = ((((( 0.104 *  c_q_RF_HAA_) +  0.19716) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.19716) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,4) = (((((( 0.08795 *  c_q_RF_HFE_) - ( 0.33797 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.08795 *  s_q_RF_HFE_) + ( 0.33797 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.285 *  c_q_RF_HFE_)) - ( 0.104 *  s_q_RF_HAA_));
    (*this)(0,5) = ((((( 0.104 *  c_q_RF_HAA_) +  0.19716) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + ((((- 0.104 *  c_q_RF_HAA_) -  0.19716) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,0) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,3) = (((((( 0.19716 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.3598 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.3598 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.19716 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.285 *  c_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.33797 *  c_q_RF_HAA_));
    (*this)(1,4) = ((((((( 0.08795 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.33797 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.33797 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08795 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.285 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.3598 *  s_q_RF_HAA_));
    (*this)(1,5) = ((((((( 0.3598 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.19716 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.285 *  c_q_RF_HAA_)) *  s_q_RF_KFE_) + ((((- 0.19716 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.3598 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.08795 *  c_q_RF_HAA_));
    (*this)(2,0) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(2,3) = (((((((- 0.19716 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.3598 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.3598 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.19716 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.285 *  s_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.33797 *  s_q_RF_HAA_));
    (*this)(2,4) = (((((((- 0.08795 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.33797 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08795 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.33797 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.285 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.3598 *  c_q_RF_HAA_));
    (*this)(2,5) = ((((((( 0.3598 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.19716 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.285 *  s_q_RF_HAA_)) *  s_q_RF_KFE_) + ((((( 0.19716 *  c_q_RF_HAA_) +  0.104) *  s_q_RF_HFE_) - (( 0.3598 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.08795 *  s_q_RF_HAA_));
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
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base::Type_fr_RF_FOOT_X_fr_base()
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base::update(const JState& q) {
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
    (*this)(0,3) = ((((( 0.104 *  c_q_RF_HAA_) +  0.19716) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.19716) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,4) = (((((( 0.19716 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.3598 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.3598 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.19716 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.285 *  c_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.33797 *  c_q_RF_HAA_));
    (*this)(0,5) = (((((((- 0.19716 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.3598 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.3598 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.19716 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.285 *  s_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.33797 *  s_q_RF_HAA_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = (((((( 0.08795 *  c_q_RF_HFE_) - ( 0.33797 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.08795 *  s_q_RF_HFE_) + ( 0.33797 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.285 *  c_q_RF_HFE_)) - ( 0.104 *  s_q_RF_HAA_));
    (*this)(1,4) = ((((((( 0.08795 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.33797 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.33797 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08795 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.285 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.3598 *  s_q_RF_HAA_));
    (*this)(1,5) = (((((((- 0.08795 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.33797 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08795 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.33797 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.285 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.3598 *  c_q_RF_HAA_));
    (*this)(2,0) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(2,1) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(2,3) = ((((( 0.104 *  c_q_RF_HAA_) +  0.19716) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + ((((- 0.104 *  c_q_RF_HAA_) -  0.19716) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,4) = ((((((( 0.3598 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.19716 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.285 *  c_q_RF_HAA_)) *  s_q_RF_KFE_) + ((((- 0.19716 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.3598 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.08795 *  c_q_RF_HAA_));
    (*this)(2,5) = ((((((( 0.3598 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.19716 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.285 *  s_q_RF_HAA_)) *  s_q_RF_KFE_) + ((((( 0.19716 *  c_q_RF_HAA_) +  0.104) *  s_q_RF_HFE_) - (( 0.3598 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.08795 *  s_q_RF_HAA_));
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
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT::Type_fr_base_X_fr_RH_FOOT()
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT::update(const JState& q) {
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
    (*this)(0,3) = ((((( 0.104 *  c_q_RH_HAA_) +  0.19716) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.19716) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,4) = ((((((- 0.33797 *  s_q_RH_HFE_) - ( 0.08795 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.33797 *  c_q_RH_HFE_) - ( 0.08795 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.285 *  c_q_RH_HFE_)) - ( 0.104 *  s_q_RH_HAA_));
    (*this)(0,5) = ((((( 0.104 *  c_q_RH_HAA_) +  0.19716) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + ((((- 0.104 *  c_q_RH_HAA_) -  0.19716) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,0) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,3) = (((((( 0.19716 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3598 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.3598 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.19716 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.285 *  c_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.33797 *  c_q_RH_HAA_));
    (*this)(1,4) = ((((((( 0.33797 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08795 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.33797 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08795 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.285 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.3598 *  s_q_RH_HAA_));
    (*this)(1,5) = (((((((- 0.3598 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.19716 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.285 *  c_q_RH_HAA_)) *  s_q_RH_KFE_) + (((( 0.3598 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.19716 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.08795 *  c_q_RH_HAA_));
    (*this)(2,0) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(2,3) = (((((((- 0.19716 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.3598 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.3598 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.19716 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.285 *  s_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.33797 *  s_q_RH_HAA_));
    (*this)(2,4) = ((((((( 0.08795 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.33797 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.33797 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08795 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.285 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.3598 *  c_q_RH_HAA_));
    (*this)(2,5) = (((((((- 0.3598 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.19716 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.285 *  s_q_RH_HAA_)) *  s_q_RH_KFE_) + ((((( 0.19716 *  c_q_RH_HAA_) +  0.104) *  s_q_RH_HFE_) + (( 0.3598 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.08795 *  s_q_RH_HAA_));
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
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base::Type_fr_RH_FOOT_X_fr_base()
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base::update(const JState& q) {
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
    (*this)(0,3) = ((((( 0.104 *  c_q_RH_HAA_) +  0.19716) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.19716) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,4) = (((((( 0.19716 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3598 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.3598 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.19716 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.285 *  c_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.33797 *  c_q_RH_HAA_));
    (*this)(0,5) = (((((((- 0.19716 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.3598 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.3598 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.19716 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.285 *  s_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.33797 *  s_q_RH_HAA_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = ((((((- 0.33797 *  s_q_RH_HFE_) - ( 0.08795 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.33797 *  c_q_RH_HFE_) - ( 0.08795 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.285 *  c_q_RH_HFE_)) - ( 0.104 *  s_q_RH_HAA_));
    (*this)(1,4) = ((((((( 0.33797 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08795 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.33797 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08795 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.285 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.3598 *  s_q_RH_HAA_));
    (*this)(1,5) = ((((((( 0.08795 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.33797 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.33797 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08795 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.285 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.3598 *  c_q_RH_HAA_));
    (*this)(2,0) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(2,1) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(2,3) = ((((( 0.104 *  c_q_RH_HAA_) +  0.19716) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + ((((- 0.104 *  c_q_RH_HAA_) -  0.19716) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,4) = (((((((- 0.3598 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.19716 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.285 *  c_q_RH_HAA_)) *  s_q_RH_KFE_) + (((( 0.3598 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.19716 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.08795 *  c_q_RH_HAA_));
    (*this)(2,5) = (((((((- 0.3598 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.19716 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.285 *  s_q_RH_HAA_)) *  s_q_RH_KFE_) + ((((( 0.19716 *  c_q_RH_HAA_) +  0.104) *  s_q_RH_HFE_) + (( 0.3598 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.08795 *  s_q_RH_HAA_));
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
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA::Type_fr_base_X_fr_LF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.104;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.2999;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0.2999;
    (*this)(2,5) = - 0.104;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE::Type_fr_base_X_fr_LF_HFE()
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,4) = ((- 0.104 *  c_q_LF_HAA_) -  0.08381);
    (*this)(0,5) = ( 0.104 *  s_q_LF_HAA_);
    (*this)(1,1) =  s_q_LF_HAA_;
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = ( 0.08381 *  s_q_LF_HAA_);
    (*this)(1,4) = ( 0.3598 *  c_q_LF_HAA_);
    (*this)(1,5) = (- 0.3598 *  s_q_LF_HAA_);
    (*this)(2,1) = - c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = ((- 0.08381 *  c_q_LF_HAA_) -  0.104);
    (*this)(2,4) = ( 0.3598 *  s_q_LF_HAA_);
    (*this)(2,5) = ( 0.3598 *  c_q_LF_HAA_);
    (*this)(4,4) =  s_q_LF_HAA_;
    (*this)(4,5) =  c_q_LF_HAA_;
    (*this)(5,4) = - c_q_LF_HAA_;
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE::Type_fr_base_X_fr_LF_KFE()
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE::update(const JState& q) {
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
    (*this)(0,3) = (((- 0.104 *  c_q_LF_HAA_) -  0.18411) *  s_q_LF_HFE_);
    (*this)(0,4) = (((- 0.104 *  c_q_LF_HAA_) -  0.18411) *  c_q_LF_HFE_);
    (*this)(0,5) = (( 0.285 *  c_q_LF_HFE_) + ( 0.104 *  s_q_LF_HAA_));
    (*this)(1,0) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(1,1) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = (((( 0.3598 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.18411 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.285 *  c_q_LF_HAA_));
    (*this)(1,4) = ((( 0.3598 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18411 *  s_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(1,5) = ((( 0.285 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - ( 0.3598 *  s_q_LF_HAA_));
    (*this)(2,0) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(2,1) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = (((( 0.3598 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.18411 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.285 *  s_q_LF_HAA_));
    (*this)(2,4) = (((( 0.18411 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.3598 *  s_q_LF_HAA_) *  c_q_LF_HFE_));
    (*this)(2,5) = (( 0.3598 *  c_q_LF_HAA_) - (( 0.285 *  c_q_LF_HAA_) *  s_q_LF_HFE_));
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
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA::Type_fr_base_X_fr_LH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.104;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.2999;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.2999;
    (*this)(2,5) = - 0.104;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE::Type_fr_base_X_fr_LH_HFE()
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,4) = ((- 0.104 *  c_q_LH_HAA_) -  0.08381);
    (*this)(0,5) = ( 0.104 *  s_q_LH_HAA_);
    (*this)(1,1) =  s_q_LH_HAA_;
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = ( 0.08381 *  s_q_LH_HAA_);
    (*this)(1,4) = (- 0.3598 *  c_q_LH_HAA_);
    (*this)(1,5) = ( 0.3598 *  s_q_LH_HAA_);
    (*this)(2,1) = - c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ((- 0.08381 *  c_q_LH_HAA_) -  0.104);
    (*this)(2,4) = (- 0.3598 *  s_q_LH_HAA_);
    (*this)(2,5) = (- 0.3598 *  c_q_LH_HAA_);
    (*this)(4,4) =  s_q_LH_HAA_;
    (*this)(4,5) =  c_q_LH_HAA_;
    (*this)(5,4) = - c_q_LH_HAA_;
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE::Type_fr_base_X_fr_LH_KFE()
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE::update(const JState& q) {
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
    (*this)(0,3) = (((- 0.104 *  c_q_LH_HAA_) -  0.18411) *  s_q_LH_HFE_);
    (*this)(0,4) = (((- 0.104 *  c_q_LH_HAA_) -  0.18411) *  c_q_LH_HFE_);
    (*this)(0,5) = (( 0.285 *  c_q_LH_HFE_) + ( 0.104 *  s_q_LH_HAA_));
    (*this)(1,0) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(1,1) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = ((((- 0.3598 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.18411 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.285 *  c_q_LH_HAA_));
    (*this)(1,4) = (((- 0.18411 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.3598 *  c_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(1,5) = ((( 0.285 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ( 0.3598 *  s_q_LH_HAA_));
    (*this)(2,0) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(2,1) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ((((- 0.3598 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.18411 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.285 *  s_q_LH_HAA_));
    (*this)(2,4) = (((( 0.18411 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.3598 *  s_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(2,5) = (((- 0.285 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - ( 0.3598 *  c_q_LH_HAA_));
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
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA::Type_fr_base_X_fr_RF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.104;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.2999;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0.2999;
    (*this)(2,5) = 0.104;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE::Type_fr_base_X_fr_RF_HFE()
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,4) = (( 0.104 *  c_q_RF_HAA_) +  0.08381);
    (*this)(0,5) = (- 0.104 *  s_q_RF_HAA_);
    (*this)(1,1) =  s_q_RF_HAA_;
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = (- 0.08381 *  s_q_RF_HAA_);
    (*this)(1,4) = ( 0.3598 *  c_q_RF_HAA_);
    (*this)(1,5) = (- 0.3598 *  s_q_RF_HAA_);
    (*this)(2,1) = - c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (( 0.08381 *  c_q_RF_HAA_) +  0.104);
    (*this)(2,4) = ( 0.3598 *  s_q_RF_HAA_);
    (*this)(2,5) = ( 0.3598 *  c_q_RF_HAA_);
    (*this)(4,4) =  s_q_RF_HAA_;
    (*this)(4,5) =  c_q_RF_HAA_;
    (*this)(5,4) = - c_q_RF_HAA_;
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE::Type_fr_base_X_fr_RF_KFE()
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE::update(const JState& q) {
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
    (*this)(0,3) = ((( 0.104 *  c_q_RF_HAA_) +  0.18411) *  s_q_RF_HFE_);
    (*this)(0,4) = ((( 0.104 *  c_q_RF_HAA_) +  0.18411) *  c_q_RF_HFE_);
    (*this)(0,5) = (( 0.285 *  c_q_RF_HFE_) - ( 0.104 *  s_q_RF_HAA_));
    (*this)(1,0) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(1,1) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = (((( 0.3598 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.18411 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.285 *  c_q_RF_HAA_));
    (*this)(1,4) = ((( 0.18411 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.3598 *  c_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(1,5) = ((( 0.285 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3598 *  s_q_RF_HAA_));
    (*this)(2,0) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(2,1) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (((( 0.3598 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.18411 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.285 *  s_q_RF_HAA_));
    (*this)(2,4) = ((((- 0.18411 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.3598 *  s_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(2,5) = (( 0.3598 *  c_q_RF_HAA_) - (( 0.285 *  c_q_RF_HAA_) *  s_q_RF_HFE_));
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
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA::Type_fr_base_X_fr_RH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.104;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.2999;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.2999;
    (*this)(2,5) = 0.104;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE::Type_fr_base_X_fr_RH_HFE()
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,4) = (( 0.104 *  c_q_RH_HAA_) +  0.08381);
    (*this)(0,5) = (- 0.104 *  s_q_RH_HAA_);
    (*this)(1,1) =  s_q_RH_HAA_;
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = (- 0.08381 *  s_q_RH_HAA_);
    (*this)(1,4) = (- 0.3598 *  c_q_RH_HAA_);
    (*this)(1,5) = ( 0.3598 *  s_q_RH_HAA_);
    (*this)(2,1) = - c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (( 0.08381 *  c_q_RH_HAA_) +  0.104);
    (*this)(2,4) = (- 0.3598 *  s_q_RH_HAA_);
    (*this)(2,5) = (- 0.3598 *  c_q_RH_HAA_);
    (*this)(4,4) =  s_q_RH_HAA_;
    (*this)(4,5) =  c_q_RH_HAA_;
    (*this)(5,4) = - c_q_RH_HAA_;
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE::Type_fr_base_X_fr_RH_KFE()
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE::update(const JState& q) {
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
    (*this)(0,3) = ((( 0.104 *  c_q_RH_HAA_) +  0.18411) *  s_q_RH_HFE_);
    (*this)(0,4) = ((( 0.104 *  c_q_RH_HAA_) +  0.18411) *  c_q_RH_HFE_);
    (*this)(0,5) = (( 0.285 *  c_q_RH_HFE_) - ( 0.104 *  s_q_RH_HAA_));
    (*this)(1,0) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(1,1) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = ((((- 0.3598 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.18411 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.285 *  c_q_RH_HAA_));
    (*this)(1,4) = ((( 0.18411 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.3598 *  c_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(1,5) = ((( 0.285 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ( 0.3598 *  s_q_RH_HAA_));
    (*this)(2,0) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(2,1) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = ((((- 0.3598 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.18411 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.285 *  s_q_RH_HAA_));
    (*this)(2,4) = ((((- 0.18411 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.3598 *  s_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(2,5) = (((- 0.285 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - ( 0.3598 *  c_q_RH_HAA_));
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
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base::Type_fr_LF_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.104;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,1) =  s_q_LF_HAA_;
    (*this)(0,2) = - c_q_LF_HAA_;
    (*this)(0,3) = (- 0.104 *  c_q_LF_HAA_);
    (*this)(0,4) = ( 0.2999 *  c_q_LF_HAA_);
    (*this)(0,5) = ( 0.2999 *  s_q_LF_HAA_);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = ( 0.104 *  s_q_LF_HAA_);
    (*this)(1,4) = (- 0.2999 *  s_q_LF_HAA_);
    (*this)(1,5) = ( 0.2999 *  c_q_LF_HAA_);
    (*this)(3,4) =  s_q_LF_HAA_;
    (*this)(3,5) = - c_q_LF_HAA_;
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP::Type_fr_base_X_fr_LF_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = - 0.104;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,3) = (- 0.104 *  c_q_LF_HAA_);
    (*this)(0,4) = ( 0.104 *  s_q_LF_HAA_);
    (*this)(1,0) =  s_q_LF_HAA_;
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,3) = ( 0.2999 *  c_q_LF_HAA_);
    (*this)(1,4) = (- 0.2999 *  s_q_LF_HAA_);
    (*this)(2,0) = - c_q_LF_HAA_;
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,3) = ( 0.2999 *  s_q_LF_HAA_);
    (*this)(2,4) = ( 0.2999 *  c_q_LF_HAA_);
    (*this)(4,3) =  s_q_LF_HAA_;
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(5,3) = - c_q_LF_HAA_;
    (*this)(5,4) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP::Type_fr_LF_THIGH_X_fr_LF_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.0599;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  s_q_LF_HFE_;
    (*this)(0,2) =  c_q_LF_HFE_;
    (*this)(0,3) = ( 0.08381 *  c_q_LF_HFE_);
    (*this)(0,4) = ( 0.0599 *  s_q_LF_HFE_);
    (*this)(0,5) = (- 0.08381 *  s_q_LF_HFE_);
    (*this)(1,0) =  c_q_LF_HFE_;
    (*this)(1,2) = - s_q_LF_HFE_;
    (*this)(1,3) = (- 0.08381 *  s_q_LF_HFE_);
    (*this)(1,4) = ( 0.0599 *  c_q_LF_HFE_);
    (*this)(1,5) = (- 0.08381 *  c_q_LF_HFE_);
    (*this)(3,3) =  s_q_LF_HFE_;
    (*this)(3,5) =  c_q_LF_HFE_;
    (*this)(4,3) =  c_q_LF_HFE_;
    (*this)(4,5) = - s_q_LF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH::Type_fr_LF_HIP_X_fr_LF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = - 0.0599;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  s_q_LF_HFE_;
    (*this)(0,1) =  c_q_LF_HFE_;
    (*this)(0,3) = ( 0.08381 *  c_q_LF_HFE_);
    (*this)(0,4) = (- 0.08381 *  s_q_LF_HFE_);
    (*this)(1,3) = ( 0.0599 *  s_q_LF_HFE_);
    (*this)(1,4) = ( 0.0599 *  c_q_LF_HFE_);
    (*this)(2,0) =  c_q_LF_HFE_;
    (*this)(2,1) = - s_q_LF_HFE_;
    (*this)(2,3) = (- 0.08381 *  s_q_LF_HFE_);
    (*this)(2,4) = (- 0.08381 *  c_q_LF_HFE_);
    (*this)(3,3) =  s_q_LF_HFE_;
    (*this)(3,4) =  c_q_LF_HFE_;
    (*this)(5,3) =  c_q_LF_HFE_;
    (*this)(5,4) = - s_q_LF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH::Type_fr_LF_SHANK_X_fr_LF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.285;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar c_q_LF_KFE_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  c_q_LF_KFE_;
    (*this)(0,1) =  s_q_LF_KFE_;
    (*this)(0,3) = (- 0.1003 *  s_q_LF_KFE_);
    (*this)(0,4) = ( 0.1003 *  c_q_LF_KFE_);
    (*this)(0,5) = (- 0.285 *  c_q_LF_KFE_);
    (*this)(1,0) = - s_q_LF_KFE_;
    (*this)(1,1) =  c_q_LF_KFE_;
    (*this)(1,3) = (- 0.1003 *  c_q_LF_KFE_);
    (*this)(1,4) = (- 0.1003 *  s_q_LF_KFE_);
    (*this)(1,5) = ( 0.285 *  s_q_LF_KFE_);
    (*this)(3,3) =  c_q_LF_KFE_;
    (*this)(3,4) =  s_q_LF_KFE_;
    (*this)(4,3) = - s_q_LF_KFE_;
    (*this)(4,4) =  c_q_LF_KFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK::Type_fr_LF_THIGH_X_fr_LF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0.285;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar c_q_LF_KFE_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  c_q_LF_KFE_;
    (*this)(0,1) = - s_q_LF_KFE_;
    (*this)(0,3) = (- 0.1003 *  s_q_LF_KFE_);
    (*this)(0,4) = (- 0.1003 *  c_q_LF_KFE_);
    (*this)(1,0) =  s_q_LF_KFE_;
    (*this)(1,1) =  c_q_LF_KFE_;
    (*this)(1,3) = ( 0.1003 *  c_q_LF_KFE_);
    (*this)(1,4) = (- 0.1003 *  s_q_LF_KFE_);
    (*this)(2,3) = (- 0.285 *  c_q_LF_KFE_);
    (*this)(2,4) = ( 0.285 *  s_q_LF_KFE_);
    (*this)(3,3) =  c_q_LF_KFE_;
    (*this)(3,4) = - s_q_LF_KFE_;
    (*this)(4,3) =  s_q_LF_KFE_;
    (*this)(4,4) =  c_q_LF_KFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base::Type_fr_RF_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0.104;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,1) =  s_q_RF_HAA_;
    (*this)(0,2) = - c_q_RF_HAA_;
    (*this)(0,3) = ( 0.104 *  c_q_RF_HAA_);
    (*this)(0,4) = ( 0.2999 *  c_q_RF_HAA_);
    (*this)(0,5) = ( 0.2999 *  s_q_RF_HAA_);
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = (- 0.104 *  s_q_RF_HAA_);
    (*this)(1,4) = (- 0.2999 *  s_q_RF_HAA_);
    (*this)(1,5) = ( 0.2999 *  c_q_RF_HAA_);
    (*this)(3,4) =  s_q_RF_HAA_;
    (*this)(3,5) = - c_q_RF_HAA_;
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP::Type_fr_base_X_fr_RF_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0.104;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,3) = ( 0.104 *  c_q_RF_HAA_);
    (*this)(0,4) = (- 0.104 *  s_q_RF_HAA_);
    (*this)(1,0) =  s_q_RF_HAA_;
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,3) = ( 0.2999 *  c_q_RF_HAA_);
    (*this)(1,4) = (- 0.2999 *  s_q_RF_HAA_);
    (*this)(2,0) = - c_q_RF_HAA_;
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,3) = ( 0.2999 *  s_q_RF_HAA_);
    (*this)(2,4) = ( 0.2999 *  c_q_RF_HAA_);
    (*this)(4,3) =  s_q_RF_HAA_;
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(5,3) = - c_q_RF_HAA_;
    (*this)(5,4) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP::Type_fr_RF_THIGH_X_fr_RF_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.0599;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  s_q_RF_HFE_;
    (*this)(0,2) =  c_q_RF_HFE_;
    (*this)(0,3) = (- 0.08381 *  c_q_RF_HFE_);
    (*this)(0,4) = ( 0.0599 *  s_q_RF_HFE_);
    (*this)(0,5) = ( 0.08381 *  s_q_RF_HFE_);
    (*this)(1,0) =  c_q_RF_HFE_;
    (*this)(1,2) = - s_q_RF_HFE_;
    (*this)(1,3) = ( 0.08381 *  s_q_RF_HFE_);
    (*this)(1,4) = ( 0.0599 *  c_q_RF_HFE_);
    (*this)(1,5) = ( 0.08381 *  c_q_RF_HFE_);
    (*this)(3,3) =  s_q_RF_HFE_;
    (*this)(3,5) =  c_q_RF_HFE_;
    (*this)(4,3) =  c_q_RF_HFE_;
    (*this)(4,5) = - s_q_RF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH::Type_fr_RF_HIP_X_fr_RF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = - 0.0599;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  s_q_RF_HFE_;
    (*this)(0,1) =  c_q_RF_HFE_;
    (*this)(0,3) = (- 0.08381 *  c_q_RF_HFE_);
    (*this)(0,4) = ( 0.08381 *  s_q_RF_HFE_);
    (*this)(1,3) = ( 0.0599 *  s_q_RF_HFE_);
    (*this)(1,4) = ( 0.0599 *  c_q_RF_HFE_);
    (*this)(2,0) =  c_q_RF_HFE_;
    (*this)(2,1) = - s_q_RF_HFE_;
    (*this)(2,3) = ( 0.08381 *  s_q_RF_HFE_);
    (*this)(2,4) = ( 0.08381 *  c_q_RF_HFE_);
    (*this)(3,3) =  s_q_RF_HFE_;
    (*this)(3,4) =  c_q_RF_HFE_;
    (*this)(5,3) =  c_q_RF_HFE_;
    (*this)(5,4) = - s_q_RF_HFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH::Type_fr_RF_SHANK_X_fr_RF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.285;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar c_q_RF_KFE_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  c_q_RF_KFE_;
    (*this)(0,1) =  s_q_RF_KFE_;
    (*this)(0,3) = ( 0.1003 *  s_q_RF_KFE_);
    (*this)(0,4) = (- 0.1003 *  c_q_RF_KFE_);
    (*this)(0,5) = (- 0.285 *  c_q_RF_KFE_);
    (*this)(1,0) = - s_q_RF_KFE_;
    (*this)(1,1) =  c_q_RF_KFE_;
    (*this)(1,3) = ( 0.1003 *  c_q_RF_KFE_);
    (*this)(1,4) = ( 0.1003 *  s_q_RF_KFE_);
    (*this)(1,5) = ( 0.285 *  s_q_RF_KFE_);
    (*this)(3,3) =  c_q_RF_KFE_;
    (*this)(3,4) =  s_q_RF_KFE_;
    (*this)(4,3) = - s_q_RF_KFE_;
    (*this)(4,4) =  c_q_RF_KFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK::Type_fr_RF_THIGH_X_fr_RF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0.285;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar c_q_RF_KFE_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  c_q_RF_KFE_;
    (*this)(0,1) = - s_q_RF_KFE_;
    (*this)(0,3) = ( 0.1003 *  s_q_RF_KFE_);
    (*this)(0,4) = ( 0.1003 *  c_q_RF_KFE_);
    (*this)(1,0) =  s_q_RF_KFE_;
    (*this)(1,1) =  c_q_RF_KFE_;
    (*this)(1,3) = (- 0.1003 *  c_q_RF_KFE_);
    (*this)(1,4) = ( 0.1003 *  s_q_RF_KFE_);
    (*this)(2,3) = (- 0.285 *  c_q_RF_KFE_);
    (*this)(2,4) = ( 0.285 *  s_q_RF_KFE_);
    (*this)(3,3) =  c_q_RF_KFE_;
    (*this)(3,4) = - s_q_RF_KFE_;
    (*this)(4,3) =  s_q_RF_KFE_;
    (*this)(4,4) =  c_q_RF_KFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base::Type_fr_LH_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.104;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,1) =  s_q_LH_HAA_;
    (*this)(0,2) = - c_q_LH_HAA_;
    (*this)(0,3) = (- 0.104 *  c_q_LH_HAA_);
    (*this)(0,4) = (- 0.2999 *  c_q_LH_HAA_);
    (*this)(0,5) = (- 0.2999 *  s_q_LH_HAA_);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = ( 0.104 *  s_q_LH_HAA_);
    (*this)(1,4) = ( 0.2999 *  s_q_LH_HAA_);
    (*this)(1,5) = (- 0.2999 *  c_q_LH_HAA_);
    (*this)(3,4) =  s_q_LH_HAA_;
    (*this)(3,5) = - c_q_LH_HAA_;
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP::Type_fr_base_X_fr_LH_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = - 0.104;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,3) = (- 0.104 *  c_q_LH_HAA_);
    (*this)(0,4) = ( 0.104 *  s_q_LH_HAA_);
    (*this)(1,0) =  s_q_LH_HAA_;
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,3) = (- 0.2999 *  c_q_LH_HAA_);
    (*this)(1,4) = ( 0.2999 *  s_q_LH_HAA_);
    (*this)(2,0) = - c_q_LH_HAA_;
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,3) = (- 0.2999 *  s_q_LH_HAA_);
    (*this)(2,4) = (- 0.2999 *  c_q_LH_HAA_);
    (*this)(4,3) =  s_q_LH_HAA_;
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(5,3) = - c_q_LH_HAA_;
    (*this)(5,4) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP::Type_fr_LH_THIGH_X_fr_LH_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.0599;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  s_q_LH_HFE_;
    (*this)(0,2) =  c_q_LH_HFE_;
    (*this)(0,3) = ( 0.08381 *  c_q_LH_HFE_);
    (*this)(0,4) = (- 0.0599 *  s_q_LH_HFE_);
    (*this)(0,5) = (- 0.08381 *  s_q_LH_HFE_);
    (*this)(1,0) =  c_q_LH_HFE_;
    (*this)(1,2) = - s_q_LH_HFE_;
    (*this)(1,3) = (- 0.08381 *  s_q_LH_HFE_);
    (*this)(1,4) = (- 0.0599 *  c_q_LH_HFE_);
    (*this)(1,5) = (- 0.08381 *  c_q_LH_HFE_);
    (*this)(3,3) =  s_q_LH_HFE_;
    (*this)(3,5) =  c_q_LH_HFE_;
    (*this)(4,3) =  c_q_LH_HFE_;
    (*this)(4,5) = - s_q_LH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH::Type_fr_LH_HIP_X_fr_LH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0.0599;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  s_q_LH_HFE_;
    (*this)(0,1) =  c_q_LH_HFE_;
    (*this)(0,3) = ( 0.08381 *  c_q_LH_HFE_);
    (*this)(0,4) = (- 0.08381 *  s_q_LH_HFE_);
    (*this)(1,3) = (- 0.0599 *  s_q_LH_HFE_);
    (*this)(1,4) = (- 0.0599 *  c_q_LH_HFE_);
    (*this)(2,0) =  c_q_LH_HFE_;
    (*this)(2,1) = - s_q_LH_HFE_;
    (*this)(2,3) = (- 0.08381 *  s_q_LH_HFE_);
    (*this)(2,4) = (- 0.08381 *  c_q_LH_HFE_);
    (*this)(3,3) =  s_q_LH_HFE_;
    (*this)(3,4) =  c_q_LH_HFE_;
    (*this)(5,3) =  c_q_LH_HFE_;
    (*this)(5,4) = - s_q_LH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH::Type_fr_LH_SHANK_X_fr_LH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.285;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar c_q_LH_KFE_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  c_q_LH_KFE_;
    (*this)(0,1) =  s_q_LH_KFE_;
    (*this)(0,3) = (- 0.1003 *  s_q_LH_KFE_);
    (*this)(0,4) = ( 0.1003 *  c_q_LH_KFE_);
    (*this)(0,5) = (- 0.285 *  c_q_LH_KFE_);
    (*this)(1,0) = - s_q_LH_KFE_;
    (*this)(1,1) =  c_q_LH_KFE_;
    (*this)(1,3) = (- 0.1003 *  c_q_LH_KFE_);
    (*this)(1,4) = (- 0.1003 *  s_q_LH_KFE_);
    (*this)(1,5) = ( 0.285 *  s_q_LH_KFE_);
    (*this)(3,3) =  c_q_LH_KFE_;
    (*this)(3,4) =  s_q_LH_KFE_;
    (*this)(4,3) = - s_q_LH_KFE_;
    (*this)(4,4) =  c_q_LH_KFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK::Type_fr_LH_THIGH_X_fr_LH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0.285;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar c_q_LH_KFE_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  c_q_LH_KFE_;
    (*this)(0,1) = - s_q_LH_KFE_;
    (*this)(0,3) = (- 0.1003 *  s_q_LH_KFE_);
    (*this)(0,4) = (- 0.1003 *  c_q_LH_KFE_);
    (*this)(1,0) =  s_q_LH_KFE_;
    (*this)(1,1) =  c_q_LH_KFE_;
    (*this)(1,3) = ( 0.1003 *  c_q_LH_KFE_);
    (*this)(1,4) = (- 0.1003 *  s_q_LH_KFE_);
    (*this)(2,3) = (- 0.285 *  c_q_LH_KFE_);
    (*this)(2,4) = ( 0.285 *  s_q_LH_KFE_);
    (*this)(3,3) =  c_q_LH_KFE_;
    (*this)(3,4) = - s_q_LH_KFE_;
    (*this)(4,3) =  s_q_LH_KFE_;
    (*this)(4,4) =  c_q_LH_KFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base::Type_fr_RH_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0.104;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,1) =  s_q_RH_HAA_;
    (*this)(0,2) = - c_q_RH_HAA_;
    (*this)(0,3) = ( 0.104 *  c_q_RH_HAA_);
    (*this)(0,4) = (- 0.2999 *  c_q_RH_HAA_);
    (*this)(0,5) = (- 0.2999 *  s_q_RH_HAA_);
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = (- 0.104 *  s_q_RH_HAA_);
    (*this)(1,4) = ( 0.2999 *  s_q_RH_HAA_);
    (*this)(1,5) = (- 0.2999 *  c_q_RH_HAA_);
    (*this)(3,4) =  s_q_RH_HAA_;
    (*this)(3,5) = - c_q_RH_HAA_;
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP::Type_fr_base_X_fr_RH_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0.104;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,3) = ( 0.104 *  c_q_RH_HAA_);
    (*this)(0,4) = (- 0.104 *  s_q_RH_HAA_);
    (*this)(1,0) =  s_q_RH_HAA_;
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,3) = (- 0.2999 *  c_q_RH_HAA_);
    (*this)(1,4) = ( 0.2999 *  s_q_RH_HAA_);
    (*this)(2,0) = - c_q_RH_HAA_;
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,3) = (- 0.2999 *  s_q_RH_HAA_);
    (*this)(2,4) = (- 0.2999 *  c_q_RH_HAA_);
    (*this)(4,3) =  s_q_RH_HAA_;
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(5,3) = - c_q_RH_HAA_;
    (*this)(5,4) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP::Type_fr_RH_THIGH_X_fr_RH_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.0599;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  s_q_RH_HFE_;
    (*this)(0,2) =  c_q_RH_HFE_;
    (*this)(0,3) = (- 0.08381 *  c_q_RH_HFE_);
    (*this)(0,4) = (- 0.0599 *  s_q_RH_HFE_);
    (*this)(0,5) = ( 0.08381 *  s_q_RH_HFE_);
    (*this)(1,0) =  c_q_RH_HFE_;
    (*this)(1,2) = - s_q_RH_HFE_;
    (*this)(1,3) = ( 0.08381 *  s_q_RH_HFE_);
    (*this)(1,4) = (- 0.0599 *  c_q_RH_HFE_);
    (*this)(1,5) = ( 0.08381 *  c_q_RH_HFE_);
    (*this)(3,3) =  s_q_RH_HFE_;
    (*this)(3,5) =  c_q_RH_HFE_;
    (*this)(4,3) =  c_q_RH_HFE_;
    (*this)(4,5) = - s_q_RH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH::Type_fr_RH_HIP_X_fr_RH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0.0599;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  s_q_RH_HFE_;
    (*this)(0,1) =  c_q_RH_HFE_;
    (*this)(0,3) = (- 0.08381 *  c_q_RH_HFE_);
    (*this)(0,4) = ( 0.08381 *  s_q_RH_HFE_);
    (*this)(1,3) = (- 0.0599 *  s_q_RH_HFE_);
    (*this)(1,4) = (- 0.0599 *  c_q_RH_HFE_);
    (*this)(2,0) =  c_q_RH_HFE_;
    (*this)(2,1) = - s_q_RH_HFE_;
    (*this)(2,3) = ( 0.08381 *  s_q_RH_HFE_);
    (*this)(2,4) = ( 0.08381 *  c_q_RH_HFE_);
    (*this)(3,3) =  s_q_RH_HFE_;
    (*this)(3,4) =  c_q_RH_HFE_;
    (*this)(5,3) =  c_q_RH_HFE_;
    (*this)(5,4) = - s_q_RH_HFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH::Type_fr_RH_SHANK_X_fr_RH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.285;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar c_q_RH_KFE_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  c_q_RH_KFE_;
    (*this)(0,1) =  s_q_RH_KFE_;
    (*this)(0,3) = ( 0.1003 *  s_q_RH_KFE_);
    (*this)(0,4) = (- 0.1003 *  c_q_RH_KFE_);
    (*this)(0,5) = (- 0.285 *  c_q_RH_KFE_);
    (*this)(1,0) = - s_q_RH_KFE_;
    (*this)(1,1) =  c_q_RH_KFE_;
    (*this)(1,3) = ( 0.1003 *  c_q_RH_KFE_);
    (*this)(1,4) = ( 0.1003 *  s_q_RH_KFE_);
    (*this)(1,5) = ( 0.285 *  s_q_RH_KFE_);
    (*this)(3,3) =  c_q_RH_KFE_;
    (*this)(3,4) =  s_q_RH_KFE_;
    (*this)(4,3) = - s_q_RH_KFE_;
    (*this)(4,4) =  c_q_RH_KFE_;
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK::Type_fr_RH_THIGH_X_fr_RH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0.285;
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
const typename iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK& iit::camel::tpl::ForceTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar c_q_RH_KFE_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  c_q_RH_KFE_;
    (*this)(0,1) = - s_q_RH_KFE_;
    (*this)(0,3) = ( 0.1003 *  s_q_RH_KFE_);
    (*this)(0,4) = ( 0.1003 *  c_q_RH_KFE_);
    (*this)(1,0) =  s_q_RH_KFE_;
    (*this)(1,1) =  c_q_RH_KFE_;
    (*this)(1,3) = (- 0.1003 *  c_q_RH_KFE_);
    (*this)(1,4) = ( 0.1003 *  s_q_RH_KFE_);
    (*this)(2,3) = (- 0.285 *  c_q_RH_KFE_);
    (*this)(2,4) = ( 0.285 *  s_q_RH_KFE_);
    (*this)(3,3) =  c_q_RH_KFE_;
    (*this)(3,4) = - s_q_RH_KFE_;
    (*this)(4,3) =  s_q_RH_KFE_;
    (*this)(4,4) =  c_q_RH_KFE_;
    return *this;
}

template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT::Type_fr_base_X_fr_LF_FOOT()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_FOOT::update(const JState& q) {
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
    (*this)(0,3) = ((((((- 0.08795 *  s_q_LF_HFE_) - ( 0.33797 *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.08795 *  c_q_LF_HFE_) - ( 0.33797 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.285 *  s_q_LF_HFE_)) +  0.3598);
    (*this)(1,0) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,3) = (((((((( 0.08795 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.33797 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08795 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.33797 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.285 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.19716 *  c_q_LF_HAA_)) +  0.104);
    (*this)(2,0) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(2,3) = ((((((( 0.33797 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08795 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.08795 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.33797 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.285 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.19716 *  s_q_LF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base::Type_fr_LF_FOOT_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_FOOT_X_fr_base::update(const JState& q) {
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
    (*this)(0,3) = (((((( 0.3598 *  s_q_LF_HFE_) - (( 0.104 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) -  0.285) *  s_q_LF_KFE_) + ((((- 0.104 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - ( 0.3598 *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) -  0.08795);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = ((- 0.104 *  c_q_LF_HAA_) -  0.19716);
    (*this)(2,0) = (( c_q_LF_HFE_ *  s_q_LF_KFE_) + ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(2,1) = ((( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,2) = ((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(2,3) = ((((((- 0.104 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - ( 0.3598 *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.3598 *  s_q_LF_HFE_) + (( 0.104 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) +  0.285) *  c_q_LF_KFE_)) +  0.33797);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT::Type_fr_base_X_fr_LH_FOOT()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_FOOT::update(const JState& q) {
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
    (*this)(0,3) = (((((( 0.08795 *  s_q_LH_HFE_) - ( 0.33797 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((- 0.33797 *  s_q_LH_HFE_) - ( 0.08795 *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.285 *  s_q_LH_HFE_)) -  0.3598);
    (*this)(1,0) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,3) = ((((((((- 0.33797 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08795 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.33797 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08795 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.285 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.19716 *  c_q_LH_HAA_)) +  0.104);
    (*this)(2,0) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(2,3) = ((((((( 0.33797 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08795 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.08795 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.33797 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.285 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.19716 *  s_q_LH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base::Type_fr_LH_FOOT_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_FOOT_X_fr_base::update(const JState& q) {
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
    (*this)(0,3) = ((((((- 0.3598 *  s_q_LH_HFE_) - (( 0.104 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) -  0.285) *  s_q_LH_KFE_) + ((( 0.3598 *  c_q_LH_HFE_) - (( 0.104 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) +  0.08795);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = ((- 0.104 *  c_q_LH_HAA_) -  0.19716);
    (*this)(2,0) = (( c_q_LH_HFE_ *  s_q_LH_KFE_) + ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(2,1) = ((( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,2) = ((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(2,3) = ((((( 0.3598 *  c_q_LH_HFE_) - (( 0.104 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.3598 *  s_q_LH_HFE_) + (( 0.104 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) +  0.285) *  c_q_LH_KFE_)) +  0.33797);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT::Type_fr_base_X_fr_RF_FOOT()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_FOOT::update(const JState& q) {
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
    (*this)(0,3) = ((((((- 0.08795 *  s_q_RF_HFE_) - ( 0.33797 *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.08795 *  c_q_RF_HFE_) - ( 0.33797 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.285 *  s_q_RF_HFE_)) +  0.3598);
    (*this)(1,0) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,3) = (((((((( 0.08795 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.33797 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08795 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.33797 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.285 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.19716 *  c_q_RF_HAA_)) -  0.104);
    (*this)(2,0) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(2,3) = ((((((( 0.33797 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08795 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.08795 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.33797 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.285 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.19716 *  s_q_RF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base::Type_fr_RF_FOOT_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_FOOT_X_fr_base::update(const JState& q) {
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
    (*this)(0,3) = (((((( 0.3598 *  s_q_RF_HFE_) + (( 0.104 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) -  0.285) *  s_q_RF_KFE_) + (((( 0.104 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3598 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) -  0.08795);
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = (( 0.104 *  c_q_RF_HAA_) +  0.19716);
    (*this)(2,0) = (( c_q_RF_HFE_ *  s_q_RF_KFE_) + ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(2,1) = ((( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,2) = ((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(2,3) = (((((( 0.104 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.3598 *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.3598 *  s_q_RF_HFE_) - (( 0.104 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) +  0.285) *  c_q_RF_KFE_)) +  0.33797);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT::Type_fr_base_X_fr_RH_FOOT()
{
    (*this)(0,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_FOOT::update(const JState& q) {
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
    (*this)(0,3) = (((((( 0.08795 *  s_q_RH_HFE_) - ( 0.33797 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((- 0.33797 *  s_q_RH_HFE_) - ( 0.08795 *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.285 *  s_q_RH_HFE_)) -  0.3598);
    (*this)(1,0) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,3) = ((((((((- 0.33797 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08795 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.33797 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08795 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.285 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.19716 *  c_q_RH_HAA_)) -  0.104);
    (*this)(2,0) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(2,3) = ((((((( 0.33797 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08795 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.08795 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.33797 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.285 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.19716 *  s_q_RH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base::Type_fr_RH_FOOT_X_fr_base()
{
    (*this)(1,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_FOOT_X_fr_base::update(const JState& q) {
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
    (*this)(0,3) = ((((((- 0.3598 *  s_q_RH_HFE_) + (( 0.104 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) -  0.285) *  s_q_RH_KFE_) + (((( 0.104 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ( 0.3598 *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) +  0.08795);
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = (( 0.104 *  c_q_RH_HAA_) +  0.19716);
    (*this)(2,0) = (( c_q_RH_HFE_ *  s_q_RH_KFE_) + ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(2,1) = ((( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,2) = ((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(2,3) = (((((( 0.104 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ( 0.3598 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.3598 *  s_q_RH_HFE_) - (( 0.104 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) +  0.285) *  c_q_RH_KFE_)) +  0.33797);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA::Type_fr_base_X_fr_LF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.2999;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.104;
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
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE::Type_fr_base_X_fr_LF_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.3598;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(1,1) =  s_q_LF_HAA_;
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = (( 0.08381 *  c_q_LF_HAA_) +  0.104);
    (*this)(2,1) = - c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = ( 0.08381 *  s_q_LF_HAA_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE::Type_fr_base_X_fr_LF_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_KFE::update(const JState& q) {
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
    (*this)(0,3) = ( 0.3598 - ( 0.285 *  s_q_LF_HFE_));
    (*this)(1,0) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(1,1) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = (((( 0.285 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ( 0.18411 *  c_q_LF_HAA_)) +  0.104);
    (*this)(2,0) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(2,1) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = (( 0.18411 *  s_q_LF_HAA_) - (( 0.285 *  c_q_LF_HAA_) *  c_q_LF_HFE_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA::Type_fr_base_X_fr_LH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.2999;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.104;
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
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE::Type_fr_base_X_fr_LH_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.3598;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(1,1) =  s_q_LH_HAA_;
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = (( 0.08381 *  c_q_LH_HAA_) +  0.104);
    (*this)(2,1) = - c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ( 0.08381 *  s_q_LH_HAA_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE::Type_fr_base_X_fr_LH_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_KFE::update(const JState& q) {
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
    (*this)(0,3) = ((- 0.285 *  s_q_LH_HFE_) -  0.3598);
    (*this)(1,0) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(1,1) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = (((( 0.285 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + ( 0.18411 *  c_q_LH_HAA_)) +  0.104);
    (*this)(2,0) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(2,1) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = (( 0.18411 *  s_q_LH_HAA_) - (( 0.285 *  c_q_LH_HAA_) *  c_q_LH_HFE_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA::Type_fr_base_X_fr_RF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.2999;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.104;
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
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE::Type_fr_base_X_fr_RF_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.3598;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(1,1) =  s_q_RF_HAA_;
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = ((- 0.08381 *  c_q_RF_HAA_) -  0.104);
    (*this)(2,1) = - c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (- 0.08381 *  s_q_RF_HAA_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE::Type_fr_base_X_fr_RF_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_KFE::update(const JState& q) {
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
    (*this)(0,3) = ( 0.3598 - ( 0.285 *  s_q_RF_HFE_));
    (*this)(1,0) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(1,1) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = (((( 0.285 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - ( 0.18411 *  c_q_RF_HAA_)) -  0.104);
    (*this)(2,0) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(2,1) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (((- 0.285 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - ( 0.18411 *  s_q_RF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA::Type_fr_base_X_fr_RH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.2999;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.104;
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
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE::Type_fr_base_X_fr_RH_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.3598;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(1,1) =  s_q_RH_HAA_;
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = ((- 0.08381 *  c_q_RH_HAA_) -  0.104);
    (*this)(2,1) = - c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (- 0.08381 *  s_q_RH_HAA_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE::Type_fr_base_X_fr_RH_KFE()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_KFE::update(const JState& q) {
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
    (*this)(0,3) = ((- 0.285 *  s_q_RH_HFE_) -  0.3598);
    (*this)(1,0) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(1,1) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = (((( 0.285 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - ( 0.18411 *  c_q_RH_HAA_)) -  0.104);
    (*this)(2,0) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(2,1) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (((- 0.285 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - ( 0.18411 *  s_q_RH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base::Type_fr_LF_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.2999;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,1) =  s_q_LF_HAA_;
    (*this)(0,2) = - c_q_LF_HAA_;
    (*this)(0,3) = (- 0.104 *  s_q_LF_HAA_);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = (- 0.104 *  c_q_LF_HAA_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP::Type_fr_base_X_fr_LF_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.2999;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.104;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP::update(const JState& q) {
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
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP::Type_fr_LF_THIGH_X_fr_LF_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.08381;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP::update(const JState& q) {
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HFE_;
    
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    
    (*this)(0,0) =  s_q_LF_HFE_;
    (*this)(0,2) =  c_q_LF_HFE_;
    (*this)(0,3) = (- 0.0599 *  c_q_LF_HFE_);
    (*this)(1,0) =  c_q_LF_HFE_;
    (*this)(1,2) = - s_q_LF_HFE_;
    (*this)(1,3) = ( 0.0599 *  s_q_LF_HFE_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH::Type_fr_LF_HIP_X_fr_LF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.08381;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.0599;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_LF_THIGH::update(const JState& q) {
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
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH::Type_fr_LF_SHANK_X_fr_LF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.1003;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_THIGH::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar c_q_LF_KFE_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    
    (*this)(0,0) =  c_q_LF_KFE_;
    (*this)(0,1) =  s_q_LF_KFE_;
    (*this)(0,3) = (- 0.285 *  s_q_LF_KFE_);
    (*this)(1,0) = - s_q_LF_KFE_;
    (*this)(1,1) =  c_q_LF_KFE_;
    (*this)(1,3) = (- 0.285 *  c_q_LF_KFE_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK::Type_fr_LF_THIGH_X_fr_LF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.285;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.1003;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK::update(const JState& q) {
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
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base::Type_fr_RF_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.2999;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,1) =  s_q_RF_HAA_;
    (*this)(0,2) = - c_q_RF_HAA_;
    (*this)(0,3) = ( 0.104 *  s_q_RF_HAA_);
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = ( 0.104 *  c_q_RF_HAA_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP::Type_fr_base_X_fr_RF_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.2999;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.104;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP::update(const JState& q) {
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
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP::Type_fr_RF_THIGH_X_fr_RF_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.08381;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP::update(const JState& q) {
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HFE_;
    
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    
    (*this)(0,0) =  s_q_RF_HFE_;
    (*this)(0,2) =  c_q_RF_HFE_;
    (*this)(0,3) = (- 0.0599 *  c_q_RF_HFE_);
    (*this)(1,0) =  c_q_RF_HFE_;
    (*this)(1,2) = - s_q_RF_HFE_;
    (*this)(1,3) = ( 0.0599 *  s_q_RF_HFE_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH::Type_fr_RF_HIP_X_fr_RF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = - 0.08381;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.0599;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_RF_THIGH::update(const JState& q) {
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
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH::Type_fr_RF_SHANK_X_fr_RF_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.1003;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_THIGH::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar c_q_RF_KFE_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    
    (*this)(0,0) =  c_q_RF_KFE_;
    (*this)(0,1) =  s_q_RF_KFE_;
    (*this)(0,3) = (- 0.285 *  s_q_RF_KFE_);
    (*this)(1,0) = - s_q_RF_KFE_;
    (*this)(1,1) =  c_q_RF_KFE_;
    (*this)(1,3) = (- 0.285 *  c_q_RF_KFE_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK::Type_fr_RF_THIGH_X_fr_RF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.285;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.1003;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK::update(const JState& q) {
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
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base::Type_fr_LH_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.2999;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,1) =  s_q_LH_HAA_;
    (*this)(0,2) = - c_q_LH_HAA_;
    (*this)(0,3) = (- 0.104 *  s_q_LH_HAA_);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = (- 0.104 *  c_q_LH_HAA_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP::Type_fr_base_X_fr_LH_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.2999;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.104;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP::update(const JState& q) {
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
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP::Type_fr_LH_THIGH_X_fr_LH_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.08381;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP::update(const JState& q) {
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HFE_;
    
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    
    (*this)(0,0) =  s_q_LH_HFE_;
    (*this)(0,2) =  c_q_LH_HFE_;
    (*this)(0,3) = ( 0.0599 *  c_q_LH_HFE_);
    (*this)(1,0) =  c_q_LH_HFE_;
    (*this)(1,2) = - s_q_LH_HFE_;
    (*this)(1,3) = (- 0.0599 *  s_q_LH_HFE_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH::Type_fr_LH_HIP_X_fr_LH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.08381;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.0599;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_LH_THIGH::update(const JState& q) {
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
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH::Type_fr_LH_SHANK_X_fr_LH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.1003;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_THIGH::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar c_q_LH_KFE_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    
    (*this)(0,0) =  c_q_LH_KFE_;
    (*this)(0,1) =  s_q_LH_KFE_;
    (*this)(0,3) = (- 0.285 *  s_q_LH_KFE_);
    (*this)(1,0) = - s_q_LH_KFE_;
    (*this)(1,1) =  c_q_LH_KFE_;
    (*this)(1,3) = (- 0.285 *  c_q_LH_KFE_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK::Type_fr_LH_THIGH_X_fr_LH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.285;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.1003;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK::update(const JState& q) {
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
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base::Type_fr_RH_HIP_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.2999;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,1) =  s_q_RH_HAA_;
    (*this)(0,2) = - c_q_RH_HAA_;
    (*this)(0,3) = ( 0.104 *  s_q_RH_HAA_);
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = ( 0.104 *  c_q_RH_HAA_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP::Type_fr_base_X_fr_RH_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.2999;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.104;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP::update(const JState& q) {
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
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP::Type_fr_RH_THIGH_X_fr_RH_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.08381;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP::update(const JState& q) {
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HFE_;
    
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    
    (*this)(0,0) =  s_q_RH_HFE_;
    (*this)(0,2) =  c_q_RH_HFE_;
    (*this)(0,3) = ( 0.0599 *  c_q_RH_HFE_);
    (*this)(1,0) =  c_q_RH_HFE_;
    (*this)(1,2) = - s_q_RH_HFE_;
    (*this)(1,3) = (- 0.0599 *  s_q_RH_HFE_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH::Type_fr_RH_HIP_X_fr_RH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = - 0.08381;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.0599;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_RH_THIGH::update(const JState& q) {
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
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH::Type_fr_RH_SHANK_X_fr_RH_THIGH()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.1003;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_THIGH::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar c_q_RH_KFE_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    
    (*this)(0,0) =  c_q_RH_KFE_;
    (*this)(0,1) =  s_q_RH_KFE_;
    (*this)(0,3) = (- 0.285 *  s_q_RH_KFE_);
    (*this)(1,0) = - s_q_RH_KFE_;
    (*this)(1,1) =  c_q_RH_KFE_;
    (*this)(1,3) = (- 0.285 *  c_q_RH_KFE_);
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK::Type_fr_RH_THIGH_X_fr_RH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.285;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.1003;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK& iit::camel::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK::update(const JState& q) {
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

