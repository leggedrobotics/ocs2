
// Constructors
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::MotionTransforms
    ()
     :
    fr_base_X_fr_LF_WHEEL_L(),
    fr_LF_WHEEL_L_X_fr_base(),
    fr_base_X_fr_LH_WHEEL_L(),
    fr_LH_WHEEL_L_X_fr_base(),
    fr_base_X_fr_RF_WHEEL_L(),
    fr_RF_WHEEL_L_X_fr_base(),
    fr_base_X_fr_RH_WHEEL_L(),
    fr_RH_WHEEL_L_X_fr_base(),
    fr_base_X_fr_LF_HAA(),
    fr_base_X_fr_LF_HFE(),
    fr_base_X_fr_LF_KFE(),
    fr_base_X_fr_LF_WHEEL(),
    fr_base_X_fr_LH_HAA(),
    fr_base_X_fr_LH_HFE(),
    fr_base_X_fr_LH_KFE(),
    fr_base_X_fr_LH_WHEEL(),
    fr_base_X_fr_RF_HAA(),
    fr_base_X_fr_RF_HFE(),
    fr_base_X_fr_RF_KFE(),
    fr_base_X_fr_RF_WHEEL(),
    fr_base_X_fr_RH_HAA(),
    fr_base_X_fr_RH_HFE(),
    fr_base_X_fr_RH_KFE(),
    fr_base_X_fr_RH_WHEEL(),
    fr_LF_HIP_X_fr_base(),
    fr_base_X_fr_LF_HIP(),
    fr_LF_THIGH_X_fr_LF_HIP(),
    fr_LF_HIP_X_fr_LF_THIGH(),
    fr_LF_SHANK_X_fr_LF_THIGH(),
    fr_LF_THIGH_X_fr_LF_SHANK(),
    fr_LF_WHEEL_L_X_fr_LF_SHANK(),
    fr_LF_SHANK_X_fr_LF_WHEEL_L(),
    fr_RF_HIP_X_fr_base(),
    fr_base_X_fr_RF_HIP(),
    fr_RF_THIGH_X_fr_RF_HIP(),
    fr_RF_HIP_X_fr_RF_THIGH(),
    fr_RF_SHANK_X_fr_RF_THIGH(),
    fr_RF_THIGH_X_fr_RF_SHANK(),
    fr_RF_WHEEL_L_X_fr_RF_SHANK(),
    fr_RF_SHANK_X_fr_RF_WHEEL_L(),
    fr_LH_HIP_X_fr_base(),
    fr_base_X_fr_LH_HIP(),
    fr_LH_THIGH_X_fr_LH_HIP(),
    fr_LH_HIP_X_fr_LH_THIGH(),
    fr_LH_SHANK_X_fr_LH_THIGH(),
    fr_LH_THIGH_X_fr_LH_SHANK(),
    fr_LH_WHEEL_L_X_fr_LH_SHANK(),
    fr_LH_SHANK_X_fr_LH_WHEEL_L(),
    fr_RH_HIP_X_fr_base(),
    fr_base_X_fr_RH_HIP(),
    fr_RH_THIGH_X_fr_RH_HIP(),
    fr_RH_HIP_X_fr_RH_THIGH(),
    fr_RH_SHANK_X_fr_RH_THIGH(),
    fr_RH_THIGH_X_fr_RH_SHANK(),
    fr_RH_WHEEL_L_X_fr_RH_SHANK(),
    fr_RH_SHANK_X_fr_RH_WHEEL_L()
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
    fr_base_X_fr_LF_WHEEL_L(),
    fr_LF_WHEEL_L_X_fr_base(),
    fr_base_X_fr_LH_WHEEL_L(),
    fr_LH_WHEEL_L_X_fr_base(),
    fr_base_X_fr_RF_WHEEL_L(),
    fr_RF_WHEEL_L_X_fr_base(),
    fr_base_X_fr_RH_WHEEL_L(),
    fr_RH_WHEEL_L_X_fr_base(),
    fr_base_X_fr_LF_HAA(),
    fr_base_X_fr_LF_HFE(),
    fr_base_X_fr_LF_KFE(),
    fr_base_X_fr_LF_WHEEL(),
    fr_base_X_fr_LH_HAA(),
    fr_base_X_fr_LH_HFE(),
    fr_base_X_fr_LH_KFE(),
    fr_base_X_fr_LH_WHEEL(),
    fr_base_X_fr_RF_HAA(),
    fr_base_X_fr_RF_HFE(),
    fr_base_X_fr_RF_KFE(),
    fr_base_X_fr_RF_WHEEL(),
    fr_base_X_fr_RH_HAA(),
    fr_base_X_fr_RH_HFE(),
    fr_base_X_fr_RH_KFE(),
    fr_base_X_fr_RH_WHEEL(),
    fr_LF_HIP_X_fr_base(),
    fr_base_X_fr_LF_HIP(),
    fr_LF_THIGH_X_fr_LF_HIP(),
    fr_LF_HIP_X_fr_LF_THIGH(),
    fr_LF_SHANK_X_fr_LF_THIGH(),
    fr_LF_THIGH_X_fr_LF_SHANK(),
    fr_LF_WHEEL_L_X_fr_LF_SHANK(),
    fr_LF_SHANK_X_fr_LF_WHEEL_L(),
    fr_RF_HIP_X_fr_base(),
    fr_base_X_fr_RF_HIP(),
    fr_RF_THIGH_X_fr_RF_HIP(),
    fr_RF_HIP_X_fr_RF_THIGH(),
    fr_RF_SHANK_X_fr_RF_THIGH(),
    fr_RF_THIGH_X_fr_RF_SHANK(),
    fr_RF_WHEEL_L_X_fr_RF_SHANK(),
    fr_RF_SHANK_X_fr_RF_WHEEL_L(),
    fr_LH_HIP_X_fr_base(),
    fr_base_X_fr_LH_HIP(),
    fr_LH_THIGH_X_fr_LH_HIP(),
    fr_LH_HIP_X_fr_LH_THIGH(),
    fr_LH_SHANK_X_fr_LH_THIGH(),
    fr_LH_THIGH_X_fr_LH_SHANK(),
    fr_LH_WHEEL_L_X_fr_LH_SHANK(),
    fr_LH_SHANK_X_fr_LH_WHEEL_L(),
    fr_RH_HIP_X_fr_base(),
    fr_base_X_fr_RH_HIP(),
    fr_RH_THIGH_X_fr_RH_HIP(),
    fr_RH_HIP_X_fr_RH_THIGH(),
    fr_RH_SHANK_X_fr_RH_THIGH(),
    fr_RH_THIGH_X_fr_RH_SHANK(),
    fr_RH_WHEEL_L_X_fr_RH_SHANK(),
    fr_RH_SHANK_X_fr_RH_WHEEL_L()
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
    fr_base_X_fr_LF_WHEEL_L(),
    fr_LF_WHEEL_L_X_fr_base(),
    fr_base_X_fr_LH_WHEEL_L(),
    fr_LH_WHEEL_L_X_fr_base(),
    fr_base_X_fr_RF_WHEEL_L(),
    fr_RF_WHEEL_L_X_fr_base(),
    fr_base_X_fr_RH_WHEEL_L(),
    fr_RH_WHEEL_L_X_fr_base(),
    fr_base_X_fr_LF_HAA(),
    fr_base_X_fr_LF_HFE(),
    fr_base_X_fr_LF_KFE(),
    fr_base_X_fr_LF_WHEEL(),
    fr_base_X_fr_LH_HAA(),
    fr_base_X_fr_LH_HFE(),
    fr_base_X_fr_LH_KFE(),
    fr_base_X_fr_LH_WHEEL(),
    fr_base_X_fr_RF_HAA(),
    fr_base_X_fr_RF_HFE(),
    fr_base_X_fr_RF_KFE(),
    fr_base_X_fr_RF_WHEEL(),
    fr_base_X_fr_RH_HAA(),
    fr_base_X_fr_RH_HFE(),
    fr_base_X_fr_RH_KFE(),
    fr_base_X_fr_RH_WHEEL(),
    fr_LF_HIP_X_fr_base(),
    fr_base_X_fr_LF_HIP(),
    fr_LF_THIGH_X_fr_LF_HIP(),
    fr_LF_HIP_X_fr_LF_THIGH(),
    fr_LF_SHANK_X_fr_LF_THIGH(),
    fr_LF_THIGH_X_fr_LF_SHANK(),
    fr_LF_WHEEL_L_X_fr_LF_SHANK(),
    fr_LF_SHANK_X_fr_LF_WHEEL_L(),
    fr_RF_HIP_X_fr_base(),
    fr_base_X_fr_RF_HIP(),
    fr_RF_THIGH_X_fr_RF_HIP(),
    fr_RF_HIP_X_fr_RF_THIGH(),
    fr_RF_SHANK_X_fr_RF_THIGH(),
    fr_RF_THIGH_X_fr_RF_SHANK(),
    fr_RF_WHEEL_L_X_fr_RF_SHANK(),
    fr_RF_SHANK_X_fr_RF_WHEEL_L(),
    fr_LH_HIP_X_fr_base(),
    fr_base_X_fr_LH_HIP(),
    fr_LH_THIGH_X_fr_LH_HIP(),
    fr_LH_HIP_X_fr_LH_THIGH(),
    fr_LH_SHANK_X_fr_LH_THIGH(),
    fr_LH_THIGH_X_fr_LH_SHANK(),
    fr_LH_WHEEL_L_X_fr_LH_SHANK(),
    fr_LH_SHANK_X_fr_LH_WHEEL_L(),
    fr_RH_HIP_X_fr_base(),
    fr_base_X_fr_RH_HIP(),
    fr_RH_THIGH_X_fr_RH_HIP(),
    fr_RH_HIP_X_fr_RH_THIGH(),
    fr_RH_SHANK_X_fr_RH_THIGH(),
    fr_RH_THIGH_X_fr_RH_SHANK(),
    fr_RH_WHEEL_L_X_fr_RH_SHANK(),
    fr_RH_SHANK_X_fr_RH_WHEEL_L()
{
    updateParameters();
}
template <typename TRAIT>
void iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL_L::Type_fr_base_X_fr_LF_WHEEL_L()
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
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL_L& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL_L::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_WHEEL_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_WHEEL_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_WHEEL_ = TRAIT::sin( q(LF_WHEEL));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_WHEEL_ = TRAIT::cos( q(LF_WHEEL));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(0,1) = (((( s_q_LF_HFE_ *  s_q_LF_KFE_) - ( c_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,0) = ((((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,1) = (((((- s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(2,0) = ((((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(2,1) = ((((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(3,0) = ((((((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(3,1) = ((((((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(3,2) = (((((( 0.08748 *  c_q_LF_HFE_) - ( 0.248 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.08748 *  s_q_LF_HFE_) + ( 0.248 *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.28263 *  c_q_LF_HFE_)) + ( 0.104 *  s_q_LF_HAA_));
    (*this)(3,3) = ((((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(3,4) = (((( s_q_LF_HFE_ *  s_q_LF_KFE_) - ( c_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(4,0) = (((((((((- 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.28263 *  c_q_LF_HAA_)) *  s_q_LF_KFE_) + (((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  c_q_LF_HAA_)) *  s_q_LF_WHEEL_) + ((((((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.28263 *  c_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.248 *  c_q_LF_HAA_)) *  c_q_LF_WHEEL_));
    (*this)(4,1) = (((((((( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.28263 *  c_q_LF_HAA_)) *  c_q_LF_KFE_)) + ( 0.248 *  c_q_LF_HAA_)) *  s_q_LF_WHEEL_) + ((((((((- 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.28263 *  c_q_LF_HAA_)) *  s_q_LF_KFE_) + (((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  c_q_LF_HAA_)) *  c_q_LF_WHEEL_));
    (*this)(4,2) = ((((((( 0.08748 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.248 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.248 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08748 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.28263 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.36 *  s_q_LF_HAA_));
    (*this)(4,3) = ((((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(4,4) = (((((- s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(4,5) =  c_q_LF_HAA_;
    (*this)(5,0) = (((((((((- 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  c_q_LF_HFE_)) + ( 0.28263 *  s_q_LF_HAA_)) *  s_q_LF_KFE_) + ((((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  s_q_LF_HAA_)) *  s_q_LF_WHEEL_) + (((((((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.18979 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.28263 *  s_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.248 *  s_q_LF_HAA_)) *  c_q_LF_WHEEL_));
    (*this)(5,1) = (((((((((- 0.18979 *  c_q_LF_HAA_) -  0.104) *  s_q_LF_HFE_) - (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  c_q_LF_HFE_)) + ( 0.28263 *  s_q_LF_HAA_)) *  c_q_LF_KFE_)) + ( 0.248 *  s_q_LF_HAA_)) *  s_q_LF_WHEEL_) + ((((((((- 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  c_q_LF_HFE_)) + ( 0.28263 *  s_q_LF_HAA_)) *  s_q_LF_KFE_) + ((((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  s_q_LF_HAA_)) *  c_q_LF_WHEEL_));
    (*this)(5,2) = (((((((- 0.08748 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.248 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08748 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.248 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.28263 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.36 *  c_q_LF_HAA_));
    (*this)(5,3) = ((((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(5,4) = ((((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_base::Type_fr_LF_WHEEL_L_X_fr_base()
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
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_WHEEL_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_WHEEL_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_WHEEL_ = TRAIT::sin( q(LF_WHEEL));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_WHEEL_ = TRAIT::cos( q(LF_WHEEL));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( 1.0 *  c_q_LF_HFE_) *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(0,1) = (((((( 1.0 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((( 1.0 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(0,2) = (((((( 1.0 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,0) = ((((( 1.0 *  s_q_LF_HFE_) *  s_q_LF_KFE_) - ( c_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,1) = (((((- s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((( 1.0 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,2) = (((((( 1.0 *  c_q_LF_HAA_) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((( 1.0 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(2,1) =  c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(3,0) = ((((((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(3,1) = (((((((((- 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.28263 *  c_q_LF_HAA_)) *  s_q_LF_KFE_) + (((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  c_q_LF_HAA_)) *  s_q_LF_WHEEL_) + ((((((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.28263 *  c_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.248 *  c_q_LF_HAA_)) *  c_q_LF_WHEEL_));
    (*this)(3,2) = (((((((((- 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  c_q_LF_HFE_)) + ( 0.28263 *  s_q_LF_HAA_)) *  s_q_LF_KFE_) + ((((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  s_q_LF_HAA_)) *  s_q_LF_WHEEL_) + (((((((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.18979 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.28263 *  s_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.248 *  s_q_LF_HAA_)) *  c_q_LF_WHEEL_));
    (*this)(3,3) = ((((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( 1.0 *  c_q_LF_HFE_) *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(3,4) = (((((( 1.0 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((( 1.0 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(3,5) = (((((( 1.0 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(4,0) = ((((((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(4,1) = (((((((( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.28263 *  c_q_LF_HAA_)) *  c_q_LF_KFE_)) + ( 0.248 *  c_q_LF_HAA_)) *  s_q_LF_WHEEL_) + ((((((((- 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.28263 *  c_q_LF_HAA_)) *  s_q_LF_KFE_) + (((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  c_q_LF_HAA_)) *  c_q_LF_WHEEL_));
    (*this)(4,2) = (((((((((- 0.18979 *  c_q_LF_HAA_) -  0.104) *  s_q_LF_HFE_) - (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  c_q_LF_HFE_)) + ( 0.28263 *  s_q_LF_HAA_)) *  c_q_LF_KFE_)) + ( 0.248 *  s_q_LF_HAA_)) *  s_q_LF_WHEEL_) + ((((((((- 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  c_q_LF_HFE_)) + ( 0.28263 *  s_q_LF_HAA_)) *  s_q_LF_KFE_) + ((((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  s_q_LF_HAA_)) *  c_q_LF_WHEEL_));
    (*this)(4,3) = ((((( 1.0 *  s_q_LF_HFE_) *  s_q_LF_KFE_) - ( c_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(4,4) = (((((- s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((( 1.0 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(4,5) = (((((( 1.0 *  c_q_LF_HAA_) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((( 1.0 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(5,0) = (((((( 0.08748 *  c_q_LF_HFE_) - ( 0.248 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.08748 *  s_q_LF_HFE_) + ( 0.248 *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.28263 *  c_q_LF_HFE_)) + ( 0.104 *  s_q_LF_HAA_));
    (*this)(5,1) = ((((((( 0.08748 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.248 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.248 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08748 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.28263 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.36 *  s_q_LF_HAA_));
    (*this)(5,2) = (((((((- 0.08748 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.248 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08748 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.248 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.28263 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.36 *  c_q_LF_HAA_));
    (*this)(5,4) =  c_q_LF_HAA_;
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL_L::Type_fr_base_X_fr_LH_WHEEL_L()
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
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL_L& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL_L::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_WHEEL_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_WHEEL_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_WHEEL_ = TRAIT::sin( q(LH_WHEEL));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_WHEEL_ = TRAIT::cos( q(LH_WHEEL));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(0,1) = (((( s_q_LH_HFE_ *  s_q_LH_KFE_) - ( c_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,0) = ((((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,1) = (((((- s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(2,0) = ((((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(2,1) = ((((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(3,0) = ((((((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(3,1) = ((((((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(3,2) = ((((((- 0.248 *  s_q_LH_HFE_) - ( 0.08748 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.248 *  c_q_LH_HFE_) - ( 0.08748 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.28263 *  c_q_LH_HFE_)) + ( 0.104 *  s_q_LH_HAA_));
    (*this)(3,3) = ((((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(3,4) = (((( s_q_LH_HFE_ *  s_q_LH_KFE_) - ( c_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(4,0) = ((((((((( 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.28263 *  c_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((- 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  c_q_LH_HAA_)) *  s_q_LH_WHEEL_) + (((((((- 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.28263 *  c_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.248 *  c_q_LH_HAA_)) *  c_q_LH_WHEEL_));
    (*this)(4,1) = (((((((( 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((( 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.28263 *  c_q_LH_HAA_)) *  c_q_LH_KFE_)) + ( 0.248 *  c_q_LH_HAA_)) *  s_q_LH_WHEEL_) + (((((((( 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.28263 *  c_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((- 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  c_q_LH_HAA_)) *  c_q_LH_WHEEL_));
    (*this)(4,2) = ((((((( 0.248 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08748 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.248 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08748 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.28263 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.36 *  s_q_LH_HAA_));
    (*this)(4,3) = ((((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(4,4) = (((((- s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(4,5) =  c_q_LH_HAA_;
    (*this)(5,0) = ((((((((( 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  c_q_LH_HFE_)) + ( 0.28263 *  s_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  s_q_LH_HAA_)) *  s_q_LH_WHEEL_) + (((((((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.18979 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.28263 *  s_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.248 *  s_q_LH_HAA_)) *  c_q_LH_WHEEL_));
    (*this)(5,1) = (((((((((- 0.18979 *  c_q_LH_HAA_) -  0.104) *  s_q_LH_HFE_) + (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((( 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  c_q_LH_HFE_)) + ( 0.28263 *  s_q_LH_HAA_)) *  c_q_LH_KFE_)) + ( 0.248 *  s_q_LH_HAA_)) *  s_q_LH_WHEEL_) + (((((((( 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  c_q_LH_HFE_)) + ( 0.28263 *  s_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  s_q_LH_HAA_)) *  c_q_LH_WHEEL_));
    (*this)(5,2) = ((((((( 0.08748 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.248 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.248 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08748 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.28263 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.36 *  c_q_LH_HAA_));
    (*this)(5,3) = ((((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(5,4) = ((((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_base::Type_fr_LH_WHEEL_L_X_fr_base()
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
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_WHEEL_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_WHEEL_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_WHEEL_ = TRAIT::sin( q(LH_WHEEL));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_WHEEL_ = TRAIT::cos( q(LH_WHEEL));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( 1.0 *  c_q_LH_HFE_) *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(0,1) = (((((( 1.0 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((( 1.0 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(0,2) = (((((( 1.0 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,0) = ((((( 1.0 *  s_q_LH_HFE_) *  s_q_LH_KFE_) - ( c_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,1) = (((((- s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((( 1.0 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,2) = (((((( 1.0 *  c_q_LH_HAA_) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((( 1.0 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(2,1) =  c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(3,0) = ((((((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(3,1) = ((((((((( 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.28263 *  c_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((- 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  c_q_LH_HAA_)) *  s_q_LH_WHEEL_) + (((((((- 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.28263 *  c_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.248 *  c_q_LH_HAA_)) *  c_q_LH_WHEEL_));
    (*this)(3,2) = ((((((((( 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  c_q_LH_HFE_)) + ( 0.28263 *  s_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  s_q_LH_HAA_)) *  s_q_LH_WHEEL_) + (((((((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.18979 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.28263 *  s_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.248 *  s_q_LH_HAA_)) *  c_q_LH_WHEEL_));
    (*this)(3,3) = ((((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( 1.0 *  c_q_LH_HFE_) *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(3,4) = (((((( 1.0 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((( 1.0 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(3,5) = (((((( 1.0 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(4,0) = ((((((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(4,1) = (((((((( 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((( 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.28263 *  c_q_LH_HAA_)) *  c_q_LH_KFE_)) + ( 0.248 *  c_q_LH_HAA_)) *  s_q_LH_WHEEL_) + (((((((( 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.28263 *  c_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((- 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  c_q_LH_HAA_)) *  c_q_LH_WHEEL_));
    (*this)(4,2) = (((((((((- 0.18979 *  c_q_LH_HAA_) -  0.104) *  s_q_LH_HFE_) + (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((( 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  c_q_LH_HFE_)) + ( 0.28263 *  s_q_LH_HAA_)) *  c_q_LH_KFE_)) + ( 0.248 *  s_q_LH_HAA_)) *  s_q_LH_WHEEL_) + (((((((( 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  c_q_LH_HFE_)) + ( 0.28263 *  s_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  s_q_LH_HAA_)) *  c_q_LH_WHEEL_));
    (*this)(4,3) = ((((( 1.0 *  s_q_LH_HFE_) *  s_q_LH_KFE_) - ( c_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(4,4) = (((((- s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((( 1.0 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(4,5) = (((((( 1.0 *  c_q_LH_HAA_) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((( 1.0 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(5,0) = ((((((- 0.248 *  s_q_LH_HFE_) - ( 0.08748 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.248 *  c_q_LH_HFE_) - ( 0.08748 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.28263 *  c_q_LH_HFE_)) + ( 0.104 *  s_q_LH_HAA_));
    (*this)(5,1) = ((((((( 0.248 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08748 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.248 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08748 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.28263 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.36 *  s_q_LH_HAA_));
    (*this)(5,2) = ((((((( 0.08748 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.248 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.248 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08748 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.28263 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.36 *  c_q_LH_HAA_));
    (*this)(5,4) =  c_q_LH_HAA_;
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL_L::Type_fr_base_X_fr_RF_WHEEL_L()
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
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL_L& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL_L::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_WHEEL_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_WHEEL_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_WHEEL_ = TRAIT::sin( q(RF_WHEEL));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_WHEEL_ = TRAIT::cos( q(RF_WHEEL));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(0,1) = (((( s_q_RF_HFE_ *  s_q_RF_KFE_) - ( c_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,0) = ((((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,1) = (((((- s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(2,0) = ((((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(2,1) = ((((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(3,0) = (((((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(3,1) = (((((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(3,2) = (((((( 0.08748 *  c_q_RF_HFE_) - ( 0.248 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.08748 *  s_q_RF_HFE_) + ( 0.248 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.28263 *  c_q_RF_HFE_)) - ( 0.104 *  s_q_RF_HAA_));
    (*this)(3,3) = ((((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(3,4) = (((( s_q_RF_HFE_ *  s_q_RF_KFE_) - ( c_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(4,0) = (((((((((- 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.28263 *  c_q_RF_HAA_)) *  s_q_RF_KFE_) + (((( 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  c_q_RF_HAA_)) *  s_q_RF_WHEEL_) + ((((((( 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.28263 *  c_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.248 *  c_q_RF_HAA_)) *  c_q_RF_WHEEL_));
    (*this)(4,1) = ((((((((- 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((((- 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.28263 *  c_q_RF_HAA_)) *  c_q_RF_KFE_)) + ( 0.248 *  c_q_RF_HAA_)) *  s_q_RF_WHEEL_) + ((((((((- 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.28263 *  c_q_RF_HAA_)) *  s_q_RF_KFE_) + (((( 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  c_q_RF_HAA_)) *  c_q_RF_WHEEL_));
    (*this)(4,2) = ((((((( 0.08748 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.248 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.248 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08748 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.28263 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.36 *  s_q_RF_HAA_));
    (*this)(4,3) = ((((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(4,4) = (((((- s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(4,5) =  c_q_RF_HAA_;
    (*this)(5,0) = (((((((((- 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  c_q_RF_HFE_)) + ( 0.28263 *  s_q_RF_HAA_)) *  s_q_RF_KFE_) + (((((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  s_q_RF_HAA_)) *  s_q_RF_WHEEL_) + ((((((((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.18979 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.28263 *  s_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.248 *  s_q_RF_HAA_)) *  c_q_RF_WHEEL_));
    (*this)(5,1) = ((((((((( 0.18979 *  c_q_RF_HAA_) +  0.104) *  s_q_RF_HFE_) - (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((((- 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  c_q_RF_HFE_)) + ( 0.28263 *  s_q_RF_HAA_)) *  c_q_RF_KFE_)) + ( 0.248 *  s_q_RF_HAA_)) *  s_q_RF_WHEEL_) + ((((((((- 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  c_q_RF_HFE_)) + ( 0.28263 *  s_q_RF_HAA_)) *  s_q_RF_KFE_) + (((((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  s_q_RF_HAA_)) *  c_q_RF_WHEEL_));
    (*this)(5,2) = (((((((- 0.08748 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.248 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08748 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.248 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.28263 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.36 *  c_q_RF_HAA_));
    (*this)(5,3) = ((((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(5,4) = ((((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_base::Type_fr_RF_WHEEL_L_X_fr_base()
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
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_WHEEL_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_WHEEL_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_WHEEL_ = TRAIT::sin( q(RF_WHEEL));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_WHEEL_ = TRAIT::cos( q(RF_WHEEL));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( 1.0 *  c_q_RF_HFE_) *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(0,1) = (((((( 1.0 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((( 1.0 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(0,2) = (((((( 1.0 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,0) = ((((( 1.0 *  s_q_RF_HFE_) *  s_q_RF_KFE_) - ( c_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,1) = (((((- s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((( 1.0 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,2) = (((((( 1.0 *  c_q_RF_HAA_) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((( 1.0 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(2,1) =  c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(3,0) = (((((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(3,1) = (((((((((- 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.28263 *  c_q_RF_HAA_)) *  s_q_RF_KFE_) + (((( 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  c_q_RF_HAA_)) *  s_q_RF_WHEEL_) + ((((((( 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.28263 *  c_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.248 *  c_q_RF_HAA_)) *  c_q_RF_WHEEL_));
    (*this)(3,2) = (((((((((- 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  c_q_RF_HFE_)) + ( 0.28263 *  s_q_RF_HAA_)) *  s_q_RF_KFE_) + (((((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  s_q_RF_HAA_)) *  s_q_RF_WHEEL_) + ((((((((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.18979 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.28263 *  s_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.248 *  s_q_RF_HAA_)) *  c_q_RF_WHEEL_));
    (*this)(3,3) = ((((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( 1.0 *  c_q_RF_HFE_) *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(3,4) = (((((( 1.0 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((( 1.0 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(3,5) = (((((( 1.0 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(4,0) = (((((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(4,1) = ((((((((- 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((((- 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.28263 *  c_q_RF_HAA_)) *  c_q_RF_KFE_)) + ( 0.248 *  c_q_RF_HAA_)) *  s_q_RF_WHEEL_) + ((((((((- 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.28263 *  c_q_RF_HAA_)) *  s_q_RF_KFE_) + (((( 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  c_q_RF_HAA_)) *  c_q_RF_WHEEL_));
    (*this)(4,2) = ((((((((( 0.18979 *  c_q_RF_HAA_) +  0.104) *  s_q_RF_HFE_) - (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((((- 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  c_q_RF_HFE_)) + ( 0.28263 *  s_q_RF_HAA_)) *  c_q_RF_KFE_)) + ( 0.248 *  s_q_RF_HAA_)) *  s_q_RF_WHEEL_) + ((((((((- 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  c_q_RF_HFE_)) + ( 0.28263 *  s_q_RF_HAA_)) *  s_q_RF_KFE_) + (((((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  s_q_RF_HAA_)) *  c_q_RF_WHEEL_));
    (*this)(4,3) = ((((( 1.0 *  s_q_RF_HFE_) *  s_q_RF_KFE_) - ( c_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(4,4) = (((((- s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((( 1.0 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(4,5) = (((((( 1.0 *  c_q_RF_HAA_) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((( 1.0 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(5,0) = (((((( 0.08748 *  c_q_RF_HFE_) - ( 0.248 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.08748 *  s_q_RF_HFE_) + ( 0.248 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.28263 *  c_q_RF_HFE_)) - ( 0.104 *  s_q_RF_HAA_));
    (*this)(5,1) = ((((((( 0.08748 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.248 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.248 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08748 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.28263 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.36 *  s_q_RF_HAA_));
    (*this)(5,2) = (((((((- 0.08748 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.248 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08748 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.248 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.28263 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.36 *  c_q_RF_HAA_));
    (*this)(5,4) =  c_q_RF_HAA_;
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL_L::Type_fr_base_X_fr_RH_WHEEL_L()
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
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL_L& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL_L::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_WHEEL_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_WHEEL_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_WHEEL_ = TRAIT::sin( q(RH_WHEEL));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_WHEEL_ = TRAIT::cos( q(RH_WHEEL));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(0,1) = (((( s_q_RH_HFE_ *  s_q_RH_KFE_) - ( c_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,0) = ((((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,1) = (((((- s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(2,0) = ((((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(2,1) = ((((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(3,0) = (((((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(3,1) = (((((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(3,2) = ((((((- 0.248 *  s_q_RH_HFE_) - ( 0.08748 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.248 *  c_q_RH_HFE_) - ( 0.08748 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.28263 *  c_q_RH_HFE_)) - ( 0.104 *  s_q_RH_HAA_));
    (*this)(3,3) = ((((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(3,4) = (((( s_q_RH_HFE_ *  s_q_RH_KFE_) - ( c_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(4,0) = ((((((((( 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.28263 *  c_q_RH_HAA_)) *  s_q_RH_KFE_) + (((( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  c_q_RH_HAA_)) *  s_q_RH_WHEEL_) + ((((((( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.28263 *  c_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.248 *  c_q_RH_HAA_)) *  c_q_RH_WHEEL_));
    (*this)(4,1) = (((((((( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((( 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.28263 *  c_q_RH_HAA_)) *  c_q_RH_KFE_)) + ( 0.248 *  c_q_RH_HAA_)) *  s_q_RH_WHEEL_) + (((((((( 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.28263 *  c_q_RH_HAA_)) *  s_q_RH_KFE_) + (((( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  c_q_RH_HAA_)) *  c_q_RH_WHEEL_));
    (*this)(4,2) = ((((((( 0.248 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08748 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.248 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08748 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.28263 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.36 *  s_q_RH_HAA_));
    (*this)(4,3) = ((((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(4,4) = (((((- s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(4,5) =  c_q_RH_HAA_;
    (*this)(5,0) = ((((((((( 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  c_q_RH_HFE_)) + ( 0.28263 *  s_q_RH_HAA_)) *  s_q_RH_KFE_) + (((((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  s_q_RH_HAA_)) *  s_q_RH_WHEEL_) + ((((((((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.18979 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.28263 *  s_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.248 *  s_q_RH_HAA_)) *  c_q_RH_WHEEL_));
    (*this)(5,1) = ((((((((( 0.18979 *  c_q_RH_HAA_) +  0.104) *  s_q_RH_HFE_) + (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((( 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  c_q_RH_HFE_)) + ( 0.28263 *  s_q_RH_HAA_)) *  c_q_RH_KFE_)) + ( 0.248 *  s_q_RH_HAA_)) *  s_q_RH_WHEEL_) + (((((((( 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  c_q_RH_HFE_)) + ( 0.28263 *  s_q_RH_HAA_)) *  s_q_RH_KFE_) + (((((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  s_q_RH_HAA_)) *  c_q_RH_WHEEL_));
    (*this)(5,2) = ((((((( 0.08748 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.248 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.248 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08748 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.28263 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.36 *  c_q_RH_HAA_));
    (*this)(5,3) = ((((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(5,4) = ((((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_base::Type_fr_RH_WHEEL_L_X_fr_base()
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
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_base& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_WHEEL_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_WHEEL_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_WHEEL_ = TRAIT::sin( q(RH_WHEEL));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_WHEEL_ = TRAIT::cos( q(RH_WHEEL));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( 1.0 *  c_q_RH_HFE_) *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(0,1) = (((((( 1.0 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((( 1.0 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(0,2) = (((((( 1.0 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,0) = ((((( 1.0 *  s_q_RH_HFE_) *  s_q_RH_KFE_) - ( c_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,1) = (((((- s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((( 1.0 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,2) = (((((( 1.0 *  c_q_RH_HAA_) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((( 1.0 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(2,1) =  c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(3,0) = (((((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(3,1) = ((((((((( 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.28263 *  c_q_RH_HAA_)) *  s_q_RH_KFE_) + (((( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  c_q_RH_HAA_)) *  s_q_RH_WHEEL_) + ((((((( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.28263 *  c_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.248 *  c_q_RH_HAA_)) *  c_q_RH_WHEEL_));
    (*this)(3,2) = ((((((((( 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  c_q_RH_HFE_)) + ( 0.28263 *  s_q_RH_HAA_)) *  s_q_RH_KFE_) + (((((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  s_q_RH_HAA_)) *  s_q_RH_WHEEL_) + ((((((((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.18979 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.28263 *  s_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.248 *  s_q_RH_HAA_)) *  c_q_RH_WHEEL_));
    (*this)(3,3) = ((((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( 1.0 *  c_q_RH_HFE_) *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(3,4) = (((((( 1.0 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((( 1.0 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(3,5) = (((((( 1.0 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(4,0) = (((((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(4,1) = (((((((( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((( 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.28263 *  c_q_RH_HAA_)) *  c_q_RH_KFE_)) + ( 0.248 *  c_q_RH_HAA_)) *  s_q_RH_WHEEL_) + (((((((( 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.28263 *  c_q_RH_HAA_)) *  s_q_RH_KFE_) + (((( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  c_q_RH_HAA_)) *  c_q_RH_WHEEL_));
    (*this)(4,2) = ((((((((( 0.18979 *  c_q_RH_HAA_) +  0.104) *  s_q_RH_HFE_) + (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((( 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  c_q_RH_HFE_)) + ( 0.28263 *  s_q_RH_HAA_)) *  c_q_RH_KFE_)) + ( 0.248 *  s_q_RH_HAA_)) *  s_q_RH_WHEEL_) + (((((((( 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  c_q_RH_HFE_)) + ( 0.28263 *  s_q_RH_HAA_)) *  s_q_RH_KFE_) + (((((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  s_q_RH_HAA_)) *  c_q_RH_WHEEL_));
    (*this)(4,3) = ((((( 1.0 *  s_q_RH_HFE_) *  s_q_RH_KFE_) - ( c_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(4,4) = (((((- s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((( 1.0 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(4,5) = (((((( 1.0 *  c_q_RH_HAA_) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((( 1.0 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(5,0) = ((((((- 0.248 *  s_q_RH_HFE_) - ( 0.08748 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.248 *  c_q_RH_HFE_) - ( 0.08748 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.28263 *  c_q_RH_HFE_)) - ( 0.104 *  s_q_RH_HAA_));
    (*this)(5,1) = ((((((( 0.248 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08748 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.248 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08748 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.28263 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.36 *  s_q_RH_HAA_));
    (*this)(5,2) = ((((((( 0.08748 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.248 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.248 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08748 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.28263 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.36 *  c_q_RH_HAA_));
    (*this)(5,4) =  c_q_RH_HAA_;
    (*this)(5,5) =  s_q_RH_HAA_;
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
    (*this)(3,0) = - 0.104;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.3;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0.3;
    (*this)(5,2) = - 0.104;
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
    (*this)(3,1) = ((- 0.104 *  c_q_LF_HAA_) -  0.0701);
    (*this)(3,2) = ( 0.104 *  s_q_LF_HAA_);
    (*this)(4,0) = ( 0.0701 *  s_q_LF_HAA_);
    (*this)(4,1) = ( 0.36 *  c_q_LF_HAA_);
    (*this)(4,2) = (- 0.36 *  s_q_LF_HAA_);
    (*this)(4,4) =  s_q_LF_HAA_;
    (*this)(4,5) =  c_q_LF_HAA_;
    (*this)(5,0) = ((- 0.0701 *  c_q_LF_HAA_) -  0.104);
    (*this)(5,1) = ( 0.36 *  s_q_LF_HAA_);
    (*this)(5,2) = ( 0.36 *  c_q_LF_HAA_);
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
    (*this)(3,0) = (((- 0.104 *  c_q_LF_HAA_) -  0.15699) *  s_q_LF_HFE_);
    (*this)(3,1) = (((- 0.104 *  c_q_LF_HAA_) -  0.15699) *  c_q_LF_HFE_);
    (*this)(3,2) = (( 0.28263 *  c_q_LF_HFE_) + ( 0.104 *  s_q_LF_HAA_));
    (*this)(3,3) =  c_q_LF_HFE_;
    (*this)(3,4) = - s_q_LF_HFE_;
    (*this)(4,0) = (((( 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.15699 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.28263 *  c_q_LF_HAA_));
    (*this)(4,1) = ((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.15699 *  s_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(4,2) = ((( 0.28263 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - ( 0.36 *  s_q_LF_HAA_));
    (*this)(4,3) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(4,4) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(4,5) =  c_q_LF_HAA_;
    (*this)(5,0) = (((( 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.15699 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.28263 *  s_q_LF_HAA_));
    (*this)(5,1) = (((( 0.15699 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_));
    (*this)(5,2) = (( 0.36 *  c_q_LF_HAA_) - (( 0.28263 *  c_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(5,3) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(5,4) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL::Type_fr_base_X_fr_LF_WHEEL()
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
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL::update(const JState& q) {
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
    (*this)(3,0) = (((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,1) = ((((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(3,2) = (((((( 0.08748 *  c_q_LF_HFE_) - ( 0.248 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.08748 *  s_q_LF_HFE_) + ( 0.248 *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.28263 *  c_q_LF_HFE_)) + ( 0.104 *  s_q_LF_HAA_));
    (*this)(3,3) = (( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_));
    (*this)(3,4) = ((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_));
    (*this)(4,0) = (((((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.28263 *  c_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.248 *  c_q_LF_HAA_));
    (*this)(4,1) = (((((((- 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.28263 *  c_q_LF_HAA_)) *  s_q_LF_KFE_) + (((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  c_q_LF_HAA_));
    (*this)(4,2) = ((((((( 0.08748 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.248 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.248 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08748 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.28263 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.36 *  s_q_LF_HAA_));
    (*this)(4,3) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,4) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(4,5) =  c_q_LF_HAA_;
    (*this)(5,0) = ((((((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.18979 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.28263 *  s_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.248 *  s_q_LF_HAA_));
    (*this)(5,1) = (((((((- 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  c_q_LF_HFE_)) + ( 0.28263 *  s_q_LF_HAA_)) *  s_q_LF_KFE_) + ((((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  s_q_LF_HAA_));
    (*this)(5,2) = (((((((- 0.08748 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.248 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08748 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.248 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.28263 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.36 *  c_q_LF_HAA_));
    (*this)(5,3) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,4) = ((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(5,5) =  s_q_LF_HAA_;
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
    (*this)(3,0) = - 0.104;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = - 0.3;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.3;
    (*this)(5,2) = - 0.104;
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
    (*this)(3,1) = ((- 0.104 *  c_q_LH_HAA_) -  0.0701);
    (*this)(3,2) = ( 0.104 *  s_q_LH_HAA_);
    (*this)(4,0) = ( 0.0701 *  s_q_LH_HAA_);
    (*this)(4,1) = (- 0.36 *  c_q_LH_HAA_);
    (*this)(4,2) = ( 0.36 *  s_q_LH_HAA_);
    (*this)(4,4) =  s_q_LH_HAA_;
    (*this)(4,5) =  c_q_LH_HAA_;
    (*this)(5,0) = ((- 0.0701 *  c_q_LH_HAA_) -  0.104);
    (*this)(5,1) = (- 0.36 *  s_q_LH_HAA_);
    (*this)(5,2) = (- 0.36 *  c_q_LH_HAA_);
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
    (*this)(3,0) = (((- 0.104 *  c_q_LH_HAA_) -  0.15699) *  s_q_LH_HFE_);
    (*this)(3,1) = (((- 0.104 *  c_q_LH_HAA_) -  0.15699) *  c_q_LH_HFE_);
    (*this)(3,2) = (( 0.28263 *  c_q_LH_HFE_) + ( 0.104 *  s_q_LH_HAA_));
    (*this)(3,3) =  c_q_LH_HFE_;
    (*this)(3,4) = - s_q_LH_HFE_;
    (*this)(4,0) = ((((- 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.15699 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.28263 *  c_q_LH_HAA_));
    (*this)(4,1) = (((- 0.15699 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(4,2) = ((( 0.28263 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ( 0.36 *  s_q_LH_HAA_));
    (*this)(4,3) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(4,4) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(4,5) =  c_q_LH_HAA_;
    (*this)(5,0) = ((((- 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.15699 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.28263 *  s_q_LH_HAA_));
    (*this)(5,1) = (((( 0.15699 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(5,2) = (((- 0.28263 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - ( 0.36 *  c_q_LH_HAA_));
    (*this)(5,3) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(5,4) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL::Type_fr_base_X_fr_LH_WHEEL()
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
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL::update(const JState& q) {
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
    (*this)(3,0) = (((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,1) = ((((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(3,2) = ((((((- 0.248 *  s_q_LH_HFE_) - ( 0.08748 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.248 *  c_q_LH_HFE_) - ( 0.08748 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.28263 *  c_q_LH_HFE_)) + ( 0.104 *  s_q_LH_HAA_));
    (*this)(3,3) = (( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_));
    (*this)(3,4) = ((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_));
    (*this)(4,0) = ((((((- 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.28263 *  c_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.248 *  c_q_LH_HAA_));
    (*this)(4,1) = ((((((( 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.28263 *  c_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((- 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  c_q_LH_HAA_));
    (*this)(4,2) = ((((((( 0.248 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08748 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.248 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08748 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.28263 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.36 *  s_q_LH_HAA_));
    (*this)(4,3) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,4) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(4,5) =  c_q_LH_HAA_;
    (*this)(5,0) = ((((((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.18979 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.28263 *  s_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.248 *  s_q_LH_HAA_));
    (*this)(5,1) = ((((((( 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  c_q_LH_HFE_)) + ( 0.28263 *  s_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  s_q_LH_HAA_));
    (*this)(5,2) = ((((((( 0.08748 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.248 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.248 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08748 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.28263 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.36 *  c_q_LH_HAA_));
    (*this)(5,3) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,4) = ((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,5) =  s_q_LH_HAA_;
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
    (*this)(3,0) = 0.104;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.3;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0.3;
    (*this)(5,2) = 0.104;
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
    (*this)(3,1) = (( 0.104 *  c_q_RF_HAA_) +  0.0701);
    (*this)(3,2) = (- 0.104 *  s_q_RF_HAA_);
    (*this)(4,0) = (- 0.0701 *  s_q_RF_HAA_);
    (*this)(4,1) = ( 0.36 *  c_q_RF_HAA_);
    (*this)(4,2) = (- 0.36 *  s_q_RF_HAA_);
    (*this)(4,4) =  s_q_RF_HAA_;
    (*this)(4,5) =  c_q_RF_HAA_;
    (*this)(5,0) = (( 0.0701 *  c_q_RF_HAA_) +  0.104);
    (*this)(5,1) = ( 0.36 *  s_q_RF_HAA_);
    (*this)(5,2) = ( 0.36 *  c_q_RF_HAA_);
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
    (*this)(3,0) = ((( 0.104 *  c_q_RF_HAA_) +  0.15699) *  s_q_RF_HFE_);
    (*this)(3,1) = ((( 0.104 *  c_q_RF_HAA_) +  0.15699) *  c_q_RF_HFE_);
    (*this)(3,2) = (( 0.28263 *  c_q_RF_HFE_) - ( 0.104 *  s_q_RF_HAA_));
    (*this)(3,3) =  c_q_RF_HFE_;
    (*this)(3,4) = - s_q_RF_HFE_;
    (*this)(4,0) = (((( 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.15699 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.28263 *  c_q_RF_HAA_));
    (*this)(4,1) = ((( 0.15699 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(4,2) = ((( 0.28263 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.36 *  s_q_RF_HAA_));
    (*this)(4,3) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(4,4) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(4,5) =  c_q_RF_HAA_;
    (*this)(5,0) = (((( 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.15699 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.28263 *  s_q_RF_HAA_));
    (*this)(5,1) = ((((- 0.15699 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(5,2) = (( 0.36 *  c_q_RF_HAA_) - (( 0.28263 *  c_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(5,3) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(5,4) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL::Type_fr_base_X_fr_RF_WHEEL()
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
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL::update(const JState& q) {
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
    (*this)(3,0) = ((((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,1) = (((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(3,2) = (((((( 0.08748 *  c_q_RF_HFE_) - ( 0.248 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.08748 *  s_q_RF_HFE_) + ( 0.248 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.28263 *  c_q_RF_HFE_)) - ( 0.104 *  s_q_RF_HAA_));
    (*this)(3,3) = (( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_));
    (*this)(3,4) = ((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_));
    (*this)(4,0) = (((((( 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.28263 *  c_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.248 *  c_q_RF_HAA_));
    (*this)(4,1) = (((((((- 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.28263 *  c_q_RF_HAA_)) *  s_q_RF_KFE_) + (((( 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  c_q_RF_HAA_));
    (*this)(4,2) = ((((((( 0.08748 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.248 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.248 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08748 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.28263 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.36 *  s_q_RF_HAA_));
    (*this)(4,3) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,4) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(4,5) =  c_q_RF_HAA_;
    (*this)(5,0) = (((((((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.18979 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.28263 *  s_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.248 *  s_q_RF_HAA_));
    (*this)(5,1) = (((((((- 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  c_q_RF_HFE_)) + ( 0.28263 *  s_q_RF_HAA_)) *  s_q_RF_KFE_) + (((((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  s_q_RF_HAA_));
    (*this)(5,2) = (((((((- 0.08748 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.248 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08748 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.248 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.28263 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.36 *  c_q_RF_HAA_));
    (*this)(5,3) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,4) = ((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(5,5) =  s_q_RF_HAA_;
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
    (*this)(3,0) = 0.104;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = - 0.3;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.3;
    (*this)(5,2) = 0.104;
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
    (*this)(3,1) = (( 0.104 *  c_q_RH_HAA_) +  0.0701);
    (*this)(3,2) = (- 0.104 *  s_q_RH_HAA_);
    (*this)(4,0) = (- 0.0701 *  s_q_RH_HAA_);
    (*this)(4,1) = (- 0.36 *  c_q_RH_HAA_);
    (*this)(4,2) = ( 0.36 *  s_q_RH_HAA_);
    (*this)(4,4) =  s_q_RH_HAA_;
    (*this)(4,5) =  c_q_RH_HAA_;
    (*this)(5,0) = (( 0.0701 *  c_q_RH_HAA_) +  0.104);
    (*this)(5,1) = (- 0.36 *  s_q_RH_HAA_);
    (*this)(5,2) = (- 0.36 *  c_q_RH_HAA_);
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
    (*this)(3,0) = ((( 0.104 *  c_q_RH_HAA_) +  0.15699) *  s_q_RH_HFE_);
    (*this)(3,1) = ((( 0.104 *  c_q_RH_HAA_) +  0.15699) *  c_q_RH_HFE_);
    (*this)(3,2) = (( 0.28263 *  c_q_RH_HFE_) - ( 0.104 *  s_q_RH_HAA_));
    (*this)(3,3) =  c_q_RH_HFE_;
    (*this)(3,4) = - s_q_RH_HFE_;
    (*this)(4,0) = ((((- 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.15699 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.28263 *  c_q_RH_HAA_));
    (*this)(4,1) = ((( 0.15699 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(4,2) = ((( 0.28263 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ( 0.36 *  s_q_RH_HAA_));
    (*this)(4,3) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(4,4) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(4,5) =  c_q_RH_HAA_;
    (*this)(5,0) = ((((- 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.15699 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.28263 *  s_q_RH_HAA_));
    (*this)(5,1) = ((((- 0.15699 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(5,2) = (((- 0.28263 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - ( 0.36 *  c_q_RH_HAA_));
    (*this)(5,3) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(5,4) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL::Type_fr_base_X_fr_RH_WHEEL()
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
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL::update(const JState& q) {
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
    (*this)(3,0) = ((((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,1) = (((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(3,2) = ((((((- 0.248 *  s_q_RH_HFE_) - ( 0.08748 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.248 *  c_q_RH_HFE_) - ( 0.08748 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.28263 *  c_q_RH_HFE_)) - ( 0.104 *  s_q_RH_HAA_));
    (*this)(3,3) = (( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_));
    (*this)(3,4) = ((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_));
    (*this)(4,0) = (((((( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.28263 *  c_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.248 *  c_q_RH_HAA_));
    (*this)(4,1) = ((((((( 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.28263 *  c_q_RH_HAA_)) *  s_q_RH_KFE_) + (((( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  c_q_RH_HAA_));
    (*this)(4,2) = ((((((( 0.248 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08748 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.248 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08748 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.28263 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.36 *  s_q_RH_HAA_));
    (*this)(4,3) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,4) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(4,5) =  c_q_RH_HAA_;
    (*this)(5,0) = (((((((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.18979 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.28263 *  s_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.248 *  s_q_RH_HAA_));
    (*this)(5,1) = ((((((( 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  c_q_RH_HFE_)) + ( 0.28263 *  s_q_RH_HAA_)) *  s_q_RH_KFE_) + (((((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  s_q_RH_HAA_));
    (*this)(5,2) = ((((((( 0.08748 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.248 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.248 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08748 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.28263 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.36 *  c_q_RH_HAA_));
    (*this)(5,3) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,4) = ((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,5) =  s_q_RH_HAA_;
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
    (*this)(5,2) = - 0.104;
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
    (*this)(3,0) = (- 0.104 *  c_q_LF_HAA_);
    (*this)(3,1) = ( 0.3 *  c_q_LF_HAA_);
    (*this)(3,2) = ( 0.3 *  s_q_LF_HAA_);
    (*this)(3,4) =  s_q_LF_HAA_;
    (*this)(3,5) = - c_q_LF_HAA_;
    (*this)(4,0) = ( 0.104 *  s_q_LF_HAA_);
    (*this)(4,1) = (- 0.3 *  s_q_LF_HAA_);
    (*this)(4,2) = ( 0.3 *  c_q_LF_HAA_);
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
    return *this;
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
    (*this)(5,2) = - 0.104;
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
    (*this)(3,0) = (- 0.104 *  c_q_LF_HAA_);
    (*this)(3,1) = ( 0.104 *  s_q_LF_HAA_);
    (*this)(4,0) = ( 0.3 *  c_q_LF_HAA_);
    (*this)(4,1) = (- 0.3 *  s_q_LF_HAA_);
    (*this)(4,3) =  s_q_LF_HAA_;
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(5,0) = ( 0.3 *  s_q_LF_HAA_);
    (*this)(5,1) = ( 0.3 *  c_q_LF_HAA_);
    (*this)(5,3) = - c_q_LF_HAA_;
    (*this)(5,4) =  s_q_LF_HAA_;
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
    (*this)(5,0) = - 0.06;
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
    (*this)(3,0) = ( 0.0701 *  c_q_LF_HFE_);
    (*this)(3,1) = ( 0.06 *  s_q_LF_HFE_);
    (*this)(3,2) = (- 0.0701 *  s_q_LF_HFE_);
    (*this)(3,3) =  s_q_LF_HFE_;
    (*this)(3,5) =  c_q_LF_HFE_;
    (*this)(4,0) = (- 0.0701 *  s_q_LF_HFE_);
    (*this)(4,1) = ( 0.06 *  c_q_LF_HFE_);
    (*this)(4,2) = (- 0.0701 *  c_q_LF_HFE_);
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
    (*this)(3,2) = - 0.06;
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
    (*this)(3,0) = ( 0.0701 *  c_q_LF_HFE_);
    (*this)(3,1) = (- 0.0701 *  s_q_LF_HFE_);
    (*this)(3,3) =  s_q_LF_HFE_;
    (*this)(3,4) =  c_q_LF_HFE_;
    (*this)(4,0) = ( 0.06 *  s_q_LF_HFE_);
    (*this)(4,1) = ( 0.06 *  c_q_LF_HFE_);
    (*this)(5,0) = (- 0.0701 *  s_q_LF_HFE_);
    (*this)(5,1) = (- 0.0701 *  c_q_LF_HFE_);
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
    (*this)(5,0) = 0.28263;
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
    (*this)(3,0) = (- 0.08689 *  s_q_LF_KFE_);
    (*this)(3,1) = ( 0.08689 *  c_q_LF_KFE_);
    (*this)(3,2) = (- 0.28263 *  c_q_LF_KFE_);
    (*this)(3,3) =  c_q_LF_KFE_;
    (*this)(3,4) =  s_q_LF_KFE_;
    (*this)(4,0) = (- 0.08689 *  c_q_LF_KFE_);
    (*this)(4,1) = (- 0.08689 *  s_q_LF_KFE_);
    (*this)(4,2) = ( 0.28263 *  s_q_LF_KFE_);
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
    (*this)(3,2) = 0.28263;
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
    (*this)(3,0) = (- 0.08689 *  s_q_LF_KFE_);
    (*this)(3,1) = (- 0.08689 *  c_q_LF_KFE_);
    (*this)(3,3) =  c_q_LF_KFE_;
    (*this)(3,4) = - s_q_LF_KFE_;
    (*this)(4,0) = ( 0.08689 *  c_q_LF_KFE_);
    (*this)(4,1) = (- 0.08689 *  s_q_LF_KFE_);
    (*this)(4,3) =  s_q_LF_KFE_;
    (*this)(4,4) =  c_q_LF_KFE_;
    (*this)(5,0) = (- 0.28263 *  c_q_LF_KFE_);
    (*this)(5,1) = ( 0.28263 *  s_q_LF_KFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_LF_SHANK::Type_fr_LF_WHEEL_L_X_fr_LF_SHANK()
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
    (*this)(5,0) = 0.248;
    (*this)(5,1) = - 0.08748;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_LF_SHANK& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_LF_SHANK::update(const JState& q) {
    Scalar s_q_LF_WHEEL_;
    Scalar c_q_LF_WHEEL_;
    
    s_q_LF_WHEEL_ = TRAIT::sin( q(LF_WHEEL));
    c_q_LF_WHEEL_ = TRAIT::cos( q(LF_WHEEL));
    
    (*this)(0,0) =  c_q_LF_WHEEL_;
    (*this)(0,1) =  s_q_LF_WHEEL_;
    (*this)(1,0) = - s_q_LF_WHEEL_;
    (*this)(1,1) =  c_q_LF_WHEEL_;
    (*this)(3,0) = (- 0.0328 *  s_q_LF_WHEEL_);
    (*this)(3,1) = ( 0.0328 *  c_q_LF_WHEEL_);
    (*this)(3,2) = (( 0.08748 *  s_q_LF_WHEEL_) - ( 0.248 *  c_q_LF_WHEEL_));
    (*this)(3,3) =  c_q_LF_WHEEL_;
    (*this)(3,4) =  s_q_LF_WHEEL_;
    (*this)(4,0) = (- 0.0328 *  c_q_LF_WHEEL_);
    (*this)(4,1) = (- 0.0328 *  s_q_LF_WHEEL_);
    (*this)(4,2) = (( 0.248 *  s_q_LF_WHEEL_) + ( 0.08748 *  c_q_LF_WHEEL_));
    (*this)(4,3) = - s_q_LF_WHEEL_;
    (*this)(4,4) =  c_q_LF_WHEEL_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_WHEEL_L::Type_fr_LF_SHANK_X_fr_LF_WHEEL_L()
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
    (*this)(3,2) = 0.248;
    (*this)(3,5) = 0;
    (*this)(4,2) = - 0.08748;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_WHEEL_L& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_WHEEL_L::update(const JState& q) {
    Scalar s_q_LF_WHEEL_;
    Scalar c_q_LF_WHEEL_;
    
    s_q_LF_WHEEL_ = TRAIT::sin( q(LF_WHEEL));
    c_q_LF_WHEEL_ = TRAIT::cos( q(LF_WHEEL));
    
    (*this)(0,0) =  c_q_LF_WHEEL_;
    (*this)(0,1) = - s_q_LF_WHEEL_;
    (*this)(1,0) =  s_q_LF_WHEEL_;
    (*this)(1,1) =  c_q_LF_WHEEL_;
    (*this)(3,0) = (- 0.0328 *  s_q_LF_WHEEL_);
    (*this)(3,1) = (- 0.0328 *  c_q_LF_WHEEL_);
    (*this)(3,3) =  c_q_LF_WHEEL_;
    (*this)(3,4) = - s_q_LF_WHEEL_;
    (*this)(4,0) = ( 0.0328 *  c_q_LF_WHEEL_);
    (*this)(4,1) = (- 0.0328 *  s_q_LF_WHEEL_);
    (*this)(4,3) =  s_q_LF_WHEEL_;
    (*this)(4,4) =  c_q_LF_WHEEL_;
    (*this)(5,0) = (( 0.08748 *  s_q_LF_WHEEL_) - ( 0.248 *  c_q_LF_WHEEL_));
    (*this)(5,1) = (( 0.248 *  s_q_LF_WHEEL_) + ( 0.08748 *  c_q_LF_WHEEL_));
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
    (*this)(5,2) = 0.104;
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
    (*this)(3,0) = ( 0.104 *  c_q_RF_HAA_);
    (*this)(3,1) = ( 0.3 *  c_q_RF_HAA_);
    (*this)(3,2) = ( 0.3 *  s_q_RF_HAA_);
    (*this)(3,4) =  s_q_RF_HAA_;
    (*this)(3,5) = - c_q_RF_HAA_;
    (*this)(4,0) = (- 0.104 *  s_q_RF_HAA_);
    (*this)(4,1) = (- 0.3 *  s_q_RF_HAA_);
    (*this)(4,2) = ( 0.3 *  c_q_RF_HAA_);
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
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
    (*this)(5,2) = 0.104;
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
    (*this)(3,0) = ( 0.104 *  c_q_RF_HAA_);
    (*this)(3,1) = (- 0.104 *  s_q_RF_HAA_);
    (*this)(4,0) = ( 0.3 *  c_q_RF_HAA_);
    (*this)(4,1) = (- 0.3 *  s_q_RF_HAA_);
    (*this)(4,3) =  s_q_RF_HAA_;
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(5,0) = ( 0.3 *  s_q_RF_HAA_);
    (*this)(5,1) = ( 0.3 *  c_q_RF_HAA_);
    (*this)(5,3) = - c_q_RF_HAA_;
    (*this)(5,4) =  s_q_RF_HAA_;
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
    (*this)(5,0) = - 0.06;
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
    (*this)(3,0) = (- 0.0701 *  c_q_RF_HFE_);
    (*this)(3,1) = ( 0.06 *  s_q_RF_HFE_);
    (*this)(3,2) = ( 0.0701 *  s_q_RF_HFE_);
    (*this)(3,3) =  s_q_RF_HFE_;
    (*this)(3,5) =  c_q_RF_HFE_;
    (*this)(4,0) = ( 0.0701 *  s_q_RF_HFE_);
    (*this)(4,1) = ( 0.06 *  c_q_RF_HFE_);
    (*this)(4,2) = ( 0.0701 *  c_q_RF_HFE_);
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
    (*this)(3,2) = - 0.06;
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
    (*this)(3,0) = (- 0.0701 *  c_q_RF_HFE_);
    (*this)(3,1) = ( 0.0701 *  s_q_RF_HFE_);
    (*this)(3,3) =  s_q_RF_HFE_;
    (*this)(3,4) =  c_q_RF_HFE_;
    (*this)(4,0) = ( 0.06 *  s_q_RF_HFE_);
    (*this)(4,1) = ( 0.06 *  c_q_RF_HFE_);
    (*this)(5,0) = ( 0.0701 *  s_q_RF_HFE_);
    (*this)(5,1) = ( 0.0701 *  c_q_RF_HFE_);
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
    (*this)(5,0) = 0.28263;
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
    (*this)(3,0) = ( 0.08689 *  s_q_RF_KFE_);
    (*this)(3,1) = (- 0.08689 *  c_q_RF_KFE_);
    (*this)(3,2) = (- 0.28263 *  c_q_RF_KFE_);
    (*this)(3,3) =  c_q_RF_KFE_;
    (*this)(3,4) =  s_q_RF_KFE_;
    (*this)(4,0) = ( 0.08689 *  c_q_RF_KFE_);
    (*this)(4,1) = ( 0.08689 *  s_q_RF_KFE_);
    (*this)(4,2) = ( 0.28263 *  s_q_RF_KFE_);
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
    (*this)(3,2) = 0.28263;
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
    (*this)(3,0) = ( 0.08689 *  s_q_RF_KFE_);
    (*this)(3,1) = ( 0.08689 *  c_q_RF_KFE_);
    (*this)(3,3) =  c_q_RF_KFE_;
    (*this)(3,4) = - s_q_RF_KFE_;
    (*this)(4,0) = (- 0.08689 *  c_q_RF_KFE_);
    (*this)(4,1) = ( 0.08689 *  s_q_RF_KFE_);
    (*this)(4,3) =  s_q_RF_KFE_;
    (*this)(4,4) =  c_q_RF_KFE_;
    (*this)(5,0) = (- 0.28263 *  c_q_RF_KFE_);
    (*this)(5,1) = ( 0.28263 *  s_q_RF_KFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_RF_SHANK::Type_fr_RF_WHEEL_L_X_fr_RF_SHANK()
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
    (*this)(5,0) = 0.248;
    (*this)(5,1) = - 0.08748;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_RF_SHANK& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_RF_SHANK::update(const JState& q) {
    Scalar s_q_RF_WHEEL_;
    Scalar c_q_RF_WHEEL_;
    
    s_q_RF_WHEEL_ = TRAIT::sin( q(RF_WHEEL));
    c_q_RF_WHEEL_ = TRAIT::cos( q(RF_WHEEL));
    
    (*this)(0,0) =  c_q_RF_WHEEL_;
    (*this)(0,1) =  s_q_RF_WHEEL_;
    (*this)(1,0) = - s_q_RF_WHEEL_;
    (*this)(1,1) =  c_q_RF_WHEEL_;
    (*this)(3,0) = ( 0.0328 *  s_q_RF_WHEEL_);
    (*this)(3,1) = (- 0.0328 *  c_q_RF_WHEEL_);
    (*this)(3,2) = (( 0.08748 *  s_q_RF_WHEEL_) - ( 0.248 *  c_q_RF_WHEEL_));
    (*this)(3,3) =  c_q_RF_WHEEL_;
    (*this)(3,4) =  s_q_RF_WHEEL_;
    (*this)(4,0) = ( 0.0328 *  c_q_RF_WHEEL_);
    (*this)(4,1) = ( 0.0328 *  s_q_RF_WHEEL_);
    (*this)(4,2) = (( 0.248 *  s_q_RF_WHEEL_) + ( 0.08748 *  c_q_RF_WHEEL_));
    (*this)(4,3) = - s_q_RF_WHEEL_;
    (*this)(4,4) =  c_q_RF_WHEEL_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_WHEEL_L::Type_fr_RF_SHANK_X_fr_RF_WHEEL_L()
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
    (*this)(3,2) = 0.248;
    (*this)(3,5) = 0;
    (*this)(4,2) = - 0.08748;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_WHEEL_L& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_WHEEL_L::update(const JState& q) {
    Scalar s_q_RF_WHEEL_;
    Scalar c_q_RF_WHEEL_;
    
    s_q_RF_WHEEL_ = TRAIT::sin( q(RF_WHEEL));
    c_q_RF_WHEEL_ = TRAIT::cos( q(RF_WHEEL));
    
    (*this)(0,0) =  c_q_RF_WHEEL_;
    (*this)(0,1) = - s_q_RF_WHEEL_;
    (*this)(1,0) =  s_q_RF_WHEEL_;
    (*this)(1,1) =  c_q_RF_WHEEL_;
    (*this)(3,0) = ( 0.0328 *  s_q_RF_WHEEL_);
    (*this)(3,1) = ( 0.0328 *  c_q_RF_WHEEL_);
    (*this)(3,3) =  c_q_RF_WHEEL_;
    (*this)(3,4) = - s_q_RF_WHEEL_;
    (*this)(4,0) = (- 0.0328 *  c_q_RF_WHEEL_);
    (*this)(4,1) = ( 0.0328 *  s_q_RF_WHEEL_);
    (*this)(4,3) =  s_q_RF_WHEEL_;
    (*this)(4,4) =  c_q_RF_WHEEL_;
    (*this)(5,0) = (( 0.08748 *  s_q_RF_WHEEL_) - ( 0.248 *  c_q_RF_WHEEL_));
    (*this)(5,1) = (( 0.248 *  s_q_RF_WHEEL_) + ( 0.08748 *  c_q_RF_WHEEL_));
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
    (*this)(5,2) = - 0.104;
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
    (*this)(3,0) = (- 0.104 *  c_q_LH_HAA_);
    (*this)(3,1) = (- 0.3 *  c_q_LH_HAA_);
    (*this)(3,2) = (- 0.3 *  s_q_LH_HAA_);
    (*this)(3,4) =  s_q_LH_HAA_;
    (*this)(3,5) = - c_q_LH_HAA_;
    (*this)(4,0) = ( 0.104 *  s_q_LH_HAA_);
    (*this)(4,1) = ( 0.3 *  s_q_LH_HAA_);
    (*this)(4,2) = (- 0.3 *  c_q_LH_HAA_);
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
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
    (*this)(5,2) = - 0.104;
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
    (*this)(3,0) = (- 0.104 *  c_q_LH_HAA_);
    (*this)(3,1) = ( 0.104 *  s_q_LH_HAA_);
    (*this)(4,0) = (- 0.3 *  c_q_LH_HAA_);
    (*this)(4,1) = ( 0.3 *  s_q_LH_HAA_);
    (*this)(4,3) =  s_q_LH_HAA_;
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(5,0) = (- 0.3 *  s_q_LH_HAA_);
    (*this)(5,1) = (- 0.3 *  c_q_LH_HAA_);
    (*this)(5,3) = - c_q_LH_HAA_;
    (*this)(5,4) =  s_q_LH_HAA_;
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
    (*this)(5,0) = 0.06;
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
    (*this)(3,0) = ( 0.0701 *  c_q_LH_HFE_);
    (*this)(3,1) = (- 0.06 *  s_q_LH_HFE_);
    (*this)(3,2) = (- 0.0701 *  s_q_LH_HFE_);
    (*this)(3,3) =  s_q_LH_HFE_;
    (*this)(3,5) =  c_q_LH_HFE_;
    (*this)(4,0) = (- 0.0701 *  s_q_LH_HFE_);
    (*this)(4,1) = (- 0.06 *  c_q_LH_HFE_);
    (*this)(4,2) = (- 0.0701 *  c_q_LH_HFE_);
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
    (*this)(3,2) = 0.06;
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
    (*this)(3,0) = ( 0.0701 *  c_q_LH_HFE_);
    (*this)(3,1) = (- 0.0701 *  s_q_LH_HFE_);
    (*this)(3,3) =  s_q_LH_HFE_;
    (*this)(3,4) =  c_q_LH_HFE_;
    (*this)(4,0) = (- 0.06 *  s_q_LH_HFE_);
    (*this)(4,1) = (- 0.06 *  c_q_LH_HFE_);
    (*this)(5,0) = (- 0.0701 *  s_q_LH_HFE_);
    (*this)(5,1) = (- 0.0701 *  c_q_LH_HFE_);
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
    (*this)(5,0) = 0.28263;
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
    (*this)(3,0) = (- 0.08689 *  s_q_LH_KFE_);
    (*this)(3,1) = ( 0.08689 *  c_q_LH_KFE_);
    (*this)(3,2) = (- 0.28263 *  c_q_LH_KFE_);
    (*this)(3,3) =  c_q_LH_KFE_;
    (*this)(3,4) =  s_q_LH_KFE_;
    (*this)(4,0) = (- 0.08689 *  c_q_LH_KFE_);
    (*this)(4,1) = (- 0.08689 *  s_q_LH_KFE_);
    (*this)(4,2) = ( 0.28263 *  s_q_LH_KFE_);
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
    (*this)(3,2) = 0.28263;
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
    (*this)(3,0) = (- 0.08689 *  s_q_LH_KFE_);
    (*this)(3,1) = (- 0.08689 *  c_q_LH_KFE_);
    (*this)(3,3) =  c_q_LH_KFE_;
    (*this)(3,4) = - s_q_LH_KFE_;
    (*this)(4,0) = ( 0.08689 *  c_q_LH_KFE_);
    (*this)(4,1) = (- 0.08689 *  s_q_LH_KFE_);
    (*this)(4,3) =  s_q_LH_KFE_;
    (*this)(4,4) =  c_q_LH_KFE_;
    (*this)(5,0) = (- 0.28263 *  c_q_LH_KFE_);
    (*this)(5,1) = ( 0.28263 *  s_q_LH_KFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_LH_SHANK::Type_fr_LH_WHEEL_L_X_fr_LH_SHANK()
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
    (*this)(5,0) = 0.248;
    (*this)(5,1) = 0.08748;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_LH_SHANK& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_LH_SHANK::update(const JState& q) {
    Scalar s_q_LH_WHEEL_;
    Scalar c_q_LH_WHEEL_;
    
    s_q_LH_WHEEL_ = TRAIT::sin( q(LH_WHEEL));
    c_q_LH_WHEEL_ = TRAIT::cos( q(LH_WHEEL));
    
    (*this)(0,0) =  c_q_LH_WHEEL_;
    (*this)(0,1) =  s_q_LH_WHEEL_;
    (*this)(1,0) = - s_q_LH_WHEEL_;
    (*this)(1,1) =  c_q_LH_WHEEL_;
    (*this)(3,0) = (- 0.0328 *  s_q_LH_WHEEL_);
    (*this)(3,1) = ( 0.0328 *  c_q_LH_WHEEL_);
    (*this)(3,2) = ((- 0.08748 *  s_q_LH_WHEEL_) - ( 0.248 *  c_q_LH_WHEEL_));
    (*this)(3,3) =  c_q_LH_WHEEL_;
    (*this)(3,4) =  s_q_LH_WHEEL_;
    (*this)(4,0) = (- 0.0328 *  c_q_LH_WHEEL_);
    (*this)(4,1) = (- 0.0328 *  s_q_LH_WHEEL_);
    (*this)(4,2) = (( 0.248 *  s_q_LH_WHEEL_) - ( 0.08748 *  c_q_LH_WHEEL_));
    (*this)(4,3) = - s_q_LH_WHEEL_;
    (*this)(4,4) =  c_q_LH_WHEEL_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_WHEEL_L::Type_fr_LH_SHANK_X_fr_LH_WHEEL_L()
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
    (*this)(3,2) = 0.248;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0.08748;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_WHEEL_L& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_WHEEL_L::update(const JState& q) {
    Scalar s_q_LH_WHEEL_;
    Scalar c_q_LH_WHEEL_;
    
    s_q_LH_WHEEL_ = TRAIT::sin( q(LH_WHEEL));
    c_q_LH_WHEEL_ = TRAIT::cos( q(LH_WHEEL));
    
    (*this)(0,0) =  c_q_LH_WHEEL_;
    (*this)(0,1) = - s_q_LH_WHEEL_;
    (*this)(1,0) =  s_q_LH_WHEEL_;
    (*this)(1,1) =  c_q_LH_WHEEL_;
    (*this)(3,0) = (- 0.0328 *  s_q_LH_WHEEL_);
    (*this)(3,1) = (- 0.0328 *  c_q_LH_WHEEL_);
    (*this)(3,3) =  c_q_LH_WHEEL_;
    (*this)(3,4) = - s_q_LH_WHEEL_;
    (*this)(4,0) = ( 0.0328 *  c_q_LH_WHEEL_);
    (*this)(4,1) = (- 0.0328 *  s_q_LH_WHEEL_);
    (*this)(4,3) =  s_q_LH_WHEEL_;
    (*this)(4,4) =  c_q_LH_WHEEL_;
    (*this)(5,0) = ((- 0.08748 *  s_q_LH_WHEEL_) - ( 0.248 *  c_q_LH_WHEEL_));
    (*this)(5,1) = (( 0.248 *  s_q_LH_WHEEL_) - ( 0.08748 *  c_q_LH_WHEEL_));
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
    (*this)(5,2) = 0.104;
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
    (*this)(3,0) = ( 0.104 *  c_q_RH_HAA_);
    (*this)(3,1) = (- 0.3 *  c_q_RH_HAA_);
    (*this)(3,2) = (- 0.3 *  s_q_RH_HAA_);
    (*this)(3,4) =  s_q_RH_HAA_;
    (*this)(3,5) = - c_q_RH_HAA_;
    (*this)(4,0) = (- 0.104 *  s_q_RH_HAA_);
    (*this)(4,1) = ( 0.3 *  s_q_RH_HAA_);
    (*this)(4,2) = (- 0.3 *  c_q_RH_HAA_);
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
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
    (*this)(5,2) = 0.104;
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
    (*this)(3,0) = ( 0.104 *  c_q_RH_HAA_);
    (*this)(3,1) = (- 0.104 *  s_q_RH_HAA_);
    (*this)(4,0) = (- 0.3 *  c_q_RH_HAA_);
    (*this)(4,1) = ( 0.3 *  s_q_RH_HAA_);
    (*this)(4,3) =  s_q_RH_HAA_;
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(5,0) = (- 0.3 *  s_q_RH_HAA_);
    (*this)(5,1) = (- 0.3 *  c_q_RH_HAA_);
    (*this)(5,3) = - c_q_RH_HAA_;
    (*this)(5,4) =  s_q_RH_HAA_;
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
    (*this)(5,0) = 0.06;
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
    (*this)(3,0) = (- 0.0701 *  c_q_RH_HFE_);
    (*this)(3,1) = (- 0.06 *  s_q_RH_HFE_);
    (*this)(3,2) = ( 0.0701 *  s_q_RH_HFE_);
    (*this)(3,3) =  s_q_RH_HFE_;
    (*this)(3,5) =  c_q_RH_HFE_;
    (*this)(4,0) = ( 0.0701 *  s_q_RH_HFE_);
    (*this)(4,1) = (- 0.06 *  c_q_RH_HFE_);
    (*this)(4,2) = ( 0.0701 *  c_q_RH_HFE_);
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
    (*this)(3,2) = 0.06;
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
    (*this)(3,0) = (- 0.0701 *  c_q_RH_HFE_);
    (*this)(3,1) = ( 0.0701 *  s_q_RH_HFE_);
    (*this)(3,3) =  s_q_RH_HFE_;
    (*this)(3,4) =  c_q_RH_HFE_;
    (*this)(4,0) = (- 0.06 *  s_q_RH_HFE_);
    (*this)(4,1) = (- 0.06 *  c_q_RH_HFE_);
    (*this)(5,0) = ( 0.0701 *  s_q_RH_HFE_);
    (*this)(5,1) = ( 0.0701 *  c_q_RH_HFE_);
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
    (*this)(5,0) = 0.28263;
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
    (*this)(3,0) = ( 0.08689 *  s_q_RH_KFE_);
    (*this)(3,1) = (- 0.08689 *  c_q_RH_KFE_);
    (*this)(3,2) = (- 0.28263 *  c_q_RH_KFE_);
    (*this)(3,3) =  c_q_RH_KFE_;
    (*this)(3,4) =  s_q_RH_KFE_;
    (*this)(4,0) = ( 0.08689 *  c_q_RH_KFE_);
    (*this)(4,1) = ( 0.08689 *  s_q_RH_KFE_);
    (*this)(4,2) = ( 0.28263 *  s_q_RH_KFE_);
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
    (*this)(3,2) = 0.28263;
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
    (*this)(3,0) = ( 0.08689 *  s_q_RH_KFE_);
    (*this)(3,1) = ( 0.08689 *  c_q_RH_KFE_);
    (*this)(3,3) =  c_q_RH_KFE_;
    (*this)(3,4) = - s_q_RH_KFE_;
    (*this)(4,0) = (- 0.08689 *  c_q_RH_KFE_);
    (*this)(4,1) = ( 0.08689 *  s_q_RH_KFE_);
    (*this)(4,3) =  s_q_RH_KFE_;
    (*this)(4,4) =  c_q_RH_KFE_;
    (*this)(5,0) = (- 0.28263 *  c_q_RH_KFE_);
    (*this)(5,1) = ( 0.28263 *  s_q_RH_KFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_RH_SHANK::Type_fr_RH_WHEEL_L_X_fr_RH_SHANK()
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
    (*this)(5,0) = 0.248;
    (*this)(5,1) = 0.08748;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_RH_SHANK& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_RH_SHANK::update(const JState& q) {
    Scalar s_q_RH_WHEEL_;
    Scalar c_q_RH_WHEEL_;
    
    s_q_RH_WHEEL_ = TRAIT::sin( q(RH_WHEEL));
    c_q_RH_WHEEL_ = TRAIT::cos( q(RH_WHEEL));
    
    (*this)(0,0) =  c_q_RH_WHEEL_;
    (*this)(0,1) =  s_q_RH_WHEEL_;
    (*this)(1,0) = - s_q_RH_WHEEL_;
    (*this)(1,1) =  c_q_RH_WHEEL_;
    (*this)(3,0) = ( 0.0328 *  s_q_RH_WHEEL_);
    (*this)(3,1) = (- 0.0328 *  c_q_RH_WHEEL_);
    (*this)(3,2) = ((- 0.08748 *  s_q_RH_WHEEL_) - ( 0.248 *  c_q_RH_WHEEL_));
    (*this)(3,3) =  c_q_RH_WHEEL_;
    (*this)(3,4) =  s_q_RH_WHEEL_;
    (*this)(4,0) = ( 0.0328 *  c_q_RH_WHEEL_);
    (*this)(4,1) = ( 0.0328 *  s_q_RH_WHEEL_);
    (*this)(4,2) = (( 0.248 *  s_q_RH_WHEEL_) - ( 0.08748 *  c_q_RH_WHEEL_));
    (*this)(4,3) = - s_q_RH_WHEEL_;
    (*this)(4,4) =  c_q_RH_WHEEL_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_WHEEL_L::Type_fr_RH_SHANK_X_fr_RH_WHEEL_L()
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
    (*this)(3,2) = 0.248;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0.08748;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_WHEEL_L& iit::ANYmal::tpl::MotionTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_WHEEL_L::update(const JState& q) {
    Scalar s_q_RH_WHEEL_;
    Scalar c_q_RH_WHEEL_;
    
    s_q_RH_WHEEL_ = TRAIT::sin( q(RH_WHEEL));
    c_q_RH_WHEEL_ = TRAIT::cos( q(RH_WHEEL));
    
    (*this)(0,0) =  c_q_RH_WHEEL_;
    (*this)(0,1) = - s_q_RH_WHEEL_;
    (*this)(1,0) =  s_q_RH_WHEEL_;
    (*this)(1,1) =  c_q_RH_WHEEL_;
    (*this)(3,0) = ( 0.0328 *  s_q_RH_WHEEL_);
    (*this)(3,1) = ( 0.0328 *  c_q_RH_WHEEL_);
    (*this)(3,3) =  c_q_RH_WHEEL_;
    (*this)(3,4) = - s_q_RH_WHEEL_;
    (*this)(4,0) = (- 0.0328 *  c_q_RH_WHEEL_);
    (*this)(4,1) = ( 0.0328 *  s_q_RH_WHEEL_);
    (*this)(4,3) =  s_q_RH_WHEEL_;
    (*this)(4,4) =  c_q_RH_WHEEL_;
    (*this)(5,0) = ((- 0.08748 *  s_q_RH_WHEEL_) - ( 0.248 *  c_q_RH_WHEEL_));
    (*this)(5,1) = (( 0.248 *  s_q_RH_WHEEL_) - ( 0.08748 *  c_q_RH_WHEEL_));
    return *this;
}

template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL_L::Type_fr_base_X_fr_LF_WHEEL_L()
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL_L& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL_L::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_WHEEL_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_WHEEL_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_WHEEL_ = TRAIT::sin( q(LF_WHEEL));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_WHEEL_ = TRAIT::cos( q(LF_WHEEL));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(0,1) = (((( s_q_LF_HFE_ *  s_q_LF_KFE_) - ( c_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(0,3) = ((((((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(0,4) = ((((((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(0,5) = (((((( 0.08748 *  c_q_LF_HFE_) - ( 0.248 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.08748 *  s_q_LF_HFE_) + ( 0.248 *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.28263 *  c_q_LF_HFE_)) + ( 0.104 *  s_q_LF_HAA_));
    (*this)(1,0) = ((((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,1) = (((((- s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = (((((((((- 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.28263 *  c_q_LF_HAA_)) *  s_q_LF_KFE_) + (((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  c_q_LF_HAA_)) *  s_q_LF_WHEEL_) + ((((((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.28263 *  c_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.248 *  c_q_LF_HAA_)) *  c_q_LF_WHEEL_));
    (*this)(1,4) = (((((((( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.28263 *  c_q_LF_HAA_)) *  c_q_LF_KFE_)) + ( 0.248 *  c_q_LF_HAA_)) *  s_q_LF_WHEEL_) + ((((((((- 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.28263 *  c_q_LF_HAA_)) *  s_q_LF_KFE_) + (((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  c_q_LF_HAA_)) *  c_q_LF_WHEEL_));
    (*this)(1,5) = ((((((( 0.08748 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.248 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.248 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08748 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.28263 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.36 *  s_q_LF_HAA_));
    (*this)(2,0) = ((((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(2,1) = ((((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = (((((((((- 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  c_q_LF_HFE_)) + ( 0.28263 *  s_q_LF_HAA_)) *  s_q_LF_KFE_) + ((((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  s_q_LF_HAA_)) *  s_q_LF_WHEEL_) + (((((((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.18979 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.28263 *  s_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.248 *  s_q_LF_HAA_)) *  c_q_LF_WHEEL_));
    (*this)(2,4) = (((((((((- 0.18979 *  c_q_LF_HAA_) -  0.104) *  s_q_LF_HFE_) - (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  c_q_LF_HFE_)) + ( 0.28263 *  s_q_LF_HAA_)) *  c_q_LF_KFE_)) + ( 0.248 *  s_q_LF_HAA_)) *  s_q_LF_WHEEL_) + ((((((((- 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  c_q_LF_HFE_)) + ( 0.28263 *  s_q_LF_HAA_)) *  s_q_LF_KFE_) + ((((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  s_q_LF_HAA_)) *  c_q_LF_WHEEL_));
    (*this)(2,5) = (((((((- 0.08748 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.248 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08748 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.248 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.28263 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.36 *  c_q_LF_HAA_));
    (*this)(3,3) = ((((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(3,4) = (((( s_q_LF_HFE_ *  s_q_LF_KFE_) - ( c_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(4,3) = ((((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(4,4) = (((((- s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(4,5) =  c_q_LF_HAA_;
    (*this)(5,3) = ((((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(5,4) = ((((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_base::Type_fr_LF_WHEEL_L_X_fr_base()
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_WHEEL_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_WHEEL_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_WHEEL_ = TRAIT::sin( q(LF_WHEEL));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_WHEEL_ = TRAIT::cos( q(LF_WHEEL));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( 1.0 *  c_q_LF_HFE_) *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(0,1) = (((((( 1.0 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((( 1.0 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(0,2) = (((((( 1.0 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(0,3) = ((((((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(0,4) = (((((((((- 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.28263 *  c_q_LF_HAA_)) *  s_q_LF_KFE_) + (((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  c_q_LF_HAA_)) *  s_q_LF_WHEEL_) + ((((((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.28263 *  c_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.248 *  c_q_LF_HAA_)) *  c_q_LF_WHEEL_));
    (*this)(0,5) = (((((((((- 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  c_q_LF_HFE_)) + ( 0.28263 *  s_q_LF_HAA_)) *  s_q_LF_KFE_) + ((((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  s_q_LF_HAA_)) *  s_q_LF_WHEEL_) + (((((((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.18979 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.28263 *  s_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.248 *  s_q_LF_HAA_)) *  c_q_LF_WHEEL_));
    (*this)(1,0) = ((((( 1.0 *  s_q_LF_HFE_) *  s_q_LF_KFE_) - ( c_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,1) = (((((- s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((( 1.0 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,2) = (((((( 1.0 *  c_q_LF_HAA_) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((( 1.0 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,3) = ((((((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,4) = (((((((( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.28263 *  c_q_LF_HAA_)) *  c_q_LF_KFE_)) + ( 0.248 *  c_q_LF_HAA_)) *  s_q_LF_WHEEL_) + ((((((((- 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.28263 *  c_q_LF_HAA_)) *  s_q_LF_KFE_) + (((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  c_q_LF_HAA_)) *  c_q_LF_WHEEL_));
    (*this)(1,5) = (((((((((- 0.18979 *  c_q_LF_HAA_) -  0.104) *  s_q_LF_HFE_) - (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((((- 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  c_q_LF_HFE_)) + ( 0.28263 *  s_q_LF_HAA_)) *  c_q_LF_KFE_)) + ( 0.248 *  s_q_LF_HAA_)) *  s_q_LF_WHEEL_) + ((((((((- 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  c_q_LF_HFE_)) + ( 0.28263 *  s_q_LF_HAA_)) *  s_q_LF_KFE_) + ((((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  s_q_LF_HAA_)) *  c_q_LF_WHEEL_));
    (*this)(2,1) =  c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = (((((( 0.08748 *  c_q_LF_HFE_) - ( 0.248 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.08748 *  s_q_LF_HFE_) + ( 0.248 *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.28263 *  c_q_LF_HFE_)) + ( 0.104 *  s_q_LF_HAA_));
    (*this)(2,4) = ((((((( 0.08748 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.248 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.248 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08748 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.28263 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.36 *  s_q_LF_HAA_));
    (*this)(2,5) = (((((((- 0.08748 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.248 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08748 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.248 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.28263 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.36 *  c_q_LF_HAA_));
    (*this)(3,3) = ((((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( 1.0 *  c_q_LF_HFE_) *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(3,4) = (((((( 1.0 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((( 1.0 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(3,5) = (((((( 1.0 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(4,3) = ((((( 1.0 *  s_q_LF_HFE_) *  s_q_LF_KFE_) - ( c_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(4,4) = (((((- s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((( 1.0 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(4,5) = (((((( 1.0 *  c_q_LF_HAA_) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((( 1.0 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(5,4) =  c_q_LF_HAA_;
    (*this)(5,5) =  s_q_LF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL_L::Type_fr_base_X_fr_LH_WHEEL_L()
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL_L& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL_L::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_WHEEL_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_WHEEL_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_WHEEL_ = TRAIT::sin( q(LH_WHEEL));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_WHEEL_ = TRAIT::cos( q(LH_WHEEL));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(0,1) = (((( s_q_LH_HFE_ *  s_q_LH_KFE_) - ( c_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(0,3) = ((((((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(0,4) = ((((((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(0,5) = ((((((- 0.248 *  s_q_LH_HFE_) - ( 0.08748 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.248 *  c_q_LH_HFE_) - ( 0.08748 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.28263 *  c_q_LH_HFE_)) + ( 0.104 *  s_q_LH_HAA_));
    (*this)(1,0) = ((((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,1) = (((((- s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = ((((((((( 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.28263 *  c_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((- 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  c_q_LH_HAA_)) *  s_q_LH_WHEEL_) + (((((((- 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.28263 *  c_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.248 *  c_q_LH_HAA_)) *  c_q_LH_WHEEL_));
    (*this)(1,4) = (((((((( 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((( 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.28263 *  c_q_LH_HAA_)) *  c_q_LH_KFE_)) + ( 0.248 *  c_q_LH_HAA_)) *  s_q_LH_WHEEL_) + (((((((( 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.28263 *  c_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((- 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  c_q_LH_HAA_)) *  c_q_LH_WHEEL_));
    (*this)(1,5) = ((((((( 0.248 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08748 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.248 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08748 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.28263 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.36 *  s_q_LH_HAA_));
    (*this)(2,0) = ((((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(2,1) = ((((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ((((((((( 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  c_q_LH_HFE_)) + ( 0.28263 *  s_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  s_q_LH_HAA_)) *  s_q_LH_WHEEL_) + (((((((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.18979 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.28263 *  s_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.248 *  s_q_LH_HAA_)) *  c_q_LH_WHEEL_));
    (*this)(2,4) = (((((((((- 0.18979 *  c_q_LH_HAA_) -  0.104) *  s_q_LH_HFE_) + (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((( 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  c_q_LH_HFE_)) + ( 0.28263 *  s_q_LH_HAA_)) *  c_q_LH_KFE_)) + ( 0.248 *  s_q_LH_HAA_)) *  s_q_LH_WHEEL_) + (((((((( 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  c_q_LH_HFE_)) + ( 0.28263 *  s_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  s_q_LH_HAA_)) *  c_q_LH_WHEEL_));
    (*this)(2,5) = ((((((( 0.08748 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.248 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.248 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08748 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.28263 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.36 *  c_q_LH_HAA_));
    (*this)(3,3) = ((((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(3,4) = (((( s_q_LH_HFE_ *  s_q_LH_KFE_) - ( c_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(4,3) = ((((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(4,4) = (((((- s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(4,5) =  c_q_LH_HAA_;
    (*this)(5,3) = ((((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(5,4) = ((((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_base::Type_fr_LH_WHEEL_L_X_fr_base()
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_WHEEL_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_WHEEL_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_WHEEL_ = TRAIT::sin( q(LH_WHEEL));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_WHEEL_ = TRAIT::cos( q(LH_WHEEL));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( 1.0 *  c_q_LH_HFE_) *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(0,1) = (((((( 1.0 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((( 1.0 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(0,2) = (((((( 1.0 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(0,3) = ((((((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(0,4) = ((((((((( 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.28263 *  c_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((- 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  c_q_LH_HAA_)) *  s_q_LH_WHEEL_) + (((((((- 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.28263 *  c_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.248 *  c_q_LH_HAA_)) *  c_q_LH_WHEEL_));
    (*this)(0,5) = ((((((((( 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  c_q_LH_HFE_)) + ( 0.28263 *  s_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  s_q_LH_HAA_)) *  s_q_LH_WHEEL_) + (((((((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.18979 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.28263 *  s_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.248 *  s_q_LH_HAA_)) *  c_q_LH_WHEEL_));
    (*this)(1,0) = ((((( 1.0 *  s_q_LH_HFE_) *  s_q_LH_KFE_) - ( c_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,1) = (((((- s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((( 1.0 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,2) = (((((( 1.0 *  c_q_LH_HAA_) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((( 1.0 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,3) = ((((((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,4) = (((((((( 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((( 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.28263 *  c_q_LH_HAA_)) *  c_q_LH_KFE_)) + ( 0.248 *  c_q_LH_HAA_)) *  s_q_LH_WHEEL_) + (((((((( 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.28263 *  c_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((- 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  c_q_LH_HAA_)) *  c_q_LH_WHEEL_));
    (*this)(1,5) = (((((((((- 0.18979 *  c_q_LH_HAA_) -  0.104) *  s_q_LH_HFE_) + (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((( 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  c_q_LH_HFE_)) + ( 0.28263 *  s_q_LH_HAA_)) *  c_q_LH_KFE_)) + ( 0.248 *  s_q_LH_HAA_)) *  s_q_LH_WHEEL_) + (((((((( 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  c_q_LH_HFE_)) + ( 0.28263 *  s_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  s_q_LH_HAA_)) *  c_q_LH_WHEEL_));
    (*this)(2,1) =  c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ((((((- 0.248 *  s_q_LH_HFE_) - ( 0.08748 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.248 *  c_q_LH_HFE_) - ( 0.08748 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.28263 *  c_q_LH_HFE_)) + ( 0.104 *  s_q_LH_HAA_));
    (*this)(2,4) = ((((((( 0.248 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08748 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.248 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08748 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.28263 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.36 *  s_q_LH_HAA_));
    (*this)(2,5) = ((((((( 0.08748 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.248 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.248 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08748 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.28263 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.36 *  c_q_LH_HAA_));
    (*this)(3,3) = ((((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( 1.0 *  c_q_LH_HFE_) *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(3,4) = (((((( 1.0 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((( 1.0 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(3,5) = (((((( 1.0 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(4,3) = ((((( 1.0 *  s_q_LH_HFE_) *  s_q_LH_KFE_) - ( c_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(4,4) = (((((- s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((( 1.0 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(4,5) = (((((( 1.0 *  c_q_LH_HAA_) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((( 1.0 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(5,4) =  c_q_LH_HAA_;
    (*this)(5,5) =  s_q_LH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL_L::Type_fr_base_X_fr_RF_WHEEL_L()
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL_L& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL_L::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_WHEEL_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_WHEEL_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_WHEEL_ = TRAIT::sin( q(RF_WHEEL));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_WHEEL_ = TRAIT::cos( q(RF_WHEEL));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(0,1) = (((( s_q_RF_HFE_ *  s_q_RF_KFE_) - ( c_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(0,3) = (((((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(0,4) = (((((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(0,5) = (((((( 0.08748 *  c_q_RF_HFE_) - ( 0.248 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.08748 *  s_q_RF_HFE_) + ( 0.248 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.28263 *  c_q_RF_HFE_)) - ( 0.104 *  s_q_RF_HAA_));
    (*this)(1,0) = ((((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,1) = (((((- s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = (((((((((- 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.28263 *  c_q_RF_HAA_)) *  s_q_RF_KFE_) + (((( 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  c_q_RF_HAA_)) *  s_q_RF_WHEEL_) + ((((((( 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.28263 *  c_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.248 *  c_q_RF_HAA_)) *  c_q_RF_WHEEL_));
    (*this)(1,4) = ((((((((- 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((((- 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.28263 *  c_q_RF_HAA_)) *  c_q_RF_KFE_)) + ( 0.248 *  c_q_RF_HAA_)) *  s_q_RF_WHEEL_) + ((((((((- 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.28263 *  c_q_RF_HAA_)) *  s_q_RF_KFE_) + (((( 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  c_q_RF_HAA_)) *  c_q_RF_WHEEL_));
    (*this)(1,5) = ((((((( 0.08748 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.248 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.248 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08748 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.28263 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.36 *  s_q_RF_HAA_));
    (*this)(2,0) = ((((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(2,1) = ((((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (((((((((- 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  c_q_RF_HFE_)) + ( 0.28263 *  s_q_RF_HAA_)) *  s_q_RF_KFE_) + (((((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  s_q_RF_HAA_)) *  s_q_RF_WHEEL_) + ((((((((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.18979 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.28263 *  s_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.248 *  s_q_RF_HAA_)) *  c_q_RF_WHEEL_));
    (*this)(2,4) = ((((((((( 0.18979 *  c_q_RF_HAA_) +  0.104) *  s_q_RF_HFE_) - (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((((- 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  c_q_RF_HFE_)) + ( 0.28263 *  s_q_RF_HAA_)) *  c_q_RF_KFE_)) + ( 0.248 *  s_q_RF_HAA_)) *  s_q_RF_WHEEL_) + ((((((((- 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  c_q_RF_HFE_)) + ( 0.28263 *  s_q_RF_HAA_)) *  s_q_RF_KFE_) + (((((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  s_q_RF_HAA_)) *  c_q_RF_WHEEL_));
    (*this)(2,5) = (((((((- 0.08748 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.248 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08748 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.248 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.28263 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.36 *  c_q_RF_HAA_));
    (*this)(3,3) = ((((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(3,4) = (((( s_q_RF_HFE_ *  s_q_RF_KFE_) - ( c_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(4,3) = ((((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(4,4) = (((((- s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(4,5) =  c_q_RF_HAA_;
    (*this)(5,3) = ((((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(5,4) = ((((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_base::Type_fr_RF_WHEEL_L_X_fr_base()
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_WHEEL_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_WHEEL_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_WHEEL_ = TRAIT::sin( q(RF_WHEEL));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_WHEEL_ = TRAIT::cos( q(RF_WHEEL));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( 1.0 *  c_q_RF_HFE_) *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(0,1) = (((((( 1.0 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((( 1.0 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(0,2) = (((((( 1.0 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(0,3) = (((((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(0,4) = (((((((((- 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.28263 *  c_q_RF_HAA_)) *  s_q_RF_KFE_) + (((( 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  c_q_RF_HAA_)) *  s_q_RF_WHEEL_) + ((((((( 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.28263 *  c_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.248 *  c_q_RF_HAA_)) *  c_q_RF_WHEEL_));
    (*this)(0,5) = (((((((((- 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  c_q_RF_HFE_)) + ( 0.28263 *  s_q_RF_HAA_)) *  s_q_RF_KFE_) + (((((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  s_q_RF_HAA_)) *  s_q_RF_WHEEL_) + ((((((((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.18979 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.28263 *  s_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.248 *  s_q_RF_HAA_)) *  c_q_RF_WHEEL_));
    (*this)(1,0) = ((((( 1.0 *  s_q_RF_HFE_) *  s_q_RF_KFE_) - ( c_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,1) = (((((- s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((( 1.0 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,2) = (((((( 1.0 *  c_q_RF_HAA_) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((( 1.0 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,3) = (((((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,4) = ((((((((- 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((((- 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.28263 *  c_q_RF_HAA_)) *  c_q_RF_KFE_)) + ( 0.248 *  c_q_RF_HAA_)) *  s_q_RF_WHEEL_) + ((((((((- 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.28263 *  c_q_RF_HAA_)) *  s_q_RF_KFE_) + (((( 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  c_q_RF_HAA_)) *  c_q_RF_WHEEL_));
    (*this)(1,5) = ((((((((( 0.18979 *  c_q_RF_HAA_) +  0.104) *  s_q_RF_HFE_) - (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((((- 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  c_q_RF_HFE_)) + ( 0.28263 *  s_q_RF_HAA_)) *  c_q_RF_KFE_)) + ( 0.248 *  s_q_RF_HAA_)) *  s_q_RF_WHEEL_) + ((((((((- 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  c_q_RF_HFE_)) + ( 0.28263 *  s_q_RF_HAA_)) *  s_q_RF_KFE_) + (((((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  s_q_RF_HAA_)) *  c_q_RF_WHEEL_));
    (*this)(2,1) =  c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (((((( 0.08748 *  c_q_RF_HFE_) - ( 0.248 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.08748 *  s_q_RF_HFE_) + ( 0.248 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.28263 *  c_q_RF_HFE_)) - ( 0.104 *  s_q_RF_HAA_));
    (*this)(2,4) = ((((((( 0.08748 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.248 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.248 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08748 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.28263 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.36 *  s_q_RF_HAA_));
    (*this)(2,5) = (((((((- 0.08748 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.248 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08748 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.248 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.28263 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.36 *  c_q_RF_HAA_));
    (*this)(3,3) = ((((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( 1.0 *  c_q_RF_HFE_) *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(3,4) = (((((( 1.0 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((( 1.0 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(3,5) = (((((( 1.0 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(4,3) = ((((( 1.0 *  s_q_RF_HFE_) *  s_q_RF_KFE_) - ( c_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(4,4) = (((((- s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((( 1.0 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(4,5) = (((((( 1.0 *  c_q_RF_HAA_) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((( 1.0 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(5,4) =  c_q_RF_HAA_;
    (*this)(5,5) =  s_q_RF_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL_L::Type_fr_base_X_fr_RH_WHEEL_L()
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL_L& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL_L::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_WHEEL_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_WHEEL_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_WHEEL_ = TRAIT::sin( q(RH_WHEEL));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_WHEEL_ = TRAIT::cos( q(RH_WHEEL));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(0,1) = (((( s_q_RH_HFE_ *  s_q_RH_KFE_) - ( c_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(0,3) = (((((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(0,4) = (((((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(0,5) = ((((((- 0.248 *  s_q_RH_HFE_) - ( 0.08748 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.248 *  c_q_RH_HFE_) - ( 0.08748 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.28263 *  c_q_RH_HFE_)) - ( 0.104 *  s_q_RH_HAA_));
    (*this)(1,0) = ((((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,1) = (((((- s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = ((((((((( 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.28263 *  c_q_RH_HAA_)) *  s_q_RH_KFE_) + (((( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  c_q_RH_HAA_)) *  s_q_RH_WHEEL_) + ((((((( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.28263 *  c_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.248 *  c_q_RH_HAA_)) *  c_q_RH_WHEEL_));
    (*this)(1,4) = (((((((( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((( 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.28263 *  c_q_RH_HAA_)) *  c_q_RH_KFE_)) + ( 0.248 *  c_q_RH_HAA_)) *  s_q_RH_WHEEL_) + (((((((( 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.28263 *  c_q_RH_HAA_)) *  s_q_RH_KFE_) + (((( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  c_q_RH_HAA_)) *  c_q_RH_WHEEL_));
    (*this)(1,5) = ((((((( 0.248 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08748 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.248 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08748 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.28263 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.36 *  s_q_RH_HAA_));
    (*this)(2,0) = ((((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(2,1) = ((((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = ((((((((( 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  c_q_RH_HFE_)) + ( 0.28263 *  s_q_RH_HAA_)) *  s_q_RH_KFE_) + (((((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  s_q_RH_HAA_)) *  s_q_RH_WHEEL_) + ((((((((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.18979 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.28263 *  s_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.248 *  s_q_RH_HAA_)) *  c_q_RH_WHEEL_));
    (*this)(2,4) = ((((((((( 0.18979 *  c_q_RH_HAA_) +  0.104) *  s_q_RH_HFE_) + (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((( 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  c_q_RH_HFE_)) + ( 0.28263 *  s_q_RH_HAA_)) *  c_q_RH_KFE_)) + ( 0.248 *  s_q_RH_HAA_)) *  s_q_RH_WHEEL_) + (((((((( 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  c_q_RH_HFE_)) + ( 0.28263 *  s_q_RH_HAA_)) *  s_q_RH_KFE_) + (((((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  s_q_RH_HAA_)) *  c_q_RH_WHEEL_));
    (*this)(2,5) = ((((((( 0.08748 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.248 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.248 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08748 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.28263 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.36 *  c_q_RH_HAA_));
    (*this)(3,3) = ((((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(3,4) = (((( s_q_RH_HFE_ *  s_q_RH_KFE_) - ( c_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(4,3) = ((((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(4,4) = (((((- s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(4,5) =  c_q_RH_HAA_;
    (*this)(5,3) = ((((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(5,4) = ((((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_base::Type_fr_RH_WHEEL_L_X_fr_base()
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_WHEEL_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_WHEEL_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_WHEEL_ = TRAIT::sin( q(RH_WHEEL));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_WHEEL_ = TRAIT::cos( q(RH_WHEEL));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( 1.0 *  c_q_RH_HFE_) *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(0,1) = (((((( 1.0 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((( 1.0 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(0,2) = (((((( 1.0 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(0,3) = (((((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(0,4) = ((((((((( 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.28263 *  c_q_RH_HAA_)) *  s_q_RH_KFE_) + (((( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  c_q_RH_HAA_)) *  s_q_RH_WHEEL_) + ((((((( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.28263 *  c_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.248 *  c_q_RH_HAA_)) *  c_q_RH_WHEEL_));
    (*this)(0,5) = ((((((((( 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  c_q_RH_HFE_)) + ( 0.28263 *  s_q_RH_HAA_)) *  s_q_RH_KFE_) + (((((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  s_q_RH_HAA_)) *  s_q_RH_WHEEL_) + ((((((((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.18979 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.28263 *  s_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.248 *  s_q_RH_HAA_)) *  c_q_RH_WHEEL_));
    (*this)(1,0) = ((((( 1.0 *  s_q_RH_HFE_) *  s_q_RH_KFE_) - ( c_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,1) = (((((- s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((( 1.0 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,2) = (((((( 1.0 *  c_q_RH_HAA_) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((( 1.0 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,3) = (((((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + ((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,4) = (((((((( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((( 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.28263 *  c_q_RH_HAA_)) *  c_q_RH_KFE_)) + ( 0.248 *  c_q_RH_HAA_)) *  s_q_RH_WHEEL_) + (((((((( 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.28263 *  c_q_RH_HAA_)) *  s_q_RH_KFE_) + (((( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  c_q_RH_HAA_)) *  c_q_RH_WHEEL_));
    (*this)(1,5) = ((((((((( 0.18979 *  c_q_RH_HAA_) +  0.104) *  s_q_RH_HFE_) + (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((( 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  c_q_RH_HFE_)) + ( 0.28263 *  s_q_RH_HAA_)) *  c_q_RH_KFE_)) + ( 0.248 *  s_q_RH_HAA_)) *  s_q_RH_WHEEL_) + (((((((( 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  c_q_RH_HFE_)) + ( 0.28263 *  s_q_RH_HAA_)) *  s_q_RH_KFE_) + (((((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  s_q_RH_HAA_)) *  c_q_RH_WHEEL_));
    (*this)(2,1) =  c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = ((((((- 0.248 *  s_q_RH_HFE_) - ( 0.08748 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.248 *  c_q_RH_HFE_) - ( 0.08748 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.28263 *  c_q_RH_HFE_)) - ( 0.104 *  s_q_RH_HAA_));
    (*this)(2,4) = ((((((( 0.248 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08748 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.248 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08748 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.28263 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.36 *  s_q_RH_HAA_));
    (*this)(2,5) = ((((((( 0.08748 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.248 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.248 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08748 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.28263 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.36 *  c_q_RH_HAA_));
    (*this)(3,3) = ((((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( 1.0 *  c_q_RH_HFE_) *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(3,4) = (((((( 1.0 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((( 1.0 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(3,5) = (((((( 1.0 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(4,3) = ((((( 1.0 *  s_q_RH_HFE_) *  s_q_RH_KFE_) - ( c_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(4,4) = (((((- s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((( 1.0 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(4,5) = (((((( 1.0 *  c_q_RH_HAA_) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((( 1.0 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(5,4) =  c_q_RH_HAA_;
    (*this)(5,5) =  s_q_RH_HAA_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA::Type_fr_base_X_fr_LF_HAA()
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
    (*this)(1,3) = 0.3;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0.3;
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
    
    (*this)(0,4) = ((- 0.104 *  c_q_LF_HAA_) -  0.0701);
    (*this)(0,5) = ( 0.104 *  s_q_LF_HAA_);
    (*this)(1,1) =  s_q_LF_HAA_;
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = ( 0.0701 *  s_q_LF_HAA_);
    (*this)(1,4) = ( 0.36 *  c_q_LF_HAA_);
    (*this)(1,5) = (- 0.36 *  s_q_LF_HAA_);
    (*this)(2,1) = - c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = ((- 0.0701 *  c_q_LF_HAA_) -  0.104);
    (*this)(2,4) = ( 0.36 *  s_q_LF_HAA_);
    (*this)(2,5) = ( 0.36 *  c_q_LF_HAA_);
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
    (*this)(0,3) = (((- 0.104 *  c_q_LF_HAA_) -  0.15699) *  s_q_LF_HFE_);
    (*this)(0,4) = (((- 0.104 *  c_q_LF_HAA_) -  0.15699) *  c_q_LF_HFE_);
    (*this)(0,5) = (( 0.28263 *  c_q_LF_HFE_) + ( 0.104 *  s_q_LF_HAA_));
    (*this)(1,0) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(1,1) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = (((( 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.15699 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.28263 *  c_q_LF_HAA_));
    (*this)(1,4) = ((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.15699 *  s_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(1,5) = ((( 0.28263 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - ( 0.36 *  s_q_LF_HAA_));
    (*this)(2,0) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(2,1) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = (((( 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.15699 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.28263 *  s_q_LF_HAA_));
    (*this)(2,4) = (((( 0.15699 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_));
    (*this)(2,5) = (( 0.36 *  c_q_LF_HAA_) - (( 0.28263 *  c_q_LF_HAA_) *  s_q_LF_HFE_));
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
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL::Type_fr_base_X_fr_LF_WHEEL()
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL::update(const JState& q) {
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
    (*this)(0,3) = (((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,4) = ((((( 0.104 *  c_q_LF_HAA_) +  0.18979) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((((- 0.104 *  c_q_LF_HAA_) -  0.18979) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(0,5) = (((((( 0.08748 *  c_q_LF_HFE_) - ( 0.248 *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.08748 *  s_q_LF_HFE_) + ( 0.248 *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.28263 *  c_q_LF_HFE_)) + ( 0.104 *  s_q_LF_HAA_));
    (*this)(1,0) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = (((((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.28263 *  c_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.248 *  c_q_LF_HAA_));
    (*this)(1,4) = (((((((- 0.36 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.28263 *  c_q_LF_HAA_)) *  s_q_LF_KFE_) + (((( 0.36 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.18979 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  c_q_LF_HAA_));
    (*this)(1,5) = ((((((( 0.08748 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.248 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.248 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08748 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.28263 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) - ( 0.36 *  s_q_LF_HAA_));
    (*this)(2,0) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) = ((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = ((((((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((( 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (((- 0.18979 *  c_q_LF_HAA_) -  0.104) *  c_q_LF_HFE_)) - ( 0.28263 *  s_q_LF_HAA_)) *  c_q_LF_KFE_)) - ( 0.248 *  s_q_LF_HAA_));
    (*this)(2,4) = (((((((- 0.36 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  c_q_LF_HFE_)) + ( 0.28263 *  s_q_LF_HAA_)) *  s_q_LF_KFE_) + ((((( 0.18979 *  c_q_LF_HAA_) +  0.104) *  s_q_LF_HFE_) + (( 0.36 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + ( 0.08748 *  s_q_LF_HAA_));
    (*this)(2,5) = (((((((- 0.08748 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.248 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08748 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.248 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.28263 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) + ( 0.36 *  c_q_LF_HAA_));
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
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA::Type_fr_base_X_fr_LH_HAA()
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
    (*this)(1,3) = - 0.3;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.3;
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
    
    (*this)(0,4) = ((- 0.104 *  c_q_LH_HAA_) -  0.0701);
    (*this)(0,5) = ( 0.104 *  s_q_LH_HAA_);
    (*this)(1,1) =  s_q_LH_HAA_;
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = ( 0.0701 *  s_q_LH_HAA_);
    (*this)(1,4) = (- 0.36 *  c_q_LH_HAA_);
    (*this)(1,5) = ( 0.36 *  s_q_LH_HAA_);
    (*this)(2,1) = - c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ((- 0.0701 *  c_q_LH_HAA_) -  0.104);
    (*this)(2,4) = (- 0.36 *  s_q_LH_HAA_);
    (*this)(2,5) = (- 0.36 *  c_q_LH_HAA_);
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
    (*this)(0,3) = (((- 0.104 *  c_q_LH_HAA_) -  0.15699) *  s_q_LH_HFE_);
    (*this)(0,4) = (((- 0.104 *  c_q_LH_HAA_) -  0.15699) *  c_q_LH_HFE_);
    (*this)(0,5) = (( 0.28263 *  c_q_LH_HFE_) + ( 0.104 *  s_q_LH_HAA_));
    (*this)(1,0) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(1,1) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = ((((- 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.15699 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.28263 *  c_q_LH_HAA_));
    (*this)(1,4) = (((- 0.15699 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(1,5) = ((( 0.28263 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ( 0.36 *  s_q_LH_HAA_));
    (*this)(2,0) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(2,1) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ((((- 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.15699 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.28263 *  s_q_LH_HAA_));
    (*this)(2,4) = (((( 0.15699 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_));
    (*this)(2,5) = (((- 0.28263 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - ( 0.36 *  c_q_LH_HAA_));
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
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL::Type_fr_base_X_fr_LH_WHEEL()
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL::update(const JState& q) {
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
    (*this)(0,3) = (((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,4) = ((((( 0.104 *  c_q_LH_HAA_) +  0.18979) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((((- 0.104 *  c_q_LH_HAA_) -  0.18979) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(0,5) = ((((((- 0.248 *  s_q_LH_HFE_) - ( 0.08748 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.248 *  c_q_LH_HFE_) - ( 0.08748 *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + ( 0.28263 *  c_q_LH_HFE_)) + ( 0.104 *  s_q_LH_HAA_));
    (*this)(1,0) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = ((((((- 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.28263 *  c_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.248 *  c_q_LH_HAA_));
    (*this)(1,4) = ((((((( 0.36 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.18979 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.28263 *  c_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((- 0.18979 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.36 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  c_q_LH_HAA_));
    (*this)(1,5) = ((((((( 0.248 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08748 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.248 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08748 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.28263 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) + ( 0.36 *  s_q_LH_HAA_));
    (*this)(2,0) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) = ((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ((((((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((((- 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (((- 0.18979 *  c_q_LH_HAA_) -  0.104) *  c_q_LH_HFE_)) - ( 0.28263 *  s_q_LH_HAA_)) *  c_q_LH_KFE_)) - ( 0.248 *  s_q_LH_HAA_));
    (*this)(2,4) = ((((((( 0.36 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + ((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  c_q_LH_HFE_)) + ( 0.28263 *  s_q_LH_HAA_)) *  s_q_LH_KFE_) + ((((( 0.18979 *  c_q_LH_HAA_) +  0.104) *  s_q_LH_HFE_) - (( 0.36 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.08748 *  s_q_LH_HAA_));
    (*this)(2,5) = ((((((( 0.08748 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.248 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.248 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08748 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.28263 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) - ( 0.36 *  c_q_LH_HAA_));
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
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA::Type_fr_base_X_fr_RF_HAA()
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
    (*this)(1,3) = 0.3;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0.3;
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
    
    (*this)(0,4) = (( 0.104 *  c_q_RF_HAA_) +  0.0701);
    (*this)(0,5) = (- 0.104 *  s_q_RF_HAA_);
    (*this)(1,1) =  s_q_RF_HAA_;
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = (- 0.0701 *  s_q_RF_HAA_);
    (*this)(1,4) = ( 0.36 *  c_q_RF_HAA_);
    (*this)(1,5) = (- 0.36 *  s_q_RF_HAA_);
    (*this)(2,1) = - c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (( 0.0701 *  c_q_RF_HAA_) +  0.104);
    (*this)(2,4) = ( 0.36 *  s_q_RF_HAA_);
    (*this)(2,5) = ( 0.36 *  c_q_RF_HAA_);
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
    (*this)(0,3) = ((( 0.104 *  c_q_RF_HAA_) +  0.15699) *  s_q_RF_HFE_);
    (*this)(0,4) = ((( 0.104 *  c_q_RF_HAA_) +  0.15699) *  c_q_RF_HFE_);
    (*this)(0,5) = (( 0.28263 *  c_q_RF_HFE_) - ( 0.104 *  s_q_RF_HAA_));
    (*this)(1,0) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(1,1) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = (((( 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.15699 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.28263 *  c_q_RF_HAA_));
    (*this)(1,4) = ((( 0.15699 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(1,5) = ((( 0.28263 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.36 *  s_q_RF_HAA_));
    (*this)(2,0) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(2,1) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (((( 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.15699 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.28263 *  s_q_RF_HAA_));
    (*this)(2,4) = ((((- 0.15699 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(2,5) = (( 0.36 *  c_q_RF_HAA_) - (( 0.28263 *  c_q_RF_HAA_) *  s_q_RF_HFE_));
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
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL::Type_fr_base_X_fr_RF_WHEEL()
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL::update(const JState& q) {
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
    (*this)(0,3) = ((((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,4) = (((((- 0.104 *  c_q_RF_HAA_) -  0.18979) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + (((( 0.104 *  c_q_RF_HAA_) +  0.18979) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(0,5) = (((((( 0.08748 *  c_q_RF_HFE_) - ( 0.248 *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.08748 *  s_q_RF_HFE_) + ( 0.248 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.28263 *  c_q_RF_HFE_)) - ( 0.104 *  s_q_RF_HAA_));
    (*this)(1,0) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = (((((( 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.28263 *  c_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.248 *  c_q_RF_HAA_));
    (*this)(1,4) = (((((((- 0.36 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.18979 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.28263 *  c_q_RF_HAA_)) *  s_q_RF_KFE_) + (((( 0.18979 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.36 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  c_q_RF_HAA_));
    (*this)(1,5) = ((((((( 0.08748 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.248 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.248 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08748 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.28263 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) - ( 0.36 *  s_q_RF_HAA_));
    (*this)(2,0) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) = ((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (((((((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((( 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + ((( 0.18979 *  c_q_RF_HAA_) +  0.104) *  c_q_RF_HFE_)) - ( 0.28263 *  s_q_RF_HAA_)) *  c_q_RF_KFE_)) - ( 0.248 *  s_q_RF_HAA_));
    (*this)(2,4) = (((((((- 0.36 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  c_q_RF_HFE_)) + ( 0.28263 *  s_q_RF_HAA_)) *  s_q_RF_KFE_) + (((((- 0.18979 *  c_q_RF_HAA_) -  0.104) *  s_q_RF_HFE_) + (( 0.36 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + ( 0.08748 *  s_q_RF_HAA_));
    (*this)(2,5) = (((((((- 0.08748 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.248 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08748 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.248 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.28263 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) + ( 0.36 *  c_q_RF_HAA_));
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
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA::Type_fr_base_X_fr_RH_HAA()
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
    (*this)(1,3) = - 0.3;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.3;
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
    
    (*this)(0,4) = (( 0.104 *  c_q_RH_HAA_) +  0.0701);
    (*this)(0,5) = (- 0.104 *  s_q_RH_HAA_);
    (*this)(1,1) =  s_q_RH_HAA_;
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = (- 0.0701 *  s_q_RH_HAA_);
    (*this)(1,4) = (- 0.36 *  c_q_RH_HAA_);
    (*this)(1,5) = ( 0.36 *  s_q_RH_HAA_);
    (*this)(2,1) = - c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (( 0.0701 *  c_q_RH_HAA_) +  0.104);
    (*this)(2,4) = (- 0.36 *  s_q_RH_HAA_);
    (*this)(2,5) = (- 0.36 *  c_q_RH_HAA_);
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
    (*this)(0,3) = ((( 0.104 *  c_q_RH_HAA_) +  0.15699) *  s_q_RH_HFE_);
    (*this)(0,4) = ((( 0.104 *  c_q_RH_HAA_) +  0.15699) *  c_q_RH_HFE_);
    (*this)(0,5) = (( 0.28263 *  c_q_RH_HFE_) - ( 0.104 *  s_q_RH_HAA_));
    (*this)(1,0) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(1,1) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = ((((- 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.15699 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.28263 *  c_q_RH_HAA_));
    (*this)(1,4) = ((( 0.15699 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(1,5) = ((( 0.28263 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ( 0.36 *  s_q_RH_HAA_));
    (*this)(2,0) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(2,1) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = ((((- 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.15699 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.28263 *  s_q_RH_HAA_));
    (*this)(2,4) = ((((- 0.15699 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_));
    (*this)(2,5) = (((- 0.28263 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - ( 0.36 *  c_q_RH_HAA_));
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
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL::Type_fr_base_X_fr_RH_WHEEL()
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL::update(const JState& q) {
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
    (*this)(0,3) = ((((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,4) = (((((- 0.104 *  c_q_RH_HAA_) -  0.18979) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + (((( 0.104 *  c_q_RH_HAA_) +  0.18979) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(0,5) = ((((((- 0.248 *  s_q_RH_HFE_) - ( 0.08748 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.248 *  c_q_RH_HFE_) - ( 0.08748 *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + ( 0.28263 *  c_q_RH_HFE_)) - ( 0.104 *  s_q_RH_HAA_));
    (*this)(1,0) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = (((((( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.28263 *  c_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.248 *  c_q_RH_HAA_));
    (*this)(1,4) = ((((((( 0.36 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.18979 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.28263 *  c_q_RH_HAA_)) *  s_q_RH_KFE_) + (((( 0.18979 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.36 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  c_q_RH_HAA_));
    (*this)(1,5) = ((((((( 0.248 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08748 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.248 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08748 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.28263 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) + ( 0.36 *  s_q_RH_HAA_));
    (*this)(2,0) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) = ((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (((((((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((((- 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ((( 0.18979 *  c_q_RH_HAA_) +  0.104) *  c_q_RH_HFE_)) - ( 0.28263 *  s_q_RH_HAA_)) *  c_q_RH_KFE_)) - ( 0.248 *  s_q_RH_HAA_));
    (*this)(2,4) = ((((((( 0.36 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  c_q_RH_HFE_)) + ( 0.28263 *  s_q_RH_HAA_)) *  s_q_RH_KFE_) + (((((- 0.18979 *  c_q_RH_HAA_) -  0.104) *  s_q_RH_HFE_) - (( 0.36 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.08748 *  s_q_RH_HAA_));
    (*this)(2,5) = ((((((( 0.08748 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.248 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.248 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08748 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.28263 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) - ( 0.36 *  c_q_RH_HAA_));
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
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base::Type_fr_LF_HIP_X_fr_base()
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,1) =  s_q_LF_HAA_;
    (*this)(0,2) = - c_q_LF_HAA_;
    (*this)(0,3) = (- 0.104 *  c_q_LF_HAA_);
    (*this)(0,4) = ( 0.3 *  c_q_LF_HAA_);
    (*this)(0,5) = ( 0.3 *  s_q_LF_HAA_);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = ( 0.104 *  s_q_LF_HAA_);
    (*this)(1,4) = (- 0.3 *  s_q_LF_HAA_);
    (*this)(1,5) = ( 0.3 *  c_q_LF_HAA_);
    (*this)(3,4) =  s_q_LF_HAA_;
    (*this)(3,5) = - c_q_LF_HAA_;
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(4,5) =  s_q_LF_HAA_;
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP::update(const JState& q) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,3) = (- 0.104 *  c_q_LF_HAA_);
    (*this)(0,4) = ( 0.104 *  s_q_LF_HAA_);
    (*this)(1,0) =  s_q_LF_HAA_;
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,3) = ( 0.3 *  c_q_LF_HAA_);
    (*this)(1,4) = (- 0.3 *  s_q_LF_HAA_);
    (*this)(2,0) = - c_q_LF_HAA_;
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,3) = ( 0.3 *  s_q_LF_HAA_);
    (*this)(2,4) = ( 0.3 *  c_q_LF_HAA_);
    (*this)(4,3) =  s_q_LF_HAA_;
    (*this)(4,4) =  c_q_LF_HAA_;
    (*this)(5,3) = - c_q_LF_HAA_;
    (*this)(5,4) =  s_q_LF_HAA_;
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
    (*this)(2,3) = - 0.06;
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
    (*this)(0,3) = ( 0.0701 *  c_q_LF_HFE_);
    (*this)(0,4) = ( 0.06 *  s_q_LF_HFE_);
    (*this)(0,5) = (- 0.0701 *  s_q_LF_HFE_);
    (*this)(1,0) =  c_q_LF_HFE_;
    (*this)(1,2) = - s_q_LF_HFE_;
    (*this)(1,3) = (- 0.0701 *  s_q_LF_HFE_);
    (*this)(1,4) = ( 0.06 *  c_q_LF_HFE_);
    (*this)(1,5) = (- 0.0701 *  c_q_LF_HFE_);
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
    (*this)(0,5) = - 0.06;
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
    (*this)(0,3) = ( 0.0701 *  c_q_LF_HFE_);
    (*this)(0,4) = (- 0.0701 *  s_q_LF_HFE_);
    (*this)(1,3) = ( 0.06 *  s_q_LF_HFE_);
    (*this)(1,4) = ( 0.06 *  c_q_LF_HFE_);
    (*this)(2,0) =  c_q_LF_HFE_;
    (*this)(2,1) = - s_q_LF_HFE_;
    (*this)(2,3) = (- 0.0701 *  s_q_LF_HFE_);
    (*this)(2,4) = (- 0.0701 *  c_q_LF_HFE_);
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
    (*this)(2,3) = 0.28263;
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
    (*this)(0,3) = (- 0.08689 *  s_q_LF_KFE_);
    (*this)(0,4) = ( 0.08689 *  c_q_LF_KFE_);
    (*this)(0,5) = (- 0.28263 *  c_q_LF_KFE_);
    (*this)(1,0) = - s_q_LF_KFE_;
    (*this)(1,1) =  c_q_LF_KFE_;
    (*this)(1,3) = (- 0.08689 *  c_q_LF_KFE_);
    (*this)(1,4) = (- 0.08689 *  s_q_LF_KFE_);
    (*this)(1,5) = ( 0.28263 *  s_q_LF_KFE_);
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
    (*this)(0,5) = 0.28263;
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
    (*this)(0,3) = (- 0.08689 *  s_q_LF_KFE_);
    (*this)(0,4) = (- 0.08689 *  c_q_LF_KFE_);
    (*this)(1,0) =  s_q_LF_KFE_;
    (*this)(1,1) =  c_q_LF_KFE_;
    (*this)(1,3) = ( 0.08689 *  c_q_LF_KFE_);
    (*this)(1,4) = (- 0.08689 *  s_q_LF_KFE_);
    (*this)(2,3) = (- 0.28263 *  c_q_LF_KFE_);
    (*this)(2,4) = ( 0.28263 *  s_q_LF_KFE_);
    (*this)(3,3) =  c_q_LF_KFE_;
    (*this)(3,4) = - s_q_LF_KFE_;
    (*this)(4,3) =  s_q_LF_KFE_;
    (*this)(4,4) =  c_q_LF_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_LF_SHANK::Type_fr_LF_WHEEL_L_X_fr_LF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.248;
    (*this)(2,4) = - 0.08748;
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_LF_SHANK& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_LF_SHANK::update(const JState& q) {
    Scalar s_q_LF_WHEEL_;
    Scalar c_q_LF_WHEEL_;
    
    s_q_LF_WHEEL_ = TRAIT::sin( q(LF_WHEEL));
    c_q_LF_WHEEL_ = TRAIT::cos( q(LF_WHEEL));
    
    (*this)(0,0) =  c_q_LF_WHEEL_;
    (*this)(0,1) =  s_q_LF_WHEEL_;
    (*this)(0,3) = (- 0.0328 *  s_q_LF_WHEEL_);
    (*this)(0,4) = ( 0.0328 *  c_q_LF_WHEEL_);
    (*this)(0,5) = (( 0.08748 *  s_q_LF_WHEEL_) - ( 0.248 *  c_q_LF_WHEEL_));
    (*this)(1,0) = - s_q_LF_WHEEL_;
    (*this)(1,1) =  c_q_LF_WHEEL_;
    (*this)(1,3) = (- 0.0328 *  c_q_LF_WHEEL_);
    (*this)(1,4) = (- 0.0328 *  s_q_LF_WHEEL_);
    (*this)(1,5) = (( 0.248 *  s_q_LF_WHEEL_) + ( 0.08748 *  c_q_LF_WHEEL_));
    (*this)(3,3) =  c_q_LF_WHEEL_;
    (*this)(3,4) =  s_q_LF_WHEEL_;
    (*this)(4,3) = - s_q_LF_WHEEL_;
    (*this)(4,4) =  c_q_LF_WHEEL_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_WHEEL_L::Type_fr_LF_SHANK_X_fr_LF_WHEEL_L()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0.248;
    (*this)(1,2) = 0;
    (*this)(1,5) = - 0.08748;
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_WHEEL_L& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_WHEEL_L::update(const JState& q) {
    Scalar s_q_LF_WHEEL_;
    Scalar c_q_LF_WHEEL_;
    
    s_q_LF_WHEEL_ = TRAIT::sin( q(LF_WHEEL));
    c_q_LF_WHEEL_ = TRAIT::cos( q(LF_WHEEL));
    
    (*this)(0,0) =  c_q_LF_WHEEL_;
    (*this)(0,1) = - s_q_LF_WHEEL_;
    (*this)(0,3) = (- 0.0328 *  s_q_LF_WHEEL_);
    (*this)(0,4) = (- 0.0328 *  c_q_LF_WHEEL_);
    (*this)(1,0) =  s_q_LF_WHEEL_;
    (*this)(1,1) =  c_q_LF_WHEEL_;
    (*this)(1,3) = ( 0.0328 *  c_q_LF_WHEEL_);
    (*this)(1,4) = (- 0.0328 *  s_q_LF_WHEEL_);
    (*this)(2,3) = (( 0.08748 *  s_q_LF_WHEEL_) - ( 0.248 *  c_q_LF_WHEEL_));
    (*this)(2,4) = (( 0.248 *  s_q_LF_WHEEL_) + ( 0.08748 *  c_q_LF_WHEEL_));
    (*this)(3,3) =  c_q_LF_WHEEL_;
    (*this)(3,4) = - s_q_LF_WHEEL_;
    (*this)(4,3) =  s_q_LF_WHEEL_;
    (*this)(4,4) =  c_q_LF_WHEEL_;
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,1) =  s_q_RF_HAA_;
    (*this)(0,2) = - c_q_RF_HAA_;
    (*this)(0,3) = ( 0.104 *  c_q_RF_HAA_);
    (*this)(0,4) = ( 0.3 *  c_q_RF_HAA_);
    (*this)(0,5) = ( 0.3 *  s_q_RF_HAA_);
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = (- 0.104 *  s_q_RF_HAA_);
    (*this)(1,4) = (- 0.3 *  s_q_RF_HAA_);
    (*this)(1,5) = ( 0.3 *  c_q_RF_HAA_);
    (*this)(3,4) =  s_q_RF_HAA_;
    (*this)(3,5) = - c_q_RF_HAA_;
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(4,5) =  s_q_RF_HAA_;
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP::update(const JState& q) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,3) = ( 0.104 *  c_q_RF_HAA_);
    (*this)(0,4) = (- 0.104 *  s_q_RF_HAA_);
    (*this)(1,0) =  s_q_RF_HAA_;
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,3) = ( 0.3 *  c_q_RF_HAA_);
    (*this)(1,4) = (- 0.3 *  s_q_RF_HAA_);
    (*this)(2,0) = - c_q_RF_HAA_;
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,3) = ( 0.3 *  s_q_RF_HAA_);
    (*this)(2,4) = ( 0.3 *  c_q_RF_HAA_);
    (*this)(4,3) =  s_q_RF_HAA_;
    (*this)(4,4) =  c_q_RF_HAA_;
    (*this)(5,3) = - c_q_RF_HAA_;
    (*this)(5,4) =  s_q_RF_HAA_;
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
    (*this)(2,3) = - 0.06;
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
    (*this)(0,3) = (- 0.0701 *  c_q_RF_HFE_);
    (*this)(0,4) = ( 0.06 *  s_q_RF_HFE_);
    (*this)(0,5) = ( 0.0701 *  s_q_RF_HFE_);
    (*this)(1,0) =  c_q_RF_HFE_;
    (*this)(1,2) = - s_q_RF_HFE_;
    (*this)(1,3) = ( 0.0701 *  s_q_RF_HFE_);
    (*this)(1,4) = ( 0.06 *  c_q_RF_HFE_);
    (*this)(1,5) = ( 0.0701 *  c_q_RF_HFE_);
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
    (*this)(0,5) = - 0.06;
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
    (*this)(0,3) = (- 0.0701 *  c_q_RF_HFE_);
    (*this)(0,4) = ( 0.0701 *  s_q_RF_HFE_);
    (*this)(1,3) = ( 0.06 *  s_q_RF_HFE_);
    (*this)(1,4) = ( 0.06 *  c_q_RF_HFE_);
    (*this)(2,0) =  c_q_RF_HFE_;
    (*this)(2,1) = - s_q_RF_HFE_;
    (*this)(2,3) = ( 0.0701 *  s_q_RF_HFE_);
    (*this)(2,4) = ( 0.0701 *  c_q_RF_HFE_);
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
    (*this)(2,3) = 0.28263;
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
    (*this)(0,3) = ( 0.08689 *  s_q_RF_KFE_);
    (*this)(0,4) = (- 0.08689 *  c_q_RF_KFE_);
    (*this)(0,5) = (- 0.28263 *  c_q_RF_KFE_);
    (*this)(1,0) = - s_q_RF_KFE_;
    (*this)(1,1) =  c_q_RF_KFE_;
    (*this)(1,3) = ( 0.08689 *  c_q_RF_KFE_);
    (*this)(1,4) = ( 0.08689 *  s_q_RF_KFE_);
    (*this)(1,5) = ( 0.28263 *  s_q_RF_KFE_);
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
    (*this)(0,5) = 0.28263;
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
    (*this)(0,3) = ( 0.08689 *  s_q_RF_KFE_);
    (*this)(0,4) = ( 0.08689 *  c_q_RF_KFE_);
    (*this)(1,0) =  s_q_RF_KFE_;
    (*this)(1,1) =  c_q_RF_KFE_;
    (*this)(1,3) = (- 0.08689 *  c_q_RF_KFE_);
    (*this)(1,4) = ( 0.08689 *  s_q_RF_KFE_);
    (*this)(2,3) = (- 0.28263 *  c_q_RF_KFE_);
    (*this)(2,4) = ( 0.28263 *  s_q_RF_KFE_);
    (*this)(3,3) =  c_q_RF_KFE_;
    (*this)(3,4) = - s_q_RF_KFE_;
    (*this)(4,3) =  s_q_RF_KFE_;
    (*this)(4,4) =  c_q_RF_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_RF_SHANK::Type_fr_RF_WHEEL_L_X_fr_RF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.248;
    (*this)(2,4) = - 0.08748;
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_RF_SHANK& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_RF_SHANK::update(const JState& q) {
    Scalar s_q_RF_WHEEL_;
    Scalar c_q_RF_WHEEL_;
    
    s_q_RF_WHEEL_ = TRAIT::sin( q(RF_WHEEL));
    c_q_RF_WHEEL_ = TRAIT::cos( q(RF_WHEEL));
    
    (*this)(0,0) =  c_q_RF_WHEEL_;
    (*this)(0,1) =  s_q_RF_WHEEL_;
    (*this)(0,3) = ( 0.0328 *  s_q_RF_WHEEL_);
    (*this)(0,4) = (- 0.0328 *  c_q_RF_WHEEL_);
    (*this)(0,5) = (( 0.08748 *  s_q_RF_WHEEL_) - ( 0.248 *  c_q_RF_WHEEL_));
    (*this)(1,0) = - s_q_RF_WHEEL_;
    (*this)(1,1) =  c_q_RF_WHEEL_;
    (*this)(1,3) = ( 0.0328 *  c_q_RF_WHEEL_);
    (*this)(1,4) = ( 0.0328 *  s_q_RF_WHEEL_);
    (*this)(1,5) = (( 0.248 *  s_q_RF_WHEEL_) + ( 0.08748 *  c_q_RF_WHEEL_));
    (*this)(3,3) =  c_q_RF_WHEEL_;
    (*this)(3,4) =  s_q_RF_WHEEL_;
    (*this)(4,3) = - s_q_RF_WHEEL_;
    (*this)(4,4) =  c_q_RF_WHEEL_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_WHEEL_L::Type_fr_RF_SHANK_X_fr_RF_WHEEL_L()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0.248;
    (*this)(1,2) = 0;
    (*this)(1,5) = - 0.08748;
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_WHEEL_L& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_WHEEL_L::update(const JState& q) {
    Scalar s_q_RF_WHEEL_;
    Scalar c_q_RF_WHEEL_;
    
    s_q_RF_WHEEL_ = TRAIT::sin( q(RF_WHEEL));
    c_q_RF_WHEEL_ = TRAIT::cos( q(RF_WHEEL));
    
    (*this)(0,0) =  c_q_RF_WHEEL_;
    (*this)(0,1) = - s_q_RF_WHEEL_;
    (*this)(0,3) = ( 0.0328 *  s_q_RF_WHEEL_);
    (*this)(0,4) = ( 0.0328 *  c_q_RF_WHEEL_);
    (*this)(1,0) =  s_q_RF_WHEEL_;
    (*this)(1,1) =  c_q_RF_WHEEL_;
    (*this)(1,3) = (- 0.0328 *  c_q_RF_WHEEL_);
    (*this)(1,4) = ( 0.0328 *  s_q_RF_WHEEL_);
    (*this)(2,3) = (( 0.08748 *  s_q_RF_WHEEL_) - ( 0.248 *  c_q_RF_WHEEL_));
    (*this)(2,4) = (( 0.248 *  s_q_RF_WHEEL_) + ( 0.08748 *  c_q_RF_WHEEL_));
    (*this)(3,3) =  c_q_RF_WHEEL_;
    (*this)(3,4) = - s_q_RF_WHEEL_;
    (*this)(4,3) =  s_q_RF_WHEEL_;
    (*this)(4,4) =  c_q_RF_WHEEL_;
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,1) =  s_q_LH_HAA_;
    (*this)(0,2) = - c_q_LH_HAA_;
    (*this)(0,3) = (- 0.104 *  c_q_LH_HAA_);
    (*this)(0,4) = (- 0.3 *  c_q_LH_HAA_);
    (*this)(0,5) = (- 0.3 *  s_q_LH_HAA_);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = ( 0.104 *  s_q_LH_HAA_);
    (*this)(1,4) = ( 0.3 *  s_q_LH_HAA_);
    (*this)(1,5) = (- 0.3 *  c_q_LH_HAA_);
    (*this)(3,4) =  s_q_LH_HAA_;
    (*this)(3,5) = - c_q_LH_HAA_;
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(4,5) =  s_q_LH_HAA_;
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP::update(const JState& q) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,3) = (- 0.104 *  c_q_LH_HAA_);
    (*this)(0,4) = ( 0.104 *  s_q_LH_HAA_);
    (*this)(1,0) =  s_q_LH_HAA_;
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,3) = (- 0.3 *  c_q_LH_HAA_);
    (*this)(1,4) = ( 0.3 *  s_q_LH_HAA_);
    (*this)(2,0) = - c_q_LH_HAA_;
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,3) = (- 0.3 *  s_q_LH_HAA_);
    (*this)(2,4) = (- 0.3 *  c_q_LH_HAA_);
    (*this)(4,3) =  s_q_LH_HAA_;
    (*this)(4,4) =  c_q_LH_HAA_;
    (*this)(5,3) = - c_q_LH_HAA_;
    (*this)(5,4) =  s_q_LH_HAA_;
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
    (*this)(2,3) = 0.06;
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
    (*this)(0,3) = ( 0.0701 *  c_q_LH_HFE_);
    (*this)(0,4) = (- 0.06 *  s_q_LH_HFE_);
    (*this)(0,5) = (- 0.0701 *  s_q_LH_HFE_);
    (*this)(1,0) =  c_q_LH_HFE_;
    (*this)(1,2) = - s_q_LH_HFE_;
    (*this)(1,3) = (- 0.0701 *  s_q_LH_HFE_);
    (*this)(1,4) = (- 0.06 *  c_q_LH_HFE_);
    (*this)(1,5) = (- 0.0701 *  c_q_LH_HFE_);
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
    (*this)(0,5) = 0.06;
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
    (*this)(0,3) = ( 0.0701 *  c_q_LH_HFE_);
    (*this)(0,4) = (- 0.0701 *  s_q_LH_HFE_);
    (*this)(1,3) = (- 0.06 *  s_q_LH_HFE_);
    (*this)(1,4) = (- 0.06 *  c_q_LH_HFE_);
    (*this)(2,0) =  c_q_LH_HFE_;
    (*this)(2,1) = - s_q_LH_HFE_;
    (*this)(2,3) = (- 0.0701 *  s_q_LH_HFE_);
    (*this)(2,4) = (- 0.0701 *  c_q_LH_HFE_);
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
    (*this)(2,3) = 0.28263;
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
    (*this)(0,3) = (- 0.08689 *  s_q_LH_KFE_);
    (*this)(0,4) = ( 0.08689 *  c_q_LH_KFE_);
    (*this)(0,5) = (- 0.28263 *  c_q_LH_KFE_);
    (*this)(1,0) = - s_q_LH_KFE_;
    (*this)(1,1) =  c_q_LH_KFE_;
    (*this)(1,3) = (- 0.08689 *  c_q_LH_KFE_);
    (*this)(1,4) = (- 0.08689 *  s_q_LH_KFE_);
    (*this)(1,5) = ( 0.28263 *  s_q_LH_KFE_);
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
    (*this)(0,5) = 0.28263;
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
    (*this)(0,3) = (- 0.08689 *  s_q_LH_KFE_);
    (*this)(0,4) = (- 0.08689 *  c_q_LH_KFE_);
    (*this)(1,0) =  s_q_LH_KFE_;
    (*this)(1,1) =  c_q_LH_KFE_;
    (*this)(1,3) = ( 0.08689 *  c_q_LH_KFE_);
    (*this)(1,4) = (- 0.08689 *  s_q_LH_KFE_);
    (*this)(2,3) = (- 0.28263 *  c_q_LH_KFE_);
    (*this)(2,4) = ( 0.28263 *  s_q_LH_KFE_);
    (*this)(3,3) =  c_q_LH_KFE_;
    (*this)(3,4) = - s_q_LH_KFE_;
    (*this)(4,3) =  s_q_LH_KFE_;
    (*this)(4,4) =  c_q_LH_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_LH_SHANK::Type_fr_LH_WHEEL_L_X_fr_LH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.248;
    (*this)(2,4) = 0.08748;
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_LH_SHANK& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_LH_SHANK::update(const JState& q) {
    Scalar s_q_LH_WHEEL_;
    Scalar c_q_LH_WHEEL_;
    
    s_q_LH_WHEEL_ = TRAIT::sin( q(LH_WHEEL));
    c_q_LH_WHEEL_ = TRAIT::cos( q(LH_WHEEL));
    
    (*this)(0,0) =  c_q_LH_WHEEL_;
    (*this)(0,1) =  s_q_LH_WHEEL_;
    (*this)(0,3) = (- 0.0328 *  s_q_LH_WHEEL_);
    (*this)(0,4) = ( 0.0328 *  c_q_LH_WHEEL_);
    (*this)(0,5) = ((- 0.08748 *  s_q_LH_WHEEL_) - ( 0.248 *  c_q_LH_WHEEL_));
    (*this)(1,0) = - s_q_LH_WHEEL_;
    (*this)(1,1) =  c_q_LH_WHEEL_;
    (*this)(1,3) = (- 0.0328 *  c_q_LH_WHEEL_);
    (*this)(1,4) = (- 0.0328 *  s_q_LH_WHEEL_);
    (*this)(1,5) = (( 0.248 *  s_q_LH_WHEEL_) - ( 0.08748 *  c_q_LH_WHEEL_));
    (*this)(3,3) =  c_q_LH_WHEEL_;
    (*this)(3,4) =  s_q_LH_WHEEL_;
    (*this)(4,3) = - s_q_LH_WHEEL_;
    (*this)(4,4) =  c_q_LH_WHEEL_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_WHEEL_L::Type_fr_LH_SHANK_X_fr_LH_WHEEL_L()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0.248;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0.08748;
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_WHEEL_L& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_WHEEL_L::update(const JState& q) {
    Scalar s_q_LH_WHEEL_;
    Scalar c_q_LH_WHEEL_;
    
    s_q_LH_WHEEL_ = TRAIT::sin( q(LH_WHEEL));
    c_q_LH_WHEEL_ = TRAIT::cos( q(LH_WHEEL));
    
    (*this)(0,0) =  c_q_LH_WHEEL_;
    (*this)(0,1) = - s_q_LH_WHEEL_;
    (*this)(0,3) = (- 0.0328 *  s_q_LH_WHEEL_);
    (*this)(0,4) = (- 0.0328 *  c_q_LH_WHEEL_);
    (*this)(1,0) =  s_q_LH_WHEEL_;
    (*this)(1,1) =  c_q_LH_WHEEL_;
    (*this)(1,3) = ( 0.0328 *  c_q_LH_WHEEL_);
    (*this)(1,4) = (- 0.0328 *  s_q_LH_WHEEL_);
    (*this)(2,3) = ((- 0.08748 *  s_q_LH_WHEEL_) - ( 0.248 *  c_q_LH_WHEEL_));
    (*this)(2,4) = (( 0.248 *  s_q_LH_WHEEL_) - ( 0.08748 *  c_q_LH_WHEEL_));
    (*this)(3,3) =  c_q_LH_WHEEL_;
    (*this)(3,4) = - s_q_LH_WHEEL_;
    (*this)(4,3) =  s_q_LH_WHEEL_;
    (*this)(4,4) =  c_q_LH_WHEEL_;
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_HIP_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,1) =  s_q_RH_HAA_;
    (*this)(0,2) = - c_q_RH_HAA_;
    (*this)(0,3) = ( 0.104 *  c_q_RH_HAA_);
    (*this)(0,4) = (- 0.3 *  c_q_RH_HAA_);
    (*this)(0,5) = (- 0.3 *  s_q_RH_HAA_);
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = (- 0.104 *  s_q_RH_HAA_);
    (*this)(1,4) = ( 0.3 *  s_q_RH_HAA_);
    (*this)(1,5) = (- 0.3 *  c_q_RH_HAA_);
    (*this)(3,4) =  s_q_RH_HAA_;
    (*this)(3,5) = - c_q_RH_HAA_;
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(4,5) =  s_q_RH_HAA_;
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP::update(const JState& q) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,3) = ( 0.104 *  c_q_RH_HAA_);
    (*this)(0,4) = (- 0.104 *  s_q_RH_HAA_);
    (*this)(1,0) =  s_q_RH_HAA_;
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,3) = (- 0.3 *  c_q_RH_HAA_);
    (*this)(1,4) = ( 0.3 *  s_q_RH_HAA_);
    (*this)(2,0) = - c_q_RH_HAA_;
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,3) = (- 0.3 *  s_q_RH_HAA_);
    (*this)(2,4) = (- 0.3 *  c_q_RH_HAA_);
    (*this)(4,3) =  s_q_RH_HAA_;
    (*this)(4,4) =  c_q_RH_HAA_;
    (*this)(5,3) = - c_q_RH_HAA_;
    (*this)(5,4) =  s_q_RH_HAA_;
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
    (*this)(2,3) = 0.06;
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
    (*this)(0,3) = (- 0.0701 *  c_q_RH_HFE_);
    (*this)(0,4) = (- 0.06 *  s_q_RH_HFE_);
    (*this)(0,5) = ( 0.0701 *  s_q_RH_HFE_);
    (*this)(1,0) =  c_q_RH_HFE_;
    (*this)(1,2) = - s_q_RH_HFE_;
    (*this)(1,3) = ( 0.0701 *  s_q_RH_HFE_);
    (*this)(1,4) = (- 0.06 *  c_q_RH_HFE_);
    (*this)(1,5) = ( 0.0701 *  c_q_RH_HFE_);
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
    (*this)(0,5) = 0.06;
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
    (*this)(0,3) = (- 0.0701 *  c_q_RH_HFE_);
    (*this)(0,4) = ( 0.0701 *  s_q_RH_HFE_);
    (*this)(1,3) = (- 0.06 *  s_q_RH_HFE_);
    (*this)(1,4) = (- 0.06 *  c_q_RH_HFE_);
    (*this)(2,0) =  c_q_RH_HFE_;
    (*this)(2,1) = - s_q_RH_HFE_;
    (*this)(2,3) = ( 0.0701 *  s_q_RH_HFE_);
    (*this)(2,4) = ( 0.0701 *  c_q_RH_HFE_);
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
    (*this)(2,3) = 0.28263;
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
    (*this)(0,3) = ( 0.08689 *  s_q_RH_KFE_);
    (*this)(0,4) = (- 0.08689 *  c_q_RH_KFE_);
    (*this)(0,5) = (- 0.28263 *  c_q_RH_KFE_);
    (*this)(1,0) = - s_q_RH_KFE_;
    (*this)(1,1) =  c_q_RH_KFE_;
    (*this)(1,3) = ( 0.08689 *  c_q_RH_KFE_);
    (*this)(1,4) = ( 0.08689 *  s_q_RH_KFE_);
    (*this)(1,5) = ( 0.28263 *  s_q_RH_KFE_);
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
    (*this)(0,5) = 0.28263;
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
    (*this)(0,3) = ( 0.08689 *  s_q_RH_KFE_);
    (*this)(0,4) = ( 0.08689 *  c_q_RH_KFE_);
    (*this)(1,0) =  s_q_RH_KFE_;
    (*this)(1,1) =  c_q_RH_KFE_;
    (*this)(1,3) = (- 0.08689 *  c_q_RH_KFE_);
    (*this)(1,4) = ( 0.08689 *  s_q_RH_KFE_);
    (*this)(2,3) = (- 0.28263 *  c_q_RH_KFE_);
    (*this)(2,4) = ( 0.28263 *  s_q_RH_KFE_);
    (*this)(3,3) =  c_q_RH_KFE_;
    (*this)(3,4) = - s_q_RH_KFE_;
    (*this)(4,3) =  s_q_RH_KFE_;
    (*this)(4,4) =  c_q_RH_KFE_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_RH_SHANK::Type_fr_RH_WHEEL_L_X_fr_RH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.248;
    (*this)(2,4) = 0.08748;
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_RH_SHANK& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_RH_SHANK::update(const JState& q) {
    Scalar s_q_RH_WHEEL_;
    Scalar c_q_RH_WHEEL_;
    
    s_q_RH_WHEEL_ = TRAIT::sin( q(RH_WHEEL));
    c_q_RH_WHEEL_ = TRAIT::cos( q(RH_WHEEL));
    
    (*this)(0,0) =  c_q_RH_WHEEL_;
    (*this)(0,1) =  s_q_RH_WHEEL_;
    (*this)(0,3) = ( 0.0328 *  s_q_RH_WHEEL_);
    (*this)(0,4) = (- 0.0328 *  c_q_RH_WHEEL_);
    (*this)(0,5) = ((- 0.08748 *  s_q_RH_WHEEL_) - ( 0.248 *  c_q_RH_WHEEL_));
    (*this)(1,0) = - s_q_RH_WHEEL_;
    (*this)(1,1) =  c_q_RH_WHEEL_;
    (*this)(1,3) = ( 0.0328 *  c_q_RH_WHEEL_);
    (*this)(1,4) = ( 0.0328 *  s_q_RH_WHEEL_);
    (*this)(1,5) = (( 0.248 *  s_q_RH_WHEEL_) - ( 0.08748 *  c_q_RH_WHEEL_));
    (*this)(3,3) =  c_q_RH_WHEEL_;
    (*this)(3,4) =  s_q_RH_WHEEL_;
    (*this)(4,3) = - s_q_RH_WHEEL_;
    (*this)(4,4) =  c_q_RH_WHEEL_;
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_WHEEL_L::Type_fr_RH_SHANK_X_fr_RH_WHEEL_L()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0.248;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0.08748;
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
const typename iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_WHEEL_L& iit::ANYmal::tpl::ForceTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_WHEEL_L::update(const JState& q) {
    Scalar s_q_RH_WHEEL_;
    Scalar c_q_RH_WHEEL_;
    
    s_q_RH_WHEEL_ = TRAIT::sin( q(RH_WHEEL));
    c_q_RH_WHEEL_ = TRAIT::cos( q(RH_WHEEL));
    
    (*this)(0,0) =  c_q_RH_WHEEL_;
    (*this)(0,1) = - s_q_RH_WHEEL_;
    (*this)(0,3) = ( 0.0328 *  s_q_RH_WHEEL_);
    (*this)(0,4) = ( 0.0328 *  c_q_RH_WHEEL_);
    (*this)(1,0) =  s_q_RH_WHEEL_;
    (*this)(1,1) =  c_q_RH_WHEEL_;
    (*this)(1,3) = (- 0.0328 *  c_q_RH_WHEEL_);
    (*this)(1,4) = ( 0.0328 *  s_q_RH_WHEEL_);
    (*this)(2,3) = ((- 0.08748 *  s_q_RH_WHEEL_) - ( 0.248 *  c_q_RH_WHEEL_));
    (*this)(2,4) = (( 0.248 *  s_q_RH_WHEEL_) - ( 0.08748 *  c_q_RH_WHEEL_));
    (*this)(3,3) =  c_q_RH_WHEEL_;
    (*this)(3,4) = - s_q_RH_WHEEL_;
    (*this)(4,3) =  s_q_RH_WHEEL_;
    (*this)(4,4) =  c_q_RH_WHEEL_;
    return *this;
}

template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL_L::Type_fr_base_X_fr_LF_WHEEL_L()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL_L& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL_L::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_WHEEL_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_WHEEL_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_WHEEL_ = TRAIT::sin( q(LF_WHEEL));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_WHEEL_ = TRAIT::cos( q(LF_WHEEL));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((( c_q_LF_HFE_ *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(0,1) = (((( s_q_LF_HFE_ *  s_q_LF_KFE_) - ( c_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(0,3) = ((((((- 0.08748 *  s_q_LF_HFE_) - ( 0.248 *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.08748 *  c_q_LF_HFE_) - ( 0.248 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.28263 *  s_q_LF_HFE_)) +  0.36);
    (*this)(1,0) = ((((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,1) = (((((- s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = (((((((( 0.08748 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.248 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08748 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.248 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.28263 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.18979 *  c_q_LF_HAA_)) +  0.104);
    (*this)(2,0) = ((((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(2,1) = ((((( c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = ((((((( 0.248 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08748 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.08748 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.248 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.28263 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.18979 *  s_q_LF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_base::Type_fr_LF_WHEEL_L_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_base::update(const JState& q) {
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_WHEEL_;
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    Scalar c_q_LF_WHEEL_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_KFE_ = TRAIT::sin( q(LF_KFE));
    s_q_LF_HFE_ = TRAIT::sin( q(LF_HFE));
    s_q_LF_WHEEL_ = TRAIT::sin( q(LF_WHEEL));
    s_q_LF_HAA_ = TRAIT::sin( q(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( q(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( q(LF_KFE));
    c_q_LF_WHEEL_ = TRAIT::cos( q(LF_WHEEL));
    c_q_LF_HAA_ = TRAIT::cos( q(LF_HAA));
    
    (*this)(0,0) = ((((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((( 1.0 *  c_q_LF_HFE_) *  c_q_LF_KFE_) - ( s_q_LF_HFE_ *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(0,1) = (((((( 1.0 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((( 1.0 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(0,2) = (((((( 1.0 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(0,3) = (((((((( 0.104 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ( 0.36 *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.36 *  s_q_LF_HFE_) - (( 0.104 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) -  0.28263) *  c_q_LF_KFE_)) -  0.248) *  s_q_LF_WHEEL_) + ((((((( 0.36 *  s_q_LF_HFE_) - (( 0.104 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) -  0.28263) *  s_q_LF_KFE_) + ((((- 0.104 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - ( 0.36 *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) -  0.08748) *  c_q_LF_WHEEL_));
    (*this)(1,0) = ((((( 1.0 *  s_q_LF_HFE_) *  s_q_LF_KFE_) - ( c_q_LF_HFE_ *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + (((- c_q_LF_HFE_ *  s_q_LF_KFE_) - ( s_q_LF_HFE_ *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,1) = (((((- s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((( 1.0 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,2) = (((((( 1.0 *  c_q_LF_HAA_) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_)) *  s_q_LF_WHEEL_) + ((((( 1.0 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_)) *  c_q_LF_WHEEL_));
    (*this)(1,3) = ((((((((- 0.36 *  s_q_LF_HFE_) + (( 0.104 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) +  0.28263) *  s_q_LF_KFE_) + (((( 0.104 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ( 0.36 *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) +  0.08748) *  s_q_LF_WHEEL_) + ((((((( 0.104 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + ( 0.36 *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.36 *  s_q_LF_HFE_) - (( 0.104 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) -  0.28263) *  c_q_LF_KFE_)) -  0.248) *  c_q_LF_WHEEL_));
    (*this)(2,1) =  c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = ((- 0.104 *  c_q_LF_HAA_) -  0.18979);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL_L::Type_fr_base_X_fr_LH_WHEEL_L()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL_L& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL_L::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_WHEEL_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_WHEEL_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_WHEEL_ = TRAIT::sin( q(LH_WHEEL));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_WHEEL_ = TRAIT::cos( q(LH_WHEEL));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((( c_q_LH_HFE_ *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(0,1) = (((( s_q_LH_HFE_ *  s_q_LH_KFE_) - ( c_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(0,3) = (((((( 0.08748 *  s_q_LH_HFE_) - ( 0.248 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((- 0.248 *  s_q_LH_HFE_) - ( 0.08748 *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.28263 *  s_q_LH_HFE_)) -  0.36);
    (*this)(1,0) = ((((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,1) = (((((- s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = ((((((((- 0.248 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08748 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.248 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08748 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.28263 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.18979 *  c_q_LH_HAA_)) +  0.104);
    (*this)(2,0) = ((((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(2,1) = ((((( c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ((((((( 0.248 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08748 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.08748 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.248 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.28263 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.18979 *  s_q_LH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_base::Type_fr_LH_WHEEL_L_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_base::update(const JState& q) {
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_WHEEL_;
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    Scalar c_q_LH_WHEEL_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_KFE_ = TRAIT::sin( q(LH_KFE));
    s_q_LH_HFE_ = TRAIT::sin( q(LH_HFE));
    s_q_LH_WHEEL_ = TRAIT::sin( q(LH_WHEEL));
    s_q_LH_HAA_ = TRAIT::sin( q(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( q(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( q(LH_KFE));
    c_q_LH_WHEEL_ = TRAIT::cos( q(LH_WHEEL));
    c_q_LH_HAA_ = TRAIT::cos( q(LH_HAA));
    
    (*this)(0,0) = ((((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((( 1.0 *  c_q_LH_HFE_) *  c_q_LH_KFE_) - ( s_q_LH_HFE_ *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(0,1) = (((((( 1.0 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((( 1.0 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(0,2) = (((((( 1.0 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(0,3) = (((((((( 0.104 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - ( 0.36 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.36 *  s_q_LH_HFE_) - (( 0.104 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) -  0.28263) *  c_q_LH_KFE_)) -  0.248) *  s_q_LH_WHEEL_) + (((((((- 0.36 *  s_q_LH_HFE_) - (( 0.104 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) -  0.28263) *  s_q_LH_KFE_) + ((( 0.36 *  c_q_LH_HFE_) - (( 0.104 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) +  0.08748) *  c_q_LH_WHEEL_));
    (*this)(1,0) = ((((( 1.0 *  s_q_LH_HFE_) *  s_q_LH_KFE_) - ( c_q_LH_HFE_ *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + (((- c_q_LH_HFE_ *  s_q_LH_KFE_) - ( s_q_LH_HFE_ *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,1) = (((((- s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((( 1.0 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,2) = (((((( 1.0 *  c_q_LH_HAA_) *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_)) *  s_q_LH_WHEEL_) + ((((( 1.0 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_)) *  c_q_LH_WHEEL_));
    (*this)(1,3) = (((((((( 0.36 *  s_q_LH_HFE_) + (( 0.104 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) +  0.28263) *  s_q_LH_KFE_) + (((( 0.104 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - ( 0.36 *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) -  0.08748) *  s_q_LH_WHEEL_) + ((((((( 0.104 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - ( 0.36 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.36 *  s_q_LH_HFE_) - (( 0.104 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) -  0.28263) *  c_q_LH_KFE_)) -  0.248) *  c_q_LH_WHEEL_));
    (*this)(2,1) =  c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ((- 0.104 *  c_q_LH_HAA_) -  0.18979);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL_L::Type_fr_base_X_fr_RF_WHEEL_L()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL_L& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL_L::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_WHEEL_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_WHEEL_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_WHEEL_ = TRAIT::sin( q(RF_WHEEL));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_WHEEL_ = TRAIT::cos( q(RF_WHEEL));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((( c_q_RF_HFE_ *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(0,1) = (((( s_q_RF_HFE_ *  s_q_RF_KFE_) - ( c_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(0,3) = ((((((- 0.08748 *  s_q_RF_HFE_) - ( 0.248 *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.08748 *  c_q_RF_HFE_) - ( 0.248 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.28263 *  s_q_RF_HFE_)) +  0.36);
    (*this)(1,0) = ((((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,1) = (((((- s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = (((((((( 0.08748 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.248 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08748 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.248 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.28263 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.18979 *  c_q_RF_HAA_)) -  0.104);
    (*this)(2,0) = ((((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(2,1) = ((((( c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = ((((((( 0.248 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08748 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.08748 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.248 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.28263 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.18979 *  s_q_RF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_base::Type_fr_RF_WHEEL_L_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_base::update(const JState& q) {
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_WHEEL_;
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    Scalar c_q_RF_WHEEL_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_KFE_ = TRAIT::sin( q(RF_KFE));
    s_q_RF_HFE_ = TRAIT::sin( q(RF_HFE));
    s_q_RF_WHEEL_ = TRAIT::sin( q(RF_WHEEL));
    s_q_RF_HAA_ = TRAIT::sin( q(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( q(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( q(RF_KFE));
    c_q_RF_WHEEL_ = TRAIT::cos( q(RF_WHEEL));
    c_q_RF_HAA_ = TRAIT::cos( q(RF_HAA));
    
    (*this)(0,0) = ((((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((( 1.0 *  c_q_RF_HFE_) *  c_q_RF_KFE_) - ( s_q_RF_HFE_ *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(0,1) = (((((( 1.0 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((( 1.0 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(0,2) = (((((( 1.0 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(0,3) = ((((((( 0.36 *  c_q_RF_HFE_) - (( 0.104 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.36 *  s_q_RF_HFE_) + (( 0.104 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) -  0.28263) *  c_q_RF_KFE_)) -  0.248) *  s_q_RF_WHEEL_) + ((((((( 0.36 *  s_q_RF_HFE_) + (( 0.104 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) -  0.28263) *  s_q_RF_KFE_) + (((( 0.104 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - ( 0.36 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) -  0.08748) *  c_q_RF_WHEEL_));
    (*this)(1,0) = ((((( 1.0 *  s_q_RF_HFE_) *  s_q_RF_KFE_) - ( c_q_RF_HFE_ *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + (((- c_q_RF_HFE_ *  s_q_RF_KFE_) - ( s_q_RF_HFE_ *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,1) = (((((- s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((( 1.0 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,2) = (((((( 1.0 *  c_q_RF_HAA_) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_)) *  s_q_RF_WHEEL_) + ((((( 1.0 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_)) *  c_q_RF_WHEEL_));
    (*this)(1,3) = ((((((((- 0.36 *  s_q_RF_HFE_) - (( 0.104 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) +  0.28263) *  s_q_RF_KFE_) + ((( 0.36 *  c_q_RF_HFE_) - (( 0.104 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) +  0.08748) *  s_q_RF_WHEEL_) + (((((( 0.36 *  c_q_RF_HFE_) - (( 0.104 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.36 *  s_q_RF_HFE_) + (( 0.104 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) -  0.28263) *  c_q_RF_KFE_)) -  0.248) *  c_q_RF_WHEEL_));
    (*this)(2,1) =  c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (( 0.104 *  c_q_RF_HAA_) +  0.18979);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL_L::Type_fr_base_X_fr_RH_WHEEL_L()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL_L& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL_L::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_WHEEL_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_WHEEL_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_WHEEL_ = TRAIT::sin( q(RH_WHEEL));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_WHEEL_ = TRAIT::cos( q(RH_WHEEL));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((( c_q_RH_HFE_ *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(0,1) = (((( s_q_RH_HFE_ *  s_q_RH_KFE_) - ( c_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(0,3) = (((((( 0.08748 *  s_q_RH_HFE_) - ( 0.248 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((- 0.248 *  s_q_RH_HFE_) - ( 0.08748 *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.28263 *  s_q_RH_HFE_)) -  0.36);
    (*this)(1,0) = ((((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,1) = (((((- s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = ((((((((- 0.248 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08748 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.248 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08748 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.28263 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.18979 *  c_q_RH_HAA_)) -  0.104);
    (*this)(2,0) = ((((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(2,1) = ((((( c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = ((((((( 0.248 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08748 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.08748 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.248 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.28263 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.18979 *  s_q_RH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_base::Type_fr_RH_WHEEL_L_X_fr_base()
{
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_base& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_base::update(const JState& q) {
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_WHEEL_;
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    Scalar c_q_RH_WHEEL_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_KFE_ = TRAIT::sin( q(RH_KFE));
    s_q_RH_HFE_ = TRAIT::sin( q(RH_HFE));
    s_q_RH_WHEEL_ = TRAIT::sin( q(RH_WHEEL));
    s_q_RH_HAA_ = TRAIT::sin( q(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( q(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( q(RH_KFE));
    c_q_RH_WHEEL_ = TRAIT::cos( q(RH_WHEEL));
    c_q_RH_HAA_ = TRAIT::cos( q(RH_HAA));
    
    (*this)(0,0) = ((((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((( 1.0 *  c_q_RH_HFE_) *  c_q_RH_KFE_) - ( s_q_RH_HFE_ *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(0,1) = (((((( 1.0 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((( 1.0 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(0,2) = (((((( 1.0 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(0,3) = ((((((((- 0.104 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - ( 0.36 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.36 *  s_q_RH_HFE_) + (( 0.104 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) -  0.28263) *  c_q_RH_KFE_)) -  0.248) *  s_q_RH_WHEEL_) + (((((((- 0.36 *  s_q_RH_HFE_) + (( 0.104 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) -  0.28263) *  s_q_RH_KFE_) + (((( 0.104 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + ( 0.36 *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) +  0.08748) *  c_q_RH_WHEEL_));
    (*this)(1,0) = ((((( 1.0 *  s_q_RH_HFE_) *  s_q_RH_KFE_) - ( c_q_RH_HFE_ *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + (((- c_q_RH_HFE_ *  s_q_RH_KFE_) - ( s_q_RH_HFE_ *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,1) = (((((- s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((( 1.0 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,2) = (((((( 1.0 *  c_q_RH_HAA_) *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_)) *  s_q_RH_WHEEL_) + ((((( 1.0 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_)) *  c_q_RH_WHEEL_));
    (*this)(1,3) = (((((((( 0.36 *  s_q_RH_HFE_) - (( 0.104 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) +  0.28263) *  s_q_RH_KFE_) + ((((- 0.104 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - ( 0.36 *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) -  0.08748) *  s_q_RH_WHEEL_) + (((((((- 0.104 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - ( 0.36 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.36 *  s_q_RH_HFE_) + (( 0.104 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) -  0.28263) *  c_q_RH_KFE_)) -  0.248) *  c_q_RH_WHEEL_));
    (*this)(2,1) =  c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (( 0.104 *  c_q_RH_HAA_) +  0.18979);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA::Type_fr_base_X_fr_LF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.3;
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
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HFE::Type_fr_base_X_fr_LF_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.36;
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
    (*this)(1,3) = (( 0.0701 *  c_q_LF_HAA_) +  0.104);
    (*this)(2,1) = - c_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = ( 0.0701 *  s_q_LF_HAA_);
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
    (*this)(0,3) = ( 0.36 - ( 0.28263 *  s_q_LF_HFE_));
    (*this)(1,0) = ( s_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(1,1) = ( s_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = (((( 0.28263 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ( 0.15699 *  c_q_LF_HAA_)) +  0.104);
    (*this)(2,0) = (- c_q_LF_HAA_ *  s_q_LF_HFE_);
    (*this)(2,1) = (- c_q_LF_HAA_ *  c_q_LF_HFE_);
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = (( 0.15699 *  s_q_LF_HAA_) - (( 0.28263 *  c_q_LF_HAA_) *  c_q_LF_HFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL::Type_fr_base_X_fr_LF_WHEEL()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_WHEEL::update(const JState& q) {
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
    (*this)(0,3) = ((((((- 0.08748 *  s_q_LF_HFE_) - ( 0.248 *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((( 0.08748 *  c_q_LF_HFE_) - ( 0.248 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.28263 *  s_q_LF_HFE_)) +  0.36);
    (*this)(1,0) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) + (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(1,1) = ((( s_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_) - (( s_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) = (((((((( 0.08748 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.248 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08748 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.248 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.28263 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.18979 *  c_q_LF_HAA_)) +  0.104);
    (*this)(2,0) = (((- c_q_LF_HAA_ *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,1) = ((( c_q_LF_HAA_ *  s_q_LF_HFE_) *  s_q_LF_KFE_) - (( c_q_LF_HAA_ *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) = ((((((( 0.248 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08748 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + ((((- 0.08748 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.248 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.28263 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.18979 *  s_q_LF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA::Type_fr_base_X_fr_LH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.3;
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
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HFE::Type_fr_base_X_fr_LH_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.36;
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
    (*this)(1,3) = (( 0.0701 *  c_q_LH_HAA_) +  0.104);
    (*this)(2,1) = - c_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ( 0.0701 *  s_q_LH_HAA_);
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
    (*this)(0,3) = ((- 0.28263 *  s_q_LH_HFE_) -  0.36);
    (*this)(1,0) = ( s_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(1,1) = ( s_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = (((( 0.28263 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + ( 0.15699 *  c_q_LH_HAA_)) +  0.104);
    (*this)(2,0) = (- c_q_LH_HAA_ *  s_q_LH_HFE_);
    (*this)(2,1) = (- c_q_LH_HAA_ *  c_q_LH_HFE_);
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = (( 0.15699 *  s_q_LH_HAA_) - (( 0.28263 *  c_q_LH_HAA_) *  c_q_LH_HFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL::Type_fr_base_X_fr_LH_WHEEL()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_WHEEL::update(const JState& q) {
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
    (*this)(0,3) = (((((( 0.08748 *  s_q_LH_HFE_) - ( 0.248 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((- 0.248 *  s_q_LH_HFE_) - ( 0.08748 *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.28263 *  s_q_LH_HFE_)) -  0.36);
    (*this)(1,0) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(1,1) = ((( s_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_) - (( s_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) = ((((((((- 0.248 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08748 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.248 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08748 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.28263 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.18979 *  c_q_LH_HAA_)) +  0.104);
    (*this)(2,0) = (((- c_q_LH_HAA_ *  c_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,1) = ((( c_q_LH_HAA_ *  s_q_LH_HFE_) *  s_q_LH_KFE_) - (( c_q_LH_HAA_ *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) = ((((((( 0.248 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08748 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.08748 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.248 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.28263 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.18979 *  s_q_LH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA::Type_fr_base_X_fr_RF_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.3;
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
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HFE::Type_fr_base_X_fr_RF_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.36;
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
    (*this)(1,3) = ((- 0.0701 *  c_q_RF_HAA_) -  0.104);
    (*this)(2,1) = - c_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (- 0.0701 *  s_q_RF_HAA_);
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
    (*this)(0,3) = ( 0.36 - ( 0.28263 *  s_q_RF_HFE_));
    (*this)(1,0) = ( s_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(1,1) = ( s_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = (((( 0.28263 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - ( 0.15699 *  c_q_RF_HAA_)) -  0.104);
    (*this)(2,0) = (- c_q_RF_HAA_ *  s_q_RF_HFE_);
    (*this)(2,1) = (- c_q_RF_HAA_ *  c_q_RF_HFE_);
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = (((- 0.28263 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - ( 0.15699 *  s_q_RF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL::Type_fr_base_X_fr_RF_WHEEL()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_WHEEL::update(const JState& q) {
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
    (*this)(0,3) = ((((((- 0.08748 *  s_q_RF_HFE_) - ( 0.248 *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((( 0.08748 *  c_q_RF_HFE_) - ( 0.248 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.28263 *  s_q_RF_HFE_)) +  0.36);
    (*this)(1,0) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) + (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(1,1) = ((( s_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_) - (( s_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) = (((((((( 0.08748 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.248 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08748 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.248 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.28263 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.18979 *  c_q_RF_HAA_)) -  0.104);
    (*this)(2,0) = (((- c_q_RF_HAA_ *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,1) = ((( c_q_RF_HAA_ *  s_q_RF_HFE_) *  s_q_RF_KFE_) - (( c_q_RF_HAA_ *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) = ((((((( 0.248 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08748 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + ((((- 0.08748 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.248 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.28263 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.18979 *  s_q_RF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA::Type_fr_base_X_fr_RH_HAA()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.3;
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
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HAA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HFE::Type_fr_base_X_fr_RH_HFE()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.36;
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
    (*this)(1,3) = ((- 0.0701 *  c_q_RH_HAA_) -  0.104);
    (*this)(2,1) = - c_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (- 0.0701 *  s_q_RH_HAA_);
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
    (*this)(0,3) = ((- 0.28263 *  s_q_RH_HFE_) -  0.36);
    (*this)(1,0) = ( s_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(1,1) = ( s_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = (((( 0.28263 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - ( 0.15699 *  c_q_RH_HAA_)) -  0.104);
    (*this)(2,0) = (- c_q_RH_HAA_ *  s_q_RH_HFE_);
    (*this)(2,1) = (- c_q_RH_HAA_ *  c_q_RH_HFE_);
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = (((- 0.28263 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - ( 0.15699 *  s_q_RH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL::Type_fr_base_X_fr_RH_WHEEL()
{
    (*this)(0,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_WHEEL::update(const JState& q) {
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
    (*this)(0,3) = (((((( 0.08748 *  s_q_RH_HFE_) - ( 0.248 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((- 0.248 *  s_q_RH_HFE_) - ( 0.08748 *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.28263 *  s_q_RH_HFE_)) -  0.36);
    (*this)(1,0) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(1,1) = ((( s_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_) - (( s_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) = ((((((((- 0.248 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08748 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.248 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08748 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.28263 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.18979 *  c_q_RH_HAA_)) -  0.104);
    (*this)(2,0) = (((- c_q_RH_HAA_ *  c_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,1) = ((( c_q_RH_HAA_ *  s_q_RH_HFE_) *  s_q_RH_KFE_) - (( c_q_RH_HAA_ *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) = ((((((( 0.248 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08748 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.08748 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.248 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.28263 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.18979 *  s_q_RH_HAA_));
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
    (*this)(2,3) = - 0.3;
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
    (*this)(0,3) = (- 0.104 *  s_q_LF_HAA_);
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  s_q_LF_HAA_;
    (*this)(1,3) = (- 0.104 *  c_q_LF_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LF_HIP::Type_fr_base_X_fr_LF_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.3;
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
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_HIP::Type_fr_LF_THIGH_X_fr_LF_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.0701;
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
    (*this)(0,3) = (- 0.06 *  c_q_LF_HFE_);
    (*this)(1,0) =  c_q_LF_HFE_;
    (*this)(1,2) = - s_q_LF_HFE_;
    (*this)(1,3) = ( 0.06 *  s_q_LF_HFE_);
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
    (*this)(1,3) = 0.0701;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.06;
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
    (*this)(2,3) = - 0.08689;
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
    (*this)(0,3) = (- 0.28263 *  s_q_LF_KFE_);
    (*this)(1,0) = - s_q_LF_KFE_;
    (*this)(1,1) =  c_q_LF_KFE_;
    (*this)(1,3) = (- 0.28263 *  c_q_LF_KFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_THIGH_X_fr_LF_SHANK::Type_fr_LF_THIGH_X_fr_LF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.28263;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.08689;
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
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_LF_SHANK::Type_fr_LF_WHEEL_L_X_fr_LF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.0328;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_LF_SHANK& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_WHEEL_L_X_fr_LF_SHANK::update(const JState& q) {
    Scalar s_q_LF_WHEEL_;
    Scalar c_q_LF_WHEEL_;
    
    s_q_LF_WHEEL_ = TRAIT::sin( q(LF_WHEEL));
    c_q_LF_WHEEL_ = TRAIT::cos( q(LF_WHEEL));
    
    (*this)(0,0) =  c_q_LF_WHEEL_;
    (*this)(0,1) =  s_q_LF_WHEEL_;
    (*this)(0,3) = ((- 0.248 *  s_q_LF_WHEEL_) - ( 0.08748 *  c_q_LF_WHEEL_));
    (*this)(1,0) = - s_q_LF_WHEEL_;
    (*this)(1,1) =  c_q_LF_WHEEL_;
    (*this)(1,3) = (( 0.08748 *  s_q_LF_WHEEL_) - ( 0.248 *  c_q_LF_WHEEL_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_WHEEL_L::Type_fr_LF_SHANK_X_fr_LF_WHEEL_L()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.08748;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.248;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0328;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_WHEEL_L& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_SHANK_X_fr_LF_WHEEL_L::update(const JState& q) {
    Scalar s_q_LF_WHEEL_;
    Scalar c_q_LF_WHEEL_;
    
    s_q_LF_WHEEL_ = TRAIT::sin( q(LF_WHEEL));
    c_q_LF_WHEEL_ = TRAIT::cos( q(LF_WHEEL));
    
    (*this)(0,0) =  c_q_LF_WHEEL_;
    (*this)(0,1) = - s_q_LF_WHEEL_;
    (*this)(1,0) =  s_q_LF_WHEEL_;
    (*this)(1,1) =  c_q_LF_WHEEL_;
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
    (*this)(2,3) = - 0.3;
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
    (*this)(0,3) = ( 0.104 *  s_q_RF_HAA_);
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  s_q_RF_HAA_;
    (*this)(1,3) = ( 0.104 *  c_q_RF_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RF_HIP::Type_fr_base_X_fr_RF_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.3;
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
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_HIP::Type_fr_RF_THIGH_X_fr_RF_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.0701;
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
    (*this)(0,3) = (- 0.06 *  c_q_RF_HFE_);
    (*this)(1,0) =  c_q_RF_HFE_;
    (*this)(1,2) = - s_q_RF_HFE_;
    (*this)(1,3) = ( 0.06 *  s_q_RF_HFE_);
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
    (*this)(1,3) = - 0.0701;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.06;
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
    (*this)(2,3) = 0.08689;
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
    (*this)(0,3) = (- 0.28263 *  s_q_RF_KFE_);
    (*this)(1,0) = - s_q_RF_KFE_;
    (*this)(1,1) =  c_q_RF_KFE_;
    (*this)(1,3) = (- 0.28263 *  c_q_RF_KFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_THIGH_X_fr_RF_SHANK::Type_fr_RF_THIGH_X_fr_RF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.28263;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.08689;
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
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_RF_SHANK::Type_fr_RF_WHEEL_L_X_fr_RF_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0328;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_RF_SHANK& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_WHEEL_L_X_fr_RF_SHANK::update(const JState& q) {
    Scalar s_q_RF_WHEEL_;
    Scalar c_q_RF_WHEEL_;
    
    s_q_RF_WHEEL_ = TRAIT::sin( q(RF_WHEEL));
    c_q_RF_WHEEL_ = TRAIT::cos( q(RF_WHEEL));
    
    (*this)(0,0) =  c_q_RF_WHEEL_;
    (*this)(0,1) =  s_q_RF_WHEEL_;
    (*this)(0,3) = ((- 0.248 *  s_q_RF_WHEEL_) - ( 0.08748 *  c_q_RF_WHEEL_));
    (*this)(1,0) = - s_q_RF_WHEEL_;
    (*this)(1,1) =  c_q_RF_WHEEL_;
    (*this)(1,3) = (( 0.08748 *  s_q_RF_WHEEL_) - ( 0.248 *  c_q_RF_WHEEL_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_WHEEL_L::Type_fr_RF_SHANK_X_fr_RF_WHEEL_L()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.08748;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.248;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.0328;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_WHEEL_L& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_SHANK_X_fr_RF_WHEEL_L::update(const JState& q) {
    Scalar s_q_RF_WHEEL_;
    Scalar c_q_RF_WHEEL_;
    
    s_q_RF_WHEEL_ = TRAIT::sin( q(RF_WHEEL));
    c_q_RF_WHEEL_ = TRAIT::cos( q(RF_WHEEL));
    
    (*this)(0,0) =  c_q_RF_WHEEL_;
    (*this)(0,1) = - s_q_RF_WHEEL_;
    (*this)(1,0) =  s_q_RF_WHEEL_;
    (*this)(1,1) =  c_q_RF_WHEEL_;
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
    (*this)(2,3) = 0.3;
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
    (*this)(0,3) = (- 0.104 *  s_q_LH_HAA_);
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  s_q_LH_HAA_;
    (*this)(1,3) = (- 0.104 *  c_q_LH_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_LH_HIP::Type_fr_base_X_fr_LH_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.3;
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
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_HIP::Type_fr_LH_THIGH_X_fr_LH_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.0701;
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
    (*this)(0,3) = ( 0.06 *  c_q_LH_HFE_);
    (*this)(1,0) =  c_q_LH_HFE_;
    (*this)(1,2) = - s_q_LH_HFE_;
    (*this)(1,3) = (- 0.06 *  s_q_LH_HFE_);
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
    (*this)(1,3) = 0.0701;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.06;
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
    (*this)(2,3) = - 0.08689;
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
    (*this)(0,3) = (- 0.28263 *  s_q_LH_KFE_);
    (*this)(1,0) = - s_q_LH_KFE_;
    (*this)(1,1) =  c_q_LH_KFE_;
    (*this)(1,3) = (- 0.28263 *  c_q_LH_KFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_THIGH_X_fr_LH_SHANK::Type_fr_LH_THIGH_X_fr_LH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.28263;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.08689;
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
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_LH_SHANK::Type_fr_LH_WHEEL_L_X_fr_LH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.0328;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_LH_SHANK& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_WHEEL_L_X_fr_LH_SHANK::update(const JState& q) {
    Scalar s_q_LH_WHEEL_;
    Scalar c_q_LH_WHEEL_;
    
    s_q_LH_WHEEL_ = TRAIT::sin( q(LH_WHEEL));
    c_q_LH_WHEEL_ = TRAIT::cos( q(LH_WHEEL));
    
    (*this)(0,0) =  c_q_LH_WHEEL_;
    (*this)(0,1) =  s_q_LH_WHEEL_;
    (*this)(0,3) = (( 0.08748 *  c_q_LH_WHEEL_) - ( 0.248 *  s_q_LH_WHEEL_));
    (*this)(1,0) = - s_q_LH_WHEEL_;
    (*this)(1,1) =  c_q_LH_WHEEL_;
    (*this)(1,3) = ((- 0.08748 *  s_q_LH_WHEEL_) - ( 0.248 *  c_q_LH_WHEEL_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_WHEEL_L::Type_fr_LH_SHANK_X_fr_LH_WHEEL_L()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.08748;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.248;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0328;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_WHEEL_L& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_SHANK_X_fr_LH_WHEEL_L::update(const JState& q) {
    Scalar s_q_LH_WHEEL_;
    Scalar c_q_LH_WHEEL_;
    
    s_q_LH_WHEEL_ = TRAIT::sin( q(LH_WHEEL));
    c_q_LH_WHEEL_ = TRAIT::cos( q(LH_WHEEL));
    
    (*this)(0,0) =  c_q_LH_WHEEL_;
    (*this)(0,1) = - s_q_LH_WHEEL_;
    (*this)(1,0) =  s_q_LH_WHEEL_;
    (*this)(1,1) =  c_q_LH_WHEEL_;
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
    (*this)(2,3) = 0.3;
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
    (*this)(0,3) = ( 0.104 *  s_q_RH_HAA_);
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  s_q_RH_HAA_;
    (*this)(1,3) = ( 0.104 *  c_q_RH_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_RH_HIP::Type_fr_base_X_fr_RH_HIP()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - 0.3;
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
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_HIP::Type_fr_RH_THIGH_X_fr_RH_HIP()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.0701;
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
    (*this)(0,3) = ( 0.06 *  c_q_RH_HFE_);
    (*this)(1,0) =  c_q_RH_HFE_;
    (*this)(1,2) = - s_q_RH_HFE_;
    (*this)(1,3) = (- 0.06 *  s_q_RH_HFE_);
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
    (*this)(1,3) = - 0.0701;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.06;
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
    (*this)(2,3) = 0.08689;
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
    (*this)(0,3) = (- 0.28263 *  s_q_RH_KFE_);
    (*this)(1,0) = - s_q_RH_KFE_;
    (*this)(1,1) =  c_q_RH_KFE_;
    (*this)(1,3) = (- 0.28263 *  c_q_RH_KFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_THIGH_X_fr_RH_SHANK::Type_fr_RH_THIGH_X_fr_RH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.28263;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.08689;
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
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_RH_SHANK::Type_fr_RH_WHEEL_L_X_fr_RH_SHANK()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0328;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_RH_SHANK& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_WHEEL_L_X_fr_RH_SHANK::update(const JState& q) {
    Scalar s_q_RH_WHEEL_;
    Scalar c_q_RH_WHEEL_;
    
    s_q_RH_WHEEL_ = TRAIT::sin( q(RH_WHEEL));
    c_q_RH_WHEEL_ = TRAIT::cos( q(RH_WHEEL));
    
    (*this)(0,0) =  c_q_RH_WHEEL_;
    (*this)(0,1) =  s_q_RH_WHEEL_;
    (*this)(0,3) = (( 0.08748 *  c_q_RH_WHEEL_) - ( 0.248 *  s_q_RH_WHEEL_));
    (*this)(1,0) = - s_q_RH_WHEEL_;
    (*this)(1,1) =  c_q_RH_WHEEL_;
    (*this)(1,3) = ((- 0.08748 *  s_q_RH_WHEEL_) - ( 0.248 *  c_q_RH_WHEEL_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_WHEEL_L::Type_fr_RH_SHANK_X_fr_RH_WHEEL_L()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.08748;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.248;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.0328;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_WHEEL_L& iit::ANYmal::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_SHANK_X_fr_RH_WHEEL_L::update(const JState& q) {
    Scalar s_q_RH_WHEEL_;
    Scalar c_q_RH_WHEEL_;
    
    s_q_RH_WHEEL_ = TRAIT::sin( q(RH_WHEEL));
    c_q_RH_WHEEL_ = TRAIT::cos( q(RH_WHEEL));
    
    (*this)(0,0) =  c_q_RH_WHEEL_;
    (*this)(0,1) = - s_q_RH_WHEEL_;
    (*this)(1,0) =  s_q_RH_WHEEL_;
    (*this)(1,1) =  c_q_RH_WHEEL_;
    return *this;
}

