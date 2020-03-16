
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Jacobians
    ()
     : 
    fr_base_J_fr_LF_WHEEL_L(), 
    fr_base_J_fr_LH_WHEEL_L(), 
    fr_base_J_fr_RF_WHEEL_L(), 
    fr_base_J_fr_RH_WHEEL_L(), 
    fr_base_J_fr_LF_WHEEL_L_COM(), 
    fr_base_J_fr_LH_WHEEL_L_COM(), 
    fr_base_J_fr_RF_WHEEL_L_COM(), 
    fr_base_J_fr_RH_WHEEL_L_COM()
{
    updateParameters();
}

template <typename TRAIT>
void iit::ANYmal::tpl::Jacobians<TRAIT>::updateParameters() {
}


template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_WHEEL_L::Type_fr_base_J_fr_LF_WHEEL_L()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 0;
    (*this)(4,3) = 0;
    (*this)(5,3) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_WHEEL_L& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_WHEEL_L::update(const JState& jState) {
    Scalar s_q_LF_HAA_;
    Scalar s_q_LF_HFE_;
    Scalar s_q_LF_KFE_;
    Scalar c_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    
    s_q_LF_HAA_ = TRAIT::sin( jState(LF_HAA));
    s_q_LF_HFE_ = TRAIT::sin( jState(LF_HFE));
    s_q_LF_KFE_ = TRAIT::sin( jState(LF_KFE));
    c_q_LF_HAA_ = TRAIT::cos( jState(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( jState(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( jState(LF_KFE));
    
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(1,3) =  c_q_LF_HAA_;
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(2,3) =  s_q_LF_HAA_;
    (*this)(3,1) = ((((( 0.2745 *  s_q_LF_HFE_) - ( 0.08748 *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((- 0.08748 *  s_q_LF_HFE_) - ( 0.2745 *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.28263 *  c_q_LF_HFE_));
    (*this)(3,2) = (((( 0.2745 *  s_q_LF_HFE_) - ( 0.08748 *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((- 0.08748 *  s_q_LF_HFE_) - ( 0.2745 *  c_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(4,0) = ((((((( 0.08748 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.2745 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08748 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.2745 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.28263 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.18979 *  s_q_LF_HAA_));
    (*this)(4,1) = ((((((- 0.08748 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.2745 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08748 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.2745 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.28263 *  s_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(4,2) = (((((- 0.08748 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.2745 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08748 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.2745 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(5,0) = ((((((( 0.08748 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.2745 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08748 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.2745 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.28263 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.18979 *  c_q_LF_HAA_));
    (*this)(5,1) = (((((( 0.08748 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.2745 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.2745 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08748 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.28263 *  c_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(5,2) = ((((( 0.08748 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.2745 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.2745 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08748 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_WHEEL_L::Type_fr_base_J_fr_LH_WHEEL_L()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 0;
    (*this)(4,3) = 0;
    (*this)(5,3) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_WHEEL_L& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_WHEEL_L::update(const JState& jState) {
    Scalar s_q_LH_HAA_;
    Scalar s_q_LH_HFE_;
    Scalar s_q_LH_KFE_;
    Scalar c_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    
    s_q_LH_HAA_ = TRAIT::sin( jState(LH_HAA));
    s_q_LH_HFE_ = TRAIT::sin( jState(LH_HFE));
    s_q_LH_KFE_ = TRAIT::sin( jState(LH_KFE));
    c_q_LH_HAA_ = TRAIT::cos( jState(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( jState(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( jState(LH_KFE));
    
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(1,3) =  c_q_LH_HAA_;
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(2,3) =  s_q_LH_HAA_;
    (*this)(3,1) = ((((( 0.2745 *  s_q_LH_HFE_) + ( 0.08748 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.08748 *  s_q_LH_HFE_) - ( 0.2745 *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.28263 *  c_q_LH_HFE_));
    (*this)(3,2) = (((( 0.2745 *  s_q_LH_HFE_) + ( 0.08748 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.08748 *  s_q_LH_HFE_) - ( 0.2745 *  c_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(4,0) = (((((((- 0.2745 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08748 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.2745 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08748 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.28263 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.18979 *  s_q_LH_HAA_));
    (*this)(4,1) = (((((( 0.08748 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.2745 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.2745 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08748 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.28263 *  s_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(4,2) = ((((( 0.08748 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.2745 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.2745 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08748 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(5,0) = (((((((- 0.2745 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08748 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.2745 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08748 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.28263 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.18979 *  c_q_LH_HAA_));
    (*this)(5,1) = (((((( 0.2745 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08748 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.2745 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08748 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.28263 *  c_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(5,2) = ((((( 0.2745 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08748 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.2745 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08748 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_WHEEL_L::Type_fr_base_J_fr_RF_WHEEL_L()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 0;
    (*this)(4,3) = 0;
    (*this)(5,3) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_WHEEL_L& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_WHEEL_L::update(const JState& jState) {
    Scalar s_q_RF_HAA_;
    Scalar s_q_RF_HFE_;
    Scalar s_q_RF_KFE_;
    Scalar c_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    
    s_q_RF_HAA_ = TRAIT::sin( jState(RF_HAA));
    s_q_RF_HFE_ = TRAIT::sin( jState(RF_HFE));
    s_q_RF_KFE_ = TRAIT::sin( jState(RF_KFE));
    c_q_RF_HAA_ = TRAIT::cos( jState(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( jState(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( jState(RF_KFE));
    
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(1,3) =  c_q_RF_HAA_;
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(2,3) =  s_q_RF_HAA_;
    (*this)(3,1) = ((((( 0.2745 *  s_q_RF_HFE_) - ( 0.08748 *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((- 0.08748 *  s_q_RF_HFE_) - ( 0.2745 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.28263 *  c_q_RF_HFE_));
    (*this)(3,2) = (((( 0.2745 *  s_q_RF_HFE_) - ( 0.08748 *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((- 0.08748 *  s_q_RF_HFE_) - ( 0.2745 *  c_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(4,0) = ((((((( 0.08748 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.2745 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08748 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.2745 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.28263 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.18979 *  s_q_RF_HAA_));
    (*this)(4,1) = ((((((- 0.08748 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.2745 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08748 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.2745 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.28263 *  s_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(4,2) = (((((- 0.08748 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.2745 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08748 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.2745 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(5,0) = ((((((( 0.08748 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.2745 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08748 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.2745 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.28263 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.18979 *  c_q_RF_HAA_));
    (*this)(5,1) = (((((( 0.08748 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.2745 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.2745 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08748 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.28263 *  c_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(5,2) = ((((( 0.08748 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.2745 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.2745 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08748 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_WHEEL_L::Type_fr_base_J_fr_RH_WHEEL_L()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,3) = 0;
    (*this)(4,3) = 0;
    (*this)(5,3) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_WHEEL_L& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_WHEEL_L::update(const JState& jState) {
    Scalar s_q_RH_HAA_;
    Scalar s_q_RH_HFE_;
    Scalar s_q_RH_KFE_;
    Scalar c_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    
    s_q_RH_HAA_ = TRAIT::sin( jState(RH_HAA));
    s_q_RH_HFE_ = TRAIT::sin( jState(RH_HFE));
    s_q_RH_KFE_ = TRAIT::sin( jState(RH_KFE));
    c_q_RH_HAA_ = TRAIT::cos( jState(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( jState(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( jState(RH_KFE));
    
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(1,3) =  c_q_RH_HAA_;
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(2,3) =  s_q_RH_HAA_;
    (*this)(3,1) = ((((( 0.2745 *  s_q_RH_HFE_) + ( 0.08748 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.08748 *  s_q_RH_HFE_) - ( 0.2745 *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.28263 *  c_q_RH_HFE_));
    (*this)(3,2) = (((( 0.2745 *  s_q_RH_HFE_) + ( 0.08748 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.08748 *  s_q_RH_HFE_) - ( 0.2745 *  c_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(4,0) = (((((((- 0.2745 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08748 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.2745 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08748 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.28263 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.18979 *  s_q_RH_HAA_));
    (*this)(4,1) = (((((( 0.08748 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.2745 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.2745 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08748 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.28263 *  s_q_RH_HAA_) *  s_q_RH_HFE_));
    (*this)(4,2) = ((((( 0.08748 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.2745 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.2745 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08748 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(5,0) = (((((((- 0.2745 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08748 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.2745 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08748 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.28263 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.18979 *  c_q_RH_HAA_));
    (*this)(5,1) = (((((( 0.2745 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08748 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.2745 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08748 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.28263 *  c_q_RH_HAA_) *  s_q_RH_HFE_));
    (*this)(5,2) = ((((( 0.2745 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08748 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.2745 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08748 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_));
    return *this;
}
