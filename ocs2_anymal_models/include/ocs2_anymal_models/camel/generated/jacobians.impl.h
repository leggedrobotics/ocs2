
template <typename TRAIT>
iit::camel::tpl::Jacobians<TRAIT>::Jacobians
    ()
     : 
    fr_base_J_fr_LF_FOOT(), 
    fr_base_J_fr_LH_FOOT(), 
    fr_base_J_fr_RF_FOOT(), 
    fr_base_J_fr_RH_FOOT()
{
    updateParameters();
}

template <typename TRAIT>
void iit::camel::tpl::Jacobians<TRAIT>::updateParameters() {
}


template <typename TRAIT>
iit::camel::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_FOOT::Type_fr_base_J_fr_LF_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::camel::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_FOOT& iit::camel::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_FOOT::update(const JState& jState) {
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
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(3,1) = ((((( 0.33797 *  s_q_LF_HFE_) - ( 0.08795 *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((- 0.08795 *  s_q_LF_HFE_) - ( 0.33797 *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.285 *  c_q_LF_HFE_));
    (*this)(3,2) = (((( 0.33797 *  s_q_LF_HFE_) - ( 0.08795 *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((- 0.08795 *  s_q_LF_HFE_) - ( 0.33797 *  c_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(4,0) = ((((((( 0.08795 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.33797 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08795 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.33797 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.285 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.19716 *  s_q_LF_HAA_));
    (*this)(4,1) = ((((((- 0.08795 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.33797 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08795 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.33797 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.285 *  s_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(4,2) = (((((- 0.08795 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.33797 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08795 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.33797 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(5,0) = ((((((( 0.08795 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.33797 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.08795 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.33797 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.285 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.19716 *  c_q_LF_HAA_));
    (*this)(5,1) = (((((( 0.08795 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.33797 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.33797 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08795 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.285 *  c_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(5,2) = ((((( 0.08795 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.33797 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.33797 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.08795 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_FOOT::Type_fr_base_J_fr_LH_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::camel::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_FOOT& iit::camel::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_FOOT::update(const JState& jState) {
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
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(3,1) = ((((( 0.33797 *  s_q_LH_HFE_) + ( 0.08795 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.08795 *  s_q_LH_HFE_) - ( 0.33797 *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.285 *  c_q_LH_HFE_));
    (*this)(3,2) = (((( 0.33797 *  s_q_LH_HFE_) + ( 0.08795 *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.08795 *  s_q_LH_HFE_) - ( 0.33797 *  c_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(4,0) = (((((((- 0.33797 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08795 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.33797 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08795 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.285 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.19716 *  s_q_LH_HAA_));
    (*this)(4,1) = (((((( 0.08795 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.33797 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.33797 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08795 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.285 *  s_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(4,2) = ((((( 0.08795 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.33797 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.33797 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08795 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(5,0) = (((((((- 0.33797 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.08795 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.33797 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08795 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.285 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.19716 *  c_q_LH_HAA_));
    (*this)(5,1) = (((((( 0.33797 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08795 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.33797 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08795 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.285 *  c_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(5,2) = ((((( 0.33797 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.08795 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.33797 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.08795 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) *  c_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_FOOT::Type_fr_base_J_fr_RF_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::camel::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_FOOT& iit::camel::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_FOOT::update(const JState& jState) {
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
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(3,1) = ((((( 0.33797 *  s_q_RF_HFE_) - ( 0.08795 *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((- 0.08795 *  s_q_RF_HFE_) - ( 0.33797 *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.285 *  c_q_RF_HFE_));
    (*this)(3,2) = (((( 0.33797 *  s_q_RF_HFE_) - ( 0.08795 *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((- 0.08795 *  s_q_RF_HFE_) - ( 0.33797 *  c_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(4,0) = ((((((( 0.08795 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.33797 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08795 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.33797 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.285 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.19716 *  s_q_RF_HAA_));
    (*this)(4,1) = ((((((- 0.08795 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.33797 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08795 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.33797 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.285 *  s_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(4,2) = (((((- 0.08795 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.33797 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08795 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.33797 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(5,0) = ((((((( 0.08795 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.33797 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.08795 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.33797 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.285 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.19716 *  c_q_RF_HAA_));
    (*this)(5,1) = (((((( 0.08795 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.33797 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.33797 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08795 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.285 *  c_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(5,2) = ((((( 0.08795 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.33797 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.33797 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.08795 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::camel::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_FOOT::Type_fr_base_J_fr_RH_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::camel::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_FOOT& iit::camel::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_FOOT::update(const JState& jState) {
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
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(3,1) = ((((( 0.33797 *  s_q_RH_HFE_) + ( 0.08795 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.08795 *  s_q_RH_HFE_) - ( 0.33797 *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.285 *  c_q_RH_HFE_));
    (*this)(3,2) = (((( 0.33797 *  s_q_RH_HFE_) + ( 0.08795 *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.08795 *  s_q_RH_HFE_) - ( 0.33797 *  c_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(4,0) = (((((((- 0.33797 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08795 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.33797 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08795 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.285 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.19716 *  s_q_RH_HAA_));
    (*this)(4,1) = (((((( 0.08795 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.33797 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.33797 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08795 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.285 *  s_q_RH_HAA_) *  s_q_RH_HFE_));
    (*this)(4,2) = ((((( 0.08795 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.33797 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.33797 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08795 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(5,0) = (((((((- 0.33797 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.08795 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.33797 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08795 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.285 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.19716 *  c_q_RH_HAA_));
    (*this)(5,1) = (((((( 0.33797 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08795 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.33797 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08795 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.285 *  c_q_RH_HAA_) *  s_q_RH_HFE_));
    (*this)(5,2) = ((((( 0.33797 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.08795 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.33797 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.08795 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) *  c_q_RH_KFE_));
    return *this;
}
