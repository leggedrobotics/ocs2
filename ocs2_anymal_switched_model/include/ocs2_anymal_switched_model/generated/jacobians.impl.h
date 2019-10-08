
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Jacobians
    ()
     : 
    fr_base_J_fr_LF_HIP(), 
    fr_base_J_fr_LF_THIGH(), 
    fr_base_J_fr_LF_SHANK(), 
    fr_base_J_fr_RF_HIP(), 
    fr_base_J_fr_RF_THIGH(), 
    fr_base_J_fr_RF_SHANK(), 
    fr_base_J_fr_LH_HIP(), 
    fr_base_J_fr_LH_THIGH(), 
    fr_base_J_fr_LH_SHANK(), 
    fr_base_J_fr_RH_HIP(), 
    fr_base_J_fr_RH_THIGH(), 
    fr_base_J_fr_RH_SHANK(), 
    fr_base_J_fr_LF_ADAPTER(), 
    fr_base_J_fr_LF_FOOT(), 
    fr_base_J_fr_LF_HIP_COM(), 
    fr_base_J_fr_LF_SHANK_COM(), 
    fr_base_J_fr_LF_THIGH_COM(), 
    fr_base_J_fr_LH_ADAPTER(), 
    fr_base_J_fr_LH_FOOT(), 
    fr_base_J_fr_LH_HIP_COM(), 
    fr_base_J_fr_LH_SHANK_COM(), 
    fr_base_J_fr_LH_THIGH_COM(), 
    fr_base_J_fr_RF_ADAPTER(), 
    fr_base_J_fr_RF_FOOT(), 
    fr_base_J_fr_RF_HIP_COM(), 
    fr_base_J_fr_RF_SHANK_COM(), 
    fr_base_J_fr_RF_THIGH_COM(), 
    fr_base_J_fr_RH_ADAPTER(), 
    fr_base_J_fr_RH_FOOT(), 
    fr_base_J_fr_RH_HIP_COM(), 
    fr_base_J_fr_RH_SHANK_COM(), 
    fr_base_J_fr_RH_THIGH_COM()
{
    updateParameters();
}

template <typename TRAIT>
void iit::ANYmal::tpl::Jacobians<TRAIT>::updateParameters() {
}


template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_HIP::Type_fr_base_J_fr_LF_HIP()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(4,0) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_HIP& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_HIP::update(const JState& jState) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_THIGH::Type_fr_base_J_fr_LF_THIGH()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(4,1) = 0;
    (*this)(5,1) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_THIGH& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_THIGH::update(const JState& jState) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( jState(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( jState(LF_HAA));
    
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(4,0) = (- 0.041 *  s_q_LF_HAA_);
    (*this)(5,0) = ( 0.041 *  c_q_LF_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_SHANK::Type_fr_base_J_fr_LF_SHANK()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(4,2) = 0;
    (*this)(5,2) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_SHANK& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_SHANK::update(const JState& jState) {
    Scalar s_q_LF_HAA_;
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    
    s_q_LF_HAA_ = TRAIT::sin( jState(LF_HAA));
    s_q_LF_HFE_ = TRAIT::sin( jState(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( jState(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( jState(LF_HFE));
    
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(3,1) = (- 0.25 *  c_q_LF_HFE_);
    (*this)(4,0) = ((( 0.25 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - ( 0.15 *  s_q_LF_HAA_));
    (*this)(4,1) = ((- 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_);
    (*this)(5,0) = ((( 0.25 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + ( 0.15 *  c_q_LF_HAA_));
    (*this)(5,1) = (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_HIP::Type_fr_base_J_fr_RF_HIP()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(4,0) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_HIP& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_HIP::update(const JState& jState) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_THIGH::Type_fr_base_J_fr_RF_THIGH()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(4,1) = 0;
    (*this)(5,1) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_THIGH& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_THIGH::update(const JState& jState) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( jState(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( jState(RF_HAA));
    
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(4,0) = ( 0.041 *  s_q_RF_HAA_);
    (*this)(5,0) = (- 0.041 *  c_q_RF_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_SHANK::Type_fr_base_J_fr_RF_SHANK()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(4,2) = 0;
    (*this)(5,2) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_SHANK& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_SHANK::update(const JState& jState) {
    Scalar s_q_RF_HAA_;
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    
    s_q_RF_HAA_ = TRAIT::sin( jState(RF_HAA));
    s_q_RF_HFE_ = TRAIT::sin( jState(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( jState(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( jState(RF_HFE));
    
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(3,1) = (- 0.25 *  c_q_RF_HFE_);
    (*this)(4,0) = ((( 0.25 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + ( 0.15 *  s_q_RF_HAA_));
    (*this)(4,1) = ((- 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_);
    (*this)(5,0) = ((( 0.25 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - ( 0.15 *  c_q_RF_HAA_));
    (*this)(5,1) = (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_HIP::Type_fr_base_J_fr_LH_HIP()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(4,0) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_HIP& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_HIP::update(const JState& jState) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_THIGH::Type_fr_base_J_fr_LH_THIGH()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(4,1) = 0;
    (*this)(5,1) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_THIGH& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_THIGH::update(const JState& jState) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( jState(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( jState(LH_HAA));
    
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(4,0) = (- 0.041 *  s_q_LH_HAA_);
    (*this)(5,0) = ( 0.041 *  c_q_LH_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_SHANK::Type_fr_base_J_fr_LH_SHANK()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(4,2) = 0;
    (*this)(5,2) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_SHANK& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_SHANK::update(const JState& jState) {
    Scalar s_q_LH_HAA_;
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    
    s_q_LH_HAA_ = TRAIT::sin( jState(LH_HAA));
    s_q_LH_HFE_ = TRAIT::sin( jState(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( jState(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( jState(LH_HFE));
    
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(3,1) = (- 0.25 *  c_q_LH_HFE_);
    (*this)(4,0) = ((( 0.25 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - ( 0.15 *  s_q_LH_HAA_));
    (*this)(4,1) = ((- 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_);
    (*this)(5,0) = ((( 0.25 *  s_q_LH_HAA_) *  c_q_LH_HFE_) + ( 0.15 *  c_q_LH_HAA_));
    (*this)(5,1) = (( 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_HIP::Type_fr_base_J_fr_RH_HIP()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(4,0) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_HIP& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_HIP::update(const JState& jState) {
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_THIGH::Type_fr_base_J_fr_RH_THIGH()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(4,1) = 0;
    (*this)(5,1) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_THIGH& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_THIGH::update(const JState& jState) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( jState(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( jState(RH_HAA));
    
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(4,0) = ( 0.041 *  s_q_RH_HAA_);
    (*this)(5,0) = (- 0.041 *  c_q_RH_HAA_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_SHANK::Type_fr_base_J_fr_RH_SHANK()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(4,2) = 0;
    (*this)(5,2) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_SHANK& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_SHANK::update(const JState& jState) {
    Scalar s_q_RH_HAA_;
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    
    s_q_RH_HAA_ = TRAIT::sin( jState(RH_HAA));
    s_q_RH_HFE_ = TRAIT::sin( jState(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( jState(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( jState(RH_HFE));
    
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(3,1) = (- 0.25 *  c_q_RH_HFE_);
    (*this)(4,0) = ((( 0.25 *  c_q_RH_HAA_) *  c_q_RH_HFE_) + ( 0.15 *  s_q_RH_HAA_));
    (*this)(4,1) = ((- 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_);
    (*this)(5,0) = ((( 0.25 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - ( 0.15 *  c_q_RH_HAA_));
    (*this)(5,1) = (( 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_);
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_ADAPTER::Type_fr_base_J_fr_LF_ADAPTER()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_ADAPTER& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_ADAPTER::update(const JState& jState) {
    Scalar s_q_LF_HAA_;
    Scalar s_q_LF_KFE_;
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    Scalar c_q_LF_KFE_;
    
    s_q_LF_HAA_ = TRAIT::sin( jState(LF_HAA));
    s_q_LF_KFE_ = TRAIT::sin( jState(LF_KFE));
    s_q_LF_HFE_ = TRAIT::sin( jState(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( jState(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( jState(LF_HFE));
    c_q_LF_KFE_ = TRAIT::cos( jState(LF_KFE));
    
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(1,2) =  c_q_LF_HAA_;
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(2,2) =  s_q_LF_HAA_;
    (*this)(3,1) = ((((- 0.1 *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( 0.1 *  s_q_LF_HFE_) *  c_q_LF_KFE_)) - ( 0.25 *  c_q_LF_HFE_));
    (*this)(3,2) = (((- 0.1 *  c_q_LF_HFE_) *  s_q_LF_KFE_) - (( 0.1 *  s_q_LF_HFE_) *  c_q_LF_KFE_));
    (*this)(4,0) = (((((( 0.1 *  c_q_LF_HAA_) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.1 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  c_q_LF_KFE_)) + (( 0.25 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.13 *  s_q_LF_HAA_));
    (*this)(4,1) = (((((- 0.1 *  s_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.1 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) - (( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(4,2) = (((( 0.1 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_) - ((( 0.1 *  s_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_));
    (*this)(5,0) = (((((( 0.1 *  s_q_LF_HAA_) *  c_q_LF_HFE_) *  s_q_LF_KFE_) + ((( 0.1 *  s_q_LF_HAA_) *  s_q_LF_HFE_) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.13 *  c_q_LF_HAA_));
    (*this)(5,1) = ((((( 0.1 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) - ((( 0.1 *  c_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_)) + (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(5,2) = (((( 0.1 *  c_q_LF_HAA_) *  s_q_LF_HFE_) *  s_q_LF_KFE_) - ((( 0.1 *  c_q_LF_HAA_) *  c_q_LF_HFE_) *  c_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_FOOT::Type_fr_base_J_fr_LF_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_FOOT& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_FOOT::update(const JState& jState) {
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
    (*this)(3,1) = ((((( 0.32125 *  s_q_LF_HFE_) - ( 0.1 *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((- 0.32125 *  c_q_LF_HFE_) - ( 0.1 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.25 *  c_q_LF_HFE_));
    (*this)(3,2) = (((( 0.32125 *  s_q_LF_HFE_) - ( 0.1 *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((- 0.32125 *  c_q_LF_HFE_) - ( 0.1 *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(4,0) = ((((((( 0.1 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.32125 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.32125 *  c_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.1 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.25 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.13 *  s_q_LF_HAA_));
    (*this)(4,1) = ((((((- 0.32125 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.1 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.1 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.32125 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(4,2) = (((((- 0.32125 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.1 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.1 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.32125 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(5,0) = ((((((( 0.1 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.32125 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.32125 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.1 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.13 *  c_q_LF_HAA_));
    (*this)(5,1) = (((((( 0.32125 *  c_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.1 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.32125 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.1 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(5,2) = ((((( 0.32125 *  c_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.1 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.32125 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.1 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_HIP_COM::Type_fr_base_J_fr_LF_HIP_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_HIP_COM& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_HIP_COM::update(const JState& jState) {
    Scalar s_q_LF_HAA_;
    Scalar c_q_LF_HAA_;
    
    s_q_LF_HAA_ = TRAIT::sin( jState(LF_HAA));
    c_q_LF_HAA_ = TRAIT::cos( jState(LF_HAA));
    
    (*this)(4,0) = (( 0.00379 *  s_q_LF_HAA_) + ( 1.5E-4 *  c_q_LF_HAA_));
    (*this)(5,0) = (( 1.5E-4 *  s_q_LF_HAA_) - ( 0.00379 *  c_q_LF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_SHANK_COM::Type_fr_base_J_fr_LF_SHANK_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_SHANK_COM& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_SHANK_COM::update(const JState& jState) {
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
    (*this)(3,1) = ((((( 0.09806 *  s_q_LF_HFE_) - ( 0.05873 *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((- 0.09806 *  c_q_LF_HFE_) - ( 0.05873 *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - ( 0.25 *  c_q_LF_HFE_));
    (*this)(3,2) = (((( 0.09806 *  s_q_LF_HFE_) - ( 0.05873 *  c_q_LF_HFE_)) *  s_q_LF_KFE_) + (((- 0.09806 *  c_q_LF_HFE_) - ( 0.05873 *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(4,0) = ((((((( 0.05873 *  c_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.09806 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.09806 *  c_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.05873 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.25 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.13918 *  s_q_LF_HAA_));
    (*this)(4,1) = ((((((- 0.09806 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.05873 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.05873 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.09806 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) - (( 0.25 *  s_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(4,2) = (((((- 0.09806 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.05873 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.05873 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.09806 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_));
    (*this)(5,0) = ((((((( 0.05873 *  s_q_LF_HAA_) *  c_q_LF_HFE_) - (( 0.09806 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.09806 *  s_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.05873 *  s_q_LF_HAA_) *  s_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.25 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.13918 *  c_q_LF_HAA_));
    (*this)(5,1) = (((((( 0.09806 *  c_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.05873 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.09806 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.05873 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_)) + (( 0.25 *  c_q_LF_HAA_) *  s_q_LF_HFE_));
    (*this)(5,2) = ((((( 0.09806 *  c_q_LF_HAA_) *  c_q_LF_HFE_) + (( 0.05873 *  c_q_LF_HAA_) *  s_q_LF_HFE_)) *  s_q_LF_KFE_) + (((( 0.09806 *  c_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.05873 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) *  c_q_LF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_THIGH_COM::Type_fr_base_J_fr_LF_THIGH_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_THIGH_COM& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LF_THIGH_COM::update(const JState& jState) {
    Scalar s_q_LF_HAA_;
    Scalar s_q_LF_HFE_;
    Scalar c_q_LF_HAA_;
    Scalar c_q_LF_HFE_;
    
    s_q_LF_HAA_ = TRAIT::sin( jState(LF_HAA));
    s_q_LF_HFE_ = TRAIT::sin( jState(LF_HFE));
    c_q_LF_HAA_ = TRAIT::cos( jState(LF_HAA));
    c_q_LF_HFE_ = TRAIT::cos( jState(LF_HFE));
    
    (*this)(1,1) =  c_q_LF_HAA_;
    (*this)(2,1) =  s_q_LF_HAA_;
    (*this)(3,1) = (( 0.0039 *  s_q_LF_HFE_) - ( 0.21458 *  c_q_LF_HFE_));
    (*this)(4,0) = ((((- 0.0039 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.21458 *  c_q_LF_HAA_) *  c_q_LF_HFE_)) - ( 0.09523 *  s_q_LF_HAA_));
    (*this)(4,1) = (((- 0.21458 *  s_q_LF_HAA_) *  s_q_LF_HFE_) - (( 0.0039 *  s_q_LF_HAA_) *  c_q_LF_HFE_));
    (*this)(5,0) = ((((- 0.0039 *  s_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.21458 *  s_q_LF_HAA_) *  c_q_LF_HFE_)) + ( 0.09523 *  c_q_LF_HAA_));
    (*this)(5,1) = ((( 0.21458 *  c_q_LF_HAA_) *  s_q_LF_HFE_) + (( 0.0039 *  c_q_LF_HAA_) *  c_q_LF_HFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_ADAPTER::Type_fr_base_J_fr_LH_ADAPTER()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_ADAPTER& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_ADAPTER::update(const JState& jState) {
    Scalar s_q_LH_HAA_;
    Scalar s_q_LH_KFE_;
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    Scalar c_q_LH_KFE_;
    
    s_q_LH_HAA_ = TRAIT::sin( jState(LH_HAA));
    s_q_LH_KFE_ = TRAIT::sin( jState(LH_KFE));
    s_q_LH_HFE_ = TRAIT::sin( jState(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( jState(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( jState(LH_HFE));
    c_q_LH_KFE_ = TRAIT::cos( jState(LH_KFE));
    
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(1,2) =  c_q_LH_HAA_;
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(2,2) =  s_q_LH_HAA_;
    (*this)(3,1) = (((( 0.1 *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( 0.1 *  s_q_LH_HFE_) *  c_q_LH_KFE_)) - ( 0.25 *  c_q_LH_HFE_));
    (*this)(3,2) = ((( 0.1 *  c_q_LH_HFE_) *  s_q_LH_KFE_) + (( 0.1 *  s_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(4,0) = ((((((- 0.1 *  c_q_LH_HAA_) *  c_q_LH_HFE_) *  s_q_LH_KFE_) - ((( 0.1 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  c_q_LH_KFE_)) + (( 0.25 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.13 *  s_q_LH_HAA_));
    (*this)(4,1) = ((((( 0.1 *  s_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) - ((( 0.1 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) - (( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(4,2) = (((( 0.1 *  s_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) - ((( 0.1 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_));
    (*this)(5,0) = ((((((- 0.1 *  s_q_LH_HAA_) *  c_q_LH_HFE_) *  s_q_LH_KFE_) - ((( 0.1 *  s_q_LH_HAA_) *  s_q_LH_HFE_) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.13 *  c_q_LH_HAA_));
    (*this)(5,1) = (((((- 0.1 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_) + ((( 0.1 *  c_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_)) + (( 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(5,2) = (((( 0.1 *  c_q_LH_HAA_) *  c_q_LH_HFE_) *  c_q_LH_KFE_) - ((( 0.1 *  c_q_LH_HAA_) *  s_q_LH_HFE_) *  s_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_FOOT::Type_fr_base_J_fr_LH_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_FOOT& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_FOOT::update(const JState& jState) {
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
    (*this)(3,1) = ((((( 0.1 *  c_q_LH_HFE_) + ( 0.32125 *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.1 *  s_q_LH_HFE_) - ( 0.32125 *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.25 *  c_q_LH_HFE_));
    (*this)(3,2) = (((( 0.1 *  c_q_LH_HFE_) + ( 0.32125 *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.1 *  s_q_LH_HFE_) - ( 0.32125 *  c_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(4,0) = (((((((- 0.1 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.32125 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.32125 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.1 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.25 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.13 *  s_q_LH_HAA_));
    (*this)(4,1) = (((((( 0.1 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.32125 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.1 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.32125 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(4,2) = ((((( 0.1 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.32125 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.1 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.32125 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(5,0) = (((((((- 0.1 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.32125 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.32125 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.1 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.13 *  c_q_LH_HAA_));
    (*this)(5,1) = (((((( 0.32125 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.1 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.1 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.32125 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(5,2) = ((((( 0.32125 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.1 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.1 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.32125 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_HIP_COM::Type_fr_base_J_fr_LH_HIP_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_HIP_COM& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_HIP_COM::update(const JState& jState) {
    Scalar s_q_LH_HAA_;
    Scalar c_q_LH_HAA_;
    
    s_q_LH_HAA_ = TRAIT::sin( jState(LH_HAA));
    c_q_LH_HAA_ = TRAIT::cos( jState(LH_HAA));
    
    (*this)(4,0) = (( 0.00379 *  s_q_LH_HAA_) + ( 1.5E-4 *  c_q_LH_HAA_));
    (*this)(5,0) = (( 1.5E-4 *  s_q_LH_HAA_) - ( 0.00379 *  c_q_LH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_SHANK_COM::Type_fr_base_J_fr_LH_SHANK_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_SHANK_COM& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_SHANK_COM::update(const JState& jState) {
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
    (*this)(3,1) = ((((( 0.05873 *  c_q_LH_HFE_) + ( 0.09806 *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.05873 *  s_q_LH_HFE_) - ( 0.09806 *  c_q_LH_HFE_)) *  c_q_LH_KFE_)) - ( 0.25 *  c_q_LH_HFE_));
    (*this)(3,2) = (((( 0.05873 *  c_q_LH_HFE_) + ( 0.09806 *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + ((( 0.05873 *  s_q_LH_HFE_) - ( 0.09806 *  c_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(4,0) = (((((((- 0.05873 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.09806 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.09806 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.05873 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.25 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.13918 *  s_q_LH_HAA_));
    (*this)(4,1) = (((((( 0.05873 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.09806 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.05873 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.09806 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) - (( 0.25 *  s_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(4,2) = ((((( 0.05873 *  s_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.09806 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) *  s_q_LH_KFE_) + ((((- 0.05873 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.09806 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    (*this)(5,0) = (((((((- 0.05873 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.09806 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.09806 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.05873 *  s_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.25 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.13918 *  c_q_LH_HAA_));
    (*this)(5,1) = (((((( 0.09806 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.05873 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.05873 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.09806 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_)) + (( 0.25 *  c_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(5,2) = ((((( 0.09806 *  c_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.05873 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  s_q_LH_KFE_) + (((( 0.05873 *  c_q_LH_HAA_) *  c_q_LH_HFE_) + (( 0.09806 *  c_q_LH_HAA_) *  s_q_LH_HFE_)) *  c_q_LH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_THIGH_COM::Type_fr_base_J_fr_LH_THIGH_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_THIGH_COM& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_LH_THIGH_COM::update(const JState& jState) {
    Scalar s_q_LH_HAA_;
    Scalar s_q_LH_HFE_;
    Scalar c_q_LH_HAA_;
    Scalar c_q_LH_HFE_;
    
    s_q_LH_HAA_ = TRAIT::sin( jState(LH_HAA));
    s_q_LH_HFE_ = TRAIT::sin( jState(LH_HFE));
    c_q_LH_HAA_ = TRAIT::cos( jState(LH_HAA));
    c_q_LH_HFE_ = TRAIT::cos( jState(LH_HFE));
    
    (*this)(1,1) =  c_q_LH_HAA_;
    (*this)(2,1) =  s_q_LH_HAA_;
    (*this)(3,1) = ((- 0.0039 *  s_q_LH_HFE_) - ( 0.21458 *  c_q_LH_HFE_));
    (*this)(4,0) = (((( 0.0039 *  c_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.21458 *  c_q_LH_HAA_) *  c_q_LH_HFE_)) - ( 0.09523 *  s_q_LH_HAA_));
    (*this)(4,1) = ((( 0.0039 *  s_q_LH_HAA_) *  c_q_LH_HFE_) - (( 0.21458 *  s_q_LH_HAA_) *  s_q_LH_HFE_));
    (*this)(5,0) = (((( 0.0039 *  s_q_LH_HAA_) *  s_q_LH_HFE_) + (( 0.21458 *  s_q_LH_HAA_) *  c_q_LH_HFE_)) + ( 0.09523 *  c_q_LH_HAA_));
    (*this)(5,1) = ((( 0.21458 *  c_q_LH_HAA_) *  s_q_LH_HFE_) - (( 0.0039 *  c_q_LH_HAA_) *  c_q_LH_HFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_ADAPTER::Type_fr_base_J_fr_RF_ADAPTER()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_ADAPTER& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_ADAPTER::update(const JState& jState) {
    Scalar s_q_RF_HAA_;
    Scalar s_q_RF_KFE_;
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    Scalar c_q_RF_KFE_;
    
    s_q_RF_HAA_ = TRAIT::sin( jState(RF_HAA));
    s_q_RF_KFE_ = TRAIT::sin( jState(RF_KFE));
    s_q_RF_HFE_ = TRAIT::sin( jState(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( jState(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( jState(RF_HFE));
    c_q_RF_KFE_ = TRAIT::cos( jState(RF_KFE));
    
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(1,2) =  c_q_RF_HAA_;
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(2,2) =  s_q_RF_HAA_;
    (*this)(3,1) = ((((- 0.1 *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( 0.1 *  s_q_RF_HFE_) *  c_q_RF_KFE_)) - ( 0.25 *  c_q_RF_HFE_));
    (*this)(3,2) = (((- 0.1 *  c_q_RF_HFE_) *  s_q_RF_KFE_) - (( 0.1 *  s_q_RF_HFE_) *  c_q_RF_KFE_));
    (*this)(4,0) = (((((( 0.1 *  c_q_RF_HAA_) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.1 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  c_q_RF_KFE_)) + (( 0.25 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.13 *  s_q_RF_HAA_));
    (*this)(4,1) = (((((- 0.1 *  s_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.1 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) - (( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(4,2) = (((( 0.1 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_) - ((( 0.1 *  s_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_));
    (*this)(5,0) = (((((( 0.1 *  s_q_RF_HAA_) *  c_q_RF_HFE_) *  s_q_RF_KFE_) + ((( 0.1 *  s_q_RF_HAA_) *  s_q_RF_HFE_) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.13 *  c_q_RF_HAA_));
    (*this)(5,1) = ((((( 0.1 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) - ((( 0.1 *  c_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_)) + (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(5,2) = (((( 0.1 *  c_q_RF_HAA_) *  s_q_RF_HFE_) *  s_q_RF_KFE_) - ((( 0.1 *  c_q_RF_HAA_) *  c_q_RF_HFE_) *  c_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_FOOT::Type_fr_base_J_fr_RF_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_FOOT& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_FOOT::update(const JState& jState) {
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
    (*this)(3,1) = ((((( 0.32125 *  s_q_RF_HFE_) - ( 0.1 *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((- 0.32125 *  c_q_RF_HFE_) - ( 0.1 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.25 *  c_q_RF_HFE_));
    (*this)(3,2) = (((( 0.32125 *  s_q_RF_HFE_) - ( 0.1 *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((- 0.32125 *  c_q_RF_HFE_) - ( 0.1 *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(4,0) = ((((((( 0.1 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.32125 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.32125 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.1 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.25 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.13 *  s_q_RF_HAA_));
    (*this)(4,1) = ((((((- 0.32125 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.1 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.1 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.32125 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(4,2) = (((((- 0.32125 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.1 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.1 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.32125 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(5,0) = ((((((( 0.1 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.32125 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.32125 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.1 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.13 *  c_q_RF_HAA_));
    (*this)(5,1) = (((((( 0.32125 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.1 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.32125 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.1 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(5,2) = ((((( 0.32125 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.1 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.32125 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.1 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_HIP_COM::Type_fr_base_J_fr_RF_HIP_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_HIP_COM& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_HIP_COM::update(const JState& jState) {
    Scalar s_q_RF_HAA_;
    Scalar c_q_RF_HAA_;
    
    s_q_RF_HAA_ = TRAIT::sin( jState(RF_HAA));
    c_q_RF_HAA_ = TRAIT::cos( jState(RF_HAA));
    
    (*this)(4,0) = (( 1.5E-4 *  c_q_RF_HAA_) - ( 0.00379 *  s_q_RF_HAA_));
    (*this)(5,0) = (( 1.5E-4 *  s_q_RF_HAA_) + ( 0.00379 *  c_q_RF_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_SHANK_COM::Type_fr_base_J_fr_RF_SHANK_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_SHANK_COM& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_SHANK_COM::update(const JState& jState) {
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
    (*this)(3,1) = ((((( 0.09806 *  s_q_RF_HFE_) - ( 0.05873 *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((- 0.09806 *  c_q_RF_HFE_) - ( 0.05873 *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - ( 0.25 *  c_q_RF_HFE_));
    (*this)(3,2) = (((( 0.09806 *  s_q_RF_HFE_) - ( 0.05873 *  c_q_RF_HFE_)) *  s_q_RF_KFE_) + (((- 0.09806 *  c_q_RF_HFE_) - ( 0.05873 *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(4,0) = ((((((( 0.05873 *  c_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.09806 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.09806 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.05873 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.25 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.13918 *  s_q_RF_HAA_));
    (*this)(4,1) = ((((((- 0.09806 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.05873 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.05873 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.09806 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) - (( 0.25 *  s_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(4,2) = (((((- 0.09806 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.05873 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.05873 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.09806 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_));
    (*this)(5,0) = ((((((( 0.05873 *  s_q_RF_HAA_) *  c_q_RF_HFE_) - (( 0.09806 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.09806 *  s_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.05873 *  s_q_RF_HAA_) *  s_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.25 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.13918 *  c_q_RF_HAA_));
    (*this)(5,1) = (((((( 0.09806 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.05873 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.09806 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.05873 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_)) + (( 0.25 *  c_q_RF_HAA_) *  s_q_RF_HFE_));
    (*this)(5,2) = ((((( 0.09806 *  c_q_RF_HAA_) *  c_q_RF_HFE_) + (( 0.05873 *  c_q_RF_HAA_) *  s_q_RF_HFE_)) *  s_q_RF_KFE_) + (((( 0.09806 *  c_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.05873 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) *  c_q_RF_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_THIGH_COM::Type_fr_base_J_fr_RF_THIGH_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_THIGH_COM& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RF_THIGH_COM::update(const JState& jState) {
    Scalar s_q_RF_HAA_;
    Scalar s_q_RF_HFE_;
    Scalar c_q_RF_HAA_;
    Scalar c_q_RF_HFE_;
    
    s_q_RF_HAA_ = TRAIT::sin( jState(RF_HAA));
    s_q_RF_HFE_ = TRAIT::sin( jState(RF_HFE));
    c_q_RF_HAA_ = TRAIT::cos( jState(RF_HAA));
    c_q_RF_HFE_ = TRAIT::cos( jState(RF_HFE));
    
    (*this)(1,1) =  c_q_RF_HAA_;
    (*this)(2,1) =  s_q_RF_HAA_;
    (*this)(3,1) = (( 0.0039 *  s_q_RF_HFE_) - ( 0.21458 *  c_q_RF_HFE_));
    (*this)(4,0) = ((((- 0.0039 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.21458 *  c_q_RF_HAA_) *  c_q_RF_HFE_)) + ( 0.09523 *  s_q_RF_HAA_));
    (*this)(4,1) = (((- 0.21458 *  s_q_RF_HAA_) *  s_q_RF_HFE_) - (( 0.0039 *  s_q_RF_HAA_) *  c_q_RF_HFE_));
    (*this)(5,0) = ((((- 0.0039 *  s_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.21458 *  s_q_RF_HAA_) *  c_q_RF_HFE_)) - ( 0.09523 *  c_q_RF_HAA_));
    (*this)(5,1) = ((( 0.21458 *  c_q_RF_HAA_) *  s_q_RF_HFE_) + (( 0.0039 *  c_q_RF_HAA_) *  c_q_RF_HFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_ADAPTER::Type_fr_base_J_fr_RH_ADAPTER()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_ADAPTER& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_ADAPTER::update(const JState& jState) {
    Scalar s_q_RH_HAA_;
    Scalar s_q_RH_KFE_;
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    Scalar c_q_RH_KFE_;
    
    s_q_RH_HAA_ = TRAIT::sin( jState(RH_HAA));
    s_q_RH_KFE_ = TRAIT::sin( jState(RH_KFE));
    s_q_RH_HFE_ = TRAIT::sin( jState(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( jState(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( jState(RH_HFE));
    c_q_RH_KFE_ = TRAIT::cos( jState(RH_KFE));
    
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(1,2) =  c_q_RH_HAA_;
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(2,2) =  s_q_RH_HAA_;
    (*this)(3,1) = (((( 0.1 *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( 0.1 *  s_q_RH_HFE_) *  c_q_RH_KFE_)) - ( 0.25 *  c_q_RH_HFE_));
    (*this)(3,2) = ((( 0.1 *  c_q_RH_HFE_) *  s_q_RH_KFE_) + (( 0.1 *  s_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(4,0) = ((((((- 0.1 *  c_q_RH_HAA_) *  c_q_RH_HFE_) *  s_q_RH_KFE_) - ((( 0.1 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  c_q_RH_KFE_)) + (( 0.25 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.13 *  s_q_RH_HAA_));
    (*this)(4,1) = ((((( 0.1 *  s_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) - ((( 0.1 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) - (( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_));
    (*this)(4,2) = (((( 0.1 *  s_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) - ((( 0.1 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_));
    (*this)(5,0) = ((((((- 0.1 *  s_q_RH_HAA_) *  c_q_RH_HFE_) *  s_q_RH_KFE_) - ((( 0.1 *  s_q_RH_HAA_) *  s_q_RH_HFE_) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.13 *  c_q_RH_HAA_));
    (*this)(5,1) = (((((- 0.1 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_) + ((( 0.1 *  c_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_)) + (( 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_));
    (*this)(5,2) = (((( 0.1 *  c_q_RH_HAA_) *  c_q_RH_HFE_) *  c_q_RH_KFE_) - ((( 0.1 *  c_q_RH_HAA_) *  s_q_RH_HFE_) *  s_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_FOOT::Type_fr_base_J_fr_RH_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_FOOT& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_FOOT::update(const JState& jState) {
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
    (*this)(3,1) = ((((( 0.1 *  c_q_RH_HFE_) + ( 0.32125 *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.1 *  s_q_RH_HFE_) - ( 0.32125 *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.25 *  c_q_RH_HFE_));
    (*this)(3,2) = (((( 0.1 *  c_q_RH_HFE_) + ( 0.32125 *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.1 *  s_q_RH_HFE_) - ( 0.32125 *  c_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(4,0) = (((((((- 0.1 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.32125 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.32125 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.1 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.25 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.13 *  s_q_RH_HAA_));
    (*this)(4,1) = (((((( 0.1 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.32125 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.1 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.32125 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_));
    (*this)(4,2) = ((((( 0.1 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.32125 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.1 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.32125 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(5,0) = (((((((- 0.1 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.32125 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.32125 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.1 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.13 *  c_q_RH_HAA_));
    (*this)(5,1) = (((((( 0.32125 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.1 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.1 *  c_q_RH_HAA_) *  c_q_RH_HFE_) + (( 0.32125 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_));
    (*this)(5,2) = ((((( 0.32125 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.1 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.1 *  c_q_RH_HAA_) *  c_q_RH_HFE_) + (( 0.32125 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_HIP_COM::Type_fr_base_J_fr_RH_HIP_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_HIP_COM& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_HIP_COM::update(const JState& jState) {
    Scalar s_q_RH_HAA_;
    Scalar c_q_RH_HAA_;
    
    s_q_RH_HAA_ = TRAIT::sin( jState(RH_HAA));
    c_q_RH_HAA_ = TRAIT::cos( jState(RH_HAA));
    
    (*this)(4,0) = (( 1.5E-4 *  c_q_RH_HAA_) - ( 0.00379 *  s_q_RH_HAA_));
    (*this)(5,0) = (( 1.5E-4 *  s_q_RH_HAA_) + ( 0.00379 *  c_q_RH_HAA_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_SHANK_COM::Type_fr_base_J_fr_RH_SHANK_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_SHANK_COM& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_SHANK_COM::update(const JState& jState) {
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
    (*this)(3,1) = ((((( 0.05873 *  c_q_RH_HFE_) + ( 0.09806 *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.05873 *  s_q_RH_HFE_) - ( 0.09806 *  c_q_RH_HFE_)) *  c_q_RH_KFE_)) - ( 0.25 *  c_q_RH_HFE_));
    (*this)(3,2) = (((( 0.05873 *  c_q_RH_HFE_) + ( 0.09806 *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + ((( 0.05873 *  s_q_RH_HFE_) - ( 0.09806 *  c_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(4,0) = (((((((- 0.05873 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.09806 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.09806 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.05873 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.25 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.13918 *  s_q_RH_HAA_));
    (*this)(4,1) = (((((( 0.05873 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.09806 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.05873 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.09806 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) - (( 0.25 *  s_q_RH_HAA_) *  s_q_RH_HFE_));
    (*this)(4,2) = ((((( 0.05873 *  s_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.09806 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) *  s_q_RH_KFE_) + ((((- 0.05873 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.09806 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    (*this)(5,0) = (((((((- 0.05873 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.09806 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.09806 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.05873 *  s_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.25 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.13918 *  c_q_RH_HAA_));
    (*this)(5,1) = (((((( 0.09806 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.05873 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.05873 *  c_q_RH_HAA_) *  c_q_RH_HFE_) + (( 0.09806 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_)) + (( 0.25 *  c_q_RH_HAA_) *  s_q_RH_HFE_));
    (*this)(5,2) = ((((( 0.09806 *  c_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.05873 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  s_q_RH_KFE_) + (((( 0.05873 *  c_q_RH_HAA_) *  c_q_RH_HFE_) + (( 0.09806 *  c_q_RH_HAA_) *  s_q_RH_HFE_)) *  c_q_RH_KFE_));
    return *this;
}
template <typename TRAIT>
iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_THIGH_COM::Type_fr_base_J_fr_RH_THIGH_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

template <typename TRAIT>
const typename iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_THIGH_COM& iit::ANYmal::tpl::Jacobians<TRAIT>::Type_fr_base_J_fr_RH_THIGH_COM::update(const JState& jState) {
    Scalar s_q_RH_HAA_;
    Scalar s_q_RH_HFE_;
    Scalar c_q_RH_HAA_;
    Scalar c_q_RH_HFE_;
    
    s_q_RH_HAA_ = TRAIT::sin( jState(RH_HAA));
    s_q_RH_HFE_ = TRAIT::sin( jState(RH_HFE));
    c_q_RH_HAA_ = TRAIT::cos( jState(RH_HAA));
    c_q_RH_HFE_ = TRAIT::cos( jState(RH_HFE));
    
    (*this)(1,1) =  c_q_RH_HAA_;
    (*this)(2,1) =  s_q_RH_HAA_;
    (*this)(3,1) = ((- 0.0039 *  s_q_RH_HFE_) - ( 0.21458 *  c_q_RH_HFE_));
    (*this)(4,0) = (((( 0.0039 *  c_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.21458 *  c_q_RH_HAA_) *  c_q_RH_HFE_)) + ( 0.09523 *  s_q_RH_HAA_));
    (*this)(4,1) = ((( 0.0039 *  s_q_RH_HAA_) *  c_q_RH_HFE_) - (( 0.21458 *  s_q_RH_HAA_) *  s_q_RH_HFE_));
    (*this)(5,0) = (((( 0.0039 *  s_q_RH_HAA_) *  s_q_RH_HFE_) + (( 0.21458 *  s_q_RH_HAA_) *  c_q_RH_HFE_)) - ( 0.09523 *  c_q_RH_HAA_));
    (*this)(5,1) = ((( 0.21458 *  c_q_RH_HAA_) *  s_q_RH_HFE_) - (( 0.0039 *  c_q_RH_HAA_) *  c_q_RH_HFE_));
    return *this;
}
