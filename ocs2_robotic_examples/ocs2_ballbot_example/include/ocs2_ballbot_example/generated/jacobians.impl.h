
template <typename TRAIT>
iit::Ballbot::tpl::Jacobians<TRAIT>::Jacobians
    ()
     : 
    fr_world_J_fr_dummy_ball1(), 
    fr_world_J_fr_ball(), 
    fr_world_J_fr_dummy_base1(), 
    fr_world_J_fr_dummy_base2(), 
    fr_world_J_fr_base(), 
    fr_world_J_fr_ball_COM(), 
    fr_world_J_fr_base_COM(), 
    fr_world_J_fr_dummy_ball1_COM(), 
    fr_world_J_fr_dummy_base1_COM(), 
    fr_world_J_fr_dummy_base2_COM()
{
    updateParameters();
}

template <typename TRAIT>
void iit::Ballbot::tpl::Jacobians<TRAIT>::updateParameters() {
}


template <typename TRAIT>
iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_ball1::Type_fr_world_J_fr_dummy_ball1()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 1.0;
    (*this)(4,0) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_ball1& iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_ball1::update(const JState& jState) {
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_ball::Type_fr_world_J_fr_ball()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(3,0) = 1.0;
    (*this)(3,1) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
}

template <typename TRAIT>
const typename iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_ball& iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_ball::update(const JState& jState) {
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_base1::Type_fr_world_J_fr_dummy_base1()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(3,0) = 1.0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 1.0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}

template <typename TRAIT>
const typename iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_base1& iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_base1::update(const JState& jState) {
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_base2::Type_fr_world_J_fr_dummy_base2()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 1.0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 1.0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}

template <typename TRAIT>
const typename iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_base2& iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_base2::update(const JState& jState) {
    Scalar s_q_jbase_z_;
    Scalar c_q_jbase_z_;
    
    s_q_jbase_z_ = TRAIT::sin( jState(JBASE_Z));
    c_q_jbase_z_ = TRAIT::cos( jState(JBASE_Z));
    
    (*this)(0,3) = - s_q_jbase_z_;
    (*this)(1,3) =  c_q_jbase_z_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_base::Type_fr_world_J_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 1.0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 1.0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
}

template <typename TRAIT>
const typename iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_base& iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_base::update(const JState& jState) {
    Scalar s_q_jbase_z_;
    Scalar s_q_jbase_y_;
    Scalar c_q_jbase_y_;
    Scalar c_q_jbase_z_;
    
    s_q_jbase_z_ = TRAIT::sin( jState(JBASE_Z));
    s_q_jbase_y_ = TRAIT::sin( jState(JBASE_Y));
    c_q_jbase_y_ = TRAIT::cos( jState(JBASE_Y));
    c_q_jbase_z_ = TRAIT::cos( jState(JBASE_Z));
    
    (*this)(0,3) = - s_q_jbase_z_;
    (*this)(0,4) = ( c_q_jbase_y_ *  c_q_jbase_z_);
    (*this)(1,3) =  c_q_jbase_z_;
    (*this)(1,4) = ( c_q_jbase_y_ *  s_q_jbase_z_);
    (*this)(2,4) = - s_q_jbase_y_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_ball_COM::Type_fr_world_J_fr_ball_COM()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(3,0) = 1.0;
    (*this)(3,1) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
}

template <typename TRAIT>
const typename iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_ball_COM& iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_ball_COM::update(const JState& jState) {
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_base_COM::Type_fr_world_J_fr_base_COM()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 1.0;
    (*this)(3,1) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}

template <typename TRAIT>
const typename iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_base_COM& iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_base_COM::update(const JState& jState) {
    Scalar s_q_jbase_z_;
    Scalar s_q_jbase_y_;
    Scalar s_q_jbase_x_;
    Scalar c_q_jbase_y_;
    Scalar c_q_jbase_z_;
    Scalar c_q_jbase_x_;
    
    s_q_jbase_z_ = TRAIT::sin( jState(JBASE_Z));
    s_q_jbase_y_ = TRAIT::sin( jState(JBASE_Y));
    s_q_jbase_x_ = TRAIT::sin( jState(JBASE_X));
    c_q_jbase_y_ = TRAIT::cos( jState(JBASE_Y));
    c_q_jbase_z_ = TRAIT::cos( jState(JBASE_Z));
    c_q_jbase_x_ = TRAIT::cos( jState(JBASE_X));
    
    (*this)(0,3) = - s_q_jbase_z_;
    (*this)(0,4) = ( c_q_jbase_y_ *  c_q_jbase_z_);
    (*this)(1,3) =  c_q_jbase_z_;
    (*this)(1,4) = ( c_q_jbase_y_ *  s_q_jbase_z_);
    (*this)(2,4) = - s_q_jbase_y_;
    (*this)(3,2) = ((((((- 0.0033 *  s_q_jbase_x_) - ( 0.1956 *  c_q_jbase_x_)) *  s_q_jbase_y_) - ( 0.003 *  c_q_jbase_y_)) *  s_q_jbase_z_) + ((( 0.1956 *  s_q_jbase_x_) - ( 0.0033 *  c_q_jbase_x_)) *  c_q_jbase_z_));
    (*this)(3,3) = ((((( 0.0033 *  s_q_jbase_x_) + ( 0.1956 *  c_q_jbase_x_)) *  c_q_jbase_y_) - ( 0.003 *  s_q_jbase_y_)) *  c_q_jbase_z_);
    (*this)(3,4) = (((( 0.0033 *  s_q_jbase_x_) + ( 0.1956 *  c_q_jbase_x_)) *  s_q_jbase_z_) + (((( 0.0033 *  c_q_jbase_x_) - ( 0.1956 *  s_q_jbase_x_)) *  s_q_jbase_y_) *  c_q_jbase_z_));
    (*this)(4,2) = (((( 0.1956 *  s_q_jbase_x_) - ( 0.0033 *  c_q_jbase_x_)) *  s_q_jbase_z_) + ((((( 0.0033 *  s_q_jbase_x_) + ( 0.1956 *  c_q_jbase_x_)) *  s_q_jbase_y_) + ( 0.003 *  c_q_jbase_y_)) *  c_q_jbase_z_));
    (*this)(4,3) = ((((( 0.0033 *  s_q_jbase_x_) + ( 0.1956 *  c_q_jbase_x_)) *  c_q_jbase_y_) - ( 0.003 *  s_q_jbase_y_)) *  s_q_jbase_z_);
    (*this)(4,4) = ((((( 0.0033 *  c_q_jbase_x_) - ( 0.1956 *  s_q_jbase_x_)) *  s_q_jbase_y_) *  s_q_jbase_z_) + (((- 0.0033 *  s_q_jbase_x_) - ( 0.1956 *  c_q_jbase_x_)) *  c_q_jbase_z_));
    (*this)(5,3) = ((((- 0.0033 *  s_q_jbase_x_) - ( 0.1956 *  c_q_jbase_x_)) *  s_q_jbase_y_) - ( 0.003 *  c_q_jbase_y_));
    (*this)(5,4) = ((( 0.0033 *  c_q_jbase_x_) - ( 0.1956 *  s_q_jbase_x_)) *  c_q_jbase_y_);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_ball1_COM::Type_fr_world_J_fr_dummy_ball1_COM()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 1.0;
    (*this)(4,0) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_ball1_COM& iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_ball1_COM::update(const JState& jState) {
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_base1_COM::Type_fr_world_J_fr_dummy_base1_COM()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(3,0) = 1.0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 1.0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}

template <typename TRAIT>
const typename iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_base1_COM& iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_base1_COM::update(const JState& jState) {
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_base2_COM::Type_fr_world_J_fr_dummy_base2_COM()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 1.0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 1.0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
}

template <typename TRAIT>
const typename iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_base2_COM& iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_world_J_fr_dummy_base2_COM::update(const JState& jState) {
    Scalar s_q_jbase_z_;
    Scalar c_q_jbase_z_;
    
    s_q_jbase_z_ = TRAIT::sin( jState(JBASE_Z));
    c_q_jbase_z_ = TRAIT::cos( jState(JBASE_Z));
    
    (*this)(0,3) = - s_q_jbase_z_;
    (*this)(1,3) =  c_q_jbase_z_;
    return *this;
}
