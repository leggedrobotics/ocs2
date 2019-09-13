
template <typename TRAIT>
iit::Ballbot::tpl::Jacobians<TRAIT>::Jacobians
    ()
     : 
    fr_base0_J_fr_control()
{
    updateParameters();
}

template <typename TRAIT>
void iit::Ballbot::tpl::Jacobians<TRAIT>::updateParameters() {
}


template <typename TRAIT>
iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_base0_J_fr_control::Type_fr_base0_J_fr_control()
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
const typename iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_base0_J_fr_control& iit::Ballbot::tpl::Jacobians<TRAIT>::Type_fr_base0_J_fr_control::update(const JState& jState) {
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
    (*this)(3,2) = ((( 0.275 *  s_q_jbase_x_) *  c_q_jbase_z_) - ((( 0.275 *  c_q_jbase_x_) *  s_q_jbase_y_) *  s_q_jbase_z_));
    (*this)(3,3) = ((( 0.275 *  c_q_jbase_x_) *  c_q_jbase_y_) *  c_q_jbase_z_);
    (*this)(3,4) = ((( 0.275 *  c_q_jbase_x_) *  s_q_jbase_z_) - ((( 0.275 *  s_q_jbase_x_) *  s_q_jbase_y_) *  c_q_jbase_z_));
    (*this)(4,2) = ((( 0.275 *  s_q_jbase_x_) *  s_q_jbase_z_) + ((( 0.275 *  c_q_jbase_x_) *  s_q_jbase_y_) *  c_q_jbase_z_));
    (*this)(4,3) = ((( 0.275 *  c_q_jbase_x_) *  c_q_jbase_y_) *  s_q_jbase_z_);
    (*this)(4,4) = ((((- 0.275 *  s_q_jbase_x_) *  s_q_jbase_y_) *  s_q_jbase_z_) - (( 0.275 *  c_q_jbase_x_) *  c_q_jbase_z_));
    (*this)(5,3) = ((- 0.275 *  c_q_jbase_x_) *  s_q_jbase_y_);
    (*this)(5,4) = ((- 0.275 *  s_q_jbase_x_) *  c_q_jbase_y_);
    return *this;
}
