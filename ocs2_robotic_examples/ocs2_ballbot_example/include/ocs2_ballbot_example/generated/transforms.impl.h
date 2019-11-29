
// Constructors
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::MotionTransforms
    ()
     :
    fr_dummy_ball1_X_fr_world(),
    fr_world_X_fr_dummy_ball1(),
    fr_ball_X_fr_dummy_ball1(),
    fr_dummy_ball1_X_fr_ball(),
    fr_dummy_base1_X_fr_ball(),
    fr_ball_X_fr_dummy_base1(),
    fr_dummy_base2_X_fr_dummy_base1(),
    fr_dummy_base1_X_fr_dummy_base2(),
    fr_base_X_fr_dummy_base2(),
    fr_dummy_base2_X_fr_base()
{
    updateParameters();
}
template <typename TRAIT>
void iit::Ballbot::tpl::MotionTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::ForceTransforms
    ()
     :
    fr_dummy_ball1_X_fr_world(),
    fr_world_X_fr_dummy_ball1(),
    fr_ball_X_fr_dummy_ball1(),
    fr_dummy_ball1_X_fr_ball(),
    fr_dummy_base1_X_fr_ball(),
    fr_ball_X_fr_dummy_base1(),
    fr_dummy_base2_X_fr_dummy_base1(),
    fr_dummy_base1_X_fr_dummy_base2(),
    fr_base_X_fr_dummy_base2(),
    fr_dummy_base2_X_fr_base()
{
    updateParameters();
}
template <typename TRAIT>
void iit::Ballbot::tpl::ForceTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::HomogeneousTransforms
    ()
     :
    fr_dummy_ball1_X_fr_world(),
    fr_world_X_fr_dummy_ball1(),
    fr_ball_X_fr_dummy_ball1(),
    fr_dummy_ball1_X_fr_ball(),
    fr_dummy_base1_X_fr_ball(),
    fr_ball_X_fr_dummy_base1(),
    fr_dummy_base2_X_fr_dummy_base1(),
    fr_dummy_base1_X_fr_dummy_base2(),
    fr_base_X_fr_dummy_base2(),
    fr_dummy_base2_X_fr_base()
{
    updateParameters();
}
template <typename TRAIT>
void iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_world::Type_fr_dummy_ball1_X_fr_world()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = - 1;
    (*this)(4,0) = - 0.125;
    (*this)(4,1) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0.125;
    (*this)(5,2) = 0;
    (*this)(5,3) = 1;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_world& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_world::update(const JState& q) {
    
    
    (*this)(3,1) =  q(JBALL_X);
    (*this)(4,2) =  q(JBALL_X);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_dummy_ball1::Type_fr_world_X_fr_dummy_ball1()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = - 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = - 0.125;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1.0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0.125;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = - 1;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_dummy_ball1& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_world_X_fr_dummy_ball1::update(const JState& q) {
    
    
    (*this)(4,0) =  q(JBALL_X);
    (*this)(5,1) =  q(JBALL_X);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_ball1::Type_fr_ball_X_fr_dummy_ball1()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 1;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,3) = 1;
    (*this)(4,4) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_ball1& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_ball1::update(const JState& q) {
    
    
    (*this)(3,0) =  q(JBALL_Y);
    (*this)(4,2) = - q(JBALL_Y);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_ball::Type_fr_dummy_ball1_X_fr_ball()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 1;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 1;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1;
    (*this)(5,0) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 1;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_ball& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_ball::update(const JState& q) {
    
    
    (*this)(3,0) =  q(JBALL_Y);
    (*this)(5,1) = - q(JBALL_Y);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_ball::Type_fr_dummy_base1_X_fr_ball()
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
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
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
    (*this)(5,4) = - 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_ball& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_ball::update(const JState& q) {
    Scalar s_q_jbase_z_;
    Scalar c_q_jbase_z_;
    
    s_q_jbase_z_ = TRAIT::sin( q(JBASE_Z));
    c_q_jbase_z_ = TRAIT::cos( q(JBASE_Z));
    
    (*this)(0,0) =  c_q_jbase_z_;
    (*this)(0,2) =  s_q_jbase_z_;
    (*this)(1,0) = - s_q_jbase_z_;
    (*this)(1,2) =  c_q_jbase_z_;
    (*this)(3,3) =  c_q_jbase_z_;
    (*this)(3,5) =  s_q_jbase_z_;
    (*this)(4,3) = - s_q_jbase_z_;
    (*this)(4,5) =  c_q_jbase_z_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_base1::Type_fr_ball_X_fr_dummy_base1()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
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
    (*this)(4,5) = - 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_base1& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_base1::update(const JState& q) {
    Scalar s_q_jbase_z_;
    Scalar c_q_jbase_z_;
    
    s_q_jbase_z_ = TRAIT::sin( q(JBASE_Z));
    c_q_jbase_z_ = TRAIT::cos( q(JBASE_Z));
    
    (*this)(0,0) =  c_q_jbase_z_;
    (*this)(0,1) = - s_q_jbase_z_;
    (*this)(2,0) =  s_q_jbase_z_;
    (*this)(2,1) =  c_q_jbase_z_;
    (*this)(3,3) =  c_q_jbase_z_;
    (*this)(3,4) = - s_q_jbase_z_;
    (*this)(5,3) =  s_q_jbase_z_;
    (*this)(5,4) =  c_q_jbase_z_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_dummy_base1::Type_fr_dummy_base2_X_fr_dummy_base1()
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
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
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
    (*this)(5,4) = 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_dummy_base1& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_dummy_base1::update(const JState& q) {
    Scalar s_q_jbase_y_;
    Scalar c_q_jbase_y_;
    
    s_q_jbase_y_ = TRAIT::sin( q(JBASE_Y));
    c_q_jbase_y_ = TRAIT::cos( q(JBASE_Y));
    
    (*this)(0,0) =  c_q_jbase_y_;
    (*this)(0,2) = - s_q_jbase_y_;
    (*this)(1,0) = - s_q_jbase_y_;
    (*this)(1,2) = - c_q_jbase_y_;
    (*this)(3,3) =  c_q_jbase_y_;
    (*this)(3,5) = - s_q_jbase_y_;
    (*this)(4,3) = - s_q_jbase_y_;
    (*this)(4,5) = - c_q_jbase_y_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_dummy_base2::Type_fr_dummy_base1_X_fr_dummy_base2()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
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
    (*this)(4,5) = 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_dummy_base2& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_dummy_base2::update(const JState& q) {
    Scalar s_q_jbase_y_;
    Scalar c_q_jbase_y_;
    
    s_q_jbase_y_ = TRAIT::sin( q(JBASE_Y));
    c_q_jbase_y_ = TRAIT::cos( q(JBASE_Y));
    
    (*this)(0,0) =  c_q_jbase_y_;
    (*this)(0,1) = - s_q_jbase_y_;
    (*this)(2,0) = - s_q_jbase_y_;
    (*this)(2,1) = - c_q_jbase_y_;
    (*this)(3,3) =  c_q_jbase_y_;
    (*this)(3,4) = - s_q_jbase_y_;
    (*this)(5,3) = - s_q_jbase_y_;
    (*this)(5,4) = - c_q_jbase_y_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_dummy_base2::Type_fr_base_X_fr_dummy_base2()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
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
    (*this)(5,3) = 1;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_dummy_base2& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_base_X_fr_dummy_base2::update(const JState& q) {
    Scalar s_q_jbase_x_;
    Scalar c_q_jbase_x_;
    
    s_q_jbase_x_ = TRAIT::sin( q(JBASE_X));
    c_q_jbase_x_ = TRAIT::cos( q(JBASE_X));
    
    (*this)(0,1) =  c_q_jbase_x_;
    (*this)(0,2) =  s_q_jbase_x_;
    (*this)(1,1) = - s_q_jbase_x_;
    (*this)(1,2) =  c_q_jbase_x_;
    (*this)(3,4) =  c_q_jbase_x_;
    (*this)(3,5) =  s_q_jbase_x_;
    (*this)(4,4) = - s_q_jbase_x_;
    (*this)(4,5) =  c_q_jbase_x_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_base::Type_fr_dummy_base2_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
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
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1;
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
const typename iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_base& iit::Ballbot::tpl::MotionTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_base::update(const JState& q) {
    Scalar s_q_jbase_x_;
    Scalar c_q_jbase_x_;
    
    s_q_jbase_x_ = TRAIT::sin( q(JBASE_X));
    c_q_jbase_x_ = TRAIT::cos( q(JBASE_X));
    
    (*this)(1,0) =  c_q_jbase_x_;
    (*this)(1,1) = - s_q_jbase_x_;
    (*this)(2,0) =  s_q_jbase_x_;
    (*this)(2,1) =  c_q_jbase_x_;
    (*this)(4,3) =  c_q_jbase_x_;
    (*this)(4,4) = - s_q_jbase_x_;
    (*this)(5,3) =  s_q_jbase_x_;
    (*this)(5,4) =  c_q_jbase_x_;
    return *this;
}

template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_world::Type_fr_dummy_ball1_X_fr_world()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.125;
    (*this)(1,4) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0.125;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = - 1;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_world& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_world::update(const JState& q) {
    
    
    (*this)(0,4) =  q(JBALL_X);
    (*this)(1,5) =  q(JBALL_X);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_dummy_ball1::Type_fr_world_X_fr_dummy_ball1()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0;
    (*this)(0,4) = - 0.125;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0.125;
    (*this)(2,0) = - 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = - 1;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_dummy_ball1& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_world_X_fr_dummy_ball1::update(const JState& q) {
    
    
    (*this)(1,3) =  q(JBALL_X);
    (*this)(2,4) =  q(JBALL_X);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_ball1::Type_fr_ball_X_fr_dummy_ball1()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 1;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 1;
    (*this)(4,4) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_ball1& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_ball1::update(const JState& q) {
    
    
    (*this)(0,3) =  q(JBALL_Y);
    (*this)(1,5) = - q(JBALL_Y);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_ball::Type_fr_dummy_ball1_X_fr_ball()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 1;
    (*this)(0,2) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 1;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 1;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_ball& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_ball::update(const JState& q) {
    
    
    (*this)(0,3) =  q(JBALL_Y);
    (*this)(2,4) = - q(JBALL_Y);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_ball::Type_fr_dummy_base1_X_fr_ball()
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
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
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
    (*this)(5,4) = - 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_ball& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_ball::update(const JState& q) {
    Scalar s_q_jbase_z_;
    Scalar c_q_jbase_z_;
    
    s_q_jbase_z_ = TRAIT::sin( q(JBASE_Z));
    c_q_jbase_z_ = TRAIT::cos( q(JBASE_Z));
    
    (*this)(0,0) =  c_q_jbase_z_;
    (*this)(0,2) =  s_q_jbase_z_;
    (*this)(1,0) = - s_q_jbase_z_;
    (*this)(1,2) =  c_q_jbase_z_;
    (*this)(3,3) =  c_q_jbase_z_;
    (*this)(3,5) =  s_q_jbase_z_;
    (*this)(4,3) = - s_q_jbase_z_;
    (*this)(4,5) =  c_q_jbase_z_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_base1::Type_fr_ball_X_fr_dummy_base1()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
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
    (*this)(4,5) = - 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_base1& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_base1::update(const JState& q) {
    Scalar s_q_jbase_z_;
    Scalar c_q_jbase_z_;
    
    s_q_jbase_z_ = TRAIT::sin( q(JBASE_Z));
    c_q_jbase_z_ = TRAIT::cos( q(JBASE_Z));
    
    (*this)(0,0) =  c_q_jbase_z_;
    (*this)(0,1) = - s_q_jbase_z_;
    (*this)(2,0) =  s_q_jbase_z_;
    (*this)(2,1) =  c_q_jbase_z_;
    (*this)(3,3) =  c_q_jbase_z_;
    (*this)(3,4) = - s_q_jbase_z_;
    (*this)(5,3) =  s_q_jbase_z_;
    (*this)(5,4) =  c_q_jbase_z_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_dummy_base1::Type_fr_dummy_base2_X_fr_dummy_base1()
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
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
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
    (*this)(5,4) = 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_dummy_base1& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_dummy_base1::update(const JState& q) {
    Scalar s_q_jbase_y_;
    Scalar c_q_jbase_y_;
    
    s_q_jbase_y_ = TRAIT::sin( q(JBASE_Y));
    c_q_jbase_y_ = TRAIT::cos( q(JBASE_Y));
    
    (*this)(0,0) =  c_q_jbase_y_;
    (*this)(0,2) = - s_q_jbase_y_;
    (*this)(1,0) = - s_q_jbase_y_;
    (*this)(1,2) = - c_q_jbase_y_;
    (*this)(3,3) =  c_q_jbase_y_;
    (*this)(3,5) = - s_q_jbase_y_;
    (*this)(4,3) = - s_q_jbase_y_;
    (*this)(4,5) = - c_q_jbase_y_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_dummy_base2::Type_fr_dummy_base1_X_fr_dummy_base2()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
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
    (*this)(4,5) = 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_dummy_base2& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_dummy_base2::update(const JState& q) {
    Scalar s_q_jbase_y_;
    Scalar c_q_jbase_y_;
    
    s_q_jbase_y_ = TRAIT::sin( q(JBASE_Y));
    c_q_jbase_y_ = TRAIT::cos( q(JBASE_Y));
    
    (*this)(0,0) =  c_q_jbase_y_;
    (*this)(0,1) = - s_q_jbase_y_;
    (*this)(2,0) = - s_q_jbase_y_;
    (*this)(2,1) = - c_q_jbase_y_;
    (*this)(3,3) =  c_q_jbase_y_;
    (*this)(3,4) = - s_q_jbase_y_;
    (*this)(5,3) = - s_q_jbase_y_;
    (*this)(5,4) = - c_q_jbase_y_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_dummy_base2::Type_fr_base_X_fr_dummy_base2()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
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
    (*this)(5,3) = 1;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_dummy_base2& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_base_X_fr_dummy_base2::update(const JState& q) {
    Scalar s_q_jbase_x_;
    Scalar c_q_jbase_x_;
    
    s_q_jbase_x_ = TRAIT::sin( q(JBASE_X));
    c_q_jbase_x_ = TRAIT::cos( q(JBASE_X));
    
    (*this)(0,1) =  c_q_jbase_x_;
    (*this)(0,2) =  s_q_jbase_x_;
    (*this)(1,1) = - s_q_jbase_x_;
    (*this)(1,2) =  c_q_jbase_x_;
    (*this)(3,4) =  c_q_jbase_x_;
    (*this)(3,5) =  s_q_jbase_x_;
    (*this)(4,4) = - s_q_jbase_x_;
    (*this)(4,5) =  c_q_jbase_x_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_base::Type_fr_dummy_base2_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
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
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 1;
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
const typename iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_base& iit::Ballbot::tpl::ForceTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_base::update(const JState& q) {
    Scalar s_q_jbase_x_;
    Scalar c_q_jbase_x_;
    
    s_q_jbase_x_ = TRAIT::sin( q(JBASE_X));
    c_q_jbase_x_ = TRAIT::cos( q(JBASE_X));
    
    (*this)(1,0) =  c_q_jbase_x_;
    (*this)(1,1) = - s_q_jbase_x_;
    (*this)(2,0) =  s_q_jbase_x_;
    (*this)(2,1) =  c_q_jbase_x_;
    (*this)(4,3) =  c_q_jbase_x_;
    (*this)(4,4) = - s_q_jbase_x_;
    (*this)(5,3) =  s_q_jbase_x_;
    (*this)(5,4) =  c_q_jbase_x_;
    return *this;
}

template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_world::Type_fr_dummy_ball1_X_fr_world()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = - 1;
    (*this)(0,3) = 0.125;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_world& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_world::update(const JState& q) {
    
    
    (*this)(2,3) = - q(JBALL_X);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_dummy_ball1::Type_fr_world_X_fr_dummy_ball1()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = - 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.125;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_dummy_ball1& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_world_X_fr_dummy_ball1::update(const JState& q) {
    
    
    (*this)(0,3) =  q(JBALL_X);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_ball1::Type_fr_ball_X_fr_dummy_ball1()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(0,3) = 0;
    (*this)(1,0) = 1;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_ball1& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_ball1::update(const JState& q) {
    
    
    (*this)(2,3) = - q(JBALL_Y);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_ball::Type_fr_dummy_ball1_X_fr_ball()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 1;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_ball& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_ball1_X_fr_ball::update(const JState& q) {
    
    
    (*this)(1,3) =  q(JBALL_Y);
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_ball::Type_fr_dummy_base1_X_fr_ball()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_ball& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_ball::update(const JState& q) {
    Scalar s_q_jbase_z_;
    Scalar c_q_jbase_z_;
    
    s_q_jbase_z_ = TRAIT::sin( q(JBASE_Z));
    c_q_jbase_z_ = TRAIT::cos( q(JBASE_Z));
    
    (*this)(0,0) =  c_q_jbase_z_;
    (*this)(0,2) =  s_q_jbase_z_;
    (*this)(1,0) = - s_q_jbase_z_;
    (*this)(1,2) =  c_q_jbase_z_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_base1::Type_fr_ball_X_fr_dummy_base1()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_base1& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ball_X_fr_dummy_base1::update(const JState& q) {
    Scalar s_q_jbase_z_;
    Scalar c_q_jbase_z_;
    
    s_q_jbase_z_ = TRAIT::sin( q(JBASE_Z));
    c_q_jbase_z_ = TRAIT::cos( q(JBASE_Z));
    
    (*this)(0,0) =  c_q_jbase_z_;
    (*this)(0,1) = - s_q_jbase_z_;
    (*this)(2,0) =  s_q_jbase_z_;
    (*this)(2,1) =  c_q_jbase_z_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_dummy_base1::Type_fr_dummy_base2_X_fr_dummy_base1()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_dummy_base1& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_dummy_base1::update(const JState& q) {
    Scalar s_q_jbase_y_;
    Scalar c_q_jbase_y_;
    
    s_q_jbase_y_ = TRAIT::sin( q(JBASE_Y));
    c_q_jbase_y_ = TRAIT::cos( q(JBASE_Y));
    
    (*this)(0,0) =  c_q_jbase_y_;
    (*this)(0,2) = - s_q_jbase_y_;
    (*this)(1,0) = - s_q_jbase_y_;
    (*this)(1,2) = - c_q_jbase_y_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_dummy_base2::Type_fr_dummy_base1_X_fr_dummy_base2()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_dummy_base2& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_base1_X_fr_dummy_base2::update(const JState& q) {
    Scalar s_q_jbase_y_;
    Scalar c_q_jbase_y_;
    
    s_q_jbase_y_ = TRAIT::sin( q(JBASE_Y));
    c_q_jbase_y_ = TRAIT::cos( q(JBASE_Y));
    
    (*this)(0,0) =  c_q_jbase_y_;
    (*this)(0,1) = - s_q_jbase_y_;
    (*this)(2,0) = - s_q_jbase_y_;
    (*this)(2,1) = - c_q_jbase_y_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_dummy_base2::Type_fr_base_X_fr_dummy_base2()
{
    (*this)(0,0) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 1;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_dummy_base2& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_X_fr_dummy_base2::update(const JState& q) {
    Scalar s_q_jbase_x_;
    Scalar c_q_jbase_x_;
    
    s_q_jbase_x_ = TRAIT::sin( q(JBASE_X));
    c_q_jbase_x_ = TRAIT::cos( q(JBASE_X));
    
    (*this)(0,1) =  c_q_jbase_x_;
    (*this)(0,2) =  s_q_jbase_x_;
    (*this)(1,1) = - s_q_jbase_x_;
    (*this)(1,2) =  c_q_jbase_x_;
    return *this;
}
template <typename TRAIT>
iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_base::Type_fr_dummy_base2_X_fr_base()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 1;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_base& iit::Ballbot::tpl::HomogeneousTransforms<TRAIT>::Type_fr_dummy_base2_X_fr_base::update(const JState& q) {
    Scalar s_q_jbase_x_;
    Scalar c_q_jbase_x_;
    
    s_q_jbase_x_ = TRAIT::sin( q(JBASE_X));
    c_q_jbase_x_ = TRAIT::cos( q(JBASE_X));
    
    (*this)(1,0) =  c_q_jbase_x_;
    (*this)(1,1) = - s_q_jbase_x_;
    (*this)(2,0) =  s_q_jbase_x_;
    (*this)(2,1) =  c_q_jbase_x_;
    return *this;
}

