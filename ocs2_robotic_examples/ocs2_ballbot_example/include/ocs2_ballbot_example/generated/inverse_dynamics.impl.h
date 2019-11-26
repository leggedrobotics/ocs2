// Initialization of static-const data
template <typename TRAIT>
const typename iit::Ballbot::dyn::tpl::InverseDynamics<TRAIT>::ExtForces
iit::Ballbot::dyn::tpl::InverseDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::Ballbot::dyn::tpl::InverseDynamics<TRAIT>::InverseDynamics(IProperties& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    dummy_ball1_I(inertiaProps->getTensor_dummy_ball1() ),
    ball_I(inertiaProps->getTensor_ball() ),
    dummy_base1_I(inertiaProps->getTensor_dummy_base1() ),
    dummy_base2_I(inertiaProps->getTensor_dummy_base2() ),
    base_I(inertiaProps->getTensor_base() )
    {
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot Ballbot, InverseDynamics<TRAIT>::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    dummy_ball1_v.setZero();
    ball_v.setZero();
    dummy_base1_v.setZero();
    dummy_base2_v.setZero();
    base_v.setZero();

    vcross.setZero();
}

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::InverseDynamics<TRAIT>::id(
    JointState& jForces,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    firstPass(qd, qdd, fext);
    secondPass(jForces);
}

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::InverseDynamics<TRAIT>::G_terms(JointState& jForces)
{
    // Link 'dummy_ball1'
    dummy_ball1_a = (xm->fr_dummy_ball1_X_fr_world).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    dummy_ball1_f = dummy_ball1_I * dummy_ball1_a;
    // Link 'ball'
    ball_a = (xm->fr_ball_X_fr_dummy_ball1) * dummy_ball1_a;
    ball_f = ball_I * ball_a;
    // Link 'dummy_base1'
    dummy_base1_a = (xm->fr_dummy_base1_X_fr_ball) * ball_a;
    dummy_base1_f = dummy_base1_I * dummy_base1_a;
    // Link 'dummy_base2'
    dummy_base2_a = (xm->fr_dummy_base2_X_fr_dummy_base1) * dummy_base1_a;
    dummy_base2_f = dummy_base2_I * dummy_base2_a;
    // Link 'base'
    base_a = (xm->fr_base_X_fr_dummy_base2) * dummy_base2_a;
    base_f = base_I * base_a;

    secondPass(jForces);
}

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::InverseDynamics<TRAIT>::C_terms(JointState& jForces, const JointState& qd)
{
    // Link 'dummy_ball1'
    dummy_ball1_v(iit::rbd::LZ) = qd(JBALL_X);   // dummy_ball1_v = vJ, for the first link of a fixed base robot
    
    // The first joint is prismatic, no centripetal terms.
    dummy_ball1_f.setZero();
    
    // Link 'ball'
    ball_v = ((xm->fr_ball_X_fr_dummy_ball1) * dummy_ball1_v);
    ball_v(iit::rbd::LZ) += qd(JBALL_Y);
    
    iit::rbd::motionCrossProductMx<Scalar>(ball_v, vcross);
    
    ball_a = (vcross.col(iit::rbd::LZ) * qd(JBALL_Y));
    
    ball_f = ball_I * ball_a + iit::rbd::vxIv(ball_v, ball_I);
    
    // Link 'dummy_base1'
    dummy_base1_v = ((xm->fr_dummy_base1_X_fr_ball) * ball_v);
    dummy_base1_v(iit::rbd::AZ) += qd(JBASE_Z);
    
    iit::rbd::motionCrossProductMx<Scalar>(dummy_base1_v, vcross);
    
    dummy_base1_a = (xm->fr_dummy_base1_X_fr_ball) * ball_a + vcross.col(iit::rbd::AZ) * qd(JBASE_Z);
    
    dummy_base1_f = dummy_base1_I * dummy_base1_a + iit::rbd::vxIv(dummy_base1_v, dummy_base1_I);
    
    // Link 'dummy_base2'
    dummy_base2_v = ((xm->fr_dummy_base2_X_fr_dummy_base1) * dummy_base1_v);
    dummy_base2_v(iit::rbd::AZ) += qd(JBASE_Y);
    
    iit::rbd::motionCrossProductMx<Scalar>(dummy_base2_v, vcross);
    
    dummy_base2_a = (xm->fr_dummy_base2_X_fr_dummy_base1) * dummy_base1_a + vcross.col(iit::rbd::AZ) * qd(JBASE_Y);
    
    dummy_base2_f = dummy_base2_I * dummy_base2_a + iit::rbd::vxIv(dummy_base2_v, dummy_base2_I);
    
    // Link 'base'
    base_v = ((xm->fr_base_X_fr_dummy_base2) * dummy_base2_v);
    base_v(iit::rbd::AZ) += qd(JBASE_X);
    
    iit::rbd::motionCrossProductMx<Scalar>(base_v, vcross);
    
    base_a = (xm->fr_base_X_fr_dummy_base2) * dummy_base2_a + vcross.col(iit::rbd::AZ) * qd(JBASE_X);
    
    base_f = base_I * base_a + iit::rbd::vxIv(base_v, base_I);
    

    secondPass(jForces);
}

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::InverseDynamics<TRAIT>::firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    // First pass, link 'dummy_ball1'
    dummy_ball1_a = (xm->fr_dummy_ball1_X_fr_world).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    dummy_ball1_a(iit::rbd::LZ) += qdd(JBALL_X);
    dummy_ball1_v(iit::rbd::LZ) = qd(JBALL_X);   // dummy_ball1_v = vJ, for the first link of a fixed base robot
    
    // The first joint is prismatic, no centripetal terms.
    dummy_ball1_f = dummy_ball1_I * dummy_ball1_a - fext[DUMMY_BALL1];
    
    // First pass, link 'ball'
    ball_v = ((xm->fr_ball_X_fr_dummy_ball1) * dummy_ball1_v);
    ball_v(iit::rbd::LZ) += qd(JBALL_Y);
    
    iit::rbd::motionCrossProductMx<Scalar>(ball_v, vcross);
    
    ball_a = (xm->fr_ball_X_fr_dummy_ball1) * dummy_ball1_a + vcross.col(iit::rbd::LZ) * qd(JBALL_Y);
    ball_a(iit::rbd::LZ) += qdd(JBALL_Y);
    
    ball_f = ball_I * ball_a + iit::rbd::vxIv(ball_v, ball_I) - fext[BALL];
    
    // First pass, link 'dummy_base1'
    dummy_base1_v = ((xm->fr_dummy_base1_X_fr_ball) * ball_v);
    dummy_base1_v(iit::rbd::AZ) += qd(JBASE_Z);
    
    iit::rbd::motionCrossProductMx<Scalar>(dummy_base1_v, vcross);
    
    dummy_base1_a = (xm->fr_dummy_base1_X_fr_ball) * ball_a + vcross.col(iit::rbd::AZ) * qd(JBASE_Z);
    dummy_base1_a(iit::rbd::AZ) += qdd(JBASE_Z);
    
    dummy_base1_f = dummy_base1_I * dummy_base1_a + iit::rbd::vxIv(dummy_base1_v, dummy_base1_I) - fext[DUMMY_BASE1];
    
    // First pass, link 'dummy_base2'
    dummy_base2_v = ((xm->fr_dummy_base2_X_fr_dummy_base1) * dummy_base1_v);
    dummy_base2_v(iit::rbd::AZ) += qd(JBASE_Y);
    
    iit::rbd::motionCrossProductMx<Scalar>(dummy_base2_v, vcross);
    
    dummy_base2_a = (xm->fr_dummy_base2_X_fr_dummy_base1) * dummy_base1_a + vcross.col(iit::rbd::AZ) * qd(JBASE_Y);
    dummy_base2_a(iit::rbd::AZ) += qdd(JBASE_Y);
    
    dummy_base2_f = dummy_base2_I * dummy_base2_a + iit::rbd::vxIv(dummy_base2_v, dummy_base2_I) - fext[DUMMY_BASE2];
    
    // First pass, link 'base'
    base_v = ((xm->fr_base_X_fr_dummy_base2) * dummy_base2_v);
    base_v(iit::rbd::AZ) += qd(JBASE_X);
    
    iit::rbd::motionCrossProductMx<Scalar>(base_v, vcross);
    
    base_a = (xm->fr_base_X_fr_dummy_base2) * dummy_base2_a + vcross.col(iit::rbd::AZ) * qd(JBASE_X);
    base_a(iit::rbd::AZ) += qdd(JBASE_X);
    
    base_f = base_I * base_a + iit::rbd::vxIv(base_v, base_I) - fext[BASE];
    
}

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::InverseDynamics<TRAIT>::secondPass(JointState& jForces)
{
    // Link 'base'
    jForces(JBASE_X) = base_f(iit::rbd::AZ);
    dummy_base2_f += xm->fr_base_X_fr_dummy_base2.transpose() * base_f;
    // Link 'dummy_base2'
    jForces(JBASE_Y) = dummy_base2_f(iit::rbd::AZ);
    dummy_base1_f += xm->fr_dummy_base2_X_fr_dummy_base1.transpose() * dummy_base2_f;
    // Link 'dummy_base1'
    jForces(JBASE_Z) = dummy_base1_f(iit::rbd::AZ);
    ball_f += xm->fr_dummy_base1_X_fr_ball.transpose() * dummy_base1_f;
    // Link 'ball'
    jForces(JBALL_Y) = ball_f(iit::rbd::LZ);
    dummy_ball1_f += xm->fr_ball_X_fr_dummy_ball1.transpose() * ball_f;
    // Link 'dummy_ball1'
    jForces(JBALL_X) = dummy_ball1_f(iit::rbd::LZ);
}
