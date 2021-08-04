
// Initialization of static-const data
template <typename TRAIT>
const typename iit::Ballbot::dyn::tpl::ForwardDynamics<TRAIT>::ExtForces
    iit::Ballbot::dyn::tpl::ForwardDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::Ballbot::dyn::tpl::ForwardDynamics<TRAIT>::ForwardDynamics(iit::Ballbot::dyn::tpl::InertiaProperties<TRAIT>& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    dummy_ball1_v.setZero();
    dummy_ball1_c.setZero();
    ball_v.setZero();
    ball_c.setZero();
    dummy_base1_v.setZero();
    dummy_base1_c.setZero();
    dummy_base2_v.setZero();
    dummy_base2_c.setZero();
    base_v.setZero();
    base_c.setZero();

    vcross.setZero();
    Ia_p.setZero();
    Ia_r.setZero();

}

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    dummy_ball1_AI = inertiaProps->getTensor_dummy_ball1();
    dummy_ball1_p = - fext[DUMMY_BALL1];
    ball_AI = inertiaProps->getTensor_ball();
    ball_p = - fext[BALL];
    dummy_base1_AI = inertiaProps->getTensor_dummy_base1();
    dummy_base1_p = - fext[DUMMY_BASE1];
    dummy_base2_AI = inertiaProps->getTensor_dummy_base2();
    dummy_base2_p = - fext[DUMMY_BASE2];
    base_AI = inertiaProps->getTensor_base();
    base_p = - fext[BASE];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link dummy_ball1
    //  - The spatial velocity:
    dummy_ball1_v(iit::rbd::LZ) = qd(JBALL_X);
    
    //  - The bias force term:
    // The first joint is prismatic, no bias force term
    
    // + Link ball
    //  - The spatial velocity:
    ball_v = (motionTransforms-> fr_ball_X_fr_dummy_ball1) * dummy_ball1_v;
    ball_v(iit::rbd::LZ) += qd(JBALL_Y);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(ball_v, vcross);
    ball_c = vcross.col(iit::rbd::LZ) * qd(JBALL_Y);
    
    //  - The bias force term:
    ball_p += iit::rbd::vxIv(ball_v, ball_AI);
    
    // + Link dummy_base1
    //  - The spatial velocity:
    dummy_base1_v = (motionTransforms-> fr_dummy_base1_X_fr_ball) * ball_v;
    dummy_base1_v(iit::rbd::AZ) += qd(JBASE_Z);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(dummy_base1_v, vcross);
    dummy_base1_c = vcross.col(iit::rbd::AZ) * qd(JBASE_Z);
    
    //  - The bias force term:
    dummy_base1_p += iit::rbd::vxIv(dummy_base1_v, dummy_base1_AI);
    
    // + Link dummy_base2
    //  - The spatial velocity:
    dummy_base2_v = (motionTransforms-> fr_dummy_base2_X_fr_dummy_base1) * dummy_base1_v;
    dummy_base2_v(iit::rbd::AZ) += qd(JBASE_Y);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(dummy_base2_v, vcross);
    dummy_base2_c = vcross.col(iit::rbd::AZ) * qd(JBASE_Y);
    
    //  - The bias force term:
    dummy_base2_p += iit::rbd::vxIv(dummy_base2_v, dummy_base2_AI);
    
    // + Link base
    //  - The spatial velocity:
    base_v = (motionTransforms-> fr_base_X_fr_dummy_base2) * dummy_base2_v;
    base_v(iit::rbd::AZ) += qd(JBASE_X);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(base_v, vcross);
    base_c = vcross.col(iit::rbd::AZ) * qd(JBASE_X);
    
    //  - The bias force term:
    base_p += iit::rbd::vxIv(base_v, base_AI);
    
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66S IaB;
    Force pa;
    
    // + Link base
    base_u = tau(JBASE_X) - base_p(iit::rbd::AZ);
    base_U = base_AI.col(iit::rbd::AZ);
    base_D = base_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(base_AI, base_U, base_D, Ia_r);  // same as: Ia_r = base_AI - base_U/base_D * base_U.transpose();
    pa = base_p + Ia_r * base_c + base_U * base_u/base_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_base_X_fr_dummy_base2, IaB);
    dummy_base2_AI += IaB;
    dummy_base2_p += (motionTransforms-> fr_base_X_fr_dummy_base2).transpose() * pa;
    
    // + Link dummy_base2
    dummy_base2_u = tau(JBASE_Y) - dummy_base2_p(iit::rbd::AZ);
    dummy_base2_U = dummy_base2_AI.col(iit::rbd::AZ);
    dummy_base2_D = dummy_base2_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(dummy_base2_AI, dummy_base2_U, dummy_base2_D, Ia_r);  // same as: Ia_r = dummy_base2_AI - dummy_base2_U/dummy_base2_D * dummy_base2_U.transpose();
    pa = dummy_base2_p + Ia_r * dummy_base2_c + dummy_base2_U * dummy_base2_u/dummy_base2_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_dummy_base2_X_fr_dummy_base1, IaB);
    dummy_base1_AI += IaB;
    dummy_base1_p += (motionTransforms-> fr_dummy_base2_X_fr_dummy_base1).transpose() * pa;
    
    // + Link dummy_base1
    dummy_base1_u = tau(JBASE_Z) - dummy_base1_p(iit::rbd::AZ);
    dummy_base1_U = dummy_base1_AI.col(iit::rbd::AZ);
    dummy_base1_D = dummy_base1_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(dummy_base1_AI, dummy_base1_U, dummy_base1_D, Ia_r);  // same as: Ia_r = dummy_base1_AI - dummy_base1_U/dummy_base1_D * dummy_base1_U.transpose();
    pa = dummy_base1_p + Ia_r * dummy_base1_c + dummy_base1_U * dummy_base1_u/dummy_base1_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_dummy_base1_X_fr_ball, IaB);
    ball_AI += IaB;
    ball_p += (motionTransforms-> fr_dummy_base1_X_fr_ball).transpose() * pa;
    
    // + Link ball
    ball_u = tau(JBALL_Y) - ball_p(iit::rbd::LZ);
    ball_U = ball_AI.col(iit::rbd::LZ);
    ball_D = ball_U(iit::rbd::LZ);
    
    iit::rbd::compute_Ia_prismatic(ball_AI, ball_U, ball_D, Ia_p);  // same as: Ia_p = ball_AI - ball_U/ball_D * ball_U.transpose();
    pa = ball_p + Ia_p * ball_c + ball_U * ball_u/ball_D;
    ctransform_Ia_prismatic(Ia_p, motionTransforms-> fr_ball_X_fr_dummy_ball1, IaB);
    dummy_ball1_AI += IaB;
    dummy_ball1_p += (motionTransforms-> fr_ball_X_fr_dummy_ball1).transpose() * pa;
    
    // + Link dummy_ball1
    dummy_ball1_u = tau(JBALL_X) - dummy_ball1_p(iit::rbd::LZ);
    dummy_ball1_U = dummy_ball1_AI.col(iit::rbd::LZ);
    dummy_ball1_D = dummy_ball1_U(iit::rbd::LZ);
    
    
    
    // ---------------------- THIRD PASS ---------------------- //
    dummy_ball1_a = (motionTransforms-> fr_dummy_ball1_X_fr_world).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    qdd(JBALL_X) = (dummy_ball1_u - dummy_ball1_U.dot(dummy_ball1_a)) / dummy_ball1_D;
    dummy_ball1_a(iit::rbd::LZ) += qdd(JBALL_X);
    
    ball_a = (motionTransforms-> fr_ball_X_fr_dummy_ball1) * dummy_ball1_a + ball_c;
    qdd(JBALL_Y) = (ball_u - ball_U.dot(ball_a)) / ball_D;
    ball_a(iit::rbd::LZ) += qdd(JBALL_Y);
    
    dummy_base1_a = (motionTransforms-> fr_dummy_base1_X_fr_ball) * ball_a + dummy_base1_c;
    qdd(JBASE_Z) = (dummy_base1_u - dummy_base1_U.dot(dummy_base1_a)) / dummy_base1_D;
    dummy_base1_a(iit::rbd::AZ) += qdd(JBASE_Z);
    
    dummy_base2_a = (motionTransforms-> fr_dummy_base2_X_fr_dummy_base1) * dummy_base1_a + dummy_base2_c;
    qdd(JBASE_Y) = (dummy_base2_u - dummy_base2_U.dot(dummy_base2_a)) / dummy_base2_D;
    dummy_base2_a(iit::rbd::AZ) += qdd(JBASE_Y);
    
    base_a = (motionTransforms-> fr_base_X_fr_dummy_base2) * dummy_base2_a + base_c;
    qdd(JBASE_X) = (base_u - base_U.dot(base_a)) / base_D;
    base_a(iit::rbd::AZ) += qdd(JBASE_X);
    
    
}
