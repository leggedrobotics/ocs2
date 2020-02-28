
// Initialization of static-const data
template <typename TRAIT>
const typename iit::ANYmal::dyn::tpl::ForwardDynamics<TRAIT>::ExtForces
    iit::ANYmal::dyn::tpl::ForwardDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::ANYmal::dyn::tpl::ForwardDynamics<TRAIT>::ForwardDynamics(iit::ANYmal::dyn::tpl::InertiaProperties<TRAIT>& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    LF_HIP_v.setZero();
    LF_HIP_c.setZero();
    LF_THIGH_v.setZero();
    LF_THIGH_c.setZero();
    LF_shank_fixed_v.setZero();
    LF_shank_fixed_c.setZero();
    LF_WHEEL_L_v.setZero();
    LF_WHEEL_L_c.setZero();
    RF_HIP_v.setZero();
    RF_HIP_c.setZero();
    RF_THIGH_v.setZero();
    RF_THIGH_c.setZero();
    RF_shank_fixed_v.setZero();
    RF_shank_fixed_c.setZero();
    RF_WHEEL_L_v.setZero();
    RF_WHEEL_L_c.setZero();
    LH_HIP_v.setZero();
    LH_HIP_c.setZero();
    LH_THIGH_v.setZero();
    LH_THIGH_c.setZero();
    LH_shank_fixed_v.setZero();
    LH_shank_fixed_c.setZero();
    LH_WHEEL_L_v.setZero();
    LH_WHEEL_L_c.setZero();
    RH_HIP_v.setZero();
    RH_HIP_c.setZero();
    RH_THIGH_v.setZero();
    RH_THIGH_c.setZero();
    RH_shank_fixed_v.setZero();
    RH_shank_fixed_c.setZero();
    RH_WHEEL_L_v.setZero();
    RH_WHEEL_L_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

template <typename TRAIT>
void iit::ANYmal::dyn::tpl::ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    Acceleration& base_a,
    const Velocity& base_v,
    const Acceleration& g,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    base_AI = inertiaProps->getTensor_base();
    base_p = - fext[BASE];
    LF_HIP_AI = inertiaProps->getTensor_LF_HIP();
    LF_HIP_p = - fext[LF_HIP];
    LF_THIGH_AI = inertiaProps->getTensor_LF_THIGH();
    LF_THIGH_p = - fext[LF_THIGH];
    LF_shank_fixed_AI = inertiaProps->getTensor_LF_shank_fixed();
    LF_shank_fixed_p = - fext[LF_SHANK_FIXED];
    LF_WHEEL_L_AI = inertiaProps->getTensor_LF_WHEEL_L();
    LF_WHEEL_L_p = - fext[LF_WHEEL_L];
    RF_HIP_AI = inertiaProps->getTensor_RF_HIP();
    RF_HIP_p = - fext[RF_HIP];
    RF_THIGH_AI = inertiaProps->getTensor_RF_THIGH();
    RF_THIGH_p = - fext[RF_THIGH];
    RF_shank_fixed_AI = inertiaProps->getTensor_RF_shank_fixed();
    RF_shank_fixed_p = - fext[RF_SHANK_FIXED];
    RF_WHEEL_L_AI = inertiaProps->getTensor_RF_WHEEL_L();
    RF_WHEEL_L_p = - fext[RF_WHEEL_L];
    LH_HIP_AI = inertiaProps->getTensor_LH_HIP();
    LH_HIP_p = - fext[LH_HIP];
    LH_THIGH_AI = inertiaProps->getTensor_LH_THIGH();
    LH_THIGH_p = - fext[LH_THIGH];
    LH_shank_fixed_AI = inertiaProps->getTensor_LH_shank_fixed();
    LH_shank_fixed_p = - fext[LH_SHANK_FIXED];
    LH_WHEEL_L_AI = inertiaProps->getTensor_LH_WHEEL_L();
    LH_WHEEL_L_p = - fext[LH_WHEEL_L];
    RH_HIP_AI = inertiaProps->getTensor_RH_HIP();
    RH_HIP_p = - fext[RH_HIP];
    RH_THIGH_AI = inertiaProps->getTensor_RH_THIGH();
    RH_THIGH_p = - fext[RH_THIGH];
    RH_shank_fixed_AI = inertiaProps->getTensor_RH_shank_fixed();
    RH_shank_fixed_p = - fext[RH_SHANK_FIXED];
    RH_WHEEL_L_AI = inertiaProps->getTensor_RH_WHEEL_L();
    RH_WHEEL_L_p = - fext[RH_WHEEL_L];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link LF_HIP
    //  - The spatial velocity:
    LF_HIP_v = (motionTransforms-> fr_LF_HIP_X_fr_base) * base_v;
    LF_HIP_v(iit::rbd::AZ) += qd(LF_HAA);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LF_HIP_v, vcross);
    LF_HIP_c = vcross.col(iit::rbd::AZ) * qd(LF_HAA);
    
    //  - The bias force term:
    LF_HIP_p += iit::rbd::vxIv(LF_HIP_v, LF_HIP_AI);
    
    // + Link LF_THIGH
    //  - The spatial velocity:
    LF_THIGH_v = (motionTransforms-> fr_LF_THIGH_X_fr_LF_HIP) * LF_HIP_v;
    LF_THIGH_v(iit::rbd::AZ) += qd(LF_HFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LF_THIGH_v, vcross);
    LF_THIGH_c = vcross.col(iit::rbd::AZ) * qd(LF_HFE);
    
    //  - The bias force term:
    LF_THIGH_p += iit::rbd::vxIv(LF_THIGH_v, LF_THIGH_AI);
    
    // + Link LF_shank_fixed
    //  - The spatial velocity:
    LF_shank_fixed_v = (motionTransforms-> fr_LF_shank_fixed_X_fr_LF_THIGH) * LF_THIGH_v;
    LF_shank_fixed_v(iit::rbd::AZ) += qd(LF_KFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LF_shank_fixed_v, vcross);
    LF_shank_fixed_c = vcross.col(iit::rbd::AZ) * qd(LF_KFE);
    
    //  - The bias force term:
    LF_shank_fixed_p += iit::rbd::vxIv(LF_shank_fixed_v, LF_shank_fixed_AI);
    
    // + Link LF_WHEEL_L
    //  - The spatial velocity:
    LF_WHEEL_L_v = (motionTransforms-> fr_LF_WHEEL_L_X_fr_LF_shank_fixed) * LF_shank_fixed_v;
    LF_WHEEL_L_v(iit::rbd::AZ) += qd(LF_WHEEL);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LF_WHEEL_L_v, vcross);
    LF_WHEEL_L_c = vcross.col(iit::rbd::AZ) * qd(LF_WHEEL);
    
    //  - The bias force term:
    LF_WHEEL_L_p += iit::rbd::vxIv(LF_WHEEL_L_v, LF_WHEEL_L_AI);
    
    // + Link RF_HIP
    //  - The spatial velocity:
    RF_HIP_v = (motionTransforms-> fr_RF_HIP_X_fr_base) * base_v;
    RF_HIP_v(iit::rbd::AZ) += qd(RF_HAA);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RF_HIP_v, vcross);
    RF_HIP_c = vcross.col(iit::rbd::AZ) * qd(RF_HAA);
    
    //  - The bias force term:
    RF_HIP_p += iit::rbd::vxIv(RF_HIP_v, RF_HIP_AI);
    
    // + Link RF_THIGH
    //  - The spatial velocity:
    RF_THIGH_v = (motionTransforms-> fr_RF_THIGH_X_fr_RF_HIP) * RF_HIP_v;
    RF_THIGH_v(iit::rbd::AZ) += qd(RF_HFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RF_THIGH_v, vcross);
    RF_THIGH_c = vcross.col(iit::rbd::AZ) * qd(RF_HFE);
    
    //  - The bias force term:
    RF_THIGH_p += iit::rbd::vxIv(RF_THIGH_v, RF_THIGH_AI);
    
    // + Link RF_shank_fixed
    //  - The spatial velocity:
    RF_shank_fixed_v = (motionTransforms-> fr_RF_shank_fixed_X_fr_RF_THIGH) * RF_THIGH_v;
    RF_shank_fixed_v(iit::rbd::AZ) += qd(RF_KFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RF_shank_fixed_v, vcross);
    RF_shank_fixed_c = vcross.col(iit::rbd::AZ) * qd(RF_KFE);
    
    //  - The bias force term:
    RF_shank_fixed_p += iit::rbd::vxIv(RF_shank_fixed_v, RF_shank_fixed_AI);
    
    // + Link RF_WHEEL_L
    //  - The spatial velocity:
    RF_WHEEL_L_v = (motionTransforms-> fr_RF_WHEEL_L_X_fr_RF_shank_fixed) * RF_shank_fixed_v;
    RF_WHEEL_L_v(iit::rbd::AZ) += qd(RF_WHEEL);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RF_WHEEL_L_v, vcross);
    RF_WHEEL_L_c = vcross.col(iit::rbd::AZ) * qd(RF_WHEEL);
    
    //  - The bias force term:
    RF_WHEEL_L_p += iit::rbd::vxIv(RF_WHEEL_L_v, RF_WHEEL_L_AI);
    
    // + Link LH_HIP
    //  - The spatial velocity:
    LH_HIP_v = (motionTransforms-> fr_LH_HIP_X_fr_base) * base_v;
    LH_HIP_v(iit::rbd::AZ) += qd(LH_HAA);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LH_HIP_v, vcross);
    LH_HIP_c = vcross.col(iit::rbd::AZ) * qd(LH_HAA);
    
    //  - The bias force term:
    LH_HIP_p += iit::rbd::vxIv(LH_HIP_v, LH_HIP_AI);
    
    // + Link LH_THIGH
    //  - The spatial velocity:
    LH_THIGH_v = (motionTransforms-> fr_LH_THIGH_X_fr_LH_HIP) * LH_HIP_v;
    LH_THIGH_v(iit::rbd::AZ) += qd(LH_HFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LH_THIGH_v, vcross);
    LH_THIGH_c = vcross.col(iit::rbd::AZ) * qd(LH_HFE);
    
    //  - The bias force term:
    LH_THIGH_p += iit::rbd::vxIv(LH_THIGH_v, LH_THIGH_AI);
    
    // + Link LH_shank_fixed
    //  - The spatial velocity:
    LH_shank_fixed_v = (motionTransforms-> fr_LH_shank_fixed_X_fr_LH_THIGH) * LH_THIGH_v;
    LH_shank_fixed_v(iit::rbd::AZ) += qd(LH_KFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LH_shank_fixed_v, vcross);
    LH_shank_fixed_c = vcross.col(iit::rbd::AZ) * qd(LH_KFE);
    
    //  - The bias force term:
    LH_shank_fixed_p += iit::rbd::vxIv(LH_shank_fixed_v, LH_shank_fixed_AI);
    
    // + Link LH_WHEEL_L
    //  - The spatial velocity:
    LH_WHEEL_L_v = (motionTransforms-> fr_LH_WHEEL_L_X_fr_LH_shank_fixed) * LH_shank_fixed_v;
    LH_WHEEL_L_v(iit::rbd::AZ) += qd(LH_WHEEL);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LH_WHEEL_L_v, vcross);
    LH_WHEEL_L_c = vcross.col(iit::rbd::AZ) * qd(LH_WHEEL);
    
    //  - The bias force term:
    LH_WHEEL_L_p += iit::rbd::vxIv(LH_WHEEL_L_v, LH_WHEEL_L_AI);
    
    // + Link RH_HIP
    //  - The spatial velocity:
    RH_HIP_v = (motionTransforms-> fr_RH_HIP_X_fr_base) * base_v;
    RH_HIP_v(iit::rbd::AZ) += qd(RH_HAA);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RH_HIP_v, vcross);
    RH_HIP_c = vcross.col(iit::rbd::AZ) * qd(RH_HAA);
    
    //  - The bias force term:
    RH_HIP_p += iit::rbd::vxIv(RH_HIP_v, RH_HIP_AI);
    
    // + Link RH_THIGH
    //  - The spatial velocity:
    RH_THIGH_v = (motionTransforms-> fr_RH_THIGH_X_fr_RH_HIP) * RH_HIP_v;
    RH_THIGH_v(iit::rbd::AZ) += qd(RH_HFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RH_THIGH_v, vcross);
    RH_THIGH_c = vcross.col(iit::rbd::AZ) * qd(RH_HFE);
    
    //  - The bias force term:
    RH_THIGH_p += iit::rbd::vxIv(RH_THIGH_v, RH_THIGH_AI);
    
    // + Link RH_shank_fixed
    //  - The spatial velocity:
    RH_shank_fixed_v = (motionTransforms-> fr_RH_shank_fixed_X_fr_RH_THIGH) * RH_THIGH_v;
    RH_shank_fixed_v(iit::rbd::AZ) += qd(RH_KFE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RH_shank_fixed_v, vcross);
    RH_shank_fixed_c = vcross.col(iit::rbd::AZ) * qd(RH_KFE);
    
    //  - The bias force term:
    RH_shank_fixed_p += iit::rbd::vxIv(RH_shank_fixed_v, RH_shank_fixed_AI);
    
    // + Link RH_WHEEL_L
    //  - The spatial velocity:
    RH_WHEEL_L_v = (motionTransforms-> fr_RH_WHEEL_L_X_fr_RH_shank_fixed) * RH_shank_fixed_v;
    RH_WHEEL_L_v(iit::rbd::AZ) += qd(RH_WHEEL);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RH_WHEEL_L_v, vcross);
    RH_WHEEL_L_c = vcross.col(iit::rbd::AZ) * qd(RH_WHEEL);
    
    //  - The bias force term:
    RH_WHEEL_L_p += iit::rbd::vxIv(RH_WHEEL_L_v, RH_WHEEL_L_AI);
    
    // + The floating base body
    base_p += iit::rbd::vxIv(base_v, base_AI);
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66S IaB;
    Force pa;
    
    // + Link RH_WHEEL_L
    RH_WHEEL_L_u = tau(RH_WHEEL) - RH_WHEEL_L_p(iit::rbd::AZ);
    RH_WHEEL_L_U = RH_WHEEL_L_AI.col(iit::rbd::AZ);
    RH_WHEEL_L_D = RH_WHEEL_L_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RH_WHEEL_L_AI, RH_WHEEL_L_U, RH_WHEEL_L_D, Ia_r);  // same as: Ia_r = RH_WHEEL_L_AI - RH_WHEEL_L_U/RH_WHEEL_L_D * RH_WHEEL_L_U.transpose();
    pa = RH_WHEEL_L_p + Ia_r * RH_WHEEL_L_c + RH_WHEEL_L_U * RH_WHEEL_L_u/RH_WHEEL_L_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RH_WHEEL_L_X_fr_RH_shank_fixed, IaB);
    RH_shank_fixed_AI += IaB;
    RH_shank_fixed_p += (motionTransforms-> fr_RH_WHEEL_L_X_fr_RH_shank_fixed).transpose() * pa;
    
    // + Link RH_shank_fixed
    RH_shank_fixed_u = tau(RH_KFE) - RH_shank_fixed_p(iit::rbd::AZ);
    RH_shank_fixed_U = RH_shank_fixed_AI.col(iit::rbd::AZ);
    RH_shank_fixed_D = RH_shank_fixed_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RH_shank_fixed_AI, RH_shank_fixed_U, RH_shank_fixed_D, Ia_r);  // same as: Ia_r = RH_shank_fixed_AI - RH_shank_fixed_U/RH_shank_fixed_D * RH_shank_fixed_U.transpose();
    pa = RH_shank_fixed_p + Ia_r * RH_shank_fixed_c + RH_shank_fixed_U * RH_shank_fixed_u/RH_shank_fixed_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RH_shank_fixed_X_fr_RH_THIGH, IaB);
    RH_THIGH_AI += IaB;
    RH_THIGH_p += (motionTransforms-> fr_RH_shank_fixed_X_fr_RH_THIGH).transpose() * pa;
    
    // + Link RH_THIGH
    RH_THIGH_u = tau(RH_HFE) - RH_THIGH_p(iit::rbd::AZ);
    RH_THIGH_U = RH_THIGH_AI.col(iit::rbd::AZ);
    RH_THIGH_D = RH_THIGH_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RH_THIGH_AI, RH_THIGH_U, RH_THIGH_D, Ia_r);  // same as: Ia_r = RH_THIGH_AI - RH_THIGH_U/RH_THIGH_D * RH_THIGH_U.transpose();
    pa = RH_THIGH_p + Ia_r * RH_THIGH_c + RH_THIGH_U * RH_THIGH_u/RH_THIGH_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RH_THIGH_X_fr_RH_HIP, IaB);
    RH_HIP_AI += IaB;
    RH_HIP_p += (motionTransforms-> fr_RH_THIGH_X_fr_RH_HIP).transpose() * pa;
    
    // + Link RH_HIP
    RH_HIP_u = tau(RH_HAA) - RH_HIP_p(iit::rbd::AZ);
    RH_HIP_U = RH_HIP_AI.col(iit::rbd::AZ);
    RH_HIP_D = RH_HIP_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RH_HIP_AI, RH_HIP_U, RH_HIP_D, Ia_r);  // same as: Ia_r = RH_HIP_AI - RH_HIP_U/RH_HIP_D * RH_HIP_U.transpose();
    pa = RH_HIP_p + Ia_r * RH_HIP_c + RH_HIP_U * RH_HIP_u/RH_HIP_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RH_HIP_X_fr_base, IaB);
    base_AI += IaB;
    base_p += (motionTransforms-> fr_RH_HIP_X_fr_base).transpose() * pa;
    
    // + Link LH_WHEEL_L
    LH_WHEEL_L_u = tau(LH_WHEEL) - LH_WHEEL_L_p(iit::rbd::AZ);
    LH_WHEEL_L_U = LH_WHEEL_L_AI.col(iit::rbd::AZ);
    LH_WHEEL_L_D = LH_WHEEL_L_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LH_WHEEL_L_AI, LH_WHEEL_L_U, LH_WHEEL_L_D, Ia_r);  // same as: Ia_r = LH_WHEEL_L_AI - LH_WHEEL_L_U/LH_WHEEL_L_D * LH_WHEEL_L_U.transpose();
    pa = LH_WHEEL_L_p + Ia_r * LH_WHEEL_L_c + LH_WHEEL_L_U * LH_WHEEL_L_u/LH_WHEEL_L_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LH_WHEEL_L_X_fr_LH_shank_fixed, IaB);
    LH_shank_fixed_AI += IaB;
    LH_shank_fixed_p += (motionTransforms-> fr_LH_WHEEL_L_X_fr_LH_shank_fixed).transpose() * pa;
    
    // + Link LH_shank_fixed
    LH_shank_fixed_u = tau(LH_KFE) - LH_shank_fixed_p(iit::rbd::AZ);
    LH_shank_fixed_U = LH_shank_fixed_AI.col(iit::rbd::AZ);
    LH_shank_fixed_D = LH_shank_fixed_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LH_shank_fixed_AI, LH_shank_fixed_U, LH_shank_fixed_D, Ia_r);  // same as: Ia_r = LH_shank_fixed_AI - LH_shank_fixed_U/LH_shank_fixed_D * LH_shank_fixed_U.transpose();
    pa = LH_shank_fixed_p + Ia_r * LH_shank_fixed_c + LH_shank_fixed_U * LH_shank_fixed_u/LH_shank_fixed_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LH_shank_fixed_X_fr_LH_THIGH, IaB);
    LH_THIGH_AI += IaB;
    LH_THIGH_p += (motionTransforms-> fr_LH_shank_fixed_X_fr_LH_THIGH).transpose() * pa;
    
    // + Link LH_THIGH
    LH_THIGH_u = tau(LH_HFE) - LH_THIGH_p(iit::rbd::AZ);
    LH_THIGH_U = LH_THIGH_AI.col(iit::rbd::AZ);
    LH_THIGH_D = LH_THIGH_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LH_THIGH_AI, LH_THIGH_U, LH_THIGH_D, Ia_r);  // same as: Ia_r = LH_THIGH_AI - LH_THIGH_U/LH_THIGH_D * LH_THIGH_U.transpose();
    pa = LH_THIGH_p + Ia_r * LH_THIGH_c + LH_THIGH_U * LH_THIGH_u/LH_THIGH_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LH_THIGH_X_fr_LH_HIP, IaB);
    LH_HIP_AI += IaB;
    LH_HIP_p += (motionTransforms-> fr_LH_THIGH_X_fr_LH_HIP).transpose() * pa;
    
    // + Link LH_HIP
    LH_HIP_u = tau(LH_HAA) - LH_HIP_p(iit::rbd::AZ);
    LH_HIP_U = LH_HIP_AI.col(iit::rbd::AZ);
    LH_HIP_D = LH_HIP_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LH_HIP_AI, LH_HIP_U, LH_HIP_D, Ia_r);  // same as: Ia_r = LH_HIP_AI - LH_HIP_U/LH_HIP_D * LH_HIP_U.transpose();
    pa = LH_HIP_p + Ia_r * LH_HIP_c + LH_HIP_U * LH_HIP_u/LH_HIP_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LH_HIP_X_fr_base, IaB);
    base_AI += IaB;
    base_p += (motionTransforms-> fr_LH_HIP_X_fr_base).transpose() * pa;
    
    // + Link RF_WHEEL_L
    RF_WHEEL_L_u = tau(RF_WHEEL) - RF_WHEEL_L_p(iit::rbd::AZ);
    RF_WHEEL_L_U = RF_WHEEL_L_AI.col(iit::rbd::AZ);
    RF_WHEEL_L_D = RF_WHEEL_L_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RF_WHEEL_L_AI, RF_WHEEL_L_U, RF_WHEEL_L_D, Ia_r);  // same as: Ia_r = RF_WHEEL_L_AI - RF_WHEEL_L_U/RF_WHEEL_L_D * RF_WHEEL_L_U.transpose();
    pa = RF_WHEEL_L_p + Ia_r * RF_WHEEL_L_c + RF_WHEEL_L_U * RF_WHEEL_L_u/RF_WHEEL_L_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_WHEEL_L_X_fr_RF_shank_fixed, IaB);
    RF_shank_fixed_AI += IaB;
    RF_shank_fixed_p += (motionTransforms-> fr_RF_WHEEL_L_X_fr_RF_shank_fixed).transpose() * pa;
    
    // + Link RF_shank_fixed
    RF_shank_fixed_u = tau(RF_KFE) - RF_shank_fixed_p(iit::rbd::AZ);
    RF_shank_fixed_U = RF_shank_fixed_AI.col(iit::rbd::AZ);
    RF_shank_fixed_D = RF_shank_fixed_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RF_shank_fixed_AI, RF_shank_fixed_U, RF_shank_fixed_D, Ia_r);  // same as: Ia_r = RF_shank_fixed_AI - RF_shank_fixed_U/RF_shank_fixed_D * RF_shank_fixed_U.transpose();
    pa = RF_shank_fixed_p + Ia_r * RF_shank_fixed_c + RF_shank_fixed_U * RF_shank_fixed_u/RF_shank_fixed_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_shank_fixed_X_fr_RF_THIGH, IaB);
    RF_THIGH_AI += IaB;
    RF_THIGH_p += (motionTransforms-> fr_RF_shank_fixed_X_fr_RF_THIGH).transpose() * pa;
    
    // + Link RF_THIGH
    RF_THIGH_u = tau(RF_HFE) - RF_THIGH_p(iit::rbd::AZ);
    RF_THIGH_U = RF_THIGH_AI.col(iit::rbd::AZ);
    RF_THIGH_D = RF_THIGH_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RF_THIGH_AI, RF_THIGH_U, RF_THIGH_D, Ia_r);  // same as: Ia_r = RF_THIGH_AI - RF_THIGH_U/RF_THIGH_D * RF_THIGH_U.transpose();
    pa = RF_THIGH_p + Ia_r * RF_THIGH_c + RF_THIGH_U * RF_THIGH_u/RF_THIGH_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_THIGH_X_fr_RF_HIP, IaB);
    RF_HIP_AI += IaB;
    RF_HIP_p += (motionTransforms-> fr_RF_THIGH_X_fr_RF_HIP).transpose() * pa;
    
    // + Link RF_HIP
    RF_HIP_u = tau(RF_HAA) - RF_HIP_p(iit::rbd::AZ);
    RF_HIP_U = RF_HIP_AI.col(iit::rbd::AZ);
    RF_HIP_D = RF_HIP_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RF_HIP_AI, RF_HIP_U, RF_HIP_D, Ia_r);  // same as: Ia_r = RF_HIP_AI - RF_HIP_U/RF_HIP_D * RF_HIP_U.transpose();
    pa = RF_HIP_p + Ia_r * RF_HIP_c + RF_HIP_U * RF_HIP_u/RF_HIP_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_HIP_X_fr_base, IaB);
    base_AI += IaB;
    base_p += (motionTransforms-> fr_RF_HIP_X_fr_base).transpose() * pa;
    
    // + Link LF_WHEEL_L
    LF_WHEEL_L_u = tau(LF_WHEEL) - LF_WHEEL_L_p(iit::rbd::AZ);
    LF_WHEEL_L_U = LF_WHEEL_L_AI.col(iit::rbd::AZ);
    LF_WHEEL_L_D = LF_WHEEL_L_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LF_WHEEL_L_AI, LF_WHEEL_L_U, LF_WHEEL_L_D, Ia_r);  // same as: Ia_r = LF_WHEEL_L_AI - LF_WHEEL_L_U/LF_WHEEL_L_D * LF_WHEEL_L_U.transpose();
    pa = LF_WHEEL_L_p + Ia_r * LF_WHEEL_L_c + LF_WHEEL_L_U * LF_WHEEL_L_u/LF_WHEEL_L_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_WHEEL_L_X_fr_LF_shank_fixed, IaB);
    LF_shank_fixed_AI += IaB;
    LF_shank_fixed_p += (motionTransforms-> fr_LF_WHEEL_L_X_fr_LF_shank_fixed).transpose() * pa;
    
    // + Link LF_shank_fixed
    LF_shank_fixed_u = tau(LF_KFE) - LF_shank_fixed_p(iit::rbd::AZ);
    LF_shank_fixed_U = LF_shank_fixed_AI.col(iit::rbd::AZ);
    LF_shank_fixed_D = LF_shank_fixed_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LF_shank_fixed_AI, LF_shank_fixed_U, LF_shank_fixed_D, Ia_r);  // same as: Ia_r = LF_shank_fixed_AI - LF_shank_fixed_U/LF_shank_fixed_D * LF_shank_fixed_U.transpose();
    pa = LF_shank_fixed_p + Ia_r * LF_shank_fixed_c + LF_shank_fixed_U * LF_shank_fixed_u/LF_shank_fixed_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_shank_fixed_X_fr_LF_THIGH, IaB);
    LF_THIGH_AI += IaB;
    LF_THIGH_p += (motionTransforms-> fr_LF_shank_fixed_X_fr_LF_THIGH).transpose() * pa;
    
    // + Link LF_THIGH
    LF_THIGH_u = tau(LF_HFE) - LF_THIGH_p(iit::rbd::AZ);
    LF_THIGH_U = LF_THIGH_AI.col(iit::rbd::AZ);
    LF_THIGH_D = LF_THIGH_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LF_THIGH_AI, LF_THIGH_U, LF_THIGH_D, Ia_r);  // same as: Ia_r = LF_THIGH_AI - LF_THIGH_U/LF_THIGH_D * LF_THIGH_U.transpose();
    pa = LF_THIGH_p + Ia_r * LF_THIGH_c + LF_THIGH_U * LF_THIGH_u/LF_THIGH_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_THIGH_X_fr_LF_HIP, IaB);
    LF_HIP_AI += IaB;
    LF_HIP_p += (motionTransforms-> fr_LF_THIGH_X_fr_LF_HIP).transpose() * pa;
    
    // + Link LF_HIP
    LF_HIP_u = tau(LF_HAA) - LF_HIP_p(iit::rbd::AZ);
    LF_HIP_U = LF_HIP_AI.col(iit::rbd::AZ);
    LF_HIP_D = LF_HIP_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LF_HIP_AI, LF_HIP_U, LF_HIP_D, Ia_r);  // same as: Ia_r = LF_HIP_AI - LF_HIP_U/LF_HIP_D * LF_HIP_U.transpose();
    pa = LF_HIP_p + Ia_r * LF_HIP_c + LF_HIP_U * LF_HIP_u/LF_HIP_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_HIP_X_fr_base, IaB);
    base_AI += IaB;
    base_p += (motionTransforms-> fr_LF_HIP_X_fr_base).transpose() * pa;
    
    // + The acceleration of the floating base base, without gravity
    base_a = - TRAIT::solve(base_AI, base_p);  // base_a = - IA^-1 * base_p
    
    // ---------------------- THIRD PASS ---------------------- //
    LF_HIP_a = (motionTransforms-> fr_LF_HIP_X_fr_base) * base_a + LF_HIP_c;
    qdd(LF_HAA) = (LF_HIP_u - LF_HIP_U.dot(LF_HIP_a)) / LF_HIP_D;
    LF_HIP_a(iit::rbd::AZ) += qdd(LF_HAA);
    
    LF_THIGH_a = (motionTransforms-> fr_LF_THIGH_X_fr_LF_HIP) * LF_HIP_a + LF_THIGH_c;
    qdd(LF_HFE) = (LF_THIGH_u - LF_THIGH_U.dot(LF_THIGH_a)) / LF_THIGH_D;
    LF_THIGH_a(iit::rbd::AZ) += qdd(LF_HFE);
    
    LF_shank_fixed_a = (motionTransforms-> fr_LF_shank_fixed_X_fr_LF_THIGH) * LF_THIGH_a + LF_shank_fixed_c;
    qdd(LF_KFE) = (LF_shank_fixed_u - LF_shank_fixed_U.dot(LF_shank_fixed_a)) / LF_shank_fixed_D;
    LF_shank_fixed_a(iit::rbd::AZ) += qdd(LF_KFE);
    
    LF_WHEEL_L_a = (motionTransforms-> fr_LF_WHEEL_L_X_fr_LF_shank_fixed) * LF_shank_fixed_a + LF_WHEEL_L_c;
    qdd(LF_WHEEL) = (LF_WHEEL_L_u - LF_WHEEL_L_U.dot(LF_WHEEL_L_a)) / LF_WHEEL_L_D;
    LF_WHEEL_L_a(iit::rbd::AZ) += qdd(LF_WHEEL);
    
    RF_HIP_a = (motionTransforms-> fr_RF_HIP_X_fr_base) * base_a + RF_HIP_c;
    qdd(RF_HAA) = (RF_HIP_u - RF_HIP_U.dot(RF_HIP_a)) / RF_HIP_D;
    RF_HIP_a(iit::rbd::AZ) += qdd(RF_HAA);
    
    RF_THIGH_a = (motionTransforms-> fr_RF_THIGH_X_fr_RF_HIP) * RF_HIP_a + RF_THIGH_c;
    qdd(RF_HFE) = (RF_THIGH_u - RF_THIGH_U.dot(RF_THIGH_a)) / RF_THIGH_D;
    RF_THIGH_a(iit::rbd::AZ) += qdd(RF_HFE);
    
    RF_shank_fixed_a = (motionTransforms-> fr_RF_shank_fixed_X_fr_RF_THIGH) * RF_THIGH_a + RF_shank_fixed_c;
    qdd(RF_KFE) = (RF_shank_fixed_u - RF_shank_fixed_U.dot(RF_shank_fixed_a)) / RF_shank_fixed_D;
    RF_shank_fixed_a(iit::rbd::AZ) += qdd(RF_KFE);
    
    RF_WHEEL_L_a = (motionTransforms-> fr_RF_WHEEL_L_X_fr_RF_shank_fixed) * RF_shank_fixed_a + RF_WHEEL_L_c;
    qdd(RF_WHEEL) = (RF_WHEEL_L_u - RF_WHEEL_L_U.dot(RF_WHEEL_L_a)) / RF_WHEEL_L_D;
    RF_WHEEL_L_a(iit::rbd::AZ) += qdd(RF_WHEEL);
    
    LH_HIP_a = (motionTransforms-> fr_LH_HIP_X_fr_base) * base_a + LH_HIP_c;
    qdd(LH_HAA) = (LH_HIP_u - LH_HIP_U.dot(LH_HIP_a)) / LH_HIP_D;
    LH_HIP_a(iit::rbd::AZ) += qdd(LH_HAA);
    
    LH_THIGH_a = (motionTransforms-> fr_LH_THIGH_X_fr_LH_HIP) * LH_HIP_a + LH_THIGH_c;
    qdd(LH_HFE) = (LH_THIGH_u - LH_THIGH_U.dot(LH_THIGH_a)) / LH_THIGH_D;
    LH_THIGH_a(iit::rbd::AZ) += qdd(LH_HFE);
    
    LH_shank_fixed_a = (motionTransforms-> fr_LH_shank_fixed_X_fr_LH_THIGH) * LH_THIGH_a + LH_shank_fixed_c;
    qdd(LH_KFE) = (LH_shank_fixed_u - LH_shank_fixed_U.dot(LH_shank_fixed_a)) / LH_shank_fixed_D;
    LH_shank_fixed_a(iit::rbd::AZ) += qdd(LH_KFE);
    
    LH_WHEEL_L_a = (motionTransforms-> fr_LH_WHEEL_L_X_fr_LH_shank_fixed) * LH_shank_fixed_a + LH_WHEEL_L_c;
    qdd(LH_WHEEL) = (LH_WHEEL_L_u - LH_WHEEL_L_U.dot(LH_WHEEL_L_a)) / LH_WHEEL_L_D;
    LH_WHEEL_L_a(iit::rbd::AZ) += qdd(LH_WHEEL);
    
    RH_HIP_a = (motionTransforms-> fr_RH_HIP_X_fr_base) * base_a + RH_HIP_c;
    qdd(RH_HAA) = (RH_HIP_u - RH_HIP_U.dot(RH_HIP_a)) / RH_HIP_D;
    RH_HIP_a(iit::rbd::AZ) += qdd(RH_HAA);
    
    RH_THIGH_a = (motionTransforms-> fr_RH_THIGH_X_fr_RH_HIP) * RH_HIP_a + RH_THIGH_c;
    qdd(RH_HFE) = (RH_THIGH_u - RH_THIGH_U.dot(RH_THIGH_a)) / RH_THIGH_D;
    RH_THIGH_a(iit::rbd::AZ) += qdd(RH_HFE);
    
    RH_shank_fixed_a = (motionTransforms-> fr_RH_shank_fixed_X_fr_RH_THIGH) * RH_THIGH_a + RH_shank_fixed_c;
    qdd(RH_KFE) = (RH_shank_fixed_u - RH_shank_fixed_U.dot(RH_shank_fixed_a)) / RH_shank_fixed_D;
    RH_shank_fixed_a(iit::rbd::AZ) += qdd(RH_KFE);
    
    RH_WHEEL_L_a = (motionTransforms-> fr_RH_WHEEL_L_X_fr_RH_shank_fixed) * RH_shank_fixed_a + RH_WHEEL_L_c;
    qdd(RH_WHEEL) = (RH_WHEEL_L_u - RH_WHEEL_L_U.dot(RH_WHEEL_L_a)) / RH_WHEEL_L_D;
    RH_WHEEL_L_a(iit::rbd::AZ) += qdd(RH_WHEEL);
    
    
    // + Add gravity to the acceleration of the floating base
    base_a += g;
}
