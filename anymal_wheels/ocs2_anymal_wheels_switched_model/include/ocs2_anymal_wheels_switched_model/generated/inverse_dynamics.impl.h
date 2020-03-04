// Initialization of static-const data
template <typename TRAIT>
const typename iit::ANYmal::dyn::tpl::InverseDynamics<TRAIT>::ExtForces
iit::ANYmal::dyn::tpl::InverseDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::ANYmal::dyn::tpl::InverseDynamics<TRAIT>::InverseDynamics(IProperties& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    LF_HIP_I(inertiaProps->getTensor_LF_HIP() ),
    LF_THIGH_I(inertiaProps->getTensor_LF_THIGH() ),
    LF_shank_fixed_I(inertiaProps->getTensor_LF_shank_fixed() ),
    LF_WHEEL_L_I(inertiaProps->getTensor_LF_WHEEL_L() ),
    RF_HIP_I(inertiaProps->getTensor_RF_HIP() ),
    RF_THIGH_I(inertiaProps->getTensor_RF_THIGH() ),
    RF_shank_fixed_I(inertiaProps->getTensor_RF_shank_fixed() ),
    RF_WHEEL_L_I(inertiaProps->getTensor_RF_WHEEL_L() ),
    LH_HIP_I(inertiaProps->getTensor_LH_HIP() ),
    LH_THIGH_I(inertiaProps->getTensor_LH_THIGH() ),
    LH_shank_fixed_I(inertiaProps->getTensor_LH_shank_fixed() ),
    LH_WHEEL_L_I(inertiaProps->getTensor_LH_WHEEL_L() ),
    RH_HIP_I(inertiaProps->getTensor_RH_HIP() ),
    RH_THIGH_I(inertiaProps->getTensor_RH_THIGH() ),
    RH_shank_fixed_I(inertiaProps->getTensor_RH_shank_fixed() ),
    RH_WHEEL_L_I(inertiaProps->getTensor_RH_WHEEL_L() )
    ,
        base_I( inertiaProps->getTensor_base() ),
        LF_WHEEL_L_Ic(LF_WHEEL_L_I),
        RF_WHEEL_L_Ic(RF_WHEEL_L_I),
        LH_WHEEL_L_Ic(LH_WHEEL_L_I),
        RH_WHEEL_L_Ic(RH_WHEEL_L_I)
{
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot ANYmal, InverseDynamics<TRAIT>::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    LF_HIP_v.setZero();
    LF_THIGH_v.setZero();
    LF_shank_fixed_v.setZero();
    LF_WHEEL_L_v.setZero();
    RF_HIP_v.setZero();
    RF_THIGH_v.setZero();
    RF_shank_fixed_v.setZero();
    RF_WHEEL_L_v.setZero();
    LH_HIP_v.setZero();
    LH_THIGH_v.setZero();
    LH_shank_fixed_v.setZero();
    LH_WHEEL_L_v.setZero();
    RH_HIP_v.setZero();
    RH_THIGH_v.setZero();
    RH_shank_fixed_v.setZero();
    RH_WHEEL_L_v.setZero();

    vcross.setZero();
}

template <typename TRAIT>
void iit::ANYmal::dyn::tpl::InverseDynamics<TRAIT>::id(
    JointState& jForces, Acceleration& base_a,
    const Acceleration& g, const Velocity& base_v,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    base_Ic = base_I;
    LF_HIP_Ic = LF_HIP_I;
    LF_THIGH_Ic = LF_THIGH_I;
    LF_shank_fixed_Ic = LF_shank_fixed_I;
    RF_HIP_Ic = RF_HIP_I;
    RF_THIGH_Ic = RF_THIGH_I;
    RF_shank_fixed_Ic = RF_shank_fixed_I;
    LH_HIP_Ic = LH_HIP_I;
    LH_THIGH_Ic = LH_THIGH_I;
    LH_shank_fixed_Ic = LH_shank_fixed_I;
    RH_HIP_Ic = RH_HIP_I;
    RH_THIGH_Ic = RH_THIGH_I;
    RH_shank_fixed_Ic = RH_shank_fixed_I;

    // First pass, link 'LF_HIP'
    LF_HIP_v = ((xm->fr_LF_HIP_X_fr_base) * base_v);
    LF_HIP_v(iit::rbd::AZ) += qd(LF_HAA);
    
    iit::rbd::motionCrossProductMx<Scalar>(LF_HIP_v, vcross);
    
    LF_HIP_a = (vcross.col(iit::rbd::AZ) * qd(LF_HAA));
    LF_HIP_a(iit::rbd::AZ) += qdd(LF_HAA);
    
    LF_HIP_f = LF_HIP_I * LF_HIP_a + iit::rbd::vxIv(LF_HIP_v, LF_HIP_I);
    
    // First pass, link 'LF_THIGH'
    LF_THIGH_v = ((xm->fr_LF_THIGH_X_fr_LF_HIP) * LF_HIP_v);
    LF_THIGH_v(iit::rbd::AZ) += qd(LF_HFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(LF_THIGH_v, vcross);
    
    LF_THIGH_a = (xm->fr_LF_THIGH_X_fr_LF_HIP) * LF_HIP_a + vcross.col(iit::rbd::AZ) * qd(LF_HFE);
    LF_THIGH_a(iit::rbd::AZ) += qdd(LF_HFE);
    
    LF_THIGH_f = LF_THIGH_I * LF_THIGH_a + iit::rbd::vxIv(LF_THIGH_v, LF_THIGH_I);
    
    // First pass, link 'LF_shank_fixed'
    LF_shank_fixed_v = ((xm->fr_LF_shank_fixed_X_fr_LF_THIGH) * LF_THIGH_v);
    LF_shank_fixed_v(iit::rbd::AZ) += qd(LF_KFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(LF_shank_fixed_v, vcross);
    
    LF_shank_fixed_a = (xm->fr_LF_shank_fixed_X_fr_LF_THIGH) * LF_THIGH_a + vcross.col(iit::rbd::AZ) * qd(LF_KFE);
    LF_shank_fixed_a(iit::rbd::AZ) += qdd(LF_KFE);
    
    LF_shank_fixed_f = LF_shank_fixed_I * LF_shank_fixed_a + iit::rbd::vxIv(LF_shank_fixed_v, LF_shank_fixed_I);
    
    // First pass, link 'LF_WHEEL_L'
    LF_WHEEL_L_v = ((xm->fr_LF_WHEEL_L_X_fr_LF_shank_fixed) * LF_shank_fixed_v);
    LF_WHEEL_L_v(iit::rbd::AZ) += qd(LF_WHEEL);
    
    iit::rbd::motionCrossProductMx<Scalar>(LF_WHEEL_L_v, vcross);
    
    LF_WHEEL_L_a = (xm->fr_LF_WHEEL_L_X_fr_LF_shank_fixed) * LF_shank_fixed_a + vcross.col(iit::rbd::AZ) * qd(LF_WHEEL);
    LF_WHEEL_L_a(iit::rbd::AZ) += qdd(LF_WHEEL);
    
    LF_WHEEL_L_f = LF_WHEEL_L_I * LF_WHEEL_L_a + iit::rbd::vxIv(LF_WHEEL_L_v, LF_WHEEL_L_I);
    
    // First pass, link 'RF_HIP'
    RF_HIP_v = ((xm->fr_RF_HIP_X_fr_base) * base_v);
    RF_HIP_v(iit::rbd::AZ) += qd(RF_HAA);
    
    iit::rbd::motionCrossProductMx<Scalar>(RF_HIP_v, vcross);
    
    RF_HIP_a = (vcross.col(iit::rbd::AZ) * qd(RF_HAA));
    RF_HIP_a(iit::rbd::AZ) += qdd(RF_HAA);
    
    RF_HIP_f = RF_HIP_I * RF_HIP_a + iit::rbd::vxIv(RF_HIP_v, RF_HIP_I);
    
    // First pass, link 'RF_THIGH'
    RF_THIGH_v = ((xm->fr_RF_THIGH_X_fr_RF_HIP) * RF_HIP_v);
    RF_THIGH_v(iit::rbd::AZ) += qd(RF_HFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(RF_THIGH_v, vcross);
    
    RF_THIGH_a = (xm->fr_RF_THIGH_X_fr_RF_HIP) * RF_HIP_a + vcross.col(iit::rbd::AZ) * qd(RF_HFE);
    RF_THIGH_a(iit::rbd::AZ) += qdd(RF_HFE);
    
    RF_THIGH_f = RF_THIGH_I * RF_THIGH_a + iit::rbd::vxIv(RF_THIGH_v, RF_THIGH_I);
    
    // First pass, link 'RF_shank_fixed'
    RF_shank_fixed_v = ((xm->fr_RF_shank_fixed_X_fr_RF_THIGH) * RF_THIGH_v);
    RF_shank_fixed_v(iit::rbd::AZ) += qd(RF_KFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(RF_shank_fixed_v, vcross);
    
    RF_shank_fixed_a = (xm->fr_RF_shank_fixed_X_fr_RF_THIGH) * RF_THIGH_a + vcross.col(iit::rbd::AZ) * qd(RF_KFE);
    RF_shank_fixed_a(iit::rbd::AZ) += qdd(RF_KFE);
    
    RF_shank_fixed_f = RF_shank_fixed_I * RF_shank_fixed_a + iit::rbd::vxIv(RF_shank_fixed_v, RF_shank_fixed_I);
    
    // First pass, link 'RF_WHEEL_L'
    RF_WHEEL_L_v = ((xm->fr_RF_WHEEL_L_X_fr_RF_shank_fixed) * RF_shank_fixed_v);
    RF_WHEEL_L_v(iit::rbd::AZ) += qd(RF_WHEEL);
    
    iit::rbd::motionCrossProductMx<Scalar>(RF_WHEEL_L_v, vcross);
    
    RF_WHEEL_L_a = (xm->fr_RF_WHEEL_L_X_fr_RF_shank_fixed) * RF_shank_fixed_a + vcross.col(iit::rbd::AZ) * qd(RF_WHEEL);
    RF_WHEEL_L_a(iit::rbd::AZ) += qdd(RF_WHEEL);
    
    RF_WHEEL_L_f = RF_WHEEL_L_I * RF_WHEEL_L_a + iit::rbd::vxIv(RF_WHEEL_L_v, RF_WHEEL_L_I);
    
    // First pass, link 'LH_HIP'
    LH_HIP_v = ((xm->fr_LH_HIP_X_fr_base) * base_v);
    LH_HIP_v(iit::rbd::AZ) += qd(LH_HAA);
    
    iit::rbd::motionCrossProductMx<Scalar>(LH_HIP_v, vcross);
    
    LH_HIP_a = (vcross.col(iit::rbd::AZ) * qd(LH_HAA));
    LH_HIP_a(iit::rbd::AZ) += qdd(LH_HAA);
    
    LH_HIP_f = LH_HIP_I * LH_HIP_a + iit::rbd::vxIv(LH_HIP_v, LH_HIP_I);
    
    // First pass, link 'LH_THIGH'
    LH_THIGH_v = ((xm->fr_LH_THIGH_X_fr_LH_HIP) * LH_HIP_v);
    LH_THIGH_v(iit::rbd::AZ) += qd(LH_HFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(LH_THIGH_v, vcross);
    
    LH_THIGH_a = (xm->fr_LH_THIGH_X_fr_LH_HIP) * LH_HIP_a + vcross.col(iit::rbd::AZ) * qd(LH_HFE);
    LH_THIGH_a(iit::rbd::AZ) += qdd(LH_HFE);
    
    LH_THIGH_f = LH_THIGH_I * LH_THIGH_a + iit::rbd::vxIv(LH_THIGH_v, LH_THIGH_I);
    
    // First pass, link 'LH_shank_fixed'
    LH_shank_fixed_v = ((xm->fr_LH_shank_fixed_X_fr_LH_THIGH) * LH_THIGH_v);
    LH_shank_fixed_v(iit::rbd::AZ) += qd(LH_KFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(LH_shank_fixed_v, vcross);
    
    LH_shank_fixed_a = (xm->fr_LH_shank_fixed_X_fr_LH_THIGH) * LH_THIGH_a + vcross.col(iit::rbd::AZ) * qd(LH_KFE);
    LH_shank_fixed_a(iit::rbd::AZ) += qdd(LH_KFE);
    
    LH_shank_fixed_f = LH_shank_fixed_I * LH_shank_fixed_a + iit::rbd::vxIv(LH_shank_fixed_v, LH_shank_fixed_I);
    
    // First pass, link 'LH_WHEEL_L'
    LH_WHEEL_L_v = ((xm->fr_LH_WHEEL_L_X_fr_LH_shank_fixed) * LH_shank_fixed_v);
    LH_WHEEL_L_v(iit::rbd::AZ) += qd(LH_WHEEL);
    
    iit::rbd::motionCrossProductMx<Scalar>(LH_WHEEL_L_v, vcross);
    
    LH_WHEEL_L_a = (xm->fr_LH_WHEEL_L_X_fr_LH_shank_fixed) * LH_shank_fixed_a + vcross.col(iit::rbd::AZ) * qd(LH_WHEEL);
    LH_WHEEL_L_a(iit::rbd::AZ) += qdd(LH_WHEEL);
    
    LH_WHEEL_L_f = LH_WHEEL_L_I * LH_WHEEL_L_a + iit::rbd::vxIv(LH_WHEEL_L_v, LH_WHEEL_L_I);
    
    // First pass, link 'RH_HIP'
    RH_HIP_v = ((xm->fr_RH_HIP_X_fr_base) * base_v);
    RH_HIP_v(iit::rbd::AZ) += qd(RH_HAA);
    
    iit::rbd::motionCrossProductMx<Scalar>(RH_HIP_v, vcross);
    
    RH_HIP_a = (vcross.col(iit::rbd::AZ) * qd(RH_HAA));
    RH_HIP_a(iit::rbd::AZ) += qdd(RH_HAA);
    
    RH_HIP_f = RH_HIP_I * RH_HIP_a + iit::rbd::vxIv(RH_HIP_v, RH_HIP_I);
    
    // First pass, link 'RH_THIGH'
    RH_THIGH_v = ((xm->fr_RH_THIGH_X_fr_RH_HIP) * RH_HIP_v);
    RH_THIGH_v(iit::rbd::AZ) += qd(RH_HFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(RH_THIGH_v, vcross);
    
    RH_THIGH_a = (xm->fr_RH_THIGH_X_fr_RH_HIP) * RH_HIP_a + vcross.col(iit::rbd::AZ) * qd(RH_HFE);
    RH_THIGH_a(iit::rbd::AZ) += qdd(RH_HFE);
    
    RH_THIGH_f = RH_THIGH_I * RH_THIGH_a + iit::rbd::vxIv(RH_THIGH_v, RH_THIGH_I);
    
    // First pass, link 'RH_shank_fixed'
    RH_shank_fixed_v = ((xm->fr_RH_shank_fixed_X_fr_RH_THIGH) * RH_THIGH_v);
    RH_shank_fixed_v(iit::rbd::AZ) += qd(RH_KFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(RH_shank_fixed_v, vcross);
    
    RH_shank_fixed_a = (xm->fr_RH_shank_fixed_X_fr_RH_THIGH) * RH_THIGH_a + vcross.col(iit::rbd::AZ) * qd(RH_KFE);
    RH_shank_fixed_a(iit::rbd::AZ) += qdd(RH_KFE);
    
    RH_shank_fixed_f = RH_shank_fixed_I * RH_shank_fixed_a + iit::rbd::vxIv(RH_shank_fixed_v, RH_shank_fixed_I);
    
    // First pass, link 'RH_WHEEL_L'
    RH_WHEEL_L_v = ((xm->fr_RH_WHEEL_L_X_fr_RH_shank_fixed) * RH_shank_fixed_v);
    RH_WHEEL_L_v(iit::rbd::AZ) += qd(RH_WHEEL);
    
    iit::rbd::motionCrossProductMx<Scalar>(RH_WHEEL_L_v, vcross);
    
    RH_WHEEL_L_a = (xm->fr_RH_WHEEL_L_X_fr_RH_shank_fixed) * RH_shank_fixed_a + vcross.col(iit::rbd::AZ) * qd(RH_WHEEL);
    RH_WHEEL_L_a(iit::rbd::AZ) += qdd(RH_WHEEL);
    
    RH_WHEEL_L_f = RH_WHEEL_L_I * RH_WHEEL_L_a + iit::rbd::vxIv(RH_WHEEL_L_v, RH_WHEEL_L_I);
    
    // The force exerted on the floating base by the links
    base_f = iit::rbd::vxIv(base_v, base_I);
    

    // Add the external forces:
    base_f -= fext[BASE];
    LF_HIP_f -= fext[LF_HIP];
    LF_THIGH_f -= fext[LF_THIGH];
    LF_shank_fixed_f -= fext[LF_SHANK_FIXED];
    LF_WHEEL_L_f -= fext[LF_WHEEL_L];
    RF_HIP_f -= fext[RF_HIP];
    RF_THIGH_f -= fext[RF_THIGH];
    RF_shank_fixed_f -= fext[RF_SHANK_FIXED];
    RF_WHEEL_L_f -= fext[RF_WHEEL_L];
    LH_HIP_f -= fext[LH_HIP];
    LH_THIGH_f -= fext[LH_THIGH];
    LH_shank_fixed_f -= fext[LH_SHANK_FIXED];
    LH_WHEEL_L_f -= fext[LH_WHEEL_L];
    RH_HIP_f -= fext[RH_HIP];
    RH_THIGH_f -= fext[RH_THIGH];
    RH_shank_fixed_f -= fext[RH_SHANK_FIXED];
    RH_WHEEL_L_f -= fext[RH_WHEEL_L];

    RH_shank_fixed_Ic = RH_shank_fixed_Ic + (xm->fr_RH_WHEEL_L_X_fr_RH_shank_fixed).transpose() * RH_WHEEL_L_Ic * (xm->fr_RH_WHEEL_L_X_fr_RH_shank_fixed);
    RH_shank_fixed_f = RH_shank_fixed_f + (xm->fr_RH_WHEEL_L_X_fr_RH_shank_fixed).transpose() * RH_WHEEL_L_f;
    
    RH_THIGH_Ic = RH_THIGH_Ic + (xm->fr_RH_shank_fixed_X_fr_RH_THIGH).transpose() * RH_shank_fixed_Ic * (xm->fr_RH_shank_fixed_X_fr_RH_THIGH);
    RH_THIGH_f = RH_THIGH_f + (xm->fr_RH_shank_fixed_X_fr_RH_THIGH).transpose() * RH_shank_fixed_f;
    
    RH_HIP_Ic = RH_HIP_Ic + (xm->fr_RH_THIGH_X_fr_RH_HIP).transpose() * RH_THIGH_Ic * (xm->fr_RH_THIGH_X_fr_RH_HIP);
    RH_HIP_f = RH_HIP_f + (xm->fr_RH_THIGH_X_fr_RH_HIP).transpose() * RH_THIGH_f;
    
    base_Ic = base_Ic + (xm->fr_RH_HIP_X_fr_base).transpose() * RH_HIP_Ic * (xm->fr_RH_HIP_X_fr_base);
    base_f = base_f + (xm->fr_RH_HIP_X_fr_base).transpose() * RH_HIP_f;
    
    LH_shank_fixed_Ic = LH_shank_fixed_Ic + (xm->fr_LH_WHEEL_L_X_fr_LH_shank_fixed).transpose() * LH_WHEEL_L_Ic * (xm->fr_LH_WHEEL_L_X_fr_LH_shank_fixed);
    LH_shank_fixed_f = LH_shank_fixed_f + (xm->fr_LH_WHEEL_L_X_fr_LH_shank_fixed).transpose() * LH_WHEEL_L_f;
    
    LH_THIGH_Ic = LH_THIGH_Ic + (xm->fr_LH_shank_fixed_X_fr_LH_THIGH).transpose() * LH_shank_fixed_Ic * (xm->fr_LH_shank_fixed_X_fr_LH_THIGH);
    LH_THIGH_f = LH_THIGH_f + (xm->fr_LH_shank_fixed_X_fr_LH_THIGH).transpose() * LH_shank_fixed_f;
    
    LH_HIP_Ic = LH_HIP_Ic + (xm->fr_LH_THIGH_X_fr_LH_HIP).transpose() * LH_THIGH_Ic * (xm->fr_LH_THIGH_X_fr_LH_HIP);
    LH_HIP_f = LH_HIP_f + (xm->fr_LH_THIGH_X_fr_LH_HIP).transpose() * LH_THIGH_f;
    
    base_Ic = base_Ic + (xm->fr_LH_HIP_X_fr_base).transpose() * LH_HIP_Ic * (xm->fr_LH_HIP_X_fr_base);
    base_f = base_f + (xm->fr_LH_HIP_X_fr_base).transpose() * LH_HIP_f;
    
    RF_shank_fixed_Ic = RF_shank_fixed_Ic + (xm->fr_RF_WHEEL_L_X_fr_RF_shank_fixed).transpose() * RF_WHEEL_L_Ic * (xm->fr_RF_WHEEL_L_X_fr_RF_shank_fixed);
    RF_shank_fixed_f = RF_shank_fixed_f + (xm->fr_RF_WHEEL_L_X_fr_RF_shank_fixed).transpose() * RF_WHEEL_L_f;
    
    RF_THIGH_Ic = RF_THIGH_Ic + (xm->fr_RF_shank_fixed_X_fr_RF_THIGH).transpose() * RF_shank_fixed_Ic * (xm->fr_RF_shank_fixed_X_fr_RF_THIGH);
    RF_THIGH_f = RF_THIGH_f + (xm->fr_RF_shank_fixed_X_fr_RF_THIGH).transpose() * RF_shank_fixed_f;
    
    RF_HIP_Ic = RF_HIP_Ic + (xm->fr_RF_THIGH_X_fr_RF_HIP).transpose() * RF_THIGH_Ic * (xm->fr_RF_THIGH_X_fr_RF_HIP);
    RF_HIP_f = RF_HIP_f + (xm->fr_RF_THIGH_X_fr_RF_HIP).transpose() * RF_THIGH_f;
    
    base_Ic = base_Ic + (xm->fr_RF_HIP_X_fr_base).transpose() * RF_HIP_Ic * (xm->fr_RF_HIP_X_fr_base);
    base_f = base_f + (xm->fr_RF_HIP_X_fr_base).transpose() * RF_HIP_f;
    
    LF_shank_fixed_Ic = LF_shank_fixed_Ic + (xm->fr_LF_WHEEL_L_X_fr_LF_shank_fixed).transpose() * LF_WHEEL_L_Ic * (xm->fr_LF_WHEEL_L_X_fr_LF_shank_fixed);
    LF_shank_fixed_f = LF_shank_fixed_f + (xm->fr_LF_WHEEL_L_X_fr_LF_shank_fixed).transpose() * LF_WHEEL_L_f;
    
    LF_THIGH_Ic = LF_THIGH_Ic + (xm->fr_LF_shank_fixed_X_fr_LF_THIGH).transpose() * LF_shank_fixed_Ic * (xm->fr_LF_shank_fixed_X_fr_LF_THIGH);
    LF_THIGH_f = LF_THIGH_f + (xm->fr_LF_shank_fixed_X_fr_LF_THIGH).transpose() * LF_shank_fixed_f;
    
    LF_HIP_Ic = LF_HIP_Ic + (xm->fr_LF_THIGH_X_fr_LF_HIP).transpose() * LF_THIGH_Ic * (xm->fr_LF_THIGH_X_fr_LF_HIP);
    LF_HIP_f = LF_HIP_f + (xm->fr_LF_THIGH_X_fr_LF_HIP).transpose() * LF_THIGH_f;
    
    base_Ic = base_Ic + (xm->fr_LF_HIP_X_fr_base).transpose() * LF_HIP_Ic * (xm->fr_LF_HIP_X_fr_base);
    base_f = base_f + (xm->fr_LF_HIP_X_fr_base).transpose() * LF_HIP_f;
    

    // The base acceleration due to the force due to the movement of the links
    base_a = - base_Ic.inverse() * base_f;
    
    LF_HIP_a = xm->fr_LF_HIP_X_fr_base * base_a;
    jForces(LF_HAA) = (LF_HIP_Ic.row(iit::rbd::AZ) * LF_HIP_a + LF_HIP_f(iit::rbd::AZ));
    
    LF_THIGH_a = xm->fr_LF_THIGH_X_fr_LF_HIP * LF_HIP_a;
    jForces(LF_HFE) = (LF_THIGH_Ic.row(iit::rbd::AZ) * LF_THIGH_a + LF_THIGH_f(iit::rbd::AZ));
    
    LF_shank_fixed_a = xm->fr_LF_shank_fixed_X_fr_LF_THIGH * LF_THIGH_a;
    jForces(LF_KFE) = (LF_shank_fixed_Ic.row(iit::rbd::AZ) * LF_shank_fixed_a + LF_shank_fixed_f(iit::rbd::AZ));
    
    LF_WHEEL_L_a = xm->fr_LF_WHEEL_L_X_fr_LF_shank_fixed * LF_shank_fixed_a;
    jForces(LF_WHEEL) = (LF_WHEEL_L_Ic.row(iit::rbd::AZ) * LF_WHEEL_L_a + LF_WHEEL_L_f(iit::rbd::AZ));
    
    RF_HIP_a = xm->fr_RF_HIP_X_fr_base * base_a;
    jForces(RF_HAA) = (RF_HIP_Ic.row(iit::rbd::AZ) * RF_HIP_a + RF_HIP_f(iit::rbd::AZ));
    
    RF_THIGH_a = xm->fr_RF_THIGH_X_fr_RF_HIP * RF_HIP_a;
    jForces(RF_HFE) = (RF_THIGH_Ic.row(iit::rbd::AZ) * RF_THIGH_a + RF_THIGH_f(iit::rbd::AZ));
    
    RF_shank_fixed_a = xm->fr_RF_shank_fixed_X_fr_RF_THIGH * RF_THIGH_a;
    jForces(RF_KFE) = (RF_shank_fixed_Ic.row(iit::rbd::AZ) * RF_shank_fixed_a + RF_shank_fixed_f(iit::rbd::AZ));
    
    RF_WHEEL_L_a = xm->fr_RF_WHEEL_L_X_fr_RF_shank_fixed * RF_shank_fixed_a;
    jForces(RF_WHEEL) = (RF_WHEEL_L_Ic.row(iit::rbd::AZ) * RF_WHEEL_L_a + RF_WHEEL_L_f(iit::rbd::AZ));
    
    LH_HIP_a = xm->fr_LH_HIP_X_fr_base * base_a;
    jForces(LH_HAA) = (LH_HIP_Ic.row(iit::rbd::AZ) * LH_HIP_a + LH_HIP_f(iit::rbd::AZ));
    
    LH_THIGH_a = xm->fr_LH_THIGH_X_fr_LH_HIP * LH_HIP_a;
    jForces(LH_HFE) = (LH_THIGH_Ic.row(iit::rbd::AZ) * LH_THIGH_a + LH_THIGH_f(iit::rbd::AZ));
    
    LH_shank_fixed_a = xm->fr_LH_shank_fixed_X_fr_LH_THIGH * LH_THIGH_a;
    jForces(LH_KFE) = (LH_shank_fixed_Ic.row(iit::rbd::AZ) * LH_shank_fixed_a + LH_shank_fixed_f(iit::rbd::AZ));
    
    LH_WHEEL_L_a = xm->fr_LH_WHEEL_L_X_fr_LH_shank_fixed * LH_shank_fixed_a;
    jForces(LH_WHEEL) = (LH_WHEEL_L_Ic.row(iit::rbd::AZ) * LH_WHEEL_L_a + LH_WHEEL_L_f(iit::rbd::AZ));
    
    RH_HIP_a = xm->fr_RH_HIP_X_fr_base * base_a;
    jForces(RH_HAA) = (RH_HIP_Ic.row(iit::rbd::AZ) * RH_HIP_a + RH_HIP_f(iit::rbd::AZ));
    
    RH_THIGH_a = xm->fr_RH_THIGH_X_fr_RH_HIP * RH_HIP_a;
    jForces(RH_HFE) = (RH_THIGH_Ic.row(iit::rbd::AZ) * RH_THIGH_a + RH_THIGH_f(iit::rbd::AZ));
    
    RH_shank_fixed_a = xm->fr_RH_shank_fixed_X_fr_RH_THIGH * RH_THIGH_a;
    jForces(RH_KFE) = (RH_shank_fixed_Ic.row(iit::rbd::AZ) * RH_shank_fixed_a + RH_shank_fixed_f(iit::rbd::AZ));
    
    RH_WHEEL_L_a = xm->fr_RH_WHEEL_L_X_fr_RH_shank_fixed * RH_shank_fixed_a;
    jForces(RH_WHEEL) = (RH_WHEEL_L_Ic.row(iit::rbd::AZ) * RH_WHEEL_L_a + RH_WHEEL_L_f(iit::rbd::AZ));
    

    base_a += g;
}

template <typename TRAIT>
void iit::ANYmal::dyn::tpl::InverseDynamics<TRAIT>::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g)
{
    const Acceleration& base_a = -g;

    // Link 'LF_HIP'
    LF_HIP_a = (xm->fr_LF_HIP_X_fr_base) * base_a;
    LF_HIP_f = LF_HIP_I * LF_HIP_a;
    // Link 'LF_THIGH'
    LF_THIGH_a = (xm->fr_LF_THIGH_X_fr_LF_HIP) * LF_HIP_a;
    LF_THIGH_f = LF_THIGH_I * LF_THIGH_a;
    // Link 'LF_shank_fixed'
    LF_shank_fixed_a = (xm->fr_LF_shank_fixed_X_fr_LF_THIGH) * LF_THIGH_a;
    LF_shank_fixed_f = LF_shank_fixed_I * LF_shank_fixed_a;
    // Link 'LF_WHEEL_L'
    LF_WHEEL_L_a = (xm->fr_LF_WHEEL_L_X_fr_LF_shank_fixed) * LF_shank_fixed_a;
    LF_WHEEL_L_f = LF_WHEEL_L_I * LF_WHEEL_L_a;
    // Link 'RF_HIP'
    RF_HIP_a = (xm->fr_RF_HIP_X_fr_base) * base_a;
    RF_HIP_f = RF_HIP_I * RF_HIP_a;
    // Link 'RF_THIGH'
    RF_THIGH_a = (xm->fr_RF_THIGH_X_fr_RF_HIP) * RF_HIP_a;
    RF_THIGH_f = RF_THIGH_I * RF_THIGH_a;
    // Link 'RF_shank_fixed'
    RF_shank_fixed_a = (xm->fr_RF_shank_fixed_X_fr_RF_THIGH) * RF_THIGH_a;
    RF_shank_fixed_f = RF_shank_fixed_I * RF_shank_fixed_a;
    // Link 'RF_WHEEL_L'
    RF_WHEEL_L_a = (xm->fr_RF_WHEEL_L_X_fr_RF_shank_fixed) * RF_shank_fixed_a;
    RF_WHEEL_L_f = RF_WHEEL_L_I * RF_WHEEL_L_a;
    // Link 'LH_HIP'
    LH_HIP_a = (xm->fr_LH_HIP_X_fr_base) * base_a;
    LH_HIP_f = LH_HIP_I * LH_HIP_a;
    // Link 'LH_THIGH'
    LH_THIGH_a = (xm->fr_LH_THIGH_X_fr_LH_HIP) * LH_HIP_a;
    LH_THIGH_f = LH_THIGH_I * LH_THIGH_a;
    // Link 'LH_shank_fixed'
    LH_shank_fixed_a = (xm->fr_LH_shank_fixed_X_fr_LH_THIGH) * LH_THIGH_a;
    LH_shank_fixed_f = LH_shank_fixed_I * LH_shank_fixed_a;
    // Link 'LH_WHEEL_L'
    LH_WHEEL_L_a = (xm->fr_LH_WHEEL_L_X_fr_LH_shank_fixed) * LH_shank_fixed_a;
    LH_WHEEL_L_f = LH_WHEEL_L_I * LH_WHEEL_L_a;
    // Link 'RH_HIP'
    RH_HIP_a = (xm->fr_RH_HIP_X_fr_base) * base_a;
    RH_HIP_f = RH_HIP_I * RH_HIP_a;
    // Link 'RH_THIGH'
    RH_THIGH_a = (xm->fr_RH_THIGH_X_fr_RH_HIP) * RH_HIP_a;
    RH_THIGH_f = RH_THIGH_I * RH_THIGH_a;
    // Link 'RH_shank_fixed'
    RH_shank_fixed_a = (xm->fr_RH_shank_fixed_X_fr_RH_THIGH) * RH_THIGH_a;
    RH_shank_fixed_f = RH_shank_fixed_I * RH_shank_fixed_a;
    // Link 'RH_WHEEL_L'
    RH_WHEEL_L_a = (xm->fr_RH_WHEEL_L_X_fr_RH_shank_fixed) * RH_shank_fixed_a;
    RH_WHEEL_L_f = RH_WHEEL_L_I * RH_WHEEL_L_a;

    base_f = base_I * base_a;

    secondPass_fullyActuated(jForces);

    baseWrench = base_f;
}

template <typename TRAIT>
void iit::ANYmal::dyn::tpl::InverseDynamics<TRAIT>::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_v, const JointState& qd)
{
    // Link 'LF_HIP'
    LF_HIP_v = ((xm->fr_LF_HIP_X_fr_base) * base_v);
    LF_HIP_v(iit::rbd::AZ) += qd(LF_HAA);
    iit::rbd::motionCrossProductMx<Scalar>(LF_HIP_v, vcross);
    LF_HIP_a = (vcross.col(iit::rbd::AZ) * qd(LF_HAA));
    LF_HIP_f = LF_HIP_I * LF_HIP_a + iit::rbd::vxIv(LF_HIP_v, LF_HIP_I);
    
    // Link 'LF_THIGH'
    LF_THIGH_v = ((xm->fr_LF_THIGH_X_fr_LF_HIP) * LF_HIP_v);
    LF_THIGH_v(iit::rbd::AZ) += qd(LF_HFE);
    iit::rbd::motionCrossProductMx<Scalar>(LF_THIGH_v, vcross);
    LF_THIGH_a = (xm->fr_LF_THIGH_X_fr_LF_HIP) * LF_HIP_a + vcross.col(iit::rbd::AZ) * qd(LF_HFE);
    LF_THIGH_f = LF_THIGH_I * LF_THIGH_a + iit::rbd::vxIv(LF_THIGH_v, LF_THIGH_I);
    
    // Link 'LF_shank_fixed'
    LF_shank_fixed_v = ((xm->fr_LF_shank_fixed_X_fr_LF_THIGH) * LF_THIGH_v);
    LF_shank_fixed_v(iit::rbd::AZ) += qd(LF_KFE);
    iit::rbd::motionCrossProductMx<Scalar>(LF_shank_fixed_v, vcross);
    LF_shank_fixed_a = (xm->fr_LF_shank_fixed_X_fr_LF_THIGH) * LF_THIGH_a + vcross.col(iit::rbd::AZ) * qd(LF_KFE);
    LF_shank_fixed_f = LF_shank_fixed_I * LF_shank_fixed_a + iit::rbd::vxIv(LF_shank_fixed_v, LF_shank_fixed_I);
    
    // Link 'LF_WHEEL_L'
    LF_WHEEL_L_v = ((xm->fr_LF_WHEEL_L_X_fr_LF_shank_fixed) * LF_shank_fixed_v);
    LF_WHEEL_L_v(iit::rbd::AZ) += qd(LF_WHEEL);
    iit::rbd::motionCrossProductMx<Scalar>(LF_WHEEL_L_v, vcross);
    LF_WHEEL_L_a = (xm->fr_LF_WHEEL_L_X_fr_LF_shank_fixed) * LF_shank_fixed_a + vcross.col(iit::rbd::AZ) * qd(LF_WHEEL);
    LF_WHEEL_L_f = LF_WHEEL_L_I * LF_WHEEL_L_a + iit::rbd::vxIv(LF_WHEEL_L_v, LF_WHEEL_L_I);
    
    // Link 'RF_HIP'
    RF_HIP_v = ((xm->fr_RF_HIP_X_fr_base) * base_v);
    RF_HIP_v(iit::rbd::AZ) += qd(RF_HAA);
    iit::rbd::motionCrossProductMx<Scalar>(RF_HIP_v, vcross);
    RF_HIP_a = (vcross.col(iit::rbd::AZ) * qd(RF_HAA));
    RF_HIP_f = RF_HIP_I * RF_HIP_a + iit::rbd::vxIv(RF_HIP_v, RF_HIP_I);
    
    // Link 'RF_THIGH'
    RF_THIGH_v = ((xm->fr_RF_THIGH_X_fr_RF_HIP) * RF_HIP_v);
    RF_THIGH_v(iit::rbd::AZ) += qd(RF_HFE);
    iit::rbd::motionCrossProductMx<Scalar>(RF_THIGH_v, vcross);
    RF_THIGH_a = (xm->fr_RF_THIGH_X_fr_RF_HIP) * RF_HIP_a + vcross.col(iit::rbd::AZ) * qd(RF_HFE);
    RF_THIGH_f = RF_THIGH_I * RF_THIGH_a + iit::rbd::vxIv(RF_THIGH_v, RF_THIGH_I);
    
    // Link 'RF_shank_fixed'
    RF_shank_fixed_v = ((xm->fr_RF_shank_fixed_X_fr_RF_THIGH) * RF_THIGH_v);
    RF_shank_fixed_v(iit::rbd::AZ) += qd(RF_KFE);
    iit::rbd::motionCrossProductMx<Scalar>(RF_shank_fixed_v, vcross);
    RF_shank_fixed_a = (xm->fr_RF_shank_fixed_X_fr_RF_THIGH) * RF_THIGH_a + vcross.col(iit::rbd::AZ) * qd(RF_KFE);
    RF_shank_fixed_f = RF_shank_fixed_I * RF_shank_fixed_a + iit::rbd::vxIv(RF_shank_fixed_v, RF_shank_fixed_I);
    
    // Link 'RF_WHEEL_L'
    RF_WHEEL_L_v = ((xm->fr_RF_WHEEL_L_X_fr_RF_shank_fixed) * RF_shank_fixed_v);
    RF_WHEEL_L_v(iit::rbd::AZ) += qd(RF_WHEEL);
    iit::rbd::motionCrossProductMx<Scalar>(RF_WHEEL_L_v, vcross);
    RF_WHEEL_L_a = (xm->fr_RF_WHEEL_L_X_fr_RF_shank_fixed) * RF_shank_fixed_a + vcross.col(iit::rbd::AZ) * qd(RF_WHEEL);
    RF_WHEEL_L_f = RF_WHEEL_L_I * RF_WHEEL_L_a + iit::rbd::vxIv(RF_WHEEL_L_v, RF_WHEEL_L_I);
    
    // Link 'LH_HIP'
    LH_HIP_v = ((xm->fr_LH_HIP_X_fr_base) * base_v);
    LH_HIP_v(iit::rbd::AZ) += qd(LH_HAA);
    iit::rbd::motionCrossProductMx<Scalar>(LH_HIP_v, vcross);
    LH_HIP_a = (vcross.col(iit::rbd::AZ) * qd(LH_HAA));
    LH_HIP_f = LH_HIP_I * LH_HIP_a + iit::rbd::vxIv(LH_HIP_v, LH_HIP_I);
    
    // Link 'LH_THIGH'
    LH_THIGH_v = ((xm->fr_LH_THIGH_X_fr_LH_HIP) * LH_HIP_v);
    LH_THIGH_v(iit::rbd::AZ) += qd(LH_HFE);
    iit::rbd::motionCrossProductMx<Scalar>(LH_THIGH_v, vcross);
    LH_THIGH_a = (xm->fr_LH_THIGH_X_fr_LH_HIP) * LH_HIP_a + vcross.col(iit::rbd::AZ) * qd(LH_HFE);
    LH_THIGH_f = LH_THIGH_I * LH_THIGH_a + iit::rbd::vxIv(LH_THIGH_v, LH_THIGH_I);
    
    // Link 'LH_shank_fixed'
    LH_shank_fixed_v = ((xm->fr_LH_shank_fixed_X_fr_LH_THIGH) * LH_THIGH_v);
    LH_shank_fixed_v(iit::rbd::AZ) += qd(LH_KFE);
    iit::rbd::motionCrossProductMx<Scalar>(LH_shank_fixed_v, vcross);
    LH_shank_fixed_a = (xm->fr_LH_shank_fixed_X_fr_LH_THIGH) * LH_THIGH_a + vcross.col(iit::rbd::AZ) * qd(LH_KFE);
    LH_shank_fixed_f = LH_shank_fixed_I * LH_shank_fixed_a + iit::rbd::vxIv(LH_shank_fixed_v, LH_shank_fixed_I);
    
    // Link 'LH_WHEEL_L'
    LH_WHEEL_L_v = ((xm->fr_LH_WHEEL_L_X_fr_LH_shank_fixed) * LH_shank_fixed_v);
    LH_WHEEL_L_v(iit::rbd::AZ) += qd(LH_WHEEL);
    iit::rbd::motionCrossProductMx<Scalar>(LH_WHEEL_L_v, vcross);
    LH_WHEEL_L_a = (xm->fr_LH_WHEEL_L_X_fr_LH_shank_fixed) * LH_shank_fixed_a + vcross.col(iit::rbd::AZ) * qd(LH_WHEEL);
    LH_WHEEL_L_f = LH_WHEEL_L_I * LH_WHEEL_L_a + iit::rbd::vxIv(LH_WHEEL_L_v, LH_WHEEL_L_I);
    
    // Link 'RH_HIP'
    RH_HIP_v = ((xm->fr_RH_HIP_X_fr_base) * base_v);
    RH_HIP_v(iit::rbd::AZ) += qd(RH_HAA);
    iit::rbd::motionCrossProductMx<Scalar>(RH_HIP_v, vcross);
    RH_HIP_a = (vcross.col(iit::rbd::AZ) * qd(RH_HAA));
    RH_HIP_f = RH_HIP_I * RH_HIP_a + iit::rbd::vxIv(RH_HIP_v, RH_HIP_I);
    
    // Link 'RH_THIGH'
    RH_THIGH_v = ((xm->fr_RH_THIGH_X_fr_RH_HIP) * RH_HIP_v);
    RH_THIGH_v(iit::rbd::AZ) += qd(RH_HFE);
    iit::rbd::motionCrossProductMx<Scalar>(RH_THIGH_v, vcross);
    RH_THIGH_a = (xm->fr_RH_THIGH_X_fr_RH_HIP) * RH_HIP_a + vcross.col(iit::rbd::AZ) * qd(RH_HFE);
    RH_THIGH_f = RH_THIGH_I * RH_THIGH_a + iit::rbd::vxIv(RH_THIGH_v, RH_THIGH_I);
    
    // Link 'RH_shank_fixed'
    RH_shank_fixed_v = ((xm->fr_RH_shank_fixed_X_fr_RH_THIGH) * RH_THIGH_v);
    RH_shank_fixed_v(iit::rbd::AZ) += qd(RH_KFE);
    iit::rbd::motionCrossProductMx<Scalar>(RH_shank_fixed_v, vcross);
    RH_shank_fixed_a = (xm->fr_RH_shank_fixed_X_fr_RH_THIGH) * RH_THIGH_a + vcross.col(iit::rbd::AZ) * qd(RH_KFE);
    RH_shank_fixed_f = RH_shank_fixed_I * RH_shank_fixed_a + iit::rbd::vxIv(RH_shank_fixed_v, RH_shank_fixed_I);
    
    // Link 'RH_WHEEL_L'
    RH_WHEEL_L_v = ((xm->fr_RH_WHEEL_L_X_fr_RH_shank_fixed) * RH_shank_fixed_v);
    RH_WHEEL_L_v(iit::rbd::AZ) += qd(RH_WHEEL);
    iit::rbd::motionCrossProductMx<Scalar>(RH_WHEEL_L_v, vcross);
    RH_WHEEL_L_a = (xm->fr_RH_WHEEL_L_X_fr_RH_shank_fixed) * RH_shank_fixed_a + vcross.col(iit::rbd::AZ) * qd(RH_WHEEL);
    RH_WHEEL_L_f = RH_WHEEL_L_I * RH_WHEEL_L_a + iit::rbd::vxIv(RH_WHEEL_L_v, RH_WHEEL_L_I);
    

    base_f = iit::rbd::vxIv(base_v, base_I);

    secondPass_fullyActuated(jForces);

    baseWrench = base_f;
}

template <typename TRAIT>
void iit::ANYmal::dyn::tpl::InverseDynamics<TRAIT>::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    Acceleration base_a = baseAccel -g;

    // First pass, link 'LF_HIP'
    LF_HIP_v = ((xm->fr_LF_HIP_X_fr_base) * base_v);
    LF_HIP_v(iit::rbd::AZ) += qd(LF_HAA);
    
    iit::rbd::motionCrossProductMx<Scalar>(LF_HIP_v, vcross);
    
    LF_HIP_a = (xm->fr_LF_HIP_X_fr_base) * base_a + vcross.col(iit::rbd::AZ) * qd(LF_HAA);
    LF_HIP_a(iit::rbd::AZ) += qdd(LF_HAA);
    
    LF_HIP_f = LF_HIP_I * LF_HIP_a + iit::rbd::vxIv(LF_HIP_v, LF_HIP_I) - fext[LF_HIP];
    
    // First pass, link 'LF_THIGH'
    LF_THIGH_v = ((xm->fr_LF_THIGH_X_fr_LF_HIP) * LF_HIP_v);
    LF_THIGH_v(iit::rbd::AZ) += qd(LF_HFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(LF_THIGH_v, vcross);
    
    LF_THIGH_a = (xm->fr_LF_THIGH_X_fr_LF_HIP) * LF_HIP_a + vcross.col(iit::rbd::AZ) * qd(LF_HFE);
    LF_THIGH_a(iit::rbd::AZ) += qdd(LF_HFE);
    
    LF_THIGH_f = LF_THIGH_I * LF_THIGH_a + iit::rbd::vxIv(LF_THIGH_v, LF_THIGH_I) - fext[LF_THIGH];
    
    // First pass, link 'LF_shank_fixed'
    LF_shank_fixed_v = ((xm->fr_LF_shank_fixed_X_fr_LF_THIGH) * LF_THIGH_v);
    LF_shank_fixed_v(iit::rbd::AZ) += qd(LF_KFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(LF_shank_fixed_v, vcross);
    
    LF_shank_fixed_a = (xm->fr_LF_shank_fixed_X_fr_LF_THIGH) * LF_THIGH_a + vcross.col(iit::rbd::AZ) * qd(LF_KFE);
    LF_shank_fixed_a(iit::rbd::AZ) += qdd(LF_KFE);
    
    LF_shank_fixed_f = LF_shank_fixed_I * LF_shank_fixed_a + iit::rbd::vxIv(LF_shank_fixed_v, LF_shank_fixed_I) - fext[LF_SHANK_FIXED];
    
    // First pass, link 'LF_WHEEL_L'
    LF_WHEEL_L_v = ((xm->fr_LF_WHEEL_L_X_fr_LF_shank_fixed) * LF_shank_fixed_v);
    LF_WHEEL_L_v(iit::rbd::AZ) += qd(LF_WHEEL);
    
    iit::rbd::motionCrossProductMx<Scalar>(LF_WHEEL_L_v, vcross);
    
    LF_WHEEL_L_a = (xm->fr_LF_WHEEL_L_X_fr_LF_shank_fixed) * LF_shank_fixed_a + vcross.col(iit::rbd::AZ) * qd(LF_WHEEL);
    LF_WHEEL_L_a(iit::rbd::AZ) += qdd(LF_WHEEL);
    
    LF_WHEEL_L_f = LF_WHEEL_L_I * LF_WHEEL_L_a + iit::rbd::vxIv(LF_WHEEL_L_v, LF_WHEEL_L_I) - fext[LF_WHEEL_L];
    
    // First pass, link 'RF_HIP'
    RF_HIP_v = ((xm->fr_RF_HIP_X_fr_base) * base_v);
    RF_HIP_v(iit::rbd::AZ) += qd(RF_HAA);
    
    iit::rbd::motionCrossProductMx<Scalar>(RF_HIP_v, vcross);
    
    RF_HIP_a = (xm->fr_RF_HIP_X_fr_base) * base_a + vcross.col(iit::rbd::AZ) * qd(RF_HAA);
    RF_HIP_a(iit::rbd::AZ) += qdd(RF_HAA);
    
    RF_HIP_f = RF_HIP_I * RF_HIP_a + iit::rbd::vxIv(RF_HIP_v, RF_HIP_I) - fext[RF_HIP];
    
    // First pass, link 'RF_THIGH'
    RF_THIGH_v = ((xm->fr_RF_THIGH_X_fr_RF_HIP) * RF_HIP_v);
    RF_THIGH_v(iit::rbd::AZ) += qd(RF_HFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(RF_THIGH_v, vcross);
    
    RF_THIGH_a = (xm->fr_RF_THIGH_X_fr_RF_HIP) * RF_HIP_a + vcross.col(iit::rbd::AZ) * qd(RF_HFE);
    RF_THIGH_a(iit::rbd::AZ) += qdd(RF_HFE);
    
    RF_THIGH_f = RF_THIGH_I * RF_THIGH_a + iit::rbd::vxIv(RF_THIGH_v, RF_THIGH_I) - fext[RF_THIGH];
    
    // First pass, link 'RF_shank_fixed'
    RF_shank_fixed_v = ((xm->fr_RF_shank_fixed_X_fr_RF_THIGH) * RF_THIGH_v);
    RF_shank_fixed_v(iit::rbd::AZ) += qd(RF_KFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(RF_shank_fixed_v, vcross);
    
    RF_shank_fixed_a = (xm->fr_RF_shank_fixed_X_fr_RF_THIGH) * RF_THIGH_a + vcross.col(iit::rbd::AZ) * qd(RF_KFE);
    RF_shank_fixed_a(iit::rbd::AZ) += qdd(RF_KFE);
    
    RF_shank_fixed_f = RF_shank_fixed_I * RF_shank_fixed_a + iit::rbd::vxIv(RF_shank_fixed_v, RF_shank_fixed_I) - fext[RF_SHANK_FIXED];
    
    // First pass, link 'RF_WHEEL_L'
    RF_WHEEL_L_v = ((xm->fr_RF_WHEEL_L_X_fr_RF_shank_fixed) * RF_shank_fixed_v);
    RF_WHEEL_L_v(iit::rbd::AZ) += qd(RF_WHEEL);
    
    iit::rbd::motionCrossProductMx<Scalar>(RF_WHEEL_L_v, vcross);
    
    RF_WHEEL_L_a = (xm->fr_RF_WHEEL_L_X_fr_RF_shank_fixed) * RF_shank_fixed_a + vcross.col(iit::rbd::AZ) * qd(RF_WHEEL);
    RF_WHEEL_L_a(iit::rbd::AZ) += qdd(RF_WHEEL);
    
    RF_WHEEL_L_f = RF_WHEEL_L_I * RF_WHEEL_L_a + iit::rbd::vxIv(RF_WHEEL_L_v, RF_WHEEL_L_I) - fext[RF_WHEEL_L];
    
    // First pass, link 'LH_HIP'
    LH_HIP_v = ((xm->fr_LH_HIP_X_fr_base) * base_v);
    LH_HIP_v(iit::rbd::AZ) += qd(LH_HAA);
    
    iit::rbd::motionCrossProductMx<Scalar>(LH_HIP_v, vcross);
    
    LH_HIP_a = (xm->fr_LH_HIP_X_fr_base) * base_a + vcross.col(iit::rbd::AZ) * qd(LH_HAA);
    LH_HIP_a(iit::rbd::AZ) += qdd(LH_HAA);
    
    LH_HIP_f = LH_HIP_I * LH_HIP_a + iit::rbd::vxIv(LH_HIP_v, LH_HIP_I) - fext[LH_HIP];
    
    // First pass, link 'LH_THIGH'
    LH_THIGH_v = ((xm->fr_LH_THIGH_X_fr_LH_HIP) * LH_HIP_v);
    LH_THIGH_v(iit::rbd::AZ) += qd(LH_HFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(LH_THIGH_v, vcross);
    
    LH_THIGH_a = (xm->fr_LH_THIGH_X_fr_LH_HIP) * LH_HIP_a + vcross.col(iit::rbd::AZ) * qd(LH_HFE);
    LH_THIGH_a(iit::rbd::AZ) += qdd(LH_HFE);
    
    LH_THIGH_f = LH_THIGH_I * LH_THIGH_a + iit::rbd::vxIv(LH_THIGH_v, LH_THIGH_I) - fext[LH_THIGH];
    
    // First pass, link 'LH_shank_fixed'
    LH_shank_fixed_v = ((xm->fr_LH_shank_fixed_X_fr_LH_THIGH) * LH_THIGH_v);
    LH_shank_fixed_v(iit::rbd::AZ) += qd(LH_KFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(LH_shank_fixed_v, vcross);
    
    LH_shank_fixed_a = (xm->fr_LH_shank_fixed_X_fr_LH_THIGH) * LH_THIGH_a + vcross.col(iit::rbd::AZ) * qd(LH_KFE);
    LH_shank_fixed_a(iit::rbd::AZ) += qdd(LH_KFE);
    
    LH_shank_fixed_f = LH_shank_fixed_I * LH_shank_fixed_a + iit::rbd::vxIv(LH_shank_fixed_v, LH_shank_fixed_I) - fext[LH_SHANK_FIXED];
    
    // First pass, link 'LH_WHEEL_L'
    LH_WHEEL_L_v = ((xm->fr_LH_WHEEL_L_X_fr_LH_shank_fixed) * LH_shank_fixed_v);
    LH_WHEEL_L_v(iit::rbd::AZ) += qd(LH_WHEEL);
    
    iit::rbd::motionCrossProductMx<Scalar>(LH_WHEEL_L_v, vcross);
    
    LH_WHEEL_L_a = (xm->fr_LH_WHEEL_L_X_fr_LH_shank_fixed) * LH_shank_fixed_a + vcross.col(iit::rbd::AZ) * qd(LH_WHEEL);
    LH_WHEEL_L_a(iit::rbd::AZ) += qdd(LH_WHEEL);
    
    LH_WHEEL_L_f = LH_WHEEL_L_I * LH_WHEEL_L_a + iit::rbd::vxIv(LH_WHEEL_L_v, LH_WHEEL_L_I) - fext[LH_WHEEL_L];
    
    // First pass, link 'RH_HIP'
    RH_HIP_v = ((xm->fr_RH_HIP_X_fr_base) * base_v);
    RH_HIP_v(iit::rbd::AZ) += qd(RH_HAA);
    
    iit::rbd::motionCrossProductMx<Scalar>(RH_HIP_v, vcross);
    
    RH_HIP_a = (xm->fr_RH_HIP_X_fr_base) * base_a + vcross.col(iit::rbd::AZ) * qd(RH_HAA);
    RH_HIP_a(iit::rbd::AZ) += qdd(RH_HAA);
    
    RH_HIP_f = RH_HIP_I * RH_HIP_a + iit::rbd::vxIv(RH_HIP_v, RH_HIP_I) - fext[RH_HIP];
    
    // First pass, link 'RH_THIGH'
    RH_THIGH_v = ((xm->fr_RH_THIGH_X_fr_RH_HIP) * RH_HIP_v);
    RH_THIGH_v(iit::rbd::AZ) += qd(RH_HFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(RH_THIGH_v, vcross);
    
    RH_THIGH_a = (xm->fr_RH_THIGH_X_fr_RH_HIP) * RH_HIP_a + vcross.col(iit::rbd::AZ) * qd(RH_HFE);
    RH_THIGH_a(iit::rbd::AZ) += qdd(RH_HFE);
    
    RH_THIGH_f = RH_THIGH_I * RH_THIGH_a + iit::rbd::vxIv(RH_THIGH_v, RH_THIGH_I) - fext[RH_THIGH];
    
    // First pass, link 'RH_shank_fixed'
    RH_shank_fixed_v = ((xm->fr_RH_shank_fixed_X_fr_RH_THIGH) * RH_THIGH_v);
    RH_shank_fixed_v(iit::rbd::AZ) += qd(RH_KFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(RH_shank_fixed_v, vcross);
    
    RH_shank_fixed_a = (xm->fr_RH_shank_fixed_X_fr_RH_THIGH) * RH_THIGH_a + vcross.col(iit::rbd::AZ) * qd(RH_KFE);
    RH_shank_fixed_a(iit::rbd::AZ) += qdd(RH_KFE);
    
    RH_shank_fixed_f = RH_shank_fixed_I * RH_shank_fixed_a + iit::rbd::vxIv(RH_shank_fixed_v, RH_shank_fixed_I) - fext[RH_SHANK_FIXED];
    
    // First pass, link 'RH_WHEEL_L'
    RH_WHEEL_L_v = ((xm->fr_RH_WHEEL_L_X_fr_RH_shank_fixed) * RH_shank_fixed_v);
    RH_WHEEL_L_v(iit::rbd::AZ) += qd(RH_WHEEL);
    
    iit::rbd::motionCrossProductMx<Scalar>(RH_WHEEL_L_v, vcross);
    
    RH_WHEEL_L_a = (xm->fr_RH_WHEEL_L_X_fr_RH_shank_fixed) * RH_shank_fixed_a + vcross.col(iit::rbd::AZ) * qd(RH_WHEEL);
    RH_WHEEL_L_a(iit::rbd::AZ) += qdd(RH_WHEEL);
    
    RH_WHEEL_L_f = RH_WHEEL_L_I * RH_WHEEL_L_a + iit::rbd::vxIv(RH_WHEEL_L_v, RH_WHEEL_L_I) - fext[RH_WHEEL_L];
    

    // The base
    base_f = base_I * base_a + iit::rbd::vxIv(base_v, base_I) - fext[BASE];

    secondPass_fullyActuated(jForces);

    baseWrench = base_f;
}

template <typename TRAIT>
void iit::ANYmal::dyn::tpl::InverseDynamics<TRAIT>::secondPass_fullyActuated(JointState& jForces)
{
    // Link 'RH_WHEEL_L'
    jForces(RH_WHEEL) = RH_WHEEL_L_f(iit::rbd::AZ);
    RH_shank_fixed_f += xm->fr_RH_WHEEL_L_X_fr_RH_shank_fixed.transpose() * RH_WHEEL_L_f;
    // Link 'RH_shank_fixed'
    jForces(RH_KFE) = RH_shank_fixed_f(iit::rbd::AZ);
    RH_THIGH_f += xm->fr_RH_shank_fixed_X_fr_RH_THIGH.transpose() * RH_shank_fixed_f;
    // Link 'RH_THIGH'
    jForces(RH_HFE) = RH_THIGH_f(iit::rbd::AZ);
    RH_HIP_f += xm->fr_RH_THIGH_X_fr_RH_HIP.transpose() * RH_THIGH_f;
    // Link 'RH_HIP'
    jForces(RH_HAA) = RH_HIP_f(iit::rbd::AZ);
    base_f += xm->fr_RH_HIP_X_fr_base.transpose() * RH_HIP_f;
    // Link 'LH_WHEEL_L'
    jForces(LH_WHEEL) = LH_WHEEL_L_f(iit::rbd::AZ);
    LH_shank_fixed_f += xm->fr_LH_WHEEL_L_X_fr_LH_shank_fixed.transpose() * LH_WHEEL_L_f;
    // Link 'LH_shank_fixed'
    jForces(LH_KFE) = LH_shank_fixed_f(iit::rbd::AZ);
    LH_THIGH_f += xm->fr_LH_shank_fixed_X_fr_LH_THIGH.transpose() * LH_shank_fixed_f;
    // Link 'LH_THIGH'
    jForces(LH_HFE) = LH_THIGH_f(iit::rbd::AZ);
    LH_HIP_f += xm->fr_LH_THIGH_X_fr_LH_HIP.transpose() * LH_THIGH_f;
    // Link 'LH_HIP'
    jForces(LH_HAA) = LH_HIP_f(iit::rbd::AZ);
    base_f += xm->fr_LH_HIP_X_fr_base.transpose() * LH_HIP_f;
    // Link 'RF_WHEEL_L'
    jForces(RF_WHEEL) = RF_WHEEL_L_f(iit::rbd::AZ);
    RF_shank_fixed_f += xm->fr_RF_WHEEL_L_X_fr_RF_shank_fixed.transpose() * RF_WHEEL_L_f;
    // Link 'RF_shank_fixed'
    jForces(RF_KFE) = RF_shank_fixed_f(iit::rbd::AZ);
    RF_THIGH_f += xm->fr_RF_shank_fixed_X_fr_RF_THIGH.transpose() * RF_shank_fixed_f;
    // Link 'RF_THIGH'
    jForces(RF_HFE) = RF_THIGH_f(iit::rbd::AZ);
    RF_HIP_f += xm->fr_RF_THIGH_X_fr_RF_HIP.transpose() * RF_THIGH_f;
    // Link 'RF_HIP'
    jForces(RF_HAA) = RF_HIP_f(iit::rbd::AZ);
    base_f += xm->fr_RF_HIP_X_fr_base.transpose() * RF_HIP_f;
    // Link 'LF_WHEEL_L'
    jForces(LF_WHEEL) = LF_WHEEL_L_f(iit::rbd::AZ);
    LF_shank_fixed_f += xm->fr_LF_WHEEL_L_X_fr_LF_shank_fixed.transpose() * LF_WHEEL_L_f;
    // Link 'LF_shank_fixed'
    jForces(LF_KFE) = LF_shank_fixed_f(iit::rbd::AZ);
    LF_THIGH_f += xm->fr_LF_shank_fixed_X_fr_LF_THIGH.transpose() * LF_shank_fixed_f;
    // Link 'LF_THIGH'
    jForces(LF_HFE) = LF_THIGH_f(iit::rbd::AZ);
    LF_HIP_f += xm->fr_LF_THIGH_X_fr_LF_HIP.transpose() * LF_THIGH_f;
    // Link 'LF_HIP'
    jForces(LF_HAA) = LF_HIP_f(iit::rbd::AZ);
    base_f += xm->fr_LF_HIP_X_fr_base.transpose() * LF_HIP_f;
}

