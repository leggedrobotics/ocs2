// Initialization of static-const data
template <typename TRAIT>
const typename iit::camel::dyn::tpl::InverseDynamics<TRAIT>::ExtForces
iit::camel::dyn::tpl::InverseDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::camel::dyn::tpl::InverseDynamics<TRAIT>::InverseDynamics(IProperties& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    LF_HIP_I(inertiaProps->getTensor_LF_HIP() ),
    LF_THIGH_I(inertiaProps->getTensor_LF_THIGH() ),
    LF_SHANK_I(inertiaProps->getTensor_LF_SHANK() ),
    RF_HIP_I(inertiaProps->getTensor_RF_HIP() ),
    RF_THIGH_I(inertiaProps->getTensor_RF_THIGH() ),
    RF_SHANK_I(inertiaProps->getTensor_RF_SHANK() ),
    LH_HIP_I(inertiaProps->getTensor_LH_HIP() ),
    LH_THIGH_I(inertiaProps->getTensor_LH_THIGH() ),
    LH_SHANK_I(inertiaProps->getTensor_LH_SHANK() ),
    RH_HIP_I(inertiaProps->getTensor_RH_HIP() ),
    RH_THIGH_I(inertiaProps->getTensor_RH_THIGH() ),
    RH_SHANK_I(inertiaProps->getTensor_RH_SHANK() )
    ,
        base_I( inertiaProps->getTensor_base() ),
        LF_SHANK_Ic(LF_SHANK_I),
        RF_SHANK_Ic(RF_SHANK_I),
        LH_SHANK_Ic(LH_SHANK_I),
        RH_SHANK_Ic(RH_SHANK_I)
{
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot camel, InverseDynamics<TRAIT>::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    LF_HIP_v.setZero();
    LF_THIGH_v.setZero();
    LF_SHANK_v.setZero();
    RF_HIP_v.setZero();
    RF_THIGH_v.setZero();
    RF_SHANK_v.setZero();
    LH_HIP_v.setZero();
    LH_THIGH_v.setZero();
    LH_SHANK_v.setZero();
    RH_HIP_v.setZero();
    RH_THIGH_v.setZero();
    RH_SHANK_v.setZero();

    vcross.setZero();
}

template <typename TRAIT>
void iit::camel::dyn::tpl::InverseDynamics<TRAIT>::id(
    JointState& jForces, Acceleration& base_a,
    const Acceleration& g, const Velocity& base_v,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    base_Ic = base_I;
    LF_HIP_Ic = LF_HIP_I;
    LF_THIGH_Ic = LF_THIGH_I;
    RF_HIP_Ic = RF_HIP_I;
    RF_THIGH_Ic = RF_THIGH_I;
    LH_HIP_Ic = LH_HIP_I;
    LH_THIGH_Ic = LH_THIGH_I;
    RH_HIP_Ic = RH_HIP_I;
    RH_THIGH_Ic = RH_THIGH_I;

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
    
    // First pass, link 'LF_SHANK'
    LF_SHANK_v = ((xm->fr_LF_SHANK_X_fr_LF_THIGH) * LF_THIGH_v);
    LF_SHANK_v(iit::rbd::AZ) += qd(LF_KFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(LF_SHANK_v, vcross);
    
    LF_SHANK_a = (xm->fr_LF_SHANK_X_fr_LF_THIGH) * LF_THIGH_a + vcross.col(iit::rbd::AZ) * qd(LF_KFE);
    LF_SHANK_a(iit::rbd::AZ) += qdd(LF_KFE);
    
    LF_SHANK_f = LF_SHANK_I * LF_SHANK_a + iit::rbd::vxIv(LF_SHANK_v, LF_SHANK_I);
    
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
    
    // First pass, link 'RF_SHANK'
    RF_SHANK_v = ((xm->fr_RF_SHANK_X_fr_RF_THIGH) * RF_THIGH_v);
    RF_SHANK_v(iit::rbd::AZ) += qd(RF_KFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(RF_SHANK_v, vcross);
    
    RF_SHANK_a = (xm->fr_RF_SHANK_X_fr_RF_THIGH) * RF_THIGH_a + vcross.col(iit::rbd::AZ) * qd(RF_KFE);
    RF_SHANK_a(iit::rbd::AZ) += qdd(RF_KFE);
    
    RF_SHANK_f = RF_SHANK_I * RF_SHANK_a + iit::rbd::vxIv(RF_SHANK_v, RF_SHANK_I);
    
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
    
    // First pass, link 'LH_SHANK'
    LH_SHANK_v = ((xm->fr_LH_SHANK_X_fr_LH_THIGH) * LH_THIGH_v);
    LH_SHANK_v(iit::rbd::AZ) += qd(LH_KFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(LH_SHANK_v, vcross);
    
    LH_SHANK_a = (xm->fr_LH_SHANK_X_fr_LH_THIGH) * LH_THIGH_a + vcross.col(iit::rbd::AZ) * qd(LH_KFE);
    LH_SHANK_a(iit::rbd::AZ) += qdd(LH_KFE);
    
    LH_SHANK_f = LH_SHANK_I * LH_SHANK_a + iit::rbd::vxIv(LH_SHANK_v, LH_SHANK_I);
    
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
    
    // First pass, link 'RH_SHANK'
    RH_SHANK_v = ((xm->fr_RH_SHANK_X_fr_RH_THIGH) * RH_THIGH_v);
    RH_SHANK_v(iit::rbd::AZ) += qd(RH_KFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(RH_SHANK_v, vcross);
    
    RH_SHANK_a = (xm->fr_RH_SHANK_X_fr_RH_THIGH) * RH_THIGH_a + vcross.col(iit::rbd::AZ) * qd(RH_KFE);
    RH_SHANK_a(iit::rbd::AZ) += qdd(RH_KFE);
    
    RH_SHANK_f = RH_SHANK_I * RH_SHANK_a + iit::rbd::vxIv(RH_SHANK_v, RH_SHANK_I);
    
    // The force exerted on the floating base by the links
    base_f = iit::rbd::vxIv(base_v, base_I);
    

    // Add the external forces:
    base_f -= fext[BASE];
    LF_HIP_f -= fext[LF_HIP];
    LF_THIGH_f -= fext[LF_THIGH];
    LF_SHANK_f -= fext[LF_SHANK];
    RF_HIP_f -= fext[RF_HIP];
    RF_THIGH_f -= fext[RF_THIGH];
    RF_SHANK_f -= fext[RF_SHANK];
    LH_HIP_f -= fext[LH_HIP];
    LH_THIGH_f -= fext[LH_THIGH];
    LH_SHANK_f -= fext[LH_SHANK];
    RH_HIP_f -= fext[RH_HIP];
    RH_THIGH_f -= fext[RH_THIGH];
    RH_SHANK_f -= fext[RH_SHANK];

    RH_THIGH_Ic = RH_THIGH_Ic + (xm->fr_RH_SHANK_X_fr_RH_THIGH).transpose() * RH_SHANK_Ic * (xm->fr_RH_SHANK_X_fr_RH_THIGH);
    RH_THIGH_f = RH_THIGH_f + (xm->fr_RH_SHANK_X_fr_RH_THIGH).transpose() * RH_SHANK_f;
    
    RH_HIP_Ic = RH_HIP_Ic + (xm->fr_RH_THIGH_X_fr_RH_HIP).transpose() * RH_THIGH_Ic * (xm->fr_RH_THIGH_X_fr_RH_HIP);
    RH_HIP_f = RH_HIP_f + (xm->fr_RH_THIGH_X_fr_RH_HIP).transpose() * RH_THIGH_f;
    
    base_Ic = base_Ic + (xm->fr_RH_HIP_X_fr_base).transpose() * RH_HIP_Ic * (xm->fr_RH_HIP_X_fr_base);
    base_f = base_f + (xm->fr_RH_HIP_X_fr_base).transpose() * RH_HIP_f;
    
    LH_THIGH_Ic = LH_THIGH_Ic + (xm->fr_LH_SHANK_X_fr_LH_THIGH).transpose() * LH_SHANK_Ic * (xm->fr_LH_SHANK_X_fr_LH_THIGH);
    LH_THIGH_f = LH_THIGH_f + (xm->fr_LH_SHANK_X_fr_LH_THIGH).transpose() * LH_SHANK_f;
    
    LH_HIP_Ic = LH_HIP_Ic + (xm->fr_LH_THIGH_X_fr_LH_HIP).transpose() * LH_THIGH_Ic * (xm->fr_LH_THIGH_X_fr_LH_HIP);
    LH_HIP_f = LH_HIP_f + (xm->fr_LH_THIGH_X_fr_LH_HIP).transpose() * LH_THIGH_f;
    
    base_Ic = base_Ic + (xm->fr_LH_HIP_X_fr_base).transpose() * LH_HIP_Ic * (xm->fr_LH_HIP_X_fr_base);
    base_f = base_f + (xm->fr_LH_HIP_X_fr_base).transpose() * LH_HIP_f;
    
    RF_THIGH_Ic = RF_THIGH_Ic + (xm->fr_RF_SHANK_X_fr_RF_THIGH).transpose() * RF_SHANK_Ic * (xm->fr_RF_SHANK_X_fr_RF_THIGH);
    RF_THIGH_f = RF_THIGH_f + (xm->fr_RF_SHANK_X_fr_RF_THIGH).transpose() * RF_SHANK_f;
    
    RF_HIP_Ic = RF_HIP_Ic + (xm->fr_RF_THIGH_X_fr_RF_HIP).transpose() * RF_THIGH_Ic * (xm->fr_RF_THIGH_X_fr_RF_HIP);
    RF_HIP_f = RF_HIP_f + (xm->fr_RF_THIGH_X_fr_RF_HIP).transpose() * RF_THIGH_f;
    
    base_Ic = base_Ic + (xm->fr_RF_HIP_X_fr_base).transpose() * RF_HIP_Ic * (xm->fr_RF_HIP_X_fr_base);
    base_f = base_f + (xm->fr_RF_HIP_X_fr_base).transpose() * RF_HIP_f;
    
    LF_THIGH_Ic = LF_THIGH_Ic + (xm->fr_LF_SHANK_X_fr_LF_THIGH).transpose() * LF_SHANK_Ic * (xm->fr_LF_SHANK_X_fr_LF_THIGH);
    LF_THIGH_f = LF_THIGH_f + (xm->fr_LF_SHANK_X_fr_LF_THIGH).transpose() * LF_SHANK_f;
    
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
    
    LF_SHANK_a = xm->fr_LF_SHANK_X_fr_LF_THIGH * LF_THIGH_a;
    jForces(LF_KFE) = (LF_SHANK_Ic.row(iit::rbd::AZ) * LF_SHANK_a + LF_SHANK_f(iit::rbd::AZ));
    
    RF_HIP_a = xm->fr_RF_HIP_X_fr_base * base_a;
    jForces(RF_HAA) = (RF_HIP_Ic.row(iit::rbd::AZ) * RF_HIP_a + RF_HIP_f(iit::rbd::AZ));
    
    RF_THIGH_a = xm->fr_RF_THIGH_X_fr_RF_HIP * RF_HIP_a;
    jForces(RF_HFE) = (RF_THIGH_Ic.row(iit::rbd::AZ) * RF_THIGH_a + RF_THIGH_f(iit::rbd::AZ));
    
    RF_SHANK_a = xm->fr_RF_SHANK_X_fr_RF_THIGH * RF_THIGH_a;
    jForces(RF_KFE) = (RF_SHANK_Ic.row(iit::rbd::AZ) * RF_SHANK_a + RF_SHANK_f(iit::rbd::AZ));
    
    LH_HIP_a = xm->fr_LH_HIP_X_fr_base * base_a;
    jForces(LH_HAA) = (LH_HIP_Ic.row(iit::rbd::AZ) * LH_HIP_a + LH_HIP_f(iit::rbd::AZ));
    
    LH_THIGH_a = xm->fr_LH_THIGH_X_fr_LH_HIP * LH_HIP_a;
    jForces(LH_HFE) = (LH_THIGH_Ic.row(iit::rbd::AZ) * LH_THIGH_a + LH_THIGH_f(iit::rbd::AZ));
    
    LH_SHANK_a = xm->fr_LH_SHANK_X_fr_LH_THIGH * LH_THIGH_a;
    jForces(LH_KFE) = (LH_SHANK_Ic.row(iit::rbd::AZ) * LH_SHANK_a + LH_SHANK_f(iit::rbd::AZ));
    
    RH_HIP_a = xm->fr_RH_HIP_X_fr_base * base_a;
    jForces(RH_HAA) = (RH_HIP_Ic.row(iit::rbd::AZ) * RH_HIP_a + RH_HIP_f(iit::rbd::AZ));
    
    RH_THIGH_a = xm->fr_RH_THIGH_X_fr_RH_HIP * RH_HIP_a;
    jForces(RH_HFE) = (RH_THIGH_Ic.row(iit::rbd::AZ) * RH_THIGH_a + RH_THIGH_f(iit::rbd::AZ));
    
    RH_SHANK_a = xm->fr_RH_SHANK_X_fr_RH_THIGH * RH_THIGH_a;
    jForces(RH_KFE) = (RH_SHANK_Ic.row(iit::rbd::AZ) * RH_SHANK_a + RH_SHANK_f(iit::rbd::AZ));
    

    base_a += g;
}

template <typename TRAIT>
void iit::camel::dyn::tpl::InverseDynamics<TRAIT>::G_terms_fully_actuated(
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
    // Link 'LF_SHANK'
    LF_SHANK_a = (xm->fr_LF_SHANK_X_fr_LF_THIGH) * LF_THIGH_a;
    LF_SHANK_f = LF_SHANK_I * LF_SHANK_a;
    // Link 'RF_HIP'
    RF_HIP_a = (xm->fr_RF_HIP_X_fr_base) * base_a;
    RF_HIP_f = RF_HIP_I * RF_HIP_a;
    // Link 'RF_THIGH'
    RF_THIGH_a = (xm->fr_RF_THIGH_X_fr_RF_HIP) * RF_HIP_a;
    RF_THIGH_f = RF_THIGH_I * RF_THIGH_a;
    // Link 'RF_SHANK'
    RF_SHANK_a = (xm->fr_RF_SHANK_X_fr_RF_THIGH) * RF_THIGH_a;
    RF_SHANK_f = RF_SHANK_I * RF_SHANK_a;
    // Link 'LH_HIP'
    LH_HIP_a = (xm->fr_LH_HIP_X_fr_base) * base_a;
    LH_HIP_f = LH_HIP_I * LH_HIP_a;
    // Link 'LH_THIGH'
    LH_THIGH_a = (xm->fr_LH_THIGH_X_fr_LH_HIP) * LH_HIP_a;
    LH_THIGH_f = LH_THIGH_I * LH_THIGH_a;
    // Link 'LH_SHANK'
    LH_SHANK_a = (xm->fr_LH_SHANK_X_fr_LH_THIGH) * LH_THIGH_a;
    LH_SHANK_f = LH_SHANK_I * LH_SHANK_a;
    // Link 'RH_HIP'
    RH_HIP_a = (xm->fr_RH_HIP_X_fr_base) * base_a;
    RH_HIP_f = RH_HIP_I * RH_HIP_a;
    // Link 'RH_THIGH'
    RH_THIGH_a = (xm->fr_RH_THIGH_X_fr_RH_HIP) * RH_HIP_a;
    RH_THIGH_f = RH_THIGH_I * RH_THIGH_a;
    // Link 'RH_SHANK'
    RH_SHANK_a = (xm->fr_RH_SHANK_X_fr_RH_THIGH) * RH_THIGH_a;
    RH_SHANK_f = RH_SHANK_I * RH_SHANK_a;

    base_f = base_I * base_a;

    secondPass_fullyActuated(jForces);

    baseWrench = base_f;
}

template <typename TRAIT>
void iit::camel::dyn::tpl::InverseDynamics<TRAIT>::C_terms_fully_actuated(
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
    
    // Link 'LF_SHANK'
    LF_SHANK_v = ((xm->fr_LF_SHANK_X_fr_LF_THIGH) * LF_THIGH_v);
    LF_SHANK_v(iit::rbd::AZ) += qd(LF_KFE);
    iit::rbd::motionCrossProductMx<Scalar>(LF_SHANK_v, vcross);
    LF_SHANK_a = (xm->fr_LF_SHANK_X_fr_LF_THIGH) * LF_THIGH_a + vcross.col(iit::rbd::AZ) * qd(LF_KFE);
    LF_SHANK_f = LF_SHANK_I * LF_SHANK_a + iit::rbd::vxIv(LF_SHANK_v, LF_SHANK_I);
    
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
    
    // Link 'RF_SHANK'
    RF_SHANK_v = ((xm->fr_RF_SHANK_X_fr_RF_THIGH) * RF_THIGH_v);
    RF_SHANK_v(iit::rbd::AZ) += qd(RF_KFE);
    iit::rbd::motionCrossProductMx<Scalar>(RF_SHANK_v, vcross);
    RF_SHANK_a = (xm->fr_RF_SHANK_X_fr_RF_THIGH) * RF_THIGH_a + vcross.col(iit::rbd::AZ) * qd(RF_KFE);
    RF_SHANK_f = RF_SHANK_I * RF_SHANK_a + iit::rbd::vxIv(RF_SHANK_v, RF_SHANK_I);
    
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
    
    // Link 'LH_SHANK'
    LH_SHANK_v = ((xm->fr_LH_SHANK_X_fr_LH_THIGH) * LH_THIGH_v);
    LH_SHANK_v(iit::rbd::AZ) += qd(LH_KFE);
    iit::rbd::motionCrossProductMx<Scalar>(LH_SHANK_v, vcross);
    LH_SHANK_a = (xm->fr_LH_SHANK_X_fr_LH_THIGH) * LH_THIGH_a + vcross.col(iit::rbd::AZ) * qd(LH_KFE);
    LH_SHANK_f = LH_SHANK_I * LH_SHANK_a + iit::rbd::vxIv(LH_SHANK_v, LH_SHANK_I);
    
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
    
    // Link 'RH_SHANK'
    RH_SHANK_v = ((xm->fr_RH_SHANK_X_fr_RH_THIGH) * RH_THIGH_v);
    RH_SHANK_v(iit::rbd::AZ) += qd(RH_KFE);
    iit::rbd::motionCrossProductMx<Scalar>(RH_SHANK_v, vcross);
    RH_SHANK_a = (xm->fr_RH_SHANK_X_fr_RH_THIGH) * RH_THIGH_a + vcross.col(iit::rbd::AZ) * qd(RH_KFE);
    RH_SHANK_f = RH_SHANK_I * RH_SHANK_a + iit::rbd::vxIv(RH_SHANK_v, RH_SHANK_I);
    

    base_f = iit::rbd::vxIv(base_v, base_I);

    secondPass_fullyActuated(jForces);

    baseWrench = base_f;
}

template <typename TRAIT>
void iit::camel::dyn::tpl::InverseDynamics<TRAIT>::id_fully_actuated(
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
    
    // First pass, link 'LF_SHANK'
    LF_SHANK_v = ((xm->fr_LF_SHANK_X_fr_LF_THIGH) * LF_THIGH_v);
    LF_SHANK_v(iit::rbd::AZ) += qd(LF_KFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(LF_SHANK_v, vcross);
    
    LF_SHANK_a = (xm->fr_LF_SHANK_X_fr_LF_THIGH) * LF_THIGH_a + vcross.col(iit::rbd::AZ) * qd(LF_KFE);
    LF_SHANK_a(iit::rbd::AZ) += qdd(LF_KFE);
    
    LF_SHANK_f = LF_SHANK_I * LF_SHANK_a + iit::rbd::vxIv(LF_SHANK_v, LF_SHANK_I) - fext[LF_SHANK];
    
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
    
    // First pass, link 'RF_SHANK'
    RF_SHANK_v = ((xm->fr_RF_SHANK_X_fr_RF_THIGH) * RF_THIGH_v);
    RF_SHANK_v(iit::rbd::AZ) += qd(RF_KFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(RF_SHANK_v, vcross);
    
    RF_SHANK_a = (xm->fr_RF_SHANK_X_fr_RF_THIGH) * RF_THIGH_a + vcross.col(iit::rbd::AZ) * qd(RF_KFE);
    RF_SHANK_a(iit::rbd::AZ) += qdd(RF_KFE);
    
    RF_SHANK_f = RF_SHANK_I * RF_SHANK_a + iit::rbd::vxIv(RF_SHANK_v, RF_SHANK_I) - fext[RF_SHANK];
    
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
    
    // First pass, link 'LH_SHANK'
    LH_SHANK_v = ((xm->fr_LH_SHANK_X_fr_LH_THIGH) * LH_THIGH_v);
    LH_SHANK_v(iit::rbd::AZ) += qd(LH_KFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(LH_SHANK_v, vcross);
    
    LH_SHANK_a = (xm->fr_LH_SHANK_X_fr_LH_THIGH) * LH_THIGH_a + vcross.col(iit::rbd::AZ) * qd(LH_KFE);
    LH_SHANK_a(iit::rbd::AZ) += qdd(LH_KFE);
    
    LH_SHANK_f = LH_SHANK_I * LH_SHANK_a + iit::rbd::vxIv(LH_SHANK_v, LH_SHANK_I) - fext[LH_SHANK];
    
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
    
    // First pass, link 'RH_SHANK'
    RH_SHANK_v = ((xm->fr_RH_SHANK_X_fr_RH_THIGH) * RH_THIGH_v);
    RH_SHANK_v(iit::rbd::AZ) += qd(RH_KFE);
    
    iit::rbd::motionCrossProductMx<Scalar>(RH_SHANK_v, vcross);
    
    RH_SHANK_a = (xm->fr_RH_SHANK_X_fr_RH_THIGH) * RH_THIGH_a + vcross.col(iit::rbd::AZ) * qd(RH_KFE);
    RH_SHANK_a(iit::rbd::AZ) += qdd(RH_KFE);
    
    RH_SHANK_f = RH_SHANK_I * RH_SHANK_a + iit::rbd::vxIv(RH_SHANK_v, RH_SHANK_I) - fext[RH_SHANK];
    

    // The base
    base_f = base_I * base_a + iit::rbd::vxIv(base_v, base_I) - fext[BASE];

    secondPass_fullyActuated(jForces);

    baseWrench = base_f;
}

template <typename TRAIT>
void iit::camel::dyn::tpl::InverseDynamics<TRAIT>::secondPass_fullyActuated(JointState& jForces)
{
    // Link 'RH_SHANK'
    jForces(RH_KFE) = RH_SHANK_f(iit::rbd::AZ);
    RH_THIGH_f += xm->fr_RH_SHANK_X_fr_RH_THIGH.transpose() * RH_SHANK_f;
    // Link 'RH_THIGH'
    jForces(RH_HFE) = RH_THIGH_f(iit::rbd::AZ);
    RH_HIP_f += xm->fr_RH_THIGH_X_fr_RH_HIP.transpose() * RH_THIGH_f;
    // Link 'RH_HIP'
    jForces(RH_HAA) = RH_HIP_f(iit::rbd::AZ);
    base_f += xm->fr_RH_HIP_X_fr_base.transpose() * RH_HIP_f;
    // Link 'LH_SHANK'
    jForces(LH_KFE) = LH_SHANK_f(iit::rbd::AZ);
    LH_THIGH_f += xm->fr_LH_SHANK_X_fr_LH_THIGH.transpose() * LH_SHANK_f;
    // Link 'LH_THIGH'
    jForces(LH_HFE) = LH_THIGH_f(iit::rbd::AZ);
    LH_HIP_f += xm->fr_LH_THIGH_X_fr_LH_HIP.transpose() * LH_THIGH_f;
    // Link 'LH_HIP'
    jForces(LH_HAA) = LH_HIP_f(iit::rbd::AZ);
    base_f += xm->fr_LH_HIP_X_fr_base.transpose() * LH_HIP_f;
    // Link 'RF_SHANK'
    jForces(RF_KFE) = RF_SHANK_f(iit::rbd::AZ);
    RF_THIGH_f += xm->fr_RF_SHANK_X_fr_RF_THIGH.transpose() * RF_SHANK_f;
    // Link 'RF_THIGH'
    jForces(RF_HFE) = RF_THIGH_f(iit::rbd::AZ);
    RF_HIP_f += xm->fr_RF_THIGH_X_fr_RF_HIP.transpose() * RF_THIGH_f;
    // Link 'RF_HIP'
    jForces(RF_HAA) = RF_HIP_f(iit::rbd::AZ);
    base_f += xm->fr_RF_HIP_X_fr_base.transpose() * RF_HIP_f;
    // Link 'LF_SHANK'
    jForces(LF_KFE) = LF_SHANK_f(iit::rbd::AZ);
    LF_THIGH_f += xm->fr_LF_SHANK_X_fr_LF_THIGH.transpose() * LF_SHANK_f;
    // Link 'LF_THIGH'
    jForces(LF_HFE) = LF_THIGH_f(iit::rbd::AZ);
    LF_HIP_f += xm->fr_LF_THIGH_X_fr_LF_HIP.transpose() * LF_THIGH_f;
    // Link 'LF_HIP'
    jForces(LF_HAA) = LF_HIP_f(iit::rbd::AZ);
    base_f += xm->fr_LF_HIP_X_fr_base.transpose() * LF_HIP_f;
}

