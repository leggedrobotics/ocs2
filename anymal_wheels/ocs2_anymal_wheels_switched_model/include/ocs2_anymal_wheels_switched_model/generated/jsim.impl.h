

//Implementation of default constructor
template <typename TRAIT>
iit::ANYmal::dyn::tpl::JSIM<TRAIT>::JSIM(IProperties& inertiaProperties, FTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    LF_WHEEL_L_Ic(linkInertias.getTensor_LF_WHEEL_L()),
    RF_WHEEL_L_Ic(linkInertias.getTensor_RF_WHEEL_L()),
    LH_WHEEL_L_Ic(linkInertias.getTensor_LH_WHEEL_L()),
    RH_WHEEL_L_Ic(linkInertias.getTensor_RH_WHEEL_L())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA tpl::JSIM<TRAIT>::operator()
#define Fcol(j) (tpl::JSIM<TRAIT>:: template block<6,1>(0,(j)+6))
#define F(i,j) DATA((i),(j)+6)

template <typename TRAIT>
const typename iit::ANYmal::dyn::tpl::JSIM<TRAIT>& iit::ANYmal::dyn::tpl::JSIM<TRAIT>::update(const JointState& state) {

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_RH_SHANK_X_fr_RH_WHEEL_L(state);
    frcTransf -> fr_RH_THIGH_X_fr_RH_SHANK(state);
    frcTransf -> fr_RH_HIP_X_fr_RH_THIGH(state);
    frcTransf -> fr_base_X_fr_RH_HIP(state);
    frcTransf -> fr_LH_SHANK_X_fr_LH_WHEEL_L(state);
    frcTransf -> fr_LH_THIGH_X_fr_LH_SHANK(state);
    frcTransf -> fr_LH_HIP_X_fr_LH_THIGH(state);
    frcTransf -> fr_base_X_fr_LH_HIP(state);
    frcTransf -> fr_RF_SHANK_X_fr_RF_WHEEL_L(state);
    frcTransf -> fr_RF_THIGH_X_fr_RF_SHANK(state);
    frcTransf -> fr_RF_HIP_X_fr_RF_THIGH(state);
    frcTransf -> fr_base_X_fr_RF_HIP(state);
    frcTransf -> fr_LF_SHANK_X_fr_LF_WHEEL_L(state);
    frcTransf -> fr_LF_THIGH_X_fr_LF_SHANK(state);
    frcTransf -> fr_LF_HIP_X_fr_LF_THIGH(state);
    frcTransf -> fr_base_X_fr_LF_HIP(state);

    // Initializes the composite inertia tensors
    base_Ic = linkInertias.getTensor_base();
    LF_HIP_Ic = linkInertias.getTensor_LF_HIP();
    LF_THIGH_Ic = linkInertias.getTensor_LF_THIGH();
    LF_SHANK_Ic = linkInertias.getTensor_LF_SHANK();
    RF_HIP_Ic = linkInertias.getTensor_RF_HIP();
    RF_THIGH_Ic = linkInertias.getTensor_RF_THIGH();
    RF_SHANK_Ic = linkInertias.getTensor_RF_SHANK();
    LH_HIP_Ic = linkInertias.getTensor_LH_HIP();
    LH_THIGH_Ic = linkInertias.getTensor_LH_THIGH();
    LH_SHANK_Ic = linkInertias.getTensor_LH_SHANK();
    RH_HIP_Ic = linkInertias.getTensor_RH_HIP();
    RH_THIGH_Ic = linkInertias.getTensor_RH_THIGH();
    RH_SHANK_Ic = linkInertias.getTensor_RH_SHANK();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link RH_WHEEL_L:
    iit::rbd::transformInertia<Scalar>(RH_WHEEL_L_Ic, frcTransf -> fr_RH_SHANK_X_fr_RH_WHEEL_L, Ic_spare);
    RH_SHANK_Ic += Ic_spare;

    Fcol(RH_WHEEL) = RH_WHEEL_L_Ic.col(iit::rbd::AZ);
    DATA(RH_WHEEL+6, RH_WHEEL+6) = Fcol(RH_WHEEL)(iit::rbd::AZ);

    Fcol(RH_WHEEL) = frcTransf -> fr_RH_SHANK_X_fr_RH_WHEEL_L * Fcol(RH_WHEEL);
    DATA(RH_WHEEL+6, RH_KFE+6) = F(iit::rbd::AZ,RH_WHEEL);
    DATA(RH_KFE+6, RH_WHEEL+6) = DATA(RH_WHEEL+6, RH_KFE+6);
    Fcol(RH_WHEEL) = frcTransf -> fr_RH_THIGH_X_fr_RH_SHANK * Fcol(RH_WHEEL);
    DATA(RH_WHEEL+6, RH_HFE+6) = F(iit::rbd::AZ,RH_WHEEL);
    DATA(RH_HFE+6, RH_WHEEL+6) = DATA(RH_WHEEL+6, RH_HFE+6);
    Fcol(RH_WHEEL) = frcTransf -> fr_RH_HIP_X_fr_RH_THIGH * Fcol(RH_WHEEL);
    DATA(RH_WHEEL+6, RH_HAA+6) = F(iit::rbd::AZ,RH_WHEEL);
    DATA(RH_HAA+6, RH_WHEEL+6) = DATA(RH_WHEEL+6, RH_HAA+6);
    Fcol(RH_WHEEL) = frcTransf -> fr_base_X_fr_RH_HIP * Fcol(RH_WHEEL);

    // Link RH_SHANK:
    iit::rbd::transformInertia<Scalar>(RH_SHANK_Ic, frcTransf -> fr_RH_THIGH_X_fr_RH_SHANK, Ic_spare);
    RH_THIGH_Ic += Ic_spare;

    Fcol(RH_KFE) = RH_SHANK_Ic.col(iit::rbd::AZ);
    DATA(RH_KFE+6, RH_KFE+6) = Fcol(RH_KFE)(iit::rbd::AZ);

    Fcol(RH_KFE) = frcTransf -> fr_RH_THIGH_X_fr_RH_SHANK * Fcol(RH_KFE);
    DATA(RH_KFE+6, RH_HFE+6) = F(iit::rbd::AZ,RH_KFE);
    DATA(RH_HFE+6, RH_KFE+6) = DATA(RH_KFE+6, RH_HFE+6);
    Fcol(RH_KFE) = frcTransf -> fr_RH_HIP_X_fr_RH_THIGH * Fcol(RH_KFE);
    DATA(RH_KFE+6, RH_HAA+6) = F(iit::rbd::AZ,RH_KFE);
    DATA(RH_HAA+6, RH_KFE+6) = DATA(RH_KFE+6, RH_HAA+6);
    Fcol(RH_KFE) = frcTransf -> fr_base_X_fr_RH_HIP * Fcol(RH_KFE);

    // Link RH_THIGH:
    iit::rbd::transformInertia<Scalar>(RH_THIGH_Ic, frcTransf -> fr_RH_HIP_X_fr_RH_THIGH, Ic_spare);
    RH_HIP_Ic += Ic_spare;

    Fcol(RH_HFE) = RH_THIGH_Ic.col(iit::rbd::AZ);
    DATA(RH_HFE+6, RH_HFE+6) = Fcol(RH_HFE)(iit::rbd::AZ);

    Fcol(RH_HFE) = frcTransf -> fr_RH_HIP_X_fr_RH_THIGH * Fcol(RH_HFE);
    DATA(RH_HFE+6, RH_HAA+6) = F(iit::rbd::AZ,RH_HFE);
    DATA(RH_HAA+6, RH_HFE+6) = DATA(RH_HFE+6, RH_HAA+6);
    Fcol(RH_HFE) = frcTransf -> fr_base_X_fr_RH_HIP * Fcol(RH_HFE);

    // Link RH_HIP:
    iit::rbd::transformInertia<Scalar>(RH_HIP_Ic, frcTransf -> fr_base_X_fr_RH_HIP, Ic_spare);
    base_Ic += Ic_spare;

    Fcol(RH_HAA) = RH_HIP_Ic.col(iit::rbd::AZ);
    DATA(RH_HAA+6, RH_HAA+6) = Fcol(RH_HAA)(iit::rbd::AZ);

    Fcol(RH_HAA) = frcTransf -> fr_base_X_fr_RH_HIP * Fcol(RH_HAA);

    // Link LH_WHEEL_L:
    iit::rbd::transformInertia<Scalar>(LH_WHEEL_L_Ic, frcTransf -> fr_LH_SHANK_X_fr_LH_WHEEL_L, Ic_spare);
    LH_SHANK_Ic += Ic_spare;

    Fcol(LH_WHEEL) = LH_WHEEL_L_Ic.col(iit::rbd::AZ);
    DATA(LH_WHEEL+6, LH_WHEEL+6) = Fcol(LH_WHEEL)(iit::rbd::AZ);

    Fcol(LH_WHEEL) = frcTransf -> fr_LH_SHANK_X_fr_LH_WHEEL_L * Fcol(LH_WHEEL);
    DATA(LH_WHEEL+6, LH_KFE+6) = F(iit::rbd::AZ,LH_WHEEL);
    DATA(LH_KFE+6, LH_WHEEL+6) = DATA(LH_WHEEL+6, LH_KFE+6);
    Fcol(LH_WHEEL) = frcTransf -> fr_LH_THIGH_X_fr_LH_SHANK * Fcol(LH_WHEEL);
    DATA(LH_WHEEL+6, LH_HFE+6) = F(iit::rbd::AZ,LH_WHEEL);
    DATA(LH_HFE+6, LH_WHEEL+6) = DATA(LH_WHEEL+6, LH_HFE+6);
    Fcol(LH_WHEEL) = frcTransf -> fr_LH_HIP_X_fr_LH_THIGH * Fcol(LH_WHEEL);
    DATA(LH_WHEEL+6, LH_HAA+6) = F(iit::rbd::AZ,LH_WHEEL);
    DATA(LH_HAA+6, LH_WHEEL+6) = DATA(LH_WHEEL+6, LH_HAA+6);
    Fcol(LH_WHEEL) = frcTransf -> fr_base_X_fr_LH_HIP * Fcol(LH_WHEEL);

    // Link LH_SHANK:
    iit::rbd::transformInertia<Scalar>(LH_SHANK_Ic, frcTransf -> fr_LH_THIGH_X_fr_LH_SHANK, Ic_spare);
    LH_THIGH_Ic += Ic_spare;

    Fcol(LH_KFE) = LH_SHANK_Ic.col(iit::rbd::AZ);
    DATA(LH_KFE+6, LH_KFE+6) = Fcol(LH_KFE)(iit::rbd::AZ);

    Fcol(LH_KFE) = frcTransf -> fr_LH_THIGH_X_fr_LH_SHANK * Fcol(LH_KFE);
    DATA(LH_KFE+6, LH_HFE+6) = F(iit::rbd::AZ,LH_KFE);
    DATA(LH_HFE+6, LH_KFE+6) = DATA(LH_KFE+6, LH_HFE+6);
    Fcol(LH_KFE) = frcTransf -> fr_LH_HIP_X_fr_LH_THIGH * Fcol(LH_KFE);
    DATA(LH_KFE+6, LH_HAA+6) = F(iit::rbd::AZ,LH_KFE);
    DATA(LH_HAA+6, LH_KFE+6) = DATA(LH_KFE+6, LH_HAA+6);
    Fcol(LH_KFE) = frcTransf -> fr_base_X_fr_LH_HIP * Fcol(LH_KFE);

    // Link LH_THIGH:
    iit::rbd::transformInertia<Scalar>(LH_THIGH_Ic, frcTransf -> fr_LH_HIP_X_fr_LH_THIGH, Ic_spare);
    LH_HIP_Ic += Ic_spare;

    Fcol(LH_HFE) = LH_THIGH_Ic.col(iit::rbd::AZ);
    DATA(LH_HFE+6, LH_HFE+6) = Fcol(LH_HFE)(iit::rbd::AZ);

    Fcol(LH_HFE) = frcTransf -> fr_LH_HIP_X_fr_LH_THIGH * Fcol(LH_HFE);
    DATA(LH_HFE+6, LH_HAA+6) = F(iit::rbd::AZ,LH_HFE);
    DATA(LH_HAA+6, LH_HFE+6) = DATA(LH_HFE+6, LH_HAA+6);
    Fcol(LH_HFE) = frcTransf -> fr_base_X_fr_LH_HIP * Fcol(LH_HFE);

    // Link LH_HIP:
    iit::rbd::transformInertia<Scalar>(LH_HIP_Ic, frcTransf -> fr_base_X_fr_LH_HIP, Ic_spare);
    base_Ic += Ic_spare;

    Fcol(LH_HAA) = LH_HIP_Ic.col(iit::rbd::AZ);
    DATA(LH_HAA+6, LH_HAA+6) = Fcol(LH_HAA)(iit::rbd::AZ);

    Fcol(LH_HAA) = frcTransf -> fr_base_X_fr_LH_HIP * Fcol(LH_HAA);

    // Link RF_WHEEL_L:
    iit::rbd::transformInertia<Scalar>(RF_WHEEL_L_Ic, frcTransf -> fr_RF_SHANK_X_fr_RF_WHEEL_L, Ic_spare);
    RF_SHANK_Ic += Ic_spare;

    Fcol(RF_WHEEL) = RF_WHEEL_L_Ic.col(iit::rbd::AZ);
    DATA(RF_WHEEL+6, RF_WHEEL+6) = Fcol(RF_WHEEL)(iit::rbd::AZ);

    Fcol(RF_WHEEL) = frcTransf -> fr_RF_SHANK_X_fr_RF_WHEEL_L * Fcol(RF_WHEEL);
    DATA(RF_WHEEL+6, RF_KFE+6) = F(iit::rbd::AZ,RF_WHEEL);
    DATA(RF_KFE+6, RF_WHEEL+6) = DATA(RF_WHEEL+6, RF_KFE+6);
    Fcol(RF_WHEEL) = frcTransf -> fr_RF_THIGH_X_fr_RF_SHANK * Fcol(RF_WHEEL);
    DATA(RF_WHEEL+6, RF_HFE+6) = F(iit::rbd::AZ,RF_WHEEL);
    DATA(RF_HFE+6, RF_WHEEL+6) = DATA(RF_WHEEL+6, RF_HFE+6);
    Fcol(RF_WHEEL) = frcTransf -> fr_RF_HIP_X_fr_RF_THIGH * Fcol(RF_WHEEL);
    DATA(RF_WHEEL+6, RF_HAA+6) = F(iit::rbd::AZ,RF_WHEEL);
    DATA(RF_HAA+6, RF_WHEEL+6) = DATA(RF_WHEEL+6, RF_HAA+6);
    Fcol(RF_WHEEL) = frcTransf -> fr_base_X_fr_RF_HIP * Fcol(RF_WHEEL);

    // Link RF_SHANK:
    iit::rbd::transformInertia<Scalar>(RF_SHANK_Ic, frcTransf -> fr_RF_THIGH_X_fr_RF_SHANK, Ic_spare);
    RF_THIGH_Ic += Ic_spare;

    Fcol(RF_KFE) = RF_SHANK_Ic.col(iit::rbd::AZ);
    DATA(RF_KFE+6, RF_KFE+6) = Fcol(RF_KFE)(iit::rbd::AZ);

    Fcol(RF_KFE) = frcTransf -> fr_RF_THIGH_X_fr_RF_SHANK * Fcol(RF_KFE);
    DATA(RF_KFE+6, RF_HFE+6) = F(iit::rbd::AZ,RF_KFE);
    DATA(RF_HFE+6, RF_KFE+6) = DATA(RF_KFE+6, RF_HFE+6);
    Fcol(RF_KFE) = frcTransf -> fr_RF_HIP_X_fr_RF_THIGH * Fcol(RF_KFE);
    DATA(RF_KFE+6, RF_HAA+6) = F(iit::rbd::AZ,RF_KFE);
    DATA(RF_HAA+6, RF_KFE+6) = DATA(RF_KFE+6, RF_HAA+6);
    Fcol(RF_KFE) = frcTransf -> fr_base_X_fr_RF_HIP * Fcol(RF_KFE);

    // Link RF_THIGH:
    iit::rbd::transformInertia<Scalar>(RF_THIGH_Ic, frcTransf -> fr_RF_HIP_X_fr_RF_THIGH, Ic_spare);
    RF_HIP_Ic += Ic_spare;

    Fcol(RF_HFE) = RF_THIGH_Ic.col(iit::rbd::AZ);
    DATA(RF_HFE+6, RF_HFE+6) = Fcol(RF_HFE)(iit::rbd::AZ);

    Fcol(RF_HFE) = frcTransf -> fr_RF_HIP_X_fr_RF_THIGH * Fcol(RF_HFE);
    DATA(RF_HFE+6, RF_HAA+6) = F(iit::rbd::AZ,RF_HFE);
    DATA(RF_HAA+6, RF_HFE+6) = DATA(RF_HFE+6, RF_HAA+6);
    Fcol(RF_HFE) = frcTransf -> fr_base_X_fr_RF_HIP * Fcol(RF_HFE);

    // Link RF_HIP:
    iit::rbd::transformInertia<Scalar>(RF_HIP_Ic, frcTransf -> fr_base_X_fr_RF_HIP, Ic_spare);
    base_Ic += Ic_spare;

    Fcol(RF_HAA) = RF_HIP_Ic.col(iit::rbd::AZ);
    DATA(RF_HAA+6, RF_HAA+6) = Fcol(RF_HAA)(iit::rbd::AZ);

    Fcol(RF_HAA) = frcTransf -> fr_base_X_fr_RF_HIP * Fcol(RF_HAA);

    // Link LF_WHEEL_L:
    iit::rbd::transformInertia<Scalar>(LF_WHEEL_L_Ic, frcTransf -> fr_LF_SHANK_X_fr_LF_WHEEL_L, Ic_spare);
    LF_SHANK_Ic += Ic_spare;

    Fcol(LF_WHEEL) = LF_WHEEL_L_Ic.col(iit::rbd::AZ);
    DATA(LF_WHEEL+6, LF_WHEEL+6) = Fcol(LF_WHEEL)(iit::rbd::AZ);

    Fcol(LF_WHEEL) = frcTransf -> fr_LF_SHANK_X_fr_LF_WHEEL_L * Fcol(LF_WHEEL);
    DATA(LF_WHEEL+6, LF_KFE+6) = F(iit::rbd::AZ,LF_WHEEL);
    DATA(LF_KFE+6, LF_WHEEL+6) = DATA(LF_WHEEL+6, LF_KFE+6);
    Fcol(LF_WHEEL) = frcTransf -> fr_LF_THIGH_X_fr_LF_SHANK * Fcol(LF_WHEEL);
    DATA(LF_WHEEL+6, LF_HFE+6) = F(iit::rbd::AZ,LF_WHEEL);
    DATA(LF_HFE+6, LF_WHEEL+6) = DATA(LF_WHEEL+6, LF_HFE+6);
    Fcol(LF_WHEEL) = frcTransf -> fr_LF_HIP_X_fr_LF_THIGH * Fcol(LF_WHEEL);
    DATA(LF_WHEEL+6, LF_HAA+6) = F(iit::rbd::AZ,LF_WHEEL);
    DATA(LF_HAA+6, LF_WHEEL+6) = DATA(LF_WHEEL+6, LF_HAA+6);
    Fcol(LF_WHEEL) = frcTransf -> fr_base_X_fr_LF_HIP * Fcol(LF_WHEEL);

    // Link LF_SHANK:
    iit::rbd::transformInertia<Scalar>(LF_SHANK_Ic, frcTransf -> fr_LF_THIGH_X_fr_LF_SHANK, Ic_spare);
    LF_THIGH_Ic += Ic_spare;

    Fcol(LF_KFE) = LF_SHANK_Ic.col(iit::rbd::AZ);
    DATA(LF_KFE+6, LF_KFE+6) = Fcol(LF_KFE)(iit::rbd::AZ);

    Fcol(LF_KFE) = frcTransf -> fr_LF_THIGH_X_fr_LF_SHANK * Fcol(LF_KFE);
    DATA(LF_KFE+6, LF_HFE+6) = F(iit::rbd::AZ,LF_KFE);
    DATA(LF_HFE+6, LF_KFE+6) = DATA(LF_KFE+6, LF_HFE+6);
    Fcol(LF_KFE) = frcTransf -> fr_LF_HIP_X_fr_LF_THIGH * Fcol(LF_KFE);
    DATA(LF_KFE+6, LF_HAA+6) = F(iit::rbd::AZ,LF_KFE);
    DATA(LF_HAA+6, LF_KFE+6) = DATA(LF_KFE+6, LF_HAA+6);
    Fcol(LF_KFE) = frcTransf -> fr_base_X_fr_LF_HIP * Fcol(LF_KFE);

    // Link LF_THIGH:
    iit::rbd::transformInertia<Scalar>(LF_THIGH_Ic, frcTransf -> fr_LF_HIP_X_fr_LF_THIGH, Ic_spare);
    LF_HIP_Ic += Ic_spare;

    Fcol(LF_HFE) = LF_THIGH_Ic.col(iit::rbd::AZ);
    DATA(LF_HFE+6, LF_HFE+6) = Fcol(LF_HFE)(iit::rbd::AZ);

    Fcol(LF_HFE) = frcTransf -> fr_LF_HIP_X_fr_LF_THIGH * Fcol(LF_HFE);
    DATA(LF_HFE+6, LF_HAA+6) = F(iit::rbd::AZ,LF_HFE);
    DATA(LF_HAA+6, LF_HFE+6) = DATA(LF_HFE+6, LF_HAA+6);
    Fcol(LF_HFE) = frcTransf -> fr_base_X_fr_LF_HIP * Fcol(LF_HFE);

    // Link LF_HIP:
    iit::rbd::transformInertia<Scalar>(LF_HIP_Ic, frcTransf -> fr_base_X_fr_LF_HIP, Ic_spare);
    base_Ic += Ic_spare;

    Fcol(LF_HAA) = LF_HIP_Ic.col(iit::rbd::AZ);
    DATA(LF_HAA+6, LF_HAA+6) = Fcol(LF_HAA)(iit::rbd::AZ);

    Fcol(LF_HAA) = frcTransf -> fr_base_X_fr_LF_HIP * Fcol(LF_HAA);

    // Copies the upper-right block into the lower-left block, after transposing
    JSIM<TRAIT>:: template block<16, 6>(6,0) = (JSIM<TRAIT>:: template block<6, 16>(0,6)).transpose();
    // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
    JSIM<TRAIT>:: template block<6,6>(0,0) = base_Ic;
    return *this;
}

#undef DATA
#undef F

template <typename TRAIT>
void iit::ANYmal::dyn::tpl::JSIM<TRAIT>::computeL() {
    L = this -> template triangularView<Eigen::Lower>();
    // Joint RH_WHEEL, index 15 :
    L(15, 15) = std::sqrt(L(15, 15));
    L(15, 14) = L(15, 14) / L(15, 15);
    L(15, 13) = L(15, 13) / L(15, 15);
    L(15, 12) = L(15, 12) / L(15, 15);
    L(14, 14) = L(14, 14) - L(15, 14) * L(15, 14);
    L(14, 13) = L(14, 13) - L(15, 14) * L(15, 13);
    L(14, 12) = L(14, 12) - L(15, 14) * L(15, 12);
    L(13, 13) = L(13, 13) - L(15, 13) * L(15, 13);
    L(13, 12) = L(13, 12) - L(15, 13) * L(15, 12);
    L(12, 12) = L(12, 12) - L(15, 12) * L(15, 12);
    
    // Joint RH_KFE, index 14 :
    L(14, 14) = std::sqrt(L(14, 14));
    L(14, 13) = L(14, 13) / L(14, 14);
    L(14, 12) = L(14, 12) / L(14, 14);
    L(13, 13) = L(13, 13) - L(14, 13) * L(14, 13);
    L(13, 12) = L(13, 12) - L(14, 13) * L(14, 12);
    L(12, 12) = L(12, 12) - L(14, 12) * L(14, 12);
    
    // Joint RH_HFE, index 13 :
    L(13, 13) = std::sqrt(L(13, 13));
    L(13, 12) = L(13, 12) / L(13, 13);
    L(12, 12) = L(12, 12) - L(13, 12) * L(13, 12);
    
    // Joint RH_HAA, index 12 :
    L(12, 12) = std::sqrt(L(12, 12));
    
    // Joint LH_WHEEL, index 11 :
    L(11, 11) = std::sqrt(L(11, 11));
    L(11, 10) = L(11, 10) / L(11, 11);
    L(11, 9) = L(11, 9) / L(11, 11);
    L(11, 8) = L(11, 8) / L(11, 11);
    L(10, 10) = L(10, 10) - L(11, 10) * L(11, 10);
    L(10, 9) = L(10, 9) - L(11, 10) * L(11, 9);
    L(10, 8) = L(10, 8) - L(11, 10) * L(11, 8);
    L(9, 9) = L(9, 9) - L(11, 9) * L(11, 9);
    L(9, 8) = L(9, 8) - L(11, 9) * L(11, 8);
    L(8, 8) = L(8, 8) - L(11, 8) * L(11, 8);
    
    // Joint LH_KFE, index 10 :
    L(10, 10) = std::sqrt(L(10, 10));
    L(10, 9) = L(10, 9) / L(10, 10);
    L(10, 8) = L(10, 8) / L(10, 10);
    L(9, 9) = L(9, 9) - L(10, 9) * L(10, 9);
    L(9, 8) = L(9, 8) - L(10, 9) * L(10, 8);
    L(8, 8) = L(8, 8) - L(10, 8) * L(10, 8);
    
    // Joint LH_HFE, index 9 :
    L(9, 9) = std::sqrt(L(9, 9));
    L(9, 8) = L(9, 8) / L(9, 9);
    L(8, 8) = L(8, 8) - L(9, 8) * L(9, 8);
    
    // Joint LH_HAA, index 8 :
    L(8, 8) = std::sqrt(L(8, 8));
    
    // Joint RF_WHEEL, index 7 :
    L(7, 7) = std::sqrt(L(7, 7));
    L(7, 6) = L(7, 6) / L(7, 7);
    L(7, 5) = L(7, 5) / L(7, 7);
    L(7, 4) = L(7, 4) / L(7, 7);
    L(6, 6) = L(6, 6) - L(7, 6) * L(7, 6);
    L(6, 5) = L(6, 5) - L(7, 6) * L(7, 5);
    L(6, 4) = L(6, 4) - L(7, 6) * L(7, 4);
    L(5, 5) = L(5, 5) - L(7, 5) * L(7, 5);
    L(5, 4) = L(5, 4) - L(7, 5) * L(7, 4);
    L(4, 4) = L(4, 4) - L(7, 4) * L(7, 4);
    
    // Joint RF_KFE, index 6 :
    L(6, 6) = std::sqrt(L(6, 6));
    L(6, 5) = L(6, 5) / L(6, 6);
    L(6, 4) = L(6, 4) / L(6, 6);
    L(5, 5) = L(5, 5) - L(6, 5) * L(6, 5);
    L(5, 4) = L(5, 4) - L(6, 5) * L(6, 4);
    L(4, 4) = L(4, 4) - L(6, 4) * L(6, 4);
    
    // Joint RF_HFE, index 5 :
    L(5, 5) = std::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    
    // Joint RF_HAA, index 4 :
    L(4, 4) = std::sqrt(L(4, 4));
    
    // Joint LF_WHEEL, index 3 :
    L(3, 3) = std::sqrt(L(3, 3));
    L(3, 2) = L(3, 2) / L(3, 3);
    L(3, 1) = L(3, 1) / L(3, 3);
    L(3, 0) = L(3, 0) / L(3, 3);
    L(2, 2) = L(2, 2) - L(3, 2) * L(3, 2);
    L(2, 1) = L(2, 1) - L(3, 2) * L(3, 1);
    L(2, 0) = L(2, 0) - L(3, 2) * L(3, 0);
    L(1, 1) = L(1, 1) - L(3, 1) * L(3, 1);
    L(1, 0) = L(1, 0) - L(3, 1) * L(3, 0);
    L(0, 0) = L(0, 0) - L(3, 0) * L(3, 0);
    
    // Joint LF_KFE, index 2 :
    L(2, 2) = std::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);
    
    // Joint LF_HFE, index 1 :
    L(1, 1) = std::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint LF_HAA, index 0 :
    L(0, 0) = std::sqrt(L(0, 0));
    
}

template <typename TRAIT>
void iit::ANYmal::dyn::tpl::JSIM<TRAIT>::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) =  + (Linv(2, 0) * Linv(2, 0)) + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 1) =  + (Linv(2, 0) * Linv(1, 0)) + (Linv(2, 1) * Linv(1, 1));
    inverse(1, 2) = inverse(2, 1);
    inverse(2, 0) =  + (Linv(2, 0) * Linv(0, 0));
    inverse(0, 2) = inverse(2, 0);
    inverse(3, 3) =  + (Linv(3, 0) * Linv(3, 0)) + (Linv(3, 1) * Linv(3, 1)) + (Linv(3, 2) * Linv(3, 2)) + (Linv(3, 3) * Linv(3, 3));
    inverse(3, 2) =  + (Linv(3, 0) * Linv(2, 0)) + (Linv(3, 1) * Linv(2, 1)) + (Linv(3, 2) * Linv(2, 2));
    inverse(2, 3) = inverse(3, 2);
    inverse(3, 1) =  + (Linv(3, 0) * Linv(1, 0)) + (Linv(3, 1) * Linv(1, 1));
    inverse(1, 3) = inverse(3, 1);
    inverse(3, 0) =  + (Linv(3, 0) * Linv(0, 0));
    inverse(0, 3) = inverse(3, 0);
    inverse(4, 4) =  + (Linv(4, 4) * Linv(4, 4));
    inverse(5, 5) =  + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(6, 6) =  + (Linv(6, 4) * Linv(6, 4)) + (Linv(6, 5) * Linv(6, 5)) + (Linv(6, 6) * Linv(6, 6));
    inverse(6, 5) =  + (Linv(6, 4) * Linv(5, 4)) + (Linv(6, 5) * Linv(5, 5));
    inverse(5, 6) = inverse(6, 5);
    inverse(6, 4) =  + (Linv(6, 4) * Linv(4, 4));
    inverse(4, 6) = inverse(6, 4);
    inverse(7, 7) =  + (Linv(7, 4) * Linv(7, 4)) + (Linv(7, 5) * Linv(7, 5)) + (Linv(7, 6) * Linv(7, 6)) + (Linv(7, 7) * Linv(7, 7));
    inverse(7, 6) =  + (Linv(7, 4) * Linv(6, 4)) + (Linv(7, 5) * Linv(6, 5)) + (Linv(7, 6) * Linv(6, 6));
    inverse(6, 7) = inverse(7, 6);
    inverse(7, 5) =  + (Linv(7, 4) * Linv(5, 4)) + (Linv(7, 5) * Linv(5, 5));
    inverse(5, 7) = inverse(7, 5);
    inverse(7, 4) =  + (Linv(7, 4) * Linv(4, 4));
    inverse(4, 7) = inverse(7, 4);
    inverse(8, 8) =  + (Linv(8, 8) * Linv(8, 8));
    inverse(9, 9) =  + (Linv(9, 8) * Linv(9, 8)) + (Linv(9, 9) * Linv(9, 9));
    inverse(9, 8) =  + (Linv(9, 8) * Linv(8, 8));
    inverse(8, 9) = inverse(9, 8);
    inverse(10, 10) =  + (Linv(10, 8) * Linv(10, 8)) + (Linv(10, 9) * Linv(10, 9)) + (Linv(10, 10) * Linv(10, 10));
    inverse(10, 9) =  + (Linv(10, 8) * Linv(9, 8)) + (Linv(10, 9) * Linv(9, 9));
    inverse(9, 10) = inverse(10, 9);
    inverse(10, 8) =  + (Linv(10, 8) * Linv(8, 8));
    inverse(8, 10) = inverse(10, 8);
    inverse(11, 11) =  + (Linv(11, 8) * Linv(11, 8)) + (Linv(11, 9) * Linv(11, 9)) + (Linv(11, 10) * Linv(11, 10)) + (Linv(11, 11) * Linv(11, 11));
    inverse(11, 10) =  + (Linv(11, 8) * Linv(10, 8)) + (Linv(11, 9) * Linv(10, 9)) + (Linv(11, 10) * Linv(10, 10));
    inverse(10, 11) = inverse(11, 10);
    inverse(11, 9) =  + (Linv(11, 8) * Linv(9, 8)) + (Linv(11, 9) * Linv(9, 9));
    inverse(9, 11) = inverse(11, 9);
    inverse(11, 8) =  + (Linv(11, 8) * Linv(8, 8));
    inverse(8, 11) = inverse(11, 8);
    inverse(12, 12) =  + (Linv(12, 12) * Linv(12, 12));
    inverse(13, 13) =  + (Linv(13, 12) * Linv(13, 12)) + (Linv(13, 13) * Linv(13, 13));
    inverse(13, 12) =  + (Linv(13, 12) * Linv(12, 12));
    inverse(12, 13) = inverse(13, 12);
    inverse(14, 14) =  + (Linv(14, 12) * Linv(14, 12)) + (Linv(14, 13) * Linv(14, 13)) + (Linv(14, 14) * Linv(14, 14));
    inverse(14, 13) =  + (Linv(14, 12) * Linv(13, 12)) + (Linv(14, 13) * Linv(13, 13));
    inverse(13, 14) = inverse(14, 13);
    inverse(14, 12) =  + (Linv(14, 12) * Linv(12, 12));
    inverse(12, 14) = inverse(14, 12);
    inverse(15, 15) =  + (Linv(15, 12) * Linv(15, 12)) + (Linv(15, 13) * Linv(15, 13)) + (Linv(15, 14) * Linv(15, 14)) + (Linv(15, 15) * Linv(15, 15));
    inverse(15, 14) =  + (Linv(15, 12) * Linv(14, 12)) + (Linv(15, 13) * Linv(14, 13)) + (Linv(15, 14) * Linv(14, 14));
    inverse(14, 15) = inverse(15, 14);
    inverse(15, 13) =  + (Linv(15, 12) * Linv(13, 12)) + (Linv(15, 13) * Linv(13, 13));
    inverse(13, 15) = inverse(15, 13);
    inverse(15, 12) =  + (Linv(15, 12) * Linv(12, 12));
    inverse(12, 15) = inverse(15, 12);
}

template <typename TRAIT>
void iit::ANYmal::dyn::tpl::JSIM<TRAIT>::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(6, 6) = 1 / L(6, 6);
    Linv(7, 7) = 1 / L(7, 7);
    Linv(8, 8) = 1 / L(8, 8);
    Linv(9, 9) = 1 / L(9, 9);
    Linv(10, 10) = 1 / L(10, 10);
    Linv(11, 11) = 1 / L(11, 11);
    Linv(12, 12) = 1 / L(12, 12);
    Linv(13, 13) = 1 / L(13, 13);
    Linv(14, 14) = 1 / L(14, 14);
    Linv(15, 15) = 1 / L(15, 15);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = - Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
    Linv(3, 2) = - Linv(2, 2) * ((Linv(3, 3) * L(3, 2)) + 0);
    Linv(3, 1) = - Linv(1, 1) * ((Linv(3, 2) * L(2, 1)) + (Linv(3, 3) * L(3, 1)) + 0);
    Linv(3, 0) = - Linv(0, 0) * ((Linv(3, 1) * L(1, 0)) + (Linv(3, 2) * L(2, 0)) + (Linv(3, 3) * L(3, 0)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(6, 5) = - Linv(5, 5) * ((Linv(6, 6) * L(6, 5)) + 0);
    Linv(6, 4) = - Linv(4, 4) * ((Linv(6, 5) * L(5, 4)) + (Linv(6, 6) * L(6, 4)) + 0);
    Linv(7, 6) = - Linv(6, 6) * ((Linv(7, 7) * L(7, 6)) + 0);
    Linv(7, 5) = - Linv(5, 5) * ((Linv(7, 6) * L(6, 5)) + (Linv(7, 7) * L(7, 5)) + 0);
    Linv(7, 4) = - Linv(4, 4) * ((Linv(7, 5) * L(5, 4)) + (Linv(7, 6) * L(6, 4)) + (Linv(7, 7) * L(7, 4)) + 0);
    Linv(9, 8) = - Linv(8, 8) * ((Linv(9, 9) * L(9, 8)) + 0);
    Linv(10, 9) = - Linv(9, 9) * ((Linv(10, 10) * L(10, 9)) + 0);
    Linv(10, 8) = - Linv(8, 8) * ((Linv(10, 9) * L(9, 8)) + (Linv(10, 10) * L(10, 8)) + 0);
    Linv(11, 10) = - Linv(10, 10) * ((Linv(11, 11) * L(11, 10)) + 0);
    Linv(11, 9) = - Linv(9, 9) * ((Linv(11, 10) * L(10, 9)) + (Linv(11, 11) * L(11, 9)) + 0);
    Linv(11, 8) = - Linv(8, 8) * ((Linv(11, 9) * L(9, 8)) + (Linv(11, 10) * L(10, 8)) + (Linv(11, 11) * L(11, 8)) + 0);
    Linv(13, 12) = - Linv(12, 12) * ((Linv(13, 13) * L(13, 12)) + 0);
    Linv(14, 13) = - Linv(13, 13) * ((Linv(14, 14) * L(14, 13)) + 0);
    Linv(14, 12) = - Linv(12, 12) * ((Linv(14, 13) * L(13, 12)) + (Linv(14, 14) * L(14, 12)) + 0);
    Linv(15, 14) = - Linv(14, 14) * ((Linv(15, 15) * L(15, 14)) + 0);
    Linv(15, 13) = - Linv(13, 13) * ((Linv(15, 14) * L(14, 13)) + (Linv(15, 15) * L(15, 13)) + 0);
    Linv(15, 12) = - Linv(12, 12) * ((Linv(15, 13) * L(13, 12)) + (Linv(15, 14) * L(14, 12)) + (Linv(15, 15) * L(15, 12)) + 0);
}

