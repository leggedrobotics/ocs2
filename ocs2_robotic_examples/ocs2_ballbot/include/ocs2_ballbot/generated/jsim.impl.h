

//Implementation of default constructor
template <typename TRAIT>
iit::Ballbot::dyn::tpl::JSIM<TRAIT>::JSIM(IProperties& inertiaProperties, FTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    base_Ic(linkInertias.getTensor_base())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA tpl::JSIM<TRAIT>::operator()

template <typename TRAIT>
const typename iit::Ballbot::dyn::tpl::JSIM<TRAIT>& iit::Ballbot::dyn::tpl::JSIM<TRAIT>::update(const JointState& state) {
    ForceVector F;

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_dummy_base2_X_fr_base(state);
    frcTransf -> fr_dummy_base1_X_fr_dummy_base2(state);
    frcTransf -> fr_ball_X_fr_dummy_base1(state);
    frcTransf -> fr_dummy_ball1_X_fr_ball(state);

    // Initializes the composite inertia tensors
    dummy_ball1_Ic = linkInertias.getTensor_dummy_ball1();
    ball_Ic = linkInertias.getTensor_ball();
    dummy_base1_Ic = linkInertias.getTensor_dummy_base1();
    dummy_base2_Ic = linkInertias.getTensor_dummy_base2();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link base:
    iit::rbd::transformInertia<Scalar>(base_Ic, frcTransf -> fr_dummy_base2_X_fr_base, Ic_spare);
    dummy_base2_Ic += Ic_spare;

    F = base_Ic.col(iit::rbd::AZ);
    DATA(JBASE_X, JBASE_X) = F(iit::rbd::AZ);

    F = frcTransf -> fr_dummy_base2_X_fr_base * F;
    DATA(JBASE_X, JBASE_Y) = F(iit::rbd::AZ);
    DATA(JBASE_Y, JBASE_X) = DATA(JBASE_X, JBASE_Y);
    F = frcTransf -> fr_dummy_base1_X_fr_dummy_base2 * F;
    DATA(JBASE_X, JBASE_Z) = F(iit::rbd::AZ);
    DATA(JBASE_Z, JBASE_X) = DATA(JBASE_X, JBASE_Z);
    F = frcTransf -> fr_ball_X_fr_dummy_base1 * F;
    DATA(JBASE_X, JBALL_Y) = F(iit::rbd::LZ);
    DATA(JBALL_Y, JBASE_X) = DATA(JBASE_X, JBALL_Y);
    F = frcTransf -> fr_dummy_ball1_X_fr_ball * F;
    DATA(JBASE_X, JBALL_X) = F(iit::rbd::LZ);
    DATA(JBALL_X, JBASE_X) = DATA(JBASE_X, JBALL_X);

    // Link dummy_base2:
    iit::rbd::transformInertia<Scalar>(dummy_base2_Ic, frcTransf -> fr_dummy_base1_X_fr_dummy_base2, Ic_spare);
    dummy_base1_Ic += Ic_spare;

    F = dummy_base2_Ic.col(iit::rbd::AZ);
    DATA(JBASE_Y, JBASE_Y) = F(iit::rbd::AZ);

    F = frcTransf -> fr_dummy_base1_X_fr_dummy_base2 * F;
    DATA(JBASE_Y, JBASE_Z) = F(iit::rbd::AZ);
    DATA(JBASE_Z, JBASE_Y) = DATA(JBASE_Y, JBASE_Z);
    F = frcTransf -> fr_ball_X_fr_dummy_base1 * F;
    DATA(JBASE_Y, JBALL_Y) = F(iit::rbd::LZ);
    DATA(JBALL_Y, JBASE_Y) = DATA(JBASE_Y, JBALL_Y);
    F = frcTransf -> fr_dummy_ball1_X_fr_ball * F;
    DATA(JBASE_Y, JBALL_X) = F(iit::rbd::LZ);
    DATA(JBALL_X, JBASE_Y) = DATA(JBASE_Y, JBALL_X);

    // Link dummy_base1:
    iit::rbd::transformInertia<Scalar>(dummy_base1_Ic, frcTransf -> fr_ball_X_fr_dummy_base1, Ic_spare);
    ball_Ic += Ic_spare;

    F = dummy_base1_Ic.col(iit::rbd::AZ);
    DATA(JBASE_Z, JBASE_Z) = F(iit::rbd::AZ);

    F = frcTransf -> fr_ball_X_fr_dummy_base1 * F;
    DATA(JBASE_Z, JBALL_Y) = F(iit::rbd::LZ);
    DATA(JBALL_Y, JBASE_Z) = DATA(JBASE_Z, JBALL_Y);
    F = frcTransf -> fr_dummy_ball1_X_fr_ball * F;
    DATA(JBASE_Z, JBALL_X) = F(iit::rbd::LZ);
    DATA(JBALL_X, JBASE_Z) = DATA(JBASE_Z, JBALL_X);

    // Link ball:
    iit::rbd::transformInertia<Scalar>(ball_Ic, frcTransf -> fr_dummy_ball1_X_fr_ball, Ic_spare);
    dummy_ball1_Ic += Ic_spare;

    F = ball_Ic.col(iit::rbd::LZ);
    DATA(JBALL_Y, JBALL_Y) = F(iit::rbd::LZ);

    F = frcTransf -> fr_dummy_ball1_X_fr_ball * F;
    DATA(JBALL_Y, JBALL_X) = F(iit::rbd::LZ);
    DATA(JBALL_X, JBALL_Y) = DATA(JBALL_Y, JBALL_X);

    // Link dummy_ball1:

    F = dummy_ball1_Ic.col(iit::rbd::LZ);
    DATA(JBALL_X, JBALL_X) = F(iit::rbd::LZ);


    return *this;
}

#undef DATA
#undef F

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::JSIM<TRAIT>::computeL() {
    L = this -> template triangularView<Eigen::Lower>();
    // Joint jbase_x, index 4 :
    L(4, 4) = std::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(4, 2) = L(4, 2) / L(4, 4);
    L(4, 1) = L(4, 1) / L(4, 4);
    L(4, 0) = L(4, 0) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    L(3, 2) = L(3, 2) - L(4, 3) * L(4, 2);
    L(3, 1) = L(3, 1) - L(4, 3) * L(4, 1);
    L(3, 0) = L(3, 0) - L(4, 3) * L(4, 0);
    L(2, 2) = L(2, 2) - L(4, 2) * L(4, 2);
    L(2, 1) = L(2, 1) - L(4, 2) * L(4, 1);
    L(2, 0) = L(2, 0) - L(4, 2) * L(4, 0);
    L(1, 1) = L(1, 1) - L(4, 1) * L(4, 1);
    L(1, 0) = L(1, 0) - L(4, 1) * L(4, 0);
    L(0, 0) = L(0, 0) - L(4, 0) * L(4, 0);
    
    // Joint jbase_y, index 3 :
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
    
    // Joint jbase_z, index 2 :
    L(2, 2) = std::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);
    
    // Joint jball_y, index 1 :
    L(1, 1) = std::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint jball_x, index 0 :
    L(0, 0) = std::sqrt(L(0, 0));
    
}

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::JSIM<TRAIT>::computeInverse() {
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
    inverse(4, 4) =  + (Linv(4, 0) * Linv(4, 0)) + (Linv(4, 1) * Linv(4, 1)) + (Linv(4, 2) * Linv(4, 2)) + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 0) * Linv(3, 0)) + (Linv(4, 1) * Linv(3, 1)) + (Linv(4, 2) * Linv(3, 2)) + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(4, 2) =  + (Linv(4, 0) * Linv(2, 0)) + (Linv(4, 1) * Linv(2, 1)) + (Linv(4, 2) * Linv(2, 2));
    inverse(2, 4) = inverse(4, 2);
    inverse(4, 1) =  + (Linv(4, 0) * Linv(1, 0)) + (Linv(4, 1) * Linv(1, 1));
    inverse(1, 4) = inverse(4, 1);
    inverse(4, 0) =  + (Linv(4, 0) * Linv(0, 0));
    inverse(0, 4) = inverse(4, 0);
}

template <typename TRAIT>
void iit::Ballbot::dyn::tpl::JSIM<TRAIT>::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = - Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
    Linv(3, 2) = - Linv(2, 2) * ((Linv(3, 3) * L(3, 2)) + 0);
    Linv(3, 1) = - Linv(1, 1) * ((Linv(3, 2) * L(2, 1)) + (Linv(3, 3) * L(3, 1)) + 0);
    Linv(3, 0) = - Linv(0, 0) * ((Linv(3, 1) * L(1, 0)) + (Linv(3, 2) * L(2, 0)) + (Linv(3, 3) * L(3, 0)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(4, 2) = - Linv(2, 2) * ((Linv(4, 3) * L(3, 2)) + (Linv(4, 4) * L(4, 2)) + 0);
    Linv(4, 1) = - Linv(1, 1) * ((Linv(4, 2) * L(2, 1)) + (Linv(4, 3) * L(3, 1)) + (Linv(4, 4) * L(4, 1)) + 0);
    Linv(4, 0) = - Linv(0, 0) * ((Linv(4, 1) * L(1, 0)) + (Linv(4, 2) * L(2, 0)) + (Linv(4, 3) * L(3, 0)) + (Linv(4, 4) * L(4, 0)) + 0);
}

