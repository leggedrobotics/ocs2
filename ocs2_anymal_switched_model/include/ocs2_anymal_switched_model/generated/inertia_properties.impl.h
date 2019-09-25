template <typename TRAIT>
iit::ANYmal::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_base = iit::rbd::Vector3d(0.005256856,0.0023927626,0.042379685).cast<Scalar>();
    tensor_base.fill(
        Scalar(21.382084),
        com_base,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.2637037),
                Scalar(0.73665464),
                Scalar(0.6771133),
                Scalar(0.001452971),
                Scalar(0.024311887),
                Scalar(0.0038386881)) );

    com_LF_HIP = iit::rbd::Vector3d(0.00524232,0.0011793415,0.06562732).cast<Scalar>();
    tensor_LF_HIP.fill(
        Scalar(1.8601753),
        com_LF_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.010860444),
                Scalar(0.01137688),
                Scalar(0.0034593414),
                Scalar(1.06557025E-4),
                Scalar(6.78611E-4),
                Scalar(1.8497455E-4)) );

    com_LF_THIGH = iit::rbd::Vector3d(-0.006388454,0.22433881,0.058594376).cast<Scalar>();
    tensor_LF_THIGH.fill(
        Scalar(2.1196263),
        com_LF_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.12759717),
                Scalar(0.011414336),
                Scalar(0.12063005),
                Scalar(-0.0028066067),
                Scalar(-0.0010147165),
                Scalar(0.026277075)) );

    com_LF_SHANK = iit::rbd::Vector3d(0.058733236,0.09806424,-0.010824396).cast<Scalar>();
    tensor_LF_SHANK.fill(
        Scalar(0.34737507),
        com_LF_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0102394),
                Scalar(0.0022584267),
                Scalar(0.0122431815),
                Scalar(0.003429438),
                Scalar(-2.5307332E-4),
                Scalar(-6.849708E-4)) );

    com_RF_HIP = iit::rbd::Vector3d(0.00524232,-0.0011793415,0.06562732).cast<Scalar>();
    tensor_RF_HIP.fill(
        Scalar(1.8601753),
        com_RF_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.010860444),
                Scalar(0.01137688),
                Scalar(0.0034593414),
                Scalar(-1.06557025E-4),
                Scalar(6.78611E-4),
                Scalar(-1.8497455E-4)) );

    com_RF_THIGH = iit::rbd::Vector3d(-0.006388454,0.22433881,-0.058594376).cast<Scalar>();
    tensor_RF_THIGH.fill(
        Scalar(2.1196263),
        com_RF_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.12759717),
                Scalar(0.011414338),
                Scalar(0.12063005),
                Scalar(-0.0028066067),
                Scalar(0.0010147165),
                Scalar(-0.026277073)) );

    com_RF_SHANK = iit::rbd::Vector3d(0.058733236,0.09806424,0.010824394).cast<Scalar>();
    tensor_RF_SHANK.fill(
        Scalar(0.34737507),
        com_RF_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0102394),
                Scalar(0.0022584265),
                Scalar(0.0122431815),
                Scalar(0.003429438),
                Scalar(2.5307338E-4),
                Scalar(6.849712E-4)) );

    com_LH_HIP = iit::rbd::Vector3d(0.00524232,0.0011793415,-0.06562732).cast<Scalar>();
    tensor_LH_HIP.fill(
        Scalar(1.8601753),
        com_LH_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.010860444),
                Scalar(0.01137688),
                Scalar(0.0034593414),
                Scalar(1.0655703E-4),
                Scalar(-6.7861105E-4),
                Scalar(-1.8497455E-4)) );

    com_LH_THIGH = iit::rbd::Vector3d(0.006388454,0.22433881,0.058594376).cast<Scalar>();
    tensor_LH_THIGH.fill(
        Scalar(2.1196263),
        com_LH_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.12759717),
                Scalar(0.011414336),
                Scalar(0.12063005),
                Scalar(0.0028066067),
                Scalar(0.0010147165),
                Scalar(0.026277075)) );

    com_LH_SHANK = iit::rbd::Vector3d(-0.058733236,0.09806424,-0.010824396).cast<Scalar>();
    tensor_LH_SHANK.fill(
        Scalar(0.34737507),
        com_LH_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0102394),
                Scalar(0.0022584267),
                Scalar(0.0122431815),
                Scalar(-0.003429438),
                Scalar(2.5307332E-4),
                Scalar(-6.849708E-4)) );

    com_RH_HIP = iit::rbd::Vector3d(0.00524232,-0.0011793415,-0.06562732).cast<Scalar>();
    tensor_RH_HIP.fill(
        Scalar(1.8601753),
        com_RH_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.010860444),
                Scalar(0.01137688),
                Scalar(0.0034593414),
                Scalar(-1.0655703E-4),
                Scalar(-6.7861105E-4),
                Scalar(1.8497455E-4)) );

    com_RH_THIGH = iit::rbd::Vector3d(0.006388454,0.22433881,-0.058594376).cast<Scalar>();
    tensor_RH_THIGH.fill(
        Scalar(2.1196263),
        com_RH_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.12759717),
                Scalar(0.011414338),
                Scalar(0.12063005),
                Scalar(0.0028066067),
                Scalar(-0.0010147165),
                Scalar(-0.026277073)) );

    com_RH_SHANK = iit::rbd::Vector3d(-0.058733236,0.09806424,0.010824394).cast<Scalar>();
    tensor_RH_SHANK.fill(
        Scalar(0.34737507),
        com_RH_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0102394),
                Scalar(0.0022584265),
                Scalar(0.0122431815),
                Scalar(-0.003429438),
                Scalar(-2.5307338E-4),
                Scalar(6.849712E-4)) );

}

