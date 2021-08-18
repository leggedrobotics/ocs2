template <typename TRAIT>
iit::chip::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_base = iit::rbd::Vector3d(-0.018795313,0.0065398864,0.0077333865).cast<Scalar>();
    tensor_base.fill(
        Scalar(25.601385),
        com_base,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.17962514),
                Scalar(1.7094786),
                Scalar(1.7444946),
                Scalar(-0.0039466317),
                Scalar(-0.045441378),
                Scalar(0.003098703)) );

    com_LF_HIP = iit::rbd::Vector3d(-0.0010103208,0.014250419,0.068031415).cast<Scalar>();
    tensor_LF_HIP.fill(
        Scalar(1.8623549),
        com_LF_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.012983099),
                Scalar(0.012171697),
                Scalar(0.0032080302),
                Scalar(-9.041602E-5),
                Scalar(-6.910495E-5),
                Scalar(0.0016106762)) );

    com_LF_THIGH = iit::rbd::Vector3d(0.0049690404,0.2226652,0.028049987).cast<Scalar>();
    tensor_LF_THIGH.fill(
        Scalar(2.3501198),
        com_LF_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.1466014),
                Scalar(0.0058492566),
                Scalar(0.14499189),
                Scalar(-2.1081958E-4),
                Scalar(2.5706907E-4),
                Scalar(0.016769057)) );

    com_LF_SHANK = iit::rbd::Vector3d(0.08773683,0.07102133,0.014560207).cast<Scalar>();
    tensor_LF_SHANK.fill(
        Scalar(0.5264926),
        com_LF_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.010520823),
                Scalar(0.0076368405),
                Scalar(0.017700527),
                Scalar(0.0069414484),
                Scalar(4.3581615E-4),
                Scalar(2.2101759E-4)) );

    com_RF_HIP = iit::rbd::Vector3d(-0.0010103208,-0.014250419,0.06809383).cast<Scalar>();
    tensor_RF_HIP.fill(
        Scalar(1.8623549),
        com_RF_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.012997046),
                Scalar(0.012185644),
                Scalar(0.0032080302),
                Scalar(9.041602E-5),
                Scalar(-6.67173E-5),
                Scalar(-0.0016123619)) );

    com_RF_THIGH = iit::rbd::Vector3d(0.005016815,0.2226524,-0.028049987).cast<Scalar>();
    tensor_RF_THIGH.fill(
        Scalar(2.3501198),
        com_RF_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.1465832),
                Scalar(0.0058504506),
                Scalar(0.14497489),
                Scalar(-1.7701977E-4),
                Scalar(-2.6058225E-4),
                Scalar(-0.016768113)) );

    com_RF_SHANK = iit::rbd::Vector3d(0.08773683,0.07102133,-0.014560207).cast<Scalar>();
    tensor_RF_SHANK.fill(
        Scalar(0.5264926),
        com_RF_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.010520823),
                Scalar(0.0076368405),
                Scalar(0.017700527),
                Scalar(0.0069414484),
                Scalar(-4.3581583E-4),
                Scalar(-2.2101695E-4)) );

    com_LH_HIP = iit::rbd::Vector3d(-0.0010103208,0.014250419,-0.06809383).cast<Scalar>();
    tensor_LH_HIP.fill(
        Scalar(1.8623549),
        com_LH_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.012997045),
                Scalar(0.012185645),
                Scalar(0.0032080302),
                Scalar(-9.0416055E-5),
                Scalar(6.6717264E-5),
                Scalar(-0.001612362)) );

    com_LH_THIGH = iit::rbd::Vector3d(-0.005016815,0.2226524,0.028049987).cast<Scalar>();
    tensor_LH_THIGH.fill(
        Scalar(2.3501198),
        com_LH_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.14658318),
                Scalar(0.0058504506),
                Scalar(0.14497489),
                Scalar(1.7701971E-4),
                Scalar(-2.6058196E-4),
                Scalar(0.016768115)) );

    com_LH_SHANK = iit::rbd::Vector3d(-0.08773683,0.07102133,0.014560207).cast<Scalar>();
    tensor_LH_SHANK.fill(
        Scalar(0.5264926),
        com_LH_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.010520823),
                Scalar(0.0076368405),
                Scalar(0.017700527),
                Scalar(-0.0069414484),
                Scalar(-4.35816E-4),
                Scalar(2.2101747E-4)) );

    com_RH_HIP = iit::rbd::Vector3d(-0.0010103208,-0.014250419,-0.068031415).cast<Scalar>();
    tensor_RH_HIP.fill(
        Scalar(1.8623549),
        com_RH_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.012983098),
                Scalar(0.012171698),
                Scalar(0.0032080302),
                Scalar(9.0416055E-5),
                Scalar(6.9104884E-5),
                Scalar(0.0016106762)) );

    com_RH_THIGH = iit::rbd::Vector3d(-0.0049690404,0.2226652,-0.028049987).cast<Scalar>();
    tensor_RH_THIGH.fill(
        Scalar(2.3501198),
        com_RH_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.14660141),
                Scalar(0.0058492566),
                Scalar(0.14499189),
                Scalar(2.1081972E-4),
                Scalar(2.5706922E-4),
                Scalar(-0.016769055)) );

    com_RH_SHANK = iit::rbd::Vector3d(-0.08773683,0.07102133,-0.014560207).cast<Scalar>();
    tensor_RH_SHANK.fill(
        Scalar(0.5264926),
        com_RH_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.010520823),
                Scalar(0.0076368405),
                Scalar(0.017700527),
                Scalar(-0.0069414484),
                Scalar(4.3581563E-4),
                Scalar(-2.2101676E-4)) );

}

