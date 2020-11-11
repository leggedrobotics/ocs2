template <typename TRAIT>
iit::wheels::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_base = iit::rbd::Vector3d(-0.017639399,-6.8206404E-4,0.0056651644).cast<Scalar>();
    tensor_base.fill(
        Scalar(23.355284),
        com_base,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.17498831),
                Scalar(1.7133757),
                Scalar(1.7474247),
                Scalar(1.4063274E-4),
                Scalar(-0.042799395),
                Scalar(-0.001587435)) );

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

    com_LF_SHANK = iit::rbd::Vector3d(0.044662513,0.030045535,0.019255718).cast<Scalar>();
    tensor_LF_SHANK.fill(
        Scalar(0.41052794),
        com_LF_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0038513276),
                Scalar(0.002628087),
                Scalar(0.0056499317),
                Scalar(0.0011111066),
                Scalar(3.5665993E-4),
                Scalar(3.30081E-4)) );

    com_LF_WHEEL_L = iit::rbd::Vector3d(-4.887037E-5,2.6632963E-5,0.006043414).cast<Scalar>();
    tensor_LF_WHEEL_L.fill(
        Scalar(0.6363391),
        com_LF_WHEEL_L,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0014915846),
                Scalar(0.0014915847),
                Scalar(0.0028924393),
                Scalar(-8.2823526E-10),
                Scalar(-1.8793887E-7),
                Scalar(1.024836E-7)) );

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

    com_RF_SHANK = iit::rbd::Vector3d(0.044662513,0.030045537,-0.019255718).cast<Scalar>();
    tensor_RF_SHANK.fill(
        Scalar(0.41052794),
        com_RF_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0038513276),
                Scalar(0.002628087),
                Scalar(0.0056499317),
                Scalar(0.0011111066),
                Scalar(-3.566599E-4),
                Scalar(-3.3008077E-4)) );

    com_RF_WHEEL_L = iit::rbd::Vector3d(-4.887037E-5,2.6632963E-5,-0.006043414).cast<Scalar>();
    tensor_RF_WHEEL_L.fill(
        Scalar(0.6363391),
        com_RF_WHEEL_L,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0014915846),
                Scalar(0.0014915847),
                Scalar(0.0028924393),
                Scalar(-8.282353E-10),
                Scalar(1.8793887E-7),
                Scalar(-1.0235909E-7)) );

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

    com_LH_SHANK = iit::rbd::Vector3d(-0.044662513,0.030045535,0.019255718).cast<Scalar>();
    tensor_LH_SHANK.fill(
        Scalar(0.41052794),
        com_LH_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0038513276),
                Scalar(0.002628087),
                Scalar(0.0056499317),
                Scalar(-0.0011111066),
                Scalar(-3.566599E-4),
                Scalar(3.3008103E-4)) );

    com_LH_WHEEL_L = iit::rbd::Vector3d(4.887037E-5,2.6632963E-5,0.006043414).cast<Scalar>();
    tensor_LH_WHEEL_L.fill(
        Scalar(0.6363391),
        com_LH_WHEEL_L,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0014915846),
                Scalar(0.0014915847),
                Scalar(0.0028924393),
                Scalar(8.2823526E-10),
                Scalar(1.8793887E-7),
                Scalar(1.024836E-7)) );

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

    com_RH_SHANK = iit::rbd::Vector3d(-0.044662513,0.030045537,-0.019255718).cast<Scalar>();
    tensor_RH_SHANK.fill(
        Scalar(0.41052794),
        com_RH_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0038513276),
                Scalar(0.002628087),
                Scalar(0.0056499317),
                Scalar(-0.0011111066),
                Scalar(3.5665985E-4),
                Scalar(-3.3008083E-4)) );

    com_RH_WHEEL_L = iit::rbd::Vector3d(4.887037E-5,2.6632963E-5,-0.006043414).cast<Scalar>();
    tensor_RH_WHEEL_L.fill(
        Scalar(0.6363391),
        com_RH_WHEEL_L,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0014915846),
                Scalar(0.0014915847),
                Scalar(0.0028924393),
                Scalar(8.282353E-10),
                Scalar(-1.8793887E-7),
                Scalar(-1.0235909E-7)) );

}

