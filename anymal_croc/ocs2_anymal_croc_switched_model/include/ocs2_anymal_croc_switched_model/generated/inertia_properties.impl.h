template <typename TRAIT>
iit::ANYmal::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_base = iit::rbd::Vector3d(-0.018088844,-2.1014918E-4,0.0019263081).cast<Scalar>();
    tensor_base.fill(
        Scalar(25.508142),
        com_base,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.21581423),
                Scalar(1.8553128),
                Scalar(1.9356725),
                Scalar(-0.0045547215),
                Scalar(-0.04372399),
                Scalar(0.001628948)) );

    com_LF_HIP = iit::rbd::Vector3d(9.952248E-4,0.01605701,0.063007474).cast<Scalar>();
    tensor_LF_HIP.fill(
        Scalar(2.631),
        com_LF_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.01691537),
                Scalar(0.014479431),
                Scalar(0.0053204754),
                Scalar(-2.5228714E-5),
                Scalar(2.1883397E-4),
                Scalar(0.002581513)) );

    com_LF_THIGH = iit::rbd::Vector3d(3.553805E-5,0.234335,0.015803616).cast<Scalar>();
    tensor_LF_THIGH.fill(
        Scalar(3.241),
        com_LF_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.21329968),
                Scalar(0.0064454637),
                Scalar(0.21149169),
                Scalar(-3.4727982E-5),
                Scalar(2.3600842E-5),
                Scalar(0.013588932)) );

    com_LF_SHANK = iit::rbd::Vector3d(0.05243166,0.053917024,0.017812103).cast<Scalar>();
    tensor_LF_SHANK.fill(
        Scalar(0.578),
        com_LF_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.008518288),
                Scalar(0.003160114),
                Scalar(0.011083069),
                Scalar(0.0028563053),
                Scalar(5.249712E-4),
                Scalar(4.983306E-4)) );

    com_RF_HIP = iit::rbd::Vector3d(9.952248E-4,-0.01605701,0.06311905).cast<Scalar>();
    tensor_RF_HIP.fill(
        Scalar(2.631),
        com_RF_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.016950594),
                Scalar(0.014514658),
                Scalar(0.0053204754),
                Scalar(2.5228248E-5),
                Scalar(2.671816E-4),
                Scalar(-0.0025431812)) );

    com_RF_THIGH = iit::rbd::Vector3d(1.2611576E-4,0.234335,-0.015803618).cast<Scalar>();
    tensor_RF_THIGH.fill(
        Scalar(3.241),
        com_RF_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.21329968),
                Scalar(0.0064454637),
                Scalar(0.21149169),
                Scalar(9.728495E-5),
                Scalar(1.5318064E-5),
                Scalar(-0.01358893)) );

    com_RF_SHANK = iit::rbd::Vector3d(0.05243166,0.053917024,-0.017812103).cast<Scalar>();
    tensor_RF_SHANK.fill(
        Scalar(0.578),
        com_RF_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.008518288),
                Scalar(0.0031601144),
                Scalar(0.011083069),
                Scalar(0.0028563053),
                Scalar(-5.249711E-4),
                Scalar(-4.9833005E-4)) );

    com_LH_HIP = iit::rbd::Vector3d(9.952243E-4,0.01605701,-0.06311905).cast<Scalar>();
    tensor_LH_HIP.fill(
        Scalar(2.631),
        com_LH_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.016950592),
                Scalar(0.014514658),
                Scalar(0.0053204754),
                Scalar(-2.5228132E-5),
                Scalar(-2.6718157E-4),
                Scalar(-0.002543181)) );

    com_LH_THIGH = iit::rbd::Vector3d(-1.2611576E-4,0.234335,0.015803618).cast<Scalar>();
    tensor_LH_THIGH.fill(
        Scalar(3.241),
        com_LH_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.21329968),
                Scalar(0.006445464),
                Scalar(0.21149169),
                Scalar(-9.728511E-5),
                Scalar(1.5317977E-5),
                Scalar(0.013588934)) );

    com_LH_SHANK = iit::rbd::Vector3d(-0.05243166,0.053917024,0.017812103).cast<Scalar>();
    tensor_LH_SHANK.fill(
        Scalar(0.578),
        com_LH_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.008518288),
                Scalar(0.0031601144),
                Scalar(0.011083069),
                Scalar(-0.0028563053),
                Scalar(-5.249711E-4),
                Scalar(4.983305E-4)) );

    com_RH_HIP = iit::rbd::Vector3d(9.952243E-4,-0.01605701,-0.063007474).cast<Scalar>();
    tensor_RH_HIP.fill(
        Scalar(2.631),
        com_RH_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.016915368),
                Scalar(0.014479431),
                Scalar(0.0053204754),
                Scalar(2.5228597E-5),
                Scalar(-2.1883396E-4),
                Scalar(0.0025815128)) );

    com_RH_THIGH = iit::rbd::Vector3d(-3.553805E-5,0.234335,-0.01580362).cast<Scalar>();
    tensor_RH_THIGH.fill(
        Scalar(3.241),
        com_RH_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.21329968),
                Scalar(0.0064454633),
                Scalar(0.21149169),
                Scalar(3.472784E-5),
                Scalar(2.3600935E-5),
                Scalar(-0.013588931)) );

    com_RH_SHANK = iit::rbd::Vector3d(-0.05243166,0.053917024,-0.017812103).cast<Scalar>();
    tensor_RH_SHANK.fill(
        Scalar(0.578),
        com_RH_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.008518288),
                Scalar(0.0031601144),
                Scalar(0.011083069),
                Scalar(-0.0028563053),
                Scalar(5.24971E-4),
                Scalar(-4.983299E-4)) );

}

