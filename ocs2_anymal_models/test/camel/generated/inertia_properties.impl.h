template <typename TRAIT>
iit::camel::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_base = iit::rbd::Vector3d(-0.013955938,-1.8214522E-4,0.008853551).cast<Scalar>();
    tensor_base.fill(
        Scalar(26.499758),
        com_base,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.22784324),
                Scalar(1.7974265),
                Scalar(1.8366772),
                Scalar(-0.008108853),
                Scalar(-0.041152608),
                Scalar(0.0011102436)) );

    com_LF_HIP = iit::rbd::Vector3d(4.608417E-4,0.017393887,0.056660626).cast<Scalar>();
    tensor_LF_HIP.fill(
        Scalar(2.781),
        com_LF_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.015602534),
                Scalar(0.013867088),
                Scalar(0.004137501),
                Scalar(1.5809995E-4),
                Scalar(-2.5296624E-5),
                Scalar(0.0027853446)) );

    com_LF_THIGH = iit::rbd::Vector3d(-4.649951E-5,0.24569574,0.030814718).cast<Scalar>();
    tensor_LF_THIGH.fill(
        Scalar(3.071),
        com_LF_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.21820532),
                Scalar(0.0077771503),
                Scalar(0.21494901),
                Scalar(-1.295655E-4),
                Scalar(3.989154E-5),
                Scalar(0.027393108)) );

    com_LF_SHANK = iit::rbd::Vector3d(0.06125277,0.080659784,0.0054619997).cast<Scalar>();
    tensor_LF_SHANK.fill(
        Scalar(0.58842),
        com_LF_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.012656195),
                Scalar(0.0037750325),
                Scalar(0.016197097),
                Scalar(0.004726522),
                Scalar(1.4329211E-4),
                Scalar(2.3657648E-4)) );

    com_RF_HIP = iit::rbd::Vector3d(4.608417E-4,-0.017393887,0.05676332).cast<Scalar>();
    tensor_RF_HIP.fill(
        Scalar(2.781),
        com_RF_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.015636748),
                Scalar(0.013901301),
                Scalar(0.004137501),
                Scalar(-1.5810088E-4),
                Scalar(1.7275191E-4),
                Scalar(-0.002700748)) );

    com_RF_THIGH = iit::rbd::Vector3d(4.649951E-5,0.24569574,-0.030814718).cast<Scalar>();
    tensor_RF_THIGH.fill(
        Scalar(3.071),
        com_RF_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.21820532),
                Scalar(0.00777715),
                Scalar(0.21494901),
                Scalar(1.4987914E-4),
                Scalar(3.9995582E-5),
                Scalar(-0.027393105)) );

    com_RF_SHANK = iit::rbd::Vector3d(0.06125277,0.080659784,-0.005461999).cast<Scalar>();
    tensor_RF_SHANK.fill(
        Scalar(0.58842),
        com_RF_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.012656195),
                Scalar(0.0037750327),
                Scalar(0.016197097),
                Scalar(0.004726522),
                Scalar(-1.4329197E-4),
                Scalar(-2.3657575E-4)) );

    com_LH_HIP = iit::rbd::Vector3d(4.6084123E-4,0.017393887,-0.05676332).cast<Scalar>();
    tensor_LH_HIP.fill(
        Scalar(2.781),
        com_LH_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.015636746),
                Scalar(0.013901301),
                Scalar(0.004137501),
                Scalar(1.58101E-4),
                Scalar(-1.7275206E-4),
                Scalar(-0.0027007482)) );

    com_LH_THIGH = iit::rbd::Vector3d(-4.649951E-5,0.24569574,0.030814718).cast<Scalar>();
    tensor_LH_THIGH.fill(
        Scalar(3.071),
        com_LH_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.21820532),
                Scalar(0.0077771503),
                Scalar(0.21494901),
                Scalar(-1.4987949E-4),
                Scalar(3.9995542E-5),
                Scalar(0.027393108)) );

    com_LH_SHANK = iit::rbd::Vector3d(-0.06125277,0.080659784,0.005461999).cast<Scalar>();
    tensor_LH_SHANK.fill(
        Scalar(0.58842),
        com_LH_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.012656195),
                Scalar(0.0037750327),
                Scalar(0.016197097),
                Scalar(-0.004726522),
                Scalar(-1.4329198E-4),
                Scalar(2.3657634E-4)) );

    com_RH_HIP = iit::rbd::Vector3d(4.6084123E-4,-0.017393887,-0.056660626).cast<Scalar>();
    tensor_RH_HIP.fill(
        Scalar(2.781),
        com_RH_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.015602534),
                Scalar(0.013867087),
                Scalar(0.004137501),
                Scalar(-1.5810053E-4),
                Scalar(2.5296595E-5),
                Scalar(0.0027853448)) );

    com_RH_THIGH = iit::rbd::Vector3d(4.649951E-5,0.24569574,-0.030814718).cast<Scalar>();
    tensor_RH_THIGH.fill(
        Scalar(3.071),
        com_RH_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.21820532),
                Scalar(0.00777715),
                Scalar(0.21494901),
                Scalar(1.2956513E-4),
                Scalar(3.989158E-5),
                Scalar(-0.027393105)) );

    com_RH_SHANK = iit::rbd::Vector3d(-0.06125277,0.080659784,-0.0054619987).cast<Scalar>();
    tensor_RH_SHANK.fill(
        Scalar(0.58842),
        com_RH_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.012656195),
                Scalar(0.0037750327),
                Scalar(0.016197097),
                Scalar(-0.004726522),
                Scalar(1.4329182E-4),
                Scalar(-2.3657558E-4)) );

}

