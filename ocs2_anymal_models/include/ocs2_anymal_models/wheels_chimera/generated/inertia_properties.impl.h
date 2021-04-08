template <typename TRAIT>
iit::wheels_chimera::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_base = iit::rbd::Vector3d(-0.021007974,0.005716143,0.012609911).cast<Scalar>();
    tensor_base.fill(
        Scalar(31.397486),
        com_base,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.2524699),
                Scalar(2.106768),
                Scalar(2.1195686),
                Scalar(-0.011331746),
                Scalar(-0.08438942),
                Scalar(0.0056773103)) );

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

    com_LF_SHANK = iit::rbd::Vector3d(0.0486941,0.025923084,0.0076155537).cast<Scalar>();
    tensor_LF_SHANK.fill(
        Scalar(0.42760074),
        com_LF_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0032048414),
                Scalar(0.002834782),
                Scalar(0.0054619443),
                Scalar(0.0011732029),
                Scalar(1.60833E-4),
                Scalar(2.329025E-4)) );

    com_LF_WHEEL_L = iit::rbd::Vector3d(-4.887037E-5,2.6632963E-5,0.006043414).cast<Scalar>();
    tensor_LF_WHEEL_L.fill(
        Scalar(0.65391314),
        com_LF_WHEEL_L,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0014922264),
                Scalar(0.0014922266),
                Scalar(0.0028924395),
                Scalar(-8.5110896E-10),
                Scalar(-1.9312924E-7),
                Scalar(1.053122E-7)) );

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

    com_RF_SHANK = iit::rbd::Vector3d(0.0486941,0.025923084,-0.007615554).cast<Scalar>();
    tensor_RF_SHANK.fill(
        Scalar(0.42760074),
        com_RF_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0032048414),
                Scalar(0.002834782),
                Scalar(0.0054619443),
                Scalar(0.0011732029),
                Scalar(-1.6083295E-4),
                Scalar(-2.3290228E-4)) );

    com_RF_WHEEL_L = iit::rbd::Vector3d(-4.887037E-5,2.6632963E-5,-0.006043414).cast<Scalar>();
    tensor_RF_WHEEL_L.fill(
        Scalar(0.65391314),
        com_RF_WHEEL_L,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0014922264),
                Scalar(0.0014922266),
                Scalar(0.0028924395),
                Scalar(-8.51109E-10),
                Scalar(1.9312924E-7),
                Scalar(-1.051877E-7)) );

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

    com_LH_SHANK = iit::rbd::Vector3d(-0.0486941,0.025923084,0.007615553).cast<Scalar>();
    tensor_LH_SHANK.fill(
        Scalar(0.42760074),
        com_LH_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0032048414),
                Scalar(0.002834782),
                Scalar(0.0054619443),
                Scalar(-0.0011732029),
                Scalar(-1.6083289E-4),
                Scalar(2.3290241E-4)) );

    com_LH_WHEEL_L = iit::rbd::Vector3d(4.887037E-5,2.6632963E-5,0.006043414).cast<Scalar>();
    tensor_LH_WHEEL_L.fill(
        Scalar(0.65391314),
        com_LH_WHEEL_L,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0014922264),
                Scalar(0.0014922266),
                Scalar(0.0028924395),
                Scalar(8.5110896E-10),
                Scalar(1.9312924E-7),
                Scalar(1.053122E-7)) );

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

    com_RH_SHANK = iit::rbd::Vector3d(-0.0486941,0.025923084,-0.0076155537).cast<Scalar>();
    tensor_RH_SHANK.fill(
        Scalar(0.42760074),
        com_RH_SHANK,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0032048414),
                Scalar(0.002834782),
                Scalar(0.0054619443),
                Scalar(-0.0011732029),
                Scalar(1.6083283E-4),
                Scalar(-2.3290221E-4)) );

    com_RH_WHEEL_L = iit::rbd::Vector3d(4.887037E-5,2.6632963E-5,-0.006043414).cast<Scalar>();
    tensor_RH_WHEEL_L.fill(
        Scalar(0.65391314),
        com_RH_WHEEL_L,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0014922264),
                Scalar(0.0014922266),
                Scalar(0.0028924395),
                Scalar(8.51109E-10),
                Scalar(-1.9312924E-7),
                Scalar(-1.051877E-7)) );

}

