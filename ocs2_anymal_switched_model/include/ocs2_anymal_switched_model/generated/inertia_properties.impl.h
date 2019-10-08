template <typename TRAIT>
iit::ANYmal::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_base = iit::rbd::Vector3d(-0.008312997,0.0032444296,0.050561193).cast<Scalar>();
    tensor_base.fill(
        Scalar(18.85),
        com_base,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.2974973),
                Scalar(0.7008077),
                Scalar(0.62285703),
                Scalar(-0.0014148138),
                Scalar(-0.003508613),
                Scalar(0.005590358)) );

    com_LF_HIP = iit::rbd::Vector3d(1.521844E-4,-0.0037871017,0.06451626).cast<Scalar>();
    tensor_LF_HIP.fill(
        Scalar(1.4246206),
        com_LF_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.007930875),
                Scalar(0.008232372),
                Scalar(0.0024506985),
                Scalar(2.5651958E-5),
                Scalar(-7.831626E-6),
                Scalar(-3.3277468E-4)) );

    com_LF_THIGH = iit::rbd::Vector3d(-0.003897968,0.21458337,0.05422662).cast<Scalar>();
    tensor_LF_THIGH.fill(
        Scalar(1.6349765),
        com_LF_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.09212865),
                Scalar(0.007326758),
                Scalar(0.087373376),
                Scalar(-0.0010797519),
                Scalar(-4.1321156E-4),
                Scalar(0.017618697)) );

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

    com_RF_HIP = iit::rbd::Vector3d(1.521844E-4,0.0037871017,0.06451626).cast<Scalar>();
    tensor_RF_HIP.fill(
        Scalar(1.4246206),
        com_RF_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.007930875),
                Scalar(0.008232372),
                Scalar(0.0024506985),
                Scalar(-2.5651958E-5),
                Scalar(-7.831626E-6),
                Scalar(3.3277468E-4)) );

    com_RF_THIGH = iit::rbd::Vector3d(-0.003897968,0.21458337,-0.05422662).cast<Scalar>();
    tensor_RF_THIGH.fill(
        Scalar(1.6349765),
        com_RF_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.09212865),
                Scalar(0.0073267594),
                Scalar(0.087373376),
                Scalar(-0.0010797519),
                Scalar(4.132116E-4),
                Scalar(-0.017618697)) );

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

    com_LH_HIP = iit::rbd::Vector3d(1.521844E-4,-0.0037871017,-0.06451626).cast<Scalar>();
    tensor_LH_HIP.fill(
        Scalar(1.4246206),
        com_LH_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.007930875),
                Scalar(0.008232372),
                Scalar(0.0024506985),
                Scalar(2.5651958E-5),
                Scalar(7.831666E-6),
                Scalar(3.3277468E-4)) );

    com_LH_THIGH = iit::rbd::Vector3d(0.003897968,0.21458337,0.05422662).cast<Scalar>();
    tensor_LH_THIGH.fill(
        Scalar(1.6349765),
        com_LH_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.09212865),
                Scalar(0.007326758),
                Scalar(0.087373376),
                Scalar(0.0010797519),
                Scalar(4.1321156E-4),
                Scalar(0.017618697)) );

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

    com_RH_HIP = iit::rbd::Vector3d(1.521844E-4,0.0037871017,-0.06451626).cast<Scalar>();
    tensor_RH_HIP.fill(
        Scalar(1.4246206),
        com_RH_HIP,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.007930875),
                Scalar(0.008232372),
                Scalar(0.0024506985),
                Scalar(-2.5651958E-5),
                Scalar(7.831666E-6),
                Scalar(-3.3277468E-4)) );

    com_RH_THIGH = iit::rbd::Vector3d(0.003897968,0.21458337,-0.05422662).cast<Scalar>();
    tensor_RH_THIGH.fill(
        Scalar(1.6349765),
        com_RH_THIGH,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.09212865),
                Scalar(0.0073267594),
                Scalar(0.087373376),
                Scalar(0.0010797519),
                Scalar(-4.132116E-4),
                Scalar(-0.017618697)) );

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

