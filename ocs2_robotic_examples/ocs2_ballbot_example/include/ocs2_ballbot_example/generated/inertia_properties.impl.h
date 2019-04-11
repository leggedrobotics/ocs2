template <typename TRAIT>
iit::Ballbot::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_dummy_ball1 = iit::rbd::Vector3d(0.0,0.0,0.0).cast<Scalar>();
    tensor_dummy_ball1.fill(
        Scalar(0.001),
        com_dummy_ball1,
        rbd::Utils::buildInertiaTensor(
                Scalar(1.0E-4),
                Scalar(1.0E-4),
                Scalar(1.0E-4),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

    com_ball = iit::rbd::Vector3d(-0.125,0.0,0.0).cast<Scalar>();
    tensor_ball.fill(
        Scalar(5.0),
        com_ball,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.102),
                Scalar(0.102),
                Scalar(0.102),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

    com_dummy_base1 = iit::rbd::Vector3d(0.0,0.0,0.0).cast<Scalar>();
    tensor_dummy_base1.fill(
        Scalar(0.001),
        com_dummy_base1,
        rbd::Utils::buildInertiaTensor(
                Scalar(1.0E-4),
                Scalar(1.0E-4),
                Scalar(1.0E-4),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

    com_dummy_base2 = iit::rbd::Vector3d(0.0,0.0,0.0).cast<Scalar>();
    tensor_dummy_base2.fill(
        Scalar(0.001),
        com_dummy_base2,
        rbd::Utils::buildInertiaTensor(
                Scalar(1.0E-4),
                Scalar(1.0E-4),
                Scalar(1.0E-4),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

    com_base = iit::rbd::Vector3d(-0.2376,0.0053,0.002).cast<Scalar>();
    tensor_base.fill(
        Scalar(16.94),
        com_base,
        rbd::Utils::buildInertiaTensor(
                Scalar(1.1241),
                Scalar(2.7611),
                Scalar(2.7574),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

}

