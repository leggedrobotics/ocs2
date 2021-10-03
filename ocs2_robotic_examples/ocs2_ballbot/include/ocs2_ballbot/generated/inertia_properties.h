#ifndef IIT_ROBOT_BALLBOT_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_BALLBOT_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"

namespace iit {
namespace Ballbot {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot Ballbot.
 */
namespace dyn {

using InertiaMatrix = iit::rbd::InertiaMatrixDense;

namespace tpl {

template <typename TRAIT>
class InertiaProperties {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef typename TRAIT::Scalar Scalar;
        typedef iit::rbd::Core<Scalar> CoreS;
        typedef iit::rbd::tpl::InertiaMatrixDense<Scalar> IMatrix;
        typedef typename CoreS::Vector3 Vec3d;

        InertiaProperties();
        ~InertiaProperties();
        const IMatrix& getTensor_dummy_ball1() const;
        const IMatrix& getTensor_ball() const;
        const IMatrix& getTensor_dummy_base1() const;
        const IMatrix& getTensor_dummy_base2() const;
        const IMatrix& getTensor_base() const;
        Scalar getMass_dummy_ball1() const;
        Scalar getMass_ball() const;
        Scalar getMass_dummy_base1() const;
        Scalar getMass_dummy_base2() const;
        Scalar getMass_base() const;
        const Vec3d& getCOM_dummy_ball1() const;
        const Vec3d& getCOM_ball() const;
        const Vec3d& getCOM_dummy_base1() const;
        const Vec3d& getCOM_dummy_base2() const;
        const Vec3d& getCOM_base() const;
        Scalar getTotalMass() const;

    private:

        IMatrix tensor_dummy_ball1;
        IMatrix tensor_ball;
        IMatrix tensor_dummy_base1;
        IMatrix tensor_dummy_base2;
        IMatrix tensor_base;
        Vec3d com_dummy_ball1;
        Vec3d com_ball;
        Vec3d com_dummy_base1;
        Vec3d com_dummy_base2;
        Vec3d com_base;
};

template <typename TRAIT>
inline InertiaProperties<TRAIT>::~InertiaProperties() {}

template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_dummy_ball1() const {
    return this->tensor_dummy_ball1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_ball() const {
    return this->tensor_ball;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_dummy_base1() const {
    return this->tensor_dummy_base1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_dummy_base2() const {
    return this->tensor_dummy_base2;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_base() const {
    return this->tensor_base;
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_dummy_ball1() const {
    return this->tensor_dummy_ball1.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_ball() const {
    return this->tensor_ball.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_dummy_base1() const {
    return this->tensor_dummy_base1.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_dummy_base2() const {
    return this->tensor_dummy_base2.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_base() const {
    return this->tensor_base.getMass();
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_dummy_ball1() const {
    return this->com_dummy_ball1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_ball() const {
    return this->com_ball;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_dummy_base1() const {
    return this->com_dummy_base1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_dummy_base2() const {
    return this->com_dummy_base2;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_base() const {
    return this->com_base;
}

template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getTotalMass() const {
    return 0.001 + 2.65 + 0.001 + 0.001 + 18.66;
}

}

using InertiaProperties = tpl::InertiaProperties<rbd::DoubleTrait>;

}
}
}

#include "inertia_properties.impl.h"

#endif
