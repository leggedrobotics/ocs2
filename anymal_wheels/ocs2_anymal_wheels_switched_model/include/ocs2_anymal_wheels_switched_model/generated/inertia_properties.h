#ifndef IIT_ROBOT_ANYMAL_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_ANYMAL_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"

namespace iit {
namespace ANYmal {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot ANYmal.
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
        const IMatrix& getTensor_base() const;
        const IMatrix& getTensor_LF_HIP() const;
        const IMatrix& getTensor_LF_THIGH() const;
        const IMatrix& getTensor_LF_shank_fixed() const;
        const IMatrix& getTensor_LF_WHEEL_L() const;
        const IMatrix& getTensor_RF_HIP() const;
        const IMatrix& getTensor_RF_THIGH() const;
        const IMatrix& getTensor_RF_shank_fixed() const;
        const IMatrix& getTensor_RF_WHEEL_L() const;
        const IMatrix& getTensor_LH_HIP() const;
        const IMatrix& getTensor_LH_THIGH() const;
        const IMatrix& getTensor_LH_shank_fixed() const;
        const IMatrix& getTensor_LH_WHEEL_L() const;
        const IMatrix& getTensor_RH_HIP() const;
        const IMatrix& getTensor_RH_THIGH() const;
        const IMatrix& getTensor_RH_shank_fixed() const;
        const IMatrix& getTensor_RH_WHEEL_L() const;
        Scalar getMass_base() const;
        Scalar getMass_LF_HIP() const;
        Scalar getMass_LF_THIGH() const;
        Scalar getMass_LF_shank_fixed() const;
        Scalar getMass_LF_WHEEL_L() const;
        Scalar getMass_RF_HIP() const;
        Scalar getMass_RF_THIGH() const;
        Scalar getMass_RF_shank_fixed() const;
        Scalar getMass_RF_WHEEL_L() const;
        Scalar getMass_LH_HIP() const;
        Scalar getMass_LH_THIGH() const;
        Scalar getMass_LH_shank_fixed() const;
        Scalar getMass_LH_WHEEL_L() const;
        Scalar getMass_RH_HIP() const;
        Scalar getMass_RH_THIGH() const;
        Scalar getMass_RH_shank_fixed() const;
        Scalar getMass_RH_WHEEL_L() const;
        const Vec3d& getCOM_base() const;
        const Vec3d& getCOM_LF_HIP() const;
        const Vec3d& getCOM_LF_THIGH() const;
        const Vec3d& getCOM_LF_shank_fixed() const;
        const Vec3d& getCOM_LF_WHEEL_L() const;
        const Vec3d& getCOM_RF_HIP() const;
        const Vec3d& getCOM_RF_THIGH() const;
        const Vec3d& getCOM_RF_shank_fixed() const;
        const Vec3d& getCOM_RF_WHEEL_L() const;
        const Vec3d& getCOM_LH_HIP() const;
        const Vec3d& getCOM_LH_THIGH() const;
        const Vec3d& getCOM_LH_shank_fixed() const;
        const Vec3d& getCOM_LH_WHEEL_L() const;
        const Vec3d& getCOM_RH_HIP() const;
        const Vec3d& getCOM_RH_THIGH() const;
        const Vec3d& getCOM_RH_shank_fixed() const;
        const Vec3d& getCOM_RH_WHEEL_L() const;
        Scalar getTotalMass() const;

    private:

        IMatrix tensor_base;
        IMatrix tensor_LF_HIP;
        IMatrix tensor_LF_THIGH;
        IMatrix tensor_LF_shank_fixed;
        IMatrix tensor_LF_WHEEL_L;
        IMatrix tensor_RF_HIP;
        IMatrix tensor_RF_THIGH;
        IMatrix tensor_RF_shank_fixed;
        IMatrix tensor_RF_WHEEL_L;
        IMatrix tensor_LH_HIP;
        IMatrix tensor_LH_THIGH;
        IMatrix tensor_LH_shank_fixed;
        IMatrix tensor_LH_WHEEL_L;
        IMatrix tensor_RH_HIP;
        IMatrix tensor_RH_THIGH;
        IMatrix tensor_RH_shank_fixed;
        IMatrix tensor_RH_WHEEL_L;
        Vec3d com_base;
        Vec3d com_LF_HIP;
        Vec3d com_LF_THIGH;
        Vec3d com_LF_shank_fixed;
        Vec3d com_LF_WHEEL_L;
        Vec3d com_RF_HIP;
        Vec3d com_RF_THIGH;
        Vec3d com_RF_shank_fixed;
        Vec3d com_RF_WHEEL_L;
        Vec3d com_LH_HIP;
        Vec3d com_LH_THIGH;
        Vec3d com_LH_shank_fixed;
        Vec3d com_LH_WHEEL_L;
        Vec3d com_RH_HIP;
        Vec3d com_RH_THIGH;
        Vec3d com_RH_shank_fixed;
        Vec3d com_RH_WHEEL_L;
};

template <typename TRAIT>
inline InertiaProperties<TRAIT>::~InertiaProperties() {}

template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_base() const {
    return this->tensor_base;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_LF_HIP() const {
    return this->tensor_LF_HIP;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_LF_THIGH() const {
    return this->tensor_LF_THIGH;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_LF_shank_fixed() const {
    return this->tensor_LF_shank_fixed;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_LF_WHEEL_L() const {
    return this->tensor_LF_WHEEL_L;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_RF_HIP() const {
    return this->tensor_RF_HIP;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_RF_THIGH() const {
    return this->tensor_RF_THIGH;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_RF_shank_fixed() const {
    return this->tensor_RF_shank_fixed;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_RF_WHEEL_L() const {
    return this->tensor_RF_WHEEL_L;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_LH_HIP() const {
    return this->tensor_LH_HIP;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_LH_THIGH() const {
    return this->tensor_LH_THIGH;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_LH_shank_fixed() const {
    return this->tensor_LH_shank_fixed;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_LH_WHEEL_L() const {
    return this->tensor_LH_WHEEL_L;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_RH_HIP() const {
    return this->tensor_RH_HIP;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_RH_THIGH() const {
    return this->tensor_RH_THIGH;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_RH_shank_fixed() const {
    return this->tensor_RH_shank_fixed;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_RH_WHEEL_L() const {
    return this->tensor_RH_WHEEL_L;
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_base() const {
    return this->tensor_base.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_LF_HIP() const {
    return this->tensor_LF_HIP.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_LF_THIGH() const {
    return this->tensor_LF_THIGH.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_LF_shank_fixed() const {
    return this->tensor_LF_shank_fixed.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_LF_WHEEL_L() const {
    return this->tensor_LF_WHEEL_L.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_RF_HIP() const {
    return this->tensor_RF_HIP.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_RF_THIGH() const {
    return this->tensor_RF_THIGH.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_RF_shank_fixed() const {
    return this->tensor_RF_shank_fixed.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_RF_WHEEL_L() const {
    return this->tensor_RF_WHEEL_L.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_LH_HIP() const {
    return this->tensor_LH_HIP.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_LH_THIGH() const {
    return this->tensor_LH_THIGH.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_LH_shank_fixed() const {
    return this->tensor_LH_shank_fixed.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_LH_WHEEL_L() const {
    return this->tensor_LH_WHEEL_L.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_RH_HIP() const {
    return this->tensor_RH_HIP.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_RH_THIGH() const {
    return this->tensor_RH_THIGH.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_RH_shank_fixed() const {
    return this->tensor_RH_shank_fixed.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_RH_WHEEL_L() const {
    return this->tensor_RH_WHEEL_L.getMass();
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_base() const {
    return this->com_base;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_LF_HIP() const {
    return this->com_LF_HIP;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_LF_THIGH() const {
    return this->com_LF_THIGH;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_LF_shank_fixed() const {
    return this->com_LF_shank_fixed;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_LF_WHEEL_L() const {
    return this->com_LF_WHEEL_L;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_RF_HIP() const {
    return this->com_RF_HIP;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_RF_THIGH() const {
    return this->com_RF_THIGH;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_RF_shank_fixed() const {
    return this->com_RF_shank_fixed;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_RF_WHEEL_L() const {
    return this->com_RF_WHEEL_L;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_LH_HIP() const {
    return this->com_LH_HIP;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_LH_THIGH() const {
    return this->com_LH_THIGH;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_LH_shank_fixed() const {
    return this->com_LH_shank_fixed;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_LH_WHEEL_L() const {
    return this->com_LH_WHEEL_L;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_RH_HIP() const {
    return this->com_RH_HIP;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_RH_THIGH() const {
    return this->com_RH_THIGH;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_RH_shank_fixed() const {
    return this->com_RH_shank_fixed;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_RH_WHEEL_L() const {
    return this->com_RH_WHEEL_L;
}

template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getTotalMass() const {
    return Scalar(25.049328 + 1.8623549 + 2.3501198 + 0.41052794 + 0.6363391 + 1.8623549 + 2.3501198 + 0.41052794 + 0.6363391 + 1.8623549 + 2.3501198 + 0.41052794 + 0.6363391 + 1.8623549 + 2.3501198 + 0.41052794 + 0.6363391);
}

}

using InertiaProperties = tpl::InertiaProperties<rbd::DoubleTrait>;

}
}
}

#include "inertia_properties.impl.h"

#endif
