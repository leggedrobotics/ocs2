#ifndef IIT_ROBOT_CAMEL_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_CAMEL_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"

namespace iit {
namespace camel {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot camel.
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
        const IMatrix& getTensor_LF_SHANK() const;
        const IMatrix& getTensor_RF_HIP() const;
        const IMatrix& getTensor_RF_THIGH() const;
        const IMatrix& getTensor_RF_SHANK() const;
        const IMatrix& getTensor_LH_HIP() const;
        const IMatrix& getTensor_LH_THIGH() const;
        const IMatrix& getTensor_LH_SHANK() const;
        const IMatrix& getTensor_RH_HIP() const;
        const IMatrix& getTensor_RH_THIGH() const;
        const IMatrix& getTensor_RH_SHANK() const;
        Scalar getMass_base() const;
        Scalar getMass_LF_HIP() const;
        Scalar getMass_LF_THIGH() const;
        Scalar getMass_LF_SHANK() const;
        Scalar getMass_RF_HIP() const;
        Scalar getMass_RF_THIGH() const;
        Scalar getMass_RF_SHANK() const;
        Scalar getMass_LH_HIP() const;
        Scalar getMass_LH_THIGH() const;
        Scalar getMass_LH_SHANK() const;
        Scalar getMass_RH_HIP() const;
        Scalar getMass_RH_THIGH() const;
        Scalar getMass_RH_SHANK() const;
        const Vec3d& getCOM_base() const;
        const Vec3d& getCOM_LF_HIP() const;
        const Vec3d& getCOM_LF_THIGH() const;
        const Vec3d& getCOM_LF_SHANK() const;
        const Vec3d& getCOM_RF_HIP() const;
        const Vec3d& getCOM_RF_THIGH() const;
        const Vec3d& getCOM_RF_SHANK() const;
        const Vec3d& getCOM_LH_HIP() const;
        const Vec3d& getCOM_LH_THIGH() const;
        const Vec3d& getCOM_LH_SHANK() const;
        const Vec3d& getCOM_RH_HIP() const;
        const Vec3d& getCOM_RH_THIGH() const;
        const Vec3d& getCOM_RH_SHANK() const;
        Scalar getTotalMass() const;

    private:

        IMatrix tensor_base;
        IMatrix tensor_LF_HIP;
        IMatrix tensor_LF_THIGH;
        IMatrix tensor_LF_SHANK;
        IMatrix tensor_RF_HIP;
        IMatrix tensor_RF_THIGH;
        IMatrix tensor_RF_SHANK;
        IMatrix tensor_LH_HIP;
        IMatrix tensor_LH_THIGH;
        IMatrix tensor_LH_SHANK;
        IMatrix tensor_RH_HIP;
        IMatrix tensor_RH_THIGH;
        IMatrix tensor_RH_SHANK;
        Vec3d com_base;
        Vec3d com_LF_HIP;
        Vec3d com_LF_THIGH;
        Vec3d com_LF_SHANK;
        Vec3d com_RF_HIP;
        Vec3d com_RF_THIGH;
        Vec3d com_RF_SHANK;
        Vec3d com_LH_HIP;
        Vec3d com_LH_THIGH;
        Vec3d com_LH_SHANK;
        Vec3d com_RH_HIP;
        Vec3d com_RH_THIGH;
        Vec3d com_RH_SHANK;
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
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_LF_SHANK() const {
    return this->tensor_LF_SHANK;
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
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_RF_SHANK() const {
    return this->tensor_RF_SHANK;
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
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_LH_SHANK() const {
    return this->tensor_LH_SHANK;
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
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_RH_SHANK() const {
    return this->tensor_RH_SHANK;
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
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_LF_SHANK() const {
    return this->tensor_LF_SHANK.getMass();
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
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_RF_SHANK() const {
    return this->tensor_RF_SHANK.getMass();
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
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_LH_SHANK() const {
    return this->tensor_LH_SHANK.getMass();
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
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_RH_SHANK() const {
    return this->tensor_RH_SHANK.getMass();
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
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_LF_SHANK() const {
    return this->com_LF_SHANK;
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
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_RF_SHANK() const {
    return this->com_RF_SHANK;
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
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_LH_SHANK() const {
    return this->com_LH_SHANK;
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
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_RH_SHANK() const {
    return this->com_RH_SHANK;
}

template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getTotalMass() const {
    return Scalar(26.499758 + 2.781 + 3.071 + 0.58842 + 2.781 + 3.071 + 0.58842 + 2.781 + 3.071 + 0.58842 + 2.781 + 3.071 + 0.58842);
}

}

using InertiaProperties = tpl::InertiaProperties<rbd::DoubleTrait>;

}
}
}

#include "inertia_properties.impl.h"

#endif
