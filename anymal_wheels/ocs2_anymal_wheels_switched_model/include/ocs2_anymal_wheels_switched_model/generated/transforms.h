#ifndef ANYMAL_TRANSFORMS_H_
#define ANYMAL_TRANSFORMS_H_

#include <Eigen/Dense>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include <iit/rbd/traits/DoubleTrait.h>
#include "kinematics_parameters.h"

namespace iit {
namespace ANYmal {

template<typename SCALAR, class M>
class TransformMotion : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename SCALAR, class M>
class TransformForce : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename SCALAR, class M>
class TransformHomogeneous : public iit::rbd::HomogeneousTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

namespace tpl {


/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
template <typename TRAIT>
class MotionTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformMotion<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_base_X_fr_LF_WHEEL_L : public TransformMotion<Scalar, Type_fr_base_X_fr_LF_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_WHEEL_L();
        const Type_fr_base_X_fr_LF_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_WHEEL_L_X_fr_base : public TransformMotion<Scalar, Type_fr_LF_WHEEL_L_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_WHEEL_L_X_fr_base();
        const Type_fr_LF_WHEEL_L_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_WHEEL_L : public TransformMotion<Scalar, Type_fr_base_X_fr_LH_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_WHEEL_L();
        const Type_fr_base_X_fr_LH_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_WHEEL_L_X_fr_base : public TransformMotion<Scalar, Type_fr_LH_WHEEL_L_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_WHEEL_L_X_fr_base();
        const Type_fr_LH_WHEEL_L_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_WHEEL_L : public TransformMotion<Scalar, Type_fr_base_X_fr_RF_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_WHEEL_L();
        const Type_fr_base_X_fr_RF_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_WHEEL_L_X_fr_base : public TransformMotion<Scalar, Type_fr_RF_WHEEL_L_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_WHEEL_L_X_fr_base();
        const Type_fr_RF_WHEEL_L_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_WHEEL_L : public TransformMotion<Scalar, Type_fr_base_X_fr_RH_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_WHEEL_L();
        const Type_fr_base_X_fr_RH_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_WHEEL_L_X_fr_base : public TransformMotion<Scalar, Type_fr_RH_WHEEL_L_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_WHEEL_L_X_fr_base();
        const Type_fr_RH_WHEEL_L_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_WHEEL_L_COM : public TransformMotion<Scalar, Type_fr_base_X_fr_LF_WHEEL_L_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_WHEEL_L_COM();
        const Type_fr_base_X_fr_LF_WHEEL_L_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_WHEEL_L_COM_X_fr_base : public TransformMotion<Scalar, Type_fr_LF_WHEEL_L_COM_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_WHEEL_L_COM_X_fr_base();
        const Type_fr_LF_WHEEL_L_COM_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_WHEEL_L_COM : public TransformMotion<Scalar, Type_fr_base_X_fr_LH_WHEEL_L_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_WHEEL_L_COM();
        const Type_fr_base_X_fr_LH_WHEEL_L_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_WHEEL_L_COM_X_fr_base : public TransformMotion<Scalar, Type_fr_LH_WHEEL_L_COM_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_WHEEL_L_COM_X_fr_base();
        const Type_fr_LH_WHEEL_L_COM_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_WHEEL_L_COM : public TransformMotion<Scalar, Type_fr_base_X_fr_RF_WHEEL_L_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_WHEEL_L_COM();
        const Type_fr_base_X_fr_RF_WHEEL_L_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_WHEEL_L_COM_X_fr_base : public TransformMotion<Scalar, Type_fr_RF_WHEEL_L_COM_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_WHEEL_L_COM_X_fr_base();
        const Type_fr_RF_WHEEL_L_COM_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_WHEEL_L_COM : public TransformMotion<Scalar, Type_fr_base_X_fr_RH_WHEEL_L_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_WHEEL_L_COM();
        const Type_fr_base_X_fr_RH_WHEEL_L_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_WHEEL_L_COM_X_fr_base : public TransformMotion<Scalar, Type_fr_RH_WHEEL_L_COM_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_WHEEL_L_COM_X_fr_base();
        const Type_fr_RH_WHEEL_L_COM_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_HAA : public TransformMotion<Scalar, Type_fr_base_X_fr_LF_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_HAA();
        const Type_fr_base_X_fr_LF_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_HFE : public TransformMotion<Scalar, Type_fr_base_X_fr_LF_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_HFE();
        const Type_fr_base_X_fr_LF_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_KFE : public TransformMotion<Scalar, Type_fr_base_X_fr_LF_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_KFE();
        const Type_fr_base_X_fr_LF_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_WHEEL : public TransformMotion<Scalar, Type_fr_base_X_fr_LF_WHEEL>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_WHEEL();
        const Type_fr_base_X_fr_LF_WHEEL& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_HAA : public TransformMotion<Scalar, Type_fr_base_X_fr_LH_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_HAA();
        const Type_fr_base_X_fr_LH_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_HFE : public TransformMotion<Scalar, Type_fr_base_X_fr_LH_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_HFE();
        const Type_fr_base_X_fr_LH_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_KFE : public TransformMotion<Scalar, Type_fr_base_X_fr_LH_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_KFE();
        const Type_fr_base_X_fr_LH_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_WHEEL : public TransformMotion<Scalar, Type_fr_base_X_fr_LH_WHEEL>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_WHEEL();
        const Type_fr_base_X_fr_LH_WHEEL& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_HAA : public TransformMotion<Scalar, Type_fr_base_X_fr_RF_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_HAA();
        const Type_fr_base_X_fr_RF_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_HFE : public TransformMotion<Scalar, Type_fr_base_X_fr_RF_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_HFE();
        const Type_fr_base_X_fr_RF_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_KFE : public TransformMotion<Scalar, Type_fr_base_X_fr_RF_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_KFE();
        const Type_fr_base_X_fr_RF_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_WHEEL : public TransformMotion<Scalar, Type_fr_base_X_fr_RF_WHEEL>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_WHEEL();
        const Type_fr_base_X_fr_RF_WHEEL& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_HAA : public TransformMotion<Scalar, Type_fr_base_X_fr_RH_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_HAA();
        const Type_fr_base_X_fr_RH_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_HFE : public TransformMotion<Scalar, Type_fr_base_X_fr_RH_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_HFE();
        const Type_fr_base_X_fr_RH_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_KFE : public TransformMotion<Scalar, Type_fr_base_X_fr_RH_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_KFE();
        const Type_fr_base_X_fr_RH_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_WHEEL : public TransformMotion<Scalar, Type_fr_base_X_fr_RH_WHEEL>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_WHEEL();
        const Type_fr_base_X_fr_RH_WHEEL& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_HIP_X_fr_base : public TransformMotion<Scalar, Type_fr_LF_HIP_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_HIP_X_fr_base();
        const Type_fr_LF_HIP_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_HIP : public TransformMotion<Scalar, Type_fr_base_X_fr_LF_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_HIP();
        const Type_fr_base_X_fr_LF_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_THIGH_X_fr_LF_HIP : public TransformMotion<Scalar, Type_fr_LF_THIGH_X_fr_LF_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_THIGH_X_fr_LF_HIP();
        const Type_fr_LF_THIGH_X_fr_LF_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_HIP_X_fr_LF_THIGH : public TransformMotion<Scalar, Type_fr_LF_HIP_X_fr_LF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_HIP_X_fr_LF_THIGH();
        const Type_fr_LF_HIP_X_fr_LF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_shank_fixed_X_fr_LF_THIGH : public TransformMotion<Scalar, Type_fr_LF_shank_fixed_X_fr_LF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_shank_fixed_X_fr_LF_THIGH();
        const Type_fr_LF_shank_fixed_X_fr_LF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_THIGH_X_fr_LF_shank_fixed : public TransformMotion<Scalar, Type_fr_LF_THIGH_X_fr_LF_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_THIGH_X_fr_LF_shank_fixed();
        const Type_fr_LF_THIGH_X_fr_LF_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_WHEEL_L_X_fr_LF_shank_fixed : public TransformMotion<Scalar, Type_fr_LF_WHEEL_L_X_fr_LF_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_WHEEL_L_X_fr_LF_shank_fixed();
        const Type_fr_LF_WHEEL_L_X_fr_LF_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_shank_fixed_X_fr_LF_WHEEL_L : public TransformMotion<Scalar, Type_fr_LF_shank_fixed_X_fr_LF_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_shank_fixed_X_fr_LF_WHEEL_L();
        const Type_fr_LF_shank_fixed_X_fr_LF_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_HIP_X_fr_base : public TransformMotion<Scalar, Type_fr_RF_HIP_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_HIP_X_fr_base();
        const Type_fr_RF_HIP_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_HIP : public TransformMotion<Scalar, Type_fr_base_X_fr_RF_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_HIP();
        const Type_fr_base_X_fr_RF_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_THIGH_X_fr_RF_HIP : public TransformMotion<Scalar, Type_fr_RF_THIGH_X_fr_RF_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_THIGH_X_fr_RF_HIP();
        const Type_fr_RF_THIGH_X_fr_RF_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_HIP_X_fr_RF_THIGH : public TransformMotion<Scalar, Type_fr_RF_HIP_X_fr_RF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_HIP_X_fr_RF_THIGH();
        const Type_fr_RF_HIP_X_fr_RF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_shank_fixed_X_fr_RF_THIGH : public TransformMotion<Scalar, Type_fr_RF_shank_fixed_X_fr_RF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_shank_fixed_X_fr_RF_THIGH();
        const Type_fr_RF_shank_fixed_X_fr_RF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_THIGH_X_fr_RF_shank_fixed : public TransformMotion<Scalar, Type_fr_RF_THIGH_X_fr_RF_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_THIGH_X_fr_RF_shank_fixed();
        const Type_fr_RF_THIGH_X_fr_RF_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_WHEEL_L_X_fr_RF_shank_fixed : public TransformMotion<Scalar, Type_fr_RF_WHEEL_L_X_fr_RF_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_WHEEL_L_X_fr_RF_shank_fixed();
        const Type_fr_RF_WHEEL_L_X_fr_RF_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_shank_fixed_X_fr_RF_WHEEL_L : public TransformMotion<Scalar, Type_fr_RF_shank_fixed_X_fr_RF_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_shank_fixed_X_fr_RF_WHEEL_L();
        const Type_fr_RF_shank_fixed_X_fr_RF_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_HIP_X_fr_base : public TransformMotion<Scalar, Type_fr_LH_HIP_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_HIP_X_fr_base();
        const Type_fr_LH_HIP_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_HIP : public TransformMotion<Scalar, Type_fr_base_X_fr_LH_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_HIP();
        const Type_fr_base_X_fr_LH_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_THIGH_X_fr_LH_HIP : public TransformMotion<Scalar, Type_fr_LH_THIGH_X_fr_LH_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_THIGH_X_fr_LH_HIP();
        const Type_fr_LH_THIGH_X_fr_LH_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_HIP_X_fr_LH_THIGH : public TransformMotion<Scalar, Type_fr_LH_HIP_X_fr_LH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_HIP_X_fr_LH_THIGH();
        const Type_fr_LH_HIP_X_fr_LH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_shank_fixed_X_fr_LH_THIGH : public TransformMotion<Scalar, Type_fr_LH_shank_fixed_X_fr_LH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_shank_fixed_X_fr_LH_THIGH();
        const Type_fr_LH_shank_fixed_X_fr_LH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_THIGH_X_fr_LH_shank_fixed : public TransformMotion<Scalar, Type_fr_LH_THIGH_X_fr_LH_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_THIGH_X_fr_LH_shank_fixed();
        const Type_fr_LH_THIGH_X_fr_LH_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_WHEEL_L_X_fr_LH_shank_fixed : public TransformMotion<Scalar, Type_fr_LH_WHEEL_L_X_fr_LH_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_WHEEL_L_X_fr_LH_shank_fixed();
        const Type_fr_LH_WHEEL_L_X_fr_LH_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_shank_fixed_X_fr_LH_WHEEL_L : public TransformMotion<Scalar, Type_fr_LH_shank_fixed_X_fr_LH_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_shank_fixed_X_fr_LH_WHEEL_L();
        const Type_fr_LH_shank_fixed_X_fr_LH_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_HIP_X_fr_base : public TransformMotion<Scalar, Type_fr_RH_HIP_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_HIP_X_fr_base();
        const Type_fr_RH_HIP_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_HIP : public TransformMotion<Scalar, Type_fr_base_X_fr_RH_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_HIP();
        const Type_fr_base_X_fr_RH_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_THIGH_X_fr_RH_HIP : public TransformMotion<Scalar, Type_fr_RH_THIGH_X_fr_RH_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_THIGH_X_fr_RH_HIP();
        const Type_fr_RH_THIGH_X_fr_RH_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_HIP_X_fr_RH_THIGH : public TransformMotion<Scalar, Type_fr_RH_HIP_X_fr_RH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_HIP_X_fr_RH_THIGH();
        const Type_fr_RH_HIP_X_fr_RH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_shank_fixed_X_fr_RH_THIGH : public TransformMotion<Scalar, Type_fr_RH_shank_fixed_X_fr_RH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_shank_fixed_X_fr_RH_THIGH();
        const Type_fr_RH_shank_fixed_X_fr_RH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_THIGH_X_fr_RH_shank_fixed : public TransformMotion<Scalar, Type_fr_RH_THIGH_X_fr_RH_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_THIGH_X_fr_RH_shank_fixed();
        const Type_fr_RH_THIGH_X_fr_RH_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_WHEEL_L_X_fr_RH_shank_fixed : public TransformMotion<Scalar, Type_fr_RH_WHEEL_L_X_fr_RH_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_WHEEL_L_X_fr_RH_shank_fixed();
        const Type_fr_RH_WHEEL_L_X_fr_RH_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_shank_fixed_X_fr_RH_WHEEL_L : public TransformMotion<Scalar, Type_fr_RH_shank_fixed_X_fr_RH_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_shank_fixed_X_fr_RH_WHEEL_L();
        const Type_fr_RH_shank_fixed_X_fr_RH_WHEEL_L& update(const JState&);
    protected:
    };
    
public:
    MotionTransforms();
    void updateParameters();
    Type_fr_base_X_fr_LF_WHEEL_L fr_base_X_fr_LF_WHEEL_L;
    Type_fr_LF_WHEEL_L_X_fr_base fr_LF_WHEEL_L_X_fr_base;
    Type_fr_base_X_fr_LH_WHEEL_L fr_base_X_fr_LH_WHEEL_L;
    Type_fr_LH_WHEEL_L_X_fr_base fr_LH_WHEEL_L_X_fr_base;
    Type_fr_base_X_fr_RF_WHEEL_L fr_base_X_fr_RF_WHEEL_L;
    Type_fr_RF_WHEEL_L_X_fr_base fr_RF_WHEEL_L_X_fr_base;
    Type_fr_base_X_fr_RH_WHEEL_L fr_base_X_fr_RH_WHEEL_L;
    Type_fr_RH_WHEEL_L_X_fr_base fr_RH_WHEEL_L_X_fr_base;
    Type_fr_base_X_fr_LF_WHEEL_L_COM fr_base_X_fr_LF_WHEEL_L_COM;
    Type_fr_LF_WHEEL_L_COM_X_fr_base fr_LF_WHEEL_L_COM_X_fr_base;
    Type_fr_base_X_fr_LH_WHEEL_L_COM fr_base_X_fr_LH_WHEEL_L_COM;
    Type_fr_LH_WHEEL_L_COM_X_fr_base fr_LH_WHEEL_L_COM_X_fr_base;
    Type_fr_base_X_fr_RF_WHEEL_L_COM fr_base_X_fr_RF_WHEEL_L_COM;
    Type_fr_RF_WHEEL_L_COM_X_fr_base fr_RF_WHEEL_L_COM_X_fr_base;
    Type_fr_base_X_fr_RH_WHEEL_L_COM fr_base_X_fr_RH_WHEEL_L_COM;
    Type_fr_RH_WHEEL_L_COM_X_fr_base fr_RH_WHEEL_L_COM_X_fr_base;
    Type_fr_base_X_fr_LF_HAA fr_base_X_fr_LF_HAA;
    Type_fr_base_X_fr_LF_HFE fr_base_X_fr_LF_HFE;
    Type_fr_base_X_fr_LF_KFE fr_base_X_fr_LF_KFE;
    Type_fr_base_X_fr_LF_WHEEL fr_base_X_fr_LF_WHEEL;
    Type_fr_base_X_fr_LH_HAA fr_base_X_fr_LH_HAA;
    Type_fr_base_X_fr_LH_HFE fr_base_X_fr_LH_HFE;
    Type_fr_base_X_fr_LH_KFE fr_base_X_fr_LH_KFE;
    Type_fr_base_X_fr_LH_WHEEL fr_base_X_fr_LH_WHEEL;
    Type_fr_base_X_fr_RF_HAA fr_base_X_fr_RF_HAA;
    Type_fr_base_X_fr_RF_HFE fr_base_X_fr_RF_HFE;
    Type_fr_base_X_fr_RF_KFE fr_base_X_fr_RF_KFE;
    Type_fr_base_X_fr_RF_WHEEL fr_base_X_fr_RF_WHEEL;
    Type_fr_base_X_fr_RH_HAA fr_base_X_fr_RH_HAA;
    Type_fr_base_X_fr_RH_HFE fr_base_X_fr_RH_HFE;
    Type_fr_base_X_fr_RH_KFE fr_base_X_fr_RH_KFE;
    Type_fr_base_X_fr_RH_WHEEL fr_base_X_fr_RH_WHEEL;
    Type_fr_LF_HIP_X_fr_base fr_LF_HIP_X_fr_base;
    Type_fr_base_X_fr_LF_HIP fr_base_X_fr_LF_HIP;
    Type_fr_LF_THIGH_X_fr_LF_HIP fr_LF_THIGH_X_fr_LF_HIP;
    Type_fr_LF_HIP_X_fr_LF_THIGH fr_LF_HIP_X_fr_LF_THIGH;
    Type_fr_LF_shank_fixed_X_fr_LF_THIGH fr_LF_shank_fixed_X_fr_LF_THIGH;
    Type_fr_LF_THIGH_X_fr_LF_shank_fixed fr_LF_THIGH_X_fr_LF_shank_fixed;
    Type_fr_LF_WHEEL_L_X_fr_LF_shank_fixed fr_LF_WHEEL_L_X_fr_LF_shank_fixed;
    Type_fr_LF_shank_fixed_X_fr_LF_WHEEL_L fr_LF_shank_fixed_X_fr_LF_WHEEL_L;
    Type_fr_RF_HIP_X_fr_base fr_RF_HIP_X_fr_base;
    Type_fr_base_X_fr_RF_HIP fr_base_X_fr_RF_HIP;
    Type_fr_RF_THIGH_X_fr_RF_HIP fr_RF_THIGH_X_fr_RF_HIP;
    Type_fr_RF_HIP_X_fr_RF_THIGH fr_RF_HIP_X_fr_RF_THIGH;
    Type_fr_RF_shank_fixed_X_fr_RF_THIGH fr_RF_shank_fixed_X_fr_RF_THIGH;
    Type_fr_RF_THIGH_X_fr_RF_shank_fixed fr_RF_THIGH_X_fr_RF_shank_fixed;
    Type_fr_RF_WHEEL_L_X_fr_RF_shank_fixed fr_RF_WHEEL_L_X_fr_RF_shank_fixed;
    Type_fr_RF_shank_fixed_X_fr_RF_WHEEL_L fr_RF_shank_fixed_X_fr_RF_WHEEL_L;
    Type_fr_LH_HIP_X_fr_base fr_LH_HIP_X_fr_base;
    Type_fr_base_X_fr_LH_HIP fr_base_X_fr_LH_HIP;
    Type_fr_LH_THIGH_X_fr_LH_HIP fr_LH_THIGH_X_fr_LH_HIP;
    Type_fr_LH_HIP_X_fr_LH_THIGH fr_LH_HIP_X_fr_LH_THIGH;
    Type_fr_LH_shank_fixed_X_fr_LH_THIGH fr_LH_shank_fixed_X_fr_LH_THIGH;
    Type_fr_LH_THIGH_X_fr_LH_shank_fixed fr_LH_THIGH_X_fr_LH_shank_fixed;
    Type_fr_LH_WHEEL_L_X_fr_LH_shank_fixed fr_LH_WHEEL_L_X_fr_LH_shank_fixed;
    Type_fr_LH_shank_fixed_X_fr_LH_WHEEL_L fr_LH_shank_fixed_X_fr_LH_WHEEL_L;
    Type_fr_RH_HIP_X_fr_base fr_RH_HIP_X_fr_base;
    Type_fr_base_X_fr_RH_HIP fr_base_X_fr_RH_HIP;
    Type_fr_RH_THIGH_X_fr_RH_HIP fr_RH_THIGH_X_fr_RH_HIP;
    Type_fr_RH_HIP_X_fr_RH_THIGH fr_RH_HIP_X_fr_RH_THIGH;
    Type_fr_RH_shank_fixed_X_fr_RH_THIGH fr_RH_shank_fixed_X_fr_RH_THIGH;
    Type_fr_RH_THIGH_X_fr_RH_shank_fixed fr_RH_THIGH_X_fr_RH_shank_fixed;
    Type_fr_RH_WHEEL_L_X_fr_RH_shank_fixed fr_RH_WHEEL_L_X_fr_RH_shank_fixed;
    Type_fr_RH_shank_fixed_X_fr_RH_WHEEL_L fr_RH_shank_fixed_X_fr_RH_WHEEL_L;

protected:

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
template <typename TRAIT>
class ForceTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformForce<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_base_X_fr_LF_WHEEL_L : public TransformForce<Scalar, Type_fr_base_X_fr_LF_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_WHEEL_L();
        const Type_fr_base_X_fr_LF_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_WHEEL_L_X_fr_base : public TransformForce<Scalar, Type_fr_LF_WHEEL_L_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_WHEEL_L_X_fr_base();
        const Type_fr_LF_WHEEL_L_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_WHEEL_L : public TransformForce<Scalar, Type_fr_base_X_fr_LH_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_WHEEL_L();
        const Type_fr_base_X_fr_LH_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_WHEEL_L_X_fr_base : public TransformForce<Scalar, Type_fr_LH_WHEEL_L_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_WHEEL_L_X_fr_base();
        const Type_fr_LH_WHEEL_L_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_WHEEL_L : public TransformForce<Scalar, Type_fr_base_X_fr_RF_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_WHEEL_L();
        const Type_fr_base_X_fr_RF_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_WHEEL_L_X_fr_base : public TransformForce<Scalar, Type_fr_RF_WHEEL_L_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_WHEEL_L_X_fr_base();
        const Type_fr_RF_WHEEL_L_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_WHEEL_L : public TransformForce<Scalar, Type_fr_base_X_fr_RH_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_WHEEL_L();
        const Type_fr_base_X_fr_RH_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_WHEEL_L_X_fr_base : public TransformForce<Scalar, Type_fr_RH_WHEEL_L_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_WHEEL_L_X_fr_base();
        const Type_fr_RH_WHEEL_L_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_WHEEL_L_COM : public TransformForce<Scalar, Type_fr_base_X_fr_LF_WHEEL_L_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_WHEEL_L_COM();
        const Type_fr_base_X_fr_LF_WHEEL_L_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_WHEEL_L_COM_X_fr_base : public TransformForce<Scalar, Type_fr_LF_WHEEL_L_COM_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_WHEEL_L_COM_X_fr_base();
        const Type_fr_LF_WHEEL_L_COM_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_WHEEL_L_COM : public TransformForce<Scalar, Type_fr_base_X_fr_LH_WHEEL_L_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_WHEEL_L_COM();
        const Type_fr_base_X_fr_LH_WHEEL_L_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_WHEEL_L_COM_X_fr_base : public TransformForce<Scalar, Type_fr_LH_WHEEL_L_COM_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_WHEEL_L_COM_X_fr_base();
        const Type_fr_LH_WHEEL_L_COM_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_WHEEL_L_COM : public TransformForce<Scalar, Type_fr_base_X_fr_RF_WHEEL_L_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_WHEEL_L_COM();
        const Type_fr_base_X_fr_RF_WHEEL_L_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_WHEEL_L_COM_X_fr_base : public TransformForce<Scalar, Type_fr_RF_WHEEL_L_COM_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_WHEEL_L_COM_X_fr_base();
        const Type_fr_RF_WHEEL_L_COM_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_WHEEL_L_COM : public TransformForce<Scalar, Type_fr_base_X_fr_RH_WHEEL_L_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_WHEEL_L_COM();
        const Type_fr_base_X_fr_RH_WHEEL_L_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_WHEEL_L_COM_X_fr_base : public TransformForce<Scalar, Type_fr_RH_WHEEL_L_COM_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_WHEEL_L_COM_X_fr_base();
        const Type_fr_RH_WHEEL_L_COM_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_HAA : public TransformForce<Scalar, Type_fr_base_X_fr_LF_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_HAA();
        const Type_fr_base_X_fr_LF_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_HFE : public TransformForce<Scalar, Type_fr_base_X_fr_LF_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_HFE();
        const Type_fr_base_X_fr_LF_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_KFE : public TransformForce<Scalar, Type_fr_base_X_fr_LF_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_KFE();
        const Type_fr_base_X_fr_LF_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_WHEEL : public TransformForce<Scalar, Type_fr_base_X_fr_LF_WHEEL>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_WHEEL();
        const Type_fr_base_X_fr_LF_WHEEL& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_HAA : public TransformForce<Scalar, Type_fr_base_X_fr_LH_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_HAA();
        const Type_fr_base_X_fr_LH_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_HFE : public TransformForce<Scalar, Type_fr_base_X_fr_LH_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_HFE();
        const Type_fr_base_X_fr_LH_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_KFE : public TransformForce<Scalar, Type_fr_base_X_fr_LH_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_KFE();
        const Type_fr_base_X_fr_LH_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_WHEEL : public TransformForce<Scalar, Type_fr_base_X_fr_LH_WHEEL>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_WHEEL();
        const Type_fr_base_X_fr_LH_WHEEL& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_HAA : public TransformForce<Scalar, Type_fr_base_X_fr_RF_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_HAA();
        const Type_fr_base_X_fr_RF_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_HFE : public TransformForce<Scalar, Type_fr_base_X_fr_RF_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_HFE();
        const Type_fr_base_X_fr_RF_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_KFE : public TransformForce<Scalar, Type_fr_base_X_fr_RF_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_KFE();
        const Type_fr_base_X_fr_RF_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_WHEEL : public TransformForce<Scalar, Type_fr_base_X_fr_RF_WHEEL>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_WHEEL();
        const Type_fr_base_X_fr_RF_WHEEL& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_HAA : public TransformForce<Scalar, Type_fr_base_X_fr_RH_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_HAA();
        const Type_fr_base_X_fr_RH_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_HFE : public TransformForce<Scalar, Type_fr_base_X_fr_RH_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_HFE();
        const Type_fr_base_X_fr_RH_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_KFE : public TransformForce<Scalar, Type_fr_base_X_fr_RH_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_KFE();
        const Type_fr_base_X_fr_RH_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_WHEEL : public TransformForce<Scalar, Type_fr_base_X_fr_RH_WHEEL>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_WHEEL();
        const Type_fr_base_X_fr_RH_WHEEL& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_HIP_X_fr_base : public TransformForce<Scalar, Type_fr_LF_HIP_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_HIP_X_fr_base();
        const Type_fr_LF_HIP_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_HIP : public TransformForce<Scalar, Type_fr_base_X_fr_LF_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_HIP();
        const Type_fr_base_X_fr_LF_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_THIGH_X_fr_LF_HIP : public TransformForce<Scalar, Type_fr_LF_THIGH_X_fr_LF_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_THIGH_X_fr_LF_HIP();
        const Type_fr_LF_THIGH_X_fr_LF_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_HIP_X_fr_LF_THIGH : public TransformForce<Scalar, Type_fr_LF_HIP_X_fr_LF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_HIP_X_fr_LF_THIGH();
        const Type_fr_LF_HIP_X_fr_LF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_shank_fixed_X_fr_LF_THIGH : public TransformForce<Scalar, Type_fr_LF_shank_fixed_X_fr_LF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_shank_fixed_X_fr_LF_THIGH();
        const Type_fr_LF_shank_fixed_X_fr_LF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_THIGH_X_fr_LF_shank_fixed : public TransformForce<Scalar, Type_fr_LF_THIGH_X_fr_LF_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_THIGH_X_fr_LF_shank_fixed();
        const Type_fr_LF_THIGH_X_fr_LF_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_WHEEL_L_X_fr_LF_shank_fixed : public TransformForce<Scalar, Type_fr_LF_WHEEL_L_X_fr_LF_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_WHEEL_L_X_fr_LF_shank_fixed();
        const Type_fr_LF_WHEEL_L_X_fr_LF_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_shank_fixed_X_fr_LF_WHEEL_L : public TransformForce<Scalar, Type_fr_LF_shank_fixed_X_fr_LF_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_shank_fixed_X_fr_LF_WHEEL_L();
        const Type_fr_LF_shank_fixed_X_fr_LF_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_HIP_X_fr_base : public TransformForce<Scalar, Type_fr_RF_HIP_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_HIP_X_fr_base();
        const Type_fr_RF_HIP_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_HIP : public TransformForce<Scalar, Type_fr_base_X_fr_RF_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_HIP();
        const Type_fr_base_X_fr_RF_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_THIGH_X_fr_RF_HIP : public TransformForce<Scalar, Type_fr_RF_THIGH_X_fr_RF_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_THIGH_X_fr_RF_HIP();
        const Type_fr_RF_THIGH_X_fr_RF_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_HIP_X_fr_RF_THIGH : public TransformForce<Scalar, Type_fr_RF_HIP_X_fr_RF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_HIP_X_fr_RF_THIGH();
        const Type_fr_RF_HIP_X_fr_RF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_shank_fixed_X_fr_RF_THIGH : public TransformForce<Scalar, Type_fr_RF_shank_fixed_X_fr_RF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_shank_fixed_X_fr_RF_THIGH();
        const Type_fr_RF_shank_fixed_X_fr_RF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_THIGH_X_fr_RF_shank_fixed : public TransformForce<Scalar, Type_fr_RF_THIGH_X_fr_RF_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_THIGH_X_fr_RF_shank_fixed();
        const Type_fr_RF_THIGH_X_fr_RF_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_WHEEL_L_X_fr_RF_shank_fixed : public TransformForce<Scalar, Type_fr_RF_WHEEL_L_X_fr_RF_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_WHEEL_L_X_fr_RF_shank_fixed();
        const Type_fr_RF_WHEEL_L_X_fr_RF_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_shank_fixed_X_fr_RF_WHEEL_L : public TransformForce<Scalar, Type_fr_RF_shank_fixed_X_fr_RF_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_shank_fixed_X_fr_RF_WHEEL_L();
        const Type_fr_RF_shank_fixed_X_fr_RF_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_HIP_X_fr_base : public TransformForce<Scalar, Type_fr_LH_HIP_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_HIP_X_fr_base();
        const Type_fr_LH_HIP_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_HIP : public TransformForce<Scalar, Type_fr_base_X_fr_LH_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_HIP();
        const Type_fr_base_X_fr_LH_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_THIGH_X_fr_LH_HIP : public TransformForce<Scalar, Type_fr_LH_THIGH_X_fr_LH_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_THIGH_X_fr_LH_HIP();
        const Type_fr_LH_THIGH_X_fr_LH_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_HIP_X_fr_LH_THIGH : public TransformForce<Scalar, Type_fr_LH_HIP_X_fr_LH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_HIP_X_fr_LH_THIGH();
        const Type_fr_LH_HIP_X_fr_LH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_shank_fixed_X_fr_LH_THIGH : public TransformForce<Scalar, Type_fr_LH_shank_fixed_X_fr_LH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_shank_fixed_X_fr_LH_THIGH();
        const Type_fr_LH_shank_fixed_X_fr_LH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_THIGH_X_fr_LH_shank_fixed : public TransformForce<Scalar, Type_fr_LH_THIGH_X_fr_LH_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_THIGH_X_fr_LH_shank_fixed();
        const Type_fr_LH_THIGH_X_fr_LH_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_WHEEL_L_X_fr_LH_shank_fixed : public TransformForce<Scalar, Type_fr_LH_WHEEL_L_X_fr_LH_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_WHEEL_L_X_fr_LH_shank_fixed();
        const Type_fr_LH_WHEEL_L_X_fr_LH_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_shank_fixed_X_fr_LH_WHEEL_L : public TransformForce<Scalar, Type_fr_LH_shank_fixed_X_fr_LH_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_shank_fixed_X_fr_LH_WHEEL_L();
        const Type_fr_LH_shank_fixed_X_fr_LH_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_HIP_X_fr_base : public TransformForce<Scalar, Type_fr_RH_HIP_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_HIP_X_fr_base();
        const Type_fr_RH_HIP_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_HIP : public TransformForce<Scalar, Type_fr_base_X_fr_RH_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_HIP();
        const Type_fr_base_X_fr_RH_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_THIGH_X_fr_RH_HIP : public TransformForce<Scalar, Type_fr_RH_THIGH_X_fr_RH_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_THIGH_X_fr_RH_HIP();
        const Type_fr_RH_THIGH_X_fr_RH_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_HIP_X_fr_RH_THIGH : public TransformForce<Scalar, Type_fr_RH_HIP_X_fr_RH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_HIP_X_fr_RH_THIGH();
        const Type_fr_RH_HIP_X_fr_RH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_shank_fixed_X_fr_RH_THIGH : public TransformForce<Scalar, Type_fr_RH_shank_fixed_X_fr_RH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_shank_fixed_X_fr_RH_THIGH();
        const Type_fr_RH_shank_fixed_X_fr_RH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_THIGH_X_fr_RH_shank_fixed : public TransformForce<Scalar, Type_fr_RH_THIGH_X_fr_RH_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_THIGH_X_fr_RH_shank_fixed();
        const Type_fr_RH_THIGH_X_fr_RH_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_WHEEL_L_X_fr_RH_shank_fixed : public TransformForce<Scalar, Type_fr_RH_WHEEL_L_X_fr_RH_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_WHEEL_L_X_fr_RH_shank_fixed();
        const Type_fr_RH_WHEEL_L_X_fr_RH_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_shank_fixed_X_fr_RH_WHEEL_L : public TransformForce<Scalar, Type_fr_RH_shank_fixed_X_fr_RH_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_shank_fixed_X_fr_RH_WHEEL_L();
        const Type_fr_RH_shank_fixed_X_fr_RH_WHEEL_L& update(const JState&);
    protected:
    };
    
public:
    ForceTransforms();
    void updateParameters();
    Type_fr_base_X_fr_LF_WHEEL_L fr_base_X_fr_LF_WHEEL_L;
    Type_fr_LF_WHEEL_L_X_fr_base fr_LF_WHEEL_L_X_fr_base;
    Type_fr_base_X_fr_LH_WHEEL_L fr_base_X_fr_LH_WHEEL_L;
    Type_fr_LH_WHEEL_L_X_fr_base fr_LH_WHEEL_L_X_fr_base;
    Type_fr_base_X_fr_RF_WHEEL_L fr_base_X_fr_RF_WHEEL_L;
    Type_fr_RF_WHEEL_L_X_fr_base fr_RF_WHEEL_L_X_fr_base;
    Type_fr_base_X_fr_RH_WHEEL_L fr_base_X_fr_RH_WHEEL_L;
    Type_fr_RH_WHEEL_L_X_fr_base fr_RH_WHEEL_L_X_fr_base;
    Type_fr_base_X_fr_LF_WHEEL_L_COM fr_base_X_fr_LF_WHEEL_L_COM;
    Type_fr_LF_WHEEL_L_COM_X_fr_base fr_LF_WHEEL_L_COM_X_fr_base;
    Type_fr_base_X_fr_LH_WHEEL_L_COM fr_base_X_fr_LH_WHEEL_L_COM;
    Type_fr_LH_WHEEL_L_COM_X_fr_base fr_LH_WHEEL_L_COM_X_fr_base;
    Type_fr_base_X_fr_RF_WHEEL_L_COM fr_base_X_fr_RF_WHEEL_L_COM;
    Type_fr_RF_WHEEL_L_COM_X_fr_base fr_RF_WHEEL_L_COM_X_fr_base;
    Type_fr_base_X_fr_RH_WHEEL_L_COM fr_base_X_fr_RH_WHEEL_L_COM;
    Type_fr_RH_WHEEL_L_COM_X_fr_base fr_RH_WHEEL_L_COM_X_fr_base;
    Type_fr_base_X_fr_LF_HAA fr_base_X_fr_LF_HAA;
    Type_fr_base_X_fr_LF_HFE fr_base_X_fr_LF_HFE;
    Type_fr_base_X_fr_LF_KFE fr_base_X_fr_LF_KFE;
    Type_fr_base_X_fr_LF_WHEEL fr_base_X_fr_LF_WHEEL;
    Type_fr_base_X_fr_LH_HAA fr_base_X_fr_LH_HAA;
    Type_fr_base_X_fr_LH_HFE fr_base_X_fr_LH_HFE;
    Type_fr_base_X_fr_LH_KFE fr_base_X_fr_LH_KFE;
    Type_fr_base_X_fr_LH_WHEEL fr_base_X_fr_LH_WHEEL;
    Type_fr_base_X_fr_RF_HAA fr_base_X_fr_RF_HAA;
    Type_fr_base_X_fr_RF_HFE fr_base_X_fr_RF_HFE;
    Type_fr_base_X_fr_RF_KFE fr_base_X_fr_RF_KFE;
    Type_fr_base_X_fr_RF_WHEEL fr_base_X_fr_RF_WHEEL;
    Type_fr_base_X_fr_RH_HAA fr_base_X_fr_RH_HAA;
    Type_fr_base_X_fr_RH_HFE fr_base_X_fr_RH_HFE;
    Type_fr_base_X_fr_RH_KFE fr_base_X_fr_RH_KFE;
    Type_fr_base_X_fr_RH_WHEEL fr_base_X_fr_RH_WHEEL;
    Type_fr_LF_HIP_X_fr_base fr_LF_HIP_X_fr_base;
    Type_fr_base_X_fr_LF_HIP fr_base_X_fr_LF_HIP;
    Type_fr_LF_THIGH_X_fr_LF_HIP fr_LF_THIGH_X_fr_LF_HIP;
    Type_fr_LF_HIP_X_fr_LF_THIGH fr_LF_HIP_X_fr_LF_THIGH;
    Type_fr_LF_shank_fixed_X_fr_LF_THIGH fr_LF_shank_fixed_X_fr_LF_THIGH;
    Type_fr_LF_THIGH_X_fr_LF_shank_fixed fr_LF_THIGH_X_fr_LF_shank_fixed;
    Type_fr_LF_WHEEL_L_X_fr_LF_shank_fixed fr_LF_WHEEL_L_X_fr_LF_shank_fixed;
    Type_fr_LF_shank_fixed_X_fr_LF_WHEEL_L fr_LF_shank_fixed_X_fr_LF_WHEEL_L;
    Type_fr_RF_HIP_X_fr_base fr_RF_HIP_X_fr_base;
    Type_fr_base_X_fr_RF_HIP fr_base_X_fr_RF_HIP;
    Type_fr_RF_THIGH_X_fr_RF_HIP fr_RF_THIGH_X_fr_RF_HIP;
    Type_fr_RF_HIP_X_fr_RF_THIGH fr_RF_HIP_X_fr_RF_THIGH;
    Type_fr_RF_shank_fixed_X_fr_RF_THIGH fr_RF_shank_fixed_X_fr_RF_THIGH;
    Type_fr_RF_THIGH_X_fr_RF_shank_fixed fr_RF_THIGH_X_fr_RF_shank_fixed;
    Type_fr_RF_WHEEL_L_X_fr_RF_shank_fixed fr_RF_WHEEL_L_X_fr_RF_shank_fixed;
    Type_fr_RF_shank_fixed_X_fr_RF_WHEEL_L fr_RF_shank_fixed_X_fr_RF_WHEEL_L;
    Type_fr_LH_HIP_X_fr_base fr_LH_HIP_X_fr_base;
    Type_fr_base_X_fr_LH_HIP fr_base_X_fr_LH_HIP;
    Type_fr_LH_THIGH_X_fr_LH_HIP fr_LH_THIGH_X_fr_LH_HIP;
    Type_fr_LH_HIP_X_fr_LH_THIGH fr_LH_HIP_X_fr_LH_THIGH;
    Type_fr_LH_shank_fixed_X_fr_LH_THIGH fr_LH_shank_fixed_X_fr_LH_THIGH;
    Type_fr_LH_THIGH_X_fr_LH_shank_fixed fr_LH_THIGH_X_fr_LH_shank_fixed;
    Type_fr_LH_WHEEL_L_X_fr_LH_shank_fixed fr_LH_WHEEL_L_X_fr_LH_shank_fixed;
    Type_fr_LH_shank_fixed_X_fr_LH_WHEEL_L fr_LH_shank_fixed_X_fr_LH_WHEEL_L;
    Type_fr_RH_HIP_X_fr_base fr_RH_HIP_X_fr_base;
    Type_fr_base_X_fr_RH_HIP fr_base_X_fr_RH_HIP;
    Type_fr_RH_THIGH_X_fr_RH_HIP fr_RH_THIGH_X_fr_RH_HIP;
    Type_fr_RH_HIP_X_fr_RH_THIGH fr_RH_HIP_X_fr_RH_THIGH;
    Type_fr_RH_shank_fixed_X_fr_RH_THIGH fr_RH_shank_fixed_X_fr_RH_THIGH;
    Type_fr_RH_THIGH_X_fr_RH_shank_fixed fr_RH_THIGH_X_fr_RH_shank_fixed;
    Type_fr_RH_WHEEL_L_X_fr_RH_shank_fixed fr_RH_WHEEL_L_X_fr_RH_shank_fixed;
    Type_fr_RH_shank_fixed_X_fr_RH_WHEEL_L fr_RH_shank_fixed_X_fr_RH_WHEEL_L;

protected:

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
template <typename TRAIT>
class HomogeneousTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformHomogeneous<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_base_X_fr_LF_WHEEL_L : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_LF_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_WHEEL_L();
        const Type_fr_base_X_fr_LF_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_WHEEL_L_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_LF_WHEEL_L_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_WHEEL_L_X_fr_base();
        const Type_fr_LF_WHEEL_L_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_WHEEL_L : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_LH_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_WHEEL_L();
        const Type_fr_base_X_fr_LH_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_WHEEL_L_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_LH_WHEEL_L_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_WHEEL_L_X_fr_base();
        const Type_fr_LH_WHEEL_L_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_WHEEL_L : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_RF_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_WHEEL_L();
        const Type_fr_base_X_fr_RF_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_WHEEL_L_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_RF_WHEEL_L_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_WHEEL_L_X_fr_base();
        const Type_fr_RF_WHEEL_L_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_WHEEL_L : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_RH_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_WHEEL_L();
        const Type_fr_base_X_fr_RH_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_WHEEL_L_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_RH_WHEEL_L_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_WHEEL_L_X_fr_base();
        const Type_fr_RH_WHEEL_L_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_WHEEL_L_COM : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_LF_WHEEL_L_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_WHEEL_L_COM();
        const Type_fr_base_X_fr_LF_WHEEL_L_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_WHEEL_L_COM_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_LF_WHEEL_L_COM_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_WHEEL_L_COM_X_fr_base();
        const Type_fr_LF_WHEEL_L_COM_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_WHEEL_L_COM : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_LH_WHEEL_L_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_WHEEL_L_COM();
        const Type_fr_base_X_fr_LH_WHEEL_L_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_WHEEL_L_COM_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_LH_WHEEL_L_COM_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_WHEEL_L_COM_X_fr_base();
        const Type_fr_LH_WHEEL_L_COM_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_WHEEL_L_COM : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_RF_WHEEL_L_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_WHEEL_L_COM();
        const Type_fr_base_X_fr_RF_WHEEL_L_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_WHEEL_L_COM_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_RF_WHEEL_L_COM_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_WHEEL_L_COM_X_fr_base();
        const Type_fr_RF_WHEEL_L_COM_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_WHEEL_L_COM : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_RH_WHEEL_L_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_WHEEL_L_COM();
        const Type_fr_base_X_fr_RH_WHEEL_L_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_WHEEL_L_COM_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_RH_WHEEL_L_COM_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_WHEEL_L_COM_X_fr_base();
        const Type_fr_RH_WHEEL_L_COM_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_HAA : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_LF_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_HAA();
        const Type_fr_base_X_fr_LF_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_HFE : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_LF_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_HFE();
        const Type_fr_base_X_fr_LF_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_KFE : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_LF_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_KFE();
        const Type_fr_base_X_fr_LF_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_WHEEL : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_LF_WHEEL>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_WHEEL();
        const Type_fr_base_X_fr_LF_WHEEL& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_HAA : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_LH_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_HAA();
        const Type_fr_base_X_fr_LH_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_HFE : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_LH_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_HFE();
        const Type_fr_base_X_fr_LH_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_KFE : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_LH_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_KFE();
        const Type_fr_base_X_fr_LH_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_WHEEL : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_LH_WHEEL>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_WHEEL();
        const Type_fr_base_X_fr_LH_WHEEL& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_HAA : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_RF_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_HAA();
        const Type_fr_base_X_fr_RF_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_HFE : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_RF_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_HFE();
        const Type_fr_base_X_fr_RF_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_KFE : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_RF_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_KFE();
        const Type_fr_base_X_fr_RF_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_WHEEL : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_RF_WHEEL>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_WHEEL();
        const Type_fr_base_X_fr_RF_WHEEL& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_HAA : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_RH_HAA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_HAA();
        const Type_fr_base_X_fr_RH_HAA& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_HFE : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_RH_HFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_HFE();
        const Type_fr_base_X_fr_RH_HFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_KFE : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_RH_KFE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_KFE();
        const Type_fr_base_X_fr_RH_KFE& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_WHEEL : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_RH_WHEEL>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_WHEEL();
        const Type_fr_base_X_fr_RH_WHEEL& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_HIP_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_LF_HIP_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_HIP_X_fr_base();
        const Type_fr_LF_HIP_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LF_HIP : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_LF_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_HIP();
        const Type_fr_base_X_fr_LF_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_THIGH_X_fr_LF_HIP : public TransformHomogeneous<Scalar, Type_fr_LF_THIGH_X_fr_LF_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_THIGH_X_fr_LF_HIP();
        const Type_fr_LF_THIGH_X_fr_LF_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_HIP_X_fr_LF_THIGH : public TransformHomogeneous<Scalar, Type_fr_LF_HIP_X_fr_LF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_HIP_X_fr_LF_THIGH();
        const Type_fr_LF_HIP_X_fr_LF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_shank_fixed_X_fr_LF_THIGH : public TransformHomogeneous<Scalar, Type_fr_LF_shank_fixed_X_fr_LF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_shank_fixed_X_fr_LF_THIGH();
        const Type_fr_LF_shank_fixed_X_fr_LF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_THIGH_X_fr_LF_shank_fixed : public TransformHomogeneous<Scalar, Type_fr_LF_THIGH_X_fr_LF_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_THIGH_X_fr_LF_shank_fixed();
        const Type_fr_LF_THIGH_X_fr_LF_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_WHEEL_L_X_fr_LF_shank_fixed : public TransformHomogeneous<Scalar, Type_fr_LF_WHEEL_L_X_fr_LF_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_WHEEL_L_X_fr_LF_shank_fixed();
        const Type_fr_LF_WHEEL_L_X_fr_LF_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_shank_fixed_X_fr_LF_WHEEL_L : public TransformHomogeneous<Scalar, Type_fr_LF_shank_fixed_X_fr_LF_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_shank_fixed_X_fr_LF_WHEEL_L();
        const Type_fr_LF_shank_fixed_X_fr_LF_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_HIP_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_RF_HIP_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_HIP_X_fr_base();
        const Type_fr_RF_HIP_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_HIP : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_RF_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_HIP();
        const Type_fr_base_X_fr_RF_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_THIGH_X_fr_RF_HIP : public TransformHomogeneous<Scalar, Type_fr_RF_THIGH_X_fr_RF_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_THIGH_X_fr_RF_HIP();
        const Type_fr_RF_THIGH_X_fr_RF_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_HIP_X_fr_RF_THIGH : public TransformHomogeneous<Scalar, Type_fr_RF_HIP_X_fr_RF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_HIP_X_fr_RF_THIGH();
        const Type_fr_RF_HIP_X_fr_RF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_shank_fixed_X_fr_RF_THIGH : public TransformHomogeneous<Scalar, Type_fr_RF_shank_fixed_X_fr_RF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_shank_fixed_X_fr_RF_THIGH();
        const Type_fr_RF_shank_fixed_X_fr_RF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_THIGH_X_fr_RF_shank_fixed : public TransformHomogeneous<Scalar, Type_fr_RF_THIGH_X_fr_RF_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_THIGH_X_fr_RF_shank_fixed();
        const Type_fr_RF_THIGH_X_fr_RF_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_WHEEL_L_X_fr_RF_shank_fixed : public TransformHomogeneous<Scalar, Type_fr_RF_WHEEL_L_X_fr_RF_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_WHEEL_L_X_fr_RF_shank_fixed();
        const Type_fr_RF_WHEEL_L_X_fr_RF_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_shank_fixed_X_fr_RF_WHEEL_L : public TransformHomogeneous<Scalar, Type_fr_RF_shank_fixed_X_fr_RF_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_shank_fixed_X_fr_RF_WHEEL_L();
        const Type_fr_RF_shank_fixed_X_fr_RF_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_HIP_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_LH_HIP_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_HIP_X_fr_base();
        const Type_fr_LH_HIP_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_HIP : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_LH_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_HIP();
        const Type_fr_base_X_fr_LH_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_THIGH_X_fr_LH_HIP : public TransformHomogeneous<Scalar, Type_fr_LH_THIGH_X_fr_LH_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_THIGH_X_fr_LH_HIP();
        const Type_fr_LH_THIGH_X_fr_LH_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_HIP_X_fr_LH_THIGH : public TransformHomogeneous<Scalar, Type_fr_LH_HIP_X_fr_LH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_HIP_X_fr_LH_THIGH();
        const Type_fr_LH_HIP_X_fr_LH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_shank_fixed_X_fr_LH_THIGH : public TransformHomogeneous<Scalar, Type_fr_LH_shank_fixed_X_fr_LH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_shank_fixed_X_fr_LH_THIGH();
        const Type_fr_LH_shank_fixed_X_fr_LH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_THIGH_X_fr_LH_shank_fixed : public TransformHomogeneous<Scalar, Type_fr_LH_THIGH_X_fr_LH_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_THIGH_X_fr_LH_shank_fixed();
        const Type_fr_LH_THIGH_X_fr_LH_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_WHEEL_L_X_fr_LH_shank_fixed : public TransformHomogeneous<Scalar, Type_fr_LH_WHEEL_L_X_fr_LH_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_WHEEL_L_X_fr_LH_shank_fixed();
        const Type_fr_LH_WHEEL_L_X_fr_LH_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_shank_fixed_X_fr_LH_WHEEL_L : public TransformHomogeneous<Scalar, Type_fr_LH_shank_fixed_X_fr_LH_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_shank_fixed_X_fr_LH_WHEEL_L();
        const Type_fr_LH_shank_fixed_X_fr_LH_WHEEL_L& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_HIP_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_RH_HIP_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_HIP_X_fr_base();
        const Type_fr_RH_HIP_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_HIP : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_RH_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_HIP();
        const Type_fr_base_X_fr_RH_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_THIGH_X_fr_RH_HIP : public TransformHomogeneous<Scalar, Type_fr_RH_THIGH_X_fr_RH_HIP>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_THIGH_X_fr_RH_HIP();
        const Type_fr_RH_THIGH_X_fr_RH_HIP& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_HIP_X_fr_RH_THIGH : public TransformHomogeneous<Scalar, Type_fr_RH_HIP_X_fr_RH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_HIP_X_fr_RH_THIGH();
        const Type_fr_RH_HIP_X_fr_RH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_shank_fixed_X_fr_RH_THIGH : public TransformHomogeneous<Scalar, Type_fr_RH_shank_fixed_X_fr_RH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_shank_fixed_X_fr_RH_THIGH();
        const Type_fr_RH_shank_fixed_X_fr_RH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_THIGH_X_fr_RH_shank_fixed : public TransformHomogeneous<Scalar, Type_fr_RH_THIGH_X_fr_RH_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_THIGH_X_fr_RH_shank_fixed();
        const Type_fr_RH_THIGH_X_fr_RH_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_WHEEL_L_X_fr_RH_shank_fixed : public TransformHomogeneous<Scalar, Type_fr_RH_WHEEL_L_X_fr_RH_shank_fixed>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_WHEEL_L_X_fr_RH_shank_fixed();
        const Type_fr_RH_WHEEL_L_X_fr_RH_shank_fixed& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_shank_fixed_X_fr_RH_WHEEL_L : public TransformHomogeneous<Scalar, Type_fr_RH_shank_fixed_X_fr_RH_WHEEL_L>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_shank_fixed_X_fr_RH_WHEEL_L();
        const Type_fr_RH_shank_fixed_X_fr_RH_WHEEL_L& update(const JState&);
    protected:
    };
    
public:
    HomogeneousTransforms();
    void updateParameters();
    Type_fr_base_X_fr_LF_WHEEL_L fr_base_X_fr_LF_WHEEL_L;
    Type_fr_LF_WHEEL_L_X_fr_base fr_LF_WHEEL_L_X_fr_base;
    Type_fr_base_X_fr_LH_WHEEL_L fr_base_X_fr_LH_WHEEL_L;
    Type_fr_LH_WHEEL_L_X_fr_base fr_LH_WHEEL_L_X_fr_base;
    Type_fr_base_X_fr_RF_WHEEL_L fr_base_X_fr_RF_WHEEL_L;
    Type_fr_RF_WHEEL_L_X_fr_base fr_RF_WHEEL_L_X_fr_base;
    Type_fr_base_X_fr_RH_WHEEL_L fr_base_X_fr_RH_WHEEL_L;
    Type_fr_RH_WHEEL_L_X_fr_base fr_RH_WHEEL_L_X_fr_base;
    Type_fr_base_X_fr_LF_WHEEL_L_COM fr_base_X_fr_LF_WHEEL_L_COM;
    Type_fr_LF_WHEEL_L_COM_X_fr_base fr_LF_WHEEL_L_COM_X_fr_base;
    Type_fr_base_X_fr_LH_WHEEL_L_COM fr_base_X_fr_LH_WHEEL_L_COM;
    Type_fr_LH_WHEEL_L_COM_X_fr_base fr_LH_WHEEL_L_COM_X_fr_base;
    Type_fr_base_X_fr_RF_WHEEL_L_COM fr_base_X_fr_RF_WHEEL_L_COM;
    Type_fr_RF_WHEEL_L_COM_X_fr_base fr_RF_WHEEL_L_COM_X_fr_base;
    Type_fr_base_X_fr_RH_WHEEL_L_COM fr_base_X_fr_RH_WHEEL_L_COM;
    Type_fr_RH_WHEEL_L_COM_X_fr_base fr_RH_WHEEL_L_COM_X_fr_base;
    Type_fr_base_X_fr_LF_HAA fr_base_X_fr_LF_HAA;
    Type_fr_base_X_fr_LF_HFE fr_base_X_fr_LF_HFE;
    Type_fr_base_X_fr_LF_KFE fr_base_X_fr_LF_KFE;
    Type_fr_base_X_fr_LF_WHEEL fr_base_X_fr_LF_WHEEL;
    Type_fr_base_X_fr_LH_HAA fr_base_X_fr_LH_HAA;
    Type_fr_base_X_fr_LH_HFE fr_base_X_fr_LH_HFE;
    Type_fr_base_X_fr_LH_KFE fr_base_X_fr_LH_KFE;
    Type_fr_base_X_fr_LH_WHEEL fr_base_X_fr_LH_WHEEL;
    Type_fr_base_X_fr_RF_HAA fr_base_X_fr_RF_HAA;
    Type_fr_base_X_fr_RF_HFE fr_base_X_fr_RF_HFE;
    Type_fr_base_X_fr_RF_KFE fr_base_X_fr_RF_KFE;
    Type_fr_base_X_fr_RF_WHEEL fr_base_X_fr_RF_WHEEL;
    Type_fr_base_X_fr_RH_HAA fr_base_X_fr_RH_HAA;
    Type_fr_base_X_fr_RH_HFE fr_base_X_fr_RH_HFE;
    Type_fr_base_X_fr_RH_KFE fr_base_X_fr_RH_KFE;
    Type_fr_base_X_fr_RH_WHEEL fr_base_X_fr_RH_WHEEL;
    Type_fr_LF_HIP_X_fr_base fr_LF_HIP_X_fr_base;
    Type_fr_base_X_fr_LF_HIP fr_base_X_fr_LF_HIP;
    Type_fr_LF_THIGH_X_fr_LF_HIP fr_LF_THIGH_X_fr_LF_HIP;
    Type_fr_LF_HIP_X_fr_LF_THIGH fr_LF_HIP_X_fr_LF_THIGH;
    Type_fr_LF_shank_fixed_X_fr_LF_THIGH fr_LF_shank_fixed_X_fr_LF_THIGH;
    Type_fr_LF_THIGH_X_fr_LF_shank_fixed fr_LF_THIGH_X_fr_LF_shank_fixed;
    Type_fr_LF_WHEEL_L_X_fr_LF_shank_fixed fr_LF_WHEEL_L_X_fr_LF_shank_fixed;
    Type_fr_LF_shank_fixed_X_fr_LF_WHEEL_L fr_LF_shank_fixed_X_fr_LF_WHEEL_L;
    Type_fr_RF_HIP_X_fr_base fr_RF_HIP_X_fr_base;
    Type_fr_base_X_fr_RF_HIP fr_base_X_fr_RF_HIP;
    Type_fr_RF_THIGH_X_fr_RF_HIP fr_RF_THIGH_X_fr_RF_HIP;
    Type_fr_RF_HIP_X_fr_RF_THIGH fr_RF_HIP_X_fr_RF_THIGH;
    Type_fr_RF_shank_fixed_X_fr_RF_THIGH fr_RF_shank_fixed_X_fr_RF_THIGH;
    Type_fr_RF_THIGH_X_fr_RF_shank_fixed fr_RF_THIGH_X_fr_RF_shank_fixed;
    Type_fr_RF_WHEEL_L_X_fr_RF_shank_fixed fr_RF_WHEEL_L_X_fr_RF_shank_fixed;
    Type_fr_RF_shank_fixed_X_fr_RF_WHEEL_L fr_RF_shank_fixed_X_fr_RF_WHEEL_L;
    Type_fr_LH_HIP_X_fr_base fr_LH_HIP_X_fr_base;
    Type_fr_base_X_fr_LH_HIP fr_base_X_fr_LH_HIP;
    Type_fr_LH_THIGH_X_fr_LH_HIP fr_LH_THIGH_X_fr_LH_HIP;
    Type_fr_LH_HIP_X_fr_LH_THIGH fr_LH_HIP_X_fr_LH_THIGH;
    Type_fr_LH_shank_fixed_X_fr_LH_THIGH fr_LH_shank_fixed_X_fr_LH_THIGH;
    Type_fr_LH_THIGH_X_fr_LH_shank_fixed fr_LH_THIGH_X_fr_LH_shank_fixed;
    Type_fr_LH_WHEEL_L_X_fr_LH_shank_fixed fr_LH_WHEEL_L_X_fr_LH_shank_fixed;
    Type_fr_LH_shank_fixed_X_fr_LH_WHEEL_L fr_LH_shank_fixed_X_fr_LH_WHEEL_L;
    Type_fr_RH_HIP_X_fr_base fr_RH_HIP_X_fr_base;
    Type_fr_base_X_fr_RH_HIP fr_base_X_fr_RH_HIP;
    Type_fr_RH_THIGH_X_fr_RH_HIP fr_RH_THIGH_X_fr_RH_HIP;
    Type_fr_RH_HIP_X_fr_RH_THIGH fr_RH_HIP_X_fr_RH_THIGH;
    Type_fr_RH_shank_fixed_X_fr_RH_THIGH fr_RH_shank_fixed_X_fr_RH_THIGH;
    Type_fr_RH_THIGH_X_fr_RH_shank_fixed fr_RH_THIGH_X_fr_RH_shank_fixed;
    Type_fr_RH_WHEEL_L_X_fr_RH_shank_fixed fr_RH_WHEEL_L_X_fr_RH_shank_fixed;
    Type_fr_RH_shank_fixed_X_fr_RH_WHEEL_L fr_RH_shank_fixed_X_fr_RH_WHEEL_L;

protected:

}; //class 'HomogeneousTransforms'

}

using MotionTransforms = tpl::MotionTransforms<rbd::DoubleTrait>;
using ForceTransforms = tpl::ForceTransforms<rbd::DoubleTrait>;
using HomogeneousTransforms = tpl::HomogeneousTransforms<rbd::DoubleTrait>;

}
}

#include "transforms.impl.h"

#endif
