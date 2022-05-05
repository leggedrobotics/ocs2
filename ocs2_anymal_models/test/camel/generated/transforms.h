#ifndef CAMEL_TRANSFORMS_H_
#define CAMEL_TRANSFORMS_H_

#include <Eigen/Dense>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include <iit/rbd/traits/DoubleTrait.h>
#include "kinematics_parameters.h"

namespace iit {
namespace camel {

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
    class Type_fr_base_X_fr_LF_FOOT : public TransformMotion<Scalar, Type_fr_base_X_fr_LF_FOOT>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_FOOT();
        const Type_fr_base_X_fr_LF_FOOT& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_FOOT_X_fr_base : public TransformMotion<Scalar, Type_fr_LF_FOOT_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_FOOT_X_fr_base();
        const Type_fr_LF_FOOT_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_FOOT : public TransformMotion<Scalar, Type_fr_base_X_fr_LH_FOOT>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_FOOT();
        const Type_fr_base_X_fr_LH_FOOT& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_FOOT_X_fr_base : public TransformMotion<Scalar, Type_fr_LH_FOOT_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_FOOT_X_fr_base();
        const Type_fr_LH_FOOT_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_FOOT : public TransformMotion<Scalar, Type_fr_base_X_fr_RF_FOOT>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_FOOT();
        const Type_fr_base_X_fr_RF_FOOT& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_FOOT_X_fr_base : public TransformMotion<Scalar, Type_fr_RF_FOOT_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_FOOT_X_fr_base();
        const Type_fr_RF_FOOT_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_FOOT : public TransformMotion<Scalar, Type_fr_base_X_fr_RH_FOOT>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_FOOT();
        const Type_fr_base_X_fr_RH_FOOT& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_FOOT_X_fr_base : public TransformMotion<Scalar, Type_fr_RH_FOOT_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_FOOT_X_fr_base();
        const Type_fr_RH_FOOT_X_fr_base& update(const JState&);
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
    
    class Type_fr_LF_SHANK_X_fr_LF_THIGH : public TransformMotion<Scalar, Type_fr_LF_SHANK_X_fr_LF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_SHANK_X_fr_LF_THIGH();
        const Type_fr_LF_SHANK_X_fr_LF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_THIGH_X_fr_LF_SHANK : public TransformMotion<Scalar, Type_fr_LF_THIGH_X_fr_LF_SHANK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_THIGH_X_fr_LF_SHANK();
        const Type_fr_LF_THIGH_X_fr_LF_SHANK& update(const JState&);
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
    
    class Type_fr_RF_SHANK_X_fr_RF_THIGH : public TransformMotion<Scalar, Type_fr_RF_SHANK_X_fr_RF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_SHANK_X_fr_RF_THIGH();
        const Type_fr_RF_SHANK_X_fr_RF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_THIGH_X_fr_RF_SHANK : public TransformMotion<Scalar, Type_fr_RF_THIGH_X_fr_RF_SHANK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_THIGH_X_fr_RF_SHANK();
        const Type_fr_RF_THIGH_X_fr_RF_SHANK& update(const JState&);
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
    
    class Type_fr_LH_SHANK_X_fr_LH_THIGH : public TransformMotion<Scalar, Type_fr_LH_SHANK_X_fr_LH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_SHANK_X_fr_LH_THIGH();
        const Type_fr_LH_SHANK_X_fr_LH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_THIGH_X_fr_LH_SHANK : public TransformMotion<Scalar, Type_fr_LH_THIGH_X_fr_LH_SHANK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_THIGH_X_fr_LH_SHANK();
        const Type_fr_LH_THIGH_X_fr_LH_SHANK& update(const JState&);
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
    
    class Type_fr_RH_SHANK_X_fr_RH_THIGH : public TransformMotion<Scalar, Type_fr_RH_SHANK_X_fr_RH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_SHANK_X_fr_RH_THIGH();
        const Type_fr_RH_SHANK_X_fr_RH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_THIGH_X_fr_RH_SHANK : public TransformMotion<Scalar, Type_fr_RH_THIGH_X_fr_RH_SHANK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_THIGH_X_fr_RH_SHANK();
        const Type_fr_RH_THIGH_X_fr_RH_SHANK& update(const JState&);
    protected:
    };
    
public:
    MotionTransforms();
    void updateParameters();
    Type_fr_base_X_fr_LF_FOOT fr_base_X_fr_LF_FOOT;
    Type_fr_LF_FOOT_X_fr_base fr_LF_FOOT_X_fr_base;
    Type_fr_base_X_fr_LH_FOOT fr_base_X_fr_LH_FOOT;
    Type_fr_LH_FOOT_X_fr_base fr_LH_FOOT_X_fr_base;
    Type_fr_base_X_fr_RF_FOOT fr_base_X_fr_RF_FOOT;
    Type_fr_RF_FOOT_X_fr_base fr_RF_FOOT_X_fr_base;
    Type_fr_base_X_fr_RH_FOOT fr_base_X_fr_RH_FOOT;
    Type_fr_RH_FOOT_X_fr_base fr_RH_FOOT_X_fr_base;
    Type_fr_base_X_fr_LF_HAA fr_base_X_fr_LF_HAA;
    Type_fr_base_X_fr_LF_HFE fr_base_X_fr_LF_HFE;
    Type_fr_base_X_fr_LF_KFE fr_base_X_fr_LF_KFE;
    Type_fr_base_X_fr_LH_HAA fr_base_X_fr_LH_HAA;
    Type_fr_base_X_fr_LH_HFE fr_base_X_fr_LH_HFE;
    Type_fr_base_X_fr_LH_KFE fr_base_X_fr_LH_KFE;
    Type_fr_base_X_fr_RF_HAA fr_base_X_fr_RF_HAA;
    Type_fr_base_X_fr_RF_HFE fr_base_X_fr_RF_HFE;
    Type_fr_base_X_fr_RF_KFE fr_base_X_fr_RF_KFE;
    Type_fr_base_X_fr_RH_HAA fr_base_X_fr_RH_HAA;
    Type_fr_base_X_fr_RH_HFE fr_base_X_fr_RH_HFE;
    Type_fr_base_X_fr_RH_KFE fr_base_X_fr_RH_KFE;
    Type_fr_LF_HIP_X_fr_base fr_LF_HIP_X_fr_base;
    Type_fr_base_X_fr_LF_HIP fr_base_X_fr_LF_HIP;
    Type_fr_LF_THIGH_X_fr_LF_HIP fr_LF_THIGH_X_fr_LF_HIP;
    Type_fr_LF_HIP_X_fr_LF_THIGH fr_LF_HIP_X_fr_LF_THIGH;
    Type_fr_LF_SHANK_X_fr_LF_THIGH fr_LF_SHANK_X_fr_LF_THIGH;
    Type_fr_LF_THIGH_X_fr_LF_SHANK fr_LF_THIGH_X_fr_LF_SHANK;
    Type_fr_RF_HIP_X_fr_base fr_RF_HIP_X_fr_base;
    Type_fr_base_X_fr_RF_HIP fr_base_X_fr_RF_HIP;
    Type_fr_RF_THIGH_X_fr_RF_HIP fr_RF_THIGH_X_fr_RF_HIP;
    Type_fr_RF_HIP_X_fr_RF_THIGH fr_RF_HIP_X_fr_RF_THIGH;
    Type_fr_RF_SHANK_X_fr_RF_THIGH fr_RF_SHANK_X_fr_RF_THIGH;
    Type_fr_RF_THIGH_X_fr_RF_SHANK fr_RF_THIGH_X_fr_RF_SHANK;
    Type_fr_LH_HIP_X_fr_base fr_LH_HIP_X_fr_base;
    Type_fr_base_X_fr_LH_HIP fr_base_X_fr_LH_HIP;
    Type_fr_LH_THIGH_X_fr_LH_HIP fr_LH_THIGH_X_fr_LH_HIP;
    Type_fr_LH_HIP_X_fr_LH_THIGH fr_LH_HIP_X_fr_LH_THIGH;
    Type_fr_LH_SHANK_X_fr_LH_THIGH fr_LH_SHANK_X_fr_LH_THIGH;
    Type_fr_LH_THIGH_X_fr_LH_SHANK fr_LH_THIGH_X_fr_LH_SHANK;
    Type_fr_RH_HIP_X_fr_base fr_RH_HIP_X_fr_base;
    Type_fr_base_X_fr_RH_HIP fr_base_X_fr_RH_HIP;
    Type_fr_RH_THIGH_X_fr_RH_HIP fr_RH_THIGH_X_fr_RH_HIP;
    Type_fr_RH_HIP_X_fr_RH_THIGH fr_RH_HIP_X_fr_RH_THIGH;
    Type_fr_RH_SHANK_X_fr_RH_THIGH fr_RH_SHANK_X_fr_RH_THIGH;
    Type_fr_RH_THIGH_X_fr_RH_SHANK fr_RH_THIGH_X_fr_RH_SHANK;

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
    class Type_fr_base_X_fr_LF_FOOT : public TransformForce<Scalar, Type_fr_base_X_fr_LF_FOOT>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_FOOT();
        const Type_fr_base_X_fr_LF_FOOT& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_FOOT_X_fr_base : public TransformForce<Scalar, Type_fr_LF_FOOT_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_FOOT_X_fr_base();
        const Type_fr_LF_FOOT_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_FOOT : public TransformForce<Scalar, Type_fr_base_X_fr_LH_FOOT>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_FOOT();
        const Type_fr_base_X_fr_LH_FOOT& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_FOOT_X_fr_base : public TransformForce<Scalar, Type_fr_LH_FOOT_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_FOOT_X_fr_base();
        const Type_fr_LH_FOOT_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_FOOT : public TransformForce<Scalar, Type_fr_base_X_fr_RF_FOOT>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_FOOT();
        const Type_fr_base_X_fr_RF_FOOT& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_FOOT_X_fr_base : public TransformForce<Scalar, Type_fr_RF_FOOT_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_FOOT_X_fr_base();
        const Type_fr_RF_FOOT_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_FOOT : public TransformForce<Scalar, Type_fr_base_X_fr_RH_FOOT>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_FOOT();
        const Type_fr_base_X_fr_RH_FOOT& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_FOOT_X_fr_base : public TransformForce<Scalar, Type_fr_RH_FOOT_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_FOOT_X_fr_base();
        const Type_fr_RH_FOOT_X_fr_base& update(const JState&);
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
    
    class Type_fr_LF_SHANK_X_fr_LF_THIGH : public TransformForce<Scalar, Type_fr_LF_SHANK_X_fr_LF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_SHANK_X_fr_LF_THIGH();
        const Type_fr_LF_SHANK_X_fr_LF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_THIGH_X_fr_LF_SHANK : public TransformForce<Scalar, Type_fr_LF_THIGH_X_fr_LF_SHANK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_THIGH_X_fr_LF_SHANK();
        const Type_fr_LF_THIGH_X_fr_LF_SHANK& update(const JState&);
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
    
    class Type_fr_RF_SHANK_X_fr_RF_THIGH : public TransformForce<Scalar, Type_fr_RF_SHANK_X_fr_RF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_SHANK_X_fr_RF_THIGH();
        const Type_fr_RF_SHANK_X_fr_RF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_THIGH_X_fr_RF_SHANK : public TransformForce<Scalar, Type_fr_RF_THIGH_X_fr_RF_SHANK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_THIGH_X_fr_RF_SHANK();
        const Type_fr_RF_THIGH_X_fr_RF_SHANK& update(const JState&);
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
    
    class Type_fr_LH_SHANK_X_fr_LH_THIGH : public TransformForce<Scalar, Type_fr_LH_SHANK_X_fr_LH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_SHANK_X_fr_LH_THIGH();
        const Type_fr_LH_SHANK_X_fr_LH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_THIGH_X_fr_LH_SHANK : public TransformForce<Scalar, Type_fr_LH_THIGH_X_fr_LH_SHANK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_THIGH_X_fr_LH_SHANK();
        const Type_fr_LH_THIGH_X_fr_LH_SHANK& update(const JState&);
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
    
    class Type_fr_RH_SHANK_X_fr_RH_THIGH : public TransformForce<Scalar, Type_fr_RH_SHANK_X_fr_RH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_SHANK_X_fr_RH_THIGH();
        const Type_fr_RH_SHANK_X_fr_RH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_THIGH_X_fr_RH_SHANK : public TransformForce<Scalar, Type_fr_RH_THIGH_X_fr_RH_SHANK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_THIGH_X_fr_RH_SHANK();
        const Type_fr_RH_THIGH_X_fr_RH_SHANK& update(const JState&);
    protected:
    };
    
public:
    ForceTransforms();
    void updateParameters();
    Type_fr_base_X_fr_LF_FOOT fr_base_X_fr_LF_FOOT;
    Type_fr_LF_FOOT_X_fr_base fr_LF_FOOT_X_fr_base;
    Type_fr_base_X_fr_LH_FOOT fr_base_X_fr_LH_FOOT;
    Type_fr_LH_FOOT_X_fr_base fr_LH_FOOT_X_fr_base;
    Type_fr_base_X_fr_RF_FOOT fr_base_X_fr_RF_FOOT;
    Type_fr_RF_FOOT_X_fr_base fr_RF_FOOT_X_fr_base;
    Type_fr_base_X_fr_RH_FOOT fr_base_X_fr_RH_FOOT;
    Type_fr_RH_FOOT_X_fr_base fr_RH_FOOT_X_fr_base;
    Type_fr_base_X_fr_LF_HAA fr_base_X_fr_LF_HAA;
    Type_fr_base_X_fr_LF_HFE fr_base_X_fr_LF_HFE;
    Type_fr_base_X_fr_LF_KFE fr_base_X_fr_LF_KFE;
    Type_fr_base_X_fr_LH_HAA fr_base_X_fr_LH_HAA;
    Type_fr_base_X_fr_LH_HFE fr_base_X_fr_LH_HFE;
    Type_fr_base_X_fr_LH_KFE fr_base_X_fr_LH_KFE;
    Type_fr_base_X_fr_RF_HAA fr_base_X_fr_RF_HAA;
    Type_fr_base_X_fr_RF_HFE fr_base_X_fr_RF_HFE;
    Type_fr_base_X_fr_RF_KFE fr_base_X_fr_RF_KFE;
    Type_fr_base_X_fr_RH_HAA fr_base_X_fr_RH_HAA;
    Type_fr_base_X_fr_RH_HFE fr_base_X_fr_RH_HFE;
    Type_fr_base_X_fr_RH_KFE fr_base_X_fr_RH_KFE;
    Type_fr_LF_HIP_X_fr_base fr_LF_HIP_X_fr_base;
    Type_fr_base_X_fr_LF_HIP fr_base_X_fr_LF_HIP;
    Type_fr_LF_THIGH_X_fr_LF_HIP fr_LF_THIGH_X_fr_LF_HIP;
    Type_fr_LF_HIP_X_fr_LF_THIGH fr_LF_HIP_X_fr_LF_THIGH;
    Type_fr_LF_SHANK_X_fr_LF_THIGH fr_LF_SHANK_X_fr_LF_THIGH;
    Type_fr_LF_THIGH_X_fr_LF_SHANK fr_LF_THIGH_X_fr_LF_SHANK;
    Type_fr_RF_HIP_X_fr_base fr_RF_HIP_X_fr_base;
    Type_fr_base_X_fr_RF_HIP fr_base_X_fr_RF_HIP;
    Type_fr_RF_THIGH_X_fr_RF_HIP fr_RF_THIGH_X_fr_RF_HIP;
    Type_fr_RF_HIP_X_fr_RF_THIGH fr_RF_HIP_X_fr_RF_THIGH;
    Type_fr_RF_SHANK_X_fr_RF_THIGH fr_RF_SHANK_X_fr_RF_THIGH;
    Type_fr_RF_THIGH_X_fr_RF_SHANK fr_RF_THIGH_X_fr_RF_SHANK;
    Type_fr_LH_HIP_X_fr_base fr_LH_HIP_X_fr_base;
    Type_fr_base_X_fr_LH_HIP fr_base_X_fr_LH_HIP;
    Type_fr_LH_THIGH_X_fr_LH_HIP fr_LH_THIGH_X_fr_LH_HIP;
    Type_fr_LH_HIP_X_fr_LH_THIGH fr_LH_HIP_X_fr_LH_THIGH;
    Type_fr_LH_SHANK_X_fr_LH_THIGH fr_LH_SHANK_X_fr_LH_THIGH;
    Type_fr_LH_THIGH_X_fr_LH_SHANK fr_LH_THIGH_X_fr_LH_SHANK;
    Type_fr_RH_HIP_X_fr_base fr_RH_HIP_X_fr_base;
    Type_fr_base_X_fr_RH_HIP fr_base_X_fr_RH_HIP;
    Type_fr_RH_THIGH_X_fr_RH_HIP fr_RH_THIGH_X_fr_RH_HIP;
    Type_fr_RH_HIP_X_fr_RH_THIGH fr_RH_HIP_X_fr_RH_THIGH;
    Type_fr_RH_SHANK_X_fr_RH_THIGH fr_RH_SHANK_X_fr_RH_THIGH;
    Type_fr_RH_THIGH_X_fr_RH_SHANK fr_RH_THIGH_X_fr_RH_SHANK;

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
    class Type_fr_base_X_fr_LF_FOOT : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_LF_FOOT>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LF_FOOT();
        const Type_fr_base_X_fr_LF_FOOT& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_FOOT_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_LF_FOOT_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_FOOT_X_fr_base();
        const Type_fr_LF_FOOT_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_LH_FOOT : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_LH_FOOT>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_LH_FOOT();
        const Type_fr_base_X_fr_LH_FOOT& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_FOOT_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_LH_FOOT_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_FOOT_X_fr_base();
        const Type_fr_LH_FOOT_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RF_FOOT : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_RF_FOOT>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RF_FOOT();
        const Type_fr_base_X_fr_RF_FOOT& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_FOOT_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_RF_FOOT_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_FOOT_X_fr_base();
        const Type_fr_RF_FOOT_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_RH_FOOT : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_RH_FOOT>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_RH_FOOT();
        const Type_fr_base_X_fr_RH_FOOT& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_FOOT_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_RH_FOOT_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_FOOT_X_fr_base();
        const Type_fr_RH_FOOT_X_fr_base& update(const JState&);
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
    
    class Type_fr_LF_SHANK_X_fr_LF_THIGH : public TransformHomogeneous<Scalar, Type_fr_LF_SHANK_X_fr_LF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_SHANK_X_fr_LF_THIGH();
        const Type_fr_LF_SHANK_X_fr_LF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LF_THIGH_X_fr_LF_SHANK : public TransformHomogeneous<Scalar, Type_fr_LF_THIGH_X_fr_LF_SHANK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LF_THIGH_X_fr_LF_SHANK();
        const Type_fr_LF_THIGH_X_fr_LF_SHANK& update(const JState&);
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
    
    class Type_fr_RF_SHANK_X_fr_RF_THIGH : public TransformHomogeneous<Scalar, Type_fr_RF_SHANK_X_fr_RF_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_SHANK_X_fr_RF_THIGH();
        const Type_fr_RF_SHANK_X_fr_RF_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RF_THIGH_X_fr_RF_SHANK : public TransformHomogeneous<Scalar, Type_fr_RF_THIGH_X_fr_RF_SHANK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RF_THIGH_X_fr_RF_SHANK();
        const Type_fr_RF_THIGH_X_fr_RF_SHANK& update(const JState&);
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
    
    class Type_fr_LH_SHANK_X_fr_LH_THIGH : public TransformHomogeneous<Scalar, Type_fr_LH_SHANK_X_fr_LH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_SHANK_X_fr_LH_THIGH();
        const Type_fr_LH_SHANK_X_fr_LH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_LH_THIGH_X_fr_LH_SHANK : public TransformHomogeneous<Scalar, Type_fr_LH_THIGH_X_fr_LH_SHANK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_LH_THIGH_X_fr_LH_SHANK();
        const Type_fr_LH_THIGH_X_fr_LH_SHANK& update(const JState&);
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
    
    class Type_fr_RH_SHANK_X_fr_RH_THIGH : public TransformHomogeneous<Scalar, Type_fr_RH_SHANK_X_fr_RH_THIGH>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_SHANK_X_fr_RH_THIGH();
        const Type_fr_RH_SHANK_X_fr_RH_THIGH& update(const JState&);
    protected:
    };
    
    class Type_fr_RH_THIGH_X_fr_RH_SHANK : public TransformHomogeneous<Scalar, Type_fr_RH_THIGH_X_fr_RH_SHANK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_RH_THIGH_X_fr_RH_SHANK();
        const Type_fr_RH_THIGH_X_fr_RH_SHANK& update(const JState&);
    protected:
    };
    
public:
    HomogeneousTransforms();
    void updateParameters();
    Type_fr_base_X_fr_LF_FOOT fr_base_X_fr_LF_FOOT;
    Type_fr_LF_FOOT_X_fr_base fr_LF_FOOT_X_fr_base;
    Type_fr_base_X_fr_LH_FOOT fr_base_X_fr_LH_FOOT;
    Type_fr_LH_FOOT_X_fr_base fr_LH_FOOT_X_fr_base;
    Type_fr_base_X_fr_RF_FOOT fr_base_X_fr_RF_FOOT;
    Type_fr_RF_FOOT_X_fr_base fr_RF_FOOT_X_fr_base;
    Type_fr_base_X_fr_RH_FOOT fr_base_X_fr_RH_FOOT;
    Type_fr_RH_FOOT_X_fr_base fr_RH_FOOT_X_fr_base;
    Type_fr_base_X_fr_LF_HAA fr_base_X_fr_LF_HAA;
    Type_fr_base_X_fr_LF_HFE fr_base_X_fr_LF_HFE;
    Type_fr_base_X_fr_LF_KFE fr_base_X_fr_LF_KFE;
    Type_fr_base_X_fr_LH_HAA fr_base_X_fr_LH_HAA;
    Type_fr_base_X_fr_LH_HFE fr_base_X_fr_LH_HFE;
    Type_fr_base_X_fr_LH_KFE fr_base_X_fr_LH_KFE;
    Type_fr_base_X_fr_RF_HAA fr_base_X_fr_RF_HAA;
    Type_fr_base_X_fr_RF_HFE fr_base_X_fr_RF_HFE;
    Type_fr_base_X_fr_RF_KFE fr_base_X_fr_RF_KFE;
    Type_fr_base_X_fr_RH_HAA fr_base_X_fr_RH_HAA;
    Type_fr_base_X_fr_RH_HFE fr_base_X_fr_RH_HFE;
    Type_fr_base_X_fr_RH_KFE fr_base_X_fr_RH_KFE;
    Type_fr_LF_HIP_X_fr_base fr_LF_HIP_X_fr_base;
    Type_fr_base_X_fr_LF_HIP fr_base_X_fr_LF_HIP;
    Type_fr_LF_THIGH_X_fr_LF_HIP fr_LF_THIGH_X_fr_LF_HIP;
    Type_fr_LF_HIP_X_fr_LF_THIGH fr_LF_HIP_X_fr_LF_THIGH;
    Type_fr_LF_SHANK_X_fr_LF_THIGH fr_LF_SHANK_X_fr_LF_THIGH;
    Type_fr_LF_THIGH_X_fr_LF_SHANK fr_LF_THIGH_X_fr_LF_SHANK;
    Type_fr_RF_HIP_X_fr_base fr_RF_HIP_X_fr_base;
    Type_fr_base_X_fr_RF_HIP fr_base_X_fr_RF_HIP;
    Type_fr_RF_THIGH_X_fr_RF_HIP fr_RF_THIGH_X_fr_RF_HIP;
    Type_fr_RF_HIP_X_fr_RF_THIGH fr_RF_HIP_X_fr_RF_THIGH;
    Type_fr_RF_SHANK_X_fr_RF_THIGH fr_RF_SHANK_X_fr_RF_THIGH;
    Type_fr_RF_THIGH_X_fr_RF_SHANK fr_RF_THIGH_X_fr_RF_SHANK;
    Type_fr_LH_HIP_X_fr_base fr_LH_HIP_X_fr_base;
    Type_fr_base_X_fr_LH_HIP fr_base_X_fr_LH_HIP;
    Type_fr_LH_THIGH_X_fr_LH_HIP fr_LH_THIGH_X_fr_LH_HIP;
    Type_fr_LH_HIP_X_fr_LH_THIGH fr_LH_HIP_X_fr_LH_THIGH;
    Type_fr_LH_SHANK_X_fr_LH_THIGH fr_LH_SHANK_X_fr_LH_THIGH;
    Type_fr_LH_THIGH_X_fr_LH_SHANK fr_LH_THIGH_X_fr_LH_SHANK;
    Type_fr_RH_HIP_X_fr_base fr_RH_HIP_X_fr_base;
    Type_fr_base_X_fr_RH_HIP fr_base_X_fr_RH_HIP;
    Type_fr_RH_THIGH_X_fr_RH_HIP fr_RH_THIGH_X_fr_RH_HIP;
    Type_fr_RH_HIP_X_fr_RH_THIGH fr_RH_HIP_X_fr_RH_THIGH;
    Type_fr_RH_SHANK_X_fr_RH_THIGH fr_RH_SHANK_X_fr_RH_THIGH;
    Type_fr_RH_THIGH_X_fr_RH_SHANK fr_RH_THIGH_X_fr_RH_SHANK;

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
