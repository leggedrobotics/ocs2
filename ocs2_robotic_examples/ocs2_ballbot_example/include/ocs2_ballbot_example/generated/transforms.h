#ifndef BALLBOT_TRANSFORMS_H_
#define BALLBOT_TRANSFORMS_H_

#include <Eigen/Dense>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include <iit/rbd/traits/DoubleTrait.h>
#include "kinematics_parameters.h"

namespace iit {
namespace Ballbot {

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
    class Type_fr_dummy_ball1_X_fr_world : public TransformMotion<Scalar, Type_fr_dummy_ball1_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_ball1_X_fr_world();
        const Type_fr_dummy_ball1_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_ball1 : public TransformMotion<Scalar, Type_fr_world_X_fr_dummy_ball1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_ball1();
        const Type_fr_world_X_fr_dummy_ball1& update(const JState&);
    protected:
    };
    
    class Type_fr_ball_X_fr_dummy_ball1 : public TransformMotion<Scalar, Type_fr_ball_X_fr_dummy_ball1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ball_X_fr_dummy_ball1();
        const Type_fr_ball_X_fr_dummy_ball1& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_ball1_X_fr_ball : public TransformMotion<Scalar, Type_fr_dummy_ball1_X_fr_ball>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_ball1_X_fr_ball();
        const Type_fr_dummy_ball1_X_fr_ball& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base1_X_fr_ball : public TransformMotion<Scalar, Type_fr_dummy_base1_X_fr_ball>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base1_X_fr_ball();
        const Type_fr_dummy_base1_X_fr_ball& update(const JState&);
    protected:
    };
    
    class Type_fr_ball_X_fr_dummy_base1 : public TransformMotion<Scalar, Type_fr_ball_X_fr_dummy_base1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ball_X_fr_dummy_base1();
        const Type_fr_ball_X_fr_dummy_base1& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base2_X_fr_dummy_base1 : public TransformMotion<Scalar, Type_fr_dummy_base2_X_fr_dummy_base1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base2_X_fr_dummy_base1();
        const Type_fr_dummy_base2_X_fr_dummy_base1& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base1_X_fr_dummy_base2 : public TransformMotion<Scalar, Type_fr_dummy_base1_X_fr_dummy_base2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base1_X_fr_dummy_base2();
        const Type_fr_dummy_base1_X_fr_dummy_base2& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_dummy_base2 : public TransformMotion<Scalar, Type_fr_base_X_fr_dummy_base2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_dummy_base2();
        const Type_fr_base_X_fr_dummy_base2& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base2_X_fr_base : public TransformMotion<Scalar, Type_fr_dummy_base2_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base2_X_fr_base();
        const Type_fr_dummy_base2_X_fr_base& update(const JState&);
    protected:
    };
    
public:
    MotionTransforms();
    void updateParameters();
    Type_fr_dummy_ball1_X_fr_world fr_dummy_ball1_X_fr_world;
    Type_fr_world_X_fr_dummy_ball1 fr_world_X_fr_dummy_ball1;
    Type_fr_ball_X_fr_dummy_ball1 fr_ball_X_fr_dummy_ball1;
    Type_fr_dummy_ball1_X_fr_ball fr_dummy_ball1_X_fr_ball;
    Type_fr_dummy_base1_X_fr_ball fr_dummy_base1_X_fr_ball;
    Type_fr_ball_X_fr_dummy_base1 fr_ball_X_fr_dummy_base1;
    Type_fr_dummy_base2_X_fr_dummy_base1 fr_dummy_base2_X_fr_dummy_base1;
    Type_fr_dummy_base1_X_fr_dummy_base2 fr_dummy_base1_X_fr_dummy_base2;
    Type_fr_base_X_fr_dummy_base2 fr_base_X_fr_dummy_base2;
    Type_fr_dummy_base2_X_fr_base fr_dummy_base2_X_fr_base;

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
    class Type_fr_dummy_ball1_X_fr_world : public TransformForce<Scalar, Type_fr_dummy_ball1_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_ball1_X_fr_world();
        const Type_fr_dummy_ball1_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_ball1 : public TransformForce<Scalar, Type_fr_world_X_fr_dummy_ball1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_ball1();
        const Type_fr_world_X_fr_dummy_ball1& update(const JState&);
    protected:
    };
    
    class Type_fr_ball_X_fr_dummy_ball1 : public TransformForce<Scalar, Type_fr_ball_X_fr_dummy_ball1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ball_X_fr_dummy_ball1();
        const Type_fr_ball_X_fr_dummy_ball1& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_ball1_X_fr_ball : public TransformForce<Scalar, Type_fr_dummy_ball1_X_fr_ball>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_ball1_X_fr_ball();
        const Type_fr_dummy_ball1_X_fr_ball& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base1_X_fr_ball : public TransformForce<Scalar, Type_fr_dummy_base1_X_fr_ball>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base1_X_fr_ball();
        const Type_fr_dummy_base1_X_fr_ball& update(const JState&);
    protected:
    };
    
    class Type_fr_ball_X_fr_dummy_base1 : public TransformForce<Scalar, Type_fr_ball_X_fr_dummy_base1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ball_X_fr_dummy_base1();
        const Type_fr_ball_X_fr_dummy_base1& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base2_X_fr_dummy_base1 : public TransformForce<Scalar, Type_fr_dummy_base2_X_fr_dummy_base1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base2_X_fr_dummy_base1();
        const Type_fr_dummy_base2_X_fr_dummy_base1& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base1_X_fr_dummy_base2 : public TransformForce<Scalar, Type_fr_dummy_base1_X_fr_dummy_base2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base1_X_fr_dummy_base2();
        const Type_fr_dummy_base1_X_fr_dummy_base2& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_dummy_base2 : public TransformForce<Scalar, Type_fr_base_X_fr_dummy_base2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_dummy_base2();
        const Type_fr_base_X_fr_dummy_base2& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base2_X_fr_base : public TransformForce<Scalar, Type_fr_dummy_base2_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base2_X_fr_base();
        const Type_fr_dummy_base2_X_fr_base& update(const JState&);
    protected:
    };
    
public:
    ForceTransforms();
    void updateParameters();
    Type_fr_dummy_ball1_X_fr_world fr_dummy_ball1_X_fr_world;
    Type_fr_world_X_fr_dummy_ball1 fr_world_X_fr_dummy_ball1;
    Type_fr_ball_X_fr_dummy_ball1 fr_ball_X_fr_dummy_ball1;
    Type_fr_dummy_ball1_X_fr_ball fr_dummy_ball1_X_fr_ball;
    Type_fr_dummy_base1_X_fr_ball fr_dummy_base1_X_fr_ball;
    Type_fr_ball_X_fr_dummy_base1 fr_ball_X_fr_dummy_base1;
    Type_fr_dummy_base2_X_fr_dummy_base1 fr_dummy_base2_X_fr_dummy_base1;
    Type_fr_dummy_base1_X_fr_dummy_base2 fr_dummy_base1_X_fr_dummy_base2;
    Type_fr_base_X_fr_dummy_base2 fr_base_X_fr_dummy_base2;
    Type_fr_dummy_base2_X_fr_base fr_dummy_base2_X_fr_base;

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
    class Type_fr_dummy_ball1_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_dummy_ball1_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_ball1_X_fr_world();
        const Type_fr_dummy_ball1_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_ball1 : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_dummy_ball1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_ball1();
        const Type_fr_world_X_fr_dummy_ball1& update(const JState&);
    protected:
    };
    
    class Type_fr_ball_X_fr_dummy_ball1 : public TransformHomogeneous<Scalar, Type_fr_ball_X_fr_dummy_ball1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ball_X_fr_dummy_ball1();
        const Type_fr_ball_X_fr_dummy_ball1& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_ball1_X_fr_ball : public TransformHomogeneous<Scalar, Type_fr_dummy_ball1_X_fr_ball>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_ball1_X_fr_ball();
        const Type_fr_dummy_ball1_X_fr_ball& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base1_X_fr_ball : public TransformHomogeneous<Scalar, Type_fr_dummy_base1_X_fr_ball>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base1_X_fr_ball();
        const Type_fr_dummy_base1_X_fr_ball& update(const JState&);
    protected:
    };
    
    class Type_fr_ball_X_fr_dummy_base1 : public TransformHomogeneous<Scalar, Type_fr_ball_X_fr_dummy_base1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ball_X_fr_dummy_base1();
        const Type_fr_ball_X_fr_dummy_base1& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base2_X_fr_dummy_base1 : public TransformHomogeneous<Scalar, Type_fr_dummy_base2_X_fr_dummy_base1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base2_X_fr_dummy_base1();
        const Type_fr_dummy_base2_X_fr_dummy_base1& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base1_X_fr_dummy_base2 : public TransformHomogeneous<Scalar, Type_fr_dummy_base1_X_fr_dummy_base2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base1_X_fr_dummy_base2();
        const Type_fr_dummy_base1_X_fr_dummy_base2& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_dummy_base2 : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_dummy_base2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_dummy_base2();
        const Type_fr_base_X_fr_dummy_base2& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base2_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_dummy_base2_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base2_X_fr_base();
        const Type_fr_dummy_base2_X_fr_base& update(const JState&);
    protected:
    };
    
public:
    HomogeneousTransforms();
    void updateParameters();
    Type_fr_dummy_ball1_X_fr_world fr_dummy_ball1_X_fr_world;
    Type_fr_world_X_fr_dummy_ball1 fr_world_X_fr_dummy_ball1;
    Type_fr_ball_X_fr_dummy_ball1 fr_ball_X_fr_dummy_ball1;
    Type_fr_dummy_ball1_X_fr_ball fr_dummy_ball1_X_fr_ball;
    Type_fr_dummy_base1_X_fr_ball fr_dummy_base1_X_fr_ball;
    Type_fr_ball_X_fr_dummy_base1 fr_ball_X_fr_dummy_base1;
    Type_fr_dummy_base2_X_fr_dummy_base1 fr_dummy_base2_X_fr_dummy_base1;
    Type_fr_dummy_base1_X_fr_dummy_base2 fr_dummy_base1_X_fr_dummy_base2;
    Type_fr_base_X_fr_dummy_base2 fr_base_X_fr_dummy_base2;
    Type_fr_dummy_base2_X_fr_base fr_dummy_base2_X_fr_base;

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
