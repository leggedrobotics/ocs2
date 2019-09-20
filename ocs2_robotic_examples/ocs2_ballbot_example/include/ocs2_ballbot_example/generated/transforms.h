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
    class Type_fr_world_X_fr_dummy_ball1 : public TransformMotion<Scalar, Type_fr_world_X_fr_dummy_ball1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_ball1();
        const Type_fr_world_X_fr_dummy_ball1& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_ball1_X_fr_world : public TransformMotion<Scalar, Type_fr_dummy_ball1_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_ball1_X_fr_world();
        const Type_fr_dummy_ball1_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_ball : public TransformMotion<Scalar, Type_fr_world_X_fr_ball>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_ball();
        const Type_fr_world_X_fr_ball& update(const JState&);
    protected:
    };
    
    class Type_fr_ball_X_fr_world : public TransformMotion<Scalar, Type_fr_ball_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ball_X_fr_world();
        const Type_fr_ball_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_base1 : public TransformMotion<Scalar, Type_fr_world_X_fr_dummy_base1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_base1();
        const Type_fr_world_X_fr_dummy_base1& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base1_X_fr_world : public TransformMotion<Scalar, Type_fr_dummy_base1_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base1_X_fr_world();
        const Type_fr_dummy_base1_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_base2 : public TransformMotion<Scalar, Type_fr_world_X_fr_dummy_base2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_base2();
        const Type_fr_world_X_fr_dummy_base2& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base2_X_fr_world : public TransformMotion<Scalar, Type_fr_dummy_base2_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base2_X_fr_world();
        const Type_fr_dummy_base2_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_base : public TransformMotion<Scalar, Type_fr_world_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_base();
        const Type_fr_world_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_world : public TransformMotion<Scalar, Type_fr_base_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_world();
        const Type_fr_base_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_ball_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_ball_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_ball_COM();
        const Type_fr_world_X_fr_ball_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_ball_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_ball_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ball_COM_X_fr_world();
        const Type_fr_ball_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_base_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_base_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_base_COM();
        const Type_fr_world_X_fr_base_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_base_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_base_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_COM_X_fr_world();
        const Type_fr_base_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_ball1_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_dummy_ball1_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_ball1_COM();
        const Type_fr_world_X_fr_dummy_ball1_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_ball1_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_dummy_ball1_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_ball1_COM_X_fr_world();
        const Type_fr_dummy_ball1_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_base1_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_dummy_base1_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_base1_COM();
        const Type_fr_world_X_fr_dummy_base1_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base1_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_dummy_base1_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base1_COM_X_fr_world();
        const Type_fr_dummy_base1_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_base2_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_dummy_base2_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_base2_COM();
        const Type_fr_world_X_fr_dummy_base2_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base2_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_dummy_base2_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base2_COM_X_fr_world();
        const Type_fr_dummy_base2_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_world_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_world_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_world_COM();
        const Type_fr_world_X_fr_world_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_world_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_world_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_COM_X_fr_world();
        const Type_fr_world_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_world_inertia : public TransformMotion<Scalar, Type_fr_world_X_fr_world_inertia>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_world_inertia();
        const Type_fr_world_X_fr_world_inertia& update(const JState&);
    protected:
    };
    
    class Type_fr_world_inertia_X_fr_world : public TransformMotion<Scalar, Type_fr_world_inertia_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_inertia_X_fr_world();
        const Type_fr_world_inertia_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_jball_x : public TransformMotion<Scalar, Type_fr_world_X_fr_jball_x>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_jball_x();
        const Type_fr_world_X_fr_jball_x& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_jball_y : public TransformMotion<Scalar, Type_fr_world_X_fr_jball_y>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_jball_y();
        const Type_fr_world_X_fr_jball_y& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_jbase_z : public TransformMotion<Scalar, Type_fr_world_X_fr_jbase_z>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_jbase_z();
        const Type_fr_world_X_fr_jbase_z& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_jbase_y : public TransformMotion<Scalar, Type_fr_world_X_fr_jbase_y>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_jbase_y();
        const Type_fr_world_X_fr_jbase_y& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_jbase_x : public TransformMotion<Scalar, Type_fr_world_X_fr_jbase_x>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_jbase_x();
        const Type_fr_world_X_fr_jbase_x& update(const JState&);
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
    Type_fr_world_X_fr_dummy_ball1 fr_world_X_fr_dummy_ball1;
    Type_fr_dummy_ball1_X_fr_world fr_dummy_ball1_X_fr_world;
    Type_fr_world_X_fr_ball fr_world_X_fr_ball;
    Type_fr_ball_X_fr_world fr_ball_X_fr_world;
    Type_fr_world_X_fr_dummy_base1 fr_world_X_fr_dummy_base1;
    Type_fr_dummy_base1_X_fr_world fr_dummy_base1_X_fr_world;
    Type_fr_world_X_fr_dummy_base2 fr_world_X_fr_dummy_base2;
    Type_fr_dummy_base2_X_fr_world fr_dummy_base2_X_fr_world;
    Type_fr_world_X_fr_base fr_world_X_fr_base;
    Type_fr_base_X_fr_world fr_base_X_fr_world;
    Type_fr_world_X_fr_ball_COM fr_world_X_fr_ball_COM;
    Type_fr_ball_COM_X_fr_world fr_ball_COM_X_fr_world;
    Type_fr_world_X_fr_base_COM fr_world_X_fr_base_COM;
    Type_fr_base_COM_X_fr_world fr_base_COM_X_fr_world;
    Type_fr_world_X_fr_dummy_ball1_COM fr_world_X_fr_dummy_ball1_COM;
    Type_fr_dummy_ball1_COM_X_fr_world fr_dummy_ball1_COM_X_fr_world;
    Type_fr_world_X_fr_dummy_base1_COM fr_world_X_fr_dummy_base1_COM;
    Type_fr_dummy_base1_COM_X_fr_world fr_dummy_base1_COM_X_fr_world;
    Type_fr_world_X_fr_dummy_base2_COM fr_world_X_fr_dummy_base2_COM;
    Type_fr_dummy_base2_COM_X_fr_world fr_dummy_base2_COM_X_fr_world;
    Type_fr_world_X_fr_world_COM fr_world_X_fr_world_COM;
    Type_fr_world_COM_X_fr_world fr_world_COM_X_fr_world;
    Type_fr_world_X_fr_world_inertia fr_world_X_fr_world_inertia;
    Type_fr_world_inertia_X_fr_world fr_world_inertia_X_fr_world;
    Type_fr_world_X_fr_jball_x fr_world_X_fr_jball_x;
    Type_fr_world_X_fr_jball_y fr_world_X_fr_jball_y;
    Type_fr_world_X_fr_jbase_z fr_world_X_fr_jbase_z;
    Type_fr_world_X_fr_jbase_y fr_world_X_fr_jbase_y;
    Type_fr_world_X_fr_jbase_x fr_world_X_fr_jbase_x;
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
    class Type_fr_world_X_fr_dummy_ball1 : public TransformForce<Scalar, Type_fr_world_X_fr_dummy_ball1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_ball1();
        const Type_fr_world_X_fr_dummy_ball1& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_ball1_X_fr_world : public TransformForce<Scalar, Type_fr_dummy_ball1_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_ball1_X_fr_world();
        const Type_fr_dummy_ball1_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_ball : public TransformForce<Scalar, Type_fr_world_X_fr_ball>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_ball();
        const Type_fr_world_X_fr_ball& update(const JState&);
    protected:
    };
    
    class Type_fr_ball_X_fr_world : public TransformForce<Scalar, Type_fr_ball_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ball_X_fr_world();
        const Type_fr_ball_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_base1 : public TransformForce<Scalar, Type_fr_world_X_fr_dummy_base1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_base1();
        const Type_fr_world_X_fr_dummy_base1& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base1_X_fr_world : public TransformForce<Scalar, Type_fr_dummy_base1_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base1_X_fr_world();
        const Type_fr_dummy_base1_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_base2 : public TransformForce<Scalar, Type_fr_world_X_fr_dummy_base2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_base2();
        const Type_fr_world_X_fr_dummy_base2& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base2_X_fr_world : public TransformForce<Scalar, Type_fr_dummy_base2_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base2_X_fr_world();
        const Type_fr_dummy_base2_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_base : public TransformForce<Scalar, Type_fr_world_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_base();
        const Type_fr_world_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_world : public TransformForce<Scalar, Type_fr_base_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_world();
        const Type_fr_base_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_ball_COM : public TransformForce<Scalar, Type_fr_world_X_fr_ball_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_ball_COM();
        const Type_fr_world_X_fr_ball_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_ball_COM_X_fr_world : public TransformForce<Scalar, Type_fr_ball_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ball_COM_X_fr_world();
        const Type_fr_ball_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_base_COM : public TransformForce<Scalar, Type_fr_world_X_fr_base_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_base_COM();
        const Type_fr_world_X_fr_base_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_base_COM_X_fr_world : public TransformForce<Scalar, Type_fr_base_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_COM_X_fr_world();
        const Type_fr_base_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_ball1_COM : public TransformForce<Scalar, Type_fr_world_X_fr_dummy_ball1_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_ball1_COM();
        const Type_fr_world_X_fr_dummy_ball1_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_ball1_COM_X_fr_world : public TransformForce<Scalar, Type_fr_dummy_ball1_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_ball1_COM_X_fr_world();
        const Type_fr_dummy_ball1_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_base1_COM : public TransformForce<Scalar, Type_fr_world_X_fr_dummy_base1_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_base1_COM();
        const Type_fr_world_X_fr_dummy_base1_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base1_COM_X_fr_world : public TransformForce<Scalar, Type_fr_dummy_base1_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base1_COM_X_fr_world();
        const Type_fr_dummy_base1_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_base2_COM : public TransformForce<Scalar, Type_fr_world_X_fr_dummy_base2_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_base2_COM();
        const Type_fr_world_X_fr_dummy_base2_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base2_COM_X_fr_world : public TransformForce<Scalar, Type_fr_dummy_base2_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base2_COM_X_fr_world();
        const Type_fr_dummy_base2_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_world_COM : public TransformForce<Scalar, Type_fr_world_X_fr_world_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_world_COM();
        const Type_fr_world_X_fr_world_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_world_COM_X_fr_world : public TransformForce<Scalar, Type_fr_world_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_COM_X_fr_world();
        const Type_fr_world_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_world_inertia : public TransformForce<Scalar, Type_fr_world_X_fr_world_inertia>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_world_inertia();
        const Type_fr_world_X_fr_world_inertia& update(const JState&);
    protected:
    };
    
    class Type_fr_world_inertia_X_fr_world : public TransformForce<Scalar, Type_fr_world_inertia_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_inertia_X_fr_world();
        const Type_fr_world_inertia_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_jball_x : public TransformForce<Scalar, Type_fr_world_X_fr_jball_x>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_jball_x();
        const Type_fr_world_X_fr_jball_x& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_jball_y : public TransformForce<Scalar, Type_fr_world_X_fr_jball_y>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_jball_y();
        const Type_fr_world_X_fr_jball_y& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_jbase_z : public TransformForce<Scalar, Type_fr_world_X_fr_jbase_z>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_jbase_z();
        const Type_fr_world_X_fr_jbase_z& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_jbase_y : public TransformForce<Scalar, Type_fr_world_X_fr_jbase_y>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_jbase_y();
        const Type_fr_world_X_fr_jbase_y& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_jbase_x : public TransformForce<Scalar, Type_fr_world_X_fr_jbase_x>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_jbase_x();
        const Type_fr_world_X_fr_jbase_x& update(const JState&);
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
    Type_fr_world_X_fr_dummy_ball1 fr_world_X_fr_dummy_ball1;
    Type_fr_dummy_ball1_X_fr_world fr_dummy_ball1_X_fr_world;
    Type_fr_world_X_fr_ball fr_world_X_fr_ball;
    Type_fr_ball_X_fr_world fr_ball_X_fr_world;
    Type_fr_world_X_fr_dummy_base1 fr_world_X_fr_dummy_base1;
    Type_fr_dummy_base1_X_fr_world fr_dummy_base1_X_fr_world;
    Type_fr_world_X_fr_dummy_base2 fr_world_X_fr_dummy_base2;
    Type_fr_dummy_base2_X_fr_world fr_dummy_base2_X_fr_world;
    Type_fr_world_X_fr_base fr_world_X_fr_base;
    Type_fr_base_X_fr_world fr_base_X_fr_world;
    Type_fr_world_X_fr_ball_COM fr_world_X_fr_ball_COM;
    Type_fr_ball_COM_X_fr_world fr_ball_COM_X_fr_world;
    Type_fr_world_X_fr_base_COM fr_world_X_fr_base_COM;
    Type_fr_base_COM_X_fr_world fr_base_COM_X_fr_world;
    Type_fr_world_X_fr_dummy_ball1_COM fr_world_X_fr_dummy_ball1_COM;
    Type_fr_dummy_ball1_COM_X_fr_world fr_dummy_ball1_COM_X_fr_world;
    Type_fr_world_X_fr_dummy_base1_COM fr_world_X_fr_dummy_base1_COM;
    Type_fr_dummy_base1_COM_X_fr_world fr_dummy_base1_COM_X_fr_world;
    Type_fr_world_X_fr_dummy_base2_COM fr_world_X_fr_dummy_base2_COM;
    Type_fr_dummy_base2_COM_X_fr_world fr_dummy_base2_COM_X_fr_world;
    Type_fr_world_X_fr_world_COM fr_world_X_fr_world_COM;
    Type_fr_world_COM_X_fr_world fr_world_COM_X_fr_world;
    Type_fr_world_X_fr_world_inertia fr_world_X_fr_world_inertia;
    Type_fr_world_inertia_X_fr_world fr_world_inertia_X_fr_world;
    Type_fr_world_X_fr_jball_x fr_world_X_fr_jball_x;
    Type_fr_world_X_fr_jball_y fr_world_X_fr_jball_y;
    Type_fr_world_X_fr_jbase_z fr_world_X_fr_jbase_z;
    Type_fr_world_X_fr_jbase_y fr_world_X_fr_jbase_y;
    Type_fr_world_X_fr_jbase_x fr_world_X_fr_jbase_x;
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
    class Type_fr_world_X_fr_dummy_ball1 : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_dummy_ball1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_ball1();
        const Type_fr_world_X_fr_dummy_ball1& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_ball1_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_dummy_ball1_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_ball1_X_fr_world();
        const Type_fr_dummy_ball1_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_ball : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_ball>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_ball();
        const Type_fr_world_X_fr_ball& update(const JState&);
    protected:
    };
    
    class Type_fr_ball_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_ball_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ball_X_fr_world();
        const Type_fr_ball_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_base1 : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_dummy_base1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_base1();
        const Type_fr_world_X_fr_dummy_base1& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base1_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_dummy_base1_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base1_X_fr_world();
        const Type_fr_dummy_base1_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_base2 : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_dummy_base2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_base2();
        const Type_fr_world_X_fr_dummy_base2& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base2_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_dummy_base2_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base2_X_fr_world();
        const Type_fr_dummy_base2_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_base : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_base>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_base();
        const Type_fr_world_X_fr_base& update(const JState&);
    protected:
    };
    
    class Type_fr_base_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_base_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_X_fr_world();
        const Type_fr_base_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_ball_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_ball_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_ball_COM();
        const Type_fr_world_X_fr_ball_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_ball_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_ball_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ball_COM_X_fr_world();
        const Type_fr_ball_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_base_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_base_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_base_COM();
        const Type_fr_world_X_fr_base_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_base_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_base_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_COM_X_fr_world();
        const Type_fr_base_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_ball1_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_dummy_ball1_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_ball1_COM();
        const Type_fr_world_X_fr_dummy_ball1_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_ball1_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_dummy_ball1_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_ball1_COM_X_fr_world();
        const Type_fr_dummy_ball1_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_base1_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_dummy_base1_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_base1_COM();
        const Type_fr_world_X_fr_dummy_base1_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base1_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_dummy_base1_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base1_COM_X_fr_world();
        const Type_fr_dummy_base1_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_dummy_base2_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_dummy_base2_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_dummy_base2_COM();
        const Type_fr_world_X_fr_dummy_base2_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_dummy_base2_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_dummy_base2_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_dummy_base2_COM_X_fr_world();
        const Type_fr_dummy_base2_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_world_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_world_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_world_COM();
        const Type_fr_world_X_fr_world_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_world_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_world_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_COM_X_fr_world();
        const Type_fr_world_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_world_inertia : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_world_inertia>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_world_inertia();
        const Type_fr_world_X_fr_world_inertia& update(const JState&);
    protected:
    };
    
    class Type_fr_world_inertia_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_world_inertia_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_inertia_X_fr_world();
        const Type_fr_world_inertia_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_jball_x : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_jball_x>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_jball_x();
        const Type_fr_world_X_fr_jball_x& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_jball_y : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_jball_y>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_jball_y();
        const Type_fr_world_X_fr_jball_y& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_jbase_z : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_jbase_z>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_jbase_z();
        const Type_fr_world_X_fr_jbase_z& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_jbase_y : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_jbase_y>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_jbase_y();
        const Type_fr_world_X_fr_jbase_y& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_jbase_x : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_jbase_x>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_jbase_x();
        const Type_fr_world_X_fr_jbase_x& update(const JState&);
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
    Type_fr_world_X_fr_dummy_ball1 fr_world_X_fr_dummy_ball1;
    Type_fr_dummy_ball1_X_fr_world fr_dummy_ball1_X_fr_world;
    Type_fr_world_X_fr_ball fr_world_X_fr_ball;
    Type_fr_ball_X_fr_world fr_ball_X_fr_world;
    Type_fr_world_X_fr_dummy_base1 fr_world_X_fr_dummy_base1;
    Type_fr_dummy_base1_X_fr_world fr_dummy_base1_X_fr_world;
    Type_fr_world_X_fr_dummy_base2 fr_world_X_fr_dummy_base2;
    Type_fr_dummy_base2_X_fr_world fr_dummy_base2_X_fr_world;
    Type_fr_world_X_fr_base fr_world_X_fr_base;
    Type_fr_base_X_fr_world fr_base_X_fr_world;
    Type_fr_world_X_fr_ball_COM fr_world_X_fr_ball_COM;
    Type_fr_ball_COM_X_fr_world fr_ball_COM_X_fr_world;
    Type_fr_world_X_fr_base_COM fr_world_X_fr_base_COM;
    Type_fr_base_COM_X_fr_world fr_base_COM_X_fr_world;
    Type_fr_world_X_fr_dummy_ball1_COM fr_world_X_fr_dummy_ball1_COM;
    Type_fr_dummy_ball1_COM_X_fr_world fr_dummy_ball1_COM_X_fr_world;
    Type_fr_world_X_fr_dummy_base1_COM fr_world_X_fr_dummy_base1_COM;
    Type_fr_dummy_base1_COM_X_fr_world fr_dummy_base1_COM_X_fr_world;
    Type_fr_world_X_fr_dummy_base2_COM fr_world_X_fr_dummy_base2_COM;
    Type_fr_dummy_base2_COM_X_fr_world fr_dummy_base2_COM_X_fr_world;
    Type_fr_world_X_fr_world_COM fr_world_X_fr_world_COM;
    Type_fr_world_COM_X_fr_world fr_world_COM_X_fr_world;
    Type_fr_world_X_fr_world_inertia fr_world_X_fr_world_inertia;
    Type_fr_world_inertia_X_fr_world fr_world_inertia_X_fr_world;
    Type_fr_world_X_fr_jball_x fr_world_X_fr_jball_x;
    Type_fr_world_X_fr_jball_y fr_world_X_fr_jball_y;
    Type_fr_world_X_fr_jbase_z fr_world_X_fr_jbase_z;
    Type_fr_world_X_fr_jbase_y fr_world_X_fr_jbase_y;
    Type_fr_world_X_fr_jbase_x fr_world_X_fr_jbase_x;
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
