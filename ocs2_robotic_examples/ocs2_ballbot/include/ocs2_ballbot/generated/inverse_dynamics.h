#ifndef IIT_BALLBOT_INVERSE_DYNAMICS_H_
#define IIT_BALLBOT_INVERSE_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/robcogen_commons.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "link_data_map.h"

namespace iit {
namespace Ballbot {
namespace dyn {

/**
 * The Inverse Dynamics routine for the robot Ballbot.
 *
 * In addition to the full Newton-Euler algorithm, specialized versions to compute
 * only certain terms are provided.
 * The parameters common to most of the methods are the joint status vector \c q, the
 * joint velocity vector \c qd and the acceleration vector \c qdd.
 *
 * Additional overloaded methods are provided without the \c q parameter. These
 * methods use the current configuration of the robot; they are provided for the
 * sake of efficiency, in case the motion transforms of the robot have already
 * been updated elsewhere with the most recent configuration (eg by a call to
 * setJointStatus()), so that it is useless to compute them again.
 *
 * Whenever present, the external forces parameter is a set of external
 * wrenches acting on the robot links. Each wrench must be expressed in
 * the reference frame of the link it is excerted on.
 */

namespace tpl {

template <typename TRAIT>
class InverseDynamics {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename TRAIT::Scalar Scalar;

    typedef iit::rbd::Core<Scalar> CoreS;

    typedef typename CoreS::ForceVector Force;
    typedef typename CoreS::VelocityVector Velocity;
    typedef typename CoreS::VelocityVector Acceleration;
    typedef typename CoreS::Matrix66 Matrix66s;
    typedef iit::rbd::tpl::InertiaMatrixDense<Scalar> InertiaMatrix;
    typedef iit::Ballbot::tpl::JointState<Scalar> JointState;
    typedef LinkDataMap<Force> ExtForces;
    typedef iit::Ballbot::tpl::MotionTransforms<TRAIT> MTransforms;
    typedef InertiaProperties<TRAIT> IProperties;

public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot Ballbot, which will be used by this instance
     *     to compute inverse-dynamics.
     */
    InverseDynamics(IProperties& in, MTransforms& tr);

    /** \name Inverse dynamics
     * The full Newton-Euler algorithm for the inverse dynamics of this robot.
     *
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */
    ///@{
    void id(
        JointState& jForces,
        const JointState& q, const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    void id(
        JointState& jForces,
        const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    ///@}

    /** \name Gravity terms
     * The joint forces (linear or rotational) required to compensate
     * for the effect of gravity, in a specific configuration.
     */
    ///@{
    void G_terms(JointState& jForces, const JointState& q);
    void G_terms(JointState& jForces);
    ///@}

    /** \name Centrifugal and Coriolis terms
     * The forces (linear or rotational) acting on the joints due to centrifugal and
     * Coriolis effects, for a specific configuration.
     */
    ///@{
    void C_terms(JointState& jForces, const JointState& q, const JointState& qd);
    void C_terms(JointState& jForces, const JointState& qd);
    ///@}
    /** Updates all the kinematics transforms used by the inverse dynamics routine. */
    void setJointStatus(const JointState& q) const;

public:
    /** \name Getters
     * These functions return various spatial quantities used internally
     * by the inverse dynamics routines, like the spatial acceleration
     * of the links.
     *
     * The getters can be useful to retrieve the additional data that is not
     * returned explicitly by the inverse dynamics routines even though it
     * is computed. For example, after a call to the inverse dynamics,
     * the spatial velocity of all the links has been determined and
     * can be accessed.
     *
     * However, beware that certain routines might not use some of the
     * spatial quantities, which therefore would retain their last value
     * without being updated nor reset (for example, the spatial velocity
     * of the links is unaffected by the computation of the gravity terms).
     */
    ///@{
    const Velocity& getVelocity_dummy_ball1() const { return dummy_ball1_v; }
    const Acceleration& getAcceleration_dummy_ball1() const { return dummy_ball1_a; }
    const Force& getForce_dummy_ball1() const { return dummy_ball1_f; }
    const Velocity& getVelocity_ball() const { return ball_v; }
    const Acceleration& getAcceleration_ball() const { return ball_a; }
    const Force& getForce_ball() const { return ball_f; }
    const Velocity& getVelocity_dummy_base1() const { return dummy_base1_v; }
    const Acceleration& getAcceleration_dummy_base1() const { return dummy_base1_a; }
    const Force& getForce_dummy_base1() const { return dummy_base1_f; }
    const Velocity& getVelocity_dummy_base2() const { return dummy_base2_v; }
    const Acceleration& getAcceleration_dummy_base2() const { return dummy_base2_a; }
    const Force& getForce_dummy_base2() const { return dummy_base2_f; }
    const Velocity& getVelocity_base() const { return base_v; }
    const Acceleration& getAcceleration_base() const { return base_a; }
    const Force& getForce_base() const { return base_f; }
    ///@}
protected:
    void firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext);
    void secondPass(JointState& jForces);

private:
    IProperties* inertiaProps;
    MTransforms* xm;
private:
    Matrix66s vcross; // support variable
    // Link 'dummy_ball1' :
    const InertiaMatrix& dummy_ball1_I;
    Velocity      dummy_ball1_v;
    Acceleration  dummy_ball1_a;
    Force         dummy_ball1_f;
    // Link 'ball' :
    const InertiaMatrix& ball_I;
    Velocity      ball_v;
    Acceleration  ball_a;
    Force         ball_f;
    // Link 'dummy_base1' :
    const InertiaMatrix& dummy_base1_I;
    Velocity      dummy_base1_v;
    Acceleration  dummy_base1_a;
    Force         dummy_base1_f;
    // Link 'dummy_base2' :
    const InertiaMatrix& dummy_base2_I;
    Velocity      dummy_base2_v;
    Acceleration  dummy_base2_a;
    Force         dummy_base2_f;
    // Link 'base' :
    const InertiaMatrix& base_I;
    Velocity      base_v;
    Acceleration  base_a;
    Force         base_f;


private:
    static const ExtForces zeroExtForces;
};

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::setJointStatus(const JointState& q) const
{
    (xm->fr_dummy_ball1_X_fr_world)(q);
    (xm->fr_ball_X_fr_dummy_ball1)(q);
    (xm->fr_dummy_base1_X_fr_ball)(q);
    (xm->fr_dummy_base2_X_fr_dummy_base1)(q);
    (xm->fr_base_X_fr_dummy_base2)(q);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::G_terms(JointState& jForces, const JointState& q)
{
    setJointStatus(q);
    G_terms(jForces);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::C_terms(JointState& jForces, const JointState& q, const JointState& qd)
{
    setJointStatus(q);
    C_terms(jForces, qd);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::id(
    JointState& jForces,
    const JointState& q, const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    setJointStatus(q);
    id(jForces, qd, qdd, fext);
}

}

typedef tpl::InverseDynamics<rbd::DoubleTrait> InverseDynamics;

}
}

}

#include "inverse_dynamics.impl.h"

#endif
