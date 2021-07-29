#ifndef IIT_ROBOT_BALLBOT_FORWARD_DYNAMICS_H_
#define IIT_ROBOT_BALLBOT_FORWARD_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/robcogen_commons.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace iit {
namespace Ballbot {
namespace dyn {

/**
 * The Forward Dynamics routine for the robot Ballbot.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */

namespace tpl {

template <typename TRAIT>
class ForwardDynamics {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Convenient type aliases:

    typedef typename TRAIT::Scalar Scalar;

    typedef iit::rbd::Core<Scalar> CoreS;

    typedef typename CoreS::ForceVector Force;
    typedef typename CoreS::VelocityVector Velocity;
    typedef typename CoreS::VelocityVector Acceleration;
    typedef typename CoreS::Column6D Column6DS;
    typedef typename CoreS::Matrix66 Matrix66S;
    typedef LinkDataMap<Force> ExtForces;
    typedef typename iit::Ballbot::tpl::JointState<Scalar> JointState;
    typedef iit::Ballbot::tpl::MotionTransforms<TRAIT> MTransforms;

public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot Ballbot, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties<TRAIT>& in, MTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
        JointState& qdd, // output parameter
        const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, // output parameter
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties<TRAIT>* inertiaProps;
    MTransforms* motionTransforms;

    Matrix66S vcross; // support variable
    Matrix66S Ia_p;   // support variable, articulated inertia in the case of a prismatic joint
    Matrix66S Ia_r;   // support variable, articulated inertia in the case of a revolute joint

    // Link 'dummy_ball1' :
    Matrix66S dummy_ball1_AI;
    Velocity dummy_ball1_a;
    Velocity dummy_ball1_v;
    Velocity dummy_ball1_c;
    Force    dummy_ball1_p;

    Column6DS dummy_ball1_U;
    Scalar dummy_ball1_D;
    Scalar dummy_ball1_u;
    // Link 'ball' :
    Matrix66S ball_AI;
    Velocity ball_a;
    Velocity ball_v;
    Velocity ball_c;
    Force    ball_p;

    Column6DS ball_U;
    Scalar ball_D;
    Scalar ball_u;
    // Link 'dummy_base1' :
    Matrix66S dummy_base1_AI;
    Velocity dummy_base1_a;
    Velocity dummy_base1_v;
    Velocity dummy_base1_c;
    Force    dummy_base1_p;

    Column6DS dummy_base1_U;
    Scalar dummy_base1_D;
    Scalar dummy_base1_u;
    // Link 'dummy_base2' :
    Matrix66S dummy_base2_AI;
    Velocity dummy_base2_a;
    Velocity dummy_base2_v;
    Velocity dummy_base2_c;
    Force    dummy_base2_p;

    Column6DS dummy_base2_U;
    Scalar dummy_base2_D;
    Scalar dummy_base2_u;
    // Link 'base' :
    Matrix66S base_AI;
    Velocity base_a;
    Velocity base_v;
    Velocity base_c;
    Force    base_p;

    Column6DS base_U;
    Scalar base_D;
    Scalar base_u;
private:
    static const ExtForces zeroExtForces;
};

template <typename TRAIT>
inline void ForwardDynamics<TRAIT>::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_dummy_ball1_X_fr_world)(q);
    (motionTransforms-> fr_ball_X_fr_dummy_ball1)(q);
    (motionTransforms-> fr_dummy_base1_X_fr_ball)(q);
    (motionTransforms-> fr_dummy_base2_X_fr_dummy_base1)(q);
    (motionTransforms-> fr_base_X_fr_dummy_base2)(q);
}

template <typename TRAIT>
inline void ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, qd, tau, fext);
}

}

typedef tpl::ForwardDynamics<iit::rbd::DoubleTrait> ForwardDynamics;

}
}
}

#include "forward_dynamics.impl.h"

#endif
