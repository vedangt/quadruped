#ifndef RCG_SHVAN1_FORWARD_DYNAMICS_H_
#define RCG_SHVAN1_FORWARD_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace rcg {
namespace shvan1 {
namespace dyn {

/**
 * The Forward Dynamics routine for the robot shvan1.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */
class ForwardDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;

    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot shvan1, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties& in, MotionTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param base_a
     * \param base_v
     * \param g the gravity acceleration vector, expressed in the
     *          base coordinates
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
       JointState& qdd, Acceleration& base_a, // output parameters,
       const Velocity& base_v, const Acceleration& g,
       const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, Acceleration& base_a, // output parameters,
        const Velocity& base_v, const Acceleration& g,
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* motionTransforms;

    Matrix66 vcross; // support variable
    Matrix66 Ia_r;   // support variable, articulated inertia in the case of a revolute joint
    // Link 'base'
    Matrix66 base_AI;
    Force base_p;

    // Link 'hip_link_LF' :
    Matrix66 hip_link_LF_AI;
    Velocity hip_link_LF_a;
    Velocity hip_link_LF_v;
    Velocity hip_link_LF_c;
    Force    hip_link_LF_p;

    Column6 hip_link_LF_U;
    Scalar hip_link_LF_D;
    Scalar hip_link_LF_u;
    // Link 'thigh_link_LF' :
    Matrix66 thigh_link_LF_AI;
    Velocity thigh_link_LF_a;
    Velocity thigh_link_LF_v;
    Velocity thigh_link_LF_c;
    Force    thigh_link_LF_p;

    Column6 thigh_link_LF_U;
    Scalar thigh_link_LF_D;
    Scalar thigh_link_LF_u;
    // Link 'shank_link_LF' :
    Matrix66 shank_link_LF_AI;
    Velocity shank_link_LF_a;
    Velocity shank_link_LF_v;
    Velocity shank_link_LF_c;
    Force    shank_link_LF_p;

    Column6 shank_link_LF_U;
    Scalar shank_link_LF_D;
    Scalar shank_link_LF_u;
    // Link 'hip_link_RF' :
    Matrix66 hip_link_RF_AI;
    Velocity hip_link_RF_a;
    Velocity hip_link_RF_v;
    Velocity hip_link_RF_c;
    Force    hip_link_RF_p;

    Column6 hip_link_RF_U;
    Scalar hip_link_RF_D;
    Scalar hip_link_RF_u;
    // Link 'thigh_link_RF' :
    Matrix66 thigh_link_RF_AI;
    Velocity thigh_link_RF_a;
    Velocity thigh_link_RF_v;
    Velocity thigh_link_RF_c;
    Force    thigh_link_RF_p;

    Column6 thigh_link_RF_U;
    Scalar thigh_link_RF_D;
    Scalar thigh_link_RF_u;
    // Link 'shank_link_RF' :
    Matrix66 shank_link_RF_AI;
    Velocity shank_link_RF_a;
    Velocity shank_link_RF_v;
    Velocity shank_link_RF_c;
    Force    shank_link_RF_p;

    Column6 shank_link_RF_U;
    Scalar shank_link_RF_D;
    Scalar shank_link_RF_u;
    // Link 'hip_link_LH' :
    Matrix66 hip_link_LH_AI;
    Velocity hip_link_LH_a;
    Velocity hip_link_LH_v;
    Velocity hip_link_LH_c;
    Force    hip_link_LH_p;

    Column6 hip_link_LH_U;
    Scalar hip_link_LH_D;
    Scalar hip_link_LH_u;
    // Link 'thigh_link_LH' :
    Matrix66 thigh_link_LH_AI;
    Velocity thigh_link_LH_a;
    Velocity thigh_link_LH_v;
    Velocity thigh_link_LH_c;
    Force    thigh_link_LH_p;

    Column6 thigh_link_LH_U;
    Scalar thigh_link_LH_D;
    Scalar thigh_link_LH_u;
    // Link 'shank_link_LH' :
    Matrix66 shank_link_LH_AI;
    Velocity shank_link_LH_a;
    Velocity shank_link_LH_v;
    Velocity shank_link_LH_c;
    Force    shank_link_LH_p;

    Column6 shank_link_LH_U;
    Scalar shank_link_LH_D;
    Scalar shank_link_LH_u;
    // Link 'hip_link_RH' :
    Matrix66 hip_link_RH_AI;
    Velocity hip_link_RH_a;
    Velocity hip_link_RH_v;
    Velocity hip_link_RH_c;
    Force    hip_link_RH_p;

    Column6 hip_link_RH_U;
    Scalar hip_link_RH_D;
    Scalar hip_link_RH_u;
    // Link 'thigh_link_RH' :
    Matrix66 thigh_link_RH_AI;
    Velocity thigh_link_RH_a;
    Velocity thigh_link_RH_v;
    Velocity thigh_link_RH_c;
    Force    thigh_link_RH_p;

    Column6 thigh_link_RH_U;
    Scalar thigh_link_RH_D;
    Scalar thigh_link_RH_u;
    // Link 'shank_link_RH' :
    Matrix66 shank_link_RH_AI;
    Velocity shank_link_RH_a;
    Velocity shank_link_RH_v;
    Velocity shank_link_RH_c;
    Force    shank_link_RH_p;

    Column6 shank_link_RH_U;
    Scalar shank_link_RH_D;
    Scalar shank_link_RH_u;
private:
    static const ExtForces zeroExtForces;
};

inline void ForwardDynamics::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_hip_link_LF_X_fr_base)(q);
    (motionTransforms-> fr_thigh_link_LF_X_fr_hip_link_LF)(q);
    (motionTransforms-> fr_shank_link_LF_X_fr_thigh_link_LF)(q);
    (motionTransforms-> fr_hip_link_RF_X_fr_base)(q);
    (motionTransforms-> fr_thigh_link_RF_X_fr_hip_link_RF)(q);
    (motionTransforms-> fr_shank_link_RF_X_fr_thigh_link_RF)(q);
    (motionTransforms-> fr_hip_link_LH_X_fr_base)(q);
    (motionTransforms-> fr_thigh_link_LH_X_fr_hip_link_LH)(q);
    (motionTransforms-> fr_shank_link_LH_X_fr_thigh_link_LH)(q);
    (motionTransforms-> fr_hip_link_RH_X_fr_base)(q);
    (motionTransforms-> fr_thigh_link_RH_X_fr_hip_link_RH)(q);
    (motionTransforms-> fr_shank_link_RH_X_fr_thigh_link_RH)(q);
}

inline void ForwardDynamics::fd(
    JointState& qdd, Acceleration& base_a, // output parameters,
    const Velocity& base_v, const Acceleration& g,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, base_a, base_v, g, qd, tau, fext);
}

}
}
}

#endif
