#ifndef RCG_SHVAN1_INVERSE_DYNAMICS_H_
#define RCG_SHVAN1_INVERSE_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "link_data_map.h"

namespace rcg {
namespace shvan1 {
namespace dyn {

/**
 * The Inverse Dynamics routine for the robot shvan1.
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
class InverseDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;

    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot shvan1, which will be used by this instance
     *     to compute inverse-dynamics.
     */
    InverseDynamics(InertiaProperties& in, MotionTransforms& tr);

    /** \name Inverse dynamics
     * The full algorithm for the inverse dynamics of this robot.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[out] baseAccel the spatial acceleration of the robot base
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] base_v the spatial velocity of the base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id(
        JointState& jForces, Acceleration& base_a,
        const Acceleration& g, const Velocity& base_v,
        const JointState& q, const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    void id(
        JointState& jForces, Acceleration& base_a,
        const Acceleration& g, const Velocity& base_v,
        const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Inverse dynamics, fully actuated base
     * The inverse dynamics algorithm for the floating base robot,
     * in the assumption of a fully actuated base.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] baseWrench the spatial force to be applied to the robot base to achieve
     *             the desired accelerations
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] base_v the spatial velocity of the base
     * \param[in] baseAccel the desired spatial acceleration of the robot base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Gravity terms, fully actuated base
     */
    ///@{
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const JointState& q);
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g);
    ///@}
    /** \name Centrifugal and Coriolis terms, fully actuated base
     *
     * These functions take only velocity inputs, that is, they assume
     * a zero spatial acceleration of the base (in addition to zero acceleration
     * at the actuated joints).
     * Note that this is NOT the same as imposing zero acceleration
     * at the virtual 6-dof-floting-base joint, which would result, in general,
     * in a non-zero spatial acceleration of the base, due to velocity
     * product terms.
     */
    ///@{
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& base_v, const JointState& q, const JointState& qd);
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& base_v, const JointState& qd);
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
    const Force& getForce_base() const { return base_f; }
    const Velocity& getVelocity_hip_link_LF() const { return hip_link_LF_v; }
    const Acceleration& getAcceleration_hip_link_LF() const { return hip_link_LF_a; }
    const Force& getForce_hip_link_LF() const { return hip_link_LF_f; }
    const Velocity& getVelocity_thigh_link_LF() const { return thigh_link_LF_v; }
    const Acceleration& getAcceleration_thigh_link_LF() const { return thigh_link_LF_a; }
    const Force& getForce_thigh_link_LF() const { return thigh_link_LF_f; }
    const Velocity& getVelocity_shank_link_LF() const { return shank_link_LF_v; }
    const Acceleration& getAcceleration_shank_link_LF() const { return shank_link_LF_a; }
    const Force& getForce_shank_link_LF() const { return shank_link_LF_f; }
    const Velocity& getVelocity_hip_link_RF() const { return hip_link_RF_v; }
    const Acceleration& getAcceleration_hip_link_RF() const { return hip_link_RF_a; }
    const Force& getForce_hip_link_RF() const { return hip_link_RF_f; }
    const Velocity& getVelocity_thigh_link_RF() const { return thigh_link_RF_v; }
    const Acceleration& getAcceleration_thigh_link_RF() const { return thigh_link_RF_a; }
    const Force& getForce_thigh_link_RF() const { return thigh_link_RF_f; }
    const Velocity& getVelocity_shank_link_RF() const { return shank_link_RF_v; }
    const Acceleration& getAcceleration_shank_link_RF() const { return shank_link_RF_a; }
    const Force& getForce_shank_link_RF() const { return shank_link_RF_f; }
    const Velocity& getVelocity_hip_link_LH() const { return hip_link_LH_v; }
    const Acceleration& getAcceleration_hip_link_LH() const { return hip_link_LH_a; }
    const Force& getForce_hip_link_LH() const { return hip_link_LH_f; }
    const Velocity& getVelocity_thigh_link_LH() const { return thigh_link_LH_v; }
    const Acceleration& getAcceleration_thigh_link_LH() const { return thigh_link_LH_a; }
    const Force& getForce_thigh_link_LH() const { return thigh_link_LH_f; }
    const Velocity& getVelocity_shank_link_LH() const { return shank_link_LH_v; }
    const Acceleration& getAcceleration_shank_link_LH() const { return shank_link_LH_a; }
    const Force& getForce_shank_link_LH() const { return shank_link_LH_f; }
    const Velocity& getVelocity_hip_link_RH() const { return hip_link_RH_v; }
    const Acceleration& getAcceleration_hip_link_RH() const { return hip_link_RH_a; }
    const Force& getForce_hip_link_RH() const { return hip_link_RH_f; }
    const Velocity& getVelocity_thigh_link_RH() const { return thigh_link_RH_v; }
    const Acceleration& getAcceleration_thigh_link_RH() const { return thigh_link_RH_a; }
    const Force& getForce_thigh_link_RH() const { return thigh_link_RH_f; }
    const Velocity& getVelocity_shank_link_RH() const { return shank_link_RH_v; }
    const Acceleration& getAcceleration_shank_link_RH() const { return shank_link_RH_a; }
    const Force& getForce_shank_link_RH() const { return shank_link_RH_f; }
    ///@}
protected:
    void secondPass_fullyActuated(JointState& jForces);

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* xm;
private:
    Matrix66 vcross; // support variable
    // Link 'hip_link_LF' :
    const InertiaMatrix& hip_link_LF_I;
    Velocity      hip_link_LF_v;
    Acceleration  hip_link_LF_a;
    Force         hip_link_LF_f;
    // Link 'thigh_link_LF' :
    const InertiaMatrix& thigh_link_LF_I;
    Velocity      thigh_link_LF_v;
    Acceleration  thigh_link_LF_a;
    Force         thigh_link_LF_f;
    // Link 'shank_link_LF' :
    const InertiaMatrix& shank_link_LF_I;
    Velocity      shank_link_LF_v;
    Acceleration  shank_link_LF_a;
    Force         shank_link_LF_f;
    // Link 'hip_link_RF' :
    const InertiaMatrix& hip_link_RF_I;
    Velocity      hip_link_RF_v;
    Acceleration  hip_link_RF_a;
    Force         hip_link_RF_f;
    // Link 'thigh_link_RF' :
    const InertiaMatrix& thigh_link_RF_I;
    Velocity      thigh_link_RF_v;
    Acceleration  thigh_link_RF_a;
    Force         thigh_link_RF_f;
    // Link 'shank_link_RF' :
    const InertiaMatrix& shank_link_RF_I;
    Velocity      shank_link_RF_v;
    Acceleration  shank_link_RF_a;
    Force         shank_link_RF_f;
    // Link 'hip_link_LH' :
    const InertiaMatrix& hip_link_LH_I;
    Velocity      hip_link_LH_v;
    Acceleration  hip_link_LH_a;
    Force         hip_link_LH_f;
    // Link 'thigh_link_LH' :
    const InertiaMatrix& thigh_link_LH_I;
    Velocity      thigh_link_LH_v;
    Acceleration  thigh_link_LH_a;
    Force         thigh_link_LH_f;
    // Link 'shank_link_LH' :
    const InertiaMatrix& shank_link_LH_I;
    Velocity      shank_link_LH_v;
    Acceleration  shank_link_LH_a;
    Force         shank_link_LH_f;
    // Link 'hip_link_RH' :
    const InertiaMatrix& hip_link_RH_I;
    Velocity      hip_link_RH_v;
    Acceleration  hip_link_RH_a;
    Force         hip_link_RH_f;
    // Link 'thigh_link_RH' :
    const InertiaMatrix& thigh_link_RH_I;
    Velocity      thigh_link_RH_v;
    Acceleration  thigh_link_RH_a;
    Force         thigh_link_RH_f;
    // Link 'shank_link_RH' :
    const InertiaMatrix& shank_link_RH_I;
    Velocity      shank_link_RH_v;
    Acceleration  shank_link_RH_a;
    Force         shank_link_RH_f;

    // The robot base
    const InertiaMatrix& base_I;
    InertiaMatrix base_Ic;
    Force         base_f;
    // The composite inertia tensors
    InertiaMatrix hip_link_LF_Ic;
    InertiaMatrix thigh_link_LF_Ic;
    const InertiaMatrix& shank_link_LF_Ic;
    InertiaMatrix hip_link_RF_Ic;
    InertiaMatrix thigh_link_RF_Ic;
    const InertiaMatrix& shank_link_RF_Ic;
    InertiaMatrix hip_link_LH_Ic;
    InertiaMatrix thigh_link_LH_Ic;
    const InertiaMatrix& shank_link_LH_Ic;
    InertiaMatrix hip_link_RH_Ic;
    InertiaMatrix thigh_link_RH_Ic;
    const InertiaMatrix& shank_link_RH_Ic;

private:
    static const ExtForces zeroExtForces;
};

inline void InverseDynamics::setJointStatus(const JointState& q) const
{
    (xm->fr_hip_link_LF_X_fr_base)(q);
    (xm->fr_thigh_link_LF_X_fr_hip_link_LF)(q);
    (xm->fr_shank_link_LF_X_fr_thigh_link_LF)(q);
    (xm->fr_hip_link_RF_X_fr_base)(q);
    (xm->fr_thigh_link_RF_X_fr_hip_link_RF)(q);
    (xm->fr_shank_link_RF_X_fr_thigh_link_RF)(q);
    (xm->fr_hip_link_LH_X_fr_base)(q);
    (xm->fr_thigh_link_LH_X_fr_hip_link_LH)(q);
    (xm->fr_shank_link_LH_X_fr_thigh_link_LH)(q);
    (xm->fr_hip_link_RH_X_fr_base)(q);
    (xm->fr_thigh_link_RH_X_fr_hip_link_RH)(q);
    (xm->fr_shank_link_RH_X_fr_thigh_link_RH)(q);
}

inline void InverseDynamics::id(
    JointState& jForces, Acceleration& base_a,
    const Acceleration& g, const Velocity& base_v,
    const JointState& q, const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    setJointStatus(q);
    id(jForces, base_a, g, base_v,
       qd, qdd, fext);
}

inline void InverseDynamics::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g, const JointState& q)
{
    setJointStatus(q);
    G_terms_fully_actuated(baseWrench, jForces, g);
}

inline void InverseDynamics::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_v, const JointState& q, const JointState& qd)
{
    setJointStatus(q);
    C_terms_fully_actuated(baseWrench, jForces, base_v, qd);
}

inline void InverseDynamics::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    setJointStatus(q);
    id_fully_actuated(baseWrench, jForces, g, base_v,
        baseAccel, qd, qdd, fext);
}

}
}
}

#endif
