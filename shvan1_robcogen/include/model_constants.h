#ifndef RCG_SHVAN1_MODEL_CONSTANTS_H_
#define RCG_SHVAN1_MODEL_CONSTANTS_H_

#include "rbd_types.h"

/**
 * \file
 * This file contains the definitions of all the non-zero numerical
 * constants of the robot model (i.e. the numbers appearing in the
 * .kindsl file).
 *
 * Varying these values (and recompiling) is a quick & dirty
 * way to vary the kinematics/dynamics model. For a much more
 * flexible way of exploring variations of the model, consider
 * using the parametrization feature of RobCoGen (see the wiki).
 *
 * Beware of inconsistencies when changing any of the inertia
 * properties.
 */

namespace rcg {
namespace shvan1 {

// Do not use 'constexpr' to allow for non-literal scalar types

const Scalar tx_hip_joint_LF = 0.30000001192092896;
const Scalar ty_hip_joint_LF = 0.20999999344348907;
const Scalar tx_elbow_joint_LF = 0.07999999821186066;
const Scalar tx_knee_joint_LF = 0.20000000298023224;
const Scalar tx_hip_joint_RF = 0.30000001192092896;
const Scalar ty_hip_joint_RF = -0.20999999344348907;
const Scalar tx_elbow_joint_RF = 0.07999999821186066;
const Scalar tx_knee_joint_RF = 0.20000000298023224;
const Scalar tx_hip_joint_LH = -0.30000001192092896;
const Scalar ty_hip_joint_LH = 0.20999999344348907;
const Scalar tx_elbow_joint_LH = 0.07999999821186066;
const Scalar tx_knee_joint_LH = 0.20000000298023224;
const Scalar tx_hip_joint_RH = -0.30000001192092896;
const Scalar ty_hip_joint_RH = -0.20999999344348907;
const Scalar tx_elbow_joint_RH = 0.07999999821186066;
const Scalar tx_knee_joint_RH = 0.20000000298023224;
const Scalar tx_LF_FOOT = 0.23000000417232513;
const Scalar tx_RF_FOOT = 0.23000000417232513;
const Scalar tx_LH_FOOT = 0.23000000417232513;
const Scalar tx_RH_FOOT = 0.23000000417232513;
const Scalar m_base = 10.0;
const Scalar ix_base = 0.1547120064496994;
const Scalar iy_base = 0.30709999799728394;
const Scalar iz_base = 0.4452120065689087;
const Scalar m_hip_link_LF = 0.6000000238418579;
const Scalar comx_hip_link_LF = 0.03999999910593033;
const Scalar ix_hip_link_LF = 6.199999916134402E-5;
const Scalar iy_hip_link_LF = 0.0013099999632686377;
const Scalar iz_hip_link_LF = 0.0013099999632686377;
const Scalar m_thigh_link_LF = 1.2000000476837158;
const Scalar comx_thigh_link_LF = 0.10000000149011612;
const Scalar ix_thigh_link_LF = 1.250000059371814E-4;
const Scalar iy_thigh_link_LF = 0.01604600064456463;
const Scalar iz_thigh_link_LF = 0.01604600064456463;
const Scalar m_shank_link_LF = 0.5;
const Scalar comx_shank_link_LF = 0.16099999845027924;
const Scalar ix_shank_link_LF = 5.199999941396527E-5;
const Scalar iy_shank_link_LF = 0.015900999307632446;
const Scalar iz_shank_link_LF = 0.015900999307632446;
const Scalar m_hip_link_RF = 0.6000000238418579;
const Scalar comx_hip_link_RF = 0.03999999910593033;
const Scalar ix_hip_link_RF = 6.199999916134402E-5;
const Scalar iy_hip_link_RF = 0.0013099999632686377;
const Scalar iz_hip_link_RF = 0.0013099999632686377;
const Scalar m_thigh_link_RF = 1.2000000476837158;
const Scalar comx_thigh_link_RF = 0.10000000149011612;
const Scalar ix_thigh_link_RF = 1.250000059371814E-4;
const Scalar iy_thigh_link_RF = 0.01604600064456463;
const Scalar iz_thigh_link_RF = 0.01604600064456463;
const Scalar m_shank_link_RF = 0.5;
const Scalar comx_shank_link_RF = 0.16099999845027924;
const Scalar ix_shank_link_RF = 5.199999941396527E-5;
const Scalar iy_shank_link_RF = 0.015900999307632446;
const Scalar iz_shank_link_RF = 0.015900999307632446;
const Scalar m_hip_link_LH = 0.6000000238418579;
const Scalar comx_hip_link_LH = 0.03999999910593033;
const Scalar ix_hip_link_LH = 6.199999916134402E-5;
const Scalar iy_hip_link_LH = 0.0013099999632686377;
const Scalar iz_hip_link_LH = 0.0013099999632686377;
const Scalar m_thigh_link_LH = 1.2000000476837158;
const Scalar comx_thigh_link_LH = 0.10000000149011612;
const Scalar ix_thigh_link_LH = 1.250000059371814E-4;
const Scalar iy_thigh_link_LH = 0.01604600064456463;
const Scalar iz_thigh_link_LH = 0.01604600064456463;
const Scalar m_shank_link_LH = 0.5;
const Scalar comx_shank_link_LH = 0.16099999845027924;
const Scalar ix_shank_link_LH = 5.199999941396527E-5;
const Scalar iy_shank_link_LH = 0.015900999307632446;
const Scalar iz_shank_link_LH = 0.015900999307632446;
const Scalar m_hip_link_RH = 0.6000000238418579;
const Scalar comx_hip_link_RH = 0.03999999910593033;
const Scalar ix_hip_link_RH = 6.199999916134402E-5;
const Scalar iy_hip_link_RH = 0.0013099999632686377;
const Scalar iz_hip_link_RH = 0.0013099999632686377;
const Scalar m_thigh_link_RH = 1.2000000476837158;
const Scalar comx_thigh_link_RH = 0.10000000149011612;
const Scalar ix_thigh_link_RH = 1.250000059371814E-4;
const Scalar iy_thigh_link_RH = 0.01604600064456463;
const Scalar iz_thigh_link_RH = 0.01604600064456463;
const Scalar m_shank_link_RH = 0.5;
const Scalar comx_shank_link_RH = 0.16099999845027924;
const Scalar ix_shank_link_RH = 5.199999941396527E-5;
const Scalar iy_shank_link_RH = 0.015900999307632446;
const Scalar iz_shank_link_RH = 0.015900999307632446;

}
}
#endif
