#include "forward_dynamics.h"

#include <Eigen/Cholesky>
#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

// Initialization of static-const data
const rcg::shvan1::dyn::ForwardDynamics::ExtForces
    rcg::shvan1::dyn::ForwardDynamics::zeroExtForces(Force::Zero());

rcg::shvan1::dyn::ForwardDynamics::ForwardDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    hip_link_LF_v.setZero();
    hip_link_LF_c.setZero();
    thigh_link_LF_v.setZero();
    thigh_link_LF_c.setZero();
    shank_link_LF_v.setZero();
    shank_link_LF_c.setZero();
    hip_link_RF_v.setZero();
    hip_link_RF_c.setZero();
    thigh_link_RF_v.setZero();
    thigh_link_RF_c.setZero();
    shank_link_RF_v.setZero();
    shank_link_RF_c.setZero();
    hip_link_LH_v.setZero();
    hip_link_LH_c.setZero();
    thigh_link_LH_v.setZero();
    thigh_link_LH_c.setZero();
    shank_link_LH_v.setZero();
    shank_link_LH_c.setZero();
    hip_link_RH_v.setZero();
    hip_link_RH_c.setZero();
    thigh_link_RH_v.setZero();
    thigh_link_RH_c.setZero();
    shank_link_RH_v.setZero();
    shank_link_RH_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

void rcg::shvan1::dyn::ForwardDynamics::fd(
    JointState& qdd,
    Acceleration& base_a,
    const Velocity& base_v,
    const Acceleration& g,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    base_AI = inertiaProps->getTensor_base();
    base_p = - fext[BASE];
    hip_link_LF_AI = inertiaProps->getTensor_hip_link_LF();
    hip_link_LF_p = - fext[HIP_LINK_LF];
    thigh_link_LF_AI = inertiaProps->getTensor_thigh_link_LF();
    thigh_link_LF_p = - fext[THIGH_LINK_LF];
    shank_link_LF_AI = inertiaProps->getTensor_shank_link_LF();
    shank_link_LF_p = - fext[SHANK_LINK_LF];
    hip_link_RF_AI = inertiaProps->getTensor_hip_link_RF();
    hip_link_RF_p = - fext[HIP_LINK_RF];
    thigh_link_RF_AI = inertiaProps->getTensor_thigh_link_RF();
    thigh_link_RF_p = - fext[THIGH_LINK_RF];
    shank_link_RF_AI = inertiaProps->getTensor_shank_link_RF();
    shank_link_RF_p = - fext[SHANK_LINK_RF];
    hip_link_LH_AI = inertiaProps->getTensor_hip_link_LH();
    hip_link_LH_p = - fext[HIP_LINK_LH];
    thigh_link_LH_AI = inertiaProps->getTensor_thigh_link_LH();
    thigh_link_LH_p = - fext[THIGH_LINK_LH];
    shank_link_LH_AI = inertiaProps->getTensor_shank_link_LH();
    shank_link_LH_p = - fext[SHANK_LINK_LH];
    hip_link_RH_AI = inertiaProps->getTensor_hip_link_RH();
    hip_link_RH_p = - fext[HIP_LINK_RH];
    thigh_link_RH_AI = inertiaProps->getTensor_thigh_link_RH();
    thigh_link_RH_p = - fext[THIGH_LINK_RH];
    shank_link_RH_AI = inertiaProps->getTensor_shank_link_RH();
    shank_link_RH_p = - fext[SHANK_LINK_RH];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link hip_link_LF
    //  - The spatial velocity:
    hip_link_LF_v = (motionTransforms-> fr_hip_link_LF_X_fr_base) * base_v;
    hip_link_LF_v(AZ) += qd(HIP_JOINT_LF);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(hip_link_LF_v, vcross);
    hip_link_LF_c = vcross.col(AZ) * qd(HIP_JOINT_LF);
    
    //  - The bias force term:
    hip_link_LF_p += vxIv(hip_link_LF_v, hip_link_LF_AI);
    
    // + Link thigh_link_LF
    //  - The spatial velocity:
    thigh_link_LF_v = (motionTransforms-> fr_thigh_link_LF_X_fr_hip_link_LF) * hip_link_LF_v;
    thigh_link_LF_v(AZ) += qd(ELBOW_JOINT_LF);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(thigh_link_LF_v, vcross);
    thigh_link_LF_c = vcross.col(AZ) * qd(ELBOW_JOINT_LF);
    
    //  - The bias force term:
    thigh_link_LF_p += vxIv(thigh_link_LF_v, thigh_link_LF_AI);
    
    // + Link shank_link_LF
    //  - The spatial velocity:
    shank_link_LF_v = (motionTransforms-> fr_shank_link_LF_X_fr_thigh_link_LF) * thigh_link_LF_v;
    shank_link_LF_v(AZ) += qd(KNEE_JOINT_LF);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(shank_link_LF_v, vcross);
    shank_link_LF_c = vcross.col(AZ) * qd(KNEE_JOINT_LF);
    
    //  - The bias force term:
    shank_link_LF_p += vxIv(shank_link_LF_v, shank_link_LF_AI);
    
    // + Link hip_link_RF
    //  - The spatial velocity:
    hip_link_RF_v = (motionTransforms-> fr_hip_link_RF_X_fr_base) * base_v;
    hip_link_RF_v(AZ) += qd(HIP_JOINT_RF);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(hip_link_RF_v, vcross);
    hip_link_RF_c = vcross.col(AZ) * qd(HIP_JOINT_RF);
    
    //  - The bias force term:
    hip_link_RF_p += vxIv(hip_link_RF_v, hip_link_RF_AI);
    
    // + Link thigh_link_RF
    //  - The spatial velocity:
    thigh_link_RF_v = (motionTransforms-> fr_thigh_link_RF_X_fr_hip_link_RF) * hip_link_RF_v;
    thigh_link_RF_v(AZ) += qd(ELBOW_JOINT_RF);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(thigh_link_RF_v, vcross);
    thigh_link_RF_c = vcross.col(AZ) * qd(ELBOW_JOINT_RF);
    
    //  - The bias force term:
    thigh_link_RF_p += vxIv(thigh_link_RF_v, thigh_link_RF_AI);
    
    // + Link shank_link_RF
    //  - The spatial velocity:
    shank_link_RF_v = (motionTransforms-> fr_shank_link_RF_X_fr_thigh_link_RF) * thigh_link_RF_v;
    shank_link_RF_v(AZ) += qd(KNEE_JOINT_RF);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(shank_link_RF_v, vcross);
    shank_link_RF_c = vcross.col(AZ) * qd(KNEE_JOINT_RF);
    
    //  - The bias force term:
    shank_link_RF_p += vxIv(shank_link_RF_v, shank_link_RF_AI);
    
    // + Link hip_link_LH
    //  - The spatial velocity:
    hip_link_LH_v = (motionTransforms-> fr_hip_link_LH_X_fr_base) * base_v;
    hip_link_LH_v(AZ) += qd(HIP_JOINT_LH);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(hip_link_LH_v, vcross);
    hip_link_LH_c = vcross.col(AZ) * qd(HIP_JOINT_LH);
    
    //  - The bias force term:
    hip_link_LH_p += vxIv(hip_link_LH_v, hip_link_LH_AI);
    
    // + Link thigh_link_LH
    //  - The spatial velocity:
    thigh_link_LH_v = (motionTransforms-> fr_thigh_link_LH_X_fr_hip_link_LH) * hip_link_LH_v;
    thigh_link_LH_v(AZ) += qd(ELBOW_JOINT_LH);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(thigh_link_LH_v, vcross);
    thigh_link_LH_c = vcross.col(AZ) * qd(ELBOW_JOINT_LH);
    
    //  - The bias force term:
    thigh_link_LH_p += vxIv(thigh_link_LH_v, thigh_link_LH_AI);
    
    // + Link shank_link_LH
    //  - The spatial velocity:
    shank_link_LH_v = (motionTransforms-> fr_shank_link_LH_X_fr_thigh_link_LH) * thigh_link_LH_v;
    shank_link_LH_v(AZ) += qd(KNEE_JOINT_LH);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(shank_link_LH_v, vcross);
    shank_link_LH_c = vcross.col(AZ) * qd(KNEE_JOINT_LH);
    
    //  - The bias force term:
    shank_link_LH_p += vxIv(shank_link_LH_v, shank_link_LH_AI);
    
    // + Link hip_link_RH
    //  - The spatial velocity:
    hip_link_RH_v = (motionTransforms-> fr_hip_link_RH_X_fr_base) * base_v;
    hip_link_RH_v(AZ) += qd(HIP_JOINT_RH);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(hip_link_RH_v, vcross);
    hip_link_RH_c = vcross.col(AZ) * qd(HIP_JOINT_RH);
    
    //  - The bias force term:
    hip_link_RH_p += vxIv(hip_link_RH_v, hip_link_RH_AI);
    
    // + Link thigh_link_RH
    //  - The spatial velocity:
    thigh_link_RH_v = (motionTransforms-> fr_thigh_link_RH_X_fr_hip_link_RH) * hip_link_RH_v;
    thigh_link_RH_v(AZ) += qd(ELBOW_JOINT_RH);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(thigh_link_RH_v, vcross);
    thigh_link_RH_c = vcross.col(AZ) * qd(ELBOW_JOINT_RH);
    
    //  - The bias force term:
    thigh_link_RH_p += vxIv(thigh_link_RH_v, thigh_link_RH_AI);
    
    // + Link shank_link_RH
    //  - The spatial velocity:
    shank_link_RH_v = (motionTransforms-> fr_shank_link_RH_X_fr_thigh_link_RH) * thigh_link_RH_v;
    shank_link_RH_v(AZ) += qd(KNEE_JOINT_RH);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(shank_link_RH_v, vcross);
    shank_link_RH_c = vcross.col(AZ) * qd(KNEE_JOINT_RH);
    
    //  - The bias force term:
    shank_link_RH_p += vxIv(shank_link_RH_v, shank_link_RH_AI);
    
    // + The floating base body
    base_p += vxIv(base_v, base_AI);
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66 IaB;
    Force pa;
    
    // + Link shank_link_RH
    shank_link_RH_u = tau(KNEE_JOINT_RH) - shank_link_RH_p(AZ);
    shank_link_RH_U = shank_link_RH_AI.col(AZ);
    shank_link_RH_D = shank_link_RH_U(AZ);
    
    compute_Ia_revolute(shank_link_RH_AI, shank_link_RH_U, shank_link_RH_D, Ia_r);  // same as: Ia_r = shank_link_RH_AI - shank_link_RH_U/shank_link_RH_D * shank_link_RH_U.transpose();
    pa = shank_link_RH_p + Ia_r * shank_link_RH_c + shank_link_RH_U * shank_link_RH_u/shank_link_RH_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_shank_link_RH_X_fr_thigh_link_RH, IaB);
    thigh_link_RH_AI += IaB;
    thigh_link_RH_p += (motionTransforms-> fr_shank_link_RH_X_fr_thigh_link_RH).transpose() * pa;
    
    // + Link thigh_link_RH
    thigh_link_RH_u = tau(ELBOW_JOINT_RH) - thigh_link_RH_p(AZ);
    thigh_link_RH_U = thigh_link_RH_AI.col(AZ);
    thigh_link_RH_D = thigh_link_RH_U(AZ);
    
    compute_Ia_revolute(thigh_link_RH_AI, thigh_link_RH_U, thigh_link_RH_D, Ia_r);  // same as: Ia_r = thigh_link_RH_AI - thigh_link_RH_U/thigh_link_RH_D * thigh_link_RH_U.transpose();
    pa = thigh_link_RH_p + Ia_r * thigh_link_RH_c + thigh_link_RH_U * thigh_link_RH_u/thigh_link_RH_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_thigh_link_RH_X_fr_hip_link_RH, IaB);
    hip_link_RH_AI += IaB;
    hip_link_RH_p += (motionTransforms-> fr_thigh_link_RH_X_fr_hip_link_RH).transpose() * pa;
    
    // + Link hip_link_RH
    hip_link_RH_u = tau(HIP_JOINT_RH) - hip_link_RH_p(AZ);
    hip_link_RH_U = hip_link_RH_AI.col(AZ);
    hip_link_RH_D = hip_link_RH_U(AZ);
    
    compute_Ia_revolute(hip_link_RH_AI, hip_link_RH_U, hip_link_RH_D, Ia_r);  // same as: Ia_r = hip_link_RH_AI - hip_link_RH_U/hip_link_RH_D * hip_link_RH_U.transpose();
    pa = hip_link_RH_p + Ia_r * hip_link_RH_c + hip_link_RH_U * hip_link_RH_u/hip_link_RH_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_hip_link_RH_X_fr_base, IaB);
    base_AI += IaB;
    base_p += (motionTransforms-> fr_hip_link_RH_X_fr_base).transpose() * pa;
    
    // + Link shank_link_LH
    shank_link_LH_u = tau(KNEE_JOINT_LH) - shank_link_LH_p(AZ);
    shank_link_LH_U = shank_link_LH_AI.col(AZ);
    shank_link_LH_D = shank_link_LH_U(AZ);
    
    compute_Ia_revolute(shank_link_LH_AI, shank_link_LH_U, shank_link_LH_D, Ia_r);  // same as: Ia_r = shank_link_LH_AI - shank_link_LH_U/shank_link_LH_D * shank_link_LH_U.transpose();
    pa = shank_link_LH_p + Ia_r * shank_link_LH_c + shank_link_LH_U * shank_link_LH_u/shank_link_LH_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_shank_link_LH_X_fr_thigh_link_LH, IaB);
    thigh_link_LH_AI += IaB;
    thigh_link_LH_p += (motionTransforms-> fr_shank_link_LH_X_fr_thigh_link_LH).transpose() * pa;
    
    // + Link thigh_link_LH
    thigh_link_LH_u = tau(ELBOW_JOINT_LH) - thigh_link_LH_p(AZ);
    thigh_link_LH_U = thigh_link_LH_AI.col(AZ);
    thigh_link_LH_D = thigh_link_LH_U(AZ);
    
    compute_Ia_revolute(thigh_link_LH_AI, thigh_link_LH_U, thigh_link_LH_D, Ia_r);  // same as: Ia_r = thigh_link_LH_AI - thigh_link_LH_U/thigh_link_LH_D * thigh_link_LH_U.transpose();
    pa = thigh_link_LH_p + Ia_r * thigh_link_LH_c + thigh_link_LH_U * thigh_link_LH_u/thigh_link_LH_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_thigh_link_LH_X_fr_hip_link_LH, IaB);
    hip_link_LH_AI += IaB;
    hip_link_LH_p += (motionTransforms-> fr_thigh_link_LH_X_fr_hip_link_LH).transpose() * pa;
    
    // + Link hip_link_LH
    hip_link_LH_u = tau(HIP_JOINT_LH) - hip_link_LH_p(AZ);
    hip_link_LH_U = hip_link_LH_AI.col(AZ);
    hip_link_LH_D = hip_link_LH_U(AZ);
    
    compute_Ia_revolute(hip_link_LH_AI, hip_link_LH_U, hip_link_LH_D, Ia_r);  // same as: Ia_r = hip_link_LH_AI - hip_link_LH_U/hip_link_LH_D * hip_link_LH_U.transpose();
    pa = hip_link_LH_p + Ia_r * hip_link_LH_c + hip_link_LH_U * hip_link_LH_u/hip_link_LH_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_hip_link_LH_X_fr_base, IaB);
    base_AI += IaB;
    base_p += (motionTransforms-> fr_hip_link_LH_X_fr_base).transpose() * pa;
    
    // + Link shank_link_RF
    shank_link_RF_u = tau(KNEE_JOINT_RF) - shank_link_RF_p(AZ);
    shank_link_RF_U = shank_link_RF_AI.col(AZ);
    shank_link_RF_D = shank_link_RF_U(AZ);
    
    compute_Ia_revolute(shank_link_RF_AI, shank_link_RF_U, shank_link_RF_D, Ia_r);  // same as: Ia_r = shank_link_RF_AI - shank_link_RF_U/shank_link_RF_D * shank_link_RF_U.transpose();
    pa = shank_link_RF_p + Ia_r * shank_link_RF_c + shank_link_RF_U * shank_link_RF_u/shank_link_RF_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_shank_link_RF_X_fr_thigh_link_RF, IaB);
    thigh_link_RF_AI += IaB;
    thigh_link_RF_p += (motionTransforms-> fr_shank_link_RF_X_fr_thigh_link_RF).transpose() * pa;
    
    // + Link thigh_link_RF
    thigh_link_RF_u = tau(ELBOW_JOINT_RF) - thigh_link_RF_p(AZ);
    thigh_link_RF_U = thigh_link_RF_AI.col(AZ);
    thigh_link_RF_D = thigh_link_RF_U(AZ);
    
    compute_Ia_revolute(thigh_link_RF_AI, thigh_link_RF_U, thigh_link_RF_D, Ia_r);  // same as: Ia_r = thigh_link_RF_AI - thigh_link_RF_U/thigh_link_RF_D * thigh_link_RF_U.transpose();
    pa = thigh_link_RF_p + Ia_r * thigh_link_RF_c + thigh_link_RF_U * thigh_link_RF_u/thigh_link_RF_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_thigh_link_RF_X_fr_hip_link_RF, IaB);
    hip_link_RF_AI += IaB;
    hip_link_RF_p += (motionTransforms-> fr_thigh_link_RF_X_fr_hip_link_RF).transpose() * pa;
    
    // + Link hip_link_RF
    hip_link_RF_u = tau(HIP_JOINT_RF) - hip_link_RF_p(AZ);
    hip_link_RF_U = hip_link_RF_AI.col(AZ);
    hip_link_RF_D = hip_link_RF_U(AZ);
    
    compute_Ia_revolute(hip_link_RF_AI, hip_link_RF_U, hip_link_RF_D, Ia_r);  // same as: Ia_r = hip_link_RF_AI - hip_link_RF_U/hip_link_RF_D * hip_link_RF_U.transpose();
    pa = hip_link_RF_p + Ia_r * hip_link_RF_c + hip_link_RF_U * hip_link_RF_u/hip_link_RF_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_hip_link_RF_X_fr_base, IaB);
    base_AI += IaB;
    base_p += (motionTransforms-> fr_hip_link_RF_X_fr_base).transpose() * pa;
    
    // + Link shank_link_LF
    shank_link_LF_u = tau(KNEE_JOINT_LF) - shank_link_LF_p(AZ);
    shank_link_LF_U = shank_link_LF_AI.col(AZ);
    shank_link_LF_D = shank_link_LF_U(AZ);
    
    compute_Ia_revolute(shank_link_LF_AI, shank_link_LF_U, shank_link_LF_D, Ia_r);  // same as: Ia_r = shank_link_LF_AI - shank_link_LF_U/shank_link_LF_D * shank_link_LF_U.transpose();
    pa = shank_link_LF_p + Ia_r * shank_link_LF_c + shank_link_LF_U * shank_link_LF_u/shank_link_LF_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_shank_link_LF_X_fr_thigh_link_LF, IaB);
    thigh_link_LF_AI += IaB;
    thigh_link_LF_p += (motionTransforms-> fr_shank_link_LF_X_fr_thigh_link_LF).transpose() * pa;
    
    // + Link thigh_link_LF
    thigh_link_LF_u = tau(ELBOW_JOINT_LF) - thigh_link_LF_p(AZ);
    thigh_link_LF_U = thigh_link_LF_AI.col(AZ);
    thigh_link_LF_D = thigh_link_LF_U(AZ);
    
    compute_Ia_revolute(thigh_link_LF_AI, thigh_link_LF_U, thigh_link_LF_D, Ia_r);  // same as: Ia_r = thigh_link_LF_AI - thigh_link_LF_U/thigh_link_LF_D * thigh_link_LF_U.transpose();
    pa = thigh_link_LF_p + Ia_r * thigh_link_LF_c + thigh_link_LF_U * thigh_link_LF_u/thigh_link_LF_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_thigh_link_LF_X_fr_hip_link_LF, IaB);
    hip_link_LF_AI += IaB;
    hip_link_LF_p += (motionTransforms-> fr_thigh_link_LF_X_fr_hip_link_LF).transpose() * pa;
    
    // + Link hip_link_LF
    hip_link_LF_u = tau(HIP_JOINT_LF) - hip_link_LF_p(AZ);
    hip_link_LF_U = hip_link_LF_AI.col(AZ);
    hip_link_LF_D = hip_link_LF_U(AZ);
    
    compute_Ia_revolute(hip_link_LF_AI, hip_link_LF_U, hip_link_LF_D, Ia_r);  // same as: Ia_r = hip_link_LF_AI - hip_link_LF_U/hip_link_LF_D * hip_link_LF_U.transpose();
    pa = hip_link_LF_p + Ia_r * hip_link_LF_c + hip_link_LF_U * hip_link_LF_u/hip_link_LF_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_hip_link_LF_X_fr_base, IaB);
    base_AI += IaB;
    base_p += (motionTransforms-> fr_hip_link_LF_X_fr_base).transpose() * pa;
    
    // + The acceleration of the floating base base, without gravity
    Eigen::LLT<Matrix66d> llt(base_AI);
    base_a = - llt.solve(base_p);  // base_a = - IA^-1 * base_p
    
    // ---------------------- THIRD PASS ---------------------- //
    hip_link_LF_a = (motionTransforms-> fr_hip_link_LF_X_fr_base) * base_a + hip_link_LF_c;
    qdd(HIP_JOINT_LF) = (hip_link_LF_u - hip_link_LF_U.dot(hip_link_LF_a)) / hip_link_LF_D;
    hip_link_LF_a(AZ) += qdd(HIP_JOINT_LF);
    
    thigh_link_LF_a = (motionTransforms-> fr_thigh_link_LF_X_fr_hip_link_LF) * hip_link_LF_a + thigh_link_LF_c;
    qdd(ELBOW_JOINT_LF) = (thigh_link_LF_u - thigh_link_LF_U.dot(thigh_link_LF_a)) / thigh_link_LF_D;
    thigh_link_LF_a(AZ) += qdd(ELBOW_JOINT_LF);
    
    shank_link_LF_a = (motionTransforms-> fr_shank_link_LF_X_fr_thigh_link_LF) * thigh_link_LF_a + shank_link_LF_c;
    qdd(KNEE_JOINT_LF) = (shank_link_LF_u - shank_link_LF_U.dot(shank_link_LF_a)) / shank_link_LF_D;
    shank_link_LF_a(AZ) += qdd(KNEE_JOINT_LF);
    
    hip_link_RF_a = (motionTransforms-> fr_hip_link_RF_X_fr_base) * base_a + hip_link_RF_c;
    qdd(HIP_JOINT_RF) = (hip_link_RF_u - hip_link_RF_U.dot(hip_link_RF_a)) / hip_link_RF_D;
    hip_link_RF_a(AZ) += qdd(HIP_JOINT_RF);
    
    thigh_link_RF_a = (motionTransforms-> fr_thigh_link_RF_X_fr_hip_link_RF) * hip_link_RF_a + thigh_link_RF_c;
    qdd(ELBOW_JOINT_RF) = (thigh_link_RF_u - thigh_link_RF_U.dot(thigh_link_RF_a)) / thigh_link_RF_D;
    thigh_link_RF_a(AZ) += qdd(ELBOW_JOINT_RF);
    
    shank_link_RF_a = (motionTransforms-> fr_shank_link_RF_X_fr_thigh_link_RF) * thigh_link_RF_a + shank_link_RF_c;
    qdd(KNEE_JOINT_RF) = (shank_link_RF_u - shank_link_RF_U.dot(shank_link_RF_a)) / shank_link_RF_D;
    shank_link_RF_a(AZ) += qdd(KNEE_JOINT_RF);
    
    hip_link_LH_a = (motionTransforms-> fr_hip_link_LH_X_fr_base) * base_a + hip_link_LH_c;
    qdd(HIP_JOINT_LH) = (hip_link_LH_u - hip_link_LH_U.dot(hip_link_LH_a)) / hip_link_LH_D;
    hip_link_LH_a(AZ) += qdd(HIP_JOINT_LH);
    
    thigh_link_LH_a = (motionTransforms-> fr_thigh_link_LH_X_fr_hip_link_LH) * hip_link_LH_a + thigh_link_LH_c;
    qdd(ELBOW_JOINT_LH) = (thigh_link_LH_u - thigh_link_LH_U.dot(thigh_link_LH_a)) / thigh_link_LH_D;
    thigh_link_LH_a(AZ) += qdd(ELBOW_JOINT_LH);
    
    shank_link_LH_a = (motionTransforms-> fr_shank_link_LH_X_fr_thigh_link_LH) * thigh_link_LH_a + shank_link_LH_c;
    qdd(KNEE_JOINT_LH) = (shank_link_LH_u - shank_link_LH_U.dot(shank_link_LH_a)) / shank_link_LH_D;
    shank_link_LH_a(AZ) += qdd(KNEE_JOINT_LH);
    
    hip_link_RH_a = (motionTransforms-> fr_hip_link_RH_X_fr_base) * base_a + hip_link_RH_c;
    qdd(HIP_JOINT_RH) = (hip_link_RH_u - hip_link_RH_U.dot(hip_link_RH_a)) / hip_link_RH_D;
    hip_link_RH_a(AZ) += qdd(HIP_JOINT_RH);
    
    thigh_link_RH_a = (motionTransforms-> fr_thigh_link_RH_X_fr_hip_link_RH) * hip_link_RH_a + thigh_link_RH_c;
    qdd(ELBOW_JOINT_RH) = (thigh_link_RH_u - thigh_link_RH_U.dot(thigh_link_RH_a)) / thigh_link_RH_D;
    thigh_link_RH_a(AZ) += qdd(ELBOW_JOINT_RH);
    
    shank_link_RH_a = (motionTransforms-> fr_shank_link_RH_X_fr_thigh_link_RH) * thigh_link_RH_a + shank_link_RH_c;
    qdd(KNEE_JOINT_RH) = (shank_link_RH_u - shank_link_RH_U.dot(shank_link_RH_a)) / shank_link_RH_D;
    shank_link_RH_a(AZ) += qdd(KNEE_JOINT_RH);
    
    
    // + Add gravity to the acceleration of the floating base
    base_a += g;
}
