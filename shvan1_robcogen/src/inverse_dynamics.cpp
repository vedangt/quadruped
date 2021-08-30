#include <iit/rbd/robcogen_commons.h>

#include "inverse_dynamics.h"
#include "inertia_properties.h"
#ifndef EIGEN_NO_DEBUG
    #include <iostream>
#endif
using namespace std;
using namespace iit::rbd;
using namespace rcg::shvan1::dyn;

// Initialization of static-const data
const rcg::shvan1::dyn::InverseDynamics::ExtForces
rcg::shvan1::dyn::InverseDynamics::zeroExtForces(Force::Zero());

rcg::shvan1::dyn::InverseDynamics::InverseDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    hip_link_LF_I(inertiaProps->getTensor_hip_link_LF() ),
    thigh_link_LF_I(inertiaProps->getTensor_thigh_link_LF() ),
    shank_link_LF_I(inertiaProps->getTensor_shank_link_LF() ),
    hip_link_RF_I(inertiaProps->getTensor_hip_link_RF() ),
    thigh_link_RF_I(inertiaProps->getTensor_thigh_link_RF() ),
    shank_link_RF_I(inertiaProps->getTensor_shank_link_RF() ),
    hip_link_LH_I(inertiaProps->getTensor_hip_link_LH() ),
    thigh_link_LH_I(inertiaProps->getTensor_thigh_link_LH() ),
    shank_link_LH_I(inertiaProps->getTensor_shank_link_LH() ),
    hip_link_RH_I(inertiaProps->getTensor_hip_link_RH() ),
    thigh_link_RH_I(inertiaProps->getTensor_thigh_link_RH() ),
    shank_link_RH_I(inertiaProps->getTensor_shank_link_RH() )
    ,
        base_I( inertiaProps->getTensor_base() ),
        shank_link_LF_Ic(shank_link_LF_I),
        shank_link_RF_Ic(shank_link_RF_I),
        shank_link_LH_Ic(shank_link_LH_I),
        shank_link_RH_Ic(shank_link_RH_I)
{
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot shvan1, InverseDynamics::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    hip_link_LF_v.setZero();
    thigh_link_LF_v.setZero();
    shank_link_LF_v.setZero();
    hip_link_RF_v.setZero();
    thigh_link_RF_v.setZero();
    shank_link_RF_v.setZero();
    hip_link_LH_v.setZero();
    thigh_link_LH_v.setZero();
    shank_link_LH_v.setZero();
    hip_link_RH_v.setZero();
    thigh_link_RH_v.setZero();
    shank_link_RH_v.setZero();

    vcross.setZero();
}

void rcg::shvan1::dyn::InverseDynamics::id(
    JointState& jForces, Acceleration& base_a,
    const Acceleration& g, const Velocity& base_v,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    base_Ic = base_I;
    hip_link_LF_Ic = hip_link_LF_I;
    thigh_link_LF_Ic = thigh_link_LF_I;
    hip_link_RF_Ic = hip_link_RF_I;
    thigh_link_RF_Ic = thigh_link_RF_I;
    hip_link_LH_Ic = hip_link_LH_I;
    thigh_link_LH_Ic = thigh_link_LH_I;
    hip_link_RH_Ic = hip_link_RH_I;
    thigh_link_RH_Ic = thigh_link_RH_I;

    // First pass, link 'hip_link_LF'
    hip_link_LF_v = ((xm->fr_hip_link_LF_X_fr_base) * base_v);
    hip_link_LF_v(iit::rbd::AZ) += qd(HIP_JOINT_LF);
    
    motionCrossProductMx<Scalar>(hip_link_LF_v, vcross);
    
    hip_link_LF_a = (vcross.col(iit::rbd::AZ) * qd(HIP_JOINT_LF));
    hip_link_LF_a(iit::rbd::AZ) += qdd(HIP_JOINT_LF);
    
    hip_link_LF_f = hip_link_LF_I * hip_link_LF_a + vxIv(hip_link_LF_v, hip_link_LF_I);
    
    // First pass, link 'thigh_link_LF'
    thigh_link_LF_v = ((xm->fr_thigh_link_LF_X_fr_hip_link_LF) * hip_link_LF_v);
    thigh_link_LF_v(iit::rbd::AZ) += qd(ELBOW_JOINT_LF);
    
    motionCrossProductMx<Scalar>(thigh_link_LF_v, vcross);
    
    thigh_link_LF_a = (xm->fr_thigh_link_LF_X_fr_hip_link_LF) * hip_link_LF_a + vcross.col(iit::rbd::AZ) * qd(ELBOW_JOINT_LF);
    thigh_link_LF_a(iit::rbd::AZ) += qdd(ELBOW_JOINT_LF);
    
    thigh_link_LF_f = thigh_link_LF_I * thigh_link_LF_a + vxIv(thigh_link_LF_v, thigh_link_LF_I);
    
    // First pass, link 'shank_link_LF'
    shank_link_LF_v = ((xm->fr_shank_link_LF_X_fr_thigh_link_LF) * thigh_link_LF_v);
    shank_link_LF_v(iit::rbd::AZ) += qd(KNEE_JOINT_LF);
    
    motionCrossProductMx<Scalar>(shank_link_LF_v, vcross);
    
    shank_link_LF_a = (xm->fr_shank_link_LF_X_fr_thigh_link_LF) * thigh_link_LF_a + vcross.col(iit::rbd::AZ) * qd(KNEE_JOINT_LF);
    shank_link_LF_a(iit::rbd::AZ) += qdd(KNEE_JOINT_LF);
    
    shank_link_LF_f = shank_link_LF_I * shank_link_LF_a + vxIv(shank_link_LF_v, shank_link_LF_I);
    
    // First pass, link 'hip_link_RF'
    hip_link_RF_v = ((xm->fr_hip_link_RF_X_fr_base) * base_v);
    hip_link_RF_v(iit::rbd::AZ) += qd(HIP_JOINT_RF);
    
    motionCrossProductMx<Scalar>(hip_link_RF_v, vcross);
    
    hip_link_RF_a = (vcross.col(iit::rbd::AZ) * qd(HIP_JOINT_RF));
    hip_link_RF_a(iit::rbd::AZ) += qdd(HIP_JOINT_RF);
    
    hip_link_RF_f = hip_link_RF_I * hip_link_RF_a + vxIv(hip_link_RF_v, hip_link_RF_I);
    
    // First pass, link 'thigh_link_RF'
    thigh_link_RF_v = ((xm->fr_thigh_link_RF_X_fr_hip_link_RF) * hip_link_RF_v);
    thigh_link_RF_v(iit::rbd::AZ) += qd(ELBOW_JOINT_RF);
    
    motionCrossProductMx<Scalar>(thigh_link_RF_v, vcross);
    
    thigh_link_RF_a = (xm->fr_thigh_link_RF_X_fr_hip_link_RF) * hip_link_RF_a + vcross.col(iit::rbd::AZ) * qd(ELBOW_JOINT_RF);
    thigh_link_RF_a(iit::rbd::AZ) += qdd(ELBOW_JOINT_RF);
    
    thigh_link_RF_f = thigh_link_RF_I * thigh_link_RF_a + vxIv(thigh_link_RF_v, thigh_link_RF_I);
    
    // First pass, link 'shank_link_RF'
    shank_link_RF_v = ((xm->fr_shank_link_RF_X_fr_thigh_link_RF) * thigh_link_RF_v);
    shank_link_RF_v(iit::rbd::AZ) += qd(KNEE_JOINT_RF);
    
    motionCrossProductMx<Scalar>(shank_link_RF_v, vcross);
    
    shank_link_RF_a = (xm->fr_shank_link_RF_X_fr_thigh_link_RF) * thigh_link_RF_a + vcross.col(iit::rbd::AZ) * qd(KNEE_JOINT_RF);
    shank_link_RF_a(iit::rbd::AZ) += qdd(KNEE_JOINT_RF);
    
    shank_link_RF_f = shank_link_RF_I * shank_link_RF_a + vxIv(shank_link_RF_v, shank_link_RF_I);
    
    // First pass, link 'hip_link_LH'
    hip_link_LH_v = ((xm->fr_hip_link_LH_X_fr_base) * base_v);
    hip_link_LH_v(iit::rbd::AZ) += qd(HIP_JOINT_LH);
    
    motionCrossProductMx<Scalar>(hip_link_LH_v, vcross);
    
    hip_link_LH_a = (vcross.col(iit::rbd::AZ) * qd(HIP_JOINT_LH));
    hip_link_LH_a(iit::rbd::AZ) += qdd(HIP_JOINT_LH);
    
    hip_link_LH_f = hip_link_LH_I * hip_link_LH_a + vxIv(hip_link_LH_v, hip_link_LH_I);
    
    // First pass, link 'thigh_link_LH'
    thigh_link_LH_v = ((xm->fr_thigh_link_LH_X_fr_hip_link_LH) * hip_link_LH_v);
    thigh_link_LH_v(iit::rbd::AZ) += qd(ELBOW_JOINT_LH);
    
    motionCrossProductMx<Scalar>(thigh_link_LH_v, vcross);
    
    thigh_link_LH_a = (xm->fr_thigh_link_LH_X_fr_hip_link_LH) * hip_link_LH_a + vcross.col(iit::rbd::AZ) * qd(ELBOW_JOINT_LH);
    thigh_link_LH_a(iit::rbd::AZ) += qdd(ELBOW_JOINT_LH);
    
    thigh_link_LH_f = thigh_link_LH_I * thigh_link_LH_a + vxIv(thigh_link_LH_v, thigh_link_LH_I);
    
    // First pass, link 'shank_link_LH'
    shank_link_LH_v = ((xm->fr_shank_link_LH_X_fr_thigh_link_LH) * thigh_link_LH_v);
    shank_link_LH_v(iit::rbd::AZ) += qd(KNEE_JOINT_LH);
    
    motionCrossProductMx<Scalar>(shank_link_LH_v, vcross);
    
    shank_link_LH_a = (xm->fr_shank_link_LH_X_fr_thigh_link_LH) * thigh_link_LH_a + vcross.col(iit::rbd::AZ) * qd(KNEE_JOINT_LH);
    shank_link_LH_a(iit::rbd::AZ) += qdd(KNEE_JOINT_LH);
    
    shank_link_LH_f = shank_link_LH_I * shank_link_LH_a + vxIv(shank_link_LH_v, shank_link_LH_I);
    
    // First pass, link 'hip_link_RH'
    hip_link_RH_v = ((xm->fr_hip_link_RH_X_fr_base) * base_v);
    hip_link_RH_v(iit::rbd::AZ) += qd(HIP_JOINT_RH);
    
    motionCrossProductMx<Scalar>(hip_link_RH_v, vcross);
    
    hip_link_RH_a = (vcross.col(iit::rbd::AZ) * qd(HIP_JOINT_RH));
    hip_link_RH_a(iit::rbd::AZ) += qdd(HIP_JOINT_RH);
    
    hip_link_RH_f = hip_link_RH_I * hip_link_RH_a + vxIv(hip_link_RH_v, hip_link_RH_I);
    
    // First pass, link 'thigh_link_RH'
    thigh_link_RH_v = ((xm->fr_thigh_link_RH_X_fr_hip_link_RH) * hip_link_RH_v);
    thigh_link_RH_v(iit::rbd::AZ) += qd(ELBOW_JOINT_RH);
    
    motionCrossProductMx<Scalar>(thigh_link_RH_v, vcross);
    
    thigh_link_RH_a = (xm->fr_thigh_link_RH_X_fr_hip_link_RH) * hip_link_RH_a + vcross.col(iit::rbd::AZ) * qd(ELBOW_JOINT_RH);
    thigh_link_RH_a(iit::rbd::AZ) += qdd(ELBOW_JOINT_RH);
    
    thigh_link_RH_f = thigh_link_RH_I * thigh_link_RH_a + vxIv(thigh_link_RH_v, thigh_link_RH_I);
    
    // First pass, link 'shank_link_RH'
    shank_link_RH_v = ((xm->fr_shank_link_RH_X_fr_thigh_link_RH) * thigh_link_RH_v);
    shank_link_RH_v(iit::rbd::AZ) += qd(KNEE_JOINT_RH);
    
    motionCrossProductMx<Scalar>(shank_link_RH_v, vcross);
    
    shank_link_RH_a = (xm->fr_shank_link_RH_X_fr_thigh_link_RH) * thigh_link_RH_a + vcross.col(iit::rbd::AZ) * qd(KNEE_JOINT_RH);
    shank_link_RH_a(iit::rbd::AZ) += qdd(KNEE_JOINT_RH);
    
    shank_link_RH_f = shank_link_RH_I * shank_link_RH_a + vxIv(shank_link_RH_v, shank_link_RH_I);
    
    // The force exerted on the floating base by the links
    base_f = vxIv(base_v, base_I);
    

    // Add the external forces:
    base_f -= fext[BASE];
    hip_link_LF_f -= fext[HIP_LINK_LF];
    thigh_link_LF_f -= fext[THIGH_LINK_LF];
    shank_link_LF_f -= fext[SHANK_LINK_LF];
    hip_link_RF_f -= fext[HIP_LINK_RF];
    thigh_link_RF_f -= fext[THIGH_LINK_RF];
    shank_link_RF_f -= fext[SHANK_LINK_RF];
    hip_link_LH_f -= fext[HIP_LINK_LH];
    thigh_link_LH_f -= fext[THIGH_LINK_LH];
    shank_link_LH_f -= fext[SHANK_LINK_LH];
    hip_link_RH_f -= fext[HIP_LINK_RH];
    thigh_link_RH_f -= fext[THIGH_LINK_RH];
    shank_link_RH_f -= fext[SHANK_LINK_RH];

    InertiaMatrix Ic_spare;
    iit::rbd::transformInertia<Scalar>(shank_link_RH_Ic, (xm->fr_shank_link_RH_X_fr_thigh_link_RH).transpose(), Ic_spare);
    thigh_link_RH_Ic += Ic_spare;
    thigh_link_RH_f = thigh_link_RH_f + (xm->fr_shank_link_RH_X_fr_thigh_link_RH).transpose() * shank_link_RH_f;
    
    iit::rbd::transformInertia<Scalar>(thigh_link_RH_Ic, (xm->fr_thigh_link_RH_X_fr_hip_link_RH).transpose(), Ic_spare);
    hip_link_RH_Ic += Ic_spare;
    hip_link_RH_f = hip_link_RH_f + (xm->fr_thigh_link_RH_X_fr_hip_link_RH).transpose() * thigh_link_RH_f;
    
    iit::rbd::transformInertia<Scalar>(hip_link_RH_Ic, (xm->fr_hip_link_RH_X_fr_base).transpose(), Ic_spare);
    base_Ic += Ic_spare;
    base_f = base_f + (xm->fr_hip_link_RH_X_fr_base).transpose() * hip_link_RH_f;
    
    iit::rbd::transformInertia<Scalar>(shank_link_LH_Ic, (xm->fr_shank_link_LH_X_fr_thigh_link_LH).transpose(), Ic_spare);
    thigh_link_LH_Ic += Ic_spare;
    thigh_link_LH_f = thigh_link_LH_f + (xm->fr_shank_link_LH_X_fr_thigh_link_LH).transpose() * shank_link_LH_f;
    
    iit::rbd::transformInertia<Scalar>(thigh_link_LH_Ic, (xm->fr_thigh_link_LH_X_fr_hip_link_LH).transpose(), Ic_spare);
    hip_link_LH_Ic += Ic_spare;
    hip_link_LH_f = hip_link_LH_f + (xm->fr_thigh_link_LH_X_fr_hip_link_LH).transpose() * thigh_link_LH_f;
    
    iit::rbd::transformInertia<Scalar>(hip_link_LH_Ic, (xm->fr_hip_link_LH_X_fr_base).transpose(), Ic_spare);
    base_Ic += Ic_spare;
    base_f = base_f + (xm->fr_hip_link_LH_X_fr_base).transpose() * hip_link_LH_f;
    
    iit::rbd::transformInertia<Scalar>(shank_link_RF_Ic, (xm->fr_shank_link_RF_X_fr_thigh_link_RF).transpose(), Ic_spare);
    thigh_link_RF_Ic += Ic_spare;
    thigh_link_RF_f = thigh_link_RF_f + (xm->fr_shank_link_RF_X_fr_thigh_link_RF).transpose() * shank_link_RF_f;
    
    iit::rbd::transformInertia<Scalar>(thigh_link_RF_Ic, (xm->fr_thigh_link_RF_X_fr_hip_link_RF).transpose(), Ic_spare);
    hip_link_RF_Ic += Ic_spare;
    hip_link_RF_f = hip_link_RF_f + (xm->fr_thigh_link_RF_X_fr_hip_link_RF).transpose() * thigh_link_RF_f;
    
    iit::rbd::transformInertia<Scalar>(hip_link_RF_Ic, (xm->fr_hip_link_RF_X_fr_base).transpose(), Ic_spare);
    base_Ic += Ic_spare;
    base_f = base_f + (xm->fr_hip_link_RF_X_fr_base).transpose() * hip_link_RF_f;
    
    iit::rbd::transformInertia<Scalar>(shank_link_LF_Ic, (xm->fr_shank_link_LF_X_fr_thigh_link_LF).transpose(), Ic_spare);
    thigh_link_LF_Ic += Ic_spare;
    thigh_link_LF_f = thigh_link_LF_f + (xm->fr_shank_link_LF_X_fr_thigh_link_LF).transpose() * shank_link_LF_f;
    
    iit::rbd::transformInertia<Scalar>(thigh_link_LF_Ic, (xm->fr_thigh_link_LF_X_fr_hip_link_LF).transpose(), Ic_spare);
    hip_link_LF_Ic += Ic_spare;
    hip_link_LF_f = hip_link_LF_f + (xm->fr_thigh_link_LF_X_fr_hip_link_LF).transpose() * thigh_link_LF_f;
    
    iit::rbd::transformInertia<Scalar>(hip_link_LF_Ic, (xm->fr_hip_link_LF_X_fr_base).transpose(), Ic_spare);
    base_Ic += Ic_spare;
    base_f = base_f + (xm->fr_hip_link_LF_X_fr_base).transpose() * hip_link_LF_f;
    

    // The base acceleration due to the force due to the movement of the links
    base_a = - base_Ic.inverse() * base_f;
    
    hip_link_LF_a = xm->fr_hip_link_LF_X_fr_base * base_a;
    jForces(HIP_JOINT_LF) = (hip_link_LF_Ic.row(iit::rbd::AZ) * hip_link_LF_a + hip_link_LF_f(iit::rbd::AZ));
    
    thigh_link_LF_a = xm->fr_thigh_link_LF_X_fr_hip_link_LF * hip_link_LF_a;
    jForces(ELBOW_JOINT_LF) = (thigh_link_LF_Ic.row(iit::rbd::AZ) * thigh_link_LF_a + thigh_link_LF_f(iit::rbd::AZ));
    
    shank_link_LF_a = xm->fr_shank_link_LF_X_fr_thigh_link_LF * thigh_link_LF_a;
    jForces(KNEE_JOINT_LF) = (shank_link_LF_Ic.row(iit::rbd::AZ) * shank_link_LF_a + shank_link_LF_f(iit::rbd::AZ));
    
    hip_link_RF_a = xm->fr_hip_link_RF_X_fr_base * base_a;
    jForces(HIP_JOINT_RF) = (hip_link_RF_Ic.row(iit::rbd::AZ) * hip_link_RF_a + hip_link_RF_f(iit::rbd::AZ));
    
    thigh_link_RF_a = xm->fr_thigh_link_RF_X_fr_hip_link_RF * hip_link_RF_a;
    jForces(ELBOW_JOINT_RF) = (thigh_link_RF_Ic.row(iit::rbd::AZ) * thigh_link_RF_a + thigh_link_RF_f(iit::rbd::AZ));
    
    shank_link_RF_a = xm->fr_shank_link_RF_X_fr_thigh_link_RF * thigh_link_RF_a;
    jForces(KNEE_JOINT_RF) = (shank_link_RF_Ic.row(iit::rbd::AZ) * shank_link_RF_a + shank_link_RF_f(iit::rbd::AZ));
    
    hip_link_LH_a = xm->fr_hip_link_LH_X_fr_base * base_a;
    jForces(HIP_JOINT_LH) = (hip_link_LH_Ic.row(iit::rbd::AZ) * hip_link_LH_a + hip_link_LH_f(iit::rbd::AZ));
    
    thigh_link_LH_a = xm->fr_thigh_link_LH_X_fr_hip_link_LH * hip_link_LH_a;
    jForces(ELBOW_JOINT_LH) = (thigh_link_LH_Ic.row(iit::rbd::AZ) * thigh_link_LH_a + thigh_link_LH_f(iit::rbd::AZ));
    
    shank_link_LH_a = xm->fr_shank_link_LH_X_fr_thigh_link_LH * thigh_link_LH_a;
    jForces(KNEE_JOINT_LH) = (shank_link_LH_Ic.row(iit::rbd::AZ) * shank_link_LH_a + shank_link_LH_f(iit::rbd::AZ));
    
    hip_link_RH_a = xm->fr_hip_link_RH_X_fr_base * base_a;
    jForces(HIP_JOINT_RH) = (hip_link_RH_Ic.row(iit::rbd::AZ) * hip_link_RH_a + hip_link_RH_f(iit::rbd::AZ));
    
    thigh_link_RH_a = xm->fr_thigh_link_RH_X_fr_hip_link_RH * hip_link_RH_a;
    jForces(ELBOW_JOINT_RH) = (thigh_link_RH_Ic.row(iit::rbd::AZ) * thigh_link_RH_a + thigh_link_RH_f(iit::rbd::AZ));
    
    shank_link_RH_a = xm->fr_shank_link_RH_X_fr_thigh_link_RH * thigh_link_RH_a;
    jForces(KNEE_JOINT_RH) = (shank_link_RH_Ic.row(iit::rbd::AZ) * shank_link_RH_a + shank_link_RH_f(iit::rbd::AZ));
    

    base_a += g;
}


void rcg::shvan1::dyn::InverseDynamics::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g)
{
    const Acceleration& base_a = -g;

    // Link 'hip_link_LF'
    hip_link_LF_a = (xm->fr_hip_link_LF_X_fr_base) * base_a;
    hip_link_LF_f = hip_link_LF_I * hip_link_LF_a;
    // Link 'thigh_link_LF'
    thigh_link_LF_a = (xm->fr_thigh_link_LF_X_fr_hip_link_LF) * hip_link_LF_a;
    thigh_link_LF_f = thigh_link_LF_I * thigh_link_LF_a;
    // Link 'shank_link_LF'
    shank_link_LF_a = (xm->fr_shank_link_LF_X_fr_thigh_link_LF) * thigh_link_LF_a;
    shank_link_LF_f = shank_link_LF_I * shank_link_LF_a;
    // Link 'hip_link_RF'
    hip_link_RF_a = (xm->fr_hip_link_RF_X_fr_base) * base_a;
    hip_link_RF_f = hip_link_RF_I * hip_link_RF_a;
    // Link 'thigh_link_RF'
    thigh_link_RF_a = (xm->fr_thigh_link_RF_X_fr_hip_link_RF) * hip_link_RF_a;
    thigh_link_RF_f = thigh_link_RF_I * thigh_link_RF_a;
    // Link 'shank_link_RF'
    shank_link_RF_a = (xm->fr_shank_link_RF_X_fr_thigh_link_RF) * thigh_link_RF_a;
    shank_link_RF_f = shank_link_RF_I * shank_link_RF_a;
    // Link 'hip_link_LH'
    hip_link_LH_a = (xm->fr_hip_link_LH_X_fr_base) * base_a;
    hip_link_LH_f = hip_link_LH_I * hip_link_LH_a;
    // Link 'thigh_link_LH'
    thigh_link_LH_a = (xm->fr_thigh_link_LH_X_fr_hip_link_LH) * hip_link_LH_a;
    thigh_link_LH_f = thigh_link_LH_I * thigh_link_LH_a;
    // Link 'shank_link_LH'
    shank_link_LH_a = (xm->fr_shank_link_LH_X_fr_thigh_link_LH) * thigh_link_LH_a;
    shank_link_LH_f = shank_link_LH_I * shank_link_LH_a;
    // Link 'hip_link_RH'
    hip_link_RH_a = (xm->fr_hip_link_RH_X_fr_base) * base_a;
    hip_link_RH_f = hip_link_RH_I * hip_link_RH_a;
    // Link 'thigh_link_RH'
    thigh_link_RH_a = (xm->fr_thigh_link_RH_X_fr_hip_link_RH) * hip_link_RH_a;
    thigh_link_RH_f = thigh_link_RH_I * thigh_link_RH_a;
    // Link 'shank_link_RH'
    shank_link_RH_a = (xm->fr_shank_link_RH_X_fr_thigh_link_RH) * thigh_link_RH_a;
    shank_link_RH_f = shank_link_RH_I * shank_link_RH_a;

    base_f = base_I * base_a;

    secondPass_fullyActuated(jForces);

    baseWrench = base_f;
}

void rcg::shvan1::dyn::InverseDynamics::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_v, const JointState& qd)
{
    // Link 'hip_link_LF'
    hip_link_LF_v = ((xm->fr_hip_link_LF_X_fr_base) * base_v);
    hip_link_LF_v(iit::rbd::AZ) += qd(HIP_JOINT_LF);
    motionCrossProductMx<Scalar>(hip_link_LF_v, vcross);
    hip_link_LF_a = (vcross.col(iit::rbd::AZ) * qd(HIP_JOINT_LF));
    hip_link_LF_f = hip_link_LF_I * hip_link_LF_a + vxIv(hip_link_LF_v, hip_link_LF_I);
    
    // Link 'thigh_link_LF'
    thigh_link_LF_v = ((xm->fr_thigh_link_LF_X_fr_hip_link_LF) * hip_link_LF_v);
    thigh_link_LF_v(iit::rbd::AZ) += qd(ELBOW_JOINT_LF);
    motionCrossProductMx<Scalar>(thigh_link_LF_v, vcross);
    thigh_link_LF_a = (xm->fr_thigh_link_LF_X_fr_hip_link_LF) * hip_link_LF_a + vcross.col(iit::rbd::AZ) * qd(ELBOW_JOINT_LF);
    thigh_link_LF_f = thigh_link_LF_I * thigh_link_LF_a + vxIv(thigh_link_LF_v, thigh_link_LF_I);
    
    // Link 'shank_link_LF'
    shank_link_LF_v = ((xm->fr_shank_link_LF_X_fr_thigh_link_LF) * thigh_link_LF_v);
    shank_link_LF_v(iit::rbd::AZ) += qd(KNEE_JOINT_LF);
    motionCrossProductMx<Scalar>(shank_link_LF_v, vcross);
    shank_link_LF_a = (xm->fr_shank_link_LF_X_fr_thigh_link_LF) * thigh_link_LF_a + vcross.col(iit::rbd::AZ) * qd(KNEE_JOINT_LF);
    shank_link_LF_f = shank_link_LF_I * shank_link_LF_a + vxIv(shank_link_LF_v, shank_link_LF_I);
    
    // Link 'hip_link_RF'
    hip_link_RF_v = ((xm->fr_hip_link_RF_X_fr_base) * base_v);
    hip_link_RF_v(iit::rbd::AZ) += qd(HIP_JOINT_RF);
    motionCrossProductMx<Scalar>(hip_link_RF_v, vcross);
    hip_link_RF_a = (vcross.col(iit::rbd::AZ) * qd(HIP_JOINT_RF));
    hip_link_RF_f = hip_link_RF_I * hip_link_RF_a + vxIv(hip_link_RF_v, hip_link_RF_I);
    
    // Link 'thigh_link_RF'
    thigh_link_RF_v = ((xm->fr_thigh_link_RF_X_fr_hip_link_RF) * hip_link_RF_v);
    thigh_link_RF_v(iit::rbd::AZ) += qd(ELBOW_JOINT_RF);
    motionCrossProductMx<Scalar>(thigh_link_RF_v, vcross);
    thigh_link_RF_a = (xm->fr_thigh_link_RF_X_fr_hip_link_RF) * hip_link_RF_a + vcross.col(iit::rbd::AZ) * qd(ELBOW_JOINT_RF);
    thigh_link_RF_f = thigh_link_RF_I * thigh_link_RF_a + vxIv(thigh_link_RF_v, thigh_link_RF_I);
    
    // Link 'shank_link_RF'
    shank_link_RF_v = ((xm->fr_shank_link_RF_X_fr_thigh_link_RF) * thigh_link_RF_v);
    shank_link_RF_v(iit::rbd::AZ) += qd(KNEE_JOINT_RF);
    motionCrossProductMx<Scalar>(shank_link_RF_v, vcross);
    shank_link_RF_a = (xm->fr_shank_link_RF_X_fr_thigh_link_RF) * thigh_link_RF_a + vcross.col(iit::rbd::AZ) * qd(KNEE_JOINT_RF);
    shank_link_RF_f = shank_link_RF_I * shank_link_RF_a + vxIv(shank_link_RF_v, shank_link_RF_I);
    
    // Link 'hip_link_LH'
    hip_link_LH_v = ((xm->fr_hip_link_LH_X_fr_base) * base_v);
    hip_link_LH_v(iit::rbd::AZ) += qd(HIP_JOINT_LH);
    motionCrossProductMx<Scalar>(hip_link_LH_v, vcross);
    hip_link_LH_a = (vcross.col(iit::rbd::AZ) * qd(HIP_JOINT_LH));
    hip_link_LH_f = hip_link_LH_I * hip_link_LH_a + vxIv(hip_link_LH_v, hip_link_LH_I);
    
    // Link 'thigh_link_LH'
    thigh_link_LH_v = ((xm->fr_thigh_link_LH_X_fr_hip_link_LH) * hip_link_LH_v);
    thigh_link_LH_v(iit::rbd::AZ) += qd(ELBOW_JOINT_LH);
    motionCrossProductMx<Scalar>(thigh_link_LH_v, vcross);
    thigh_link_LH_a = (xm->fr_thigh_link_LH_X_fr_hip_link_LH) * hip_link_LH_a + vcross.col(iit::rbd::AZ) * qd(ELBOW_JOINT_LH);
    thigh_link_LH_f = thigh_link_LH_I * thigh_link_LH_a + vxIv(thigh_link_LH_v, thigh_link_LH_I);
    
    // Link 'shank_link_LH'
    shank_link_LH_v = ((xm->fr_shank_link_LH_X_fr_thigh_link_LH) * thigh_link_LH_v);
    shank_link_LH_v(iit::rbd::AZ) += qd(KNEE_JOINT_LH);
    motionCrossProductMx<Scalar>(shank_link_LH_v, vcross);
    shank_link_LH_a = (xm->fr_shank_link_LH_X_fr_thigh_link_LH) * thigh_link_LH_a + vcross.col(iit::rbd::AZ) * qd(KNEE_JOINT_LH);
    shank_link_LH_f = shank_link_LH_I * shank_link_LH_a + vxIv(shank_link_LH_v, shank_link_LH_I);
    
    // Link 'hip_link_RH'
    hip_link_RH_v = ((xm->fr_hip_link_RH_X_fr_base) * base_v);
    hip_link_RH_v(iit::rbd::AZ) += qd(HIP_JOINT_RH);
    motionCrossProductMx<Scalar>(hip_link_RH_v, vcross);
    hip_link_RH_a = (vcross.col(iit::rbd::AZ) * qd(HIP_JOINT_RH));
    hip_link_RH_f = hip_link_RH_I * hip_link_RH_a + vxIv(hip_link_RH_v, hip_link_RH_I);
    
    // Link 'thigh_link_RH'
    thigh_link_RH_v = ((xm->fr_thigh_link_RH_X_fr_hip_link_RH) * hip_link_RH_v);
    thigh_link_RH_v(iit::rbd::AZ) += qd(ELBOW_JOINT_RH);
    motionCrossProductMx<Scalar>(thigh_link_RH_v, vcross);
    thigh_link_RH_a = (xm->fr_thigh_link_RH_X_fr_hip_link_RH) * hip_link_RH_a + vcross.col(iit::rbd::AZ) * qd(ELBOW_JOINT_RH);
    thigh_link_RH_f = thigh_link_RH_I * thigh_link_RH_a + vxIv(thigh_link_RH_v, thigh_link_RH_I);
    
    // Link 'shank_link_RH'
    shank_link_RH_v = ((xm->fr_shank_link_RH_X_fr_thigh_link_RH) * thigh_link_RH_v);
    shank_link_RH_v(iit::rbd::AZ) += qd(KNEE_JOINT_RH);
    motionCrossProductMx<Scalar>(shank_link_RH_v, vcross);
    shank_link_RH_a = (xm->fr_shank_link_RH_X_fr_thigh_link_RH) * thigh_link_RH_a + vcross.col(iit::rbd::AZ) * qd(KNEE_JOINT_RH);
    shank_link_RH_f = shank_link_RH_I * shank_link_RH_a + vxIv(shank_link_RH_v, shank_link_RH_I);
    

    base_f = vxIv(base_v, base_I);

    secondPass_fullyActuated(jForces);

    baseWrench = base_f;
}

void rcg::shvan1::dyn::InverseDynamics::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    Acceleration base_a = baseAccel -g;

    // First pass, link 'hip_link_LF'
    hip_link_LF_v = ((xm->fr_hip_link_LF_X_fr_base) * base_v);
    hip_link_LF_v(iit::rbd::AZ) += qd(HIP_JOINT_LF);
    
    motionCrossProductMx<Scalar>(hip_link_LF_v, vcross);
    
    hip_link_LF_a = (xm->fr_hip_link_LF_X_fr_base) * base_a + vcross.col(iit::rbd::AZ) * qd(HIP_JOINT_LF);
    hip_link_LF_a(iit::rbd::AZ) += qdd(HIP_JOINT_LF);
    
    hip_link_LF_f = hip_link_LF_I * hip_link_LF_a + vxIv(hip_link_LF_v, hip_link_LF_I) - fext[HIP_LINK_LF];
    
    // First pass, link 'thigh_link_LF'
    thigh_link_LF_v = ((xm->fr_thigh_link_LF_X_fr_hip_link_LF) * hip_link_LF_v);
    thigh_link_LF_v(iit::rbd::AZ) += qd(ELBOW_JOINT_LF);
    
    motionCrossProductMx<Scalar>(thigh_link_LF_v, vcross);
    
    thigh_link_LF_a = (xm->fr_thigh_link_LF_X_fr_hip_link_LF) * hip_link_LF_a + vcross.col(iit::rbd::AZ) * qd(ELBOW_JOINT_LF);
    thigh_link_LF_a(iit::rbd::AZ) += qdd(ELBOW_JOINT_LF);
    
    thigh_link_LF_f = thigh_link_LF_I * thigh_link_LF_a + vxIv(thigh_link_LF_v, thigh_link_LF_I) - fext[THIGH_LINK_LF];
    
    // First pass, link 'shank_link_LF'
    shank_link_LF_v = ((xm->fr_shank_link_LF_X_fr_thigh_link_LF) * thigh_link_LF_v);
    shank_link_LF_v(iit::rbd::AZ) += qd(KNEE_JOINT_LF);
    
    motionCrossProductMx<Scalar>(shank_link_LF_v, vcross);
    
    shank_link_LF_a = (xm->fr_shank_link_LF_X_fr_thigh_link_LF) * thigh_link_LF_a + vcross.col(iit::rbd::AZ) * qd(KNEE_JOINT_LF);
    shank_link_LF_a(iit::rbd::AZ) += qdd(KNEE_JOINT_LF);
    
    shank_link_LF_f = shank_link_LF_I * shank_link_LF_a + vxIv(shank_link_LF_v, shank_link_LF_I) - fext[SHANK_LINK_LF];
    
    // First pass, link 'hip_link_RF'
    hip_link_RF_v = ((xm->fr_hip_link_RF_X_fr_base) * base_v);
    hip_link_RF_v(iit::rbd::AZ) += qd(HIP_JOINT_RF);
    
    motionCrossProductMx<Scalar>(hip_link_RF_v, vcross);
    
    hip_link_RF_a = (xm->fr_hip_link_RF_X_fr_base) * base_a + vcross.col(iit::rbd::AZ) * qd(HIP_JOINT_RF);
    hip_link_RF_a(iit::rbd::AZ) += qdd(HIP_JOINT_RF);
    
    hip_link_RF_f = hip_link_RF_I * hip_link_RF_a + vxIv(hip_link_RF_v, hip_link_RF_I) - fext[HIP_LINK_RF];
    
    // First pass, link 'thigh_link_RF'
    thigh_link_RF_v = ((xm->fr_thigh_link_RF_X_fr_hip_link_RF) * hip_link_RF_v);
    thigh_link_RF_v(iit::rbd::AZ) += qd(ELBOW_JOINT_RF);
    
    motionCrossProductMx<Scalar>(thigh_link_RF_v, vcross);
    
    thigh_link_RF_a = (xm->fr_thigh_link_RF_X_fr_hip_link_RF) * hip_link_RF_a + vcross.col(iit::rbd::AZ) * qd(ELBOW_JOINT_RF);
    thigh_link_RF_a(iit::rbd::AZ) += qdd(ELBOW_JOINT_RF);
    
    thigh_link_RF_f = thigh_link_RF_I * thigh_link_RF_a + vxIv(thigh_link_RF_v, thigh_link_RF_I) - fext[THIGH_LINK_RF];
    
    // First pass, link 'shank_link_RF'
    shank_link_RF_v = ((xm->fr_shank_link_RF_X_fr_thigh_link_RF) * thigh_link_RF_v);
    shank_link_RF_v(iit::rbd::AZ) += qd(KNEE_JOINT_RF);
    
    motionCrossProductMx<Scalar>(shank_link_RF_v, vcross);
    
    shank_link_RF_a = (xm->fr_shank_link_RF_X_fr_thigh_link_RF) * thigh_link_RF_a + vcross.col(iit::rbd::AZ) * qd(KNEE_JOINT_RF);
    shank_link_RF_a(iit::rbd::AZ) += qdd(KNEE_JOINT_RF);
    
    shank_link_RF_f = shank_link_RF_I * shank_link_RF_a + vxIv(shank_link_RF_v, shank_link_RF_I) - fext[SHANK_LINK_RF];
    
    // First pass, link 'hip_link_LH'
    hip_link_LH_v = ((xm->fr_hip_link_LH_X_fr_base) * base_v);
    hip_link_LH_v(iit::rbd::AZ) += qd(HIP_JOINT_LH);
    
    motionCrossProductMx<Scalar>(hip_link_LH_v, vcross);
    
    hip_link_LH_a = (xm->fr_hip_link_LH_X_fr_base) * base_a + vcross.col(iit::rbd::AZ) * qd(HIP_JOINT_LH);
    hip_link_LH_a(iit::rbd::AZ) += qdd(HIP_JOINT_LH);
    
    hip_link_LH_f = hip_link_LH_I * hip_link_LH_a + vxIv(hip_link_LH_v, hip_link_LH_I) - fext[HIP_LINK_LH];
    
    // First pass, link 'thigh_link_LH'
    thigh_link_LH_v = ((xm->fr_thigh_link_LH_X_fr_hip_link_LH) * hip_link_LH_v);
    thigh_link_LH_v(iit::rbd::AZ) += qd(ELBOW_JOINT_LH);
    
    motionCrossProductMx<Scalar>(thigh_link_LH_v, vcross);
    
    thigh_link_LH_a = (xm->fr_thigh_link_LH_X_fr_hip_link_LH) * hip_link_LH_a + vcross.col(iit::rbd::AZ) * qd(ELBOW_JOINT_LH);
    thigh_link_LH_a(iit::rbd::AZ) += qdd(ELBOW_JOINT_LH);
    
    thigh_link_LH_f = thigh_link_LH_I * thigh_link_LH_a + vxIv(thigh_link_LH_v, thigh_link_LH_I) - fext[THIGH_LINK_LH];
    
    // First pass, link 'shank_link_LH'
    shank_link_LH_v = ((xm->fr_shank_link_LH_X_fr_thigh_link_LH) * thigh_link_LH_v);
    shank_link_LH_v(iit::rbd::AZ) += qd(KNEE_JOINT_LH);
    
    motionCrossProductMx<Scalar>(shank_link_LH_v, vcross);
    
    shank_link_LH_a = (xm->fr_shank_link_LH_X_fr_thigh_link_LH) * thigh_link_LH_a + vcross.col(iit::rbd::AZ) * qd(KNEE_JOINT_LH);
    shank_link_LH_a(iit::rbd::AZ) += qdd(KNEE_JOINT_LH);
    
    shank_link_LH_f = shank_link_LH_I * shank_link_LH_a + vxIv(shank_link_LH_v, shank_link_LH_I) - fext[SHANK_LINK_LH];
    
    // First pass, link 'hip_link_RH'
    hip_link_RH_v = ((xm->fr_hip_link_RH_X_fr_base) * base_v);
    hip_link_RH_v(iit::rbd::AZ) += qd(HIP_JOINT_RH);
    
    motionCrossProductMx<Scalar>(hip_link_RH_v, vcross);
    
    hip_link_RH_a = (xm->fr_hip_link_RH_X_fr_base) * base_a + vcross.col(iit::rbd::AZ) * qd(HIP_JOINT_RH);
    hip_link_RH_a(iit::rbd::AZ) += qdd(HIP_JOINT_RH);
    
    hip_link_RH_f = hip_link_RH_I * hip_link_RH_a + vxIv(hip_link_RH_v, hip_link_RH_I) - fext[HIP_LINK_RH];
    
    // First pass, link 'thigh_link_RH'
    thigh_link_RH_v = ((xm->fr_thigh_link_RH_X_fr_hip_link_RH) * hip_link_RH_v);
    thigh_link_RH_v(iit::rbd::AZ) += qd(ELBOW_JOINT_RH);
    
    motionCrossProductMx<Scalar>(thigh_link_RH_v, vcross);
    
    thigh_link_RH_a = (xm->fr_thigh_link_RH_X_fr_hip_link_RH) * hip_link_RH_a + vcross.col(iit::rbd::AZ) * qd(ELBOW_JOINT_RH);
    thigh_link_RH_a(iit::rbd::AZ) += qdd(ELBOW_JOINT_RH);
    
    thigh_link_RH_f = thigh_link_RH_I * thigh_link_RH_a + vxIv(thigh_link_RH_v, thigh_link_RH_I) - fext[THIGH_LINK_RH];
    
    // First pass, link 'shank_link_RH'
    shank_link_RH_v = ((xm->fr_shank_link_RH_X_fr_thigh_link_RH) * thigh_link_RH_v);
    shank_link_RH_v(iit::rbd::AZ) += qd(KNEE_JOINT_RH);
    
    motionCrossProductMx<Scalar>(shank_link_RH_v, vcross);
    
    shank_link_RH_a = (xm->fr_shank_link_RH_X_fr_thigh_link_RH) * thigh_link_RH_a + vcross.col(iit::rbd::AZ) * qd(KNEE_JOINT_RH);
    shank_link_RH_a(iit::rbd::AZ) += qdd(KNEE_JOINT_RH);
    
    shank_link_RH_f = shank_link_RH_I * shank_link_RH_a + vxIv(shank_link_RH_v, shank_link_RH_I) - fext[SHANK_LINK_RH];
    

    // The base
    base_f = base_I * base_a + vxIv(base_v, base_I) - fext[BASE];

    secondPass_fullyActuated(jForces);

    baseWrench = base_f;
}


void rcg::shvan1::dyn::InverseDynamics::secondPass_fullyActuated(JointState& jForces)
{
    // Link 'shank_link_RH'
    jForces(KNEE_JOINT_RH) = shank_link_RH_f(iit::rbd::AZ);
    thigh_link_RH_f += xm->fr_shank_link_RH_X_fr_thigh_link_RH.transpose() * shank_link_RH_f;
    // Link 'thigh_link_RH'
    jForces(ELBOW_JOINT_RH) = thigh_link_RH_f(iit::rbd::AZ);
    hip_link_RH_f += xm->fr_thigh_link_RH_X_fr_hip_link_RH.transpose() * thigh_link_RH_f;
    // Link 'hip_link_RH'
    jForces(HIP_JOINT_RH) = hip_link_RH_f(iit::rbd::AZ);
    base_f += xm->fr_hip_link_RH_X_fr_base.transpose() * hip_link_RH_f;
    // Link 'shank_link_LH'
    jForces(KNEE_JOINT_LH) = shank_link_LH_f(iit::rbd::AZ);
    thigh_link_LH_f += xm->fr_shank_link_LH_X_fr_thigh_link_LH.transpose() * shank_link_LH_f;
    // Link 'thigh_link_LH'
    jForces(ELBOW_JOINT_LH) = thigh_link_LH_f(iit::rbd::AZ);
    hip_link_LH_f += xm->fr_thigh_link_LH_X_fr_hip_link_LH.transpose() * thigh_link_LH_f;
    // Link 'hip_link_LH'
    jForces(HIP_JOINT_LH) = hip_link_LH_f(iit::rbd::AZ);
    base_f += xm->fr_hip_link_LH_X_fr_base.transpose() * hip_link_LH_f;
    // Link 'shank_link_RF'
    jForces(KNEE_JOINT_RF) = shank_link_RF_f(iit::rbd::AZ);
    thigh_link_RF_f += xm->fr_shank_link_RF_X_fr_thigh_link_RF.transpose() * shank_link_RF_f;
    // Link 'thigh_link_RF'
    jForces(ELBOW_JOINT_RF) = thigh_link_RF_f(iit::rbd::AZ);
    hip_link_RF_f += xm->fr_thigh_link_RF_X_fr_hip_link_RF.transpose() * thigh_link_RF_f;
    // Link 'hip_link_RF'
    jForces(HIP_JOINT_RF) = hip_link_RF_f(iit::rbd::AZ);
    base_f += xm->fr_hip_link_RF_X_fr_base.transpose() * hip_link_RF_f;
    // Link 'shank_link_LF'
    jForces(KNEE_JOINT_LF) = shank_link_LF_f(iit::rbd::AZ);
    thigh_link_LF_f += xm->fr_shank_link_LF_X_fr_thigh_link_LF.transpose() * shank_link_LF_f;
    // Link 'thigh_link_LF'
    jForces(ELBOW_JOINT_LF) = thigh_link_LF_f(iit::rbd::AZ);
    hip_link_LF_f += xm->fr_thigh_link_LF_X_fr_hip_link_LF.transpose() * thigh_link_LF_f;
    // Link 'hip_link_LF'
    jForces(HIP_JOINT_LF) = hip_link_LF_f(iit::rbd::AZ);
    base_f += xm->fr_hip_link_LF_X_fr_base.transpose() * hip_link_LF_f;
}
