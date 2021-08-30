#include "jacobians.h"

rcg::shvan1::Jacobians::Jacobians()
:    fr_base_J_LF_FOOT(), 
    fr_base_J_RF_FOOT(), 
    fr_base_J_LH_FOOT(), 
    fr_base_J_RH_FOOT(), 
    imu_link_J_LF_FOOT(), 
    imu_link_J_RF_FOOT(), 
    imu_link_J_LH_FOOT(), 
    imu_link_J_RH_FOOT()
{}

void rcg::shvan1::Jacobians::updateParameters(const Params_lengths& _lengths, const Params_angles& _angles)
{
    params.lengths = _lengths;
    params.angles = _angles;
    params.trig.update();
}

rcg::shvan1::Jacobians::Type_fr_base_J_LF_FOOT::Type_fr_base_J_LF_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const rcg::shvan1::Jacobians::Type_fr_base_J_LF_FOOT& rcg::shvan1::Jacobians::Type_fr_base_J_LF_FOOT::update(const JointState& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    Scalar sin_q_knee_joint_LF  = ScalarTraits::sin( q(KNEE_JOINT_LF) );
    Scalar cos_q_knee_joint_LF  = ScalarTraits::cos( q(KNEE_JOINT_LF) );
    (*this)(1,1) = -cos_q_hip_joint_LF;
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(2,1) = -sin_q_hip_joint_LF;
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(3,1) = (- tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_knee_joint_LF)+( tx_knee_joint_LF * cos_q_elbow_joint_LF);
    (*this)(3,2) = ( tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_knee_joint_LF)-( tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_knee_joint_LF);
    (*this)(4,0) = (- tx_LF_FOOT * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF)+((( tx_knee_joint_LF * cos_q_elbow_joint_LF)+ tx_elbow_joint_LF) * cos_q_hip_joint_LF);
    (*this)(4,1) = (- tx_LF_FOOT * cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)-( tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)-( tx_knee_joint_LF * sin_q_elbow_joint_LF * sin_q_hip_joint_LF);
    (*this)(4,2) = (- tx_LF_FOOT * cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)-( tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(5,0) = (- tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * cos_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)+((( tx_knee_joint_LF * cos_q_elbow_joint_LF)+ tx_elbow_joint_LF) * sin_q_hip_joint_LF);
    (*this)(5,1) = ( tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF)+( tx_knee_joint_LF * sin_q_elbow_joint_LF * cos_q_hip_joint_LF);
    (*this)(5,2) = ( tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    return *this;
}

rcg::shvan1::Jacobians::Type_fr_base_J_RF_FOOT::Type_fr_base_J_RF_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const rcg::shvan1::Jacobians::Type_fr_base_J_RF_FOOT& rcg::shvan1::Jacobians::Type_fr_base_J_RF_FOOT::update(const JointState& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    Scalar sin_q_knee_joint_RF  = ScalarTraits::sin( q(KNEE_JOINT_RF) );
    Scalar cos_q_knee_joint_RF  = ScalarTraits::cos( q(KNEE_JOINT_RF) );
    (*this)(1,1) = -cos_q_hip_joint_RF;
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(2,1) = -sin_q_hip_joint_RF;
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(3,1) = (- tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_knee_joint_RF)+( tx_knee_joint_RF * cos_q_elbow_joint_RF);
    (*this)(3,2) = ( tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_knee_joint_RF)-( tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_knee_joint_RF);
    (*this)(4,0) = (- tx_RF_FOOT * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF)+((( tx_knee_joint_RF * cos_q_elbow_joint_RF)+ tx_elbow_joint_RF) * cos_q_hip_joint_RF);
    (*this)(4,1) = (- tx_RF_FOOT * cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)-( tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)-( tx_knee_joint_RF * sin_q_elbow_joint_RF * sin_q_hip_joint_RF);
    (*this)(4,2) = (- tx_RF_FOOT * cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)-( tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(5,0) = (- tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * cos_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)+((( tx_knee_joint_RF * cos_q_elbow_joint_RF)+ tx_elbow_joint_RF) * sin_q_hip_joint_RF);
    (*this)(5,1) = ( tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF)+( tx_knee_joint_RF * sin_q_elbow_joint_RF * cos_q_hip_joint_RF);
    (*this)(5,2) = ( tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    return *this;
}

rcg::shvan1::Jacobians::Type_fr_base_J_LH_FOOT::Type_fr_base_J_LH_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const rcg::shvan1::Jacobians::Type_fr_base_J_LH_FOOT& rcg::shvan1::Jacobians::Type_fr_base_J_LH_FOOT::update(const JointState& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    Scalar sin_q_knee_joint_LH  = ScalarTraits::sin( q(KNEE_JOINT_LH) );
    Scalar cos_q_knee_joint_LH  = ScalarTraits::cos( q(KNEE_JOINT_LH) );
    (*this)(1,1) = -cos_q_hip_joint_LH;
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(2,1) = -sin_q_hip_joint_LH;
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(3,1) = (- tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_knee_joint_LH)+( tx_knee_joint_LH * cos_q_elbow_joint_LH);
    (*this)(3,2) = ( tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_knee_joint_LH)-( tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_knee_joint_LH);
    (*this)(4,0) = (- tx_LH_FOOT * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH)+((( tx_knee_joint_LH * cos_q_elbow_joint_LH)+ tx_elbow_joint_LH) * cos_q_hip_joint_LH);
    (*this)(4,1) = (- tx_LH_FOOT * cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)-( tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)-( tx_knee_joint_LH * sin_q_elbow_joint_LH * sin_q_hip_joint_LH);
    (*this)(4,2) = (- tx_LH_FOOT * cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)-( tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(5,0) = (- tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * cos_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)+((( tx_knee_joint_LH * cos_q_elbow_joint_LH)+ tx_elbow_joint_LH) * sin_q_hip_joint_LH);
    (*this)(5,1) = ( tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH)+( tx_knee_joint_LH * sin_q_elbow_joint_LH * cos_q_hip_joint_LH);
    (*this)(5,2) = ( tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    return *this;
}

rcg::shvan1::Jacobians::Type_fr_base_J_RH_FOOT::Type_fr_base_J_RH_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const rcg::shvan1::Jacobians::Type_fr_base_J_RH_FOOT& rcg::shvan1::Jacobians::Type_fr_base_J_RH_FOOT::update(const JointState& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    Scalar sin_q_knee_joint_RH  = ScalarTraits::sin( q(KNEE_JOINT_RH) );
    Scalar cos_q_knee_joint_RH  = ScalarTraits::cos( q(KNEE_JOINT_RH) );
    (*this)(1,1) = -cos_q_hip_joint_RH;
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(2,1) = -sin_q_hip_joint_RH;
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(3,1) = (- tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_knee_joint_RH)+( tx_knee_joint_RH * cos_q_elbow_joint_RH);
    (*this)(3,2) = ( tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_knee_joint_RH)-( tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_knee_joint_RH);
    (*this)(4,0) = (- tx_RH_FOOT * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH)+((( tx_knee_joint_RH * cos_q_elbow_joint_RH)+ tx_elbow_joint_RH) * cos_q_hip_joint_RH);
    (*this)(4,1) = (- tx_RH_FOOT * cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)-( tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)-( tx_knee_joint_RH * sin_q_elbow_joint_RH * sin_q_hip_joint_RH);
    (*this)(4,2) = (- tx_RH_FOOT * cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)-( tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(5,0) = (- tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * cos_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)+((( tx_knee_joint_RH * cos_q_elbow_joint_RH)+ tx_elbow_joint_RH) * sin_q_hip_joint_RH);
    (*this)(5,1) = ( tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH)+( tx_knee_joint_RH * sin_q_elbow_joint_RH * cos_q_hip_joint_RH);
    (*this)(5,2) = ( tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    return *this;
}

rcg::shvan1::Jacobians::Type_imu_link_J_LF_FOOT::Type_imu_link_J_LF_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const rcg::shvan1::Jacobians::Type_imu_link_J_LF_FOOT& rcg::shvan1::Jacobians::Type_imu_link_J_LF_FOOT::update(const JointState& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    Scalar sin_q_knee_joint_LF  = ScalarTraits::sin( q(KNEE_JOINT_LF) );
    Scalar cos_q_knee_joint_LF  = ScalarTraits::cos( q(KNEE_JOINT_LF) );
    (*this)(1,1) = -cos_q_hip_joint_LF;
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(2,1) = -sin_q_hip_joint_LF;
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(3,1) = (- tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_knee_joint_LF)+( tx_knee_joint_LF * cos_q_elbow_joint_LF);
    (*this)(3,2) = ( tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_knee_joint_LF)-( tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_knee_joint_LF);
    (*this)(4,0) = (- tx_LF_FOOT * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF)+((( tx_knee_joint_LF * cos_q_elbow_joint_LF)+ tx_elbow_joint_LF) * cos_q_hip_joint_LF);
    (*this)(4,1) = (- tx_LF_FOOT * cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)-( tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)-( tx_knee_joint_LF * sin_q_elbow_joint_LF * sin_q_hip_joint_LF);
    (*this)(4,2) = (- tx_LF_FOOT * cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)-( tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(5,0) = (- tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * cos_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)+((( tx_knee_joint_LF * cos_q_elbow_joint_LF)+ tx_elbow_joint_LF) * sin_q_hip_joint_LF);
    (*this)(5,1) = ( tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF)+( tx_knee_joint_LF * sin_q_elbow_joint_LF * cos_q_hip_joint_LF);
    (*this)(5,2) = ( tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    return *this;
}

rcg::shvan1::Jacobians::Type_imu_link_J_RF_FOOT::Type_imu_link_J_RF_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const rcg::shvan1::Jacobians::Type_imu_link_J_RF_FOOT& rcg::shvan1::Jacobians::Type_imu_link_J_RF_FOOT::update(const JointState& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    Scalar sin_q_knee_joint_RF  = ScalarTraits::sin( q(KNEE_JOINT_RF) );
    Scalar cos_q_knee_joint_RF  = ScalarTraits::cos( q(KNEE_JOINT_RF) );
    (*this)(1,1) = -cos_q_hip_joint_RF;
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(2,1) = -sin_q_hip_joint_RF;
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(3,1) = (- tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_knee_joint_RF)+( tx_knee_joint_RF * cos_q_elbow_joint_RF);
    (*this)(3,2) = ( tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_knee_joint_RF)-( tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_knee_joint_RF);
    (*this)(4,0) = (- tx_RF_FOOT * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF)+((( tx_knee_joint_RF * cos_q_elbow_joint_RF)+ tx_elbow_joint_RF) * cos_q_hip_joint_RF);
    (*this)(4,1) = (- tx_RF_FOOT * cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)-( tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)-( tx_knee_joint_RF * sin_q_elbow_joint_RF * sin_q_hip_joint_RF);
    (*this)(4,2) = (- tx_RF_FOOT * cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)-( tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(5,0) = (- tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * cos_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)+((( tx_knee_joint_RF * cos_q_elbow_joint_RF)+ tx_elbow_joint_RF) * sin_q_hip_joint_RF);
    (*this)(5,1) = ( tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF)+( tx_knee_joint_RF * sin_q_elbow_joint_RF * cos_q_hip_joint_RF);
    (*this)(5,2) = ( tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    return *this;
}

rcg::shvan1::Jacobians::Type_imu_link_J_LH_FOOT::Type_imu_link_J_LH_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const rcg::shvan1::Jacobians::Type_imu_link_J_LH_FOOT& rcg::shvan1::Jacobians::Type_imu_link_J_LH_FOOT::update(const JointState& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    Scalar sin_q_knee_joint_LH  = ScalarTraits::sin( q(KNEE_JOINT_LH) );
    Scalar cos_q_knee_joint_LH  = ScalarTraits::cos( q(KNEE_JOINT_LH) );
    (*this)(1,1) = -cos_q_hip_joint_LH;
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(2,1) = -sin_q_hip_joint_LH;
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(3,1) = (- tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_knee_joint_LH)+( tx_knee_joint_LH * cos_q_elbow_joint_LH);
    (*this)(3,2) = ( tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_knee_joint_LH)-( tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_knee_joint_LH);
    (*this)(4,0) = (- tx_LH_FOOT * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH)+((( tx_knee_joint_LH * cos_q_elbow_joint_LH)+ tx_elbow_joint_LH) * cos_q_hip_joint_LH);
    (*this)(4,1) = (- tx_LH_FOOT * cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)-( tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)-( tx_knee_joint_LH * sin_q_elbow_joint_LH * sin_q_hip_joint_LH);
    (*this)(4,2) = (- tx_LH_FOOT * cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)-( tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(5,0) = (- tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * cos_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)+((( tx_knee_joint_LH * cos_q_elbow_joint_LH)+ tx_elbow_joint_LH) * sin_q_hip_joint_LH);
    (*this)(5,1) = ( tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH)+( tx_knee_joint_LH * sin_q_elbow_joint_LH * cos_q_hip_joint_LH);
    (*this)(5,2) = ( tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    return *this;
}

rcg::shvan1::Jacobians::Type_imu_link_J_RH_FOOT::Type_imu_link_J_RH_FOOT()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const rcg::shvan1::Jacobians::Type_imu_link_J_RH_FOOT& rcg::shvan1::Jacobians::Type_imu_link_J_RH_FOOT::update(const JointState& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    Scalar sin_q_knee_joint_RH  = ScalarTraits::sin( q(KNEE_JOINT_RH) );
    Scalar cos_q_knee_joint_RH  = ScalarTraits::cos( q(KNEE_JOINT_RH) );
    (*this)(1,1) = -cos_q_hip_joint_RH;
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(2,1) = -sin_q_hip_joint_RH;
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(3,1) = (- tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_knee_joint_RH)+( tx_knee_joint_RH * cos_q_elbow_joint_RH);
    (*this)(3,2) = ( tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_knee_joint_RH)-( tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_knee_joint_RH);
    (*this)(4,0) = (- tx_RH_FOOT * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH)+((( tx_knee_joint_RH * cos_q_elbow_joint_RH)+ tx_elbow_joint_RH) * cos_q_hip_joint_RH);
    (*this)(4,1) = (- tx_RH_FOOT * cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)-( tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)-( tx_knee_joint_RH * sin_q_elbow_joint_RH * sin_q_hip_joint_RH);
    (*this)(4,2) = (- tx_RH_FOOT * cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)-( tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(5,0) = (- tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * cos_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)+((( tx_knee_joint_RH * cos_q_elbow_joint_RH)+ tx_elbow_joint_RH) * sin_q_hip_joint_RH);
    (*this)(5,1) = ( tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH)+( tx_knee_joint_RH * sin_q_elbow_joint_RH * cos_q_hip_joint_RH);
    (*this)(5,2) = ( tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    return *this;
}

