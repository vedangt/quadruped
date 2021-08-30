#include "transforms.h"

using namespace rcg::shvan1;

// Constructors

MotionTransforms::MotionTransforms()
 :     fr_base_X_LF_FOOT(),
    fr_base_X_RF_FOOT(),
    fr_base_X_LH_FOOT(),
    fr_base_X_RH_FOOT(),
    imu_link_X_LF_FOOT(),
    imu_link_X_RF_FOOT(),
    imu_link_X_LH_FOOT(),
    imu_link_X_RH_FOOT(),
    fr_base_X_fr_hip_joint_LF(),
    fr_base_X_fr_elbow_joint_LF(),
    fr_base_X_fr_knee_joint_LF(),
    fr_base_X_fr_hip_joint_RF(),
    fr_base_X_fr_elbow_joint_RF(),
    fr_base_X_fr_knee_joint_RF(),
    fr_base_X_fr_hip_joint_LH(),
    fr_base_X_fr_elbow_joint_LH(),
    fr_base_X_fr_knee_joint_LH(),
    fr_base_X_fr_hip_joint_RH(),
    fr_base_X_fr_elbow_joint_RH(),
    fr_base_X_fr_knee_joint_RH(),
    imu_link_X_fr_hip_joint_LF(),
    imu_link_X_fr_elbow_joint_LF(),
    imu_link_X_fr_knee_joint_LF(),
    imu_link_X_fr_hip_joint_RF(),
    imu_link_X_fr_elbow_joint_RF(),
    imu_link_X_fr_knee_joint_RF(),
    imu_link_X_fr_hip_joint_LH(),
    imu_link_X_fr_elbow_joint_LH(),
    imu_link_X_fr_knee_joint_LH(),
    imu_link_X_fr_hip_joint_RH(),
    imu_link_X_fr_elbow_joint_RH(),
    imu_link_X_fr_knee_joint_RH(),
    fr_hip_link_LF_X_fr_base(),
    fr_base_X_fr_hip_link_LF(),
    fr_thigh_link_LF_X_fr_hip_link_LF(),
    fr_hip_link_LF_X_fr_thigh_link_LF(),
    fr_shank_link_LF_X_fr_thigh_link_LF(),
    fr_thigh_link_LF_X_fr_shank_link_LF(),
    fr_hip_link_RF_X_fr_base(),
    fr_base_X_fr_hip_link_RF(),
    fr_thigh_link_RF_X_fr_hip_link_RF(),
    fr_hip_link_RF_X_fr_thigh_link_RF(),
    fr_shank_link_RF_X_fr_thigh_link_RF(),
    fr_thigh_link_RF_X_fr_shank_link_RF(),
    fr_hip_link_LH_X_fr_base(),
    fr_base_X_fr_hip_link_LH(),
    fr_thigh_link_LH_X_fr_hip_link_LH(),
    fr_hip_link_LH_X_fr_thigh_link_LH(),
    fr_shank_link_LH_X_fr_thigh_link_LH(),
    fr_thigh_link_LH_X_fr_shank_link_LH(),
    fr_hip_link_RH_X_fr_base(),
    fr_base_X_fr_hip_link_RH(),
    fr_thigh_link_RH_X_fr_hip_link_RH(),
    fr_hip_link_RH_X_fr_thigh_link_RH(),
    fr_shank_link_RH_X_fr_thigh_link_RH(),
    fr_thigh_link_RH_X_fr_shank_link_RH()
{}
void MotionTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

ForceTransforms::ForceTransforms()
 :     fr_base_X_LF_FOOT(),
    fr_base_X_RF_FOOT(),
    fr_base_X_LH_FOOT(),
    fr_base_X_RH_FOOT(),
    imu_link_X_LF_FOOT(),
    imu_link_X_RF_FOOT(),
    imu_link_X_LH_FOOT(),
    imu_link_X_RH_FOOT(),
    fr_base_X_fr_hip_joint_LF(),
    fr_base_X_fr_elbow_joint_LF(),
    fr_base_X_fr_knee_joint_LF(),
    fr_base_X_fr_hip_joint_RF(),
    fr_base_X_fr_elbow_joint_RF(),
    fr_base_X_fr_knee_joint_RF(),
    fr_base_X_fr_hip_joint_LH(),
    fr_base_X_fr_elbow_joint_LH(),
    fr_base_X_fr_knee_joint_LH(),
    fr_base_X_fr_hip_joint_RH(),
    fr_base_X_fr_elbow_joint_RH(),
    fr_base_X_fr_knee_joint_RH(),
    imu_link_X_fr_hip_joint_LF(),
    imu_link_X_fr_elbow_joint_LF(),
    imu_link_X_fr_knee_joint_LF(),
    imu_link_X_fr_hip_joint_RF(),
    imu_link_X_fr_elbow_joint_RF(),
    imu_link_X_fr_knee_joint_RF(),
    imu_link_X_fr_hip_joint_LH(),
    imu_link_X_fr_elbow_joint_LH(),
    imu_link_X_fr_knee_joint_LH(),
    imu_link_X_fr_hip_joint_RH(),
    imu_link_X_fr_elbow_joint_RH(),
    imu_link_X_fr_knee_joint_RH(),
    fr_hip_link_LF_X_fr_base(),
    fr_base_X_fr_hip_link_LF(),
    fr_thigh_link_LF_X_fr_hip_link_LF(),
    fr_hip_link_LF_X_fr_thigh_link_LF(),
    fr_shank_link_LF_X_fr_thigh_link_LF(),
    fr_thigh_link_LF_X_fr_shank_link_LF(),
    fr_hip_link_RF_X_fr_base(),
    fr_base_X_fr_hip_link_RF(),
    fr_thigh_link_RF_X_fr_hip_link_RF(),
    fr_hip_link_RF_X_fr_thigh_link_RF(),
    fr_shank_link_RF_X_fr_thigh_link_RF(),
    fr_thigh_link_RF_X_fr_shank_link_RF(),
    fr_hip_link_LH_X_fr_base(),
    fr_base_X_fr_hip_link_LH(),
    fr_thigh_link_LH_X_fr_hip_link_LH(),
    fr_hip_link_LH_X_fr_thigh_link_LH(),
    fr_shank_link_LH_X_fr_thigh_link_LH(),
    fr_thigh_link_LH_X_fr_shank_link_LH(),
    fr_hip_link_RH_X_fr_base(),
    fr_base_X_fr_hip_link_RH(),
    fr_thigh_link_RH_X_fr_hip_link_RH(),
    fr_hip_link_RH_X_fr_thigh_link_RH(),
    fr_shank_link_RH_X_fr_thigh_link_RH(),
    fr_thigh_link_RH_X_fr_shank_link_RH()
{}
void ForceTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

HomogeneousTransforms::HomogeneousTransforms()
 :     fr_base_X_LF_FOOT(),
    fr_base_X_RF_FOOT(),
    fr_base_X_LH_FOOT(),
    fr_base_X_RH_FOOT(),
    imu_link_X_LF_FOOT(),
    imu_link_X_RF_FOOT(),
    imu_link_X_LH_FOOT(),
    imu_link_X_RH_FOOT(),
    fr_base_X_fr_hip_joint_LF(),
    fr_base_X_fr_elbow_joint_LF(),
    fr_base_X_fr_knee_joint_LF(),
    fr_base_X_fr_hip_joint_RF(),
    fr_base_X_fr_elbow_joint_RF(),
    fr_base_X_fr_knee_joint_RF(),
    fr_base_X_fr_hip_joint_LH(),
    fr_base_X_fr_elbow_joint_LH(),
    fr_base_X_fr_knee_joint_LH(),
    fr_base_X_fr_hip_joint_RH(),
    fr_base_X_fr_elbow_joint_RH(),
    fr_base_X_fr_knee_joint_RH(),
    imu_link_X_fr_hip_joint_LF(),
    imu_link_X_fr_elbow_joint_LF(),
    imu_link_X_fr_knee_joint_LF(),
    imu_link_X_fr_hip_joint_RF(),
    imu_link_X_fr_elbow_joint_RF(),
    imu_link_X_fr_knee_joint_RF(),
    imu_link_X_fr_hip_joint_LH(),
    imu_link_X_fr_elbow_joint_LH(),
    imu_link_X_fr_knee_joint_LH(),
    imu_link_X_fr_hip_joint_RH(),
    imu_link_X_fr_elbow_joint_RH(),
    imu_link_X_fr_knee_joint_RH(),
    fr_hip_link_LF_X_fr_base(),
    fr_base_X_fr_hip_link_LF(),
    fr_thigh_link_LF_X_fr_hip_link_LF(),
    fr_hip_link_LF_X_fr_thigh_link_LF(),
    fr_shank_link_LF_X_fr_thigh_link_LF(),
    fr_thigh_link_LF_X_fr_shank_link_LF(),
    fr_hip_link_RF_X_fr_base(),
    fr_base_X_fr_hip_link_RF(),
    fr_thigh_link_RF_X_fr_hip_link_RF(),
    fr_hip_link_RF_X_fr_thigh_link_RF(),
    fr_shank_link_RF_X_fr_thigh_link_RF(),
    fr_thigh_link_RF_X_fr_shank_link_RF(),
    fr_hip_link_LH_X_fr_base(),
    fr_base_X_fr_hip_link_LH(),
    fr_thigh_link_LH_X_fr_hip_link_LH(),
    fr_hip_link_LH_X_fr_thigh_link_LH(),
    fr_shank_link_LH_X_fr_thigh_link_LH(),
    fr_thigh_link_LH_X_fr_shank_link_LH(),
    fr_hip_link_RH_X_fr_base(),
    fr_base_X_fr_hip_link_RH(),
    fr_thigh_link_RH_X_fr_hip_link_RH(),
    fr_hip_link_RH_X_fr_thigh_link_RH(),
    fr_shank_link_RH_X_fr_thigh_link_RH(),
    fr_thigh_link_RH_X_fr_shank_link_RH()
{}
void HomogeneousTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

MotionTransforms::Type_fr_base_X_LF_FOOT::Type_fr_base_X_LF_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_LF_FOOT& MotionTransforms::Type_fr_base_X_LF_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    Scalar sin_q_knee_joint_LF  = ScalarTraits::sin( q(KNEE_JOINT_LF) );
    Scalar cos_q_knee_joint_LF  = ScalarTraits::cos( q(KNEE_JOINT_LF) );
    (*this)(0,0) = (cos_q_elbow_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_knee_joint_LF);
    (*this)(0,1) = (cos_q_elbow_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_knee_joint_LF);
    (*this)(1,0) = (cos_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF);
    (*this)(1,1) = (-cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(2,0) = (sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-(cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(2,1) = (cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(3,0) = ( ty_hip_joint_LF * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-( ty_hip_joint_LF * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(3,1) = ( ty_hip_joint_LF * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+( ty_hip_joint_LF * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(3,2) = ( tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_knee_joint_LF)-( tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_knee_joint_LF)-( ty_hip_joint_LF * sin_q_hip_joint_LF)-( tx_knee_joint_LF * cos_q_elbow_joint_LF)- tx_elbow_joint_LF;
    (*this)(3,3) = (cos_q_elbow_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_knee_joint_LF);
    (*this)(3,4) = (cos_q_elbow_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_knee_joint_LF);
    (*this)(4,0) = (((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+((( tx_hip_joint_LF * cos_q_elbow_joint_LF)-( tx_elbow_joint_LF * sin_q_elbow_joint_LF)) * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(4,1) = ((( tx_elbow_joint_LF * sin_q_elbow_joint_LF)-( tx_hip_joint_LF * cos_q_elbow_joint_LF)) * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+(((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * cos_q_hip_joint_LF * cos_q_knee_joint_LF)-( tx_LF_FOOT * cos_q_hip_joint_LF);
    (*this)(4,2) = ( tx_LF_FOOT * cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)+((( tx_knee_joint_LF * sin_q_elbow_joint_LF)+ tx_hip_joint_LF) * sin_q_hip_joint_LF);
    (*this)(4,3) = (cos_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF);
    (*this)(4,4) = (-cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(4,5) = -cos_q_hip_joint_LF;
    (*this)(5,0) = (((((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * cos_q_elbow_joint_LF)) * sin_q_knee_joint_LF)+((((( tx_hip_joint_LF * cos_q_elbow_joint_LF)-( tx_elbow_joint_LF * sin_q_elbow_joint_LF)) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * sin_q_elbow_joint_LF)) * cos_q_knee_joint_LF);
    (*this)(5,1) = ((((( tx_elbow_joint_LF * sin_q_elbow_joint_LF)-( tx_hip_joint_LF * cos_q_elbow_joint_LF)) * sin_q_hip_joint_LF)+( ty_hip_joint_LF * sin_q_elbow_joint_LF)) * sin_q_knee_joint_LF)+(((((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * cos_q_elbow_joint_LF)) * cos_q_knee_joint_LF)-( tx_LF_FOOT * sin_q_hip_joint_LF);
    (*this)(5,2) = (- tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-( tx_LF_FOOT * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF)+(((- tx_knee_joint_LF * sin_q_elbow_joint_LF)- tx_hip_joint_LF) * cos_q_hip_joint_LF);
    (*this)(5,3) = (sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-(cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(5,4) = (cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(5,5) = -sin_q_hip_joint_LF;
    return *this;
}
MotionTransforms::Type_fr_base_X_RF_FOOT::Type_fr_base_X_RF_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_RF_FOOT& MotionTransforms::Type_fr_base_X_RF_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    Scalar sin_q_knee_joint_RF  = ScalarTraits::sin( q(KNEE_JOINT_RF) );
    Scalar cos_q_knee_joint_RF  = ScalarTraits::cos( q(KNEE_JOINT_RF) );
    (*this)(0,0) = (cos_q_elbow_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_knee_joint_RF);
    (*this)(0,1) = (cos_q_elbow_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_knee_joint_RF);
    (*this)(1,0) = (cos_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF);
    (*this)(1,1) = (-cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(2,0) = (sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-(cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(2,1) = (cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(3,0) = ( ty_hip_joint_RF * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-( ty_hip_joint_RF * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(3,1) = ( ty_hip_joint_RF * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+( ty_hip_joint_RF * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(3,2) = ( tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_knee_joint_RF)-( tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_knee_joint_RF)-( ty_hip_joint_RF * sin_q_hip_joint_RF)-( tx_knee_joint_RF * cos_q_elbow_joint_RF)- tx_elbow_joint_RF;
    (*this)(3,3) = (cos_q_elbow_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_knee_joint_RF);
    (*this)(3,4) = (cos_q_elbow_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_knee_joint_RF);
    (*this)(4,0) = (((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+((( tx_hip_joint_RF * cos_q_elbow_joint_RF)-( tx_elbow_joint_RF * sin_q_elbow_joint_RF)) * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(4,1) = ((( tx_elbow_joint_RF * sin_q_elbow_joint_RF)-( tx_hip_joint_RF * cos_q_elbow_joint_RF)) * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+(((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * cos_q_hip_joint_RF * cos_q_knee_joint_RF)-( tx_RF_FOOT * cos_q_hip_joint_RF);
    (*this)(4,2) = ( tx_RF_FOOT * cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)+((( tx_knee_joint_RF * sin_q_elbow_joint_RF)+ tx_hip_joint_RF) * sin_q_hip_joint_RF);
    (*this)(4,3) = (cos_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF);
    (*this)(4,4) = (-cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(4,5) = -cos_q_hip_joint_RF;
    (*this)(5,0) = (((((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * cos_q_elbow_joint_RF)) * sin_q_knee_joint_RF)+((((( tx_hip_joint_RF * cos_q_elbow_joint_RF)-( tx_elbow_joint_RF * sin_q_elbow_joint_RF)) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * sin_q_elbow_joint_RF)) * cos_q_knee_joint_RF);
    (*this)(5,1) = ((((( tx_elbow_joint_RF * sin_q_elbow_joint_RF)-( tx_hip_joint_RF * cos_q_elbow_joint_RF)) * sin_q_hip_joint_RF)+( ty_hip_joint_RF * sin_q_elbow_joint_RF)) * sin_q_knee_joint_RF)+(((((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * cos_q_elbow_joint_RF)) * cos_q_knee_joint_RF)-( tx_RF_FOOT * sin_q_hip_joint_RF);
    (*this)(5,2) = (- tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-( tx_RF_FOOT * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF)+(((- tx_knee_joint_RF * sin_q_elbow_joint_RF)- tx_hip_joint_RF) * cos_q_hip_joint_RF);
    (*this)(5,3) = (sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-(cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(5,4) = (cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(5,5) = -sin_q_hip_joint_RF;
    return *this;
}
MotionTransforms::Type_fr_base_X_LH_FOOT::Type_fr_base_X_LH_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_LH_FOOT& MotionTransforms::Type_fr_base_X_LH_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    Scalar sin_q_knee_joint_LH  = ScalarTraits::sin( q(KNEE_JOINT_LH) );
    Scalar cos_q_knee_joint_LH  = ScalarTraits::cos( q(KNEE_JOINT_LH) );
    (*this)(0,0) = (cos_q_elbow_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_knee_joint_LH);
    (*this)(0,1) = (cos_q_elbow_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_knee_joint_LH);
    (*this)(1,0) = (cos_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH);
    (*this)(1,1) = (-cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(2,0) = (sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-(cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(2,1) = (cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(3,0) = ( ty_hip_joint_LH * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-( ty_hip_joint_LH * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(3,1) = ( ty_hip_joint_LH * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+( ty_hip_joint_LH * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(3,2) = ( tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_knee_joint_LH)-( tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_knee_joint_LH)-( ty_hip_joint_LH * sin_q_hip_joint_LH)-( tx_knee_joint_LH * cos_q_elbow_joint_LH)- tx_elbow_joint_LH;
    (*this)(3,3) = (cos_q_elbow_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_knee_joint_LH);
    (*this)(3,4) = (cos_q_elbow_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_knee_joint_LH);
    (*this)(4,0) = (((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+((( tx_hip_joint_LH * cos_q_elbow_joint_LH)-( tx_elbow_joint_LH * sin_q_elbow_joint_LH)) * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(4,1) = ((( tx_elbow_joint_LH * sin_q_elbow_joint_LH)-( tx_hip_joint_LH * cos_q_elbow_joint_LH)) * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+(((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * cos_q_hip_joint_LH * cos_q_knee_joint_LH)-( tx_LH_FOOT * cos_q_hip_joint_LH);
    (*this)(4,2) = ( tx_LH_FOOT * cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)+((( tx_knee_joint_LH * sin_q_elbow_joint_LH)+ tx_hip_joint_LH) * sin_q_hip_joint_LH);
    (*this)(4,3) = (cos_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH);
    (*this)(4,4) = (-cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(4,5) = -cos_q_hip_joint_LH;
    (*this)(5,0) = (((((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * cos_q_elbow_joint_LH)) * sin_q_knee_joint_LH)+((((( tx_hip_joint_LH * cos_q_elbow_joint_LH)-( tx_elbow_joint_LH * sin_q_elbow_joint_LH)) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * sin_q_elbow_joint_LH)) * cos_q_knee_joint_LH);
    (*this)(5,1) = ((((( tx_elbow_joint_LH * sin_q_elbow_joint_LH)-( tx_hip_joint_LH * cos_q_elbow_joint_LH)) * sin_q_hip_joint_LH)+( ty_hip_joint_LH * sin_q_elbow_joint_LH)) * sin_q_knee_joint_LH)+(((((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * cos_q_elbow_joint_LH)) * cos_q_knee_joint_LH)-( tx_LH_FOOT * sin_q_hip_joint_LH);
    (*this)(5,2) = (- tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-( tx_LH_FOOT * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH)+(((- tx_knee_joint_LH * sin_q_elbow_joint_LH)- tx_hip_joint_LH) * cos_q_hip_joint_LH);
    (*this)(5,3) = (sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-(cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(5,4) = (cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(5,5) = -sin_q_hip_joint_LH;
    return *this;
}
MotionTransforms::Type_fr_base_X_RH_FOOT::Type_fr_base_X_RH_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_RH_FOOT& MotionTransforms::Type_fr_base_X_RH_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    Scalar sin_q_knee_joint_RH  = ScalarTraits::sin( q(KNEE_JOINT_RH) );
    Scalar cos_q_knee_joint_RH  = ScalarTraits::cos( q(KNEE_JOINT_RH) );
    (*this)(0,0) = (cos_q_elbow_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_knee_joint_RH);
    (*this)(0,1) = (cos_q_elbow_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_knee_joint_RH);
    (*this)(1,0) = (cos_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH);
    (*this)(1,1) = (-cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(2,0) = (sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-(cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(2,1) = (cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(3,0) = ( ty_hip_joint_RH * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-( ty_hip_joint_RH * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(3,1) = ( ty_hip_joint_RH * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+( ty_hip_joint_RH * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(3,2) = ( tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_knee_joint_RH)-( tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_knee_joint_RH)-( ty_hip_joint_RH * sin_q_hip_joint_RH)-( tx_knee_joint_RH * cos_q_elbow_joint_RH)- tx_elbow_joint_RH;
    (*this)(3,3) = (cos_q_elbow_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_knee_joint_RH);
    (*this)(3,4) = (cos_q_elbow_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_knee_joint_RH);
    (*this)(4,0) = (((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+((( tx_hip_joint_RH * cos_q_elbow_joint_RH)-( tx_elbow_joint_RH * sin_q_elbow_joint_RH)) * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(4,1) = ((( tx_elbow_joint_RH * sin_q_elbow_joint_RH)-( tx_hip_joint_RH * cos_q_elbow_joint_RH)) * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+(((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * cos_q_hip_joint_RH * cos_q_knee_joint_RH)-( tx_RH_FOOT * cos_q_hip_joint_RH);
    (*this)(4,2) = ( tx_RH_FOOT * cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)+((( tx_knee_joint_RH * sin_q_elbow_joint_RH)+ tx_hip_joint_RH) * sin_q_hip_joint_RH);
    (*this)(4,3) = (cos_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH);
    (*this)(4,4) = (-cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(4,5) = -cos_q_hip_joint_RH;
    (*this)(5,0) = (((((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * cos_q_elbow_joint_RH)) * sin_q_knee_joint_RH)+((((( tx_hip_joint_RH * cos_q_elbow_joint_RH)-( tx_elbow_joint_RH * sin_q_elbow_joint_RH)) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * sin_q_elbow_joint_RH)) * cos_q_knee_joint_RH);
    (*this)(5,1) = ((((( tx_elbow_joint_RH * sin_q_elbow_joint_RH)-( tx_hip_joint_RH * cos_q_elbow_joint_RH)) * sin_q_hip_joint_RH)+( ty_hip_joint_RH * sin_q_elbow_joint_RH)) * sin_q_knee_joint_RH)+(((((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * cos_q_elbow_joint_RH)) * cos_q_knee_joint_RH)-( tx_RH_FOOT * sin_q_hip_joint_RH);
    (*this)(5,2) = (- tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-( tx_RH_FOOT * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH)+(((- tx_knee_joint_RH * sin_q_elbow_joint_RH)- tx_hip_joint_RH) * cos_q_hip_joint_RH);
    (*this)(5,3) = (sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-(cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(5,4) = (cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(5,5) = -sin_q_hip_joint_RH;
    return *this;
}
MotionTransforms::Type_imu_link_X_LF_FOOT::Type_imu_link_X_LF_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_imu_link_X_LF_FOOT& MotionTransforms::Type_imu_link_X_LF_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    Scalar sin_q_knee_joint_LF  = ScalarTraits::sin( q(KNEE_JOINT_LF) );
    Scalar cos_q_knee_joint_LF  = ScalarTraits::cos( q(KNEE_JOINT_LF) );
    (*this)(0,0) = (cos_q_elbow_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_knee_joint_LF);
    (*this)(0,1) = (cos_q_elbow_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_knee_joint_LF);
    (*this)(1,0) = (cos_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF);
    (*this)(1,1) = (-cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(2,0) = (sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-(cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(2,1) = (cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(3,0) = ( ty_hip_joint_LF * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-( ty_hip_joint_LF * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(3,1) = ( ty_hip_joint_LF * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+( ty_hip_joint_LF * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(3,2) = ( tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_knee_joint_LF)-( tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_knee_joint_LF)-( ty_hip_joint_LF * sin_q_hip_joint_LF)-( tx_knee_joint_LF * cos_q_elbow_joint_LF)- tx_elbow_joint_LF;
    (*this)(3,3) = (cos_q_elbow_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_knee_joint_LF);
    (*this)(3,4) = (cos_q_elbow_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_knee_joint_LF);
    (*this)(4,0) = (((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+((( tx_hip_joint_LF * cos_q_elbow_joint_LF)-( tx_elbow_joint_LF * sin_q_elbow_joint_LF)) * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(4,1) = ((( tx_elbow_joint_LF * sin_q_elbow_joint_LF)-( tx_hip_joint_LF * cos_q_elbow_joint_LF)) * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+(((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * cos_q_hip_joint_LF * cos_q_knee_joint_LF)-( tx_LF_FOOT * cos_q_hip_joint_LF);
    (*this)(4,2) = ( tx_LF_FOOT * cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)+((( tx_knee_joint_LF * sin_q_elbow_joint_LF)+ tx_hip_joint_LF) * sin_q_hip_joint_LF);
    (*this)(4,3) = (cos_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF);
    (*this)(4,4) = (-cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(4,5) = -cos_q_hip_joint_LF;
    (*this)(5,0) = (((((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * cos_q_elbow_joint_LF)) * sin_q_knee_joint_LF)+((((( tx_hip_joint_LF * cos_q_elbow_joint_LF)-( tx_elbow_joint_LF * sin_q_elbow_joint_LF)) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * sin_q_elbow_joint_LF)) * cos_q_knee_joint_LF);
    (*this)(5,1) = ((((( tx_elbow_joint_LF * sin_q_elbow_joint_LF)-( tx_hip_joint_LF * cos_q_elbow_joint_LF)) * sin_q_hip_joint_LF)+( ty_hip_joint_LF * sin_q_elbow_joint_LF)) * sin_q_knee_joint_LF)+(((((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * cos_q_elbow_joint_LF)) * cos_q_knee_joint_LF)-( tx_LF_FOOT * sin_q_hip_joint_LF);
    (*this)(5,2) = (- tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-( tx_LF_FOOT * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF)+(((- tx_knee_joint_LF * sin_q_elbow_joint_LF)- tx_hip_joint_LF) * cos_q_hip_joint_LF);
    (*this)(5,3) = (sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-(cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(5,4) = (cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(5,5) = -sin_q_hip_joint_LF;
    return *this;
}
MotionTransforms::Type_imu_link_X_RF_FOOT::Type_imu_link_X_RF_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_imu_link_X_RF_FOOT& MotionTransforms::Type_imu_link_X_RF_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    Scalar sin_q_knee_joint_RF  = ScalarTraits::sin( q(KNEE_JOINT_RF) );
    Scalar cos_q_knee_joint_RF  = ScalarTraits::cos( q(KNEE_JOINT_RF) );
    (*this)(0,0) = (cos_q_elbow_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_knee_joint_RF);
    (*this)(0,1) = (cos_q_elbow_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_knee_joint_RF);
    (*this)(1,0) = (cos_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF);
    (*this)(1,1) = (-cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(2,0) = (sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-(cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(2,1) = (cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(3,0) = ( ty_hip_joint_RF * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-( ty_hip_joint_RF * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(3,1) = ( ty_hip_joint_RF * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+( ty_hip_joint_RF * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(3,2) = ( tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_knee_joint_RF)-( tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_knee_joint_RF)-( ty_hip_joint_RF * sin_q_hip_joint_RF)-( tx_knee_joint_RF * cos_q_elbow_joint_RF)- tx_elbow_joint_RF;
    (*this)(3,3) = (cos_q_elbow_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_knee_joint_RF);
    (*this)(3,4) = (cos_q_elbow_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_knee_joint_RF);
    (*this)(4,0) = (((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+((( tx_hip_joint_RF * cos_q_elbow_joint_RF)-( tx_elbow_joint_RF * sin_q_elbow_joint_RF)) * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(4,1) = ((( tx_elbow_joint_RF * sin_q_elbow_joint_RF)-( tx_hip_joint_RF * cos_q_elbow_joint_RF)) * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+(((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * cos_q_hip_joint_RF * cos_q_knee_joint_RF)-( tx_RF_FOOT * cos_q_hip_joint_RF);
    (*this)(4,2) = ( tx_RF_FOOT * cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)+((( tx_knee_joint_RF * sin_q_elbow_joint_RF)+ tx_hip_joint_RF) * sin_q_hip_joint_RF);
    (*this)(4,3) = (cos_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF);
    (*this)(4,4) = (-cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(4,5) = -cos_q_hip_joint_RF;
    (*this)(5,0) = (((((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * cos_q_elbow_joint_RF)) * sin_q_knee_joint_RF)+((((( tx_hip_joint_RF * cos_q_elbow_joint_RF)-( tx_elbow_joint_RF * sin_q_elbow_joint_RF)) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * sin_q_elbow_joint_RF)) * cos_q_knee_joint_RF);
    (*this)(5,1) = ((((( tx_elbow_joint_RF * sin_q_elbow_joint_RF)-( tx_hip_joint_RF * cos_q_elbow_joint_RF)) * sin_q_hip_joint_RF)+( ty_hip_joint_RF * sin_q_elbow_joint_RF)) * sin_q_knee_joint_RF)+(((((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * cos_q_elbow_joint_RF)) * cos_q_knee_joint_RF)-( tx_RF_FOOT * sin_q_hip_joint_RF);
    (*this)(5,2) = (- tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-( tx_RF_FOOT * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF)+(((- tx_knee_joint_RF * sin_q_elbow_joint_RF)- tx_hip_joint_RF) * cos_q_hip_joint_RF);
    (*this)(5,3) = (sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-(cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(5,4) = (cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(5,5) = -sin_q_hip_joint_RF;
    return *this;
}
MotionTransforms::Type_imu_link_X_LH_FOOT::Type_imu_link_X_LH_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_imu_link_X_LH_FOOT& MotionTransforms::Type_imu_link_X_LH_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    Scalar sin_q_knee_joint_LH  = ScalarTraits::sin( q(KNEE_JOINT_LH) );
    Scalar cos_q_knee_joint_LH  = ScalarTraits::cos( q(KNEE_JOINT_LH) );
    (*this)(0,0) = (cos_q_elbow_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_knee_joint_LH);
    (*this)(0,1) = (cos_q_elbow_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_knee_joint_LH);
    (*this)(1,0) = (cos_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH);
    (*this)(1,1) = (-cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(2,0) = (sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-(cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(2,1) = (cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(3,0) = ( ty_hip_joint_LH * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-( ty_hip_joint_LH * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(3,1) = ( ty_hip_joint_LH * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+( ty_hip_joint_LH * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(3,2) = ( tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_knee_joint_LH)-( tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_knee_joint_LH)-( ty_hip_joint_LH * sin_q_hip_joint_LH)-( tx_knee_joint_LH * cos_q_elbow_joint_LH)- tx_elbow_joint_LH;
    (*this)(3,3) = (cos_q_elbow_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_knee_joint_LH);
    (*this)(3,4) = (cos_q_elbow_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_knee_joint_LH);
    (*this)(4,0) = (((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+((( tx_hip_joint_LH * cos_q_elbow_joint_LH)-( tx_elbow_joint_LH * sin_q_elbow_joint_LH)) * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(4,1) = ((( tx_elbow_joint_LH * sin_q_elbow_joint_LH)-( tx_hip_joint_LH * cos_q_elbow_joint_LH)) * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+(((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * cos_q_hip_joint_LH * cos_q_knee_joint_LH)-( tx_LH_FOOT * cos_q_hip_joint_LH);
    (*this)(4,2) = ( tx_LH_FOOT * cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)+((( tx_knee_joint_LH * sin_q_elbow_joint_LH)+ tx_hip_joint_LH) * sin_q_hip_joint_LH);
    (*this)(4,3) = (cos_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH);
    (*this)(4,4) = (-cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(4,5) = -cos_q_hip_joint_LH;
    (*this)(5,0) = (((((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * cos_q_elbow_joint_LH)) * sin_q_knee_joint_LH)+((((( tx_hip_joint_LH * cos_q_elbow_joint_LH)-( tx_elbow_joint_LH * sin_q_elbow_joint_LH)) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * sin_q_elbow_joint_LH)) * cos_q_knee_joint_LH);
    (*this)(5,1) = ((((( tx_elbow_joint_LH * sin_q_elbow_joint_LH)-( tx_hip_joint_LH * cos_q_elbow_joint_LH)) * sin_q_hip_joint_LH)+( ty_hip_joint_LH * sin_q_elbow_joint_LH)) * sin_q_knee_joint_LH)+(((((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * cos_q_elbow_joint_LH)) * cos_q_knee_joint_LH)-( tx_LH_FOOT * sin_q_hip_joint_LH);
    (*this)(5,2) = (- tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-( tx_LH_FOOT * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH)+(((- tx_knee_joint_LH * sin_q_elbow_joint_LH)- tx_hip_joint_LH) * cos_q_hip_joint_LH);
    (*this)(5,3) = (sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-(cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(5,4) = (cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(5,5) = -sin_q_hip_joint_LH;
    return *this;
}
MotionTransforms::Type_imu_link_X_RH_FOOT::Type_imu_link_X_RH_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_imu_link_X_RH_FOOT& MotionTransforms::Type_imu_link_X_RH_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    Scalar sin_q_knee_joint_RH  = ScalarTraits::sin( q(KNEE_JOINT_RH) );
    Scalar cos_q_knee_joint_RH  = ScalarTraits::cos( q(KNEE_JOINT_RH) );
    (*this)(0,0) = (cos_q_elbow_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_knee_joint_RH);
    (*this)(0,1) = (cos_q_elbow_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_knee_joint_RH);
    (*this)(1,0) = (cos_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH);
    (*this)(1,1) = (-cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(2,0) = (sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-(cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(2,1) = (cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(3,0) = ( ty_hip_joint_RH * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-( ty_hip_joint_RH * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(3,1) = ( ty_hip_joint_RH * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+( ty_hip_joint_RH * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(3,2) = ( tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_knee_joint_RH)-( tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_knee_joint_RH)-( ty_hip_joint_RH * sin_q_hip_joint_RH)-( tx_knee_joint_RH * cos_q_elbow_joint_RH)- tx_elbow_joint_RH;
    (*this)(3,3) = (cos_q_elbow_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_knee_joint_RH);
    (*this)(3,4) = (cos_q_elbow_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_knee_joint_RH);
    (*this)(4,0) = (((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+((( tx_hip_joint_RH * cos_q_elbow_joint_RH)-( tx_elbow_joint_RH * sin_q_elbow_joint_RH)) * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(4,1) = ((( tx_elbow_joint_RH * sin_q_elbow_joint_RH)-( tx_hip_joint_RH * cos_q_elbow_joint_RH)) * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+(((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * cos_q_hip_joint_RH * cos_q_knee_joint_RH)-( tx_RH_FOOT * cos_q_hip_joint_RH);
    (*this)(4,2) = ( tx_RH_FOOT * cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)+((( tx_knee_joint_RH * sin_q_elbow_joint_RH)+ tx_hip_joint_RH) * sin_q_hip_joint_RH);
    (*this)(4,3) = (cos_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH);
    (*this)(4,4) = (-cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(4,5) = -cos_q_hip_joint_RH;
    (*this)(5,0) = (((((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * cos_q_elbow_joint_RH)) * sin_q_knee_joint_RH)+((((( tx_hip_joint_RH * cos_q_elbow_joint_RH)-( tx_elbow_joint_RH * sin_q_elbow_joint_RH)) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * sin_q_elbow_joint_RH)) * cos_q_knee_joint_RH);
    (*this)(5,1) = ((((( tx_elbow_joint_RH * sin_q_elbow_joint_RH)-( tx_hip_joint_RH * cos_q_elbow_joint_RH)) * sin_q_hip_joint_RH)+( ty_hip_joint_RH * sin_q_elbow_joint_RH)) * sin_q_knee_joint_RH)+(((((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * cos_q_elbow_joint_RH)) * cos_q_knee_joint_RH)-( tx_RH_FOOT * sin_q_hip_joint_RH);
    (*this)(5,2) = (- tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-( tx_RH_FOOT * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH)+(((- tx_knee_joint_RH * sin_q_elbow_joint_RH)- tx_hip_joint_RH) * cos_q_hip_joint_RH);
    (*this)(5,3) = (sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-(cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(5,4) = (cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(5,5) = -sin_q_hip_joint_RH;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_hip_joint_LF::Type_fr_base_X_fr_hip_joint_LF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = - ty_hip_joint_LF;    // Maxima DSL: -_k__ty_hip_joint_LF
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) =  tx_hip_joint_LF;    // Maxima DSL: _k__tx_hip_joint_LF
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tx_hip_joint_LF;    // Maxima DSL: _k__tx_hip_joint_LF
    (*this)(5,2) = - ty_hip_joint_LF;    // Maxima DSL: -_k__ty_hip_joint_LF
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_hip_joint_LF& MotionTransforms::Type_fr_base_X_fr_hip_joint_LF::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_elbow_joint_LF::Type_fr_base_X_fr_elbow_joint_LF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_elbow_joint_LF& MotionTransforms::Type_fr_base_X_fr_elbow_joint_LF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    (*this)(1,0) = sin_q_hip_joint_LF;
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(2,0) = -cos_q_hip_joint_LF;
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(3,0) = - ty_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(3,2) = (- ty_hip_joint_LF * sin_q_hip_joint_LF)- tx_elbow_joint_LF;
    (*this)(4,0) =  tx_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(4,1) = - tx_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(4,2) =  tx_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(4,3) = sin_q_hip_joint_LF;
    (*this)(4,5) = -cos_q_hip_joint_LF;
    (*this)(5,0) =  tx_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(5,1) = (- tx_elbow_joint_LF * sin_q_hip_joint_LF)- ty_hip_joint_LF;
    (*this)(5,2) = - tx_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(5,3) = -cos_q_hip_joint_LF;
    (*this)(5,5) = -sin_q_hip_joint_LF;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_knee_joint_LF::Type_fr_base_X_fr_knee_joint_LF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_knee_joint_LF& MotionTransforms::Type_fr_base_X_fr_knee_joint_LF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    (*this)(0,0) = sin_q_elbow_joint_LF;
    (*this)(0,1) = cos_q_elbow_joint_LF;
    (*this)(1,0) = cos_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,1) = -sin_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(2,0) = -cos_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(2,1) = sin_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(3,0) = - ty_hip_joint_LF * cos_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(3,1) =  ty_hip_joint_LF * sin_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(3,2) = (- ty_hip_joint_LF * sin_q_hip_joint_LF)-( tx_knee_joint_LF * cos_q_elbow_joint_LF)- tx_elbow_joint_LF;
    (*this)(3,3) = sin_q_elbow_joint_LF;
    (*this)(3,4) = cos_q_elbow_joint_LF;
    (*this)(4,0) = (( tx_hip_joint_LF * cos_q_elbow_joint_LF)-( tx_elbow_joint_LF * sin_q_elbow_joint_LF)) * cos_q_hip_joint_LF;
    (*this)(4,1) = ((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * cos_q_hip_joint_LF;
    (*this)(4,2) = (( tx_knee_joint_LF * sin_q_elbow_joint_LF)+ tx_hip_joint_LF) * sin_q_hip_joint_LF;
    (*this)(4,3) = cos_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(4,4) = -sin_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(4,5) = -cos_q_hip_joint_LF;
    (*this)(5,0) = ((( tx_hip_joint_LF * cos_q_elbow_joint_LF)-( tx_elbow_joint_LF * sin_q_elbow_joint_LF)) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * sin_q_elbow_joint_LF);
    (*this)(5,1) = (((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * cos_q_elbow_joint_LF);
    (*this)(5,2) = ((- tx_knee_joint_LF * sin_q_elbow_joint_LF)- tx_hip_joint_LF) * cos_q_hip_joint_LF;
    (*this)(5,3) = -cos_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(5,4) = sin_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(5,5) = -sin_q_hip_joint_LF;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_hip_joint_RF::Type_fr_base_X_fr_hip_joint_RF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = - ty_hip_joint_RF;    // Maxima DSL: -_k__ty_hip_joint_RF
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) =  tx_hip_joint_RF;    // Maxima DSL: _k__tx_hip_joint_RF
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tx_hip_joint_RF;    // Maxima DSL: _k__tx_hip_joint_RF
    (*this)(5,2) = - ty_hip_joint_RF;    // Maxima DSL: -_k__ty_hip_joint_RF
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_hip_joint_RF& MotionTransforms::Type_fr_base_X_fr_hip_joint_RF::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_elbow_joint_RF::Type_fr_base_X_fr_elbow_joint_RF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_elbow_joint_RF& MotionTransforms::Type_fr_base_X_fr_elbow_joint_RF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    (*this)(1,0) = sin_q_hip_joint_RF;
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(2,0) = -cos_q_hip_joint_RF;
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(3,0) = - ty_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(3,2) = (- ty_hip_joint_RF * sin_q_hip_joint_RF)- tx_elbow_joint_RF;
    (*this)(4,0) =  tx_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(4,1) = - tx_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(4,2) =  tx_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(4,3) = sin_q_hip_joint_RF;
    (*this)(4,5) = -cos_q_hip_joint_RF;
    (*this)(5,0) =  tx_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(5,1) = (- tx_elbow_joint_RF * sin_q_hip_joint_RF)- ty_hip_joint_RF;
    (*this)(5,2) = - tx_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(5,3) = -cos_q_hip_joint_RF;
    (*this)(5,5) = -sin_q_hip_joint_RF;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_knee_joint_RF::Type_fr_base_X_fr_knee_joint_RF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_knee_joint_RF& MotionTransforms::Type_fr_base_X_fr_knee_joint_RF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    (*this)(0,0) = sin_q_elbow_joint_RF;
    (*this)(0,1) = cos_q_elbow_joint_RF;
    (*this)(1,0) = cos_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,1) = -sin_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(2,0) = -cos_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(2,1) = sin_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(3,0) = - ty_hip_joint_RF * cos_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(3,1) =  ty_hip_joint_RF * sin_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(3,2) = (- ty_hip_joint_RF * sin_q_hip_joint_RF)-( tx_knee_joint_RF * cos_q_elbow_joint_RF)- tx_elbow_joint_RF;
    (*this)(3,3) = sin_q_elbow_joint_RF;
    (*this)(3,4) = cos_q_elbow_joint_RF;
    (*this)(4,0) = (( tx_hip_joint_RF * cos_q_elbow_joint_RF)-( tx_elbow_joint_RF * sin_q_elbow_joint_RF)) * cos_q_hip_joint_RF;
    (*this)(4,1) = ((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * cos_q_hip_joint_RF;
    (*this)(4,2) = (( tx_knee_joint_RF * sin_q_elbow_joint_RF)+ tx_hip_joint_RF) * sin_q_hip_joint_RF;
    (*this)(4,3) = cos_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(4,4) = -sin_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(4,5) = -cos_q_hip_joint_RF;
    (*this)(5,0) = ((( tx_hip_joint_RF * cos_q_elbow_joint_RF)-( tx_elbow_joint_RF * sin_q_elbow_joint_RF)) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * sin_q_elbow_joint_RF);
    (*this)(5,1) = (((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * cos_q_elbow_joint_RF);
    (*this)(5,2) = ((- tx_knee_joint_RF * sin_q_elbow_joint_RF)- tx_hip_joint_RF) * cos_q_hip_joint_RF;
    (*this)(5,3) = -cos_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(5,4) = sin_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(5,5) = -sin_q_hip_joint_RF;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_hip_joint_LH::Type_fr_base_X_fr_hip_joint_LH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = - ty_hip_joint_LH;    // Maxima DSL: -_k__ty_hip_joint_LH
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) =  tx_hip_joint_LH;    // Maxima DSL: _k__tx_hip_joint_LH
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tx_hip_joint_LH;    // Maxima DSL: _k__tx_hip_joint_LH
    (*this)(5,2) = - ty_hip_joint_LH;    // Maxima DSL: -_k__ty_hip_joint_LH
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_hip_joint_LH& MotionTransforms::Type_fr_base_X_fr_hip_joint_LH::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_elbow_joint_LH::Type_fr_base_X_fr_elbow_joint_LH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_elbow_joint_LH& MotionTransforms::Type_fr_base_X_fr_elbow_joint_LH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    (*this)(1,0) = sin_q_hip_joint_LH;
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(2,0) = -cos_q_hip_joint_LH;
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(3,0) = - ty_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(3,2) = (- ty_hip_joint_LH * sin_q_hip_joint_LH)- tx_elbow_joint_LH;
    (*this)(4,0) =  tx_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(4,1) = - tx_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(4,2) =  tx_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(4,3) = sin_q_hip_joint_LH;
    (*this)(4,5) = -cos_q_hip_joint_LH;
    (*this)(5,0) =  tx_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(5,1) = (- tx_elbow_joint_LH * sin_q_hip_joint_LH)- ty_hip_joint_LH;
    (*this)(5,2) = - tx_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(5,3) = -cos_q_hip_joint_LH;
    (*this)(5,5) = -sin_q_hip_joint_LH;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_knee_joint_LH::Type_fr_base_X_fr_knee_joint_LH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_knee_joint_LH& MotionTransforms::Type_fr_base_X_fr_knee_joint_LH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    (*this)(0,0) = sin_q_elbow_joint_LH;
    (*this)(0,1) = cos_q_elbow_joint_LH;
    (*this)(1,0) = cos_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,1) = -sin_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(2,0) = -cos_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(2,1) = sin_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(3,0) = - ty_hip_joint_LH * cos_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(3,1) =  ty_hip_joint_LH * sin_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(3,2) = (- ty_hip_joint_LH * sin_q_hip_joint_LH)-( tx_knee_joint_LH * cos_q_elbow_joint_LH)- tx_elbow_joint_LH;
    (*this)(3,3) = sin_q_elbow_joint_LH;
    (*this)(3,4) = cos_q_elbow_joint_LH;
    (*this)(4,0) = (( tx_hip_joint_LH * cos_q_elbow_joint_LH)-( tx_elbow_joint_LH * sin_q_elbow_joint_LH)) * cos_q_hip_joint_LH;
    (*this)(4,1) = ((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * cos_q_hip_joint_LH;
    (*this)(4,2) = (( tx_knee_joint_LH * sin_q_elbow_joint_LH)+ tx_hip_joint_LH) * sin_q_hip_joint_LH;
    (*this)(4,3) = cos_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(4,4) = -sin_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(4,5) = -cos_q_hip_joint_LH;
    (*this)(5,0) = ((( tx_hip_joint_LH * cos_q_elbow_joint_LH)-( tx_elbow_joint_LH * sin_q_elbow_joint_LH)) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * sin_q_elbow_joint_LH);
    (*this)(5,1) = (((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * cos_q_elbow_joint_LH);
    (*this)(5,2) = ((- tx_knee_joint_LH * sin_q_elbow_joint_LH)- tx_hip_joint_LH) * cos_q_hip_joint_LH;
    (*this)(5,3) = -cos_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(5,4) = sin_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(5,5) = -sin_q_hip_joint_LH;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_hip_joint_RH::Type_fr_base_X_fr_hip_joint_RH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = - ty_hip_joint_RH;    // Maxima DSL: -_k__ty_hip_joint_RH
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) =  tx_hip_joint_RH;    // Maxima DSL: _k__tx_hip_joint_RH
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tx_hip_joint_RH;    // Maxima DSL: _k__tx_hip_joint_RH
    (*this)(5,2) = - ty_hip_joint_RH;    // Maxima DSL: -_k__ty_hip_joint_RH
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_hip_joint_RH& MotionTransforms::Type_fr_base_X_fr_hip_joint_RH::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_elbow_joint_RH::Type_fr_base_X_fr_elbow_joint_RH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_elbow_joint_RH& MotionTransforms::Type_fr_base_X_fr_elbow_joint_RH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    (*this)(1,0) = sin_q_hip_joint_RH;
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(2,0) = -cos_q_hip_joint_RH;
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(3,0) = - ty_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(3,2) = (- ty_hip_joint_RH * sin_q_hip_joint_RH)- tx_elbow_joint_RH;
    (*this)(4,0) =  tx_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(4,1) = - tx_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(4,2) =  tx_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(4,3) = sin_q_hip_joint_RH;
    (*this)(4,5) = -cos_q_hip_joint_RH;
    (*this)(5,0) =  tx_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(5,1) = (- tx_elbow_joint_RH * sin_q_hip_joint_RH)- ty_hip_joint_RH;
    (*this)(5,2) = - tx_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(5,3) = -cos_q_hip_joint_RH;
    (*this)(5,5) = -sin_q_hip_joint_RH;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_knee_joint_RH::Type_fr_base_X_fr_knee_joint_RH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_knee_joint_RH& MotionTransforms::Type_fr_base_X_fr_knee_joint_RH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    (*this)(0,0) = sin_q_elbow_joint_RH;
    (*this)(0,1) = cos_q_elbow_joint_RH;
    (*this)(1,0) = cos_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,1) = -sin_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(2,0) = -cos_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(2,1) = sin_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(3,0) = - ty_hip_joint_RH * cos_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(3,1) =  ty_hip_joint_RH * sin_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(3,2) = (- ty_hip_joint_RH * sin_q_hip_joint_RH)-( tx_knee_joint_RH * cos_q_elbow_joint_RH)- tx_elbow_joint_RH;
    (*this)(3,3) = sin_q_elbow_joint_RH;
    (*this)(3,4) = cos_q_elbow_joint_RH;
    (*this)(4,0) = (( tx_hip_joint_RH * cos_q_elbow_joint_RH)-( tx_elbow_joint_RH * sin_q_elbow_joint_RH)) * cos_q_hip_joint_RH;
    (*this)(4,1) = ((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * cos_q_hip_joint_RH;
    (*this)(4,2) = (( tx_knee_joint_RH * sin_q_elbow_joint_RH)+ tx_hip_joint_RH) * sin_q_hip_joint_RH;
    (*this)(4,3) = cos_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(4,4) = -sin_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(4,5) = -cos_q_hip_joint_RH;
    (*this)(5,0) = ((( tx_hip_joint_RH * cos_q_elbow_joint_RH)-( tx_elbow_joint_RH * sin_q_elbow_joint_RH)) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * sin_q_elbow_joint_RH);
    (*this)(5,1) = (((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * cos_q_elbow_joint_RH);
    (*this)(5,2) = ((- tx_knee_joint_RH * sin_q_elbow_joint_RH)- tx_hip_joint_RH) * cos_q_hip_joint_RH;
    (*this)(5,3) = -cos_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(5,4) = sin_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(5,5) = -sin_q_hip_joint_RH;
    return *this;
}
MotionTransforms::Type_imu_link_X_fr_hip_joint_LF::Type_imu_link_X_fr_hip_joint_LF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = - ty_hip_joint_LF;    // Maxima DSL: -_k__ty_hip_joint_LF
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) =  tx_hip_joint_LF;    // Maxima DSL: _k__tx_hip_joint_LF
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tx_hip_joint_LF;    // Maxima DSL: _k__tx_hip_joint_LF
    (*this)(5,2) = - ty_hip_joint_LF;    // Maxima DSL: -_k__ty_hip_joint_LF
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_imu_link_X_fr_hip_joint_LF& MotionTransforms::Type_imu_link_X_fr_hip_joint_LF::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_imu_link_X_fr_elbow_joint_LF::Type_imu_link_X_fr_elbow_joint_LF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_imu_link_X_fr_elbow_joint_LF& MotionTransforms::Type_imu_link_X_fr_elbow_joint_LF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    (*this)(1,0) = sin_q_hip_joint_LF;
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(2,0) = -cos_q_hip_joint_LF;
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(3,0) = - ty_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(3,2) = (- ty_hip_joint_LF * sin_q_hip_joint_LF)- tx_elbow_joint_LF;
    (*this)(4,0) =  tx_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(4,1) = - tx_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(4,2) =  tx_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(4,3) = sin_q_hip_joint_LF;
    (*this)(4,5) = -cos_q_hip_joint_LF;
    (*this)(5,0) =  tx_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(5,1) = (- tx_elbow_joint_LF * sin_q_hip_joint_LF)- ty_hip_joint_LF;
    (*this)(5,2) = - tx_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(5,3) = -cos_q_hip_joint_LF;
    (*this)(5,5) = -sin_q_hip_joint_LF;
    return *this;
}
MotionTransforms::Type_imu_link_X_fr_knee_joint_LF::Type_imu_link_X_fr_knee_joint_LF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_imu_link_X_fr_knee_joint_LF& MotionTransforms::Type_imu_link_X_fr_knee_joint_LF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    (*this)(0,0) = sin_q_elbow_joint_LF;
    (*this)(0,1) = cos_q_elbow_joint_LF;
    (*this)(1,0) = cos_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,1) = -sin_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(2,0) = -cos_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(2,1) = sin_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(3,0) = - ty_hip_joint_LF * cos_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(3,1) =  ty_hip_joint_LF * sin_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(3,2) = (- ty_hip_joint_LF * sin_q_hip_joint_LF)-( tx_knee_joint_LF * cos_q_elbow_joint_LF)- tx_elbow_joint_LF;
    (*this)(3,3) = sin_q_elbow_joint_LF;
    (*this)(3,4) = cos_q_elbow_joint_LF;
    (*this)(4,0) = (( tx_hip_joint_LF * cos_q_elbow_joint_LF)-( tx_elbow_joint_LF * sin_q_elbow_joint_LF)) * cos_q_hip_joint_LF;
    (*this)(4,1) = ((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * cos_q_hip_joint_LF;
    (*this)(4,2) = (( tx_knee_joint_LF * sin_q_elbow_joint_LF)+ tx_hip_joint_LF) * sin_q_hip_joint_LF;
    (*this)(4,3) = cos_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(4,4) = -sin_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(4,5) = -cos_q_hip_joint_LF;
    (*this)(5,0) = ((( tx_hip_joint_LF * cos_q_elbow_joint_LF)-( tx_elbow_joint_LF * sin_q_elbow_joint_LF)) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * sin_q_elbow_joint_LF);
    (*this)(5,1) = (((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * cos_q_elbow_joint_LF);
    (*this)(5,2) = ((- tx_knee_joint_LF * sin_q_elbow_joint_LF)- tx_hip_joint_LF) * cos_q_hip_joint_LF;
    (*this)(5,3) = -cos_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(5,4) = sin_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(5,5) = -sin_q_hip_joint_LF;
    return *this;
}
MotionTransforms::Type_imu_link_X_fr_hip_joint_RF::Type_imu_link_X_fr_hip_joint_RF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = - ty_hip_joint_RF;    // Maxima DSL: -_k__ty_hip_joint_RF
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) =  tx_hip_joint_RF;    // Maxima DSL: _k__tx_hip_joint_RF
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tx_hip_joint_RF;    // Maxima DSL: _k__tx_hip_joint_RF
    (*this)(5,2) = - ty_hip_joint_RF;    // Maxima DSL: -_k__ty_hip_joint_RF
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_imu_link_X_fr_hip_joint_RF& MotionTransforms::Type_imu_link_X_fr_hip_joint_RF::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_imu_link_X_fr_elbow_joint_RF::Type_imu_link_X_fr_elbow_joint_RF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_imu_link_X_fr_elbow_joint_RF& MotionTransforms::Type_imu_link_X_fr_elbow_joint_RF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    (*this)(1,0) = sin_q_hip_joint_RF;
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(2,0) = -cos_q_hip_joint_RF;
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(3,0) = - ty_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(3,2) = (- ty_hip_joint_RF * sin_q_hip_joint_RF)- tx_elbow_joint_RF;
    (*this)(4,0) =  tx_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(4,1) = - tx_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(4,2) =  tx_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(4,3) = sin_q_hip_joint_RF;
    (*this)(4,5) = -cos_q_hip_joint_RF;
    (*this)(5,0) =  tx_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(5,1) = (- tx_elbow_joint_RF * sin_q_hip_joint_RF)- ty_hip_joint_RF;
    (*this)(5,2) = - tx_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(5,3) = -cos_q_hip_joint_RF;
    (*this)(5,5) = -sin_q_hip_joint_RF;
    return *this;
}
MotionTransforms::Type_imu_link_X_fr_knee_joint_RF::Type_imu_link_X_fr_knee_joint_RF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_imu_link_X_fr_knee_joint_RF& MotionTransforms::Type_imu_link_X_fr_knee_joint_RF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    (*this)(0,0) = sin_q_elbow_joint_RF;
    (*this)(0,1) = cos_q_elbow_joint_RF;
    (*this)(1,0) = cos_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,1) = -sin_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(2,0) = -cos_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(2,1) = sin_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(3,0) = - ty_hip_joint_RF * cos_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(3,1) =  ty_hip_joint_RF * sin_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(3,2) = (- ty_hip_joint_RF * sin_q_hip_joint_RF)-( tx_knee_joint_RF * cos_q_elbow_joint_RF)- tx_elbow_joint_RF;
    (*this)(3,3) = sin_q_elbow_joint_RF;
    (*this)(3,4) = cos_q_elbow_joint_RF;
    (*this)(4,0) = (( tx_hip_joint_RF * cos_q_elbow_joint_RF)-( tx_elbow_joint_RF * sin_q_elbow_joint_RF)) * cos_q_hip_joint_RF;
    (*this)(4,1) = ((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * cos_q_hip_joint_RF;
    (*this)(4,2) = (( tx_knee_joint_RF * sin_q_elbow_joint_RF)+ tx_hip_joint_RF) * sin_q_hip_joint_RF;
    (*this)(4,3) = cos_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(4,4) = -sin_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(4,5) = -cos_q_hip_joint_RF;
    (*this)(5,0) = ((( tx_hip_joint_RF * cos_q_elbow_joint_RF)-( tx_elbow_joint_RF * sin_q_elbow_joint_RF)) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * sin_q_elbow_joint_RF);
    (*this)(5,1) = (((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * cos_q_elbow_joint_RF);
    (*this)(5,2) = ((- tx_knee_joint_RF * sin_q_elbow_joint_RF)- tx_hip_joint_RF) * cos_q_hip_joint_RF;
    (*this)(5,3) = -cos_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(5,4) = sin_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(5,5) = -sin_q_hip_joint_RF;
    return *this;
}
MotionTransforms::Type_imu_link_X_fr_hip_joint_LH::Type_imu_link_X_fr_hip_joint_LH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = - ty_hip_joint_LH;    // Maxima DSL: -_k__ty_hip_joint_LH
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) =  tx_hip_joint_LH;    // Maxima DSL: _k__tx_hip_joint_LH
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tx_hip_joint_LH;    // Maxima DSL: _k__tx_hip_joint_LH
    (*this)(5,2) = - ty_hip_joint_LH;    // Maxima DSL: -_k__ty_hip_joint_LH
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_imu_link_X_fr_hip_joint_LH& MotionTransforms::Type_imu_link_X_fr_hip_joint_LH::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_imu_link_X_fr_elbow_joint_LH::Type_imu_link_X_fr_elbow_joint_LH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_imu_link_X_fr_elbow_joint_LH& MotionTransforms::Type_imu_link_X_fr_elbow_joint_LH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    (*this)(1,0) = sin_q_hip_joint_LH;
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(2,0) = -cos_q_hip_joint_LH;
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(3,0) = - ty_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(3,2) = (- ty_hip_joint_LH * sin_q_hip_joint_LH)- tx_elbow_joint_LH;
    (*this)(4,0) =  tx_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(4,1) = - tx_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(4,2) =  tx_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(4,3) = sin_q_hip_joint_LH;
    (*this)(4,5) = -cos_q_hip_joint_LH;
    (*this)(5,0) =  tx_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(5,1) = (- tx_elbow_joint_LH * sin_q_hip_joint_LH)- ty_hip_joint_LH;
    (*this)(5,2) = - tx_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(5,3) = -cos_q_hip_joint_LH;
    (*this)(5,5) = -sin_q_hip_joint_LH;
    return *this;
}
MotionTransforms::Type_imu_link_X_fr_knee_joint_LH::Type_imu_link_X_fr_knee_joint_LH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_imu_link_X_fr_knee_joint_LH& MotionTransforms::Type_imu_link_X_fr_knee_joint_LH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    (*this)(0,0) = sin_q_elbow_joint_LH;
    (*this)(0,1) = cos_q_elbow_joint_LH;
    (*this)(1,0) = cos_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,1) = -sin_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(2,0) = -cos_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(2,1) = sin_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(3,0) = - ty_hip_joint_LH * cos_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(3,1) =  ty_hip_joint_LH * sin_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(3,2) = (- ty_hip_joint_LH * sin_q_hip_joint_LH)-( tx_knee_joint_LH * cos_q_elbow_joint_LH)- tx_elbow_joint_LH;
    (*this)(3,3) = sin_q_elbow_joint_LH;
    (*this)(3,4) = cos_q_elbow_joint_LH;
    (*this)(4,0) = (( tx_hip_joint_LH * cos_q_elbow_joint_LH)-( tx_elbow_joint_LH * sin_q_elbow_joint_LH)) * cos_q_hip_joint_LH;
    (*this)(4,1) = ((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * cos_q_hip_joint_LH;
    (*this)(4,2) = (( tx_knee_joint_LH * sin_q_elbow_joint_LH)+ tx_hip_joint_LH) * sin_q_hip_joint_LH;
    (*this)(4,3) = cos_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(4,4) = -sin_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(4,5) = -cos_q_hip_joint_LH;
    (*this)(5,0) = ((( tx_hip_joint_LH * cos_q_elbow_joint_LH)-( tx_elbow_joint_LH * sin_q_elbow_joint_LH)) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * sin_q_elbow_joint_LH);
    (*this)(5,1) = (((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * cos_q_elbow_joint_LH);
    (*this)(5,2) = ((- tx_knee_joint_LH * sin_q_elbow_joint_LH)- tx_hip_joint_LH) * cos_q_hip_joint_LH;
    (*this)(5,3) = -cos_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(5,4) = sin_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(5,5) = -sin_q_hip_joint_LH;
    return *this;
}
MotionTransforms::Type_imu_link_X_fr_hip_joint_RH::Type_imu_link_X_fr_hip_joint_RH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = - ty_hip_joint_RH;    // Maxima DSL: -_k__ty_hip_joint_RH
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) =  tx_hip_joint_RH;    // Maxima DSL: _k__tx_hip_joint_RH
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tx_hip_joint_RH;    // Maxima DSL: _k__tx_hip_joint_RH
    (*this)(5,2) = - ty_hip_joint_RH;    // Maxima DSL: -_k__ty_hip_joint_RH
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_imu_link_X_fr_hip_joint_RH& MotionTransforms::Type_imu_link_X_fr_hip_joint_RH::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_imu_link_X_fr_elbow_joint_RH::Type_imu_link_X_fr_elbow_joint_RH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_imu_link_X_fr_elbow_joint_RH& MotionTransforms::Type_imu_link_X_fr_elbow_joint_RH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    (*this)(1,0) = sin_q_hip_joint_RH;
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(2,0) = -cos_q_hip_joint_RH;
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(3,0) = - ty_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(3,2) = (- ty_hip_joint_RH * sin_q_hip_joint_RH)- tx_elbow_joint_RH;
    (*this)(4,0) =  tx_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(4,1) = - tx_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(4,2) =  tx_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(4,3) = sin_q_hip_joint_RH;
    (*this)(4,5) = -cos_q_hip_joint_RH;
    (*this)(5,0) =  tx_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(5,1) = (- tx_elbow_joint_RH * sin_q_hip_joint_RH)- ty_hip_joint_RH;
    (*this)(5,2) = - tx_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(5,3) = -cos_q_hip_joint_RH;
    (*this)(5,5) = -sin_q_hip_joint_RH;
    return *this;
}
MotionTransforms::Type_imu_link_X_fr_knee_joint_RH::Type_imu_link_X_fr_knee_joint_RH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_imu_link_X_fr_knee_joint_RH& MotionTransforms::Type_imu_link_X_fr_knee_joint_RH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    (*this)(0,0) = sin_q_elbow_joint_RH;
    (*this)(0,1) = cos_q_elbow_joint_RH;
    (*this)(1,0) = cos_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,1) = -sin_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(2,0) = -cos_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(2,1) = sin_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(3,0) = - ty_hip_joint_RH * cos_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(3,1) =  ty_hip_joint_RH * sin_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(3,2) = (- ty_hip_joint_RH * sin_q_hip_joint_RH)-( tx_knee_joint_RH * cos_q_elbow_joint_RH)- tx_elbow_joint_RH;
    (*this)(3,3) = sin_q_elbow_joint_RH;
    (*this)(3,4) = cos_q_elbow_joint_RH;
    (*this)(4,0) = (( tx_hip_joint_RH * cos_q_elbow_joint_RH)-( tx_elbow_joint_RH * sin_q_elbow_joint_RH)) * cos_q_hip_joint_RH;
    (*this)(4,1) = ((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * cos_q_hip_joint_RH;
    (*this)(4,2) = (( tx_knee_joint_RH * sin_q_elbow_joint_RH)+ tx_hip_joint_RH) * sin_q_hip_joint_RH;
    (*this)(4,3) = cos_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(4,4) = -sin_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(4,5) = -cos_q_hip_joint_RH;
    (*this)(5,0) = ((( tx_hip_joint_RH * cos_q_elbow_joint_RH)-( tx_elbow_joint_RH * sin_q_elbow_joint_RH)) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * sin_q_elbow_joint_RH);
    (*this)(5,1) = (((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * cos_q_elbow_joint_RH);
    (*this)(5,2) = ((- tx_knee_joint_RH * sin_q_elbow_joint_RH)- tx_hip_joint_RH) * cos_q_hip_joint_RH;
    (*this)(5,3) = -cos_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(5,4) = sin_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(5,5) = -sin_q_hip_joint_RH;
    return *this;
}
MotionTransforms::Type_fr_hip_link_LF_X_fr_base::Type_fr_hip_link_LF_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = - ty_hip_joint_LF;    // Maxima DSL: -_k__ty_hip_joint_LF
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_hip_link_LF_X_fr_base& MotionTransforms::Type_fr_hip_link_LF_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    (*this)(0,1) = sin_q_hip_joint_LF;
    (*this)(0,2) = -cos_q_hip_joint_LF;
    (*this)(1,1) = cos_q_hip_joint_LF;
    (*this)(1,2) = sin_q_hip_joint_LF;
    (*this)(3,0) = - ty_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(3,1) =  tx_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(3,2) =  tx_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(3,4) = sin_q_hip_joint_LF;
    (*this)(3,5) = -cos_q_hip_joint_LF;
    (*this)(4,0) =  ty_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(4,1) = - tx_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(4,2) =  tx_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(4,4) = cos_q_hip_joint_LF;
    (*this)(4,5) = sin_q_hip_joint_LF;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_hip_link_LF::Type_fr_base_X_fr_hip_link_LF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = - ty_hip_joint_LF;    // Maxima DSL: -_k__ty_hip_joint_LF
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_hip_link_LF& MotionTransforms::Type_fr_base_X_fr_hip_link_LF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    (*this)(1,0) = sin_q_hip_joint_LF;
    (*this)(1,1) = cos_q_hip_joint_LF;
    (*this)(2,0) = -cos_q_hip_joint_LF;
    (*this)(2,1) = sin_q_hip_joint_LF;
    (*this)(3,0) = - ty_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(3,1) =  ty_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(4,0) =  tx_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(4,1) = - tx_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(4,3) = sin_q_hip_joint_LF;
    (*this)(4,4) = cos_q_hip_joint_LF;
    (*this)(5,0) =  tx_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(5,1) =  tx_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(5,3) = -cos_q_hip_joint_LF;
    (*this)(5,4) = sin_q_hip_joint_LF;
    return *this;
}
MotionTransforms::Type_fr_thigh_link_LF_X_fr_hip_link_LF::Type_fr_thigh_link_LF_X_fr_hip_link_LF()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = - tx_elbow_joint_LF;    // Maxima DSL: -_k__tx_elbow_joint_LF
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_thigh_link_LF_X_fr_hip_link_LF& MotionTransforms::Type_fr_thigh_link_LF_X_fr_hip_link_LF::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    (*this)(0,0) = cos_q_elbow_joint_LF;
    (*this)(0,2) = sin_q_elbow_joint_LF;
    (*this)(1,0) = -sin_q_elbow_joint_LF;
    (*this)(1,2) = cos_q_elbow_joint_LF;
    (*this)(3,1) = - tx_elbow_joint_LF * sin_q_elbow_joint_LF;
    (*this)(3,3) = cos_q_elbow_joint_LF;
    (*this)(3,5) = sin_q_elbow_joint_LF;
    (*this)(4,1) = - tx_elbow_joint_LF * cos_q_elbow_joint_LF;
    (*this)(4,3) = -sin_q_elbow_joint_LF;
    (*this)(4,5) = cos_q_elbow_joint_LF;
    return *this;
}
MotionTransforms::Type_fr_hip_link_LF_X_fr_thigh_link_LF::Type_fr_hip_link_LF_X_fr_thigh_link_LF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = - tx_elbow_joint_LF;    // Maxima DSL: -_k__tx_elbow_joint_LF
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_hip_link_LF_X_fr_thigh_link_LF& MotionTransforms::Type_fr_hip_link_LF_X_fr_thigh_link_LF::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    (*this)(0,0) = cos_q_elbow_joint_LF;
    (*this)(0,1) = -sin_q_elbow_joint_LF;
    (*this)(2,0) = sin_q_elbow_joint_LF;
    (*this)(2,1) = cos_q_elbow_joint_LF;
    (*this)(3,3) = cos_q_elbow_joint_LF;
    (*this)(3,4) = -sin_q_elbow_joint_LF;
    (*this)(4,0) = - tx_elbow_joint_LF * sin_q_elbow_joint_LF;
    (*this)(4,1) = - tx_elbow_joint_LF * cos_q_elbow_joint_LF;
    (*this)(5,3) = sin_q_elbow_joint_LF;
    (*this)(5,4) = cos_q_elbow_joint_LF;
    return *this;
}
MotionTransforms::Type_fr_shank_link_LF_X_fr_thigh_link_LF::Type_fr_shank_link_LF_X_fr_thigh_link_LF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tx_knee_joint_LF;    // Maxima DSL: -_k__tx_knee_joint_LF
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_shank_link_LF_X_fr_thigh_link_LF& MotionTransforms::Type_fr_shank_link_LF_X_fr_thigh_link_LF::update(const state_t& q)
{
    Scalar sin_q_knee_joint_LF  = ScalarTraits::sin( q(KNEE_JOINT_LF) );
    Scalar cos_q_knee_joint_LF  = ScalarTraits::cos( q(KNEE_JOINT_LF) );
    (*this)(0,0) = cos_q_knee_joint_LF;
    (*this)(0,1) = sin_q_knee_joint_LF;
    (*this)(1,0) = -sin_q_knee_joint_LF;
    (*this)(1,1) = cos_q_knee_joint_LF;
    (*this)(3,2) =  tx_knee_joint_LF * sin_q_knee_joint_LF;
    (*this)(3,3) = cos_q_knee_joint_LF;
    (*this)(3,4) = sin_q_knee_joint_LF;
    (*this)(4,2) =  tx_knee_joint_LF * cos_q_knee_joint_LF;
    (*this)(4,3) = -sin_q_knee_joint_LF;
    (*this)(4,4) = cos_q_knee_joint_LF;
    return *this;
}
MotionTransforms::Type_fr_thigh_link_LF_X_fr_shank_link_LF::Type_fr_thigh_link_LF_X_fr_shank_link_LF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = - tx_knee_joint_LF;    // Maxima DSL: -_k__tx_knee_joint_LF
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_thigh_link_LF_X_fr_shank_link_LF& MotionTransforms::Type_fr_thigh_link_LF_X_fr_shank_link_LF::update(const state_t& q)
{
    Scalar sin_q_knee_joint_LF  = ScalarTraits::sin( q(KNEE_JOINT_LF) );
    Scalar cos_q_knee_joint_LF  = ScalarTraits::cos( q(KNEE_JOINT_LF) );
    (*this)(0,0) = cos_q_knee_joint_LF;
    (*this)(0,1) = -sin_q_knee_joint_LF;
    (*this)(1,0) = sin_q_knee_joint_LF;
    (*this)(1,1) = cos_q_knee_joint_LF;
    (*this)(3,3) = cos_q_knee_joint_LF;
    (*this)(3,4) = -sin_q_knee_joint_LF;
    (*this)(4,3) = sin_q_knee_joint_LF;
    (*this)(4,4) = cos_q_knee_joint_LF;
    (*this)(5,0) =  tx_knee_joint_LF * sin_q_knee_joint_LF;
    (*this)(5,1) =  tx_knee_joint_LF * cos_q_knee_joint_LF;
    return *this;
}
MotionTransforms::Type_fr_hip_link_RF_X_fr_base::Type_fr_hip_link_RF_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = - ty_hip_joint_RF;    // Maxima DSL: -_k__ty_hip_joint_RF
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_hip_link_RF_X_fr_base& MotionTransforms::Type_fr_hip_link_RF_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    (*this)(0,1) = sin_q_hip_joint_RF;
    (*this)(0,2) = -cos_q_hip_joint_RF;
    (*this)(1,1) = cos_q_hip_joint_RF;
    (*this)(1,2) = sin_q_hip_joint_RF;
    (*this)(3,0) = - ty_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(3,1) =  tx_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(3,2) =  tx_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(3,4) = sin_q_hip_joint_RF;
    (*this)(3,5) = -cos_q_hip_joint_RF;
    (*this)(4,0) =  ty_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(4,1) = - tx_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(4,2) =  tx_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(4,4) = cos_q_hip_joint_RF;
    (*this)(4,5) = sin_q_hip_joint_RF;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_hip_link_RF::Type_fr_base_X_fr_hip_link_RF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = - ty_hip_joint_RF;    // Maxima DSL: -_k__ty_hip_joint_RF
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_hip_link_RF& MotionTransforms::Type_fr_base_X_fr_hip_link_RF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    (*this)(1,0) = sin_q_hip_joint_RF;
    (*this)(1,1) = cos_q_hip_joint_RF;
    (*this)(2,0) = -cos_q_hip_joint_RF;
    (*this)(2,1) = sin_q_hip_joint_RF;
    (*this)(3,0) = - ty_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(3,1) =  ty_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(4,0) =  tx_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(4,1) = - tx_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(4,3) = sin_q_hip_joint_RF;
    (*this)(4,4) = cos_q_hip_joint_RF;
    (*this)(5,0) =  tx_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(5,1) =  tx_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(5,3) = -cos_q_hip_joint_RF;
    (*this)(5,4) = sin_q_hip_joint_RF;
    return *this;
}
MotionTransforms::Type_fr_thigh_link_RF_X_fr_hip_link_RF::Type_fr_thigh_link_RF_X_fr_hip_link_RF()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = - tx_elbow_joint_RF;    // Maxima DSL: -_k__tx_elbow_joint_RF
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_thigh_link_RF_X_fr_hip_link_RF& MotionTransforms::Type_fr_thigh_link_RF_X_fr_hip_link_RF::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    (*this)(0,0) = cos_q_elbow_joint_RF;
    (*this)(0,2) = sin_q_elbow_joint_RF;
    (*this)(1,0) = -sin_q_elbow_joint_RF;
    (*this)(1,2) = cos_q_elbow_joint_RF;
    (*this)(3,1) = - tx_elbow_joint_RF * sin_q_elbow_joint_RF;
    (*this)(3,3) = cos_q_elbow_joint_RF;
    (*this)(3,5) = sin_q_elbow_joint_RF;
    (*this)(4,1) = - tx_elbow_joint_RF * cos_q_elbow_joint_RF;
    (*this)(4,3) = -sin_q_elbow_joint_RF;
    (*this)(4,5) = cos_q_elbow_joint_RF;
    return *this;
}
MotionTransforms::Type_fr_hip_link_RF_X_fr_thigh_link_RF::Type_fr_hip_link_RF_X_fr_thigh_link_RF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = - tx_elbow_joint_RF;    // Maxima DSL: -_k__tx_elbow_joint_RF
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_hip_link_RF_X_fr_thigh_link_RF& MotionTransforms::Type_fr_hip_link_RF_X_fr_thigh_link_RF::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    (*this)(0,0) = cos_q_elbow_joint_RF;
    (*this)(0,1) = -sin_q_elbow_joint_RF;
    (*this)(2,0) = sin_q_elbow_joint_RF;
    (*this)(2,1) = cos_q_elbow_joint_RF;
    (*this)(3,3) = cos_q_elbow_joint_RF;
    (*this)(3,4) = -sin_q_elbow_joint_RF;
    (*this)(4,0) = - tx_elbow_joint_RF * sin_q_elbow_joint_RF;
    (*this)(4,1) = - tx_elbow_joint_RF * cos_q_elbow_joint_RF;
    (*this)(5,3) = sin_q_elbow_joint_RF;
    (*this)(5,4) = cos_q_elbow_joint_RF;
    return *this;
}
MotionTransforms::Type_fr_shank_link_RF_X_fr_thigh_link_RF::Type_fr_shank_link_RF_X_fr_thigh_link_RF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tx_knee_joint_RF;    // Maxima DSL: -_k__tx_knee_joint_RF
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_shank_link_RF_X_fr_thigh_link_RF& MotionTransforms::Type_fr_shank_link_RF_X_fr_thigh_link_RF::update(const state_t& q)
{
    Scalar sin_q_knee_joint_RF  = ScalarTraits::sin( q(KNEE_JOINT_RF) );
    Scalar cos_q_knee_joint_RF  = ScalarTraits::cos( q(KNEE_JOINT_RF) );
    (*this)(0,0) = cos_q_knee_joint_RF;
    (*this)(0,1) = sin_q_knee_joint_RF;
    (*this)(1,0) = -sin_q_knee_joint_RF;
    (*this)(1,1) = cos_q_knee_joint_RF;
    (*this)(3,2) =  tx_knee_joint_RF * sin_q_knee_joint_RF;
    (*this)(3,3) = cos_q_knee_joint_RF;
    (*this)(3,4) = sin_q_knee_joint_RF;
    (*this)(4,2) =  tx_knee_joint_RF * cos_q_knee_joint_RF;
    (*this)(4,3) = -sin_q_knee_joint_RF;
    (*this)(4,4) = cos_q_knee_joint_RF;
    return *this;
}
MotionTransforms::Type_fr_thigh_link_RF_X_fr_shank_link_RF::Type_fr_thigh_link_RF_X_fr_shank_link_RF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = - tx_knee_joint_RF;    // Maxima DSL: -_k__tx_knee_joint_RF
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_thigh_link_RF_X_fr_shank_link_RF& MotionTransforms::Type_fr_thigh_link_RF_X_fr_shank_link_RF::update(const state_t& q)
{
    Scalar sin_q_knee_joint_RF  = ScalarTraits::sin( q(KNEE_JOINT_RF) );
    Scalar cos_q_knee_joint_RF  = ScalarTraits::cos( q(KNEE_JOINT_RF) );
    (*this)(0,0) = cos_q_knee_joint_RF;
    (*this)(0,1) = -sin_q_knee_joint_RF;
    (*this)(1,0) = sin_q_knee_joint_RF;
    (*this)(1,1) = cos_q_knee_joint_RF;
    (*this)(3,3) = cos_q_knee_joint_RF;
    (*this)(3,4) = -sin_q_knee_joint_RF;
    (*this)(4,3) = sin_q_knee_joint_RF;
    (*this)(4,4) = cos_q_knee_joint_RF;
    (*this)(5,0) =  tx_knee_joint_RF * sin_q_knee_joint_RF;
    (*this)(5,1) =  tx_knee_joint_RF * cos_q_knee_joint_RF;
    return *this;
}
MotionTransforms::Type_fr_hip_link_LH_X_fr_base::Type_fr_hip_link_LH_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = - ty_hip_joint_LH;    // Maxima DSL: -_k__ty_hip_joint_LH
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_hip_link_LH_X_fr_base& MotionTransforms::Type_fr_hip_link_LH_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    (*this)(0,1) = sin_q_hip_joint_LH;
    (*this)(0,2) = -cos_q_hip_joint_LH;
    (*this)(1,1) = cos_q_hip_joint_LH;
    (*this)(1,2) = sin_q_hip_joint_LH;
    (*this)(3,0) = - ty_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(3,1) =  tx_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(3,2) =  tx_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(3,4) = sin_q_hip_joint_LH;
    (*this)(3,5) = -cos_q_hip_joint_LH;
    (*this)(4,0) =  ty_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(4,1) = - tx_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(4,2) =  tx_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(4,4) = cos_q_hip_joint_LH;
    (*this)(4,5) = sin_q_hip_joint_LH;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_hip_link_LH::Type_fr_base_X_fr_hip_link_LH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = - ty_hip_joint_LH;    // Maxima DSL: -_k__ty_hip_joint_LH
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_hip_link_LH& MotionTransforms::Type_fr_base_X_fr_hip_link_LH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    (*this)(1,0) = sin_q_hip_joint_LH;
    (*this)(1,1) = cos_q_hip_joint_LH;
    (*this)(2,0) = -cos_q_hip_joint_LH;
    (*this)(2,1) = sin_q_hip_joint_LH;
    (*this)(3,0) = - ty_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(3,1) =  ty_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(4,0) =  tx_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(4,1) = - tx_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(4,3) = sin_q_hip_joint_LH;
    (*this)(4,4) = cos_q_hip_joint_LH;
    (*this)(5,0) =  tx_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(5,1) =  tx_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(5,3) = -cos_q_hip_joint_LH;
    (*this)(5,4) = sin_q_hip_joint_LH;
    return *this;
}
MotionTransforms::Type_fr_thigh_link_LH_X_fr_hip_link_LH::Type_fr_thigh_link_LH_X_fr_hip_link_LH()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = - tx_elbow_joint_LH;    // Maxima DSL: -_k__tx_elbow_joint_LH
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_thigh_link_LH_X_fr_hip_link_LH& MotionTransforms::Type_fr_thigh_link_LH_X_fr_hip_link_LH::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    (*this)(0,0) = cos_q_elbow_joint_LH;
    (*this)(0,2) = sin_q_elbow_joint_LH;
    (*this)(1,0) = -sin_q_elbow_joint_LH;
    (*this)(1,2) = cos_q_elbow_joint_LH;
    (*this)(3,1) = - tx_elbow_joint_LH * sin_q_elbow_joint_LH;
    (*this)(3,3) = cos_q_elbow_joint_LH;
    (*this)(3,5) = sin_q_elbow_joint_LH;
    (*this)(4,1) = - tx_elbow_joint_LH * cos_q_elbow_joint_LH;
    (*this)(4,3) = -sin_q_elbow_joint_LH;
    (*this)(4,5) = cos_q_elbow_joint_LH;
    return *this;
}
MotionTransforms::Type_fr_hip_link_LH_X_fr_thigh_link_LH::Type_fr_hip_link_LH_X_fr_thigh_link_LH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = - tx_elbow_joint_LH;    // Maxima DSL: -_k__tx_elbow_joint_LH
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_hip_link_LH_X_fr_thigh_link_LH& MotionTransforms::Type_fr_hip_link_LH_X_fr_thigh_link_LH::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    (*this)(0,0) = cos_q_elbow_joint_LH;
    (*this)(0,1) = -sin_q_elbow_joint_LH;
    (*this)(2,0) = sin_q_elbow_joint_LH;
    (*this)(2,1) = cos_q_elbow_joint_LH;
    (*this)(3,3) = cos_q_elbow_joint_LH;
    (*this)(3,4) = -sin_q_elbow_joint_LH;
    (*this)(4,0) = - tx_elbow_joint_LH * sin_q_elbow_joint_LH;
    (*this)(4,1) = - tx_elbow_joint_LH * cos_q_elbow_joint_LH;
    (*this)(5,3) = sin_q_elbow_joint_LH;
    (*this)(5,4) = cos_q_elbow_joint_LH;
    return *this;
}
MotionTransforms::Type_fr_shank_link_LH_X_fr_thigh_link_LH::Type_fr_shank_link_LH_X_fr_thigh_link_LH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tx_knee_joint_LH;    // Maxima DSL: -_k__tx_knee_joint_LH
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_shank_link_LH_X_fr_thigh_link_LH& MotionTransforms::Type_fr_shank_link_LH_X_fr_thigh_link_LH::update(const state_t& q)
{
    Scalar sin_q_knee_joint_LH  = ScalarTraits::sin( q(KNEE_JOINT_LH) );
    Scalar cos_q_knee_joint_LH  = ScalarTraits::cos( q(KNEE_JOINT_LH) );
    (*this)(0,0) = cos_q_knee_joint_LH;
    (*this)(0,1) = sin_q_knee_joint_LH;
    (*this)(1,0) = -sin_q_knee_joint_LH;
    (*this)(1,1) = cos_q_knee_joint_LH;
    (*this)(3,2) =  tx_knee_joint_LH * sin_q_knee_joint_LH;
    (*this)(3,3) = cos_q_knee_joint_LH;
    (*this)(3,4) = sin_q_knee_joint_LH;
    (*this)(4,2) =  tx_knee_joint_LH * cos_q_knee_joint_LH;
    (*this)(4,3) = -sin_q_knee_joint_LH;
    (*this)(4,4) = cos_q_knee_joint_LH;
    return *this;
}
MotionTransforms::Type_fr_thigh_link_LH_X_fr_shank_link_LH::Type_fr_thigh_link_LH_X_fr_shank_link_LH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = - tx_knee_joint_LH;    // Maxima DSL: -_k__tx_knee_joint_LH
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_thigh_link_LH_X_fr_shank_link_LH& MotionTransforms::Type_fr_thigh_link_LH_X_fr_shank_link_LH::update(const state_t& q)
{
    Scalar sin_q_knee_joint_LH  = ScalarTraits::sin( q(KNEE_JOINT_LH) );
    Scalar cos_q_knee_joint_LH  = ScalarTraits::cos( q(KNEE_JOINT_LH) );
    (*this)(0,0) = cos_q_knee_joint_LH;
    (*this)(0,1) = -sin_q_knee_joint_LH;
    (*this)(1,0) = sin_q_knee_joint_LH;
    (*this)(1,1) = cos_q_knee_joint_LH;
    (*this)(3,3) = cos_q_knee_joint_LH;
    (*this)(3,4) = -sin_q_knee_joint_LH;
    (*this)(4,3) = sin_q_knee_joint_LH;
    (*this)(4,4) = cos_q_knee_joint_LH;
    (*this)(5,0) =  tx_knee_joint_LH * sin_q_knee_joint_LH;
    (*this)(5,1) =  tx_knee_joint_LH * cos_q_knee_joint_LH;
    return *this;
}
MotionTransforms::Type_fr_hip_link_RH_X_fr_base::Type_fr_hip_link_RH_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = - ty_hip_joint_RH;    // Maxima DSL: -_k__ty_hip_joint_RH
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_hip_link_RH_X_fr_base& MotionTransforms::Type_fr_hip_link_RH_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    (*this)(0,1) = sin_q_hip_joint_RH;
    (*this)(0,2) = -cos_q_hip_joint_RH;
    (*this)(1,1) = cos_q_hip_joint_RH;
    (*this)(1,2) = sin_q_hip_joint_RH;
    (*this)(3,0) = - ty_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(3,1) =  tx_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(3,2) =  tx_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(3,4) = sin_q_hip_joint_RH;
    (*this)(3,5) = -cos_q_hip_joint_RH;
    (*this)(4,0) =  ty_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(4,1) = - tx_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(4,2) =  tx_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(4,4) = cos_q_hip_joint_RH;
    (*this)(4,5) = sin_q_hip_joint_RH;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_hip_link_RH::Type_fr_base_X_fr_hip_link_RH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = - ty_hip_joint_RH;    // Maxima DSL: -_k__ty_hip_joint_RH
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_hip_link_RH& MotionTransforms::Type_fr_base_X_fr_hip_link_RH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    (*this)(1,0) = sin_q_hip_joint_RH;
    (*this)(1,1) = cos_q_hip_joint_RH;
    (*this)(2,0) = -cos_q_hip_joint_RH;
    (*this)(2,1) = sin_q_hip_joint_RH;
    (*this)(3,0) = - ty_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(3,1) =  ty_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(4,0) =  tx_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(4,1) = - tx_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(4,3) = sin_q_hip_joint_RH;
    (*this)(4,4) = cos_q_hip_joint_RH;
    (*this)(5,0) =  tx_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(5,1) =  tx_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(5,3) = -cos_q_hip_joint_RH;
    (*this)(5,4) = sin_q_hip_joint_RH;
    return *this;
}
MotionTransforms::Type_fr_thigh_link_RH_X_fr_hip_link_RH::Type_fr_thigh_link_RH_X_fr_hip_link_RH()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = - tx_elbow_joint_RH;    // Maxima DSL: -_k__tx_elbow_joint_RH
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_thigh_link_RH_X_fr_hip_link_RH& MotionTransforms::Type_fr_thigh_link_RH_X_fr_hip_link_RH::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    (*this)(0,0) = cos_q_elbow_joint_RH;
    (*this)(0,2) = sin_q_elbow_joint_RH;
    (*this)(1,0) = -sin_q_elbow_joint_RH;
    (*this)(1,2) = cos_q_elbow_joint_RH;
    (*this)(3,1) = - tx_elbow_joint_RH * sin_q_elbow_joint_RH;
    (*this)(3,3) = cos_q_elbow_joint_RH;
    (*this)(3,5) = sin_q_elbow_joint_RH;
    (*this)(4,1) = - tx_elbow_joint_RH * cos_q_elbow_joint_RH;
    (*this)(4,3) = -sin_q_elbow_joint_RH;
    (*this)(4,5) = cos_q_elbow_joint_RH;
    return *this;
}
MotionTransforms::Type_fr_hip_link_RH_X_fr_thigh_link_RH::Type_fr_hip_link_RH_X_fr_thigh_link_RH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = - tx_elbow_joint_RH;    // Maxima DSL: -_k__tx_elbow_joint_RH
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_hip_link_RH_X_fr_thigh_link_RH& MotionTransforms::Type_fr_hip_link_RH_X_fr_thigh_link_RH::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    (*this)(0,0) = cos_q_elbow_joint_RH;
    (*this)(0,1) = -sin_q_elbow_joint_RH;
    (*this)(2,0) = sin_q_elbow_joint_RH;
    (*this)(2,1) = cos_q_elbow_joint_RH;
    (*this)(3,3) = cos_q_elbow_joint_RH;
    (*this)(3,4) = -sin_q_elbow_joint_RH;
    (*this)(4,0) = - tx_elbow_joint_RH * sin_q_elbow_joint_RH;
    (*this)(4,1) = - tx_elbow_joint_RH * cos_q_elbow_joint_RH;
    (*this)(5,3) = sin_q_elbow_joint_RH;
    (*this)(5,4) = cos_q_elbow_joint_RH;
    return *this;
}
MotionTransforms::Type_fr_shank_link_RH_X_fr_thigh_link_RH::Type_fr_shank_link_RH_X_fr_thigh_link_RH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tx_knee_joint_RH;    // Maxima DSL: -_k__tx_knee_joint_RH
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_shank_link_RH_X_fr_thigh_link_RH& MotionTransforms::Type_fr_shank_link_RH_X_fr_thigh_link_RH::update(const state_t& q)
{
    Scalar sin_q_knee_joint_RH  = ScalarTraits::sin( q(KNEE_JOINT_RH) );
    Scalar cos_q_knee_joint_RH  = ScalarTraits::cos( q(KNEE_JOINT_RH) );
    (*this)(0,0) = cos_q_knee_joint_RH;
    (*this)(0,1) = sin_q_knee_joint_RH;
    (*this)(1,0) = -sin_q_knee_joint_RH;
    (*this)(1,1) = cos_q_knee_joint_RH;
    (*this)(3,2) =  tx_knee_joint_RH * sin_q_knee_joint_RH;
    (*this)(3,3) = cos_q_knee_joint_RH;
    (*this)(3,4) = sin_q_knee_joint_RH;
    (*this)(4,2) =  tx_knee_joint_RH * cos_q_knee_joint_RH;
    (*this)(4,3) = -sin_q_knee_joint_RH;
    (*this)(4,4) = cos_q_knee_joint_RH;
    return *this;
}
MotionTransforms::Type_fr_thigh_link_RH_X_fr_shank_link_RH::Type_fr_thigh_link_RH_X_fr_shank_link_RH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = - tx_knee_joint_RH;    // Maxima DSL: -_k__tx_knee_joint_RH
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_thigh_link_RH_X_fr_shank_link_RH& MotionTransforms::Type_fr_thigh_link_RH_X_fr_shank_link_RH::update(const state_t& q)
{
    Scalar sin_q_knee_joint_RH  = ScalarTraits::sin( q(KNEE_JOINT_RH) );
    Scalar cos_q_knee_joint_RH  = ScalarTraits::cos( q(KNEE_JOINT_RH) );
    (*this)(0,0) = cos_q_knee_joint_RH;
    (*this)(0,1) = -sin_q_knee_joint_RH;
    (*this)(1,0) = sin_q_knee_joint_RH;
    (*this)(1,1) = cos_q_knee_joint_RH;
    (*this)(3,3) = cos_q_knee_joint_RH;
    (*this)(3,4) = -sin_q_knee_joint_RH;
    (*this)(4,3) = sin_q_knee_joint_RH;
    (*this)(4,4) = cos_q_knee_joint_RH;
    (*this)(5,0) =  tx_knee_joint_RH * sin_q_knee_joint_RH;
    (*this)(5,1) =  tx_knee_joint_RH * cos_q_knee_joint_RH;
    return *this;
}

ForceTransforms::Type_fr_base_X_LF_FOOT::Type_fr_base_X_LF_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_LF_FOOT& ForceTransforms::Type_fr_base_X_LF_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    Scalar sin_q_knee_joint_LF  = ScalarTraits::sin( q(KNEE_JOINT_LF) );
    Scalar cos_q_knee_joint_LF  = ScalarTraits::cos( q(KNEE_JOINT_LF) );
    (*this)(0,0) = (cos_q_elbow_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_knee_joint_LF);
    (*this)(0,1) = (cos_q_elbow_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_knee_joint_LF);
    (*this)(0,3) = ( ty_hip_joint_LF * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-( ty_hip_joint_LF * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(0,4) = ( ty_hip_joint_LF * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+( ty_hip_joint_LF * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(0,5) = ( tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_knee_joint_LF)-( tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_knee_joint_LF)-( ty_hip_joint_LF * sin_q_hip_joint_LF)-( tx_knee_joint_LF * cos_q_elbow_joint_LF)- tx_elbow_joint_LF;
    (*this)(1,0) = (cos_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF);
    (*this)(1,1) = (-cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(1,3) = (((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+((( tx_hip_joint_LF * cos_q_elbow_joint_LF)-( tx_elbow_joint_LF * sin_q_elbow_joint_LF)) * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(1,4) = ((( tx_elbow_joint_LF * sin_q_elbow_joint_LF)-( tx_hip_joint_LF * cos_q_elbow_joint_LF)) * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+(((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * cos_q_hip_joint_LF * cos_q_knee_joint_LF)-( tx_LF_FOOT * cos_q_hip_joint_LF);
    (*this)(1,5) = ( tx_LF_FOOT * cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)+((( tx_knee_joint_LF * sin_q_elbow_joint_LF)+ tx_hip_joint_LF) * sin_q_hip_joint_LF);
    (*this)(2,0) = (sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-(cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(2,1) = (cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(2,3) = (((((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * cos_q_elbow_joint_LF)) * sin_q_knee_joint_LF)+((((( tx_hip_joint_LF * cos_q_elbow_joint_LF)-( tx_elbow_joint_LF * sin_q_elbow_joint_LF)) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * sin_q_elbow_joint_LF)) * cos_q_knee_joint_LF);
    (*this)(2,4) = ((((( tx_elbow_joint_LF * sin_q_elbow_joint_LF)-( tx_hip_joint_LF * cos_q_elbow_joint_LF)) * sin_q_hip_joint_LF)+( ty_hip_joint_LF * sin_q_elbow_joint_LF)) * sin_q_knee_joint_LF)+(((((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * cos_q_elbow_joint_LF)) * cos_q_knee_joint_LF)-( tx_LF_FOOT * sin_q_hip_joint_LF);
    (*this)(2,5) = (- tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-( tx_LF_FOOT * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF)+(((- tx_knee_joint_LF * sin_q_elbow_joint_LF)- tx_hip_joint_LF) * cos_q_hip_joint_LF);
    (*this)(3,3) = (cos_q_elbow_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_knee_joint_LF);
    (*this)(3,4) = (cos_q_elbow_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_knee_joint_LF);
    (*this)(4,3) = (cos_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF);
    (*this)(4,4) = (-cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(4,5) = -cos_q_hip_joint_LF;
    (*this)(5,3) = (sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-(cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(5,4) = (cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(5,5) = -sin_q_hip_joint_LF;
    return *this;
}
ForceTransforms::Type_fr_base_X_RF_FOOT::Type_fr_base_X_RF_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_RF_FOOT& ForceTransforms::Type_fr_base_X_RF_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    Scalar sin_q_knee_joint_RF  = ScalarTraits::sin( q(KNEE_JOINT_RF) );
    Scalar cos_q_knee_joint_RF  = ScalarTraits::cos( q(KNEE_JOINT_RF) );
    (*this)(0,0) = (cos_q_elbow_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_knee_joint_RF);
    (*this)(0,1) = (cos_q_elbow_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_knee_joint_RF);
    (*this)(0,3) = ( ty_hip_joint_RF * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-( ty_hip_joint_RF * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(0,4) = ( ty_hip_joint_RF * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+( ty_hip_joint_RF * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(0,5) = ( tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_knee_joint_RF)-( tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_knee_joint_RF)-( ty_hip_joint_RF * sin_q_hip_joint_RF)-( tx_knee_joint_RF * cos_q_elbow_joint_RF)- tx_elbow_joint_RF;
    (*this)(1,0) = (cos_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF);
    (*this)(1,1) = (-cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(1,3) = (((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+((( tx_hip_joint_RF * cos_q_elbow_joint_RF)-( tx_elbow_joint_RF * sin_q_elbow_joint_RF)) * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(1,4) = ((( tx_elbow_joint_RF * sin_q_elbow_joint_RF)-( tx_hip_joint_RF * cos_q_elbow_joint_RF)) * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+(((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * cos_q_hip_joint_RF * cos_q_knee_joint_RF)-( tx_RF_FOOT * cos_q_hip_joint_RF);
    (*this)(1,5) = ( tx_RF_FOOT * cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)+((( tx_knee_joint_RF * sin_q_elbow_joint_RF)+ tx_hip_joint_RF) * sin_q_hip_joint_RF);
    (*this)(2,0) = (sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-(cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(2,1) = (cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(2,3) = (((((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * cos_q_elbow_joint_RF)) * sin_q_knee_joint_RF)+((((( tx_hip_joint_RF * cos_q_elbow_joint_RF)-( tx_elbow_joint_RF * sin_q_elbow_joint_RF)) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * sin_q_elbow_joint_RF)) * cos_q_knee_joint_RF);
    (*this)(2,4) = ((((( tx_elbow_joint_RF * sin_q_elbow_joint_RF)-( tx_hip_joint_RF * cos_q_elbow_joint_RF)) * sin_q_hip_joint_RF)+( ty_hip_joint_RF * sin_q_elbow_joint_RF)) * sin_q_knee_joint_RF)+(((((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * cos_q_elbow_joint_RF)) * cos_q_knee_joint_RF)-( tx_RF_FOOT * sin_q_hip_joint_RF);
    (*this)(2,5) = (- tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-( tx_RF_FOOT * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF)+(((- tx_knee_joint_RF * sin_q_elbow_joint_RF)- tx_hip_joint_RF) * cos_q_hip_joint_RF);
    (*this)(3,3) = (cos_q_elbow_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_knee_joint_RF);
    (*this)(3,4) = (cos_q_elbow_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_knee_joint_RF);
    (*this)(4,3) = (cos_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF);
    (*this)(4,4) = (-cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(4,5) = -cos_q_hip_joint_RF;
    (*this)(5,3) = (sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-(cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(5,4) = (cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(5,5) = -sin_q_hip_joint_RF;
    return *this;
}
ForceTransforms::Type_fr_base_X_LH_FOOT::Type_fr_base_X_LH_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_LH_FOOT& ForceTransforms::Type_fr_base_X_LH_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    Scalar sin_q_knee_joint_LH  = ScalarTraits::sin( q(KNEE_JOINT_LH) );
    Scalar cos_q_knee_joint_LH  = ScalarTraits::cos( q(KNEE_JOINT_LH) );
    (*this)(0,0) = (cos_q_elbow_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_knee_joint_LH);
    (*this)(0,1) = (cos_q_elbow_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_knee_joint_LH);
    (*this)(0,3) = ( ty_hip_joint_LH * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-( ty_hip_joint_LH * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(0,4) = ( ty_hip_joint_LH * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+( ty_hip_joint_LH * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(0,5) = ( tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_knee_joint_LH)-( tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_knee_joint_LH)-( ty_hip_joint_LH * sin_q_hip_joint_LH)-( tx_knee_joint_LH * cos_q_elbow_joint_LH)- tx_elbow_joint_LH;
    (*this)(1,0) = (cos_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH);
    (*this)(1,1) = (-cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(1,3) = (((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+((( tx_hip_joint_LH * cos_q_elbow_joint_LH)-( tx_elbow_joint_LH * sin_q_elbow_joint_LH)) * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(1,4) = ((( tx_elbow_joint_LH * sin_q_elbow_joint_LH)-( tx_hip_joint_LH * cos_q_elbow_joint_LH)) * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+(((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * cos_q_hip_joint_LH * cos_q_knee_joint_LH)-( tx_LH_FOOT * cos_q_hip_joint_LH);
    (*this)(1,5) = ( tx_LH_FOOT * cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)+((( tx_knee_joint_LH * sin_q_elbow_joint_LH)+ tx_hip_joint_LH) * sin_q_hip_joint_LH);
    (*this)(2,0) = (sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-(cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(2,1) = (cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(2,3) = (((((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * cos_q_elbow_joint_LH)) * sin_q_knee_joint_LH)+((((( tx_hip_joint_LH * cos_q_elbow_joint_LH)-( tx_elbow_joint_LH * sin_q_elbow_joint_LH)) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * sin_q_elbow_joint_LH)) * cos_q_knee_joint_LH);
    (*this)(2,4) = ((((( tx_elbow_joint_LH * sin_q_elbow_joint_LH)-( tx_hip_joint_LH * cos_q_elbow_joint_LH)) * sin_q_hip_joint_LH)+( ty_hip_joint_LH * sin_q_elbow_joint_LH)) * sin_q_knee_joint_LH)+(((((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * cos_q_elbow_joint_LH)) * cos_q_knee_joint_LH)-( tx_LH_FOOT * sin_q_hip_joint_LH);
    (*this)(2,5) = (- tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-( tx_LH_FOOT * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH)+(((- tx_knee_joint_LH * sin_q_elbow_joint_LH)- tx_hip_joint_LH) * cos_q_hip_joint_LH);
    (*this)(3,3) = (cos_q_elbow_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_knee_joint_LH);
    (*this)(3,4) = (cos_q_elbow_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_knee_joint_LH);
    (*this)(4,3) = (cos_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH);
    (*this)(4,4) = (-cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(4,5) = -cos_q_hip_joint_LH;
    (*this)(5,3) = (sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-(cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(5,4) = (cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(5,5) = -sin_q_hip_joint_LH;
    return *this;
}
ForceTransforms::Type_fr_base_X_RH_FOOT::Type_fr_base_X_RH_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_RH_FOOT& ForceTransforms::Type_fr_base_X_RH_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    Scalar sin_q_knee_joint_RH  = ScalarTraits::sin( q(KNEE_JOINT_RH) );
    Scalar cos_q_knee_joint_RH  = ScalarTraits::cos( q(KNEE_JOINT_RH) );
    (*this)(0,0) = (cos_q_elbow_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_knee_joint_RH);
    (*this)(0,1) = (cos_q_elbow_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_knee_joint_RH);
    (*this)(0,3) = ( ty_hip_joint_RH * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-( ty_hip_joint_RH * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(0,4) = ( ty_hip_joint_RH * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+( ty_hip_joint_RH * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(0,5) = ( tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_knee_joint_RH)-( tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_knee_joint_RH)-( ty_hip_joint_RH * sin_q_hip_joint_RH)-( tx_knee_joint_RH * cos_q_elbow_joint_RH)- tx_elbow_joint_RH;
    (*this)(1,0) = (cos_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH);
    (*this)(1,1) = (-cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(1,3) = (((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+((( tx_hip_joint_RH * cos_q_elbow_joint_RH)-( tx_elbow_joint_RH * sin_q_elbow_joint_RH)) * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(1,4) = ((( tx_elbow_joint_RH * sin_q_elbow_joint_RH)-( tx_hip_joint_RH * cos_q_elbow_joint_RH)) * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+(((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * cos_q_hip_joint_RH * cos_q_knee_joint_RH)-( tx_RH_FOOT * cos_q_hip_joint_RH);
    (*this)(1,5) = ( tx_RH_FOOT * cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)+((( tx_knee_joint_RH * sin_q_elbow_joint_RH)+ tx_hip_joint_RH) * sin_q_hip_joint_RH);
    (*this)(2,0) = (sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-(cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(2,1) = (cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(2,3) = (((((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * cos_q_elbow_joint_RH)) * sin_q_knee_joint_RH)+((((( tx_hip_joint_RH * cos_q_elbow_joint_RH)-( tx_elbow_joint_RH * sin_q_elbow_joint_RH)) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * sin_q_elbow_joint_RH)) * cos_q_knee_joint_RH);
    (*this)(2,4) = ((((( tx_elbow_joint_RH * sin_q_elbow_joint_RH)-( tx_hip_joint_RH * cos_q_elbow_joint_RH)) * sin_q_hip_joint_RH)+( ty_hip_joint_RH * sin_q_elbow_joint_RH)) * sin_q_knee_joint_RH)+(((((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * cos_q_elbow_joint_RH)) * cos_q_knee_joint_RH)-( tx_RH_FOOT * sin_q_hip_joint_RH);
    (*this)(2,5) = (- tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-( tx_RH_FOOT * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH)+(((- tx_knee_joint_RH * sin_q_elbow_joint_RH)- tx_hip_joint_RH) * cos_q_hip_joint_RH);
    (*this)(3,3) = (cos_q_elbow_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_knee_joint_RH);
    (*this)(3,4) = (cos_q_elbow_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_knee_joint_RH);
    (*this)(4,3) = (cos_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH);
    (*this)(4,4) = (-cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(4,5) = -cos_q_hip_joint_RH;
    (*this)(5,3) = (sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-(cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(5,4) = (cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(5,5) = -sin_q_hip_joint_RH;
    return *this;
}
ForceTransforms::Type_imu_link_X_LF_FOOT::Type_imu_link_X_LF_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_imu_link_X_LF_FOOT& ForceTransforms::Type_imu_link_X_LF_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    Scalar sin_q_knee_joint_LF  = ScalarTraits::sin( q(KNEE_JOINT_LF) );
    Scalar cos_q_knee_joint_LF  = ScalarTraits::cos( q(KNEE_JOINT_LF) );
    (*this)(0,0) = (cos_q_elbow_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_knee_joint_LF);
    (*this)(0,1) = (cos_q_elbow_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_knee_joint_LF);
    (*this)(0,3) = ( ty_hip_joint_LF * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-( ty_hip_joint_LF * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(0,4) = ( ty_hip_joint_LF * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+( ty_hip_joint_LF * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(0,5) = ( tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_knee_joint_LF)-( tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_knee_joint_LF)-( ty_hip_joint_LF * sin_q_hip_joint_LF)-( tx_knee_joint_LF * cos_q_elbow_joint_LF)- tx_elbow_joint_LF;
    (*this)(1,0) = (cos_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF);
    (*this)(1,1) = (-cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(1,3) = (((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+((( tx_hip_joint_LF * cos_q_elbow_joint_LF)-( tx_elbow_joint_LF * sin_q_elbow_joint_LF)) * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(1,4) = ((( tx_elbow_joint_LF * sin_q_elbow_joint_LF)-( tx_hip_joint_LF * cos_q_elbow_joint_LF)) * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+(((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * cos_q_hip_joint_LF * cos_q_knee_joint_LF)-( tx_LF_FOOT * cos_q_hip_joint_LF);
    (*this)(1,5) = ( tx_LF_FOOT * cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)+((( tx_knee_joint_LF * sin_q_elbow_joint_LF)+ tx_hip_joint_LF) * sin_q_hip_joint_LF);
    (*this)(2,0) = (sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-(cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(2,1) = (cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(2,3) = (((((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * cos_q_elbow_joint_LF)) * sin_q_knee_joint_LF)+((((( tx_hip_joint_LF * cos_q_elbow_joint_LF)-( tx_elbow_joint_LF * sin_q_elbow_joint_LF)) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * sin_q_elbow_joint_LF)) * cos_q_knee_joint_LF);
    (*this)(2,4) = ((((( tx_elbow_joint_LF * sin_q_elbow_joint_LF)-( tx_hip_joint_LF * cos_q_elbow_joint_LF)) * sin_q_hip_joint_LF)+( ty_hip_joint_LF * sin_q_elbow_joint_LF)) * sin_q_knee_joint_LF)+(((((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * cos_q_elbow_joint_LF)) * cos_q_knee_joint_LF)-( tx_LF_FOOT * sin_q_hip_joint_LF);
    (*this)(2,5) = (- tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-( tx_LF_FOOT * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF)+(((- tx_knee_joint_LF * sin_q_elbow_joint_LF)- tx_hip_joint_LF) * cos_q_hip_joint_LF);
    (*this)(3,3) = (cos_q_elbow_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_knee_joint_LF);
    (*this)(3,4) = (cos_q_elbow_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_knee_joint_LF);
    (*this)(4,3) = (cos_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF);
    (*this)(4,4) = (-cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(4,5) = -cos_q_hip_joint_LF;
    (*this)(5,3) = (sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-(cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(5,4) = (cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(5,5) = -sin_q_hip_joint_LF;
    return *this;
}
ForceTransforms::Type_imu_link_X_RF_FOOT::Type_imu_link_X_RF_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_imu_link_X_RF_FOOT& ForceTransforms::Type_imu_link_X_RF_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    Scalar sin_q_knee_joint_RF  = ScalarTraits::sin( q(KNEE_JOINT_RF) );
    Scalar cos_q_knee_joint_RF  = ScalarTraits::cos( q(KNEE_JOINT_RF) );
    (*this)(0,0) = (cos_q_elbow_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_knee_joint_RF);
    (*this)(0,1) = (cos_q_elbow_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_knee_joint_RF);
    (*this)(0,3) = ( ty_hip_joint_RF * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-( ty_hip_joint_RF * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(0,4) = ( ty_hip_joint_RF * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+( ty_hip_joint_RF * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(0,5) = ( tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_knee_joint_RF)-( tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_knee_joint_RF)-( ty_hip_joint_RF * sin_q_hip_joint_RF)-( tx_knee_joint_RF * cos_q_elbow_joint_RF)- tx_elbow_joint_RF;
    (*this)(1,0) = (cos_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF);
    (*this)(1,1) = (-cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(1,3) = (((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+((( tx_hip_joint_RF * cos_q_elbow_joint_RF)-( tx_elbow_joint_RF * sin_q_elbow_joint_RF)) * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(1,4) = ((( tx_elbow_joint_RF * sin_q_elbow_joint_RF)-( tx_hip_joint_RF * cos_q_elbow_joint_RF)) * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+(((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * cos_q_hip_joint_RF * cos_q_knee_joint_RF)-( tx_RF_FOOT * cos_q_hip_joint_RF);
    (*this)(1,5) = ( tx_RF_FOOT * cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)+((( tx_knee_joint_RF * sin_q_elbow_joint_RF)+ tx_hip_joint_RF) * sin_q_hip_joint_RF);
    (*this)(2,0) = (sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-(cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(2,1) = (cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(2,3) = (((((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * cos_q_elbow_joint_RF)) * sin_q_knee_joint_RF)+((((( tx_hip_joint_RF * cos_q_elbow_joint_RF)-( tx_elbow_joint_RF * sin_q_elbow_joint_RF)) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * sin_q_elbow_joint_RF)) * cos_q_knee_joint_RF);
    (*this)(2,4) = ((((( tx_elbow_joint_RF * sin_q_elbow_joint_RF)-( tx_hip_joint_RF * cos_q_elbow_joint_RF)) * sin_q_hip_joint_RF)+( ty_hip_joint_RF * sin_q_elbow_joint_RF)) * sin_q_knee_joint_RF)+(((((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * cos_q_elbow_joint_RF)) * cos_q_knee_joint_RF)-( tx_RF_FOOT * sin_q_hip_joint_RF);
    (*this)(2,5) = (- tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-( tx_RF_FOOT * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF)+(((- tx_knee_joint_RF * sin_q_elbow_joint_RF)- tx_hip_joint_RF) * cos_q_hip_joint_RF);
    (*this)(3,3) = (cos_q_elbow_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_knee_joint_RF);
    (*this)(3,4) = (cos_q_elbow_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_knee_joint_RF);
    (*this)(4,3) = (cos_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF);
    (*this)(4,4) = (-cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(4,5) = -cos_q_hip_joint_RF;
    (*this)(5,3) = (sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-(cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(5,4) = (cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(5,5) = -sin_q_hip_joint_RF;
    return *this;
}
ForceTransforms::Type_imu_link_X_LH_FOOT::Type_imu_link_X_LH_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_imu_link_X_LH_FOOT& ForceTransforms::Type_imu_link_X_LH_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    Scalar sin_q_knee_joint_LH  = ScalarTraits::sin( q(KNEE_JOINT_LH) );
    Scalar cos_q_knee_joint_LH  = ScalarTraits::cos( q(KNEE_JOINT_LH) );
    (*this)(0,0) = (cos_q_elbow_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_knee_joint_LH);
    (*this)(0,1) = (cos_q_elbow_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_knee_joint_LH);
    (*this)(0,3) = ( ty_hip_joint_LH * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-( ty_hip_joint_LH * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(0,4) = ( ty_hip_joint_LH * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+( ty_hip_joint_LH * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(0,5) = ( tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_knee_joint_LH)-( tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_knee_joint_LH)-( ty_hip_joint_LH * sin_q_hip_joint_LH)-( tx_knee_joint_LH * cos_q_elbow_joint_LH)- tx_elbow_joint_LH;
    (*this)(1,0) = (cos_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH);
    (*this)(1,1) = (-cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(1,3) = (((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+((( tx_hip_joint_LH * cos_q_elbow_joint_LH)-( tx_elbow_joint_LH * sin_q_elbow_joint_LH)) * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(1,4) = ((( tx_elbow_joint_LH * sin_q_elbow_joint_LH)-( tx_hip_joint_LH * cos_q_elbow_joint_LH)) * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+(((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * cos_q_hip_joint_LH * cos_q_knee_joint_LH)-( tx_LH_FOOT * cos_q_hip_joint_LH);
    (*this)(1,5) = ( tx_LH_FOOT * cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)+((( tx_knee_joint_LH * sin_q_elbow_joint_LH)+ tx_hip_joint_LH) * sin_q_hip_joint_LH);
    (*this)(2,0) = (sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-(cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(2,1) = (cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(2,3) = (((((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * cos_q_elbow_joint_LH)) * sin_q_knee_joint_LH)+((((( tx_hip_joint_LH * cos_q_elbow_joint_LH)-( tx_elbow_joint_LH * sin_q_elbow_joint_LH)) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * sin_q_elbow_joint_LH)) * cos_q_knee_joint_LH);
    (*this)(2,4) = ((((( tx_elbow_joint_LH * sin_q_elbow_joint_LH)-( tx_hip_joint_LH * cos_q_elbow_joint_LH)) * sin_q_hip_joint_LH)+( ty_hip_joint_LH * sin_q_elbow_joint_LH)) * sin_q_knee_joint_LH)+(((((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * cos_q_elbow_joint_LH)) * cos_q_knee_joint_LH)-( tx_LH_FOOT * sin_q_hip_joint_LH);
    (*this)(2,5) = (- tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-( tx_LH_FOOT * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH)+(((- tx_knee_joint_LH * sin_q_elbow_joint_LH)- tx_hip_joint_LH) * cos_q_hip_joint_LH);
    (*this)(3,3) = (cos_q_elbow_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_knee_joint_LH);
    (*this)(3,4) = (cos_q_elbow_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_knee_joint_LH);
    (*this)(4,3) = (cos_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH);
    (*this)(4,4) = (-cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(4,5) = -cos_q_hip_joint_LH;
    (*this)(5,3) = (sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-(cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(5,4) = (cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(5,5) = -sin_q_hip_joint_LH;
    return *this;
}
ForceTransforms::Type_imu_link_X_RH_FOOT::Type_imu_link_X_RH_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_imu_link_X_RH_FOOT& ForceTransforms::Type_imu_link_X_RH_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    Scalar sin_q_knee_joint_RH  = ScalarTraits::sin( q(KNEE_JOINT_RH) );
    Scalar cos_q_knee_joint_RH  = ScalarTraits::cos( q(KNEE_JOINT_RH) );
    (*this)(0,0) = (cos_q_elbow_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_knee_joint_RH);
    (*this)(0,1) = (cos_q_elbow_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_knee_joint_RH);
    (*this)(0,3) = ( ty_hip_joint_RH * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-( ty_hip_joint_RH * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(0,4) = ( ty_hip_joint_RH * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+( ty_hip_joint_RH * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(0,5) = ( tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_knee_joint_RH)-( tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_knee_joint_RH)-( ty_hip_joint_RH * sin_q_hip_joint_RH)-( tx_knee_joint_RH * cos_q_elbow_joint_RH)- tx_elbow_joint_RH;
    (*this)(1,0) = (cos_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH);
    (*this)(1,1) = (-cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(1,3) = (((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+((( tx_hip_joint_RH * cos_q_elbow_joint_RH)-( tx_elbow_joint_RH * sin_q_elbow_joint_RH)) * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(1,4) = ((( tx_elbow_joint_RH * sin_q_elbow_joint_RH)-( tx_hip_joint_RH * cos_q_elbow_joint_RH)) * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+(((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * cos_q_hip_joint_RH * cos_q_knee_joint_RH)-( tx_RH_FOOT * cos_q_hip_joint_RH);
    (*this)(1,5) = ( tx_RH_FOOT * cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)+((( tx_knee_joint_RH * sin_q_elbow_joint_RH)+ tx_hip_joint_RH) * sin_q_hip_joint_RH);
    (*this)(2,0) = (sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-(cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(2,1) = (cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(2,3) = (((((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * cos_q_elbow_joint_RH)) * sin_q_knee_joint_RH)+((((( tx_hip_joint_RH * cos_q_elbow_joint_RH)-( tx_elbow_joint_RH * sin_q_elbow_joint_RH)) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * sin_q_elbow_joint_RH)) * cos_q_knee_joint_RH);
    (*this)(2,4) = ((((( tx_elbow_joint_RH * sin_q_elbow_joint_RH)-( tx_hip_joint_RH * cos_q_elbow_joint_RH)) * sin_q_hip_joint_RH)+( ty_hip_joint_RH * sin_q_elbow_joint_RH)) * sin_q_knee_joint_RH)+(((((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * cos_q_elbow_joint_RH)) * cos_q_knee_joint_RH)-( tx_RH_FOOT * sin_q_hip_joint_RH);
    (*this)(2,5) = (- tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-( tx_RH_FOOT * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH)+(((- tx_knee_joint_RH * sin_q_elbow_joint_RH)- tx_hip_joint_RH) * cos_q_hip_joint_RH);
    (*this)(3,3) = (cos_q_elbow_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_knee_joint_RH);
    (*this)(3,4) = (cos_q_elbow_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_knee_joint_RH);
    (*this)(4,3) = (cos_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH);
    (*this)(4,4) = (-cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(4,5) = -cos_q_hip_joint_RH;
    (*this)(5,3) = (sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-(cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(5,4) = (cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(5,5) = -sin_q_hip_joint_RH;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_hip_joint_LF::Type_fr_base_X_fr_hip_joint_LF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - ty_hip_joint_LF;    // Maxima DSL: -_k__ty_hip_joint_LF
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tx_hip_joint_LF;    // Maxima DSL: _k__tx_hip_joint_LF
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tx_hip_joint_LF;    // Maxima DSL: _k__tx_hip_joint_LF
    (*this)(2,5) = - ty_hip_joint_LF;    // Maxima DSL: -_k__ty_hip_joint_LF
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_hip_joint_LF& ForceTransforms::Type_fr_base_X_fr_hip_joint_LF::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_elbow_joint_LF::Type_fr_base_X_fr_elbow_joint_LF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_elbow_joint_LF& ForceTransforms::Type_fr_base_X_fr_elbow_joint_LF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    (*this)(0,3) = - ty_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(0,5) = (- ty_hip_joint_LF * sin_q_hip_joint_LF)- tx_elbow_joint_LF;
    (*this)(1,0) = sin_q_hip_joint_LF;
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(1,3) =  tx_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(1,4) = - tx_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(1,5) =  tx_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(2,0) = -cos_q_hip_joint_LF;
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(2,3) =  tx_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(2,4) = (- tx_elbow_joint_LF * sin_q_hip_joint_LF)- ty_hip_joint_LF;
    (*this)(2,5) = - tx_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(4,3) = sin_q_hip_joint_LF;
    (*this)(4,5) = -cos_q_hip_joint_LF;
    (*this)(5,3) = -cos_q_hip_joint_LF;
    (*this)(5,5) = -sin_q_hip_joint_LF;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_knee_joint_LF::Type_fr_base_X_fr_knee_joint_LF()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_knee_joint_LF& ForceTransforms::Type_fr_base_X_fr_knee_joint_LF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    (*this)(0,0) = sin_q_elbow_joint_LF;
    (*this)(0,1) = cos_q_elbow_joint_LF;
    (*this)(0,3) = - ty_hip_joint_LF * cos_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(0,4) =  ty_hip_joint_LF * sin_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(0,5) = (- ty_hip_joint_LF * sin_q_hip_joint_LF)-( tx_knee_joint_LF * cos_q_elbow_joint_LF)- tx_elbow_joint_LF;
    (*this)(1,0) = cos_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,1) = -sin_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(1,3) = (( tx_hip_joint_LF * cos_q_elbow_joint_LF)-( tx_elbow_joint_LF * sin_q_elbow_joint_LF)) * cos_q_hip_joint_LF;
    (*this)(1,4) = ((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * cos_q_hip_joint_LF;
    (*this)(1,5) = (( tx_knee_joint_LF * sin_q_elbow_joint_LF)+ tx_hip_joint_LF) * sin_q_hip_joint_LF;
    (*this)(2,0) = -cos_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(2,1) = sin_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(2,3) = ((( tx_hip_joint_LF * cos_q_elbow_joint_LF)-( tx_elbow_joint_LF * sin_q_elbow_joint_LF)) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * sin_q_elbow_joint_LF);
    (*this)(2,4) = (((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * cos_q_elbow_joint_LF);
    (*this)(2,5) = ((- tx_knee_joint_LF * sin_q_elbow_joint_LF)- tx_hip_joint_LF) * cos_q_hip_joint_LF;
    (*this)(3,3) = sin_q_elbow_joint_LF;
    (*this)(3,4) = cos_q_elbow_joint_LF;
    (*this)(4,3) = cos_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(4,4) = -sin_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(4,5) = -cos_q_hip_joint_LF;
    (*this)(5,3) = -cos_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(5,4) = sin_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(5,5) = -sin_q_hip_joint_LF;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_hip_joint_RF::Type_fr_base_X_fr_hip_joint_RF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - ty_hip_joint_RF;    // Maxima DSL: -_k__ty_hip_joint_RF
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tx_hip_joint_RF;    // Maxima DSL: _k__tx_hip_joint_RF
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tx_hip_joint_RF;    // Maxima DSL: _k__tx_hip_joint_RF
    (*this)(2,5) = - ty_hip_joint_RF;    // Maxima DSL: -_k__ty_hip_joint_RF
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_hip_joint_RF& ForceTransforms::Type_fr_base_X_fr_hip_joint_RF::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_elbow_joint_RF::Type_fr_base_X_fr_elbow_joint_RF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_elbow_joint_RF& ForceTransforms::Type_fr_base_X_fr_elbow_joint_RF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    (*this)(0,3) = - ty_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(0,5) = (- ty_hip_joint_RF * sin_q_hip_joint_RF)- tx_elbow_joint_RF;
    (*this)(1,0) = sin_q_hip_joint_RF;
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(1,3) =  tx_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(1,4) = - tx_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(1,5) =  tx_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(2,0) = -cos_q_hip_joint_RF;
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(2,3) =  tx_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(2,4) = (- tx_elbow_joint_RF * sin_q_hip_joint_RF)- ty_hip_joint_RF;
    (*this)(2,5) = - tx_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(4,3) = sin_q_hip_joint_RF;
    (*this)(4,5) = -cos_q_hip_joint_RF;
    (*this)(5,3) = -cos_q_hip_joint_RF;
    (*this)(5,5) = -sin_q_hip_joint_RF;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_knee_joint_RF::Type_fr_base_X_fr_knee_joint_RF()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_knee_joint_RF& ForceTransforms::Type_fr_base_X_fr_knee_joint_RF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    (*this)(0,0) = sin_q_elbow_joint_RF;
    (*this)(0,1) = cos_q_elbow_joint_RF;
    (*this)(0,3) = - ty_hip_joint_RF * cos_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(0,4) =  ty_hip_joint_RF * sin_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(0,5) = (- ty_hip_joint_RF * sin_q_hip_joint_RF)-( tx_knee_joint_RF * cos_q_elbow_joint_RF)- tx_elbow_joint_RF;
    (*this)(1,0) = cos_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,1) = -sin_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(1,3) = (( tx_hip_joint_RF * cos_q_elbow_joint_RF)-( tx_elbow_joint_RF * sin_q_elbow_joint_RF)) * cos_q_hip_joint_RF;
    (*this)(1,4) = ((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * cos_q_hip_joint_RF;
    (*this)(1,5) = (( tx_knee_joint_RF * sin_q_elbow_joint_RF)+ tx_hip_joint_RF) * sin_q_hip_joint_RF;
    (*this)(2,0) = -cos_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(2,1) = sin_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(2,3) = ((( tx_hip_joint_RF * cos_q_elbow_joint_RF)-( tx_elbow_joint_RF * sin_q_elbow_joint_RF)) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * sin_q_elbow_joint_RF);
    (*this)(2,4) = (((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * cos_q_elbow_joint_RF);
    (*this)(2,5) = ((- tx_knee_joint_RF * sin_q_elbow_joint_RF)- tx_hip_joint_RF) * cos_q_hip_joint_RF;
    (*this)(3,3) = sin_q_elbow_joint_RF;
    (*this)(3,4) = cos_q_elbow_joint_RF;
    (*this)(4,3) = cos_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(4,4) = -sin_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(4,5) = -cos_q_hip_joint_RF;
    (*this)(5,3) = -cos_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(5,4) = sin_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(5,5) = -sin_q_hip_joint_RF;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_hip_joint_LH::Type_fr_base_X_fr_hip_joint_LH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - ty_hip_joint_LH;    // Maxima DSL: -_k__ty_hip_joint_LH
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tx_hip_joint_LH;    // Maxima DSL: _k__tx_hip_joint_LH
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tx_hip_joint_LH;    // Maxima DSL: _k__tx_hip_joint_LH
    (*this)(2,5) = - ty_hip_joint_LH;    // Maxima DSL: -_k__ty_hip_joint_LH
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_hip_joint_LH& ForceTransforms::Type_fr_base_X_fr_hip_joint_LH::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_elbow_joint_LH::Type_fr_base_X_fr_elbow_joint_LH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_elbow_joint_LH& ForceTransforms::Type_fr_base_X_fr_elbow_joint_LH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    (*this)(0,3) = - ty_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(0,5) = (- ty_hip_joint_LH * sin_q_hip_joint_LH)- tx_elbow_joint_LH;
    (*this)(1,0) = sin_q_hip_joint_LH;
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(1,3) =  tx_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(1,4) = - tx_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(1,5) =  tx_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(2,0) = -cos_q_hip_joint_LH;
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(2,3) =  tx_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(2,4) = (- tx_elbow_joint_LH * sin_q_hip_joint_LH)- ty_hip_joint_LH;
    (*this)(2,5) = - tx_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(4,3) = sin_q_hip_joint_LH;
    (*this)(4,5) = -cos_q_hip_joint_LH;
    (*this)(5,3) = -cos_q_hip_joint_LH;
    (*this)(5,5) = -sin_q_hip_joint_LH;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_knee_joint_LH::Type_fr_base_X_fr_knee_joint_LH()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_knee_joint_LH& ForceTransforms::Type_fr_base_X_fr_knee_joint_LH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    (*this)(0,0) = sin_q_elbow_joint_LH;
    (*this)(0,1) = cos_q_elbow_joint_LH;
    (*this)(0,3) = - ty_hip_joint_LH * cos_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(0,4) =  ty_hip_joint_LH * sin_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(0,5) = (- ty_hip_joint_LH * sin_q_hip_joint_LH)-( tx_knee_joint_LH * cos_q_elbow_joint_LH)- tx_elbow_joint_LH;
    (*this)(1,0) = cos_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,1) = -sin_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(1,3) = (( tx_hip_joint_LH * cos_q_elbow_joint_LH)-( tx_elbow_joint_LH * sin_q_elbow_joint_LH)) * cos_q_hip_joint_LH;
    (*this)(1,4) = ((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * cos_q_hip_joint_LH;
    (*this)(1,5) = (( tx_knee_joint_LH * sin_q_elbow_joint_LH)+ tx_hip_joint_LH) * sin_q_hip_joint_LH;
    (*this)(2,0) = -cos_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(2,1) = sin_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(2,3) = ((( tx_hip_joint_LH * cos_q_elbow_joint_LH)-( tx_elbow_joint_LH * sin_q_elbow_joint_LH)) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * sin_q_elbow_joint_LH);
    (*this)(2,4) = (((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * cos_q_elbow_joint_LH);
    (*this)(2,5) = ((- tx_knee_joint_LH * sin_q_elbow_joint_LH)- tx_hip_joint_LH) * cos_q_hip_joint_LH;
    (*this)(3,3) = sin_q_elbow_joint_LH;
    (*this)(3,4) = cos_q_elbow_joint_LH;
    (*this)(4,3) = cos_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(4,4) = -sin_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(4,5) = -cos_q_hip_joint_LH;
    (*this)(5,3) = -cos_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(5,4) = sin_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(5,5) = -sin_q_hip_joint_LH;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_hip_joint_RH::Type_fr_base_X_fr_hip_joint_RH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - ty_hip_joint_RH;    // Maxima DSL: -_k__ty_hip_joint_RH
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tx_hip_joint_RH;    // Maxima DSL: _k__tx_hip_joint_RH
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tx_hip_joint_RH;    // Maxima DSL: _k__tx_hip_joint_RH
    (*this)(2,5) = - ty_hip_joint_RH;    // Maxima DSL: -_k__ty_hip_joint_RH
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_hip_joint_RH& ForceTransforms::Type_fr_base_X_fr_hip_joint_RH::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_elbow_joint_RH::Type_fr_base_X_fr_elbow_joint_RH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_elbow_joint_RH& ForceTransforms::Type_fr_base_X_fr_elbow_joint_RH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    (*this)(0,3) = - ty_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(0,5) = (- ty_hip_joint_RH * sin_q_hip_joint_RH)- tx_elbow_joint_RH;
    (*this)(1,0) = sin_q_hip_joint_RH;
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(1,3) =  tx_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(1,4) = - tx_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(1,5) =  tx_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(2,0) = -cos_q_hip_joint_RH;
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(2,3) =  tx_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(2,4) = (- tx_elbow_joint_RH * sin_q_hip_joint_RH)- ty_hip_joint_RH;
    (*this)(2,5) = - tx_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(4,3) = sin_q_hip_joint_RH;
    (*this)(4,5) = -cos_q_hip_joint_RH;
    (*this)(5,3) = -cos_q_hip_joint_RH;
    (*this)(5,5) = -sin_q_hip_joint_RH;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_knee_joint_RH::Type_fr_base_X_fr_knee_joint_RH()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_knee_joint_RH& ForceTransforms::Type_fr_base_X_fr_knee_joint_RH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    (*this)(0,0) = sin_q_elbow_joint_RH;
    (*this)(0,1) = cos_q_elbow_joint_RH;
    (*this)(0,3) = - ty_hip_joint_RH * cos_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(0,4) =  ty_hip_joint_RH * sin_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(0,5) = (- ty_hip_joint_RH * sin_q_hip_joint_RH)-( tx_knee_joint_RH * cos_q_elbow_joint_RH)- tx_elbow_joint_RH;
    (*this)(1,0) = cos_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,1) = -sin_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(1,3) = (( tx_hip_joint_RH * cos_q_elbow_joint_RH)-( tx_elbow_joint_RH * sin_q_elbow_joint_RH)) * cos_q_hip_joint_RH;
    (*this)(1,4) = ((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * cos_q_hip_joint_RH;
    (*this)(1,5) = (( tx_knee_joint_RH * sin_q_elbow_joint_RH)+ tx_hip_joint_RH) * sin_q_hip_joint_RH;
    (*this)(2,0) = -cos_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(2,1) = sin_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(2,3) = ((( tx_hip_joint_RH * cos_q_elbow_joint_RH)-( tx_elbow_joint_RH * sin_q_elbow_joint_RH)) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * sin_q_elbow_joint_RH);
    (*this)(2,4) = (((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * cos_q_elbow_joint_RH);
    (*this)(2,5) = ((- tx_knee_joint_RH * sin_q_elbow_joint_RH)- tx_hip_joint_RH) * cos_q_hip_joint_RH;
    (*this)(3,3) = sin_q_elbow_joint_RH;
    (*this)(3,4) = cos_q_elbow_joint_RH;
    (*this)(4,3) = cos_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(4,4) = -sin_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(4,5) = -cos_q_hip_joint_RH;
    (*this)(5,3) = -cos_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(5,4) = sin_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(5,5) = -sin_q_hip_joint_RH;
    return *this;
}
ForceTransforms::Type_imu_link_X_fr_hip_joint_LF::Type_imu_link_X_fr_hip_joint_LF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - ty_hip_joint_LF;    // Maxima DSL: -_k__ty_hip_joint_LF
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tx_hip_joint_LF;    // Maxima DSL: _k__tx_hip_joint_LF
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tx_hip_joint_LF;    // Maxima DSL: _k__tx_hip_joint_LF
    (*this)(2,5) = - ty_hip_joint_LF;    // Maxima DSL: -_k__ty_hip_joint_LF
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_imu_link_X_fr_hip_joint_LF& ForceTransforms::Type_imu_link_X_fr_hip_joint_LF::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_imu_link_X_fr_elbow_joint_LF::Type_imu_link_X_fr_elbow_joint_LF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_imu_link_X_fr_elbow_joint_LF& ForceTransforms::Type_imu_link_X_fr_elbow_joint_LF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    (*this)(0,3) = - ty_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(0,5) = (- ty_hip_joint_LF * sin_q_hip_joint_LF)- tx_elbow_joint_LF;
    (*this)(1,0) = sin_q_hip_joint_LF;
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(1,3) =  tx_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(1,4) = - tx_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(1,5) =  tx_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(2,0) = -cos_q_hip_joint_LF;
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(2,3) =  tx_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(2,4) = (- tx_elbow_joint_LF * sin_q_hip_joint_LF)- ty_hip_joint_LF;
    (*this)(2,5) = - tx_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(4,3) = sin_q_hip_joint_LF;
    (*this)(4,5) = -cos_q_hip_joint_LF;
    (*this)(5,3) = -cos_q_hip_joint_LF;
    (*this)(5,5) = -sin_q_hip_joint_LF;
    return *this;
}
ForceTransforms::Type_imu_link_X_fr_knee_joint_LF::Type_imu_link_X_fr_knee_joint_LF()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_imu_link_X_fr_knee_joint_LF& ForceTransforms::Type_imu_link_X_fr_knee_joint_LF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    (*this)(0,0) = sin_q_elbow_joint_LF;
    (*this)(0,1) = cos_q_elbow_joint_LF;
    (*this)(0,3) = - ty_hip_joint_LF * cos_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(0,4) =  ty_hip_joint_LF * sin_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(0,5) = (- ty_hip_joint_LF * sin_q_hip_joint_LF)-( tx_knee_joint_LF * cos_q_elbow_joint_LF)- tx_elbow_joint_LF;
    (*this)(1,0) = cos_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,1) = -sin_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(1,3) = (( tx_hip_joint_LF * cos_q_elbow_joint_LF)-( tx_elbow_joint_LF * sin_q_elbow_joint_LF)) * cos_q_hip_joint_LF;
    (*this)(1,4) = ((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * cos_q_hip_joint_LF;
    (*this)(1,5) = (( tx_knee_joint_LF * sin_q_elbow_joint_LF)+ tx_hip_joint_LF) * sin_q_hip_joint_LF;
    (*this)(2,0) = -cos_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(2,1) = sin_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(2,3) = ((( tx_hip_joint_LF * cos_q_elbow_joint_LF)-( tx_elbow_joint_LF * sin_q_elbow_joint_LF)) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * sin_q_elbow_joint_LF);
    (*this)(2,4) = (((- tx_hip_joint_LF * sin_q_elbow_joint_LF)-( tx_elbow_joint_LF * cos_q_elbow_joint_LF)- tx_knee_joint_LF) * sin_q_hip_joint_LF)-( ty_hip_joint_LF * cos_q_elbow_joint_LF);
    (*this)(2,5) = ((- tx_knee_joint_LF * sin_q_elbow_joint_LF)- tx_hip_joint_LF) * cos_q_hip_joint_LF;
    (*this)(3,3) = sin_q_elbow_joint_LF;
    (*this)(3,4) = cos_q_elbow_joint_LF;
    (*this)(4,3) = cos_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(4,4) = -sin_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(4,5) = -cos_q_hip_joint_LF;
    (*this)(5,3) = -cos_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(5,4) = sin_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(5,5) = -sin_q_hip_joint_LF;
    return *this;
}
ForceTransforms::Type_imu_link_X_fr_hip_joint_RF::Type_imu_link_X_fr_hip_joint_RF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - ty_hip_joint_RF;    // Maxima DSL: -_k__ty_hip_joint_RF
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tx_hip_joint_RF;    // Maxima DSL: _k__tx_hip_joint_RF
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tx_hip_joint_RF;    // Maxima DSL: _k__tx_hip_joint_RF
    (*this)(2,5) = - ty_hip_joint_RF;    // Maxima DSL: -_k__ty_hip_joint_RF
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_imu_link_X_fr_hip_joint_RF& ForceTransforms::Type_imu_link_X_fr_hip_joint_RF::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_imu_link_X_fr_elbow_joint_RF::Type_imu_link_X_fr_elbow_joint_RF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_imu_link_X_fr_elbow_joint_RF& ForceTransforms::Type_imu_link_X_fr_elbow_joint_RF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    (*this)(0,3) = - ty_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(0,5) = (- ty_hip_joint_RF * sin_q_hip_joint_RF)- tx_elbow_joint_RF;
    (*this)(1,0) = sin_q_hip_joint_RF;
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(1,3) =  tx_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(1,4) = - tx_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(1,5) =  tx_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(2,0) = -cos_q_hip_joint_RF;
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(2,3) =  tx_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(2,4) = (- tx_elbow_joint_RF * sin_q_hip_joint_RF)- ty_hip_joint_RF;
    (*this)(2,5) = - tx_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(4,3) = sin_q_hip_joint_RF;
    (*this)(4,5) = -cos_q_hip_joint_RF;
    (*this)(5,3) = -cos_q_hip_joint_RF;
    (*this)(5,5) = -sin_q_hip_joint_RF;
    return *this;
}
ForceTransforms::Type_imu_link_X_fr_knee_joint_RF::Type_imu_link_X_fr_knee_joint_RF()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_imu_link_X_fr_knee_joint_RF& ForceTransforms::Type_imu_link_X_fr_knee_joint_RF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    (*this)(0,0) = sin_q_elbow_joint_RF;
    (*this)(0,1) = cos_q_elbow_joint_RF;
    (*this)(0,3) = - ty_hip_joint_RF * cos_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(0,4) =  ty_hip_joint_RF * sin_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(0,5) = (- ty_hip_joint_RF * sin_q_hip_joint_RF)-( tx_knee_joint_RF * cos_q_elbow_joint_RF)- tx_elbow_joint_RF;
    (*this)(1,0) = cos_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,1) = -sin_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(1,3) = (( tx_hip_joint_RF * cos_q_elbow_joint_RF)-( tx_elbow_joint_RF * sin_q_elbow_joint_RF)) * cos_q_hip_joint_RF;
    (*this)(1,4) = ((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * cos_q_hip_joint_RF;
    (*this)(1,5) = (( tx_knee_joint_RF * sin_q_elbow_joint_RF)+ tx_hip_joint_RF) * sin_q_hip_joint_RF;
    (*this)(2,0) = -cos_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(2,1) = sin_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(2,3) = ((( tx_hip_joint_RF * cos_q_elbow_joint_RF)-( tx_elbow_joint_RF * sin_q_elbow_joint_RF)) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * sin_q_elbow_joint_RF);
    (*this)(2,4) = (((- tx_hip_joint_RF * sin_q_elbow_joint_RF)-( tx_elbow_joint_RF * cos_q_elbow_joint_RF)- tx_knee_joint_RF) * sin_q_hip_joint_RF)-( ty_hip_joint_RF * cos_q_elbow_joint_RF);
    (*this)(2,5) = ((- tx_knee_joint_RF * sin_q_elbow_joint_RF)- tx_hip_joint_RF) * cos_q_hip_joint_RF;
    (*this)(3,3) = sin_q_elbow_joint_RF;
    (*this)(3,4) = cos_q_elbow_joint_RF;
    (*this)(4,3) = cos_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(4,4) = -sin_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(4,5) = -cos_q_hip_joint_RF;
    (*this)(5,3) = -cos_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(5,4) = sin_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(5,5) = -sin_q_hip_joint_RF;
    return *this;
}
ForceTransforms::Type_imu_link_X_fr_hip_joint_LH::Type_imu_link_X_fr_hip_joint_LH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - ty_hip_joint_LH;    // Maxima DSL: -_k__ty_hip_joint_LH
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tx_hip_joint_LH;    // Maxima DSL: _k__tx_hip_joint_LH
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tx_hip_joint_LH;    // Maxima DSL: _k__tx_hip_joint_LH
    (*this)(2,5) = - ty_hip_joint_LH;    // Maxima DSL: -_k__ty_hip_joint_LH
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_imu_link_X_fr_hip_joint_LH& ForceTransforms::Type_imu_link_X_fr_hip_joint_LH::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_imu_link_X_fr_elbow_joint_LH::Type_imu_link_X_fr_elbow_joint_LH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_imu_link_X_fr_elbow_joint_LH& ForceTransforms::Type_imu_link_X_fr_elbow_joint_LH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    (*this)(0,3) = - ty_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(0,5) = (- ty_hip_joint_LH * sin_q_hip_joint_LH)- tx_elbow_joint_LH;
    (*this)(1,0) = sin_q_hip_joint_LH;
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(1,3) =  tx_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(1,4) = - tx_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(1,5) =  tx_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(2,0) = -cos_q_hip_joint_LH;
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(2,3) =  tx_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(2,4) = (- tx_elbow_joint_LH * sin_q_hip_joint_LH)- ty_hip_joint_LH;
    (*this)(2,5) = - tx_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(4,3) = sin_q_hip_joint_LH;
    (*this)(4,5) = -cos_q_hip_joint_LH;
    (*this)(5,3) = -cos_q_hip_joint_LH;
    (*this)(5,5) = -sin_q_hip_joint_LH;
    return *this;
}
ForceTransforms::Type_imu_link_X_fr_knee_joint_LH::Type_imu_link_X_fr_knee_joint_LH()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_imu_link_X_fr_knee_joint_LH& ForceTransforms::Type_imu_link_X_fr_knee_joint_LH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    (*this)(0,0) = sin_q_elbow_joint_LH;
    (*this)(0,1) = cos_q_elbow_joint_LH;
    (*this)(0,3) = - ty_hip_joint_LH * cos_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(0,4) =  ty_hip_joint_LH * sin_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(0,5) = (- ty_hip_joint_LH * sin_q_hip_joint_LH)-( tx_knee_joint_LH * cos_q_elbow_joint_LH)- tx_elbow_joint_LH;
    (*this)(1,0) = cos_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,1) = -sin_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(1,3) = (( tx_hip_joint_LH * cos_q_elbow_joint_LH)-( tx_elbow_joint_LH * sin_q_elbow_joint_LH)) * cos_q_hip_joint_LH;
    (*this)(1,4) = ((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * cos_q_hip_joint_LH;
    (*this)(1,5) = (( tx_knee_joint_LH * sin_q_elbow_joint_LH)+ tx_hip_joint_LH) * sin_q_hip_joint_LH;
    (*this)(2,0) = -cos_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(2,1) = sin_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(2,3) = ((( tx_hip_joint_LH * cos_q_elbow_joint_LH)-( tx_elbow_joint_LH * sin_q_elbow_joint_LH)) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * sin_q_elbow_joint_LH);
    (*this)(2,4) = (((- tx_hip_joint_LH * sin_q_elbow_joint_LH)-( tx_elbow_joint_LH * cos_q_elbow_joint_LH)- tx_knee_joint_LH) * sin_q_hip_joint_LH)-( ty_hip_joint_LH * cos_q_elbow_joint_LH);
    (*this)(2,5) = ((- tx_knee_joint_LH * sin_q_elbow_joint_LH)- tx_hip_joint_LH) * cos_q_hip_joint_LH;
    (*this)(3,3) = sin_q_elbow_joint_LH;
    (*this)(3,4) = cos_q_elbow_joint_LH;
    (*this)(4,3) = cos_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(4,4) = -sin_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(4,5) = -cos_q_hip_joint_LH;
    (*this)(5,3) = -cos_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(5,4) = sin_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(5,5) = -sin_q_hip_joint_LH;
    return *this;
}
ForceTransforms::Type_imu_link_X_fr_hip_joint_RH::Type_imu_link_X_fr_hip_joint_RH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - ty_hip_joint_RH;    // Maxima DSL: -_k__ty_hip_joint_RH
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tx_hip_joint_RH;    // Maxima DSL: _k__tx_hip_joint_RH
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tx_hip_joint_RH;    // Maxima DSL: _k__tx_hip_joint_RH
    (*this)(2,5) = - ty_hip_joint_RH;    // Maxima DSL: -_k__ty_hip_joint_RH
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_imu_link_X_fr_hip_joint_RH& ForceTransforms::Type_imu_link_X_fr_hip_joint_RH::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_imu_link_X_fr_elbow_joint_RH::Type_imu_link_X_fr_elbow_joint_RH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_imu_link_X_fr_elbow_joint_RH& ForceTransforms::Type_imu_link_X_fr_elbow_joint_RH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    (*this)(0,3) = - ty_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(0,5) = (- ty_hip_joint_RH * sin_q_hip_joint_RH)- tx_elbow_joint_RH;
    (*this)(1,0) = sin_q_hip_joint_RH;
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(1,3) =  tx_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(1,4) = - tx_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(1,5) =  tx_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(2,0) = -cos_q_hip_joint_RH;
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(2,3) =  tx_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(2,4) = (- tx_elbow_joint_RH * sin_q_hip_joint_RH)- ty_hip_joint_RH;
    (*this)(2,5) = - tx_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(4,3) = sin_q_hip_joint_RH;
    (*this)(4,5) = -cos_q_hip_joint_RH;
    (*this)(5,3) = -cos_q_hip_joint_RH;
    (*this)(5,5) = -sin_q_hip_joint_RH;
    return *this;
}
ForceTransforms::Type_imu_link_X_fr_knee_joint_RH::Type_imu_link_X_fr_knee_joint_RH()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_imu_link_X_fr_knee_joint_RH& ForceTransforms::Type_imu_link_X_fr_knee_joint_RH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    (*this)(0,0) = sin_q_elbow_joint_RH;
    (*this)(0,1) = cos_q_elbow_joint_RH;
    (*this)(0,3) = - ty_hip_joint_RH * cos_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(0,4) =  ty_hip_joint_RH * sin_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(0,5) = (- ty_hip_joint_RH * sin_q_hip_joint_RH)-( tx_knee_joint_RH * cos_q_elbow_joint_RH)- tx_elbow_joint_RH;
    (*this)(1,0) = cos_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,1) = -sin_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(1,3) = (( tx_hip_joint_RH * cos_q_elbow_joint_RH)-( tx_elbow_joint_RH * sin_q_elbow_joint_RH)) * cos_q_hip_joint_RH;
    (*this)(1,4) = ((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * cos_q_hip_joint_RH;
    (*this)(1,5) = (( tx_knee_joint_RH * sin_q_elbow_joint_RH)+ tx_hip_joint_RH) * sin_q_hip_joint_RH;
    (*this)(2,0) = -cos_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(2,1) = sin_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(2,3) = ((( tx_hip_joint_RH * cos_q_elbow_joint_RH)-( tx_elbow_joint_RH * sin_q_elbow_joint_RH)) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * sin_q_elbow_joint_RH);
    (*this)(2,4) = (((- tx_hip_joint_RH * sin_q_elbow_joint_RH)-( tx_elbow_joint_RH * cos_q_elbow_joint_RH)- tx_knee_joint_RH) * sin_q_hip_joint_RH)-( ty_hip_joint_RH * cos_q_elbow_joint_RH);
    (*this)(2,5) = ((- tx_knee_joint_RH * sin_q_elbow_joint_RH)- tx_hip_joint_RH) * cos_q_hip_joint_RH;
    (*this)(3,3) = sin_q_elbow_joint_RH;
    (*this)(3,4) = cos_q_elbow_joint_RH;
    (*this)(4,3) = cos_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(4,4) = -sin_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(4,5) = -cos_q_hip_joint_RH;
    (*this)(5,3) = -cos_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(5,4) = sin_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(5,5) = -sin_q_hip_joint_RH;
    return *this;
}
ForceTransforms::Type_fr_hip_link_LF_X_fr_base::Type_fr_hip_link_LF_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = - ty_hip_joint_LF;    // Maxima DSL: -_k__ty_hip_joint_LF
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_hip_link_LF_X_fr_base& ForceTransforms::Type_fr_hip_link_LF_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    (*this)(0,1) = sin_q_hip_joint_LF;
    (*this)(0,2) = -cos_q_hip_joint_LF;
    (*this)(0,3) = - ty_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(0,4) =  tx_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(0,5) =  tx_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,1) = cos_q_hip_joint_LF;
    (*this)(1,2) = sin_q_hip_joint_LF;
    (*this)(1,3) =  ty_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,4) = - tx_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,5) =  tx_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(3,4) = sin_q_hip_joint_LF;
    (*this)(3,5) = -cos_q_hip_joint_LF;
    (*this)(4,4) = cos_q_hip_joint_LF;
    (*this)(4,5) = sin_q_hip_joint_LF;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_hip_link_LF::Type_fr_base_X_fr_hip_link_LF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = - ty_hip_joint_LF;    // Maxima DSL: -_k__ty_hip_joint_LF
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_hip_link_LF& ForceTransforms::Type_fr_base_X_fr_hip_link_LF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    (*this)(0,3) = - ty_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(0,4) =  ty_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,0) = sin_q_hip_joint_LF;
    (*this)(1,1) = cos_q_hip_joint_LF;
    (*this)(1,3) =  tx_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(1,4) = - tx_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(2,0) = -cos_q_hip_joint_LF;
    (*this)(2,1) = sin_q_hip_joint_LF;
    (*this)(2,3) =  tx_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(2,4) =  tx_hip_joint_LF * cos_q_hip_joint_LF;
    (*this)(4,3) = sin_q_hip_joint_LF;
    (*this)(4,4) = cos_q_hip_joint_LF;
    (*this)(5,3) = -cos_q_hip_joint_LF;
    (*this)(5,4) = sin_q_hip_joint_LF;
    return *this;
}
ForceTransforms::Type_fr_thigh_link_LF_X_fr_hip_link_LF::Type_fr_thigh_link_LF_X_fr_hip_link_LF()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = - tx_elbow_joint_LF;    // Maxima DSL: -_k__tx_elbow_joint_LF
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_thigh_link_LF_X_fr_hip_link_LF& ForceTransforms::Type_fr_thigh_link_LF_X_fr_hip_link_LF::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    (*this)(0,0) = cos_q_elbow_joint_LF;
    (*this)(0,2) = sin_q_elbow_joint_LF;
    (*this)(0,4) = - tx_elbow_joint_LF * sin_q_elbow_joint_LF;
    (*this)(1,0) = -sin_q_elbow_joint_LF;
    (*this)(1,2) = cos_q_elbow_joint_LF;
    (*this)(1,4) = - tx_elbow_joint_LF * cos_q_elbow_joint_LF;
    (*this)(3,3) = cos_q_elbow_joint_LF;
    (*this)(3,5) = sin_q_elbow_joint_LF;
    (*this)(4,3) = -sin_q_elbow_joint_LF;
    (*this)(4,5) = cos_q_elbow_joint_LF;
    return *this;
}
ForceTransforms::Type_fr_hip_link_LF_X_fr_thigh_link_LF::Type_fr_hip_link_LF_X_fr_thigh_link_LF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = - tx_elbow_joint_LF;    // Maxima DSL: -_k__tx_elbow_joint_LF
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_hip_link_LF_X_fr_thigh_link_LF& ForceTransforms::Type_fr_hip_link_LF_X_fr_thigh_link_LF::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    (*this)(0,0) = cos_q_elbow_joint_LF;
    (*this)(0,1) = -sin_q_elbow_joint_LF;
    (*this)(1,3) = - tx_elbow_joint_LF * sin_q_elbow_joint_LF;
    (*this)(1,4) = - tx_elbow_joint_LF * cos_q_elbow_joint_LF;
    (*this)(2,0) = sin_q_elbow_joint_LF;
    (*this)(2,1) = cos_q_elbow_joint_LF;
    (*this)(3,3) = cos_q_elbow_joint_LF;
    (*this)(3,4) = -sin_q_elbow_joint_LF;
    (*this)(5,3) = sin_q_elbow_joint_LF;
    (*this)(5,4) = cos_q_elbow_joint_LF;
    return *this;
}
ForceTransforms::Type_fr_shank_link_LF_X_fr_thigh_link_LF::Type_fr_shank_link_LF_X_fr_thigh_link_LF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tx_knee_joint_LF;    // Maxima DSL: -_k__tx_knee_joint_LF
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_shank_link_LF_X_fr_thigh_link_LF& ForceTransforms::Type_fr_shank_link_LF_X_fr_thigh_link_LF::update(const state_t& q)
{
    Scalar sin_q_knee_joint_LF  = ScalarTraits::sin( q(KNEE_JOINT_LF) );
    Scalar cos_q_knee_joint_LF  = ScalarTraits::cos( q(KNEE_JOINT_LF) );
    (*this)(0,0) = cos_q_knee_joint_LF;
    (*this)(0,1) = sin_q_knee_joint_LF;
    (*this)(0,5) =  tx_knee_joint_LF * sin_q_knee_joint_LF;
    (*this)(1,0) = -sin_q_knee_joint_LF;
    (*this)(1,1) = cos_q_knee_joint_LF;
    (*this)(1,5) =  tx_knee_joint_LF * cos_q_knee_joint_LF;
    (*this)(3,3) = cos_q_knee_joint_LF;
    (*this)(3,4) = sin_q_knee_joint_LF;
    (*this)(4,3) = -sin_q_knee_joint_LF;
    (*this)(4,4) = cos_q_knee_joint_LF;
    return *this;
}
ForceTransforms::Type_fr_thigh_link_LF_X_fr_shank_link_LF::Type_fr_thigh_link_LF_X_fr_shank_link_LF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = - tx_knee_joint_LF;    // Maxima DSL: -_k__tx_knee_joint_LF
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_thigh_link_LF_X_fr_shank_link_LF& ForceTransforms::Type_fr_thigh_link_LF_X_fr_shank_link_LF::update(const state_t& q)
{
    Scalar sin_q_knee_joint_LF  = ScalarTraits::sin( q(KNEE_JOINT_LF) );
    Scalar cos_q_knee_joint_LF  = ScalarTraits::cos( q(KNEE_JOINT_LF) );
    (*this)(0,0) = cos_q_knee_joint_LF;
    (*this)(0,1) = -sin_q_knee_joint_LF;
    (*this)(1,0) = sin_q_knee_joint_LF;
    (*this)(1,1) = cos_q_knee_joint_LF;
    (*this)(2,3) =  tx_knee_joint_LF * sin_q_knee_joint_LF;
    (*this)(2,4) =  tx_knee_joint_LF * cos_q_knee_joint_LF;
    (*this)(3,3) = cos_q_knee_joint_LF;
    (*this)(3,4) = -sin_q_knee_joint_LF;
    (*this)(4,3) = sin_q_knee_joint_LF;
    (*this)(4,4) = cos_q_knee_joint_LF;
    return *this;
}
ForceTransforms::Type_fr_hip_link_RF_X_fr_base::Type_fr_hip_link_RF_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = - ty_hip_joint_RF;    // Maxima DSL: -_k__ty_hip_joint_RF
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_hip_link_RF_X_fr_base& ForceTransforms::Type_fr_hip_link_RF_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    (*this)(0,1) = sin_q_hip_joint_RF;
    (*this)(0,2) = -cos_q_hip_joint_RF;
    (*this)(0,3) = - ty_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(0,4) =  tx_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(0,5) =  tx_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,1) = cos_q_hip_joint_RF;
    (*this)(1,2) = sin_q_hip_joint_RF;
    (*this)(1,3) =  ty_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,4) = - tx_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,5) =  tx_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(3,4) = sin_q_hip_joint_RF;
    (*this)(3,5) = -cos_q_hip_joint_RF;
    (*this)(4,4) = cos_q_hip_joint_RF;
    (*this)(4,5) = sin_q_hip_joint_RF;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_hip_link_RF::Type_fr_base_X_fr_hip_link_RF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = - ty_hip_joint_RF;    // Maxima DSL: -_k__ty_hip_joint_RF
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_hip_link_RF& ForceTransforms::Type_fr_base_X_fr_hip_link_RF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    (*this)(0,3) = - ty_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(0,4) =  ty_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,0) = sin_q_hip_joint_RF;
    (*this)(1,1) = cos_q_hip_joint_RF;
    (*this)(1,3) =  tx_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(1,4) = - tx_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(2,0) = -cos_q_hip_joint_RF;
    (*this)(2,1) = sin_q_hip_joint_RF;
    (*this)(2,3) =  tx_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(2,4) =  tx_hip_joint_RF * cos_q_hip_joint_RF;
    (*this)(4,3) = sin_q_hip_joint_RF;
    (*this)(4,4) = cos_q_hip_joint_RF;
    (*this)(5,3) = -cos_q_hip_joint_RF;
    (*this)(5,4) = sin_q_hip_joint_RF;
    return *this;
}
ForceTransforms::Type_fr_thigh_link_RF_X_fr_hip_link_RF::Type_fr_thigh_link_RF_X_fr_hip_link_RF()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = - tx_elbow_joint_RF;    // Maxima DSL: -_k__tx_elbow_joint_RF
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_thigh_link_RF_X_fr_hip_link_RF& ForceTransforms::Type_fr_thigh_link_RF_X_fr_hip_link_RF::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    (*this)(0,0) = cos_q_elbow_joint_RF;
    (*this)(0,2) = sin_q_elbow_joint_RF;
    (*this)(0,4) = - tx_elbow_joint_RF * sin_q_elbow_joint_RF;
    (*this)(1,0) = -sin_q_elbow_joint_RF;
    (*this)(1,2) = cos_q_elbow_joint_RF;
    (*this)(1,4) = - tx_elbow_joint_RF * cos_q_elbow_joint_RF;
    (*this)(3,3) = cos_q_elbow_joint_RF;
    (*this)(3,5) = sin_q_elbow_joint_RF;
    (*this)(4,3) = -sin_q_elbow_joint_RF;
    (*this)(4,5) = cos_q_elbow_joint_RF;
    return *this;
}
ForceTransforms::Type_fr_hip_link_RF_X_fr_thigh_link_RF::Type_fr_hip_link_RF_X_fr_thigh_link_RF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = - tx_elbow_joint_RF;    // Maxima DSL: -_k__tx_elbow_joint_RF
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_hip_link_RF_X_fr_thigh_link_RF& ForceTransforms::Type_fr_hip_link_RF_X_fr_thigh_link_RF::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    (*this)(0,0) = cos_q_elbow_joint_RF;
    (*this)(0,1) = -sin_q_elbow_joint_RF;
    (*this)(1,3) = - tx_elbow_joint_RF * sin_q_elbow_joint_RF;
    (*this)(1,4) = - tx_elbow_joint_RF * cos_q_elbow_joint_RF;
    (*this)(2,0) = sin_q_elbow_joint_RF;
    (*this)(2,1) = cos_q_elbow_joint_RF;
    (*this)(3,3) = cos_q_elbow_joint_RF;
    (*this)(3,4) = -sin_q_elbow_joint_RF;
    (*this)(5,3) = sin_q_elbow_joint_RF;
    (*this)(5,4) = cos_q_elbow_joint_RF;
    return *this;
}
ForceTransforms::Type_fr_shank_link_RF_X_fr_thigh_link_RF::Type_fr_shank_link_RF_X_fr_thigh_link_RF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tx_knee_joint_RF;    // Maxima DSL: -_k__tx_knee_joint_RF
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_shank_link_RF_X_fr_thigh_link_RF& ForceTransforms::Type_fr_shank_link_RF_X_fr_thigh_link_RF::update(const state_t& q)
{
    Scalar sin_q_knee_joint_RF  = ScalarTraits::sin( q(KNEE_JOINT_RF) );
    Scalar cos_q_knee_joint_RF  = ScalarTraits::cos( q(KNEE_JOINT_RF) );
    (*this)(0,0) = cos_q_knee_joint_RF;
    (*this)(0,1) = sin_q_knee_joint_RF;
    (*this)(0,5) =  tx_knee_joint_RF * sin_q_knee_joint_RF;
    (*this)(1,0) = -sin_q_knee_joint_RF;
    (*this)(1,1) = cos_q_knee_joint_RF;
    (*this)(1,5) =  tx_knee_joint_RF * cos_q_knee_joint_RF;
    (*this)(3,3) = cos_q_knee_joint_RF;
    (*this)(3,4) = sin_q_knee_joint_RF;
    (*this)(4,3) = -sin_q_knee_joint_RF;
    (*this)(4,4) = cos_q_knee_joint_RF;
    return *this;
}
ForceTransforms::Type_fr_thigh_link_RF_X_fr_shank_link_RF::Type_fr_thigh_link_RF_X_fr_shank_link_RF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = - tx_knee_joint_RF;    // Maxima DSL: -_k__tx_knee_joint_RF
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_thigh_link_RF_X_fr_shank_link_RF& ForceTransforms::Type_fr_thigh_link_RF_X_fr_shank_link_RF::update(const state_t& q)
{
    Scalar sin_q_knee_joint_RF  = ScalarTraits::sin( q(KNEE_JOINT_RF) );
    Scalar cos_q_knee_joint_RF  = ScalarTraits::cos( q(KNEE_JOINT_RF) );
    (*this)(0,0) = cos_q_knee_joint_RF;
    (*this)(0,1) = -sin_q_knee_joint_RF;
    (*this)(1,0) = sin_q_knee_joint_RF;
    (*this)(1,1) = cos_q_knee_joint_RF;
    (*this)(2,3) =  tx_knee_joint_RF * sin_q_knee_joint_RF;
    (*this)(2,4) =  tx_knee_joint_RF * cos_q_knee_joint_RF;
    (*this)(3,3) = cos_q_knee_joint_RF;
    (*this)(3,4) = -sin_q_knee_joint_RF;
    (*this)(4,3) = sin_q_knee_joint_RF;
    (*this)(4,4) = cos_q_knee_joint_RF;
    return *this;
}
ForceTransforms::Type_fr_hip_link_LH_X_fr_base::Type_fr_hip_link_LH_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = - ty_hip_joint_LH;    // Maxima DSL: -_k__ty_hip_joint_LH
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_hip_link_LH_X_fr_base& ForceTransforms::Type_fr_hip_link_LH_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    (*this)(0,1) = sin_q_hip_joint_LH;
    (*this)(0,2) = -cos_q_hip_joint_LH;
    (*this)(0,3) = - ty_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(0,4) =  tx_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(0,5) =  tx_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,1) = cos_q_hip_joint_LH;
    (*this)(1,2) = sin_q_hip_joint_LH;
    (*this)(1,3) =  ty_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,4) = - tx_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,5) =  tx_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(3,4) = sin_q_hip_joint_LH;
    (*this)(3,5) = -cos_q_hip_joint_LH;
    (*this)(4,4) = cos_q_hip_joint_LH;
    (*this)(4,5) = sin_q_hip_joint_LH;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_hip_link_LH::Type_fr_base_X_fr_hip_link_LH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = - ty_hip_joint_LH;    // Maxima DSL: -_k__ty_hip_joint_LH
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_hip_link_LH& ForceTransforms::Type_fr_base_X_fr_hip_link_LH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    (*this)(0,3) = - ty_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(0,4) =  ty_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,0) = sin_q_hip_joint_LH;
    (*this)(1,1) = cos_q_hip_joint_LH;
    (*this)(1,3) =  tx_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(1,4) = - tx_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(2,0) = -cos_q_hip_joint_LH;
    (*this)(2,1) = sin_q_hip_joint_LH;
    (*this)(2,3) =  tx_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(2,4) =  tx_hip_joint_LH * cos_q_hip_joint_LH;
    (*this)(4,3) = sin_q_hip_joint_LH;
    (*this)(4,4) = cos_q_hip_joint_LH;
    (*this)(5,3) = -cos_q_hip_joint_LH;
    (*this)(5,4) = sin_q_hip_joint_LH;
    return *this;
}
ForceTransforms::Type_fr_thigh_link_LH_X_fr_hip_link_LH::Type_fr_thigh_link_LH_X_fr_hip_link_LH()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = - tx_elbow_joint_LH;    // Maxima DSL: -_k__tx_elbow_joint_LH
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_thigh_link_LH_X_fr_hip_link_LH& ForceTransforms::Type_fr_thigh_link_LH_X_fr_hip_link_LH::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    (*this)(0,0) = cos_q_elbow_joint_LH;
    (*this)(0,2) = sin_q_elbow_joint_LH;
    (*this)(0,4) = - tx_elbow_joint_LH * sin_q_elbow_joint_LH;
    (*this)(1,0) = -sin_q_elbow_joint_LH;
    (*this)(1,2) = cos_q_elbow_joint_LH;
    (*this)(1,4) = - tx_elbow_joint_LH * cos_q_elbow_joint_LH;
    (*this)(3,3) = cos_q_elbow_joint_LH;
    (*this)(3,5) = sin_q_elbow_joint_LH;
    (*this)(4,3) = -sin_q_elbow_joint_LH;
    (*this)(4,5) = cos_q_elbow_joint_LH;
    return *this;
}
ForceTransforms::Type_fr_hip_link_LH_X_fr_thigh_link_LH::Type_fr_hip_link_LH_X_fr_thigh_link_LH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = - tx_elbow_joint_LH;    // Maxima DSL: -_k__tx_elbow_joint_LH
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_hip_link_LH_X_fr_thigh_link_LH& ForceTransforms::Type_fr_hip_link_LH_X_fr_thigh_link_LH::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    (*this)(0,0) = cos_q_elbow_joint_LH;
    (*this)(0,1) = -sin_q_elbow_joint_LH;
    (*this)(1,3) = - tx_elbow_joint_LH * sin_q_elbow_joint_LH;
    (*this)(1,4) = - tx_elbow_joint_LH * cos_q_elbow_joint_LH;
    (*this)(2,0) = sin_q_elbow_joint_LH;
    (*this)(2,1) = cos_q_elbow_joint_LH;
    (*this)(3,3) = cos_q_elbow_joint_LH;
    (*this)(3,4) = -sin_q_elbow_joint_LH;
    (*this)(5,3) = sin_q_elbow_joint_LH;
    (*this)(5,4) = cos_q_elbow_joint_LH;
    return *this;
}
ForceTransforms::Type_fr_shank_link_LH_X_fr_thigh_link_LH::Type_fr_shank_link_LH_X_fr_thigh_link_LH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tx_knee_joint_LH;    // Maxima DSL: -_k__tx_knee_joint_LH
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_shank_link_LH_X_fr_thigh_link_LH& ForceTransforms::Type_fr_shank_link_LH_X_fr_thigh_link_LH::update(const state_t& q)
{
    Scalar sin_q_knee_joint_LH  = ScalarTraits::sin( q(KNEE_JOINT_LH) );
    Scalar cos_q_knee_joint_LH  = ScalarTraits::cos( q(KNEE_JOINT_LH) );
    (*this)(0,0) = cos_q_knee_joint_LH;
    (*this)(0,1) = sin_q_knee_joint_LH;
    (*this)(0,5) =  tx_knee_joint_LH * sin_q_knee_joint_LH;
    (*this)(1,0) = -sin_q_knee_joint_LH;
    (*this)(1,1) = cos_q_knee_joint_LH;
    (*this)(1,5) =  tx_knee_joint_LH * cos_q_knee_joint_LH;
    (*this)(3,3) = cos_q_knee_joint_LH;
    (*this)(3,4) = sin_q_knee_joint_LH;
    (*this)(4,3) = -sin_q_knee_joint_LH;
    (*this)(4,4) = cos_q_knee_joint_LH;
    return *this;
}
ForceTransforms::Type_fr_thigh_link_LH_X_fr_shank_link_LH::Type_fr_thigh_link_LH_X_fr_shank_link_LH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = - tx_knee_joint_LH;    // Maxima DSL: -_k__tx_knee_joint_LH
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_thigh_link_LH_X_fr_shank_link_LH& ForceTransforms::Type_fr_thigh_link_LH_X_fr_shank_link_LH::update(const state_t& q)
{
    Scalar sin_q_knee_joint_LH  = ScalarTraits::sin( q(KNEE_JOINT_LH) );
    Scalar cos_q_knee_joint_LH  = ScalarTraits::cos( q(KNEE_JOINT_LH) );
    (*this)(0,0) = cos_q_knee_joint_LH;
    (*this)(0,1) = -sin_q_knee_joint_LH;
    (*this)(1,0) = sin_q_knee_joint_LH;
    (*this)(1,1) = cos_q_knee_joint_LH;
    (*this)(2,3) =  tx_knee_joint_LH * sin_q_knee_joint_LH;
    (*this)(2,4) =  tx_knee_joint_LH * cos_q_knee_joint_LH;
    (*this)(3,3) = cos_q_knee_joint_LH;
    (*this)(3,4) = -sin_q_knee_joint_LH;
    (*this)(4,3) = sin_q_knee_joint_LH;
    (*this)(4,4) = cos_q_knee_joint_LH;
    return *this;
}
ForceTransforms::Type_fr_hip_link_RH_X_fr_base::Type_fr_hip_link_RH_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = - ty_hip_joint_RH;    // Maxima DSL: -_k__ty_hip_joint_RH
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_hip_link_RH_X_fr_base& ForceTransforms::Type_fr_hip_link_RH_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    (*this)(0,1) = sin_q_hip_joint_RH;
    (*this)(0,2) = -cos_q_hip_joint_RH;
    (*this)(0,3) = - ty_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(0,4) =  tx_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(0,5) =  tx_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,1) = cos_q_hip_joint_RH;
    (*this)(1,2) = sin_q_hip_joint_RH;
    (*this)(1,3) =  ty_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,4) = - tx_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,5) =  tx_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(3,4) = sin_q_hip_joint_RH;
    (*this)(3,5) = -cos_q_hip_joint_RH;
    (*this)(4,4) = cos_q_hip_joint_RH;
    (*this)(4,5) = sin_q_hip_joint_RH;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_hip_link_RH::Type_fr_base_X_fr_hip_link_RH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = - ty_hip_joint_RH;    // Maxima DSL: -_k__ty_hip_joint_RH
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_hip_link_RH& ForceTransforms::Type_fr_base_X_fr_hip_link_RH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    (*this)(0,3) = - ty_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(0,4) =  ty_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,0) = sin_q_hip_joint_RH;
    (*this)(1,1) = cos_q_hip_joint_RH;
    (*this)(1,3) =  tx_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(1,4) = - tx_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(2,0) = -cos_q_hip_joint_RH;
    (*this)(2,1) = sin_q_hip_joint_RH;
    (*this)(2,3) =  tx_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(2,4) =  tx_hip_joint_RH * cos_q_hip_joint_RH;
    (*this)(4,3) = sin_q_hip_joint_RH;
    (*this)(4,4) = cos_q_hip_joint_RH;
    (*this)(5,3) = -cos_q_hip_joint_RH;
    (*this)(5,4) = sin_q_hip_joint_RH;
    return *this;
}
ForceTransforms::Type_fr_thigh_link_RH_X_fr_hip_link_RH::Type_fr_thigh_link_RH_X_fr_hip_link_RH()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = - tx_elbow_joint_RH;    // Maxima DSL: -_k__tx_elbow_joint_RH
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_thigh_link_RH_X_fr_hip_link_RH& ForceTransforms::Type_fr_thigh_link_RH_X_fr_hip_link_RH::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    (*this)(0,0) = cos_q_elbow_joint_RH;
    (*this)(0,2) = sin_q_elbow_joint_RH;
    (*this)(0,4) = - tx_elbow_joint_RH * sin_q_elbow_joint_RH;
    (*this)(1,0) = -sin_q_elbow_joint_RH;
    (*this)(1,2) = cos_q_elbow_joint_RH;
    (*this)(1,4) = - tx_elbow_joint_RH * cos_q_elbow_joint_RH;
    (*this)(3,3) = cos_q_elbow_joint_RH;
    (*this)(3,5) = sin_q_elbow_joint_RH;
    (*this)(4,3) = -sin_q_elbow_joint_RH;
    (*this)(4,5) = cos_q_elbow_joint_RH;
    return *this;
}
ForceTransforms::Type_fr_hip_link_RH_X_fr_thigh_link_RH::Type_fr_hip_link_RH_X_fr_thigh_link_RH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = - tx_elbow_joint_RH;    // Maxima DSL: -_k__tx_elbow_joint_RH
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_hip_link_RH_X_fr_thigh_link_RH& ForceTransforms::Type_fr_hip_link_RH_X_fr_thigh_link_RH::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    (*this)(0,0) = cos_q_elbow_joint_RH;
    (*this)(0,1) = -sin_q_elbow_joint_RH;
    (*this)(1,3) = - tx_elbow_joint_RH * sin_q_elbow_joint_RH;
    (*this)(1,4) = - tx_elbow_joint_RH * cos_q_elbow_joint_RH;
    (*this)(2,0) = sin_q_elbow_joint_RH;
    (*this)(2,1) = cos_q_elbow_joint_RH;
    (*this)(3,3) = cos_q_elbow_joint_RH;
    (*this)(3,4) = -sin_q_elbow_joint_RH;
    (*this)(5,3) = sin_q_elbow_joint_RH;
    (*this)(5,4) = cos_q_elbow_joint_RH;
    return *this;
}
ForceTransforms::Type_fr_shank_link_RH_X_fr_thigh_link_RH::Type_fr_shank_link_RH_X_fr_thigh_link_RH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tx_knee_joint_RH;    // Maxima DSL: -_k__tx_knee_joint_RH
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_shank_link_RH_X_fr_thigh_link_RH& ForceTransforms::Type_fr_shank_link_RH_X_fr_thigh_link_RH::update(const state_t& q)
{
    Scalar sin_q_knee_joint_RH  = ScalarTraits::sin( q(KNEE_JOINT_RH) );
    Scalar cos_q_knee_joint_RH  = ScalarTraits::cos( q(KNEE_JOINT_RH) );
    (*this)(0,0) = cos_q_knee_joint_RH;
    (*this)(0,1) = sin_q_knee_joint_RH;
    (*this)(0,5) =  tx_knee_joint_RH * sin_q_knee_joint_RH;
    (*this)(1,0) = -sin_q_knee_joint_RH;
    (*this)(1,1) = cos_q_knee_joint_RH;
    (*this)(1,5) =  tx_knee_joint_RH * cos_q_knee_joint_RH;
    (*this)(3,3) = cos_q_knee_joint_RH;
    (*this)(3,4) = sin_q_knee_joint_RH;
    (*this)(4,3) = -sin_q_knee_joint_RH;
    (*this)(4,4) = cos_q_knee_joint_RH;
    return *this;
}
ForceTransforms::Type_fr_thigh_link_RH_X_fr_shank_link_RH::Type_fr_thigh_link_RH_X_fr_shank_link_RH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = - tx_knee_joint_RH;    // Maxima DSL: -_k__tx_knee_joint_RH
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_thigh_link_RH_X_fr_shank_link_RH& ForceTransforms::Type_fr_thigh_link_RH_X_fr_shank_link_RH::update(const state_t& q)
{
    Scalar sin_q_knee_joint_RH  = ScalarTraits::sin( q(KNEE_JOINT_RH) );
    Scalar cos_q_knee_joint_RH  = ScalarTraits::cos( q(KNEE_JOINT_RH) );
    (*this)(0,0) = cos_q_knee_joint_RH;
    (*this)(0,1) = -sin_q_knee_joint_RH;
    (*this)(1,0) = sin_q_knee_joint_RH;
    (*this)(1,1) = cos_q_knee_joint_RH;
    (*this)(2,3) =  tx_knee_joint_RH * sin_q_knee_joint_RH;
    (*this)(2,4) =  tx_knee_joint_RH * cos_q_knee_joint_RH;
    (*this)(3,3) = cos_q_knee_joint_RH;
    (*this)(3,4) = -sin_q_knee_joint_RH;
    (*this)(4,3) = sin_q_knee_joint_RH;
    (*this)(4,4) = cos_q_knee_joint_RH;
    return *this;
}

HomogeneousTransforms::Type_fr_base_X_LF_FOOT::Type_fr_base_X_LF_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_LF_FOOT& HomogeneousTransforms::Type_fr_base_X_LF_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    Scalar sin_q_knee_joint_LF  = ScalarTraits::sin( q(KNEE_JOINT_LF) );
    Scalar cos_q_knee_joint_LF  = ScalarTraits::cos( q(KNEE_JOINT_LF) );
    (*this)(0,0) = (cos_q_elbow_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_knee_joint_LF);
    (*this)(0,1) = (cos_q_elbow_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_knee_joint_LF);
    (*this)(0,3) = ( tx_LF_FOOT * cos_q_elbow_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * sin_q_elbow_joint_LF * cos_q_knee_joint_LF)+( tx_knee_joint_LF * sin_q_elbow_joint_LF)+ tx_hip_joint_LF;
    (*this)(1,0) = (cos_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF);
    (*this)(1,1) = (-cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(1,3) = (- tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * cos_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)+((( tx_knee_joint_LF * cos_q_elbow_joint_LF)+ tx_elbow_joint_LF) * sin_q_hip_joint_LF)+ ty_hip_joint_LF;
    (*this)(2,0) = (sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-(cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(2,1) = (cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(2,3) = ( tx_LF_FOOT * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-( tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF)+(((- tx_knee_joint_LF * cos_q_elbow_joint_LF)- tx_elbow_joint_LF) * cos_q_hip_joint_LF);
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_RF_FOOT::Type_fr_base_X_RF_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_RF_FOOT& HomogeneousTransforms::Type_fr_base_X_RF_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    Scalar sin_q_knee_joint_RF  = ScalarTraits::sin( q(KNEE_JOINT_RF) );
    Scalar cos_q_knee_joint_RF  = ScalarTraits::cos( q(KNEE_JOINT_RF) );
    (*this)(0,0) = (cos_q_elbow_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_knee_joint_RF);
    (*this)(0,1) = (cos_q_elbow_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_knee_joint_RF);
    (*this)(0,3) = ( tx_RF_FOOT * cos_q_elbow_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * sin_q_elbow_joint_RF * cos_q_knee_joint_RF)+( tx_knee_joint_RF * sin_q_elbow_joint_RF)+ tx_hip_joint_RF;
    (*this)(1,0) = (cos_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF);
    (*this)(1,1) = (-cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(1,3) = (- tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * cos_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)+((( tx_knee_joint_RF * cos_q_elbow_joint_RF)+ tx_elbow_joint_RF) * sin_q_hip_joint_RF)+ ty_hip_joint_RF;
    (*this)(2,0) = (sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-(cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(2,1) = (cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(2,3) = ( tx_RF_FOOT * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-( tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF)+(((- tx_knee_joint_RF * cos_q_elbow_joint_RF)- tx_elbow_joint_RF) * cos_q_hip_joint_RF);
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_LH_FOOT::Type_fr_base_X_LH_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_LH_FOOT& HomogeneousTransforms::Type_fr_base_X_LH_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    Scalar sin_q_knee_joint_LH  = ScalarTraits::sin( q(KNEE_JOINT_LH) );
    Scalar cos_q_knee_joint_LH  = ScalarTraits::cos( q(KNEE_JOINT_LH) );
    (*this)(0,0) = (cos_q_elbow_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_knee_joint_LH);
    (*this)(0,1) = (cos_q_elbow_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_knee_joint_LH);
    (*this)(0,3) = ( tx_LH_FOOT * cos_q_elbow_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * sin_q_elbow_joint_LH * cos_q_knee_joint_LH)+( tx_knee_joint_LH * sin_q_elbow_joint_LH)+ tx_hip_joint_LH;
    (*this)(1,0) = (cos_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH);
    (*this)(1,1) = (-cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(1,3) = (- tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * cos_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)+((( tx_knee_joint_LH * cos_q_elbow_joint_LH)+ tx_elbow_joint_LH) * sin_q_hip_joint_LH)+ ty_hip_joint_LH;
    (*this)(2,0) = (sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-(cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(2,1) = (cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(2,3) = ( tx_LH_FOOT * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-( tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH)+(((- tx_knee_joint_LH * cos_q_elbow_joint_LH)- tx_elbow_joint_LH) * cos_q_hip_joint_LH);
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_RH_FOOT::Type_fr_base_X_RH_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_RH_FOOT& HomogeneousTransforms::Type_fr_base_X_RH_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    Scalar sin_q_knee_joint_RH  = ScalarTraits::sin( q(KNEE_JOINT_RH) );
    Scalar cos_q_knee_joint_RH  = ScalarTraits::cos( q(KNEE_JOINT_RH) );
    (*this)(0,0) = (cos_q_elbow_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_knee_joint_RH);
    (*this)(0,1) = (cos_q_elbow_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_knee_joint_RH);
    (*this)(0,3) = ( tx_RH_FOOT * cos_q_elbow_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * sin_q_elbow_joint_RH * cos_q_knee_joint_RH)+( tx_knee_joint_RH * sin_q_elbow_joint_RH)+ tx_hip_joint_RH;
    (*this)(1,0) = (cos_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH);
    (*this)(1,1) = (-cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(1,3) = (- tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * cos_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)+((( tx_knee_joint_RH * cos_q_elbow_joint_RH)+ tx_elbow_joint_RH) * sin_q_hip_joint_RH)+ ty_hip_joint_RH;
    (*this)(2,0) = (sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-(cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(2,1) = (cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(2,3) = ( tx_RH_FOOT * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-( tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH)+(((- tx_knee_joint_RH * cos_q_elbow_joint_RH)- tx_elbow_joint_RH) * cos_q_hip_joint_RH);
    return *this;
}
HomogeneousTransforms::Type_imu_link_X_LF_FOOT::Type_imu_link_X_LF_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_imu_link_X_LF_FOOT& HomogeneousTransforms::Type_imu_link_X_LF_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    Scalar sin_q_knee_joint_LF  = ScalarTraits::sin( q(KNEE_JOINT_LF) );
    Scalar cos_q_knee_joint_LF  = ScalarTraits::cos( q(KNEE_JOINT_LF) );
    (*this)(0,0) = (cos_q_elbow_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_knee_joint_LF);
    (*this)(0,1) = (cos_q_elbow_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_knee_joint_LF);
    (*this)(0,3) = ( tx_LF_FOOT * cos_q_elbow_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * sin_q_elbow_joint_LF * cos_q_knee_joint_LF)+( tx_knee_joint_LF * sin_q_elbow_joint_LF)+ tx_hip_joint_LF;
    (*this)(1,0) = (cos_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF);
    (*this)(1,1) = (-cos_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)-(sin_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(1,3) = (- tx_LF_FOOT * sin_q_elbow_joint_LF * sin_q_hip_joint_LF * sin_q_knee_joint_LF)+( tx_LF_FOOT * cos_q_elbow_joint_LF * sin_q_hip_joint_LF * cos_q_knee_joint_LF)+((( tx_knee_joint_LF * cos_q_elbow_joint_LF)+ tx_elbow_joint_LF) * sin_q_hip_joint_LF)+ ty_hip_joint_LF;
    (*this)(2,0) = (sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-(cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(2,1) = (cos_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)+(sin_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF);
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(2,3) = ( tx_LF_FOOT * sin_q_elbow_joint_LF * cos_q_hip_joint_LF * sin_q_knee_joint_LF)-( tx_LF_FOOT * cos_q_elbow_joint_LF * cos_q_hip_joint_LF * cos_q_knee_joint_LF)+(((- tx_knee_joint_LF * cos_q_elbow_joint_LF)- tx_elbow_joint_LF) * cos_q_hip_joint_LF);
    return *this;
}
HomogeneousTransforms::Type_imu_link_X_RF_FOOT::Type_imu_link_X_RF_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_imu_link_X_RF_FOOT& HomogeneousTransforms::Type_imu_link_X_RF_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    Scalar sin_q_knee_joint_RF  = ScalarTraits::sin( q(KNEE_JOINT_RF) );
    Scalar cos_q_knee_joint_RF  = ScalarTraits::cos( q(KNEE_JOINT_RF) );
    (*this)(0,0) = (cos_q_elbow_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_knee_joint_RF);
    (*this)(0,1) = (cos_q_elbow_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_knee_joint_RF);
    (*this)(0,3) = ( tx_RF_FOOT * cos_q_elbow_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * sin_q_elbow_joint_RF * cos_q_knee_joint_RF)+( tx_knee_joint_RF * sin_q_elbow_joint_RF)+ tx_hip_joint_RF;
    (*this)(1,0) = (cos_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF);
    (*this)(1,1) = (-cos_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)-(sin_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(1,3) = (- tx_RF_FOOT * sin_q_elbow_joint_RF * sin_q_hip_joint_RF * sin_q_knee_joint_RF)+( tx_RF_FOOT * cos_q_elbow_joint_RF * sin_q_hip_joint_RF * cos_q_knee_joint_RF)+((( tx_knee_joint_RF * cos_q_elbow_joint_RF)+ tx_elbow_joint_RF) * sin_q_hip_joint_RF)+ ty_hip_joint_RF;
    (*this)(2,0) = (sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-(cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(2,1) = (cos_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)+(sin_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF);
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(2,3) = ( tx_RF_FOOT * sin_q_elbow_joint_RF * cos_q_hip_joint_RF * sin_q_knee_joint_RF)-( tx_RF_FOOT * cos_q_elbow_joint_RF * cos_q_hip_joint_RF * cos_q_knee_joint_RF)+(((- tx_knee_joint_RF * cos_q_elbow_joint_RF)- tx_elbow_joint_RF) * cos_q_hip_joint_RF);
    return *this;
}
HomogeneousTransforms::Type_imu_link_X_LH_FOOT::Type_imu_link_X_LH_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_imu_link_X_LH_FOOT& HomogeneousTransforms::Type_imu_link_X_LH_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    Scalar sin_q_knee_joint_LH  = ScalarTraits::sin( q(KNEE_JOINT_LH) );
    Scalar cos_q_knee_joint_LH  = ScalarTraits::cos( q(KNEE_JOINT_LH) );
    (*this)(0,0) = (cos_q_elbow_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_knee_joint_LH);
    (*this)(0,1) = (cos_q_elbow_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_knee_joint_LH);
    (*this)(0,3) = ( tx_LH_FOOT * cos_q_elbow_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * sin_q_elbow_joint_LH * cos_q_knee_joint_LH)+( tx_knee_joint_LH * sin_q_elbow_joint_LH)+ tx_hip_joint_LH;
    (*this)(1,0) = (cos_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH);
    (*this)(1,1) = (-cos_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)-(sin_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(1,3) = (- tx_LH_FOOT * sin_q_elbow_joint_LH * sin_q_hip_joint_LH * sin_q_knee_joint_LH)+( tx_LH_FOOT * cos_q_elbow_joint_LH * sin_q_hip_joint_LH * cos_q_knee_joint_LH)+((( tx_knee_joint_LH * cos_q_elbow_joint_LH)+ tx_elbow_joint_LH) * sin_q_hip_joint_LH)+ ty_hip_joint_LH;
    (*this)(2,0) = (sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-(cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(2,1) = (cos_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)+(sin_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH);
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(2,3) = ( tx_LH_FOOT * sin_q_elbow_joint_LH * cos_q_hip_joint_LH * sin_q_knee_joint_LH)-( tx_LH_FOOT * cos_q_elbow_joint_LH * cos_q_hip_joint_LH * cos_q_knee_joint_LH)+(((- tx_knee_joint_LH * cos_q_elbow_joint_LH)- tx_elbow_joint_LH) * cos_q_hip_joint_LH);
    return *this;
}
HomogeneousTransforms::Type_imu_link_X_RH_FOOT::Type_imu_link_X_RH_FOOT()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_imu_link_X_RH_FOOT& HomogeneousTransforms::Type_imu_link_X_RH_FOOT::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    Scalar sin_q_knee_joint_RH  = ScalarTraits::sin( q(KNEE_JOINT_RH) );
    Scalar cos_q_knee_joint_RH  = ScalarTraits::cos( q(KNEE_JOINT_RH) );
    (*this)(0,0) = (cos_q_elbow_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_knee_joint_RH);
    (*this)(0,1) = (cos_q_elbow_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_knee_joint_RH);
    (*this)(0,3) = ( tx_RH_FOOT * cos_q_elbow_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * sin_q_elbow_joint_RH * cos_q_knee_joint_RH)+( tx_knee_joint_RH * sin_q_elbow_joint_RH)+ tx_hip_joint_RH;
    (*this)(1,0) = (cos_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH);
    (*this)(1,1) = (-cos_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)-(sin_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(1,3) = (- tx_RH_FOOT * sin_q_elbow_joint_RH * sin_q_hip_joint_RH * sin_q_knee_joint_RH)+( tx_RH_FOOT * cos_q_elbow_joint_RH * sin_q_hip_joint_RH * cos_q_knee_joint_RH)+((( tx_knee_joint_RH * cos_q_elbow_joint_RH)+ tx_elbow_joint_RH) * sin_q_hip_joint_RH)+ ty_hip_joint_RH;
    (*this)(2,0) = (sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-(cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(2,1) = (cos_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)+(sin_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH);
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(2,3) = ( tx_RH_FOOT * sin_q_elbow_joint_RH * cos_q_hip_joint_RH * sin_q_knee_joint_RH)-( tx_RH_FOOT * cos_q_elbow_joint_RH * cos_q_hip_joint_RH * cos_q_knee_joint_RH)+(((- tx_knee_joint_RH * cos_q_elbow_joint_RH)- tx_elbow_joint_RH) * cos_q_hip_joint_RH);
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_hip_joint_LF::Type_fr_base_X_fr_hip_joint_LF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_hip_joint_LF;    // Maxima DSL: _k__tx_hip_joint_LF
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_hip_joint_LF;    // Maxima DSL: _k__ty_hip_joint_LF
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_hip_joint_LF& HomogeneousTransforms::Type_fr_base_X_fr_hip_joint_LF::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_elbow_joint_LF::Type_fr_base_X_fr_elbow_joint_LF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_hip_joint_LF;    // Maxima DSL: _k__tx_hip_joint_LF
    (*this)(1,1) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_elbow_joint_LF& HomogeneousTransforms::Type_fr_base_X_fr_elbow_joint_LF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    (*this)(1,0) = sin_q_hip_joint_LF;
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(1,3) = ( tx_elbow_joint_LF * sin_q_hip_joint_LF)+ ty_hip_joint_LF;
    (*this)(2,0) = -cos_q_hip_joint_LF;
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(2,3) = - tx_elbow_joint_LF * cos_q_hip_joint_LF;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_knee_joint_LF::Type_fr_base_X_fr_knee_joint_LF()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_knee_joint_LF& HomogeneousTransforms::Type_fr_base_X_fr_knee_joint_LF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    (*this)(0,0) = sin_q_elbow_joint_LF;
    (*this)(0,1) = cos_q_elbow_joint_LF;
    (*this)(0,3) = ( tx_knee_joint_LF * sin_q_elbow_joint_LF)+ tx_hip_joint_LF;
    (*this)(1,0) = cos_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,1) = -sin_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(1,3) = ((( tx_knee_joint_LF * cos_q_elbow_joint_LF)+ tx_elbow_joint_LF) * sin_q_hip_joint_LF)+ ty_hip_joint_LF;
    (*this)(2,0) = -cos_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(2,1) = sin_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(2,3) = ((- tx_knee_joint_LF * cos_q_elbow_joint_LF)- tx_elbow_joint_LF) * cos_q_hip_joint_LF;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_hip_joint_RF::Type_fr_base_X_fr_hip_joint_RF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_hip_joint_RF;    // Maxima DSL: _k__tx_hip_joint_RF
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_hip_joint_RF;    // Maxima DSL: _k__ty_hip_joint_RF
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_hip_joint_RF& HomogeneousTransforms::Type_fr_base_X_fr_hip_joint_RF::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_elbow_joint_RF::Type_fr_base_X_fr_elbow_joint_RF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_hip_joint_RF;    // Maxima DSL: _k__tx_hip_joint_RF
    (*this)(1,1) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_elbow_joint_RF& HomogeneousTransforms::Type_fr_base_X_fr_elbow_joint_RF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    (*this)(1,0) = sin_q_hip_joint_RF;
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(1,3) = ( tx_elbow_joint_RF * sin_q_hip_joint_RF)+ ty_hip_joint_RF;
    (*this)(2,0) = -cos_q_hip_joint_RF;
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(2,3) = - tx_elbow_joint_RF * cos_q_hip_joint_RF;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_knee_joint_RF::Type_fr_base_X_fr_knee_joint_RF()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_knee_joint_RF& HomogeneousTransforms::Type_fr_base_X_fr_knee_joint_RF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    (*this)(0,0) = sin_q_elbow_joint_RF;
    (*this)(0,1) = cos_q_elbow_joint_RF;
    (*this)(0,3) = ( tx_knee_joint_RF * sin_q_elbow_joint_RF)+ tx_hip_joint_RF;
    (*this)(1,0) = cos_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,1) = -sin_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(1,3) = ((( tx_knee_joint_RF * cos_q_elbow_joint_RF)+ tx_elbow_joint_RF) * sin_q_hip_joint_RF)+ ty_hip_joint_RF;
    (*this)(2,0) = -cos_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(2,1) = sin_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(2,3) = ((- tx_knee_joint_RF * cos_q_elbow_joint_RF)- tx_elbow_joint_RF) * cos_q_hip_joint_RF;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_hip_joint_LH::Type_fr_base_X_fr_hip_joint_LH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_hip_joint_LH;    // Maxima DSL: _k__tx_hip_joint_LH
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_hip_joint_LH;    // Maxima DSL: _k__ty_hip_joint_LH
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_hip_joint_LH& HomogeneousTransforms::Type_fr_base_X_fr_hip_joint_LH::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_elbow_joint_LH::Type_fr_base_X_fr_elbow_joint_LH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_hip_joint_LH;    // Maxima DSL: _k__tx_hip_joint_LH
    (*this)(1,1) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_elbow_joint_LH& HomogeneousTransforms::Type_fr_base_X_fr_elbow_joint_LH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    (*this)(1,0) = sin_q_hip_joint_LH;
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(1,3) = ( tx_elbow_joint_LH * sin_q_hip_joint_LH)+ ty_hip_joint_LH;
    (*this)(2,0) = -cos_q_hip_joint_LH;
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(2,3) = - tx_elbow_joint_LH * cos_q_hip_joint_LH;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_knee_joint_LH::Type_fr_base_X_fr_knee_joint_LH()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_knee_joint_LH& HomogeneousTransforms::Type_fr_base_X_fr_knee_joint_LH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    (*this)(0,0) = sin_q_elbow_joint_LH;
    (*this)(0,1) = cos_q_elbow_joint_LH;
    (*this)(0,3) = ( tx_knee_joint_LH * sin_q_elbow_joint_LH)+ tx_hip_joint_LH;
    (*this)(1,0) = cos_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,1) = -sin_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(1,3) = ((( tx_knee_joint_LH * cos_q_elbow_joint_LH)+ tx_elbow_joint_LH) * sin_q_hip_joint_LH)+ ty_hip_joint_LH;
    (*this)(2,0) = -cos_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(2,1) = sin_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(2,3) = ((- tx_knee_joint_LH * cos_q_elbow_joint_LH)- tx_elbow_joint_LH) * cos_q_hip_joint_LH;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_hip_joint_RH::Type_fr_base_X_fr_hip_joint_RH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_hip_joint_RH;    // Maxima DSL: _k__tx_hip_joint_RH
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_hip_joint_RH;    // Maxima DSL: _k__ty_hip_joint_RH
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_hip_joint_RH& HomogeneousTransforms::Type_fr_base_X_fr_hip_joint_RH::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_elbow_joint_RH::Type_fr_base_X_fr_elbow_joint_RH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_hip_joint_RH;    // Maxima DSL: _k__tx_hip_joint_RH
    (*this)(1,1) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_elbow_joint_RH& HomogeneousTransforms::Type_fr_base_X_fr_elbow_joint_RH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    (*this)(1,0) = sin_q_hip_joint_RH;
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(1,3) = ( tx_elbow_joint_RH * sin_q_hip_joint_RH)+ ty_hip_joint_RH;
    (*this)(2,0) = -cos_q_hip_joint_RH;
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(2,3) = - tx_elbow_joint_RH * cos_q_hip_joint_RH;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_knee_joint_RH::Type_fr_base_X_fr_knee_joint_RH()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_knee_joint_RH& HomogeneousTransforms::Type_fr_base_X_fr_knee_joint_RH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    (*this)(0,0) = sin_q_elbow_joint_RH;
    (*this)(0,1) = cos_q_elbow_joint_RH;
    (*this)(0,3) = ( tx_knee_joint_RH * sin_q_elbow_joint_RH)+ tx_hip_joint_RH;
    (*this)(1,0) = cos_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,1) = -sin_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(1,3) = ((( tx_knee_joint_RH * cos_q_elbow_joint_RH)+ tx_elbow_joint_RH) * sin_q_hip_joint_RH)+ ty_hip_joint_RH;
    (*this)(2,0) = -cos_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(2,1) = sin_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(2,3) = ((- tx_knee_joint_RH * cos_q_elbow_joint_RH)- tx_elbow_joint_RH) * cos_q_hip_joint_RH;
    return *this;
}
HomogeneousTransforms::Type_imu_link_X_fr_hip_joint_LF::Type_imu_link_X_fr_hip_joint_LF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_hip_joint_LF;    // Maxima DSL: _k__tx_hip_joint_LF
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_hip_joint_LF;    // Maxima DSL: _k__ty_hip_joint_LF
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_imu_link_X_fr_hip_joint_LF& HomogeneousTransforms::Type_imu_link_X_fr_hip_joint_LF::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_imu_link_X_fr_elbow_joint_LF::Type_imu_link_X_fr_elbow_joint_LF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_hip_joint_LF;    // Maxima DSL: _k__tx_hip_joint_LF
    (*this)(1,1) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_imu_link_X_fr_elbow_joint_LF& HomogeneousTransforms::Type_imu_link_X_fr_elbow_joint_LF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    (*this)(1,0) = sin_q_hip_joint_LF;
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(1,3) = ( tx_elbow_joint_LF * sin_q_hip_joint_LF)+ ty_hip_joint_LF;
    (*this)(2,0) = -cos_q_hip_joint_LF;
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(2,3) = - tx_elbow_joint_LF * cos_q_hip_joint_LF;
    return *this;
}
HomogeneousTransforms::Type_imu_link_X_fr_knee_joint_LF::Type_imu_link_X_fr_knee_joint_LF()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_imu_link_X_fr_knee_joint_LF& HomogeneousTransforms::Type_imu_link_X_fr_knee_joint_LF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    (*this)(0,0) = sin_q_elbow_joint_LF;
    (*this)(0,1) = cos_q_elbow_joint_LF;
    (*this)(0,3) = ( tx_knee_joint_LF * sin_q_elbow_joint_LF)+ tx_hip_joint_LF;
    (*this)(1,0) = cos_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,1) = -sin_q_elbow_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,2) = -cos_q_hip_joint_LF;
    (*this)(1,3) = ((( tx_knee_joint_LF * cos_q_elbow_joint_LF)+ tx_elbow_joint_LF) * sin_q_hip_joint_LF)+ ty_hip_joint_LF;
    (*this)(2,0) = -cos_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(2,1) = sin_q_elbow_joint_LF * cos_q_hip_joint_LF;
    (*this)(2,2) = -sin_q_hip_joint_LF;
    (*this)(2,3) = ((- tx_knee_joint_LF * cos_q_elbow_joint_LF)- tx_elbow_joint_LF) * cos_q_hip_joint_LF;
    return *this;
}
HomogeneousTransforms::Type_imu_link_X_fr_hip_joint_RF::Type_imu_link_X_fr_hip_joint_RF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_hip_joint_RF;    // Maxima DSL: _k__tx_hip_joint_RF
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_hip_joint_RF;    // Maxima DSL: _k__ty_hip_joint_RF
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_imu_link_X_fr_hip_joint_RF& HomogeneousTransforms::Type_imu_link_X_fr_hip_joint_RF::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_imu_link_X_fr_elbow_joint_RF::Type_imu_link_X_fr_elbow_joint_RF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_hip_joint_RF;    // Maxima DSL: _k__tx_hip_joint_RF
    (*this)(1,1) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_imu_link_X_fr_elbow_joint_RF& HomogeneousTransforms::Type_imu_link_X_fr_elbow_joint_RF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    (*this)(1,0) = sin_q_hip_joint_RF;
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(1,3) = ( tx_elbow_joint_RF * sin_q_hip_joint_RF)+ ty_hip_joint_RF;
    (*this)(2,0) = -cos_q_hip_joint_RF;
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(2,3) = - tx_elbow_joint_RF * cos_q_hip_joint_RF;
    return *this;
}
HomogeneousTransforms::Type_imu_link_X_fr_knee_joint_RF::Type_imu_link_X_fr_knee_joint_RF()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_imu_link_X_fr_knee_joint_RF& HomogeneousTransforms::Type_imu_link_X_fr_knee_joint_RF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    (*this)(0,0) = sin_q_elbow_joint_RF;
    (*this)(0,1) = cos_q_elbow_joint_RF;
    (*this)(0,3) = ( tx_knee_joint_RF * sin_q_elbow_joint_RF)+ tx_hip_joint_RF;
    (*this)(1,0) = cos_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,1) = -sin_q_elbow_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,2) = -cos_q_hip_joint_RF;
    (*this)(1,3) = ((( tx_knee_joint_RF * cos_q_elbow_joint_RF)+ tx_elbow_joint_RF) * sin_q_hip_joint_RF)+ ty_hip_joint_RF;
    (*this)(2,0) = -cos_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(2,1) = sin_q_elbow_joint_RF * cos_q_hip_joint_RF;
    (*this)(2,2) = -sin_q_hip_joint_RF;
    (*this)(2,3) = ((- tx_knee_joint_RF * cos_q_elbow_joint_RF)- tx_elbow_joint_RF) * cos_q_hip_joint_RF;
    return *this;
}
HomogeneousTransforms::Type_imu_link_X_fr_hip_joint_LH::Type_imu_link_X_fr_hip_joint_LH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_hip_joint_LH;    // Maxima DSL: _k__tx_hip_joint_LH
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_hip_joint_LH;    // Maxima DSL: _k__ty_hip_joint_LH
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_imu_link_X_fr_hip_joint_LH& HomogeneousTransforms::Type_imu_link_X_fr_hip_joint_LH::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_imu_link_X_fr_elbow_joint_LH::Type_imu_link_X_fr_elbow_joint_LH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_hip_joint_LH;    // Maxima DSL: _k__tx_hip_joint_LH
    (*this)(1,1) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_imu_link_X_fr_elbow_joint_LH& HomogeneousTransforms::Type_imu_link_X_fr_elbow_joint_LH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    (*this)(1,0) = sin_q_hip_joint_LH;
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(1,3) = ( tx_elbow_joint_LH * sin_q_hip_joint_LH)+ ty_hip_joint_LH;
    (*this)(2,0) = -cos_q_hip_joint_LH;
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(2,3) = - tx_elbow_joint_LH * cos_q_hip_joint_LH;
    return *this;
}
HomogeneousTransforms::Type_imu_link_X_fr_knee_joint_LH::Type_imu_link_X_fr_knee_joint_LH()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_imu_link_X_fr_knee_joint_LH& HomogeneousTransforms::Type_imu_link_X_fr_knee_joint_LH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    (*this)(0,0) = sin_q_elbow_joint_LH;
    (*this)(0,1) = cos_q_elbow_joint_LH;
    (*this)(0,3) = ( tx_knee_joint_LH * sin_q_elbow_joint_LH)+ tx_hip_joint_LH;
    (*this)(1,0) = cos_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,1) = -sin_q_elbow_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,2) = -cos_q_hip_joint_LH;
    (*this)(1,3) = ((( tx_knee_joint_LH * cos_q_elbow_joint_LH)+ tx_elbow_joint_LH) * sin_q_hip_joint_LH)+ ty_hip_joint_LH;
    (*this)(2,0) = -cos_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(2,1) = sin_q_elbow_joint_LH * cos_q_hip_joint_LH;
    (*this)(2,2) = -sin_q_hip_joint_LH;
    (*this)(2,3) = ((- tx_knee_joint_LH * cos_q_elbow_joint_LH)- tx_elbow_joint_LH) * cos_q_hip_joint_LH;
    return *this;
}
HomogeneousTransforms::Type_imu_link_X_fr_hip_joint_RH::Type_imu_link_X_fr_hip_joint_RH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_hip_joint_RH;    // Maxima DSL: _k__tx_hip_joint_RH
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_hip_joint_RH;    // Maxima DSL: _k__ty_hip_joint_RH
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_imu_link_X_fr_hip_joint_RH& HomogeneousTransforms::Type_imu_link_X_fr_hip_joint_RH::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_imu_link_X_fr_elbow_joint_RH::Type_imu_link_X_fr_elbow_joint_RH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_hip_joint_RH;    // Maxima DSL: _k__tx_hip_joint_RH
    (*this)(1,1) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_imu_link_X_fr_elbow_joint_RH& HomogeneousTransforms::Type_imu_link_X_fr_elbow_joint_RH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    (*this)(1,0) = sin_q_hip_joint_RH;
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(1,3) = ( tx_elbow_joint_RH * sin_q_hip_joint_RH)+ ty_hip_joint_RH;
    (*this)(2,0) = -cos_q_hip_joint_RH;
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(2,3) = - tx_elbow_joint_RH * cos_q_hip_joint_RH;
    return *this;
}
HomogeneousTransforms::Type_imu_link_X_fr_knee_joint_RH::Type_imu_link_X_fr_knee_joint_RH()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_imu_link_X_fr_knee_joint_RH& HomogeneousTransforms::Type_imu_link_X_fr_knee_joint_RH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    (*this)(0,0) = sin_q_elbow_joint_RH;
    (*this)(0,1) = cos_q_elbow_joint_RH;
    (*this)(0,3) = ( tx_knee_joint_RH * sin_q_elbow_joint_RH)+ tx_hip_joint_RH;
    (*this)(1,0) = cos_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,1) = -sin_q_elbow_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,2) = -cos_q_hip_joint_RH;
    (*this)(1,3) = ((( tx_knee_joint_RH * cos_q_elbow_joint_RH)+ tx_elbow_joint_RH) * sin_q_hip_joint_RH)+ ty_hip_joint_RH;
    (*this)(2,0) = -cos_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(2,1) = sin_q_elbow_joint_RH * cos_q_hip_joint_RH;
    (*this)(2,2) = -sin_q_hip_joint_RH;
    (*this)(2,3) = ((- tx_knee_joint_RH * cos_q_elbow_joint_RH)- tx_elbow_joint_RH) * cos_q_hip_joint_RH;
    return *this;
}
HomogeneousTransforms::Type_fr_hip_link_LF_X_fr_base::Type_fr_hip_link_LF_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tx_hip_joint_LF;    // Maxima DSL: -_k__tx_hip_joint_LF
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_hip_link_LF_X_fr_base& HomogeneousTransforms::Type_fr_hip_link_LF_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    (*this)(0,1) = sin_q_hip_joint_LF;
    (*this)(0,2) = -cos_q_hip_joint_LF;
    (*this)(0,3) = - ty_hip_joint_LF * sin_q_hip_joint_LF;
    (*this)(1,1) = cos_q_hip_joint_LF;
    (*this)(1,2) = sin_q_hip_joint_LF;
    (*this)(1,3) = - ty_hip_joint_LF * cos_q_hip_joint_LF;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_hip_link_LF::Type_fr_base_X_fr_hip_link_LF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_hip_joint_LF;    // Maxima DSL: _k__tx_hip_joint_LF
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_hip_joint_LF;    // Maxima DSL: _k__ty_hip_joint_LF
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_hip_link_LF& HomogeneousTransforms::Type_fr_base_X_fr_hip_link_LF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LF  = ScalarTraits::sin( q(HIP_JOINT_LF) );
    Scalar cos_q_hip_joint_LF  = ScalarTraits::cos( q(HIP_JOINT_LF) );
    (*this)(1,0) = sin_q_hip_joint_LF;
    (*this)(1,1) = cos_q_hip_joint_LF;
    (*this)(2,0) = -cos_q_hip_joint_LF;
    (*this)(2,1) = sin_q_hip_joint_LF;
    return *this;
}
HomogeneousTransforms::Type_fr_thigh_link_LF_X_fr_hip_link_LF::Type_fr_thigh_link_LF_X_fr_hip_link_LF()
{
    (*this)(0,1) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_thigh_link_LF_X_fr_hip_link_LF& HomogeneousTransforms::Type_fr_thigh_link_LF_X_fr_hip_link_LF::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    (*this)(0,0) = cos_q_elbow_joint_LF;
    (*this)(0,2) = sin_q_elbow_joint_LF;
    (*this)(0,3) = - tx_elbow_joint_LF * cos_q_elbow_joint_LF;
    (*this)(1,0) = -sin_q_elbow_joint_LF;
    (*this)(1,2) = cos_q_elbow_joint_LF;
    (*this)(1,3) =  tx_elbow_joint_LF * sin_q_elbow_joint_LF;
    return *this;
}
HomogeneousTransforms::Type_fr_hip_link_LF_X_fr_thigh_link_LF::Type_fr_hip_link_LF_X_fr_thigh_link_LF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_elbow_joint_LF;    // Maxima DSL: _k__tx_elbow_joint_LF
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_hip_link_LF_X_fr_thigh_link_LF& HomogeneousTransforms::Type_fr_hip_link_LF_X_fr_thigh_link_LF::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_LF  = ScalarTraits::sin( q(ELBOW_JOINT_LF) );
    Scalar cos_q_elbow_joint_LF  = ScalarTraits::cos( q(ELBOW_JOINT_LF) );
    (*this)(0,0) = cos_q_elbow_joint_LF;
    (*this)(0,1) = -sin_q_elbow_joint_LF;
    (*this)(2,0) = sin_q_elbow_joint_LF;
    (*this)(2,1) = cos_q_elbow_joint_LF;
    return *this;
}
HomogeneousTransforms::Type_fr_shank_link_LF_X_fr_thigh_link_LF::Type_fr_shank_link_LF_X_fr_thigh_link_LF()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_shank_link_LF_X_fr_thigh_link_LF& HomogeneousTransforms::Type_fr_shank_link_LF_X_fr_thigh_link_LF::update(const state_t& q)
{
    Scalar sin_q_knee_joint_LF  = ScalarTraits::sin( q(KNEE_JOINT_LF) );
    Scalar cos_q_knee_joint_LF  = ScalarTraits::cos( q(KNEE_JOINT_LF) );
    (*this)(0,0) = cos_q_knee_joint_LF;
    (*this)(0,1) = sin_q_knee_joint_LF;
    (*this)(0,3) = - tx_knee_joint_LF * cos_q_knee_joint_LF;
    (*this)(1,0) = -sin_q_knee_joint_LF;
    (*this)(1,1) = cos_q_knee_joint_LF;
    (*this)(1,3) =  tx_knee_joint_LF * sin_q_knee_joint_LF;
    return *this;
}
HomogeneousTransforms::Type_fr_thigh_link_LF_X_fr_shank_link_LF::Type_fr_thigh_link_LF_X_fr_shank_link_LF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_knee_joint_LF;    // Maxima DSL: _k__tx_knee_joint_LF
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_thigh_link_LF_X_fr_shank_link_LF& HomogeneousTransforms::Type_fr_thigh_link_LF_X_fr_shank_link_LF::update(const state_t& q)
{
    Scalar sin_q_knee_joint_LF  = ScalarTraits::sin( q(KNEE_JOINT_LF) );
    Scalar cos_q_knee_joint_LF  = ScalarTraits::cos( q(KNEE_JOINT_LF) );
    (*this)(0,0) = cos_q_knee_joint_LF;
    (*this)(0,1) = -sin_q_knee_joint_LF;
    (*this)(1,0) = sin_q_knee_joint_LF;
    (*this)(1,1) = cos_q_knee_joint_LF;
    return *this;
}
HomogeneousTransforms::Type_fr_hip_link_RF_X_fr_base::Type_fr_hip_link_RF_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tx_hip_joint_RF;    // Maxima DSL: -_k__tx_hip_joint_RF
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_hip_link_RF_X_fr_base& HomogeneousTransforms::Type_fr_hip_link_RF_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    (*this)(0,1) = sin_q_hip_joint_RF;
    (*this)(0,2) = -cos_q_hip_joint_RF;
    (*this)(0,3) = - ty_hip_joint_RF * sin_q_hip_joint_RF;
    (*this)(1,1) = cos_q_hip_joint_RF;
    (*this)(1,2) = sin_q_hip_joint_RF;
    (*this)(1,3) = - ty_hip_joint_RF * cos_q_hip_joint_RF;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_hip_link_RF::Type_fr_base_X_fr_hip_link_RF()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_hip_joint_RF;    // Maxima DSL: _k__tx_hip_joint_RF
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_hip_joint_RF;    // Maxima DSL: _k__ty_hip_joint_RF
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_hip_link_RF& HomogeneousTransforms::Type_fr_base_X_fr_hip_link_RF::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RF  = ScalarTraits::sin( q(HIP_JOINT_RF) );
    Scalar cos_q_hip_joint_RF  = ScalarTraits::cos( q(HIP_JOINT_RF) );
    (*this)(1,0) = sin_q_hip_joint_RF;
    (*this)(1,1) = cos_q_hip_joint_RF;
    (*this)(2,0) = -cos_q_hip_joint_RF;
    (*this)(2,1) = sin_q_hip_joint_RF;
    return *this;
}
HomogeneousTransforms::Type_fr_thigh_link_RF_X_fr_hip_link_RF::Type_fr_thigh_link_RF_X_fr_hip_link_RF()
{
    (*this)(0,1) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_thigh_link_RF_X_fr_hip_link_RF& HomogeneousTransforms::Type_fr_thigh_link_RF_X_fr_hip_link_RF::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    (*this)(0,0) = cos_q_elbow_joint_RF;
    (*this)(0,2) = sin_q_elbow_joint_RF;
    (*this)(0,3) = - tx_elbow_joint_RF * cos_q_elbow_joint_RF;
    (*this)(1,0) = -sin_q_elbow_joint_RF;
    (*this)(1,2) = cos_q_elbow_joint_RF;
    (*this)(1,3) =  tx_elbow_joint_RF * sin_q_elbow_joint_RF;
    return *this;
}
HomogeneousTransforms::Type_fr_hip_link_RF_X_fr_thigh_link_RF::Type_fr_hip_link_RF_X_fr_thigh_link_RF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_elbow_joint_RF;    // Maxima DSL: _k__tx_elbow_joint_RF
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_hip_link_RF_X_fr_thigh_link_RF& HomogeneousTransforms::Type_fr_hip_link_RF_X_fr_thigh_link_RF::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_RF  = ScalarTraits::sin( q(ELBOW_JOINT_RF) );
    Scalar cos_q_elbow_joint_RF  = ScalarTraits::cos( q(ELBOW_JOINT_RF) );
    (*this)(0,0) = cos_q_elbow_joint_RF;
    (*this)(0,1) = -sin_q_elbow_joint_RF;
    (*this)(2,0) = sin_q_elbow_joint_RF;
    (*this)(2,1) = cos_q_elbow_joint_RF;
    return *this;
}
HomogeneousTransforms::Type_fr_shank_link_RF_X_fr_thigh_link_RF::Type_fr_shank_link_RF_X_fr_thigh_link_RF()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_shank_link_RF_X_fr_thigh_link_RF& HomogeneousTransforms::Type_fr_shank_link_RF_X_fr_thigh_link_RF::update(const state_t& q)
{
    Scalar sin_q_knee_joint_RF  = ScalarTraits::sin( q(KNEE_JOINT_RF) );
    Scalar cos_q_knee_joint_RF  = ScalarTraits::cos( q(KNEE_JOINT_RF) );
    (*this)(0,0) = cos_q_knee_joint_RF;
    (*this)(0,1) = sin_q_knee_joint_RF;
    (*this)(0,3) = - tx_knee_joint_RF * cos_q_knee_joint_RF;
    (*this)(1,0) = -sin_q_knee_joint_RF;
    (*this)(1,1) = cos_q_knee_joint_RF;
    (*this)(1,3) =  tx_knee_joint_RF * sin_q_knee_joint_RF;
    return *this;
}
HomogeneousTransforms::Type_fr_thigh_link_RF_X_fr_shank_link_RF::Type_fr_thigh_link_RF_X_fr_shank_link_RF()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_knee_joint_RF;    // Maxima DSL: _k__tx_knee_joint_RF
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_thigh_link_RF_X_fr_shank_link_RF& HomogeneousTransforms::Type_fr_thigh_link_RF_X_fr_shank_link_RF::update(const state_t& q)
{
    Scalar sin_q_knee_joint_RF  = ScalarTraits::sin( q(KNEE_JOINT_RF) );
    Scalar cos_q_knee_joint_RF  = ScalarTraits::cos( q(KNEE_JOINT_RF) );
    (*this)(0,0) = cos_q_knee_joint_RF;
    (*this)(0,1) = -sin_q_knee_joint_RF;
    (*this)(1,0) = sin_q_knee_joint_RF;
    (*this)(1,1) = cos_q_knee_joint_RF;
    return *this;
}
HomogeneousTransforms::Type_fr_hip_link_LH_X_fr_base::Type_fr_hip_link_LH_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tx_hip_joint_LH;    // Maxima DSL: -_k__tx_hip_joint_LH
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_hip_link_LH_X_fr_base& HomogeneousTransforms::Type_fr_hip_link_LH_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    (*this)(0,1) = sin_q_hip_joint_LH;
    (*this)(0,2) = -cos_q_hip_joint_LH;
    (*this)(0,3) = - ty_hip_joint_LH * sin_q_hip_joint_LH;
    (*this)(1,1) = cos_q_hip_joint_LH;
    (*this)(1,2) = sin_q_hip_joint_LH;
    (*this)(1,3) = - ty_hip_joint_LH * cos_q_hip_joint_LH;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_hip_link_LH::Type_fr_base_X_fr_hip_link_LH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_hip_joint_LH;    // Maxima DSL: _k__tx_hip_joint_LH
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_hip_joint_LH;    // Maxima DSL: _k__ty_hip_joint_LH
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_hip_link_LH& HomogeneousTransforms::Type_fr_base_X_fr_hip_link_LH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_LH  = ScalarTraits::sin( q(HIP_JOINT_LH) );
    Scalar cos_q_hip_joint_LH  = ScalarTraits::cos( q(HIP_JOINT_LH) );
    (*this)(1,0) = sin_q_hip_joint_LH;
    (*this)(1,1) = cos_q_hip_joint_LH;
    (*this)(2,0) = -cos_q_hip_joint_LH;
    (*this)(2,1) = sin_q_hip_joint_LH;
    return *this;
}
HomogeneousTransforms::Type_fr_thigh_link_LH_X_fr_hip_link_LH::Type_fr_thigh_link_LH_X_fr_hip_link_LH()
{
    (*this)(0,1) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_thigh_link_LH_X_fr_hip_link_LH& HomogeneousTransforms::Type_fr_thigh_link_LH_X_fr_hip_link_LH::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    (*this)(0,0) = cos_q_elbow_joint_LH;
    (*this)(0,2) = sin_q_elbow_joint_LH;
    (*this)(0,3) = - tx_elbow_joint_LH * cos_q_elbow_joint_LH;
    (*this)(1,0) = -sin_q_elbow_joint_LH;
    (*this)(1,2) = cos_q_elbow_joint_LH;
    (*this)(1,3) =  tx_elbow_joint_LH * sin_q_elbow_joint_LH;
    return *this;
}
HomogeneousTransforms::Type_fr_hip_link_LH_X_fr_thigh_link_LH::Type_fr_hip_link_LH_X_fr_thigh_link_LH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_elbow_joint_LH;    // Maxima DSL: _k__tx_elbow_joint_LH
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_hip_link_LH_X_fr_thigh_link_LH& HomogeneousTransforms::Type_fr_hip_link_LH_X_fr_thigh_link_LH::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_LH  = ScalarTraits::sin( q(ELBOW_JOINT_LH) );
    Scalar cos_q_elbow_joint_LH  = ScalarTraits::cos( q(ELBOW_JOINT_LH) );
    (*this)(0,0) = cos_q_elbow_joint_LH;
    (*this)(0,1) = -sin_q_elbow_joint_LH;
    (*this)(2,0) = sin_q_elbow_joint_LH;
    (*this)(2,1) = cos_q_elbow_joint_LH;
    return *this;
}
HomogeneousTransforms::Type_fr_shank_link_LH_X_fr_thigh_link_LH::Type_fr_shank_link_LH_X_fr_thigh_link_LH()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_shank_link_LH_X_fr_thigh_link_LH& HomogeneousTransforms::Type_fr_shank_link_LH_X_fr_thigh_link_LH::update(const state_t& q)
{
    Scalar sin_q_knee_joint_LH  = ScalarTraits::sin( q(KNEE_JOINT_LH) );
    Scalar cos_q_knee_joint_LH  = ScalarTraits::cos( q(KNEE_JOINT_LH) );
    (*this)(0,0) = cos_q_knee_joint_LH;
    (*this)(0,1) = sin_q_knee_joint_LH;
    (*this)(0,3) = - tx_knee_joint_LH * cos_q_knee_joint_LH;
    (*this)(1,0) = -sin_q_knee_joint_LH;
    (*this)(1,1) = cos_q_knee_joint_LH;
    (*this)(1,3) =  tx_knee_joint_LH * sin_q_knee_joint_LH;
    return *this;
}
HomogeneousTransforms::Type_fr_thigh_link_LH_X_fr_shank_link_LH::Type_fr_thigh_link_LH_X_fr_shank_link_LH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_knee_joint_LH;    // Maxima DSL: _k__tx_knee_joint_LH
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_thigh_link_LH_X_fr_shank_link_LH& HomogeneousTransforms::Type_fr_thigh_link_LH_X_fr_shank_link_LH::update(const state_t& q)
{
    Scalar sin_q_knee_joint_LH  = ScalarTraits::sin( q(KNEE_JOINT_LH) );
    Scalar cos_q_knee_joint_LH  = ScalarTraits::cos( q(KNEE_JOINT_LH) );
    (*this)(0,0) = cos_q_knee_joint_LH;
    (*this)(0,1) = -sin_q_knee_joint_LH;
    (*this)(1,0) = sin_q_knee_joint_LH;
    (*this)(1,1) = cos_q_knee_joint_LH;
    return *this;
}
HomogeneousTransforms::Type_fr_hip_link_RH_X_fr_base::Type_fr_hip_link_RH_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tx_hip_joint_RH;    // Maxima DSL: -_k__tx_hip_joint_RH
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_hip_link_RH_X_fr_base& HomogeneousTransforms::Type_fr_hip_link_RH_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    (*this)(0,1) = sin_q_hip_joint_RH;
    (*this)(0,2) = -cos_q_hip_joint_RH;
    (*this)(0,3) = - ty_hip_joint_RH * sin_q_hip_joint_RH;
    (*this)(1,1) = cos_q_hip_joint_RH;
    (*this)(1,2) = sin_q_hip_joint_RH;
    (*this)(1,3) = - ty_hip_joint_RH * cos_q_hip_joint_RH;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_hip_link_RH::Type_fr_base_X_fr_hip_link_RH()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_hip_joint_RH;    // Maxima DSL: _k__tx_hip_joint_RH
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_hip_joint_RH;    // Maxima DSL: _k__ty_hip_joint_RH
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_hip_link_RH& HomogeneousTransforms::Type_fr_base_X_fr_hip_link_RH::update(const state_t& q)
{
    Scalar sin_q_hip_joint_RH  = ScalarTraits::sin( q(HIP_JOINT_RH) );
    Scalar cos_q_hip_joint_RH  = ScalarTraits::cos( q(HIP_JOINT_RH) );
    (*this)(1,0) = sin_q_hip_joint_RH;
    (*this)(1,1) = cos_q_hip_joint_RH;
    (*this)(2,0) = -cos_q_hip_joint_RH;
    (*this)(2,1) = sin_q_hip_joint_RH;
    return *this;
}
HomogeneousTransforms::Type_fr_thigh_link_RH_X_fr_hip_link_RH::Type_fr_thigh_link_RH_X_fr_hip_link_RH()
{
    (*this)(0,1) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_thigh_link_RH_X_fr_hip_link_RH& HomogeneousTransforms::Type_fr_thigh_link_RH_X_fr_hip_link_RH::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    (*this)(0,0) = cos_q_elbow_joint_RH;
    (*this)(0,2) = sin_q_elbow_joint_RH;
    (*this)(0,3) = - tx_elbow_joint_RH * cos_q_elbow_joint_RH;
    (*this)(1,0) = -sin_q_elbow_joint_RH;
    (*this)(1,2) = cos_q_elbow_joint_RH;
    (*this)(1,3) =  tx_elbow_joint_RH * sin_q_elbow_joint_RH;
    return *this;
}
HomogeneousTransforms::Type_fr_hip_link_RH_X_fr_thigh_link_RH::Type_fr_hip_link_RH_X_fr_thigh_link_RH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_elbow_joint_RH;    // Maxima DSL: _k__tx_elbow_joint_RH
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_hip_link_RH_X_fr_thigh_link_RH& HomogeneousTransforms::Type_fr_hip_link_RH_X_fr_thigh_link_RH::update(const state_t& q)
{
    Scalar sin_q_elbow_joint_RH  = ScalarTraits::sin( q(ELBOW_JOINT_RH) );
    Scalar cos_q_elbow_joint_RH  = ScalarTraits::cos( q(ELBOW_JOINT_RH) );
    (*this)(0,0) = cos_q_elbow_joint_RH;
    (*this)(0,1) = -sin_q_elbow_joint_RH;
    (*this)(2,0) = sin_q_elbow_joint_RH;
    (*this)(2,1) = cos_q_elbow_joint_RH;
    return *this;
}
HomogeneousTransforms::Type_fr_shank_link_RH_X_fr_thigh_link_RH::Type_fr_shank_link_RH_X_fr_thigh_link_RH()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_shank_link_RH_X_fr_thigh_link_RH& HomogeneousTransforms::Type_fr_shank_link_RH_X_fr_thigh_link_RH::update(const state_t& q)
{
    Scalar sin_q_knee_joint_RH  = ScalarTraits::sin( q(KNEE_JOINT_RH) );
    Scalar cos_q_knee_joint_RH  = ScalarTraits::cos( q(KNEE_JOINT_RH) );
    (*this)(0,0) = cos_q_knee_joint_RH;
    (*this)(0,1) = sin_q_knee_joint_RH;
    (*this)(0,3) = - tx_knee_joint_RH * cos_q_knee_joint_RH;
    (*this)(1,0) = -sin_q_knee_joint_RH;
    (*this)(1,1) = cos_q_knee_joint_RH;
    (*this)(1,3) =  tx_knee_joint_RH * sin_q_knee_joint_RH;
    return *this;
}
HomogeneousTransforms::Type_fr_thigh_link_RH_X_fr_shank_link_RH::Type_fr_thigh_link_RH_X_fr_shank_link_RH()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_knee_joint_RH;    // Maxima DSL: _k__tx_knee_joint_RH
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_thigh_link_RH_X_fr_shank_link_RH& HomogeneousTransforms::Type_fr_thigh_link_RH_X_fr_shank_link_RH::update(const state_t& q)
{
    Scalar sin_q_knee_joint_RH  = ScalarTraits::sin( q(KNEE_JOINT_RH) );
    Scalar cos_q_knee_joint_RH  = ScalarTraits::cos( q(KNEE_JOINT_RH) );
    (*this)(0,0) = cos_q_knee_joint_RH;
    (*this)(0,1) = -sin_q_knee_joint_RH;
    (*this)(1,0) = sin_q_knee_joint_RH;
    (*this)(1,1) = cos_q_knee_joint_RH;
    return *this;
}

