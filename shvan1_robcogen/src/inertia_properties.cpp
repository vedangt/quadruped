#include "inertia_properties.h"

using namespace std;
using namespace iit::rbd;

rcg::shvan1::dyn::InertiaProperties::InertiaProperties()
{
    com_base = Vector3(0.0,0.0,0.0);
    tensor_base.fill(
        m_base,
        com_base,
        Utils::buildInertiaTensor<Scalar>(ix_base,iy_base,iz_base,0.0,0.0,0.0) );

    com_hip_link_LF = Vector3(comx_hip_link_LF,0.0,0.0);
    tensor_hip_link_LF.fill(
        m_hip_link_LF,
        com_hip_link_LF,
        Utils::buildInertiaTensor<Scalar>(ix_hip_link_LF,iy_hip_link_LF,iz_hip_link_LF,0.0,0.0,0.0) );

    com_thigh_link_LF = Vector3(comx_thigh_link_LF,0.0,0.0);
    tensor_thigh_link_LF.fill(
        m_thigh_link_LF,
        com_thigh_link_LF,
        Utils::buildInertiaTensor<Scalar>(ix_thigh_link_LF,iy_thigh_link_LF,iz_thigh_link_LF,0.0,0.0,0.0) );

    com_shank_link_LF = Vector3(comx_shank_link_LF,0.0,0.0);
    tensor_shank_link_LF.fill(
        m_shank_link_LF,
        com_shank_link_LF,
        Utils::buildInertiaTensor<Scalar>(ix_shank_link_LF,iy_shank_link_LF,iz_shank_link_LF,0.0,0.0,0.0) );

    com_hip_link_RF = Vector3(comx_hip_link_RF,0.0,0.0);
    tensor_hip_link_RF.fill(
        m_hip_link_RF,
        com_hip_link_RF,
        Utils::buildInertiaTensor<Scalar>(ix_hip_link_RF,iy_hip_link_RF,iz_hip_link_RF,0.0,0.0,0.0) );

    com_thigh_link_RF = Vector3(comx_thigh_link_RF,0.0,0.0);
    tensor_thigh_link_RF.fill(
        m_thigh_link_RF,
        com_thigh_link_RF,
        Utils::buildInertiaTensor<Scalar>(ix_thigh_link_RF,iy_thigh_link_RF,iz_thigh_link_RF,0.0,0.0,0.0) );

    com_shank_link_RF = Vector3(comx_shank_link_RF,0.0,0.0);
    tensor_shank_link_RF.fill(
        m_shank_link_RF,
        com_shank_link_RF,
        Utils::buildInertiaTensor<Scalar>(ix_shank_link_RF,iy_shank_link_RF,iz_shank_link_RF,0.0,0.0,0.0) );

    com_hip_link_LH = Vector3(comx_hip_link_LH,0.0,0.0);
    tensor_hip_link_LH.fill(
        m_hip_link_LH,
        com_hip_link_LH,
        Utils::buildInertiaTensor<Scalar>(ix_hip_link_LH,iy_hip_link_LH,iz_hip_link_LH,0.0,0.0,0.0) );

    com_thigh_link_LH = Vector3(comx_thigh_link_LH,0.0,0.0);
    tensor_thigh_link_LH.fill(
        m_thigh_link_LH,
        com_thigh_link_LH,
        Utils::buildInertiaTensor<Scalar>(ix_thigh_link_LH,iy_thigh_link_LH,iz_thigh_link_LH,0.0,0.0,0.0) );

    com_shank_link_LH = Vector3(comx_shank_link_LH,0.0,0.0);
    tensor_shank_link_LH.fill(
        m_shank_link_LH,
        com_shank_link_LH,
        Utils::buildInertiaTensor<Scalar>(ix_shank_link_LH,iy_shank_link_LH,iz_shank_link_LH,0.0,0.0,0.0) );

    com_hip_link_RH = Vector3(comx_hip_link_RH,0.0,0.0);
    tensor_hip_link_RH.fill(
        m_hip_link_RH,
        com_hip_link_RH,
        Utils::buildInertiaTensor<Scalar>(ix_hip_link_RH,iy_hip_link_RH,iz_hip_link_RH,0.0,0.0,0.0) );

    com_thigh_link_RH = Vector3(comx_thigh_link_RH,0.0,0.0);
    tensor_thigh_link_RH.fill(
        m_thigh_link_RH,
        com_thigh_link_RH,
        Utils::buildInertiaTensor<Scalar>(ix_thigh_link_RH,iy_thigh_link_RH,iz_thigh_link_RH,0.0,0.0,0.0) );

    com_shank_link_RH = Vector3(comx_shank_link_RH,0.0,0.0);
    tensor_shank_link_RH.fill(
        m_shank_link_RH,
        com_shank_link_RH,
        Utils::buildInertiaTensor<Scalar>(ix_shank_link_RH,iy_shank_link_RH,iz_shank_link_RH,0.0,0.0,0.0) );

}


void rcg::shvan1::dyn::InertiaProperties::updateParameters(const RuntimeInertiaParams& fresh)
{
    this-> params = fresh;
}
