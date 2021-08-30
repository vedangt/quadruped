#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace rcg::shvan1;
using namespace rcg::shvan1::dyn;

Vector3 rcg::shvan1::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    Vector3 tmpSum(Vector3::Zero());

    tmpSum += inertiaProps.getCOM_base() * inertiaProps.getMass_base();

    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    HomogeneousTransforms::MatrixType base_X_hip_joint_LF_chain;
    HomogeneousTransforms::MatrixType base_X_hip_joint_RF_chain;
    HomogeneousTransforms::MatrixType base_X_hip_joint_LH_chain;
    HomogeneousTransforms::MatrixType base_X_hip_joint_RH_chain;
    
    
    base_X_hip_joint_LF_chain = tmpX * ht.fr_base_X_fr_hip_link_LF;
    tmpSum += inertiaProps.getMass_hip_link_LF() *
            ( iit::rbd::Utils::transform(base_X_hip_joint_LF_chain, inertiaProps.getCOM_hip_link_LF()));
    
    base_X_hip_joint_LF_chain = base_X_hip_joint_LF_chain * ht.fr_hip_link_LF_X_fr_thigh_link_LF;
    tmpSum += inertiaProps.getMass_thigh_link_LF() *
            ( iit::rbd::Utils::transform(base_X_hip_joint_LF_chain, inertiaProps.getCOM_thigh_link_LF()));
    
    base_X_hip_joint_LF_chain = base_X_hip_joint_LF_chain * ht.fr_thigh_link_LF_X_fr_shank_link_LF;
    tmpSum += inertiaProps.getMass_shank_link_LF() *
            ( iit::rbd::Utils::transform(base_X_hip_joint_LF_chain, inertiaProps.getCOM_shank_link_LF()));
    
    base_X_hip_joint_RF_chain = tmpX * ht.fr_base_X_fr_hip_link_RF;
    tmpSum += inertiaProps.getMass_hip_link_RF() *
            ( iit::rbd::Utils::transform(base_X_hip_joint_RF_chain, inertiaProps.getCOM_hip_link_RF()));
    
    base_X_hip_joint_RF_chain = base_X_hip_joint_RF_chain * ht.fr_hip_link_RF_X_fr_thigh_link_RF;
    tmpSum += inertiaProps.getMass_thigh_link_RF() *
            ( iit::rbd::Utils::transform(base_X_hip_joint_RF_chain, inertiaProps.getCOM_thigh_link_RF()));
    
    base_X_hip_joint_RF_chain = base_X_hip_joint_RF_chain * ht.fr_thigh_link_RF_X_fr_shank_link_RF;
    tmpSum += inertiaProps.getMass_shank_link_RF() *
            ( iit::rbd::Utils::transform(base_X_hip_joint_RF_chain, inertiaProps.getCOM_shank_link_RF()));
    
    base_X_hip_joint_LH_chain = tmpX * ht.fr_base_X_fr_hip_link_LH;
    tmpSum += inertiaProps.getMass_hip_link_LH() *
            ( iit::rbd::Utils::transform(base_X_hip_joint_LH_chain, inertiaProps.getCOM_hip_link_LH()));
    
    base_X_hip_joint_LH_chain = base_X_hip_joint_LH_chain * ht.fr_hip_link_LH_X_fr_thigh_link_LH;
    tmpSum += inertiaProps.getMass_thigh_link_LH() *
            ( iit::rbd::Utils::transform(base_X_hip_joint_LH_chain, inertiaProps.getCOM_thigh_link_LH()));
    
    base_X_hip_joint_LH_chain = base_X_hip_joint_LH_chain * ht.fr_thigh_link_LH_X_fr_shank_link_LH;
    tmpSum += inertiaProps.getMass_shank_link_LH() *
            ( iit::rbd::Utils::transform(base_X_hip_joint_LH_chain, inertiaProps.getCOM_shank_link_LH()));
    
    base_X_hip_joint_RH_chain = tmpX * ht.fr_base_X_fr_hip_link_RH;
    tmpSum += inertiaProps.getMass_hip_link_RH() *
            ( iit::rbd::Utils::transform(base_X_hip_joint_RH_chain, inertiaProps.getCOM_hip_link_RH()));
    
    base_X_hip_joint_RH_chain = base_X_hip_joint_RH_chain * ht.fr_hip_link_RH_X_fr_thigh_link_RH;
    tmpSum += inertiaProps.getMass_thigh_link_RH() *
            ( iit::rbd::Utils::transform(base_X_hip_joint_RH_chain, inertiaProps.getCOM_thigh_link_RH()));
    
    base_X_hip_joint_RH_chain = base_X_hip_joint_RH_chain * ht.fr_thigh_link_RH_X_fr_shank_link_RH;
    tmpSum += inertiaProps.getMass_shank_link_RH() *
            ( iit::rbd::Utils::transform(base_X_hip_joint_RH_chain, inertiaProps.getCOM_shank_link_RH()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

Vector3 rcg::shvan1::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_base_X_fr_hip_link_LF(q);
    ht.fr_base_X_fr_hip_link_RF(q);
    ht.fr_base_X_fr_hip_link_LH(q);
    ht.fr_base_X_fr_hip_link_RH(q);
    ht.fr_hip_link_LF_X_fr_thigh_link_LF(q);
    ht.fr_thigh_link_LF_X_fr_shank_link_LF(q);
    ht.fr_hip_link_RF_X_fr_thigh_link_RF(q);
    ht.fr_thigh_link_RF_X_fr_shank_link_RF(q);
    ht.fr_hip_link_LH_X_fr_thigh_link_LH(q);
    ht.fr_thigh_link_LH_X_fr_shank_link_LH(q);
    ht.fr_hip_link_RH_X_fr_thigh_link_RH(q);
    ht.fr_thigh_link_RH_X_fr_shank_link_RH(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
