#include "transforms.h"
#include "jsim.h"

#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

//Implementation of default constructor
rcg::shvan1::dyn::JSIM::JSIM(InertiaProperties& inertiaProperties, ForceTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    shank_link_LF_Ic(linkInertias.getTensor_shank_link_LF()),
    shank_link_RF_Ic(linkInertias.getTensor_shank_link_RF()),
    shank_link_LH_Ic(linkInertias.getTensor_shank_link_LH()),
    shank_link_RH_Ic(linkInertias.getTensor_shank_link_RH())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA operator()
#define Fcol(j) (block<6,1>(0,(j)+6))
#define F(i,j) DATA((i),(j)+6)
const rcg::shvan1::dyn::JSIM& rcg::shvan1::dyn::JSIM::update(const JointState& state) {

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_thigh_link_RH_X_fr_shank_link_RH(state);
    frcTransf -> fr_hip_link_RH_X_fr_thigh_link_RH(state);
    frcTransf -> fr_base_X_fr_hip_link_RH(state);
    frcTransf -> fr_thigh_link_LH_X_fr_shank_link_LH(state);
    frcTransf -> fr_hip_link_LH_X_fr_thigh_link_LH(state);
    frcTransf -> fr_base_X_fr_hip_link_LH(state);
    frcTransf -> fr_thigh_link_RF_X_fr_shank_link_RF(state);
    frcTransf -> fr_hip_link_RF_X_fr_thigh_link_RF(state);
    frcTransf -> fr_base_X_fr_hip_link_RF(state);
    frcTransf -> fr_thigh_link_LF_X_fr_shank_link_LF(state);
    frcTransf -> fr_hip_link_LF_X_fr_thigh_link_LF(state);
    frcTransf -> fr_base_X_fr_hip_link_LF(state);

    // Initializes the composite inertia tensors
    base_Ic = linkInertias.getTensor_base();
    hip_link_LF_Ic = linkInertias.getTensor_hip_link_LF();
    thigh_link_LF_Ic = linkInertias.getTensor_thigh_link_LF();
    hip_link_RF_Ic = linkInertias.getTensor_hip_link_RF();
    thigh_link_RF_Ic = linkInertias.getTensor_thigh_link_RF();
    hip_link_LH_Ic = linkInertias.getTensor_hip_link_LH();
    thigh_link_LH_Ic = linkInertias.getTensor_thigh_link_LH();
    hip_link_RH_Ic = linkInertias.getTensor_hip_link_RH();
    thigh_link_RH_Ic = linkInertias.getTensor_thigh_link_RH();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link shank_link_RH:
    iit::rbd::transformInertia<Scalar>(shank_link_RH_Ic, frcTransf -> fr_thigh_link_RH_X_fr_shank_link_RH, Ic_spare);
    thigh_link_RH_Ic += Ic_spare;

    Fcol(KNEE_JOINT_RH) = shank_link_RH_Ic.col(AZ);
    DATA(KNEE_JOINT_RH+6, KNEE_JOINT_RH+6) = Fcol(KNEE_JOINT_RH)(AZ);

    Fcol(KNEE_JOINT_RH) = frcTransf -> fr_thigh_link_RH_X_fr_shank_link_RH * Fcol(KNEE_JOINT_RH);
    DATA(KNEE_JOINT_RH+6, ELBOW_JOINT_RH+6) = F(AZ,KNEE_JOINT_RH);
    DATA(ELBOW_JOINT_RH+6, KNEE_JOINT_RH+6) = DATA(KNEE_JOINT_RH+6, ELBOW_JOINT_RH+6);
    Fcol(KNEE_JOINT_RH) = frcTransf -> fr_hip_link_RH_X_fr_thigh_link_RH * Fcol(KNEE_JOINT_RH);
    DATA(KNEE_JOINT_RH+6, HIP_JOINT_RH+6) = F(AZ,KNEE_JOINT_RH);
    DATA(HIP_JOINT_RH+6, KNEE_JOINT_RH+6) = DATA(KNEE_JOINT_RH+6, HIP_JOINT_RH+6);
    Fcol(KNEE_JOINT_RH) = frcTransf -> fr_base_X_fr_hip_link_RH * Fcol(KNEE_JOINT_RH);

    // Link thigh_link_RH:
    iit::rbd::transformInertia<Scalar>(thigh_link_RH_Ic, frcTransf -> fr_hip_link_RH_X_fr_thigh_link_RH, Ic_spare);
    hip_link_RH_Ic += Ic_spare;

    Fcol(ELBOW_JOINT_RH) = thigh_link_RH_Ic.col(AZ);
    DATA(ELBOW_JOINT_RH+6, ELBOW_JOINT_RH+6) = Fcol(ELBOW_JOINT_RH)(AZ);

    Fcol(ELBOW_JOINT_RH) = frcTransf -> fr_hip_link_RH_X_fr_thigh_link_RH * Fcol(ELBOW_JOINT_RH);
    DATA(ELBOW_JOINT_RH+6, HIP_JOINT_RH+6) = F(AZ,ELBOW_JOINT_RH);
    DATA(HIP_JOINT_RH+6, ELBOW_JOINT_RH+6) = DATA(ELBOW_JOINT_RH+6, HIP_JOINT_RH+6);
    Fcol(ELBOW_JOINT_RH) = frcTransf -> fr_base_X_fr_hip_link_RH * Fcol(ELBOW_JOINT_RH);

    // Link hip_link_RH:
    iit::rbd::transformInertia<Scalar>(hip_link_RH_Ic, frcTransf -> fr_base_X_fr_hip_link_RH, Ic_spare);
    base_Ic += Ic_spare;

    Fcol(HIP_JOINT_RH) = hip_link_RH_Ic.col(AZ);
    DATA(HIP_JOINT_RH+6, HIP_JOINT_RH+6) = Fcol(HIP_JOINT_RH)(AZ);

    Fcol(HIP_JOINT_RH) = frcTransf -> fr_base_X_fr_hip_link_RH * Fcol(HIP_JOINT_RH);

    // Link shank_link_LH:
    iit::rbd::transformInertia<Scalar>(shank_link_LH_Ic, frcTransf -> fr_thigh_link_LH_X_fr_shank_link_LH, Ic_spare);
    thigh_link_LH_Ic += Ic_spare;

    Fcol(KNEE_JOINT_LH) = shank_link_LH_Ic.col(AZ);
    DATA(KNEE_JOINT_LH+6, KNEE_JOINT_LH+6) = Fcol(KNEE_JOINT_LH)(AZ);

    Fcol(KNEE_JOINT_LH) = frcTransf -> fr_thigh_link_LH_X_fr_shank_link_LH * Fcol(KNEE_JOINT_LH);
    DATA(KNEE_JOINT_LH+6, ELBOW_JOINT_LH+6) = F(AZ,KNEE_JOINT_LH);
    DATA(ELBOW_JOINT_LH+6, KNEE_JOINT_LH+6) = DATA(KNEE_JOINT_LH+6, ELBOW_JOINT_LH+6);
    Fcol(KNEE_JOINT_LH) = frcTransf -> fr_hip_link_LH_X_fr_thigh_link_LH * Fcol(KNEE_JOINT_LH);
    DATA(KNEE_JOINT_LH+6, HIP_JOINT_LH+6) = F(AZ,KNEE_JOINT_LH);
    DATA(HIP_JOINT_LH+6, KNEE_JOINT_LH+6) = DATA(KNEE_JOINT_LH+6, HIP_JOINT_LH+6);
    Fcol(KNEE_JOINT_LH) = frcTransf -> fr_base_X_fr_hip_link_LH * Fcol(KNEE_JOINT_LH);

    // Link thigh_link_LH:
    iit::rbd::transformInertia<Scalar>(thigh_link_LH_Ic, frcTransf -> fr_hip_link_LH_X_fr_thigh_link_LH, Ic_spare);
    hip_link_LH_Ic += Ic_spare;

    Fcol(ELBOW_JOINT_LH) = thigh_link_LH_Ic.col(AZ);
    DATA(ELBOW_JOINT_LH+6, ELBOW_JOINT_LH+6) = Fcol(ELBOW_JOINT_LH)(AZ);

    Fcol(ELBOW_JOINT_LH) = frcTransf -> fr_hip_link_LH_X_fr_thigh_link_LH * Fcol(ELBOW_JOINT_LH);
    DATA(ELBOW_JOINT_LH+6, HIP_JOINT_LH+6) = F(AZ,ELBOW_JOINT_LH);
    DATA(HIP_JOINT_LH+6, ELBOW_JOINT_LH+6) = DATA(ELBOW_JOINT_LH+6, HIP_JOINT_LH+6);
    Fcol(ELBOW_JOINT_LH) = frcTransf -> fr_base_X_fr_hip_link_LH * Fcol(ELBOW_JOINT_LH);

    // Link hip_link_LH:
    iit::rbd::transformInertia<Scalar>(hip_link_LH_Ic, frcTransf -> fr_base_X_fr_hip_link_LH, Ic_spare);
    base_Ic += Ic_spare;

    Fcol(HIP_JOINT_LH) = hip_link_LH_Ic.col(AZ);
    DATA(HIP_JOINT_LH+6, HIP_JOINT_LH+6) = Fcol(HIP_JOINT_LH)(AZ);

    Fcol(HIP_JOINT_LH) = frcTransf -> fr_base_X_fr_hip_link_LH * Fcol(HIP_JOINT_LH);

    // Link shank_link_RF:
    iit::rbd::transformInertia<Scalar>(shank_link_RF_Ic, frcTransf -> fr_thigh_link_RF_X_fr_shank_link_RF, Ic_spare);
    thigh_link_RF_Ic += Ic_spare;

    Fcol(KNEE_JOINT_RF) = shank_link_RF_Ic.col(AZ);
    DATA(KNEE_JOINT_RF+6, KNEE_JOINT_RF+6) = Fcol(KNEE_JOINT_RF)(AZ);

    Fcol(KNEE_JOINT_RF) = frcTransf -> fr_thigh_link_RF_X_fr_shank_link_RF * Fcol(KNEE_JOINT_RF);
    DATA(KNEE_JOINT_RF+6, ELBOW_JOINT_RF+6) = F(AZ,KNEE_JOINT_RF);
    DATA(ELBOW_JOINT_RF+6, KNEE_JOINT_RF+6) = DATA(KNEE_JOINT_RF+6, ELBOW_JOINT_RF+6);
    Fcol(KNEE_JOINT_RF) = frcTransf -> fr_hip_link_RF_X_fr_thigh_link_RF * Fcol(KNEE_JOINT_RF);
    DATA(KNEE_JOINT_RF+6, HIP_JOINT_RF+6) = F(AZ,KNEE_JOINT_RF);
    DATA(HIP_JOINT_RF+6, KNEE_JOINT_RF+6) = DATA(KNEE_JOINT_RF+6, HIP_JOINT_RF+6);
    Fcol(KNEE_JOINT_RF) = frcTransf -> fr_base_X_fr_hip_link_RF * Fcol(KNEE_JOINT_RF);

    // Link thigh_link_RF:
    iit::rbd::transformInertia<Scalar>(thigh_link_RF_Ic, frcTransf -> fr_hip_link_RF_X_fr_thigh_link_RF, Ic_spare);
    hip_link_RF_Ic += Ic_spare;

    Fcol(ELBOW_JOINT_RF) = thigh_link_RF_Ic.col(AZ);
    DATA(ELBOW_JOINT_RF+6, ELBOW_JOINT_RF+6) = Fcol(ELBOW_JOINT_RF)(AZ);

    Fcol(ELBOW_JOINT_RF) = frcTransf -> fr_hip_link_RF_X_fr_thigh_link_RF * Fcol(ELBOW_JOINT_RF);
    DATA(ELBOW_JOINT_RF+6, HIP_JOINT_RF+6) = F(AZ,ELBOW_JOINT_RF);
    DATA(HIP_JOINT_RF+6, ELBOW_JOINT_RF+6) = DATA(ELBOW_JOINT_RF+6, HIP_JOINT_RF+6);
    Fcol(ELBOW_JOINT_RF) = frcTransf -> fr_base_X_fr_hip_link_RF * Fcol(ELBOW_JOINT_RF);

    // Link hip_link_RF:
    iit::rbd::transformInertia<Scalar>(hip_link_RF_Ic, frcTransf -> fr_base_X_fr_hip_link_RF, Ic_spare);
    base_Ic += Ic_spare;

    Fcol(HIP_JOINT_RF) = hip_link_RF_Ic.col(AZ);
    DATA(HIP_JOINT_RF+6, HIP_JOINT_RF+6) = Fcol(HIP_JOINT_RF)(AZ);

    Fcol(HIP_JOINT_RF) = frcTransf -> fr_base_X_fr_hip_link_RF * Fcol(HIP_JOINT_RF);

    // Link shank_link_LF:
    iit::rbd::transformInertia<Scalar>(shank_link_LF_Ic, frcTransf -> fr_thigh_link_LF_X_fr_shank_link_LF, Ic_spare);
    thigh_link_LF_Ic += Ic_spare;

    Fcol(KNEE_JOINT_LF) = shank_link_LF_Ic.col(AZ);
    DATA(KNEE_JOINT_LF+6, KNEE_JOINT_LF+6) = Fcol(KNEE_JOINT_LF)(AZ);

    Fcol(KNEE_JOINT_LF) = frcTransf -> fr_thigh_link_LF_X_fr_shank_link_LF * Fcol(KNEE_JOINT_LF);
    DATA(KNEE_JOINT_LF+6, ELBOW_JOINT_LF+6) = F(AZ,KNEE_JOINT_LF);
    DATA(ELBOW_JOINT_LF+6, KNEE_JOINT_LF+6) = DATA(KNEE_JOINT_LF+6, ELBOW_JOINT_LF+6);
    Fcol(KNEE_JOINT_LF) = frcTransf -> fr_hip_link_LF_X_fr_thigh_link_LF * Fcol(KNEE_JOINT_LF);
    DATA(KNEE_JOINT_LF+6, HIP_JOINT_LF+6) = F(AZ,KNEE_JOINT_LF);
    DATA(HIP_JOINT_LF+6, KNEE_JOINT_LF+6) = DATA(KNEE_JOINT_LF+6, HIP_JOINT_LF+6);
    Fcol(KNEE_JOINT_LF) = frcTransf -> fr_base_X_fr_hip_link_LF * Fcol(KNEE_JOINT_LF);

    // Link thigh_link_LF:
    iit::rbd::transformInertia<Scalar>(thigh_link_LF_Ic, frcTransf -> fr_hip_link_LF_X_fr_thigh_link_LF, Ic_spare);
    hip_link_LF_Ic += Ic_spare;

    Fcol(ELBOW_JOINT_LF) = thigh_link_LF_Ic.col(AZ);
    DATA(ELBOW_JOINT_LF+6, ELBOW_JOINT_LF+6) = Fcol(ELBOW_JOINT_LF)(AZ);

    Fcol(ELBOW_JOINT_LF) = frcTransf -> fr_hip_link_LF_X_fr_thigh_link_LF * Fcol(ELBOW_JOINT_LF);
    DATA(ELBOW_JOINT_LF+6, HIP_JOINT_LF+6) = F(AZ,ELBOW_JOINT_LF);
    DATA(HIP_JOINT_LF+6, ELBOW_JOINT_LF+6) = DATA(ELBOW_JOINT_LF+6, HIP_JOINT_LF+6);
    Fcol(ELBOW_JOINT_LF) = frcTransf -> fr_base_X_fr_hip_link_LF * Fcol(ELBOW_JOINT_LF);

    // Link hip_link_LF:
    iit::rbd::transformInertia<Scalar>(hip_link_LF_Ic, frcTransf -> fr_base_X_fr_hip_link_LF, Ic_spare);
    base_Ic += Ic_spare;

    Fcol(HIP_JOINT_LF) = hip_link_LF_Ic.col(AZ);
    DATA(HIP_JOINT_LF+6, HIP_JOINT_LF+6) = Fcol(HIP_JOINT_LF)(AZ);

    Fcol(HIP_JOINT_LF) = frcTransf -> fr_base_X_fr_hip_link_LF * Fcol(HIP_JOINT_LF);

    // Copies the upper-right block into the lower-left block, after transposing
    block<12, 6>(6,0) = (block<6, 12>(0,6)).transpose();
    // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
    block<6,6>(0,0) = base_Ic;
    return *this;
}

#undef DATA
#undef F

void rcg::shvan1::dyn::JSIM::computeL() {
    L = this -> triangularView<Eigen::Lower>();
    // Joint knee_joint_RH, index 11 :
    L(11, 11) = ScalarTraits::sqrt(L(11, 11));
    L(11, 10) = L(11, 10) / L(11, 11);
    L(11, 9) = L(11, 9) / L(11, 11);
    L(10, 10) = L(10, 10) - L(11, 10) * L(11, 10);
    L(10, 9) = L(10, 9) - L(11, 10) * L(11, 9);
    L(9, 9) = L(9, 9) - L(11, 9) * L(11, 9);
    
    // Joint elbow_joint_RH, index 10 :
    L(10, 10) = ScalarTraits::sqrt(L(10, 10));
    L(10, 9) = L(10, 9) / L(10, 10);
    L(9, 9) = L(9, 9) - L(10, 9) * L(10, 9);
    
    // Joint hip_joint_RH, index 9 :
    L(9, 9) = ScalarTraits::sqrt(L(9, 9));
    
    // Joint knee_joint_LH, index 8 :
    L(8, 8) = ScalarTraits::sqrt(L(8, 8));
    L(8, 7) = L(8, 7) / L(8, 8);
    L(8, 6) = L(8, 6) / L(8, 8);
    L(7, 7) = L(7, 7) - L(8, 7) * L(8, 7);
    L(7, 6) = L(7, 6) - L(8, 7) * L(8, 6);
    L(6, 6) = L(6, 6) - L(8, 6) * L(8, 6);
    
    // Joint elbow_joint_LH, index 7 :
    L(7, 7) = ScalarTraits::sqrt(L(7, 7));
    L(7, 6) = L(7, 6) / L(7, 7);
    L(6, 6) = L(6, 6) - L(7, 6) * L(7, 6);
    
    // Joint hip_joint_LH, index 6 :
    L(6, 6) = ScalarTraits::sqrt(L(6, 6));
    
    // Joint knee_joint_RF, index 5 :
    L(5, 5) = ScalarTraits::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);
    
    // Joint elbow_joint_RF, index 4 :
    L(4, 4) = ScalarTraits::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    
    // Joint hip_joint_RF, index 3 :
    L(3, 3) = ScalarTraits::sqrt(L(3, 3));
    
    // Joint knee_joint_LF, index 2 :
    L(2, 2) = ScalarTraits::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);
    
    // Joint elbow_joint_LF, index 1 :
    L(1, 1) = ScalarTraits::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint hip_joint_LF, index 0 :
    L(0, 0) = ScalarTraits::sqrt(L(0, 0));
    
}

void rcg::shvan1::dyn::JSIM::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) =  + (Linv(2, 0) * Linv(2, 0)) + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 1) =  + (Linv(2, 0) * Linv(1, 0)) + (Linv(2, 1) * Linv(1, 1));
    inverse(1, 2) = inverse(2, 1);
    inverse(2, 0) =  + (Linv(2, 0) * Linv(0, 0));
    inverse(0, 2) = inverse(2, 0);
    inverse(3, 3) =  + (Linv(3, 3) * Linv(3, 3));
    inverse(4, 4) =  + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(5, 5) =  + (Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =  + (Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
    inverse(6, 6) =  + (Linv(6, 6) * Linv(6, 6));
    inverse(7, 7) =  + (Linv(7, 6) * Linv(7, 6)) + (Linv(7, 7) * Linv(7, 7));
    inverse(7, 6) =  + (Linv(7, 6) * Linv(6, 6));
    inverse(6, 7) = inverse(7, 6);
    inverse(8, 8) =  + (Linv(8, 6) * Linv(8, 6)) + (Linv(8, 7) * Linv(8, 7)) + (Linv(8, 8) * Linv(8, 8));
    inverse(8, 7) =  + (Linv(8, 6) * Linv(7, 6)) + (Linv(8, 7) * Linv(7, 7));
    inverse(7, 8) = inverse(8, 7);
    inverse(8, 6) =  + (Linv(8, 6) * Linv(6, 6));
    inverse(6, 8) = inverse(8, 6);
    inverse(9, 9) =  + (Linv(9, 9) * Linv(9, 9));
    inverse(10, 10) =  + (Linv(10, 9) * Linv(10, 9)) + (Linv(10, 10) * Linv(10, 10));
    inverse(10, 9) =  + (Linv(10, 9) * Linv(9, 9));
    inverse(9, 10) = inverse(10, 9);
    inverse(11, 11) =  + (Linv(11, 9) * Linv(11, 9)) + (Linv(11, 10) * Linv(11, 10)) + (Linv(11, 11) * Linv(11, 11));
    inverse(11, 10) =  + (Linv(11, 9) * Linv(10, 9)) + (Linv(11, 10) * Linv(10, 10));
    inverse(10, 11) = inverse(11, 10);
    inverse(11, 9) =  + (Linv(11, 9) * Linv(9, 9));
    inverse(9, 11) = inverse(11, 9);
}

void rcg::shvan1::dyn::JSIM::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(6, 6) = 1 / L(6, 6);
    Linv(7, 7) = 1 / L(7, 7);
    Linv(8, 8) = 1 / L(8, 8);
    Linv(9, 9) = 1 / L(9, 9);
    Linv(10, 10) = 1 / L(10, 10);
    Linv(11, 11) = 1 / L(11, 11);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = - Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = - Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
    Linv(7, 6) = - Linv(6, 6) * ((Linv(7, 7) * L(7, 6)) + 0);
    Linv(8, 7) = - Linv(7, 7) * ((Linv(8, 8) * L(8, 7)) + 0);
    Linv(8, 6) = - Linv(6, 6) * ((Linv(8, 7) * L(7, 6)) + (Linv(8, 8) * L(8, 6)) + 0);
    Linv(10, 9) = - Linv(9, 9) * ((Linv(10, 10) * L(10, 9)) + 0);
    Linv(11, 10) = - Linv(10, 10) * ((Linv(11, 11) * L(11, 10)) + 0);
    Linv(11, 9) = - Linv(9, 9) * ((Linv(11, 10) * L(10, 9)) + (Linv(11, 11) * L(11, 9)) + 0);
}
