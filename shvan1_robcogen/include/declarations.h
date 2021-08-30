#ifndef RCG_SHVAN1_DECLARATIONS_H_
#define RCG_SHVAN1_DECLARATIONS_H_

#include "rbd_types.h"

namespace rcg {
namespace shvan1 {

static constexpr int JointSpaceDimension = 12;
static constexpr int jointsCount = 12;
/** The total number of rigid bodies of this robot, including the base */
static constexpr int linksCount  = 13;

typedef Matrix<12, 1> Column12d;
typedef Column12d JointState;

enum JointIdentifiers {
    HIP_JOINT_LF = 0
    , ELBOW_JOINT_LF
    , KNEE_JOINT_LF
    , HIP_JOINT_RF
    , ELBOW_JOINT_RF
    , KNEE_JOINT_RF
    , HIP_JOINT_LH
    , ELBOW_JOINT_LH
    , KNEE_JOINT_LH
    , HIP_JOINT_RH
    , ELBOW_JOINT_RH
    , KNEE_JOINT_RH
};

enum LinkIdentifiers {
    BASE = 0
    , HIP_LINK_LF
    , THIGH_LINK_LF
    , SHANK_LINK_LF
    , HIP_LINK_RF
    , THIGH_LINK_RF
    , SHANK_LINK_RF
    , HIP_LINK_LH
    , THIGH_LINK_LH
    , SHANK_LINK_LH
    , HIP_LINK_RH
    , THIGH_LINK_RH
    , SHANK_LINK_RH
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {HIP_JOINT_LF,ELBOW_JOINT_LF,KNEE_JOINT_LF,HIP_JOINT_RF,ELBOW_JOINT_RF,KNEE_JOINT_RF,HIP_JOINT_LH,ELBOW_JOINT_LH,KNEE_JOINT_LH,HIP_JOINT_RH,ELBOW_JOINT_RH,KNEE_JOINT_RH};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE,HIP_LINK_LF,THIGH_LINK_LF,SHANK_LINK_LF,HIP_LINK_RF,THIGH_LINK_RF,SHANK_LINK_RF,HIP_LINK_LH,THIGH_LINK_LH,SHANK_LINK_LH,HIP_LINK_RH,THIGH_LINK_RH,SHANK_LINK_RH};

}
}
#endif
