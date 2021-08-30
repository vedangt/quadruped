#ifndef RCG__SHVAN1_TRAITS_H_
#define RCG__SHVAN1_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace rcg {
namespace shvan1 {
struct Traits {
    typedef typename rcg::shvan1::ScalarTraits ScalarTraits;

    typedef typename rcg::shvan1::JointState JointState;

    typedef typename rcg::shvan1::JointIdentifiers JointID;
    typedef typename rcg::shvan1::LinkIdentifiers  LinkID;

    typedef typename rcg::shvan1::HomogeneousTransforms HomogeneousTransforms;
    typedef typename rcg::shvan1::MotionTransforms MotionTransforms;
    typedef typename rcg::shvan1::ForceTransforms ForceTransforms;

    typedef typename rcg::shvan1::dyn::InertiaProperties InertiaProperties;
    typedef typename rcg::shvan1::dyn::ForwardDynamics FwdDynEngine;
    typedef typename rcg::shvan1::dyn::InverseDynamics InvDynEngine;
    typedef typename rcg::shvan1::dyn::JSIM JSIM;

    static const int joints_count = rcg::shvan1::jointsCount;
    static const int links_count  = rcg::shvan1::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return rcg::shvan1::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return rcg::shvan1::orderedLinkIDs;
}

}
}

#endif
