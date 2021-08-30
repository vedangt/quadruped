#ifndef RCG_SHVAN1_JOINT_DATA_MAP_H_
#define RCG_SHVAN1_JOINT_DATA_MAP_H_

#include "declarations.h"

namespace rcg {
namespace shvan1 {

/**
 * A very simple container to associate a generic data item to each joint
 */
template<typename T> class JointDataMap {
private:
    T data[jointsCount];
public:
    JointDataMap() {};
    JointDataMap(const T& defaultValue);
    JointDataMap(const JointDataMap& rhs);
    JointDataMap& operator=(const JointDataMap& rhs);
    JointDataMap& operator=(const T& rhs);
          T& operator[](JointIdentifiers which);
    const T& operator[](JointIdentifiers which) const;
private:
    void copydata(const JointDataMap& rhs);
    void assigndata(const T& rhs);
};

template<typename T> inline
JointDataMap<T>::JointDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
JointDataMap<T>::JointDataMap(const JointDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const JointDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& JointDataMap<T>::operator[](JointIdentifiers j) {
    return data[j];
}

template<typename T> inline
const T& JointDataMap<T>::operator[](JointIdentifiers j) const {
    return data[j];
}

template<typename T> inline
void JointDataMap<T>::copydata(const JointDataMap& rhs) {
    data[HIP_JOINT_LF] = rhs[HIP_JOINT_LF];
    data[ELBOW_JOINT_LF] = rhs[ELBOW_JOINT_LF];
    data[KNEE_JOINT_LF] = rhs[KNEE_JOINT_LF];
    data[HIP_JOINT_RF] = rhs[HIP_JOINT_RF];
    data[ELBOW_JOINT_RF] = rhs[ELBOW_JOINT_RF];
    data[KNEE_JOINT_RF] = rhs[KNEE_JOINT_RF];
    data[HIP_JOINT_LH] = rhs[HIP_JOINT_LH];
    data[ELBOW_JOINT_LH] = rhs[ELBOW_JOINT_LH];
    data[KNEE_JOINT_LH] = rhs[KNEE_JOINT_LH];
    data[HIP_JOINT_RH] = rhs[HIP_JOINT_RH];
    data[ELBOW_JOINT_RH] = rhs[ELBOW_JOINT_RH];
    data[KNEE_JOINT_RH] = rhs[KNEE_JOINT_RH];
}

template<typename T> inline
void JointDataMap<T>::assigndata(const T& value) {
    data[HIP_JOINT_LF] = value;
    data[ELBOW_JOINT_LF] = value;
    data[KNEE_JOINT_LF] = value;
    data[HIP_JOINT_RF] = value;
    data[ELBOW_JOINT_RF] = value;
    data[KNEE_JOINT_RF] = value;
    data[HIP_JOINT_LH] = value;
    data[ELBOW_JOINT_LH] = value;
    data[KNEE_JOINT_LH] = value;
    data[HIP_JOINT_RH] = value;
    data[ELBOW_JOINT_RH] = value;
    data[KNEE_JOINT_RH] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const JointDataMap<T>& map) {
    out
    << "   hip_joint_LF = "
    << map[HIP_JOINT_LF]
    << "   elbow_joint_LF = "
    << map[ELBOW_JOINT_LF]
    << "   knee_joint_LF = "
    << map[KNEE_JOINT_LF]
    << "   hip_joint_RF = "
    << map[HIP_JOINT_RF]
    << "   elbow_joint_RF = "
    << map[ELBOW_JOINT_RF]
    << "   knee_joint_RF = "
    << map[KNEE_JOINT_RF]
    << "   hip_joint_LH = "
    << map[HIP_JOINT_LH]
    << "   elbow_joint_LH = "
    << map[ELBOW_JOINT_LH]
    << "   knee_joint_LH = "
    << map[KNEE_JOINT_LH]
    << "   hip_joint_RH = "
    << map[HIP_JOINT_RH]
    << "   elbow_joint_RH = "
    << map[ELBOW_JOINT_RH]
    << "   knee_joint_RH = "
    << map[KNEE_JOINT_RH]
    ;
    return out;
}

}
}
#endif
