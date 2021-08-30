#ifndef RCG_SHVAN1_LINK_DATA_MAP_H_
#define RCG_SHVAN1_LINK_DATA_MAP_H_

#include "declarations.h"

namespace rcg {
namespace shvan1 {

/**
 * A very simple container to associate a generic data item to each link
 */
template<typename T> class LinkDataMap {
private:
    T data[linksCount];
public:
    LinkDataMap() {};
    LinkDataMap(const T& defaultValue);
    LinkDataMap(const LinkDataMap& rhs);
    LinkDataMap& operator=(const LinkDataMap& rhs);
    LinkDataMap& operator=(const T& rhs);
          T& operator[](LinkIdentifiers which);
    const T& operator[](LinkIdentifiers which) const;
private:
    void copydata(const LinkDataMap& rhs);
    void assigndata(const T& commonValue);
};

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const LinkDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const LinkDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& LinkDataMap<T>::operator[](LinkIdentifiers l) {
    return data[l];
}

template<typename T> inline
const T& LinkDataMap<T>::operator[](LinkIdentifiers l) const {
    return data[l];
}

template<typename T> inline
void LinkDataMap<T>::copydata(const LinkDataMap& rhs) {
    data[BASE] = rhs[BASE];
    data[HIP_LINK_LF] = rhs[HIP_LINK_LF];
    data[THIGH_LINK_LF] = rhs[THIGH_LINK_LF];
    data[SHANK_LINK_LF] = rhs[SHANK_LINK_LF];
    data[HIP_LINK_RF] = rhs[HIP_LINK_RF];
    data[THIGH_LINK_RF] = rhs[THIGH_LINK_RF];
    data[SHANK_LINK_RF] = rhs[SHANK_LINK_RF];
    data[HIP_LINK_LH] = rhs[HIP_LINK_LH];
    data[THIGH_LINK_LH] = rhs[THIGH_LINK_LH];
    data[SHANK_LINK_LH] = rhs[SHANK_LINK_LH];
    data[HIP_LINK_RH] = rhs[HIP_LINK_RH];
    data[THIGH_LINK_RH] = rhs[THIGH_LINK_RH];
    data[SHANK_LINK_RH] = rhs[SHANK_LINK_RH];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[BASE] = value;
    data[HIP_LINK_LF] = value;
    data[THIGH_LINK_LF] = value;
    data[SHANK_LINK_LF] = value;
    data[HIP_LINK_RF] = value;
    data[THIGH_LINK_RF] = value;
    data[SHANK_LINK_RF] = value;
    data[HIP_LINK_LH] = value;
    data[THIGH_LINK_LH] = value;
    data[SHANK_LINK_LH] = value;
    data[HIP_LINK_RH] = value;
    data[THIGH_LINK_RH] = value;
    data[SHANK_LINK_RH] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   base = "
    << map[BASE]
    << "   hip_link_LF = "
    << map[HIP_LINK_LF]
    << "   thigh_link_LF = "
    << map[THIGH_LINK_LF]
    << "   shank_link_LF = "
    << map[SHANK_LINK_LF]
    << "   hip_link_RF = "
    << map[HIP_LINK_RF]
    << "   thigh_link_RF = "
    << map[THIGH_LINK_RF]
    << "   shank_link_RF = "
    << map[SHANK_LINK_RF]
    << "   hip_link_LH = "
    << map[HIP_LINK_LH]
    << "   thigh_link_LH = "
    << map[THIGH_LINK_LH]
    << "   shank_link_LH = "
    << map[SHANK_LINK_LH]
    << "   hip_link_RH = "
    << map[HIP_LINK_RH]
    << "   thigh_link_RH = "
    << map[THIGH_LINK_RH]
    << "   shank_link_RH = "
    << map[SHANK_LINK_RH]
    ;
    return out;
}

}
}
#endif
