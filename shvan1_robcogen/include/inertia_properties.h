#ifndef RCG_SHVAN1_INERTIA_PROPERTIES_H_
#define RCG_SHVAN1_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "model_constants.h"
#include "dynamics_parameters.h"

namespace rcg {
namespace shvan1 {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot shvan1.
 */
namespace dyn {

class InertiaProperties {
    public:
        InertiaProperties();
        ~InertiaProperties();
        const InertiaMatrix& getTensor_base() const;
        const InertiaMatrix& getTensor_hip_link_LF() const;
        const InertiaMatrix& getTensor_thigh_link_LF() const;
        const InertiaMatrix& getTensor_shank_link_LF() const;
        const InertiaMatrix& getTensor_hip_link_RF() const;
        const InertiaMatrix& getTensor_thigh_link_RF() const;
        const InertiaMatrix& getTensor_shank_link_RF() const;
        const InertiaMatrix& getTensor_hip_link_LH() const;
        const InertiaMatrix& getTensor_thigh_link_LH() const;
        const InertiaMatrix& getTensor_shank_link_LH() const;
        const InertiaMatrix& getTensor_hip_link_RH() const;
        const InertiaMatrix& getTensor_thigh_link_RH() const;
        const InertiaMatrix& getTensor_shank_link_RH() const;
        Scalar getMass_base() const;
        Scalar getMass_hip_link_LF() const;
        Scalar getMass_thigh_link_LF() const;
        Scalar getMass_shank_link_LF() const;
        Scalar getMass_hip_link_RF() const;
        Scalar getMass_thigh_link_RF() const;
        Scalar getMass_shank_link_RF() const;
        Scalar getMass_hip_link_LH() const;
        Scalar getMass_thigh_link_LH() const;
        Scalar getMass_shank_link_LH() const;
        Scalar getMass_hip_link_RH() const;
        Scalar getMass_thigh_link_RH() const;
        Scalar getMass_shank_link_RH() const;
        const Vector3& getCOM_base() const;
        const Vector3& getCOM_hip_link_LF() const;
        const Vector3& getCOM_thigh_link_LF() const;
        const Vector3& getCOM_shank_link_LF() const;
        const Vector3& getCOM_hip_link_RF() const;
        const Vector3& getCOM_thigh_link_RF() const;
        const Vector3& getCOM_shank_link_RF() const;
        const Vector3& getCOM_hip_link_LH() const;
        const Vector3& getCOM_thigh_link_LH() const;
        const Vector3& getCOM_shank_link_LH() const;
        const Vector3& getCOM_hip_link_RH() const;
        const Vector3& getCOM_thigh_link_RH() const;
        const Vector3& getCOM_shank_link_RH() const;
        Scalar getTotalMass() const;


        /*!
         * Fresh values for the runtime parameters of the robot shvan1,
         * causing the update of the inertia properties modeled by this
         * instance.
         */
        void updateParameters(const RuntimeInertiaParams&);

    private:
        RuntimeInertiaParams params;

        InertiaMatrix tensor_base;
        InertiaMatrix tensor_hip_link_LF;
        InertiaMatrix tensor_thigh_link_LF;
        InertiaMatrix tensor_shank_link_LF;
        InertiaMatrix tensor_hip_link_RF;
        InertiaMatrix tensor_thigh_link_RF;
        InertiaMatrix tensor_shank_link_RF;
        InertiaMatrix tensor_hip_link_LH;
        InertiaMatrix tensor_thigh_link_LH;
        InertiaMatrix tensor_shank_link_LH;
        InertiaMatrix tensor_hip_link_RH;
        InertiaMatrix tensor_thigh_link_RH;
        InertiaMatrix tensor_shank_link_RH;
        Vector3 com_base;
        Vector3 com_hip_link_LF;
        Vector3 com_thigh_link_LF;
        Vector3 com_shank_link_LF;
        Vector3 com_hip_link_RF;
        Vector3 com_thigh_link_RF;
        Vector3 com_shank_link_RF;
        Vector3 com_hip_link_LH;
        Vector3 com_thigh_link_LH;
        Vector3 com_shank_link_LH;
        Vector3 com_hip_link_RH;
        Vector3 com_thigh_link_RH;
        Vector3 com_shank_link_RH;
};


inline InertiaProperties::~InertiaProperties() {}

inline const InertiaMatrix& InertiaProperties::getTensor_base() const {
    return this->tensor_base;
}
inline const InertiaMatrix& InertiaProperties::getTensor_hip_link_LF() const {
    return this->tensor_hip_link_LF;
}
inline const InertiaMatrix& InertiaProperties::getTensor_thigh_link_LF() const {
    return this->tensor_thigh_link_LF;
}
inline const InertiaMatrix& InertiaProperties::getTensor_shank_link_LF() const {
    return this->tensor_shank_link_LF;
}
inline const InertiaMatrix& InertiaProperties::getTensor_hip_link_RF() const {
    return this->tensor_hip_link_RF;
}
inline const InertiaMatrix& InertiaProperties::getTensor_thigh_link_RF() const {
    return this->tensor_thigh_link_RF;
}
inline const InertiaMatrix& InertiaProperties::getTensor_shank_link_RF() const {
    return this->tensor_shank_link_RF;
}
inline const InertiaMatrix& InertiaProperties::getTensor_hip_link_LH() const {
    return this->tensor_hip_link_LH;
}
inline const InertiaMatrix& InertiaProperties::getTensor_thigh_link_LH() const {
    return this->tensor_thigh_link_LH;
}
inline const InertiaMatrix& InertiaProperties::getTensor_shank_link_LH() const {
    return this->tensor_shank_link_LH;
}
inline const InertiaMatrix& InertiaProperties::getTensor_hip_link_RH() const {
    return this->tensor_hip_link_RH;
}
inline const InertiaMatrix& InertiaProperties::getTensor_thigh_link_RH() const {
    return this->tensor_thigh_link_RH;
}
inline const InertiaMatrix& InertiaProperties::getTensor_shank_link_RH() const {
    return this->tensor_shank_link_RH;
}
inline Scalar InertiaProperties::getMass_base() const {
    return this->tensor_base.getMass();
}
inline Scalar InertiaProperties::getMass_hip_link_LF() const {
    return this->tensor_hip_link_LF.getMass();
}
inline Scalar InertiaProperties::getMass_thigh_link_LF() const {
    return this->tensor_thigh_link_LF.getMass();
}
inline Scalar InertiaProperties::getMass_shank_link_LF() const {
    return this->tensor_shank_link_LF.getMass();
}
inline Scalar InertiaProperties::getMass_hip_link_RF() const {
    return this->tensor_hip_link_RF.getMass();
}
inline Scalar InertiaProperties::getMass_thigh_link_RF() const {
    return this->tensor_thigh_link_RF.getMass();
}
inline Scalar InertiaProperties::getMass_shank_link_RF() const {
    return this->tensor_shank_link_RF.getMass();
}
inline Scalar InertiaProperties::getMass_hip_link_LH() const {
    return this->tensor_hip_link_LH.getMass();
}
inline Scalar InertiaProperties::getMass_thigh_link_LH() const {
    return this->tensor_thigh_link_LH.getMass();
}
inline Scalar InertiaProperties::getMass_shank_link_LH() const {
    return this->tensor_shank_link_LH.getMass();
}
inline Scalar InertiaProperties::getMass_hip_link_RH() const {
    return this->tensor_hip_link_RH.getMass();
}
inline Scalar InertiaProperties::getMass_thigh_link_RH() const {
    return this->tensor_thigh_link_RH.getMass();
}
inline Scalar InertiaProperties::getMass_shank_link_RH() const {
    return this->tensor_shank_link_RH.getMass();
}
inline const Vector3& InertiaProperties::getCOM_base() const {
    return this->com_base;
}
inline const Vector3& InertiaProperties::getCOM_hip_link_LF() const {
    return this->com_hip_link_LF;
}
inline const Vector3& InertiaProperties::getCOM_thigh_link_LF() const {
    return this->com_thigh_link_LF;
}
inline const Vector3& InertiaProperties::getCOM_shank_link_LF() const {
    return this->com_shank_link_LF;
}
inline const Vector3& InertiaProperties::getCOM_hip_link_RF() const {
    return this->com_hip_link_RF;
}
inline const Vector3& InertiaProperties::getCOM_thigh_link_RF() const {
    return this->com_thigh_link_RF;
}
inline const Vector3& InertiaProperties::getCOM_shank_link_RF() const {
    return this->com_shank_link_RF;
}
inline const Vector3& InertiaProperties::getCOM_hip_link_LH() const {
    return this->com_hip_link_LH;
}
inline const Vector3& InertiaProperties::getCOM_thigh_link_LH() const {
    return this->com_thigh_link_LH;
}
inline const Vector3& InertiaProperties::getCOM_shank_link_LH() const {
    return this->com_shank_link_LH;
}
inline const Vector3& InertiaProperties::getCOM_hip_link_RH() const {
    return this->com_hip_link_RH;
}
inline const Vector3& InertiaProperties::getCOM_thigh_link_RH() const {
    return this->com_thigh_link_RH;
}
inline const Vector3& InertiaProperties::getCOM_shank_link_RH() const {
    return this->com_shank_link_RH;
}

inline Scalar InertiaProperties::getTotalMass() const {
    return m_base + m_hip_link_LF + m_thigh_link_LF + m_shank_link_LF + m_hip_link_RF + m_thigh_link_RF + m_shank_link_RF + m_hip_link_LH + m_thigh_link_LH + m_shank_link_LH + m_hip_link_RH + m_thigh_link_RH + m_shank_link_RH;
}

}
}
}

#endif
