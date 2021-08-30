#ifndef SHVAN1_TRANSFORMS_H_
#define SHVAN1_TRANSFORMS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "model_constants.h"
#include "kinematics_parameters.h"

namespace rcg {
namespace shvan1 {

struct Parameters
{
    struct AngleFuncValues {
        AngleFuncValues() {
            update();
        }

        void update()
        {
        }
    };

    Params_lengths lengths;
    Params_angles angles;
    AngleFuncValues trig = AngleFuncValues();
};

// The type of the "vector" with the status of the variables
typedef JointState state_t;

template<class M>
using TransformMotion = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformForce = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformHomogeneous = iit::rbd::HomogeneousTransformBase<state_t, M>;

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
class MotionTransforms
{
public:
    class Dummy {};
    typedef TransformMotion<Dummy>::MatrixType MatrixType;

    struct Type_fr_base_X_LF_FOOT : public TransformMotion<Type_fr_base_X_LF_FOOT>
    {
        Type_fr_base_X_LF_FOOT();
        const Type_fr_base_X_LF_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_RF_FOOT : public TransformMotion<Type_fr_base_X_RF_FOOT>
    {
        Type_fr_base_X_RF_FOOT();
        const Type_fr_base_X_RF_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_LH_FOOT : public TransformMotion<Type_fr_base_X_LH_FOOT>
    {
        Type_fr_base_X_LH_FOOT();
        const Type_fr_base_X_LH_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_RH_FOOT : public TransformMotion<Type_fr_base_X_RH_FOOT>
    {
        Type_fr_base_X_RH_FOOT();
        const Type_fr_base_X_RH_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_LF_FOOT : public TransformMotion<Type_imu_link_X_LF_FOOT>
    {
        Type_imu_link_X_LF_FOOT();
        const Type_imu_link_X_LF_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_RF_FOOT : public TransformMotion<Type_imu_link_X_RF_FOOT>
    {
        Type_imu_link_X_RF_FOOT();
        const Type_imu_link_X_RF_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_LH_FOOT : public TransformMotion<Type_imu_link_X_LH_FOOT>
    {
        Type_imu_link_X_LH_FOOT();
        const Type_imu_link_X_LH_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_RH_FOOT : public TransformMotion<Type_imu_link_X_RH_FOOT>
    {
        Type_imu_link_X_RH_FOOT();
        const Type_imu_link_X_RH_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_joint_LF : public TransformMotion<Type_fr_base_X_fr_hip_joint_LF>
    {
        Type_fr_base_X_fr_hip_joint_LF();
        const Type_fr_base_X_fr_hip_joint_LF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_elbow_joint_LF : public TransformMotion<Type_fr_base_X_fr_elbow_joint_LF>
    {
        Type_fr_base_X_fr_elbow_joint_LF();
        const Type_fr_base_X_fr_elbow_joint_LF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_knee_joint_LF : public TransformMotion<Type_fr_base_X_fr_knee_joint_LF>
    {
        Type_fr_base_X_fr_knee_joint_LF();
        const Type_fr_base_X_fr_knee_joint_LF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_joint_RF : public TransformMotion<Type_fr_base_X_fr_hip_joint_RF>
    {
        Type_fr_base_X_fr_hip_joint_RF();
        const Type_fr_base_X_fr_hip_joint_RF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_elbow_joint_RF : public TransformMotion<Type_fr_base_X_fr_elbow_joint_RF>
    {
        Type_fr_base_X_fr_elbow_joint_RF();
        const Type_fr_base_X_fr_elbow_joint_RF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_knee_joint_RF : public TransformMotion<Type_fr_base_X_fr_knee_joint_RF>
    {
        Type_fr_base_X_fr_knee_joint_RF();
        const Type_fr_base_X_fr_knee_joint_RF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_joint_LH : public TransformMotion<Type_fr_base_X_fr_hip_joint_LH>
    {
        Type_fr_base_X_fr_hip_joint_LH();
        const Type_fr_base_X_fr_hip_joint_LH& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_elbow_joint_LH : public TransformMotion<Type_fr_base_X_fr_elbow_joint_LH>
    {
        Type_fr_base_X_fr_elbow_joint_LH();
        const Type_fr_base_X_fr_elbow_joint_LH& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_knee_joint_LH : public TransformMotion<Type_fr_base_X_fr_knee_joint_LH>
    {
        Type_fr_base_X_fr_knee_joint_LH();
        const Type_fr_base_X_fr_knee_joint_LH& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_joint_RH : public TransformMotion<Type_fr_base_X_fr_hip_joint_RH>
    {
        Type_fr_base_X_fr_hip_joint_RH();
        const Type_fr_base_X_fr_hip_joint_RH& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_elbow_joint_RH : public TransformMotion<Type_fr_base_X_fr_elbow_joint_RH>
    {
        Type_fr_base_X_fr_elbow_joint_RH();
        const Type_fr_base_X_fr_elbow_joint_RH& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_knee_joint_RH : public TransformMotion<Type_fr_base_X_fr_knee_joint_RH>
    {
        Type_fr_base_X_fr_knee_joint_RH();
        const Type_fr_base_X_fr_knee_joint_RH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_hip_joint_LF : public TransformMotion<Type_imu_link_X_fr_hip_joint_LF>
    {
        Type_imu_link_X_fr_hip_joint_LF();
        const Type_imu_link_X_fr_hip_joint_LF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_elbow_joint_LF : public TransformMotion<Type_imu_link_X_fr_elbow_joint_LF>
    {
        Type_imu_link_X_fr_elbow_joint_LF();
        const Type_imu_link_X_fr_elbow_joint_LF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_knee_joint_LF : public TransformMotion<Type_imu_link_X_fr_knee_joint_LF>
    {
        Type_imu_link_X_fr_knee_joint_LF();
        const Type_imu_link_X_fr_knee_joint_LF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_hip_joint_RF : public TransformMotion<Type_imu_link_X_fr_hip_joint_RF>
    {
        Type_imu_link_X_fr_hip_joint_RF();
        const Type_imu_link_X_fr_hip_joint_RF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_elbow_joint_RF : public TransformMotion<Type_imu_link_X_fr_elbow_joint_RF>
    {
        Type_imu_link_X_fr_elbow_joint_RF();
        const Type_imu_link_X_fr_elbow_joint_RF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_knee_joint_RF : public TransformMotion<Type_imu_link_X_fr_knee_joint_RF>
    {
        Type_imu_link_X_fr_knee_joint_RF();
        const Type_imu_link_X_fr_knee_joint_RF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_hip_joint_LH : public TransformMotion<Type_imu_link_X_fr_hip_joint_LH>
    {
        Type_imu_link_X_fr_hip_joint_LH();
        const Type_imu_link_X_fr_hip_joint_LH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_elbow_joint_LH : public TransformMotion<Type_imu_link_X_fr_elbow_joint_LH>
    {
        Type_imu_link_X_fr_elbow_joint_LH();
        const Type_imu_link_X_fr_elbow_joint_LH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_knee_joint_LH : public TransformMotion<Type_imu_link_X_fr_knee_joint_LH>
    {
        Type_imu_link_X_fr_knee_joint_LH();
        const Type_imu_link_X_fr_knee_joint_LH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_hip_joint_RH : public TransformMotion<Type_imu_link_X_fr_hip_joint_RH>
    {
        Type_imu_link_X_fr_hip_joint_RH();
        const Type_imu_link_X_fr_hip_joint_RH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_elbow_joint_RH : public TransformMotion<Type_imu_link_X_fr_elbow_joint_RH>
    {
        Type_imu_link_X_fr_elbow_joint_RH();
        const Type_imu_link_X_fr_elbow_joint_RH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_knee_joint_RH : public TransformMotion<Type_imu_link_X_fr_knee_joint_RH>
    {
        Type_imu_link_X_fr_knee_joint_RH();
        const Type_imu_link_X_fr_knee_joint_RH& update(const state_t&);
    };
    
    struct Type_fr_hip_link_LF_X_fr_base : public TransformMotion<Type_fr_hip_link_LF_X_fr_base>
    {
        Type_fr_hip_link_LF_X_fr_base();
        const Type_fr_hip_link_LF_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_link_LF : public TransformMotion<Type_fr_base_X_fr_hip_link_LF>
    {
        Type_fr_base_X_fr_hip_link_LF();
        const Type_fr_base_X_fr_hip_link_LF& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_LF_X_fr_hip_link_LF : public TransformMotion<Type_fr_thigh_link_LF_X_fr_hip_link_LF>
    {
        Type_fr_thigh_link_LF_X_fr_hip_link_LF();
        const Type_fr_thigh_link_LF_X_fr_hip_link_LF& update(const state_t&);
    };
    
    struct Type_fr_hip_link_LF_X_fr_thigh_link_LF : public TransformMotion<Type_fr_hip_link_LF_X_fr_thigh_link_LF>
    {
        Type_fr_hip_link_LF_X_fr_thigh_link_LF();
        const Type_fr_hip_link_LF_X_fr_thigh_link_LF& update(const state_t&);
    };
    
    struct Type_fr_shank_link_LF_X_fr_thigh_link_LF : public TransformMotion<Type_fr_shank_link_LF_X_fr_thigh_link_LF>
    {
        Type_fr_shank_link_LF_X_fr_thigh_link_LF();
        const Type_fr_shank_link_LF_X_fr_thigh_link_LF& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_LF_X_fr_shank_link_LF : public TransformMotion<Type_fr_thigh_link_LF_X_fr_shank_link_LF>
    {
        Type_fr_thigh_link_LF_X_fr_shank_link_LF();
        const Type_fr_thigh_link_LF_X_fr_shank_link_LF& update(const state_t&);
    };
    
    struct Type_fr_hip_link_RF_X_fr_base : public TransformMotion<Type_fr_hip_link_RF_X_fr_base>
    {
        Type_fr_hip_link_RF_X_fr_base();
        const Type_fr_hip_link_RF_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_link_RF : public TransformMotion<Type_fr_base_X_fr_hip_link_RF>
    {
        Type_fr_base_X_fr_hip_link_RF();
        const Type_fr_base_X_fr_hip_link_RF& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_RF_X_fr_hip_link_RF : public TransformMotion<Type_fr_thigh_link_RF_X_fr_hip_link_RF>
    {
        Type_fr_thigh_link_RF_X_fr_hip_link_RF();
        const Type_fr_thigh_link_RF_X_fr_hip_link_RF& update(const state_t&);
    };
    
    struct Type_fr_hip_link_RF_X_fr_thigh_link_RF : public TransformMotion<Type_fr_hip_link_RF_X_fr_thigh_link_RF>
    {
        Type_fr_hip_link_RF_X_fr_thigh_link_RF();
        const Type_fr_hip_link_RF_X_fr_thigh_link_RF& update(const state_t&);
    };
    
    struct Type_fr_shank_link_RF_X_fr_thigh_link_RF : public TransformMotion<Type_fr_shank_link_RF_X_fr_thigh_link_RF>
    {
        Type_fr_shank_link_RF_X_fr_thigh_link_RF();
        const Type_fr_shank_link_RF_X_fr_thigh_link_RF& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_RF_X_fr_shank_link_RF : public TransformMotion<Type_fr_thigh_link_RF_X_fr_shank_link_RF>
    {
        Type_fr_thigh_link_RF_X_fr_shank_link_RF();
        const Type_fr_thigh_link_RF_X_fr_shank_link_RF& update(const state_t&);
    };
    
    struct Type_fr_hip_link_LH_X_fr_base : public TransformMotion<Type_fr_hip_link_LH_X_fr_base>
    {
        Type_fr_hip_link_LH_X_fr_base();
        const Type_fr_hip_link_LH_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_link_LH : public TransformMotion<Type_fr_base_X_fr_hip_link_LH>
    {
        Type_fr_base_X_fr_hip_link_LH();
        const Type_fr_base_X_fr_hip_link_LH& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_LH_X_fr_hip_link_LH : public TransformMotion<Type_fr_thigh_link_LH_X_fr_hip_link_LH>
    {
        Type_fr_thigh_link_LH_X_fr_hip_link_LH();
        const Type_fr_thigh_link_LH_X_fr_hip_link_LH& update(const state_t&);
    };
    
    struct Type_fr_hip_link_LH_X_fr_thigh_link_LH : public TransformMotion<Type_fr_hip_link_LH_X_fr_thigh_link_LH>
    {
        Type_fr_hip_link_LH_X_fr_thigh_link_LH();
        const Type_fr_hip_link_LH_X_fr_thigh_link_LH& update(const state_t&);
    };
    
    struct Type_fr_shank_link_LH_X_fr_thigh_link_LH : public TransformMotion<Type_fr_shank_link_LH_X_fr_thigh_link_LH>
    {
        Type_fr_shank_link_LH_X_fr_thigh_link_LH();
        const Type_fr_shank_link_LH_X_fr_thigh_link_LH& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_LH_X_fr_shank_link_LH : public TransformMotion<Type_fr_thigh_link_LH_X_fr_shank_link_LH>
    {
        Type_fr_thigh_link_LH_X_fr_shank_link_LH();
        const Type_fr_thigh_link_LH_X_fr_shank_link_LH& update(const state_t&);
    };
    
    struct Type_fr_hip_link_RH_X_fr_base : public TransformMotion<Type_fr_hip_link_RH_X_fr_base>
    {
        Type_fr_hip_link_RH_X_fr_base();
        const Type_fr_hip_link_RH_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_link_RH : public TransformMotion<Type_fr_base_X_fr_hip_link_RH>
    {
        Type_fr_base_X_fr_hip_link_RH();
        const Type_fr_base_X_fr_hip_link_RH& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_RH_X_fr_hip_link_RH : public TransformMotion<Type_fr_thigh_link_RH_X_fr_hip_link_RH>
    {
        Type_fr_thigh_link_RH_X_fr_hip_link_RH();
        const Type_fr_thigh_link_RH_X_fr_hip_link_RH& update(const state_t&);
    };
    
    struct Type_fr_hip_link_RH_X_fr_thigh_link_RH : public TransformMotion<Type_fr_hip_link_RH_X_fr_thigh_link_RH>
    {
        Type_fr_hip_link_RH_X_fr_thigh_link_RH();
        const Type_fr_hip_link_RH_X_fr_thigh_link_RH& update(const state_t&);
    };
    
    struct Type_fr_shank_link_RH_X_fr_thigh_link_RH : public TransformMotion<Type_fr_shank_link_RH_X_fr_thigh_link_RH>
    {
        Type_fr_shank_link_RH_X_fr_thigh_link_RH();
        const Type_fr_shank_link_RH_X_fr_thigh_link_RH& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_RH_X_fr_shank_link_RH : public TransformMotion<Type_fr_thigh_link_RH_X_fr_shank_link_RH>
    {
        Type_fr_thigh_link_RH_X_fr_shank_link_RH();
        const Type_fr_thigh_link_RH_X_fr_shank_link_RH& update(const state_t&);
    };
    
public:
    MotionTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_base_X_LF_FOOT fr_base_X_LF_FOOT;
    Type_fr_base_X_RF_FOOT fr_base_X_RF_FOOT;
    Type_fr_base_X_LH_FOOT fr_base_X_LH_FOOT;
    Type_fr_base_X_RH_FOOT fr_base_X_RH_FOOT;
    Type_imu_link_X_LF_FOOT imu_link_X_LF_FOOT;
    Type_imu_link_X_RF_FOOT imu_link_X_RF_FOOT;
    Type_imu_link_X_LH_FOOT imu_link_X_LH_FOOT;
    Type_imu_link_X_RH_FOOT imu_link_X_RH_FOOT;
    Type_fr_base_X_fr_hip_joint_LF fr_base_X_fr_hip_joint_LF;
    Type_fr_base_X_fr_elbow_joint_LF fr_base_X_fr_elbow_joint_LF;
    Type_fr_base_X_fr_knee_joint_LF fr_base_X_fr_knee_joint_LF;
    Type_fr_base_X_fr_hip_joint_RF fr_base_X_fr_hip_joint_RF;
    Type_fr_base_X_fr_elbow_joint_RF fr_base_X_fr_elbow_joint_RF;
    Type_fr_base_X_fr_knee_joint_RF fr_base_X_fr_knee_joint_RF;
    Type_fr_base_X_fr_hip_joint_LH fr_base_X_fr_hip_joint_LH;
    Type_fr_base_X_fr_elbow_joint_LH fr_base_X_fr_elbow_joint_LH;
    Type_fr_base_X_fr_knee_joint_LH fr_base_X_fr_knee_joint_LH;
    Type_fr_base_X_fr_hip_joint_RH fr_base_X_fr_hip_joint_RH;
    Type_fr_base_X_fr_elbow_joint_RH fr_base_X_fr_elbow_joint_RH;
    Type_fr_base_X_fr_knee_joint_RH fr_base_X_fr_knee_joint_RH;
    Type_imu_link_X_fr_hip_joint_LF imu_link_X_fr_hip_joint_LF;
    Type_imu_link_X_fr_elbow_joint_LF imu_link_X_fr_elbow_joint_LF;
    Type_imu_link_X_fr_knee_joint_LF imu_link_X_fr_knee_joint_LF;
    Type_imu_link_X_fr_hip_joint_RF imu_link_X_fr_hip_joint_RF;
    Type_imu_link_X_fr_elbow_joint_RF imu_link_X_fr_elbow_joint_RF;
    Type_imu_link_X_fr_knee_joint_RF imu_link_X_fr_knee_joint_RF;
    Type_imu_link_X_fr_hip_joint_LH imu_link_X_fr_hip_joint_LH;
    Type_imu_link_X_fr_elbow_joint_LH imu_link_X_fr_elbow_joint_LH;
    Type_imu_link_X_fr_knee_joint_LH imu_link_X_fr_knee_joint_LH;
    Type_imu_link_X_fr_hip_joint_RH imu_link_X_fr_hip_joint_RH;
    Type_imu_link_X_fr_elbow_joint_RH imu_link_X_fr_elbow_joint_RH;
    Type_imu_link_X_fr_knee_joint_RH imu_link_X_fr_knee_joint_RH;
    Type_fr_hip_link_LF_X_fr_base fr_hip_link_LF_X_fr_base;
    Type_fr_base_X_fr_hip_link_LF fr_base_X_fr_hip_link_LF;
    Type_fr_thigh_link_LF_X_fr_hip_link_LF fr_thigh_link_LF_X_fr_hip_link_LF;
    Type_fr_hip_link_LF_X_fr_thigh_link_LF fr_hip_link_LF_X_fr_thigh_link_LF;
    Type_fr_shank_link_LF_X_fr_thigh_link_LF fr_shank_link_LF_X_fr_thigh_link_LF;
    Type_fr_thigh_link_LF_X_fr_shank_link_LF fr_thigh_link_LF_X_fr_shank_link_LF;
    Type_fr_hip_link_RF_X_fr_base fr_hip_link_RF_X_fr_base;
    Type_fr_base_X_fr_hip_link_RF fr_base_X_fr_hip_link_RF;
    Type_fr_thigh_link_RF_X_fr_hip_link_RF fr_thigh_link_RF_X_fr_hip_link_RF;
    Type_fr_hip_link_RF_X_fr_thigh_link_RF fr_hip_link_RF_X_fr_thigh_link_RF;
    Type_fr_shank_link_RF_X_fr_thigh_link_RF fr_shank_link_RF_X_fr_thigh_link_RF;
    Type_fr_thigh_link_RF_X_fr_shank_link_RF fr_thigh_link_RF_X_fr_shank_link_RF;
    Type_fr_hip_link_LH_X_fr_base fr_hip_link_LH_X_fr_base;
    Type_fr_base_X_fr_hip_link_LH fr_base_X_fr_hip_link_LH;
    Type_fr_thigh_link_LH_X_fr_hip_link_LH fr_thigh_link_LH_X_fr_hip_link_LH;
    Type_fr_hip_link_LH_X_fr_thigh_link_LH fr_hip_link_LH_X_fr_thigh_link_LH;
    Type_fr_shank_link_LH_X_fr_thigh_link_LH fr_shank_link_LH_X_fr_thigh_link_LH;
    Type_fr_thigh_link_LH_X_fr_shank_link_LH fr_thigh_link_LH_X_fr_shank_link_LH;
    Type_fr_hip_link_RH_X_fr_base fr_hip_link_RH_X_fr_base;
    Type_fr_base_X_fr_hip_link_RH fr_base_X_fr_hip_link_RH;
    Type_fr_thigh_link_RH_X_fr_hip_link_RH fr_thigh_link_RH_X_fr_hip_link_RH;
    Type_fr_hip_link_RH_X_fr_thigh_link_RH fr_hip_link_RH_X_fr_thigh_link_RH;
    Type_fr_shank_link_RH_X_fr_thigh_link_RH fr_shank_link_RH_X_fr_thigh_link_RH;
    Type_fr_thigh_link_RH_X_fr_shank_link_RH fr_thigh_link_RH_X_fr_shank_link_RH;

protected:
    Parameters params;

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
class ForceTransforms
{
public:
    class Dummy {};
    typedef TransformForce<Dummy>::MatrixType MatrixType;

    struct Type_fr_base_X_LF_FOOT : public TransformForce<Type_fr_base_X_LF_FOOT>
    {
        Type_fr_base_X_LF_FOOT();
        const Type_fr_base_X_LF_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_RF_FOOT : public TransformForce<Type_fr_base_X_RF_FOOT>
    {
        Type_fr_base_X_RF_FOOT();
        const Type_fr_base_X_RF_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_LH_FOOT : public TransformForce<Type_fr_base_X_LH_FOOT>
    {
        Type_fr_base_X_LH_FOOT();
        const Type_fr_base_X_LH_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_RH_FOOT : public TransformForce<Type_fr_base_X_RH_FOOT>
    {
        Type_fr_base_X_RH_FOOT();
        const Type_fr_base_X_RH_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_LF_FOOT : public TransformForce<Type_imu_link_X_LF_FOOT>
    {
        Type_imu_link_X_LF_FOOT();
        const Type_imu_link_X_LF_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_RF_FOOT : public TransformForce<Type_imu_link_X_RF_FOOT>
    {
        Type_imu_link_X_RF_FOOT();
        const Type_imu_link_X_RF_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_LH_FOOT : public TransformForce<Type_imu_link_X_LH_FOOT>
    {
        Type_imu_link_X_LH_FOOT();
        const Type_imu_link_X_LH_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_RH_FOOT : public TransformForce<Type_imu_link_X_RH_FOOT>
    {
        Type_imu_link_X_RH_FOOT();
        const Type_imu_link_X_RH_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_joint_LF : public TransformForce<Type_fr_base_X_fr_hip_joint_LF>
    {
        Type_fr_base_X_fr_hip_joint_LF();
        const Type_fr_base_X_fr_hip_joint_LF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_elbow_joint_LF : public TransformForce<Type_fr_base_X_fr_elbow_joint_LF>
    {
        Type_fr_base_X_fr_elbow_joint_LF();
        const Type_fr_base_X_fr_elbow_joint_LF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_knee_joint_LF : public TransformForce<Type_fr_base_X_fr_knee_joint_LF>
    {
        Type_fr_base_X_fr_knee_joint_LF();
        const Type_fr_base_X_fr_knee_joint_LF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_joint_RF : public TransformForce<Type_fr_base_X_fr_hip_joint_RF>
    {
        Type_fr_base_X_fr_hip_joint_RF();
        const Type_fr_base_X_fr_hip_joint_RF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_elbow_joint_RF : public TransformForce<Type_fr_base_X_fr_elbow_joint_RF>
    {
        Type_fr_base_X_fr_elbow_joint_RF();
        const Type_fr_base_X_fr_elbow_joint_RF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_knee_joint_RF : public TransformForce<Type_fr_base_X_fr_knee_joint_RF>
    {
        Type_fr_base_X_fr_knee_joint_RF();
        const Type_fr_base_X_fr_knee_joint_RF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_joint_LH : public TransformForce<Type_fr_base_X_fr_hip_joint_LH>
    {
        Type_fr_base_X_fr_hip_joint_LH();
        const Type_fr_base_X_fr_hip_joint_LH& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_elbow_joint_LH : public TransformForce<Type_fr_base_X_fr_elbow_joint_LH>
    {
        Type_fr_base_X_fr_elbow_joint_LH();
        const Type_fr_base_X_fr_elbow_joint_LH& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_knee_joint_LH : public TransformForce<Type_fr_base_X_fr_knee_joint_LH>
    {
        Type_fr_base_X_fr_knee_joint_LH();
        const Type_fr_base_X_fr_knee_joint_LH& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_joint_RH : public TransformForce<Type_fr_base_X_fr_hip_joint_RH>
    {
        Type_fr_base_X_fr_hip_joint_RH();
        const Type_fr_base_X_fr_hip_joint_RH& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_elbow_joint_RH : public TransformForce<Type_fr_base_X_fr_elbow_joint_RH>
    {
        Type_fr_base_X_fr_elbow_joint_RH();
        const Type_fr_base_X_fr_elbow_joint_RH& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_knee_joint_RH : public TransformForce<Type_fr_base_X_fr_knee_joint_RH>
    {
        Type_fr_base_X_fr_knee_joint_RH();
        const Type_fr_base_X_fr_knee_joint_RH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_hip_joint_LF : public TransformForce<Type_imu_link_X_fr_hip_joint_LF>
    {
        Type_imu_link_X_fr_hip_joint_LF();
        const Type_imu_link_X_fr_hip_joint_LF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_elbow_joint_LF : public TransformForce<Type_imu_link_X_fr_elbow_joint_LF>
    {
        Type_imu_link_X_fr_elbow_joint_LF();
        const Type_imu_link_X_fr_elbow_joint_LF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_knee_joint_LF : public TransformForce<Type_imu_link_X_fr_knee_joint_LF>
    {
        Type_imu_link_X_fr_knee_joint_LF();
        const Type_imu_link_X_fr_knee_joint_LF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_hip_joint_RF : public TransformForce<Type_imu_link_X_fr_hip_joint_RF>
    {
        Type_imu_link_X_fr_hip_joint_RF();
        const Type_imu_link_X_fr_hip_joint_RF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_elbow_joint_RF : public TransformForce<Type_imu_link_X_fr_elbow_joint_RF>
    {
        Type_imu_link_X_fr_elbow_joint_RF();
        const Type_imu_link_X_fr_elbow_joint_RF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_knee_joint_RF : public TransformForce<Type_imu_link_X_fr_knee_joint_RF>
    {
        Type_imu_link_X_fr_knee_joint_RF();
        const Type_imu_link_X_fr_knee_joint_RF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_hip_joint_LH : public TransformForce<Type_imu_link_X_fr_hip_joint_LH>
    {
        Type_imu_link_X_fr_hip_joint_LH();
        const Type_imu_link_X_fr_hip_joint_LH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_elbow_joint_LH : public TransformForce<Type_imu_link_X_fr_elbow_joint_LH>
    {
        Type_imu_link_X_fr_elbow_joint_LH();
        const Type_imu_link_X_fr_elbow_joint_LH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_knee_joint_LH : public TransformForce<Type_imu_link_X_fr_knee_joint_LH>
    {
        Type_imu_link_X_fr_knee_joint_LH();
        const Type_imu_link_X_fr_knee_joint_LH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_hip_joint_RH : public TransformForce<Type_imu_link_X_fr_hip_joint_RH>
    {
        Type_imu_link_X_fr_hip_joint_RH();
        const Type_imu_link_X_fr_hip_joint_RH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_elbow_joint_RH : public TransformForce<Type_imu_link_X_fr_elbow_joint_RH>
    {
        Type_imu_link_X_fr_elbow_joint_RH();
        const Type_imu_link_X_fr_elbow_joint_RH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_knee_joint_RH : public TransformForce<Type_imu_link_X_fr_knee_joint_RH>
    {
        Type_imu_link_X_fr_knee_joint_RH();
        const Type_imu_link_X_fr_knee_joint_RH& update(const state_t&);
    };
    
    struct Type_fr_hip_link_LF_X_fr_base : public TransformForce<Type_fr_hip_link_LF_X_fr_base>
    {
        Type_fr_hip_link_LF_X_fr_base();
        const Type_fr_hip_link_LF_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_link_LF : public TransformForce<Type_fr_base_X_fr_hip_link_LF>
    {
        Type_fr_base_X_fr_hip_link_LF();
        const Type_fr_base_X_fr_hip_link_LF& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_LF_X_fr_hip_link_LF : public TransformForce<Type_fr_thigh_link_LF_X_fr_hip_link_LF>
    {
        Type_fr_thigh_link_LF_X_fr_hip_link_LF();
        const Type_fr_thigh_link_LF_X_fr_hip_link_LF& update(const state_t&);
    };
    
    struct Type_fr_hip_link_LF_X_fr_thigh_link_LF : public TransformForce<Type_fr_hip_link_LF_X_fr_thigh_link_LF>
    {
        Type_fr_hip_link_LF_X_fr_thigh_link_LF();
        const Type_fr_hip_link_LF_X_fr_thigh_link_LF& update(const state_t&);
    };
    
    struct Type_fr_shank_link_LF_X_fr_thigh_link_LF : public TransformForce<Type_fr_shank_link_LF_X_fr_thigh_link_LF>
    {
        Type_fr_shank_link_LF_X_fr_thigh_link_LF();
        const Type_fr_shank_link_LF_X_fr_thigh_link_LF& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_LF_X_fr_shank_link_LF : public TransformForce<Type_fr_thigh_link_LF_X_fr_shank_link_LF>
    {
        Type_fr_thigh_link_LF_X_fr_shank_link_LF();
        const Type_fr_thigh_link_LF_X_fr_shank_link_LF& update(const state_t&);
    };
    
    struct Type_fr_hip_link_RF_X_fr_base : public TransformForce<Type_fr_hip_link_RF_X_fr_base>
    {
        Type_fr_hip_link_RF_X_fr_base();
        const Type_fr_hip_link_RF_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_link_RF : public TransformForce<Type_fr_base_X_fr_hip_link_RF>
    {
        Type_fr_base_X_fr_hip_link_RF();
        const Type_fr_base_X_fr_hip_link_RF& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_RF_X_fr_hip_link_RF : public TransformForce<Type_fr_thigh_link_RF_X_fr_hip_link_RF>
    {
        Type_fr_thigh_link_RF_X_fr_hip_link_RF();
        const Type_fr_thigh_link_RF_X_fr_hip_link_RF& update(const state_t&);
    };
    
    struct Type_fr_hip_link_RF_X_fr_thigh_link_RF : public TransformForce<Type_fr_hip_link_RF_X_fr_thigh_link_RF>
    {
        Type_fr_hip_link_RF_X_fr_thigh_link_RF();
        const Type_fr_hip_link_RF_X_fr_thigh_link_RF& update(const state_t&);
    };
    
    struct Type_fr_shank_link_RF_X_fr_thigh_link_RF : public TransformForce<Type_fr_shank_link_RF_X_fr_thigh_link_RF>
    {
        Type_fr_shank_link_RF_X_fr_thigh_link_RF();
        const Type_fr_shank_link_RF_X_fr_thigh_link_RF& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_RF_X_fr_shank_link_RF : public TransformForce<Type_fr_thigh_link_RF_X_fr_shank_link_RF>
    {
        Type_fr_thigh_link_RF_X_fr_shank_link_RF();
        const Type_fr_thigh_link_RF_X_fr_shank_link_RF& update(const state_t&);
    };
    
    struct Type_fr_hip_link_LH_X_fr_base : public TransformForce<Type_fr_hip_link_LH_X_fr_base>
    {
        Type_fr_hip_link_LH_X_fr_base();
        const Type_fr_hip_link_LH_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_link_LH : public TransformForce<Type_fr_base_X_fr_hip_link_LH>
    {
        Type_fr_base_X_fr_hip_link_LH();
        const Type_fr_base_X_fr_hip_link_LH& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_LH_X_fr_hip_link_LH : public TransformForce<Type_fr_thigh_link_LH_X_fr_hip_link_LH>
    {
        Type_fr_thigh_link_LH_X_fr_hip_link_LH();
        const Type_fr_thigh_link_LH_X_fr_hip_link_LH& update(const state_t&);
    };
    
    struct Type_fr_hip_link_LH_X_fr_thigh_link_LH : public TransformForce<Type_fr_hip_link_LH_X_fr_thigh_link_LH>
    {
        Type_fr_hip_link_LH_X_fr_thigh_link_LH();
        const Type_fr_hip_link_LH_X_fr_thigh_link_LH& update(const state_t&);
    };
    
    struct Type_fr_shank_link_LH_X_fr_thigh_link_LH : public TransformForce<Type_fr_shank_link_LH_X_fr_thigh_link_LH>
    {
        Type_fr_shank_link_LH_X_fr_thigh_link_LH();
        const Type_fr_shank_link_LH_X_fr_thigh_link_LH& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_LH_X_fr_shank_link_LH : public TransformForce<Type_fr_thigh_link_LH_X_fr_shank_link_LH>
    {
        Type_fr_thigh_link_LH_X_fr_shank_link_LH();
        const Type_fr_thigh_link_LH_X_fr_shank_link_LH& update(const state_t&);
    };
    
    struct Type_fr_hip_link_RH_X_fr_base : public TransformForce<Type_fr_hip_link_RH_X_fr_base>
    {
        Type_fr_hip_link_RH_X_fr_base();
        const Type_fr_hip_link_RH_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_link_RH : public TransformForce<Type_fr_base_X_fr_hip_link_RH>
    {
        Type_fr_base_X_fr_hip_link_RH();
        const Type_fr_base_X_fr_hip_link_RH& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_RH_X_fr_hip_link_RH : public TransformForce<Type_fr_thigh_link_RH_X_fr_hip_link_RH>
    {
        Type_fr_thigh_link_RH_X_fr_hip_link_RH();
        const Type_fr_thigh_link_RH_X_fr_hip_link_RH& update(const state_t&);
    };
    
    struct Type_fr_hip_link_RH_X_fr_thigh_link_RH : public TransformForce<Type_fr_hip_link_RH_X_fr_thigh_link_RH>
    {
        Type_fr_hip_link_RH_X_fr_thigh_link_RH();
        const Type_fr_hip_link_RH_X_fr_thigh_link_RH& update(const state_t&);
    };
    
    struct Type_fr_shank_link_RH_X_fr_thigh_link_RH : public TransformForce<Type_fr_shank_link_RH_X_fr_thigh_link_RH>
    {
        Type_fr_shank_link_RH_X_fr_thigh_link_RH();
        const Type_fr_shank_link_RH_X_fr_thigh_link_RH& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_RH_X_fr_shank_link_RH : public TransformForce<Type_fr_thigh_link_RH_X_fr_shank_link_RH>
    {
        Type_fr_thigh_link_RH_X_fr_shank_link_RH();
        const Type_fr_thigh_link_RH_X_fr_shank_link_RH& update(const state_t&);
    };
    
public:
    ForceTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_base_X_LF_FOOT fr_base_X_LF_FOOT;
    Type_fr_base_X_RF_FOOT fr_base_X_RF_FOOT;
    Type_fr_base_X_LH_FOOT fr_base_X_LH_FOOT;
    Type_fr_base_X_RH_FOOT fr_base_X_RH_FOOT;
    Type_imu_link_X_LF_FOOT imu_link_X_LF_FOOT;
    Type_imu_link_X_RF_FOOT imu_link_X_RF_FOOT;
    Type_imu_link_X_LH_FOOT imu_link_X_LH_FOOT;
    Type_imu_link_X_RH_FOOT imu_link_X_RH_FOOT;
    Type_fr_base_X_fr_hip_joint_LF fr_base_X_fr_hip_joint_LF;
    Type_fr_base_X_fr_elbow_joint_LF fr_base_X_fr_elbow_joint_LF;
    Type_fr_base_X_fr_knee_joint_LF fr_base_X_fr_knee_joint_LF;
    Type_fr_base_X_fr_hip_joint_RF fr_base_X_fr_hip_joint_RF;
    Type_fr_base_X_fr_elbow_joint_RF fr_base_X_fr_elbow_joint_RF;
    Type_fr_base_X_fr_knee_joint_RF fr_base_X_fr_knee_joint_RF;
    Type_fr_base_X_fr_hip_joint_LH fr_base_X_fr_hip_joint_LH;
    Type_fr_base_X_fr_elbow_joint_LH fr_base_X_fr_elbow_joint_LH;
    Type_fr_base_X_fr_knee_joint_LH fr_base_X_fr_knee_joint_LH;
    Type_fr_base_X_fr_hip_joint_RH fr_base_X_fr_hip_joint_RH;
    Type_fr_base_X_fr_elbow_joint_RH fr_base_X_fr_elbow_joint_RH;
    Type_fr_base_X_fr_knee_joint_RH fr_base_X_fr_knee_joint_RH;
    Type_imu_link_X_fr_hip_joint_LF imu_link_X_fr_hip_joint_LF;
    Type_imu_link_X_fr_elbow_joint_LF imu_link_X_fr_elbow_joint_LF;
    Type_imu_link_X_fr_knee_joint_LF imu_link_X_fr_knee_joint_LF;
    Type_imu_link_X_fr_hip_joint_RF imu_link_X_fr_hip_joint_RF;
    Type_imu_link_X_fr_elbow_joint_RF imu_link_X_fr_elbow_joint_RF;
    Type_imu_link_X_fr_knee_joint_RF imu_link_X_fr_knee_joint_RF;
    Type_imu_link_X_fr_hip_joint_LH imu_link_X_fr_hip_joint_LH;
    Type_imu_link_X_fr_elbow_joint_LH imu_link_X_fr_elbow_joint_LH;
    Type_imu_link_X_fr_knee_joint_LH imu_link_X_fr_knee_joint_LH;
    Type_imu_link_X_fr_hip_joint_RH imu_link_X_fr_hip_joint_RH;
    Type_imu_link_X_fr_elbow_joint_RH imu_link_X_fr_elbow_joint_RH;
    Type_imu_link_X_fr_knee_joint_RH imu_link_X_fr_knee_joint_RH;
    Type_fr_hip_link_LF_X_fr_base fr_hip_link_LF_X_fr_base;
    Type_fr_base_X_fr_hip_link_LF fr_base_X_fr_hip_link_LF;
    Type_fr_thigh_link_LF_X_fr_hip_link_LF fr_thigh_link_LF_X_fr_hip_link_LF;
    Type_fr_hip_link_LF_X_fr_thigh_link_LF fr_hip_link_LF_X_fr_thigh_link_LF;
    Type_fr_shank_link_LF_X_fr_thigh_link_LF fr_shank_link_LF_X_fr_thigh_link_LF;
    Type_fr_thigh_link_LF_X_fr_shank_link_LF fr_thigh_link_LF_X_fr_shank_link_LF;
    Type_fr_hip_link_RF_X_fr_base fr_hip_link_RF_X_fr_base;
    Type_fr_base_X_fr_hip_link_RF fr_base_X_fr_hip_link_RF;
    Type_fr_thigh_link_RF_X_fr_hip_link_RF fr_thigh_link_RF_X_fr_hip_link_RF;
    Type_fr_hip_link_RF_X_fr_thigh_link_RF fr_hip_link_RF_X_fr_thigh_link_RF;
    Type_fr_shank_link_RF_X_fr_thigh_link_RF fr_shank_link_RF_X_fr_thigh_link_RF;
    Type_fr_thigh_link_RF_X_fr_shank_link_RF fr_thigh_link_RF_X_fr_shank_link_RF;
    Type_fr_hip_link_LH_X_fr_base fr_hip_link_LH_X_fr_base;
    Type_fr_base_X_fr_hip_link_LH fr_base_X_fr_hip_link_LH;
    Type_fr_thigh_link_LH_X_fr_hip_link_LH fr_thigh_link_LH_X_fr_hip_link_LH;
    Type_fr_hip_link_LH_X_fr_thigh_link_LH fr_hip_link_LH_X_fr_thigh_link_LH;
    Type_fr_shank_link_LH_X_fr_thigh_link_LH fr_shank_link_LH_X_fr_thigh_link_LH;
    Type_fr_thigh_link_LH_X_fr_shank_link_LH fr_thigh_link_LH_X_fr_shank_link_LH;
    Type_fr_hip_link_RH_X_fr_base fr_hip_link_RH_X_fr_base;
    Type_fr_base_X_fr_hip_link_RH fr_base_X_fr_hip_link_RH;
    Type_fr_thigh_link_RH_X_fr_hip_link_RH fr_thigh_link_RH_X_fr_hip_link_RH;
    Type_fr_hip_link_RH_X_fr_thigh_link_RH fr_hip_link_RH_X_fr_thigh_link_RH;
    Type_fr_shank_link_RH_X_fr_thigh_link_RH fr_shank_link_RH_X_fr_thigh_link_RH;
    Type_fr_thigh_link_RH_X_fr_shank_link_RH fr_thigh_link_RH_X_fr_shank_link_RH;

protected:
    Parameters params;

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
class HomogeneousTransforms
{
public:
    class Dummy {};
    typedef TransformHomogeneous<Dummy>::MatrixType MatrixType;

    struct Type_fr_base_X_LF_FOOT : public TransformHomogeneous<Type_fr_base_X_LF_FOOT>
    {
        Type_fr_base_X_LF_FOOT();
        const Type_fr_base_X_LF_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_RF_FOOT : public TransformHomogeneous<Type_fr_base_X_RF_FOOT>
    {
        Type_fr_base_X_RF_FOOT();
        const Type_fr_base_X_RF_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_LH_FOOT : public TransformHomogeneous<Type_fr_base_X_LH_FOOT>
    {
        Type_fr_base_X_LH_FOOT();
        const Type_fr_base_X_LH_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_RH_FOOT : public TransformHomogeneous<Type_fr_base_X_RH_FOOT>
    {
        Type_fr_base_X_RH_FOOT();
        const Type_fr_base_X_RH_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_LF_FOOT : public TransformHomogeneous<Type_imu_link_X_LF_FOOT>
    {
        Type_imu_link_X_LF_FOOT();
        const Type_imu_link_X_LF_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_RF_FOOT : public TransformHomogeneous<Type_imu_link_X_RF_FOOT>
    {
        Type_imu_link_X_RF_FOOT();
        const Type_imu_link_X_RF_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_LH_FOOT : public TransformHomogeneous<Type_imu_link_X_LH_FOOT>
    {
        Type_imu_link_X_LH_FOOT();
        const Type_imu_link_X_LH_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_RH_FOOT : public TransformHomogeneous<Type_imu_link_X_RH_FOOT>
    {
        Type_imu_link_X_RH_FOOT();
        const Type_imu_link_X_RH_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_joint_LF : public TransformHomogeneous<Type_fr_base_X_fr_hip_joint_LF>
    {
        Type_fr_base_X_fr_hip_joint_LF();
        const Type_fr_base_X_fr_hip_joint_LF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_elbow_joint_LF : public TransformHomogeneous<Type_fr_base_X_fr_elbow_joint_LF>
    {
        Type_fr_base_X_fr_elbow_joint_LF();
        const Type_fr_base_X_fr_elbow_joint_LF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_knee_joint_LF : public TransformHomogeneous<Type_fr_base_X_fr_knee_joint_LF>
    {
        Type_fr_base_X_fr_knee_joint_LF();
        const Type_fr_base_X_fr_knee_joint_LF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_joint_RF : public TransformHomogeneous<Type_fr_base_X_fr_hip_joint_RF>
    {
        Type_fr_base_X_fr_hip_joint_RF();
        const Type_fr_base_X_fr_hip_joint_RF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_elbow_joint_RF : public TransformHomogeneous<Type_fr_base_X_fr_elbow_joint_RF>
    {
        Type_fr_base_X_fr_elbow_joint_RF();
        const Type_fr_base_X_fr_elbow_joint_RF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_knee_joint_RF : public TransformHomogeneous<Type_fr_base_X_fr_knee_joint_RF>
    {
        Type_fr_base_X_fr_knee_joint_RF();
        const Type_fr_base_X_fr_knee_joint_RF& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_joint_LH : public TransformHomogeneous<Type_fr_base_X_fr_hip_joint_LH>
    {
        Type_fr_base_X_fr_hip_joint_LH();
        const Type_fr_base_X_fr_hip_joint_LH& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_elbow_joint_LH : public TransformHomogeneous<Type_fr_base_X_fr_elbow_joint_LH>
    {
        Type_fr_base_X_fr_elbow_joint_LH();
        const Type_fr_base_X_fr_elbow_joint_LH& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_knee_joint_LH : public TransformHomogeneous<Type_fr_base_X_fr_knee_joint_LH>
    {
        Type_fr_base_X_fr_knee_joint_LH();
        const Type_fr_base_X_fr_knee_joint_LH& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_joint_RH : public TransformHomogeneous<Type_fr_base_X_fr_hip_joint_RH>
    {
        Type_fr_base_X_fr_hip_joint_RH();
        const Type_fr_base_X_fr_hip_joint_RH& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_elbow_joint_RH : public TransformHomogeneous<Type_fr_base_X_fr_elbow_joint_RH>
    {
        Type_fr_base_X_fr_elbow_joint_RH();
        const Type_fr_base_X_fr_elbow_joint_RH& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_knee_joint_RH : public TransformHomogeneous<Type_fr_base_X_fr_knee_joint_RH>
    {
        Type_fr_base_X_fr_knee_joint_RH();
        const Type_fr_base_X_fr_knee_joint_RH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_hip_joint_LF : public TransformHomogeneous<Type_imu_link_X_fr_hip_joint_LF>
    {
        Type_imu_link_X_fr_hip_joint_LF();
        const Type_imu_link_X_fr_hip_joint_LF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_elbow_joint_LF : public TransformHomogeneous<Type_imu_link_X_fr_elbow_joint_LF>
    {
        Type_imu_link_X_fr_elbow_joint_LF();
        const Type_imu_link_X_fr_elbow_joint_LF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_knee_joint_LF : public TransformHomogeneous<Type_imu_link_X_fr_knee_joint_LF>
    {
        Type_imu_link_X_fr_knee_joint_LF();
        const Type_imu_link_X_fr_knee_joint_LF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_hip_joint_RF : public TransformHomogeneous<Type_imu_link_X_fr_hip_joint_RF>
    {
        Type_imu_link_X_fr_hip_joint_RF();
        const Type_imu_link_X_fr_hip_joint_RF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_elbow_joint_RF : public TransformHomogeneous<Type_imu_link_X_fr_elbow_joint_RF>
    {
        Type_imu_link_X_fr_elbow_joint_RF();
        const Type_imu_link_X_fr_elbow_joint_RF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_knee_joint_RF : public TransformHomogeneous<Type_imu_link_X_fr_knee_joint_RF>
    {
        Type_imu_link_X_fr_knee_joint_RF();
        const Type_imu_link_X_fr_knee_joint_RF& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_hip_joint_LH : public TransformHomogeneous<Type_imu_link_X_fr_hip_joint_LH>
    {
        Type_imu_link_X_fr_hip_joint_LH();
        const Type_imu_link_X_fr_hip_joint_LH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_elbow_joint_LH : public TransformHomogeneous<Type_imu_link_X_fr_elbow_joint_LH>
    {
        Type_imu_link_X_fr_elbow_joint_LH();
        const Type_imu_link_X_fr_elbow_joint_LH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_knee_joint_LH : public TransformHomogeneous<Type_imu_link_X_fr_knee_joint_LH>
    {
        Type_imu_link_X_fr_knee_joint_LH();
        const Type_imu_link_X_fr_knee_joint_LH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_hip_joint_RH : public TransformHomogeneous<Type_imu_link_X_fr_hip_joint_RH>
    {
        Type_imu_link_X_fr_hip_joint_RH();
        const Type_imu_link_X_fr_hip_joint_RH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_elbow_joint_RH : public TransformHomogeneous<Type_imu_link_X_fr_elbow_joint_RH>
    {
        Type_imu_link_X_fr_elbow_joint_RH();
        const Type_imu_link_X_fr_elbow_joint_RH& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_knee_joint_RH : public TransformHomogeneous<Type_imu_link_X_fr_knee_joint_RH>
    {
        Type_imu_link_X_fr_knee_joint_RH();
        const Type_imu_link_X_fr_knee_joint_RH& update(const state_t&);
    };
    
    struct Type_fr_hip_link_LF_X_fr_base : public TransformHomogeneous<Type_fr_hip_link_LF_X_fr_base>
    {
        Type_fr_hip_link_LF_X_fr_base();
        const Type_fr_hip_link_LF_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_link_LF : public TransformHomogeneous<Type_fr_base_X_fr_hip_link_LF>
    {
        Type_fr_base_X_fr_hip_link_LF();
        const Type_fr_base_X_fr_hip_link_LF& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_LF_X_fr_hip_link_LF : public TransformHomogeneous<Type_fr_thigh_link_LF_X_fr_hip_link_LF>
    {
        Type_fr_thigh_link_LF_X_fr_hip_link_LF();
        const Type_fr_thigh_link_LF_X_fr_hip_link_LF& update(const state_t&);
    };
    
    struct Type_fr_hip_link_LF_X_fr_thigh_link_LF : public TransformHomogeneous<Type_fr_hip_link_LF_X_fr_thigh_link_LF>
    {
        Type_fr_hip_link_LF_X_fr_thigh_link_LF();
        const Type_fr_hip_link_LF_X_fr_thigh_link_LF& update(const state_t&);
    };
    
    struct Type_fr_shank_link_LF_X_fr_thigh_link_LF : public TransformHomogeneous<Type_fr_shank_link_LF_X_fr_thigh_link_LF>
    {
        Type_fr_shank_link_LF_X_fr_thigh_link_LF();
        const Type_fr_shank_link_LF_X_fr_thigh_link_LF& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_LF_X_fr_shank_link_LF : public TransformHomogeneous<Type_fr_thigh_link_LF_X_fr_shank_link_LF>
    {
        Type_fr_thigh_link_LF_X_fr_shank_link_LF();
        const Type_fr_thigh_link_LF_X_fr_shank_link_LF& update(const state_t&);
    };
    
    struct Type_fr_hip_link_RF_X_fr_base : public TransformHomogeneous<Type_fr_hip_link_RF_X_fr_base>
    {
        Type_fr_hip_link_RF_X_fr_base();
        const Type_fr_hip_link_RF_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_link_RF : public TransformHomogeneous<Type_fr_base_X_fr_hip_link_RF>
    {
        Type_fr_base_X_fr_hip_link_RF();
        const Type_fr_base_X_fr_hip_link_RF& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_RF_X_fr_hip_link_RF : public TransformHomogeneous<Type_fr_thigh_link_RF_X_fr_hip_link_RF>
    {
        Type_fr_thigh_link_RF_X_fr_hip_link_RF();
        const Type_fr_thigh_link_RF_X_fr_hip_link_RF& update(const state_t&);
    };
    
    struct Type_fr_hip_link_RF_X_fr_thigh_link_RF : public TransformHomogeneous<Type_fr_hip_link_RF_X_fr_thigh_link_RF>
    {
        Type_fr_hip_link_RF_X_fr_thigh_link_RF();
        const Type_fr_hip_link_RF_X_fr_thigh_link_RF& update(const state_t&);
    };
    
    struct Type_fr_shank_link_RF_X_fr_thigh_link_RF : public TransformHomogeneous<Type_fr_shank_link_RF_X_fr_thigh_link_RF>
    {
        Type_fr_shank_link_RF_X_fr_thigh_link_RF();
        const Type_fr_shank_link_RF_X_fr_thigh_link_RF& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_RF_X_fr_shank_link_RF : public TransformHomogeneous<Type_fr_thigh_link_RF_X_fr_shank_link_RF>
    {
        Type_fr_thigh_link_RF_X_fr_shank_link_RF();
        const Type_fr_thigh_link_RF_X_fr_shank_link_RF& update(const state_t&);
    };
    
    struct Type_fr_hip_link_LH_X_fr_base : public TransformHomogeneous<Type_fr_hip_link_LH_X_fr_base>
    {
        Type_fr_hip_link_LH_X_fr_base();
        const Type_fr_hip_link_LH_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_link_LH : public TransformHomogeneous<Type_fr_base_X_fr_hip_link_LH>
    {
        Type_fr_base_X_fr_hip_link_LH();
        const Type_fr_base_X_fr_hip_link_LH& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_LH_X_fr_hip_link_LH : public TransformHomogeneous<Type_fr_thigh_link_LH_X_fr_hip_link_LH>
    {
        Type_fr_thigh_link_LH_X_fr_hip_link_LH();
        const Type_fr_thigh_link_LH_X_fr_hip_link_LH& update(const state_t&);
    };
    
    struct Type_fr_hip_link_LH_X_fr_thigh_link_LH : public TransformHomogeneous<Type_fr_hip_link_LH_X_fr_thigh_link_LH>
    {
        Type_fr_hip_link_LH_X_fr_thigh_link_LH();
        const Type_fr_hip_link_LH_X_fr_thigh_link_LH& update(const state_t&);
    };
    
    struct Type_fr_shank_link_LH_X_fr_thigh_link_LH : public TransformHomogeneous<Type_fr_shank_link_LH_X_fr_thigh_link_LH>
    {
        Type_fr_shank_link_LH_X_fr_thigh_link_LH();
        const Type_fr_shank_link_LH_X_fr_thigh_link_LH& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_LH_X_fr_shank_link_LH : public TransformHomogeneous<Type_fr_thigh_link_LH_X_fr_shank_link_LH>
    {
        Type_fr_thigh_link_LH_X_fr_shank_link_LH();
        const Type_fr_thigh_link_LH_X_fr_shank_link_LH& update(const state_t&);
    };
    
    struct Type_fr_hip_link_RH_X_fr_base : public TransformHomogeneous<Type_fr_hip_link_RH_X_fr_base>
    {
        Type_fr_hip_link_RH_X_fr_base();
        const Type_fr_hip_link_RH_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_hip_link_RH : public TransformHomogeneous<Type_fr_base_X_fr_hip_link_RH>
    {
        Type_fr_base_X_fr_hip_link_RH();
        const Type_fr_base_X_fr_hip_link_RH& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_RH_X_fr_hip_link_RH : public TransformHomogeneous<Type_fr_thigh_link_RH_X_fr_hip_link_RH>
    {
        Type_fr_thigh_link_RH_X_fr_hip_link_RH();
        const Type_fr_thigh_link_RH_X_fr_hip_link_RH& update(const state_t&);
    };
    
    struct Type_fr_hip_link_RH_X_fr_thigh_link_RH : public TransformHomogeneous<Type_fr_hip_link_RH_X_fr_thigh_link_RH>
    {
        Type_fr_hip_link_RH_X_fr_thigh_link_RH();
        const Type_fr_hip_link_RH_X_fr_thigh_link_RH& update(const state_t&);
    };
    
    struct Type_fr_shank_link_RH_X_fr_thigh_link_RH : public TransformHomogeneous<Type_fr_shank_link_RH_X_fr_thigh_link_RH>
    {
        Type_fr_shank_link_RH_X_fr_thigh_link_RH();
        const Type_fr_shank_link_RH_X_fr_thigh_link_RH& update(const state_t&);
    };
    
    struct Type_fr_thigh_link_RH_X_fr_shank_link_RH : public TransformHomogeneous<Type_fr_thigh_link_RH_X_fr_shank_link_RH>
    {
        Type_fr_thigh_link_RH_X_fr_shank_link_RH();
        const Type_fr_thigh_link_RH_X_fr_shank_link_RH& update(const state_t&);
    };
    
public:
    HomogeneousTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_base_X_LF_FOOT fr_base_X_LF_FOOT;
    Type_fr_base_X_RF_FOOT fr_base_X_RF_FOOT;
    Type_fr_base_X_LH_FOOT fr_base_X_LH_FOOT;
    Type_fr_base_X_RH_FOOT fr_base_X_RH_FOOT;
    Type_imu_link_X_LF_FOOT imu_link_X_LF_FOOT;
    Type_imu_link_X_RF_FOOT imu_link_X_RF_FOOT;
    Type_imu_link_X_LH_FOOT imu_link_X_LH_FOOT;
    Type_imu_link_X_RH_FOOT imu_link_X_RH_FOOT;
    Type_fr_base_X_fr_hip_joint_LF fr_base_X_fr_hip_joint_LF;
    Type_fr_base_X_fr_elbow_joint_LF fr_base_X_fr_elbow_joint_LF;
    Type_fr_base_X_fr_knee_joint_LF fr_base_X_fr_knee_joint_LF;
    Type_fr_base_X_fr_hip_joint_RF fr_base_X_fr_hip_joint_RF;
    Type_fr_base_X_fr_elbow_joint_RF fr_base_X_fr_elbow_joint_RF;
    Type_fr_base_X_fr_knee_joint_RF fr_base_X_fr_knee_joint_RF;
    Type_fr_base_X_fr_hip_joint_LH fr_base_X_fr_hip_joint_LH;
    Type_fr_base_X_fr_elbow_joint_LH fr_base_X_fr_elbow_joint_LH;
    Type_fr_base_X_fr_knee_joint_LH fr_base_X_fr_knee_joint_LH;
    Type_fr_base_X_fr_hip_joint_RH fr_base_X_fr_hip_joint_RH;
    Type_fr_base_X_fr_elbow_joint_RH fr_base_X_fr_elbow_joint_RH;
    Type_fr_base_X_fr_knee_joint_RH fr_base_X_fr_knee_joint_RH;
    Type_imu_link_X_fr_hip_joint_LF imu_link_X_fr_hip_joint_LF;
    Type_imu_link_X_fr_elbow_joint_LF imu_link_X_fr_elbow_joint_LF;
    Type_imu_link_X_fr_knee_joint_LF imu_link_X_fr_knee_joint_LF;
    Type_imu_link_X_fr_hip_joint_RF imu_link_X_fr_hip_joint_RF;
    Type_imu_link_X_fr_elbow_joint_RF imu_link_X_fr_elbow_joint_RF;
    Type_imu_link_X_fr_knee_joint_RF imu_link_X_fr_knee_joint_RF;
    Type_imu_link_X_fr_hip_joint_LH imu_link_X_fr_hip_joint_LH;
    Type_imu_link_X_fr_elbow_joint_LH imu_link_X_fr_elbow_joint_LH;
    Type_imu_link_X_fr_knee_joint_LH imu_link_X_fr_knee_joint_LH;
    Type_imu_link_X_fr_hip_joint_RH imu_link_X_fr_hip_joint_RH;
    Type_imu_link_X_fr_elbow_joint_RH imu_link_X_fr_elbow_joint_RH;
    Type_imu_link_X_fr_knee_joint_RH imu_link_X_fr_knee_joint_RH;
    Type_fr_hip_link_LF_X_fr_base fr_hip_link_LF_X_fr_base;
    Type_fr_base_X_fr_hip_link_LF fr_base_X_fr_hip_link_LF;
    Type_fr_thigh_link_LF_X_fr_hip_link_LF fr_thigh_link_LF_X_fr_hip_link_LF;
    Type_fr_hip_link_LF_X_fr_thigh_link_LF fr_hip_link_LF_X_fr_thigh_link_LF;
    Type_fr_shank_link_LF_X_fr_thigh_link_LF fr_shank_link_LF_X_fr_thigh_link_LF;
    Type_fr_thigh_link_LF_X_fr_shank_link_LF fr_thigh_link_LF_X_fr_shank_link_LF;
    Type_fr_hip_link_RF_X_fr_base fr_hip_link_RF_X_fr_base;
    Type_fr_base_X_fr_hip_link_RF fr_base_X_fr_hip_link_RF;
    Type_fr_thigh_link_RF_X_fr_hip_link_RF fr_thigh_link_RF_X_fr_hip_link_RF;
    Type_fr_hip_link_RF_X_fr_thigh_link_RF fr_hip_link_RF_X_fr_thigh_link_RF;
    Type_fr_shank_link_RF_X_fr_thigh_link_RF fr_shank_link_RF_X_fr_thigh_link_RF;
    Type_fr_thigh_link_RF_X_fr_shank_link_RF fr_thigh_link_RF_X_fr_shank_link_RF;
    Type_fr_hip_link_LH_X_fr_base fr_hip_link_LH_X_fr_base;
    Type_fr_base_X_fr_hip_link_LH fr_base_X_fr_hip_link_LH;
    Type_fr_thigh_link_LH_X_fr_hip_link_LH fr_thigh_link_LH_X_fr_hip_link_LH;
    Type_fr_hip_link_LH_X_fr_thigh_link_LH fr_hip_link_LH_X_fr_thigh_link_LH;
    Type_fr_shank_link_LH_X_fr_thigh_link_LH fr_shank_link_LH_X_fr_thigh_link_LH;
    Type_fr_thigh_link_LH_X_fr_shank_link_LH fr_thigh_link_LH_X_fr_shank_link_LH;
    Type_fr_hip_link_RH_X_fr_base fr_hip_link_RH_X_fr_base;
    Type_fr_base_X_fr_hip_link_RH fr_base_X_fr_hip_link_RH;
    Type_fr_thigh_link_RH_X_fr_hip_link_RH fr_thigh_link_RH_X_fr_hip_link_RH;
    Type_fr_hip_link_RH_X_fr_thigh_link_RH fr_hip_link_RH_X_fr_thigh_link_RH;
    Type_fr_shank_link_RH_X_fr_thigh_link_RH fr_shank_link_RH_X_fr_thigh_link_RH;
    Type_fr_thigh_link_RH_X_fr_shank_link_RH fr_thigh_link_RH_X_fr_shank_link_RH;

protected:
    Parameters params;

}; //class 'HomogeneousTransforms'

}
}

#endif
