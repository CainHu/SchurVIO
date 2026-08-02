#include "landmark_parameterization.h"

#include <Eigen/LU>

namespace slam {
    LandmarkParameterizationLinearization
    linearizeLandmarkParameterization(
        Landmark &landmark,
        const Mat3_3 &Ric,
        const Vec3 &t_ic,
        const bool use_fej,
        const LandmarkParameterization mode) {
        LandmarkParameterizationLinearization result;
        if (!isAnchoredLandmarkParameterization(mode)) {
            return result;
        }

        Frame *anchor = landmark.anchor_obs && landmark.anchor_obs->fet
            ? landmark.anchor_obs->fet->frame : nullptr;
        if (!anchor) {
            result.valid = false;
            return result;
        }
        result.anchor = anchor;

        const Mat3_3 Rwi_anchor = use_fej
            ? anchor->q_fej().toRotationMatrix()
            : anchor->q().toRotationMatrix();
        Vec3 pwi_anchor = anchor->p();
        if (use_fej) {
            pwi_anchor = anchor->p_fej();
        }
        const Vec3 point_anchor_camera = Ric.transpose() *
            (Rwi_anchor.transpose() * (landmark.position - pwi_anchor) - t_ic);
        if (!point_anchor_camera.allFinite() ||
            point_anchor_camera.z() <= TYPE(0.05)) {
            result.valid = false;
            return result;
        }

        Mat3_3 camera_parameter_jacobian = Mat3_3::Identity();
        const TYPE x = point_anchor_camera.x();
        const TYPE y = point_anchor_camera.y();
        const TYPE z = point_anchor_camera.z();
        if (mode == LandmarkParameterization::AnchoredInverseDepth) {
            camera_parameter_jacobian <<
                z, TYPE(0), -x * z,
                TYPE(0), z, -y * z,
                TYPE(0), TYPE(0), -z * z;
        } else if (mode == LandmarkParameterization::AnchoredLogDepth) {
            camera_parameter_jacobian <<
                z, TYPE(0), x,
                TYPE(0), z, y,
                TYPE(0), TYPE(0), z;
        }
        result.parameter_to_world.noalias() =
            Rwi_anchor * Ric * camera_parameter_jacobian;
        return result;
    }

    Mat2_6 landmarkAnchorPoseJacobian(
        const Mat2_3 &J_landmark_world,
        const Landmark &landmark,
        const Frame &anchor,
        const bool use_fej) {
        const Vec3 anchor_position = use_fej
            ? anchor.p_fej() : anchor.p();
        const Vec3 anchor_offset = landmark.position - anchor_position;
        Mat2_6 jacobian = Mat2_6::Zero();
        jacobian.leftCols<3>().noalias() =
            -J_landmark_world * hat(anchor_offset);
        jacobian.rightCols<3>().noalias() = J_landmark_world;
        return jacobian;
    }

    bool transformLandmarkNormalToWorld(
        const Mat3_3 &parameter_to_world,
        Mat3_3 &Hll,
        Vec3 &gradient) {
        Eigen::FullPivLU<Mat3_3> transform_lu(parameter_to_world);
        if (!transform_lu.isInvertible()) {
            return false;
        }
        const Mat3_3 world_to_parameter = transform_lu.inverse();
        Hll = world_to_parameter.transpose() * Hll * world_to_parameter;
        gradient = world_to_parameter.transpose() * gradient;
        Hll = TYPE(0.5) * (Hll + Hll.transpose());
        return Hll.allFinite() && gradient.allFinite();
    }
}
