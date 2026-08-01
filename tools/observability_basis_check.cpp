#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <iomanip>
#include <iostream>

using Mat3 = Eigen::Matrix3d;
using Vec3 = Eigen::Vector3d;

static Mat3 hat(const Vec3 &v) {
    Mat3 result;
    result << 0.0, -v.z(), v.y(),
              v.z(), 0.0, -v.x(),
             -v.y(), v.x(), 0.0;
    return result;
}

int main() {
    constexpr int views = 6;
    std::array<Mat3, views> rotations;
    std::array<Vec3, views> positions;
    for (int i = 0; i < views; ++i) {
        const double angle = 0.25 * i;
        positions[i] = Vec3(2.0 * std::cos(angle), 2.0 * std::sin(angle), 0.15 * i);
        rotations[i] = Eigen::AngleAxisd(0.12 * i, Vec3(0.2, 0.1, 1.0).normalized())
                           .toRotationMatrix();
    }

    const Vec3 landmark(0.7, -0.4, 6.0);
    Eigen::MatrixXd Jp = Eigen::MatrixXd::Zero(2 * views, 6 * views);
    Eigen::MatrixXd Jl(2 * views, 3);
    for (int i = 0; i < views; ++i) {
        const Vec3 d = landmark - positions[i];
        const Vec3 c = rotations[i].transpose() * d;
        Eigen::Matrix<double, 2, 3> J;
        J << 1.0 / c.z(), 0.0, -c.x() / (c.z() * c.z()),
             0.0, 1.0 / c.z(), -c.y() / (c.z() * c.z());
        const Eigen::Matrix<double, 2, 3> J_lmk = J * rotations[i].transpose();
        Eigen::Matrix<double, 2, 6> J_pose;
        J_pose << J_lmk * hat(d), -J_lmk;
        Jp.block<2, 6>(2 * i, 6 * i) = J_pose;
        Jl.block<2, 3>(2 * i, 0) = J_lmk;
    }

    const Eigen::MatrixXd Hpp = Jp.transpose() * Jp;
    const Eigen::MatrixXd Hpl = Jp.transpose() * Jl;
    const Mat3 Hll = Jl.transpose() * Jl;
    const Eigen::MatrixXd Hs = Hpp - Hpl * Hll.completeOrthogonalDecomposition().pseudoInverse()
                                      * Hpl.transpose();
    const Vec3 gravity_axis = Vec3::UnitZ();
    const Vec3 anchor = positions.front();

    std::cout << std::scientific << std::setprecision(3);
    bool passed = true;
    for (const int q_sign : {1, -1}) {
        for (const int p_sign : {1, -1}) {
            Eigen::VectorXd n = Eigen::VectorXd::Zero(6 * views);
            for (int i = 0; i < views; ++i) {
                n.segment<3>(6 * i) = q_sign * gravity_axis;
                n.segment<3>(6 * i + 3) =
                    p_sign * hat(positions[i] - anchor) * gravity_axis;
            }
            const Vec3 landmark_gauge =
                -q_sign * hat(landmark - anchor) * gravity_axis;
            const double schur_leak = (Hs * n).norm() / (Hs.norm() * n.norm());
            const double joint_leak = (Jp * n + Jl * landmark_gauge).norm();
            const bool expected_null = p_sign == -q_sign;
            const bool is_null = schur_leak < 1e-12 && joint_leak < 1e-12;
            passed = passed && (is_null == expected_null);
            std::cout << "q_sign=" << q_sign << ", p_hat_sign=" << p_sign
                      << ", expected_null=" << expected_null
                      << ", schur_leak=" << schur_leak
                      << ", joint_JN=" << joint_leak << '\n';
        }
    }

    Eigen::MatrixXd translation = Eigen::MatrixXd::Zero(6 * views, 3);
    for (int i = 0; i < views; ++i) {
        translation.block<3, 3>(6 * i + 3, 0).setIdentity();
    }
    const double translation_leak =
        (Hs * translation).norm() / (Hs.norm() * translation.norm());
    passed = passed && translation_leak < 1e-12;
    std::cout << "translation_schur_leak=" << translation_leak << '\n';
    std::cout << "observability_basis_check=" << (passed ? "PASS" : "FAIL") << '\n';
    return passed ? 0 : 1;
}
