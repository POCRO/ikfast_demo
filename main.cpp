#include <iostream>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
// #include <pinocchio/utils/eigen.hpp>

#include "ikfast_wrapper.h"

int main() {
    // 末端位姿
    std::vector<double> pos = {0.20925336, -0.00051973, 0.25531326};
    std::vector<double> rot = {
        1, 0, 0,
        0, -1, 0,
        0, 0, -1
    };
    std::vector<double> free = {0.0}; // 冗余关节
    std::vector<std::vector<double>> ik_solutions;

    if (!ComputeIK(pos, rot, free, ik_solutions)) {
        std::cerr << "IK Failed!" << std::endl;
        return 1;
    }

    // 载入 URDF 模型
    pinocchio::Model model;
    const std::string urdf_filename = "../eng_v2.urdf";
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);

    std::cout << "IK vs FK Check:\n";
    for (size_t i = 0; i < ik_solutions.size(); ++i) {
        const auto& q = ik_solutions[i];
        if (q.size() != model.nq) {
            std::cerr << "Solution size mismatch: got " << q.size()
                      << ", expected " << model.nq << std::endl;
            continue;
        }

        Eigen::VectorXd q_pin = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
        pinocchio::forwardKinematics(model, data, q_pin);
        pinocchio::updateFramePlacements(model, data);

        // 获取末端 link 的变换
        const std::string tip_name = "Link7";
        pinocchio::FrameIndex ee_id = model.getFrameId(tip_name, pinocchio::BODY);
        const pinocchio::SE3& oMi = data.oMf[ee_id];

        std::cout << "Solution " << i << ":\n";
        std::cout << "Joint values: ";
        for (double j : q) std::cout << j << " ";
        std::cout << "\nFK position: " << oMi.translation().transpose() << std::endl;
        std::cout << "FK rotation:\n" << oMi.rotation() << "\n\n";
    }

    return 0;
}
