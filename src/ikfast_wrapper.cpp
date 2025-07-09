#include "ikfast_wrapper.h"
#include "ikfastdemo.cpp"  // 注意：此处真正 include .cpp

bool ComputeIK(const std::vector<double>& position,
               const std::vector<double>& rotation,
               const std::vector<double>& free_joint_values,
               std::vector<std::vector<double>>& solutions_out) {
    IkSolutionList<IkReal> solutions;
    bool success = ComputeIk(position.data(), rotation.data(), free_joint_values.data(), solutions);
    if (!success) return false;

    std::vector<IkReal> sol(GetNumJoints());
    for (size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        const auto& s = solutions.GetSolution(i);
        s.GetSolution(sol.data(), nullptr);
        std::vector<double> sol_d(sol.begin(), sol.end());
        solutions_out.push_back(sol_d);
    }
    return true;
}
