#pragma once
#include "ikfast.h"  // 如果有的话；否则可以省略
#include <vector>

bool ComputeIK(const std::vector<double>& position,
               const std::vector<double>& rotation,
               const std::vector<double>& free_joint_values,
               std::vector<std::vector<double>>& solutions);
