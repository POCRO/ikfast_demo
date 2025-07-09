# 用pinocchio生成测试数据

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pathlib import Path
import numpy as np
from ament_index_python.packages import get_package_share_directory

# === 配置区 ===
pkgPath = get_package_share_directory('engineer_control_py')
descriptionPath = get_package_share_directory('engineer_description')
urdf_path = descriptionPath + '/urdf/demo.urdf'

URDF_PATH = urdf_path
BASE_LINK = "base_link"  
EE_LINK   = "Link7"      
JOINT_VALUES = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
ikfast_value = [-0.004312419202563, 0.877527805834321, -0.691364640585096, -0.004310065844754, -2.526657691284108, -2.187625151747947, 0.000000000000000]
ik1 = [-2.651191977149462, 0.865437378916566, -0.705341525070743, -2.651189615974760, -2.524701538556816, -2.187662072373914, 0.000000000000000]

# === 加载模型 ===
model = pin.buildModelFromUrdf(URDF_PATH)
data = model.createData()

# === 设置状态并计算 FK ===
q = np.array(ik1)
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

# 获取末端位姿
frame_id = model.getFrameId(EE_LINK)
T = data.oMf[frame_id]  # base_link 到末端 effector 的变换矩阵

print("末端位姿 T(base->ee):")
print("旋转矩阵：\n", T.rotation)
print("平移向量：", T.translation)

# 如果要喂给 ikfast，可以转换成单行输入：
R = T.rotation
t = T.translation
print("\nikfast 输入格式：")
print(f"{R[0,0]} {R[0,1]} {R[0,2]} {t[0]} "
      f"{R[1,0]} {R[1,1]} {R[1,2]} {t[1]} "
      f"{R[2,0]} {R[2,1]} {R[2,2]} {t[2]}")
