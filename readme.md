- [Ref links](#ref-links)
- [方法一：moveit官方脚本从urdf生成](#方法一moveit官方脚本从urdf生成)
- [方法二：使用小鱼镜像](#方法二使用小鱼镜像)
  - [1. 手动导出DAE和CPP](#1-手动导出dae和cpp)
  - [2. 快速测试](#2-快速测试)
  - [3. 封装使用](#3-封装使用)

# Ref links

[openrave doc](https://openrave.org/docs/latest_stable/openravepy/ikfast/#features)

[ikfast作者主页](http://programmingvision.com/)

[ikfast博论](http://www.programmingvision.com/rosen_diankov_thesis.pdf)

主要是chap4.1章节  Inverse Kinematics 
 
 
 
# 方法一：moveit官方脚本从urdf生成
[moveit2生成](https://www.ncnynl.com/ros2docs/cn/moveit2/doc/examples/ikfast/ikfast_tutorial.html)

```shell
ros2 run moveit_kinematics auto_create_ikfast_moveit_plugin.sh --iktype Transform6D $MYROBOT_NAME.urdf <planning_group_name> <base_link> <eef_link>
```
但是这个不能输入freeindex，手动改他的sh还不如我docker直接搞起来

该脚本和demo在moveit仓库里面，但很粗糙

[moveit脚本仓库](
https://github.com/moveit/moveit2/tree/950322e0018b86a64258ebc1241b5d0509331ef7/moveit_kinematics/ikfast_kinematics_plugin)

# 方法二：使用小鱼镜像
[鱼香ros教程](
https://fishros.org.cn/forum/topic/680/moveit-ikfast%E8%BF%90%E5%8A%A8%E5%AD%A6%E6%8F%92%E4%BB%B6%E9%85%8D%E7%BD%AE-%E6%9C%80%E8%AF%A6%E7%BB%86-%E6%B2%A1%E6%9C%89%E4%B9%8B%E4%B8%80)


## 1. 手动导出DAE和CPP



1. **urdf预先处理**

openrave可以适配的urdf版本较为古早，且只需处理纯几何关系，需要对urdf做简单预先处理


删掉ros control gazebo相关内容

删掉 visual collision inertial等跟stl 质量相关的标签

记得保留限位相关


2. **用小鱼的docker镜像导出dae和cpp文件**

生成dae文件
在urdf所在目录下运行，因为docker会把其启动目录自动挂载到/data下面

```shell

xhost + && sudo docker run  -it --rm  -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/snd -e DISPLAY=unix$DISPLAY  -v `pwd`:`pwd`  -w `pwd` fishros2/openrave  #拉镜像，第一次可能较慢
sudo apt update
sudo apt install python3-pip
sudo pip3 install elirobots transforms3d pytest rosdepc
sudo rosdepc init
sudo rosdepc update
rosdepc install --from-path src --ignore-src -y -r      # 检查依赖项
catkin_make
source devel/setup.bash
rosrun collada_urdf urdf_to_collada xxx.urdf yyy.dae        # 生成dae文件
rosrun moveit_kinematics round_collada_numbers.py yyy.dae yyy.dae 5     #修改dae文件精度，否则ikfast求解过程中可能因为浮点数问题出错

```

查看生成的dae文件信息
```shell
openrave-robot.py demo.dae --info links
```
得到结果为
```shell
root@5cfbc65c6216:/home/yy/arm_ws/src/engineer_description/urdf# openrave-robot.py demo9.dae --info links
name      index parents  
-------------------------
base_link 0              
Link1     1     base_link
Link2     2     Link1    
Link3     3     Link2    
Link4     4     Link3    
Link5     5     Link4    
Link6     6     Link5    
Link7     7     Link6    
-------------------------
name      index parents
```

根据这个info可以确定下列命令中的 baselink 和 eelink 

baselink选link序列的第一个就行

eelink选link序列最后一个  end effector link应该是

freeindex：ikfast也是只能求六自由度机械臂的完全解析解，七轴以上通过人为给定free parameter的方式使其退化

> 因为末端位姿只有六自由度，SE3群，刚好对应六个独立控制量;
> 当系统自由度>6时，系统是欠约束的，有无穷多个可行解

关于free parameter的选择：moveit的ikfast插件是咋弄的

```shell

python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=demo.dae --iktype=transform6d --baselink=0 --eelink=7 --savefile=$(pwd)/ikfastdemo.cpp --freeindex=6

```

如果一切顺利，就可以生成ikfast_demo.cpp了（等待时间十分钟左右）


##  2. <a name='cpp'></a>快速测试  

cpp不要任何改动

编译
```shell
g++ ikfastdemo.cpp -o ikbest -llapack -std=c++11
```
使用方法
```shell
yy@yy--ubuntu:~/arm_ws/src/engineer_description/urdf/ikfast_demo$ ./ikbest --help

Usage: ./ik r00 r01 r02 t0 r10 r11 r12 t1 r20 r21 r22 t2 free0 ...

Returns the ik solutions given the transformation of the end effector specified by
a 3x3 rotation R (rXX), and a 3x1 translation (tX).
There are 1 free parameters that have to be specified.
```

测试数据（用demo.py从关节角生成末端位姿，记得最后加上freeindex的值）

```shell
./ikbest 0.9999999999472127 2.3611691540582357e-06 1.0000024959561181e-05 0.20925693486730298 2.361087095414382e-06 -0.9999999999635453 8.20584302683952e-06 -0.0005214805172206309 1.0000044334626828e-05 -8.205819415389866e-06 -0.999999999916332 0.2553153552550598 0
```
解出八组解就对了
```shell
./ikbest 0.9999999999472127 2.3611691540582357e-06 1.0000024959561181e-05 0.20925693486730298 2.361087095414382e-06 -0.9999999999635453 8.20584302683952e-06 -0.0005214805172206309 1.0000044334626828e-05 -8.205819415389866e-06 -0.999999999916332 0.2553153552550598 0
Found 8 ik solutions:
sol0 (free=0): -0.004312419202562, 0.877527805834321, -0.691364640585095, -0.004310065844752, -2.526657691284108, -2.187625151747948, 0.000000000000000, 
sol1 (free=0): -2.651191977223232, 0.865437378916429, -0.705341525070878, -2.651189615975389, -2.524701538556814, -2.187662072373914, 0.000000000000000, 
sol2 (free=0): -0.000043524491898, 3.047829529556834, -1.467791047836373, 0.000041973260675, -2.013061129825681, 0.245506400250723, 0.000000000000000, 
sol3 (free=0): -3.136778962716202, -1.424111460654788, 0.148388718823495, 3.136776594556423, -2.146621277438951, 0.577538783611929, 0.000000000000000, 
sol4 (free=0): -0.000044212972782, 1.433465981638321, 0.056282062338797, -0.000042648668666, -1.150707218356980, 2.527901137870770, 0.000000000000000, 
sol5 (free=0): -3.135666525259149, 0.217178910988316, -1.352233275336395, -3.135664158411167, -1.279887181265567, 2.852077624250877, 0.000000000000000, 
sol6 (free=0): 0.000346328851010, -2.161563607202880, -0.614296216639936, -0.000348786561724, -0.210086754187744, -1.337170637761264, 0.000000000000000, 
sol7 (free=0): 3.095943313213332, -2.347845372201471, -0.777228874824301, -3.095945673645566, -0.229445049811836, -1.341520919038487, 0.000000000000000,

```

##  3. <a name=''></a>封装使用

下载ikfast.h，将生成的cpp最底下的main函数注释就可以用了

本仓库内使用wrapper对生成的cpp文件进行了如下封装

```cpp
bool ComputeIK(const std::vector<double>& position,
               const std::vector<double>& rotation,
               const std::vector<double>& free_joint_values,
               std::vector<std::vector<double>>& solutions);
```

需要在cpp工程中使用时候需修改：


cmake添加wrapper就行
```cmake
add_executable(ikfast_demo main.cpp src/ikfast_wrapper.cpp)
```

main.c里面
```cpp
#include "ikfast_wrapper.h"
```
main.c里面将ikfast解和pinocchio解进行对比，所以要用的话需要先安装pinocchio

[Installation Procedure](https://stack-of-tasks.github.io/pinocchio/download.html)

cmake前记得导入 pinocchio安装目录

```cpp
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```
然后调用就可以了

