【运行环境】
操作系统：Ubuntu 18.04.5 LTS
Python 版本：Python 2.7.17
ROS 版本：Melodic
Gazebo 版本：9.0.0

【依赖库安装命令】
pip install numpy scipy matplotlib opencv-python cvxpy

【编译与运行指令】
# 1. 进入 catkin 工作空间
cd ~/catkin_ws

# 2. 编译功能包
catkin_make

# 3. 添加 Gazebo 模型路径
export GAZEBO_MODEL_PATH=/home/headless/catkin_ws/src/cylinder_robot/models:$GAZEBO_MODEL_PATH

# 4. 给所有 Python 脚本添加可执行权限
chmod +x src/cylinder_robot/script/*

# 5. 刷新 ROS 环境变量
source devel/setup.bash

# 6. 启动仿真
roslaunch cylinder_robot runCylinder.launch

【使用说明】
1. 启动Gazebo后，机器人自动执行对应轨迹控制
2. 运动结束终端按下 Ctrl+C
3. 程序自动调用绘图函数，生成轨迹对比图