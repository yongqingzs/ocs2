## 使用
四足机器人启动
```bash
# Build the example
catkin build ocs2_legged_robot_ros
# Source workspace
# Do not forget to change <...> parts
source devel/setup.bash

# Launch the example for DDP
roslaunch ocs2_legged_robot_ros legged_robot_ddp.launch
# OR launch the example for SQP
roslaunch ocs2_legged_robot_ros legged_robot_sqp.launch
```