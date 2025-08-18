# ocs2_legged_robot_ros

## 概述

`ocs2_legged_robot_ros` 包是 `ocs2_legged_robot` 的 ROS 接口层。它负责将 `ocs2_legged_robot` 包中实现的 MPC 控制器与 ROS 环境集成，提供了节点、可视化、以及通过 ROS 话题和服务进行交互的能力。

该包主要包含以下功能：
- **MPC 节点**: 提供了三种不同求解器（DDP, SQP, IPM）的 MPC 节点。
- **仿真节点**: 一个用于模拟机器人行为的虚拟节点。
- **指令节点**: 用于通过键盘发送目标位姿和步态指令。
- **可视化**: 在 Rviz 中可视化机器人的状态、期望轨迹、足端力等。

## 节点关系

`ocs2_legged_robot_ros` 包中的节点通过 ROS 话题和服务进行交互，构成一个完整的控制系统。

```mermaid
graph TD
    subgraph UserInput["用户输入"]
        A[legged_robot_target]
        B[legged_robot_gait_command]
    end

    subgraph ControlSystem["控制系统"]
        C["MPC Node (DDP/SQP/IPM)"]
        D[legged_robot_dummy]
    end

    subgraph Visualization["可视化"]
        E[Rviz]
    end

    A -- "/legged_robot_target" --> C
    B -- "/legged_robot_mpc_mode_schedule" --> C
    C -- "/legged_robot_mpc_policy" --> D
    D -- "/legged_robot_mpc_observation" --> C
    D -- TF & Markers --> E

```

**节点关系文字描述:**

1.  **指令节点**:
    *   `legged_robot_target`: 一个键盘输入节点，用于发布目标轨迹（`ocs2_msgs/mpc_target_trajectories`）到 `/legged_robot_target` 话题。
    *   `legged_robot_gait_command`: 一个键盘输入节点，用于发布步态指令（`ocs2_msgs/mode_schedule`）到 `/legged_robot_mpc_mode_schedule` 话题。

2.  **控制系统**:
    *   **MPC 节点** (`legged_robot_ddp_mpc`, `legged_robot_sqp_mpc`, `legged_robot_ipm_mpc`):
        *   订阅 `/legged_robot_mpc_observation` 话题，获取当前机器人状态。
        *   订阅 `/legged_robot_target` 话题，获取目标轨迹。
        *   订阅 `/legged_robot_mpc_mode_schedule` 话题，获取步态指令。
        *   运行 MPC 优化，计算出最优的控制策略。
        *   发布最优控制策略（`ocs2_msgs/mpc_flattened_controller`）到 `/legged_robot_mpc_policy` 话题。
    *   **`legged_robot_dummy`**:
        *   一个仿真节点，模拟机器人的行为。
        *   订阅 `/legged_robot_mpc_policy` 话题，获取并执行 MPC 计算出的控制策略。
        *   发布机器人的当前状态（`ocs2_msgs/mpc_observation`）到 `/legged_robot_mpc_observation` 话题。
        *   通过 `LeggedRobotVisualizer` 发布可视化信息到 Rviz。

3.  **可视化**:
    *   **Rviz**: 用于可视化机器人模型、期望轨迹、足端力、支撑多边形等。数据由 `legged_robot_dummy` 节点中的 `LeggedRobotVisualizer` 提供。

## 主要方法功能说明

### MPC 节点 (例如 `LeggedRobotSqpMpcNode.cpp`)

这些节点是 MPC 控制器的 ROS 封装。它们的功能类似，只是使用了不同的求解器。

- **`main()`**:
  - 初始化 ROS 节点。
  - 从参数服务器获取配置文件路径（`taskFile`, `urdfFile`, `referenceFile`）。
  - 创建 `LeggedRobotInterface` 对象，该对象包含了最优控制问题的完整定义。
  - 创建 `GaitReceiver`，用于接收和处理来自 `/legged_robot_mpc_mode_schedule` 的步态指令。
  - 创建 `RosReferenceManager`，用于接收和处理来自 `/legged_robot_target` 的目标轨迹。
  - 创建 MPC 求解器实例（例如 `SqpMpc`）。
  - 将 `GaitReceiver` 和 `RosReferenceManager` 添加到求解器的同步模块中。
  - 创建 `MPC_ROS_Interface` 对象，它负责 MPC 与 ROS 之间的通信。
  - 启动 `MPC_ROS_Interface`。

### `legged_robot_dummy`

该节点模拟机器人的动力学，并与 MPC 节点形成闭环。

- **`main()`**:
  - 初始化 ROS 节点。
  - 创建 `LeggedRobotInterface` 对象。
  - 创建 `MRT_ROS_Interface`，它负责与 MPC 节点进行通信（订阅策略，发布观测值）。
  - 创建 `LeggedRobotVisualizer`，用于发布可视化信息。
  - 创建 `MRT_ROS_Dummy_Loop`，这是仿真的核心，它以指定的频率运行，执行 MPC 策略，并发布新的观测值。
  - 设置初始状态和初始指令。
  - 启动仿真循环。

### 指令节点

- **`LeggedRobotPoseCommandNode.cpp`**:
  - 创建 `TargetTrajectoriesKeyboardPublisher` 对象。
  - 提供一个命令行界面，让用户输入期望的基座平移和旋转，然后将其转换为 `TargetTrajectories` 并发布。
- **`LeggedRobotGaitCommandNode.cpp`**:
  - 创建 `GaitKeyboardPublisher` 对象。
  - 从配置文件加载可用的步态列表。
  - 提供一个命令行界面，让用户选择并发布期望的步态。

### Gait

- **`GaitReceiver`**:
  - 继承自 `SolverSynchronizedModule`，可以在 MPC 求解器运行前后执行特定操作。
  - **`mpcModeSequenceCallback(...)`**: 订阅 `/legged_robot_mpc_mode_schedule` 话题，当接收到新的步态指令时，将其存储起来。
  - **`preSolverRun(...)`**: 在 MPC 求解器每次运行前被调用。如果接收到了新的步态，它会调用 `gaitSchedulePtr_->insertModeSequenceTemplate(...)` 来更新步态调度。
- **`GaitKeyboardPublisher`**:
  - **`getKeyboardCommand()`**: 等待用户通过键盘输入步态指令，然后将其发布为 `ocs2_msgs::mode_schedule` 消息。

### Visualization

- **`LeggedRobotVisualizer`**:
  - **`update(...)`**: 在每次接收到新的观测值和控制策略时被调用，用于更新和发布可视化信息。
  - **`publishObservation(...)`**: 发布机器人的当前状态，包括关节位置、基座姿态、足端位置和力。
  - **`publishDesiredTrajectory(...)`**: 发布期望的基座和足端轨迹。
  - **`publishOptimizedStateTrajectory(...)`**: 发布 MPC 优化出的状态轨迹。
  - **`publishJointTransforms(...)` / `publishBaseTransform(...)`**: 发布 TF 变换，用于在 Rviz 中显示机器人模型。
  - **`publishCartesianMarkers(...)`**: 发布足端位置、接触力、压力中心和支撑多边形等 Marker 信息。

## 总结

`ocs2_legged_robot_ros` 包成功地将 `ocs2_legged_robot` 的核心控制算法与 ROS 生态系统集成在一起。它通过一系列定义清晰的 ROS 节点和话题，实现了用户指令输入、MPC 控制、仿真和可视化之间的解耦和通信。这种模块化的设计使得系统的各个部分可以独立开发和测试，同时也为用户提供了一个易于使用和扩展的框架。
