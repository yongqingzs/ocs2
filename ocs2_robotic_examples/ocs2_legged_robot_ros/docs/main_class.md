# 模块类功能跳转目录

- [MPC_ROS_Interface 类功能解析](#mpc_ros_interface-类功能解析)
- [MRT_ROS_Interface 类功能解析](#mrt_ros_interface-类功能解析)
- [LeggedRobotInterface 类功能解析](#leggedrobotinterface-类功能解析)

---

# MPC_ROS_Interface 类功能解析

`MPC_ROS_Interface` 是一个基于 ROS 的接口类，用于与 **MPC（Model Predictive Control）** 模块交互。以下是该类的主要功能和各个函数的详细解析：

---

## **1. 构造函数和析构函数**

### **1.1 构造函数**
```cpp
MPC_ROS_Interface::MPC_ROS_Interface(MPC_BASE& mpc, std::string topicPrefix)
```
- **功能**:
  - 初始化 `MPC_ROS_Interface` 对象。
  - 设置 MPC 实例（`mpc_`）和 ROS 主题前缀（`topicPrefix_`）。
  - 初始化缓冲区和多线程发布机制。
  - 如果定义了 `PUBLISH_THREAD`，启动一个独立线程用于发布策略。

### **1.2 析构函数**
```cpp
MPC_ROS_Interface::~MPC_ROS_Interface()
```
- **功能**:
  - 调用 `shutdownNode()` 关闭 ROS 节点和发布线程，释放资源。

---

## **2. MPC 重置相关**

### **2.1 resetMpcNode**
```cpp
void MPC_ROS_Interface::resetMpcNode(TargetTrajectories&& initTargetTrajectories)
```
- **功能**:
  - 重置 MPC 节点。
  - 设置新的目标轨迹（`initTargetTrajectories`）。
  - 重置计时器和发布状态。

### **2.2 resetMpcCallback**
```cpp
bool MPC_ROS_Interface::resetMpcCallback(ocs2_msgs::reset::Request& req, ocs2_msgs::reset::Response& res)
```
- **功能**:
  - 处理 ROS 服务请求，重置 MPC 节点。
  - 从请求消息中读取目标轨迹并调用 `resetMpcNode`。
  - 返回重置结果。

---

## **3. MPC 策略发布相关**

### **3.1 createMpcPolicyMsg**
```cpp
ocs2_msgs::mpc_flattened_controller MPC_ROS_Interface::createMpcPolicyMsg(const PrimalSolution& primalSolution,
                                                                          const CommandData& commandData,
                                                                          const PerformanceIndex& performanceIndices)
```
- **功能**:
  - 根据 MPC 的原始解（`primalSolution`）、命令数据（`commandData`）和性能指标（`performanceIndices`），生成 ROS 消息 `mpc_flattened_controller`。
  - 包括时间轨迹、状态轨迹、输入轨迹和控制器数据的序列化。

### **3.2 publisherWorker**
```cpp
void MPC_ROS_Interface::publisherWorker()
```
- **功能**:
  - 在独立线程中发布 MPC 策略消息。
  - 等待策略准备好后，将其发布到 ROS 主题。

### **3.3 copyToBuffer**
```cpp
void MPC_ROS_Interface::copyToBuffer(const SystemObservation& mpcInitObservation)
```
- **功能**:
  - 从 MPC 获取最新的原始解、命令数据和性能指标，并存储到缓冲区中。
  - 用于后续的策略发布。

---

## **4. MPC 输入相关**

### **4.1 mpcObservationCallback**
```cpp
void MPC_ROS_Interface::mpcObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg)
```
- **功能**:
  - 处理 ROS 订阅的观测值消息（`mpc_observation`）。
  - 调用 MPC 的 `run` 方法，计算新的控制策略。
  - 将结果存储到缓冲区，并触发策略发布。

---

## **5. ROS 节点管理**

### **5.1 shutdownNode**
```cpp
void MPC_ROS_Interface::shutdownNode()
```
- **功能**:
  - 关闭 ROS 发布者和订阅者。
  - 停止发布线程（如果启用）。

### **5.2 spin**
```cpp
void MPC_ROS_Interface::spin()
```
- **功能**:
  - 启动 ROS 回调循环，处理订阅的消息和服务请求。

### **5.3 launchNodes**
```cpp
void MPC_ROS_Interface::launchNodes(ros::NodeHandle& nodeHandle)
```
- **功能**:
  - 初始化 ROS 发布者、订阅者和服务。
  - 订阅观测值（`_mpc_observation`）。
  - 发布策略（`_mpc_policy`）。
  - 提供重置服务（`_mpc_reset`）。
  - 启动 ROS 回调循环。

---

## **总结**
`MPC_ROS_Interface` 类的主要功能包括：
1. **与 MPC 模块交互**:
   - 接收观测值，运行 MPC，生成控制策略。
   - 提供重置服务，设置新的目标轨迹。
2. **ROS 通信**:
   - 通过 ROS 发布控制策略。
   - 订阅观测值，处理服务请求。
3. **多线程支持**:
   - 独立线程发布策略，确保实时性。
4. **消息序列化**:
   - 将 MPC 的内部数据结构转换为 ROS 消息格式。

该类是 MPC 和 ROS 系统之间的桥梁，确保两者能够高效、实时地协同工作。

# MRT_ROS_Interface 类功能解析

`MRT_ROS_Interface` 是一个基于 ROS 的接口类，用于与 **MRT（Model Reference Tracking）** 模块交互。以下是该类的主要功能和各个函数的详细解析：

---

## **1. 构造函数和析构函数**

### **1.1 构造函数**
```cpp
MRT_ROS_Interface::MRT_ROS_Interface(std::string topicPrefix, ros::TransportHints mrtTransportHints)
```
- **功能**:
  - 初始化 `MRT_ROS_Interface` 对象。
  - 设置 ROS 主题前缀（`topicPrefix_`）和传输提示（`mrtTransportHints_`）。
  - 如果定义了 `PUBLISH_THREAD`，启动一个独立线程用于发布观测值。

### **1.2 析构函数**
```cpp
MRT_ROS_Interface::~MRT_ROS_Interface()
```
- **功能**:
  - 调用 `shutdownNodes()` 关闭 ROS 节点和发布线程，释放资源。

---

## **2. ROS 节点管理**

### **2.1 launchNodes**
```cpp
void MRT_ROS_Interface::launchNodes(ros::NodeHandle& nodeHandle)
```
- **功能**:
  - 初始化 ROS 发布者、订阅者和服务。
  - 发布观测值（`_mpc_observation`）。
  - 订阅策略（`_mpc_policy`）。
  - 提供重置服务（`_mpc_reset`）。
  - 启动 ROS 回调循环。

### **2.2 shutdownNodes**
```cpp
void MRT_ROS_Interface::shutdownNodes()
```
- **功能**:
  - 关闭 ROS 发布者和订阅者。
  - 停止发布线程（如果启用）。

### **2.3 spinMRT**
```cpp
void MRT_ROS_Interface::spinMRT()
```
- **功能**:
  - 启动 ROS 回调循环，处理订阅的消息和服务请求。

---

## **3. 发布线程管理**

### **3.1 publisherWorkerThread**
```cpp
void MRT_ROS_Interface::publisherWorkerThread()
```
- **功能**:
  - 在独立线程中发布观测值消息。
  - 等待消息准备好后，将其发布到 ROS 主题。

### **3.2 shutdownPublisher**
```cpp
void MRT_ROS_Interface::shutdownPublisher()
```
- **功能**:
  - 停止发布线程。
  - 通知所有等待的线程退出。

---

## **4. 数据处理**

### **4.1 setCurrentObservation**
```cpp
void MRT_ROS_Interface::setCurrentObservation(const SystemObservation& currentObservation)
```
- **功能**:
  - 接收当前观测值并生成 ROS 消息。
  - 如果启用了发布线程，将消息存储到缓冲区并触发发布。

### **4.2 readPolicyMsg**
```cpp
void MRT_ROS_Interface::readPolicyMsg(const ocs2_msgs::mpc_flattened_controller& msg, CommandData& commandData,
                                      PrimalSolution& primalSolution, PerformanceIndex& performanceIndices)
```
- **功能**:
  - 从 ROS 消息中读取策略数据，包括时间轨迹、状态轨迹、输入轨迹和控制器数据。
  - 根据控制器类型实例化相应的控制器。

### **4.3 mpcPolicyCallback**
```cpp
void MRT_ROS_Interface::mpcPolicyCallback(const ocs2_msgs::mpc_flattened_controller::ConstPtr& msg)
```
- **功能**:
  - 处理 ROS 订阅的策略消息。
  - 调用 `readPolicyMsg` 解析消息并将结果存储到缓冲区。

---

## **5. MPC 重置相关**

### **5.1 resetMpcNode**
```cpp
void MRT_ROS_Interface::resetMpcNode(const TargetTrajectories& initTargetTrajectories)
```
- **功能**:
  - 重置 MPC 节点。
  - 设置新的目标轨迹（`initTargetTrajectories`）。
  - 调用 ROS 服务完成重置。

---

## **总结**
`MRT_ROS_Interface` 类的主要功能包括：
1. **与 MRT 模块交互**:
   - 接收观测值，解析策略消息。
   - 提供重置服务，设置新的目标轨迹。
2. **ROS 通信**:
   - 通过 ROS 发布观测值。
   - 订阅策略消息，处理服务请求。
3. **多线程支持**:
   - 独立线程发布观测值，确保实时性。
4. **消息解析**:
   - 将 ROS 消息转换为内部数据结构，支持多种控制器类型。

该类是 MRT 和 ROS 系统之间的桥梁，确保两者能够高效、实时地协同工作。

# LeggedRobotInterface 类功能解析

`LeggedRobotInterface` 是一个核心类，用于管理腿式机器人模型的设置、优化问题的定义以及与控制器的交互。以下是该类的主要功能和各个函数的详细解析：

---

## **1. 构造函数**

### **1.1 构造函数**
```cpp
LeggedRobotInterface::LeggedRobotInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                           bool useHardFrictionConeConstraint)
```
- **功能**:
  - 初始化腿式机器人接口。
  - 加载任务文件（`taskFile`）、URDF 文件（`urdfFile`）和参考文件（`referenceFile`）。
  - 设置机器人模型、优化问题和初始状态。
  - 支持硬摩擦锥约束的配置。

---

## **2. 优化问题设置**

### **2.1 setupOptimalConrolProblem**
```cpp
void LeggedRobotInterface::setupOptimalConrolProblem(const std::string& taskFile, const std::string& urdfFile,
                                                     const std::string& referenceFile, bool verbose)
```
- **功能**:
  - 初始化 Pinocchio 接口，用于机器人动力学和运动学计算。
  - 创建质心模型信息（`CentroidalModelInfo`）。
  - 设置摆动轨迹规划器（`SwingTrajectoryPlanner`）。
  - 初始化模式调度管理器（`SwitchedModelReferenceManager`）。
  - 定义优化问题，包括动力学、成本函数和约束。
  - 配置预计算模块和滚动优化器。

---

## **3. 模式调度管理**

### **3.1 loadGaitSchedule**
```cpp
std::shared_ptr<GaitSchedule> LeggedRobotInterface::loadGaitSchedule(const std::string& file, bool verbose) const
```
- **功能**:
  - 从文件加载初始模式调度和默认模式序列模板。
  - 创建并返回步态调度对象（`GaitSchedule`）。

---

## **4. 成本函数和约束**

### **4.1 getBaseTrackingCost**
```cpp
std::unique_ptr<StateInputCost> LeggedRobotInterface::getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info,
                                                                          bool verbose)
```
- **功能**:
  - 定义基于状态和输入的二次跟踪成本函数。
  - 加载状态权重矩阵（`Q`）和输入权重矩阵（`R`）。

### **4.2 getFrictionConeConstraint**
```cpp
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getFrictionConeConstraint(size_t contactPointIndex,
                                                                                      scalar_t frictionCoefficient)
```
- **功能**:
  - 定义摩擦锥约束，用于限制接触点的力。

### **4.3 getZeroForceConstraint**
```cpp
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getZeroForceConstraint(size_t contactPointIndex)
```
- **功能**:
  - 定义零力约束，用于非接触点。

### **4.4 getZeroVelocityConstraint**
```cpp
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                      size_t contactPointIndex,
                                                                                      bool useAnalyticalGradients)
```
- **功能**:
  - 定义零速度约束，用于接触点的速度限制。

---

## **5. 初始化和预计算**

### **5.1 initializeInputCostWeight**
```cpp
matrix_t LeggedRobotInterface::initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info)
```
- **功能**:
  - 初始化输入成本权重矩阵。
  - 计算基于接触点雅可比矩阵的权重。

### **5.2 loadFrictionConeSettings**
```cpp
std::pair<scalar_t, RelaxedBarrierPenalty::Config> LeggedRobotInterface::loadFrictionConeSettings(const std::string& taskFile,
                                                                                                  bool verbose) const
```
- **功能**:
  - 加载摩擦锥设置，包括摩擦系数和松弛屏障惩罚配置。

---

## **总结**
`LeggedRobotInterface` 类的主要功能包括：
1. **机器人模型管理**:
   - 加载和初始化机器人模型。
   - 设置质心模型和摆动轨迹规划器。
2. **优化问题定义**:
   - 配置动力学、成本函数和约束。
   - 支持多种约束类型（如摩擦锥、零力、零速度）。
3. **模式调度管理**:
   - 加载和管理步态调度。
4. **初始化和预计算**:
   - 初始化状态和输入权重。
   - 配置预计算模块以提高计算效率。

该类是腿式机器人控制系统的核心接口，负责将机器人模型与优化问题无缝集成。