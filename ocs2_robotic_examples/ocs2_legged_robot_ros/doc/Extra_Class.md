# GaitSchedule 类功能解析

`GaitSchedule` 是一个用于管理机器人步态模式和切换时间的核心类。以下是该类的主要功能和各个函数的详细解析：

---

## **1. 构造函数**

### **1.1 构造函数**
```cpp
GaitSchedule::GaitSchedule(ModeSchedule initModeSchedule, ModeSequenceTemplate initModeSequenceTemplate, scalar_t phaseTransitionStanceTime)
```
- **功能**:
  - 初始化步态调度对象。
  - 设置初始模式调度（`initModeSchedule`）、模式序列模板（`initModeSequenceTemplate`）和相位转换站立时间（`phaseTransitionStanceTime`）。

---

## **2. 模式序列管理**

### **2.1 insertModeSequenceTemplate**
```cpp
void GaitSchedule::insertModeSequenceTemplate(const ModeSequenceTemplate& modeSequenceTemplate, scalar_t startTime, scalar_t finalTime)
```
- **功能**:
  - 插入新的模式序列模板。
  - 根据起始时间（`startTime`）和结束时间（`finalTime`）更新模式调度。
  - 添加中间站立相位并平铺模式序列模板。

### **2.2 tileModeSequenceTemplate**
```cpp
void GaitSchedule::tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime)
```
- **功能**:
  - 平铺模式序列模板。
  - 从指定的起始时间（`startTime`）到结束时间（`finalTime`）重复模式序列。

---

## **3. 模式调度获取**

### **3.1 getModeSchedule**
```cpp
ModeSchedule GaitSchedule::getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime)
```
- **功能**:
  - 获取指定时间范围内的模式调度。
  - 删除旧的逻辑并设置默认初始相位为站立（`STANCE`）。
  - 平铺模式序列模板以覆盖指定的时间范围。

---

## **总结**
`GaitSchedule` 类的主要功能包括：
1. **步态模式管理**:
   - 管理步态模式的切换时间和序列。
   - 支持插入和平铺模式序列模板。
2. **时间范围支持**:
   - 提供获取指定时间范围内模式调度的功能。
3. **灵活性**:
   - 支持动态更新模式序列和相位转换时间。

该类是机器人步态调度的核心组件，确保步态模式能够根据时间和任务需求动态调整。