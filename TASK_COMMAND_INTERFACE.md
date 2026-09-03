# 空间蛇形机械臂任务指令控制接口规范 (Task Command Interface Specification)

> **版本**：v2.0  
> **适用模块**：UI 任务调度端 (`SpaceSnakeVisionUI`) $\longleftrightarrow$ 机械臂运动控制端 (`Motion Control / MockControlServer`)  
> **文件通信协议**：  
> - **命令下发路径**：`data/outbox/CMD-YYYYMMDD-NNNNN.json`  
> - **状态反馈路径**：`data/inbox/{command_id}_status.json`  
> - **实时视觉流路径**：`data/vision/latest_targets.json`

---

## 一、通用协议与外层数据结构

每次操作员在 UI 点击“生成任务”并“发布命令”后，UI 会在 `data/outbox/` 生成一个统一的 JSON 任务文件。其顶层公共字段结构如下：

```json
{
  "schema_version": "2.0",
  "command_id": "CMD-20260903-00001",
  "timestamp": 1788410000.123,
  "source": "SpaceSnakeVisionUI",
  "command_type": "<指令类型字符串>",
  "params": {
    /* 针对该指令的具体控制参数字典 */
  },
  "selected_target": {
    /* 当前锁定的视觉目标快照，若无则为 null */
  },
  "destination": {
    /* 目标工位信息，若无则为 null */
  },
  "motion_params": {
    /* 向后兼容字典，内容与 params 相同 */
  },
  "safety": {
    "require_user_confirm": true,
    "allow_execute": true,
    "allow_real_execute": true,
    "execution_mode": "real_robot",
    "validation_passed": true,
    "validation_message": "validation passed",
    "checks": {},
    "estop_active": false,
    "robot_busy": false
  }
}
```

### 公共字段说明：
1. **`schema_version`** (`string`)：固定为 `"2.0"`。
2. **`command_id`** (`string`)：唯一任务编号，如 `"CMD-20260903-00001"`。控制模块反馈执行状态时，**必须以该 ID 命名状态文件**（`data/inbox/{command_id}_status.json`）。
3. **`timestamp`** (`float`)：命令生成的时间戳（Unix 时间戳，单位：秒）。
4. **`source`** (`string`)：发布源，固定为 `"SpaceSnakeVisionUI"`。
5. **`command_type`** (`string`)：任务类型名称（详见下文 12 种指令）。
6. **`params`** (`object`)：核心参数字典，存放该指令的具体数值。
7. **`safety`** (`object`)：安全校验结论：
   - `allow_execute` (`bool`): 是否允许控制端执行。
   - `allow_real_execute` (`bool`): 是否允许真实物理机执行（若未标定或缺乏 6DoF，通常为 `false`）。
   - `execution_mode` (`string`): `"real_robot"`（真机）或 `"simulation_only"`（仅仿真/空跑）。
   - `estop_active` (`bool`): 当前 UI 是否处于急停锁定态。

---

## 二、每种任务指令具体信息与参数规范

### 1. `move_to`：末端移动到指定坐标 (x, y)

- **指令功能**：驱动机械臂末端执行器平移移动到基座坐标系或俯视工作平面的目标坐标 $(x, y)$。
- **输入交互**：可在 UI 界面微调框输入，或**在下方 MISSION MAP 俯视地图上单击任意位置快速拾取坐标**。
- **`params` 结构**：
  ```json
  "params": {
    "x": 0.350,
    "y": -0.120
  }
  ```
- **字段规范与物理量**：
  - `x` (`float`)：目标横向位置，单位：**米 (m)**，范围 $[-1.5, +1.5]$。基座坐标系中指向机器人右侧为正。
  - `y` (`float`)：目标纵向（前进）位置，单位：**米 (m)**，范围 $[-1.5, +1.5]$。基座坐标系中向前为正。
- **`selected_target`**：通常为 `null`（若当前界面锁定了目标，会同时附带该目标快照作为参考）。
- **`destination`**：`null`。

---

### 2. `move_along`：末端向 $\theta$ 方向移动 $d$ 米

- **指令功能**：从当前末端位置出发，在水平面内沿方位角 $\theta$ 方向直线延伸/位移 $d$ 米。
- **`params` 结构**：
  ```json
  "params": {
    "theta_deg": 45.0,
    "distance_m": 0.20
  }
  ```
- **字段规范与物理量**：
  - `theta_deg` (`float`)：移动方向方位角，单位：**度 (°)**，范围 $[-180.0, +180.0]$。$0^\circ$ 表示正前方，$+90^\circ$ 表示右侧，$-90^\circ$ 表示左侧。
  - `distance_m` (`float`)：单次位移距离，单位：**米 (m)**，范围 $(0, 1.5]$。
- **`selected_target`**：`null`。
- **`destination`**：`null`。

---

### 3. `move_for_pick`：根据实时相对位置向物体移动

- **指令功能**：视觉伺服（Visual Servoing）靠近动作。驱动末端从当前姿态向视觉锁定的靶标物体自主靠近，直至到达抓取预备姿态（Pre-grasp Pose）。
- **前置依赖**：必须在 UI 画面或目标列表中成功锁定一个目标（`selected_id` 不为空）。
- **`params` 结构**：
  ```json
  "params": {}
  ```
- **`selected_target`（携带目标完整位姿与质量快照）**：
  ```json
  "selected_target": {
    "target_id": "target_1",
    "class_name": "201",
    "display_name": "Marker 201",
    "detection_mode": "yolo_marker",
    "confidence": 0.95,
    "status": "POSE_6DOF",
    "depth_m": 0.285,
    "bbox_xyxy": [120, 80, 260, 220],
    "center_pixel": [190, 150],
    "bearing": {
      "pixel_center": [190, 150],
      "pixel_error": [20, -10],
      "ray_camera": [0.08, -0.04, 0.99]
    },
    "pose_camera": {
      "frame_id": "camera_left",
      "position": {"x": 0.025, "y": -0.012, "z": 0.285},
      "orientation": {"qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
      "orientation_euler": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    },
    "pose_base": null,
    "quality": {
      "num_dots": 8,
      "valid_depth_points": 8,
      "match_error_m": 0.0012
    }
  }
  ```
- **控制端执行规范**：
  1. 读取本指令获取初始锁定目标 ID (`target_id`)；
  2. 在移动过程中，控制模块需持续以高频（如 30Hz）读取 `data/vision/latest_targets.json` 获得实时闭环位姿；
  3. 当 `status == "BEARING_ONLY"` 时沿视线向量 `bearing.ray_camera` 靠近；
  4. 当 `status == "POSE_6DOF"` 时，利用 6 自由度位姿精确调整末端法向对齐。

---

### 4. `move_for_place`：根据硬编码向放置位置移动

- **指令功能**：装配放置寻位。驱动机械臂从当前抓持状态移动到预设的硬编码装配工位（如接口 A、接口 B）。
- **`params` 结构**：
  ```json
  "params": {
    "destination": "Assembly_Port_A"
  }
  ```
- **`destination` 字段结构**：
  ```json
  "destination": {
    "name": "Assembly_Port_A",
    "pose_base": {
      "frame_id": "robot_base",
      "position": {"x": 0.35, "y": 0.10, "z": 0.20},
      "orientation": {"qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
    }
  }
  ```
- **字段规范与物理量**：
  - `destination.name` (`string`)：工位标识，可选 `"Assembly_Port_A"`、`"Assembly_Port_B"`、`"Holding_Zone"`、`"Safe_Zone"`。
  - `destination.pose_base` (`object`)：预置的基座坐标系空间位姿（米与四元数）。

---

### 5. `rotate`：末端固定位置旋转 $\alpha$ 角

- **指令功能**：末端执行器固定当前三维位置不动，仅绕工具末端中心轴（Tool Roll 轴）自转 $\alpha$ 角度。
- **`params` 结构**：
  ```json
  "params": {
    "alpha_deg": 90.0
  }
  ```
- **字段规范与物理量**：
  - `alpha_deg` (`float`)：相对旋转角度，单位：**度 (°)**，范围 $[-180.0, +180.0]$。正值为顺时针，负值为逆时针。

---

### 6. `rotate_arm`：第 $n$ 关节旋转 $\alpha$ 度

- **指令功能**：单关节独立控制。针对蛇形臂的第 $n$ 级旋转关节，使其在当前关节角基础上增量旋转 $\alpha$ 度。
- **`params` 结构**：
  ```json
  "params": {
    "joint_index": 3,
    "alpha_deg": -30.0
  }
  ```
- **字段规范与物理量**：
  - `joint_index` (`int`)：目标关节序号（1 索引），范围 $[1, 16]$。1 表示最靠近底座的第 1 关节。
  - `alpha_deg` (`float`)：增量旋转角度，单位：**度 (°)**，范围 $[-180.0, +180.0]$。

---

### 7. `facing_arm`：第 $n$ 关节面向 $\theta$ 方向

- **指令功能**：关节绝对朝向控制。调整从底座至第 $n$ 关节的子运动链，使第 $n$ 关节法线或弯曲平面直接指向物理空间的绝对方位角 $\theta$。
- **`params` 结构**：
  ```json
  "params": {
    "joint_index": 2,
    "theta_deg": 60.0
  }
  ```
- **字段规范与物理量**：
  - `joint_index` (`int`)：目标关节序号，范围 $[1, 16]$。
  - `theta_deg` (`float`)：绝对朝向角，单位：**度 (°)**，基座坐标系下范围 $[-180.0, +180.0]$。

---

### 8. `pick`：夹爪抓取动作链

- **指令功能**：触发末端夹爪的标准闭合抓取与装配卡扣锁紧流程（动作链包括：使能预紧力矩 $\to$ 夹爪闭合 $\to$ 接触传感器力控反馈确认 $\to$ 锁定确认）。
- **`params` 结构**：
  ```json
  "params": {}
  ```
- **字段规范**：无额外参数，控制端执行内置的标准化 Gripper Grasp 动作链。

---

### 9. `place`：夹爪放置动作链

- **指令功能**：触发末端夹爪的标准释放与装配到位脱钩流程（动作链包括：解锁卡扣 $\to$ 夹爪张开至全开位置 $\to$ 传感器确认脱离）。
- **`params` 结构**：
  ```json
  "params": {}
  ```
- **字段规范**：无额外参数，控制端执行内置的标准化 Gripper Release 动作链。

---

### 10. `withdraw`：退回上一状态

- **指令功能**：轨迹回退/回撤。沿着此前执行轨迹反向回退设定距离（如反向倒退 10cm）或恢复到上一个稳态断点，通常用于插入/对接失败后的安全退出。
- **`params` 结构**：
  ```json
  "params": {}
  ```

---

### 11. `reset`：恢复初始位置

- **指令功能**：系统零位复位。驱动蛇形臂所有关节（1~16 关节）依次回归机械零点（Home Position / 收拢姿态），夹爪复位，清除内部临时误差。
- **`params` 结构**：
  ```json
  "params": {}
  ```

---

### 12. `emergency_stop`：最高优先级急停

- **指令功能**：安全急停。立即切断伺服力矩或进入强力抱闸停机状态，中断所有正在执行的规划任务。
- **`params` 结构**：
  ```json
  "params": {}
  ```
- **`safety` 特殊标识**：
  ```json
  "safety": {
    "require_user_confirm": false,
    "allow_execute": true,
    "allow_real_execute": true,
    "execution_mode": "real_robot",
    "estop_active": true
  }
  ```

---

### 13. `cancel_task`：取消当前任务

- **指令功能**：非紧急性任务终止。机械臂以平滑减速度在当前轨迹上刹车停止，保持就地等待姿态，不清除关节使能。
- **`params` 结构**：
  ```json
  "params": {}
  ```

---

## 三、控制模块状态反馈规范 (TaskStatus)

当控制模块在 `data/outbox/` 读取到任一任务文件后，**必须在 `data/inbox/{command_id}_status.json` 中写回状态信息**，UI 界面将每 600ms 轮询一次并更新界面任务列表和状态指示灯。

### 反馈 JSON 格式：
```json
{
  "schema_version": "1.0",
  "command_id": "CMD-20260903-00001",
  "timestamp": 1788410002.500,
  "status": "EXECUTING",
  "current_step": "EXECUTING_MOVE_TO",
  "progress": 0.65,
  "message": "Moving end-effector to (0.35, -0.12)",
  "robot_state": {
    "state": "EXECUTING",
    "joint_positions": [0.0, 12.5, -5.2, 0.0],
    "message": "normal"
  }
}
```

### 状态枚举值（`status`）：
- **`RECEIVED`**：控制模块已成功读取文件。
- **`ACCEPTED`**：指令参数及碰撞自检通过，已加入执行队列。
- **`PLANNING`**：正在进行运动学逆解（IK）与避障轨迹规划。
- **`EXECUTING`**：控制器正在驱动关节电机运动，`progress` 字段需从 `0.0` 递增至 `1.0`。
- **`COMPLETED`**：任务圆满完成，末端到达预定容差范围内。
- **`REJECTED`**：安全校验未通过或逆解无解，任务拒绝。
- **`CANCELED`**：收到 `cancel_task` 后已平稳刹车停机。
- **`ESTOP_TRIGGERED`**：进入急停停机状态。
- **`FAILED`**：执行过程中遇到异常（如关节超温、力矩超限或视觉目标丢失超时）。
