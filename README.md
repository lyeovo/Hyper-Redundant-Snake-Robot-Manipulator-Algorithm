# PlanarArm — 平面高冗余蛇形机械臂 IK 求解与仿真平台（重构版）

## 项目定位

面向**地面（水平平面）夹取/放置**任务的超冗余蛇形机械臂运动学仿真与执行模块。作为机械臂系统的**中间模块**：

```
相机/视觉模块(SpaceSnakeVisionUI) ──TaskCommand JSON──▶ [ 本模块：任务执行 + 仿真 ] ──motorCmd──▶ dSPACE 电控
   (D405 白点/YOLO + UI)            (data/outbox ⇄ inbox)        │ 5 种方法 + auto 调度               （离线轨迹文件）
                                                              ▼
                                              误差模型 / 鲁棒性评估
```

- 上游：视觉模块下达任务（目标位姿 3D→2D 投影，手眼标定前仅仿真模式）；
- 下游：输出电机指令序列（关节角快照 + 夹爪状态机），电控（dSPACE）离线加载、实时插值执行；
- 障碍**仅存在于仿真**（圆/可旋转矩形），视觉不传障碍。

对接细节见：
- `文档/系统架构与接口总览.md` — 系统全景/模块分层/接口契约/概念逻辑图（快速上手）
- `文档/实现方案.md` — 完整设计（架构/算法/误差/调度链）
- `文档/电控对接说明.md` — dSPACE 对接（motorCmd 格式/坐标系/安全/错误码）

## 快速上手（MATLAB）

```matlab
addpath('ArmSimulator2D');

% —— 快捷接口（无障碍，auto 调度） ——
q = solveIK([2.5, 1.0]);                 % 仅位置
q = solveIK([2.5, 1.0], 0.5);            % 含末端角度
q = solveIK([2.5, 1.0], 0.5, 'rrt');     % 指定方法 momentum/sa/rrt/prm/rl/auto

% —— 完整接口（障碍、误差、快照/回调） ——
model = createArmModel(struct(...
    'N', 4, 'L_seg', 1.0, ...
    'X_target', [3.5, 0.0], 'theta_target', 0.0, ...
    'obstacles', struct('circles', [2.0, 0.5, 0.35], 'rects', [1.0, 1.0, 0.3, 0.6, 0.3])));
info = simulateMotion(model, 'auto', zeros(1,4), [3.5, 0.0, 0.0], 'Snapshot', 10);
% info.q_snapshot [K×N] 每 m 步关节角（电机指令序列）
% info.success / error_code / vel_ok / safety_ok / q_final
```

## 统一价值函数

所有方法共享单一标量势（`armValue`/`armGradient`，机器精度梯度校验）：

```
V(q) = w_pos·‖p−X_t‖² + w_ang·wrap(θ−θ_t)²
     + w_obs·Σ −log(g_eff)   （对数屏障，激活距离 barrier_range 内，侵入后饱和排斥）
     + w_var·Σ(q−μ)²/σ²      （关节先验，默认关闭）
```

## 五种迭代方法 + auto 调度

| 方法 | 定位 | 说明 |
|---|---|---|
| `momentum` | 局部精修 | 动量 + β 近目标衰减 + 回溯线搜索，精度达 tol_pos/tol_ang；卡住早停（error_code=7） |
| `sa` | 逃逸兜底 | Metropolis + 几何冷却，侵入样本拒绝，终点无梯度精修 |
| `rrt\*` | 采样层(主) | **渐近最优**（rewire + 代价优化），auto 链默认；`rrt` 保留对比 |
| `graph` | 图引导层 | **走廊图 + A* + 逐段 RRT\***：窄通道/多障碍逃逸（详见实现方案 §5.8） |
| `prm` | 采样层(辅) | 路线图 + 真 A* + 终点精修 |
| `rl` | 经验加速 | OpenAI-ES 策略梯度 + residual 集成（θ=0 起步不劣于纯梯度） |
| `cvae` | 学习层 | L2 策略（见下节），GUI 方法下拉可选 |
| `auto` | 默认推荐 | momentum 快路径 → **RRT\*** → **graph** → PRM → SA，失败逐层升级 |

**局部最优检测**：`detectLocalMin`（‖∇V‖<tol 且末端未达 ⇒ 势阱），auto 链据此决策。

## 模仿学习（L2）—— 已交付 6 关节 0.5m 模型

完整管线（示范生成 → CVAE 训练 → 部署闭环）已跑通并交付：

```matlab
% 部署推理（策略优先 + 安全回退）
addpath('ArmSimulator2D');
m = createArmModel(struct('N',6,'L_seg',0.5));            % 6 关节 0.5m 臂
out = cvaePolicyDeploy('rl_pipeline/policy_cvae_n6.mat', m, q0, [x,y,th]);
% out.traj 轨迹 / out.q_final 终态 / out.via ('cvae'|'rrtstar')
```

- **示范数据**：6000 条（`data/demonstrations_n6/`，RRT* 渐近最优生成，约 2.3h）；**升级版 4200 条**（`data/demonstrations_opt*/`，预算内最优 RRT* 3000 样本 + `optimizeTraj` 短切，路径质量提升约 40%）
- **策略**：`rl_pipeline/policy_cvae_n6.mat`（CVAE 多模态轨迹生成 + 潜空间优化推理 + 终点精修；MATLAB 手写 forward，无 Deep Learning Toolbox 依赖）；升级版 `rl_pipeline/policy_cvae_opt.mat`
- **验收**（与 RRT* 基线同测试集同预算）：成功率与基线相当（78-90%）、**碰撞率 0%**、路径最优性差距约 -3%、生成 ~0.5s；**升级后（5.7.12）**：相对近全局参考（15000 样本+优化）最优性差距 **+86.4% → +20.8%**（路径约为近全局最优 1.2 倍），成功率 88% > 参考 69%，生成 3s
- **关键升级（5.7.12）**：示范从"早停可行解"→"预算内最优 + 轨迹优化"——解决"策略与 RRT* 拉不开差距"的根因（老师天花板 = 学生天花板）
- 重新训练/扩展：`python rl_pipeline/train_cvae.py --data <示范目录> --out <模型.mat>`

## 主要参数（`modelDefaults.m` 唯一事实来源）

几何：`N` `L_seg`(标量或逐段) `q_min/q_max` `rod_offset_arr`
权重：`w_pos/w_ang/w_obs/w_var/w_acc` 屏障：`rho0/safe_margin/barrier_C/barrier_range`
障碍：`obstacles.circles=[x,y,r]` `obstacles.rects=[x,y,θ,w,h]`（可旋转）
迭代：`max_iter/dq_max/tol_pos/tol_ang` 方法：`momentum_beta/rrt_*/sa_*/prm_*`
误差：`error.on/sigma_motor/backlash/kappa`（默认关）

## 文件结构

```
ArmSimulator2D/          # 核心求解器（重构产物，纯函数）
├── createArmModel.m / simulateMotion.m / modelDefaults.m   # 接口 + 默认参数
├── armValue.m / armGradient.m          # 统一价值函数与解析梯度
├── kinematics2D 组 / obstacle2D 组      # 运动学 + 障碍几何（距离/梯度）
├── method_momentum/sa/rrt/rrtstar/graph/prm/rl.m + method_auto.m
├── refineRandomGreedy.m / inverseKinPose.m / detectLocalMin.m
├── errorModel.m / assessRobustness.m / feedbackCorrect.m
├── projectTo2D.m / rasterize2D.m
└── 学习层：generateDemonstrations.m / sampleTask2D.m / cvaePolicy*.m / kalman*.m / handEyeEstimate2D.m
runTaskLoop.m / taskExecute.m / taskToSegments.m / taskWriteStatus.m   # 任务桥（根目录）
exportMotorCmd.m / solveIK.m / ArmSimApp.m / runTopLevel.m             # 导出/GUI/顶层闭环（根目录）
rl_pipeline/            # Python 训练管线（train_cvae.py / train_policy.py / policy_*.mat）
data/                   # 数据：demonstrations_*/ 示范 + failures_*.mat
scripts/                # 评估/实验/诊断脚本（eval_*/verifyPolicy/collectFailures/generateVariants/test_*_diag）
test/                   # 断言式测试：test_all 汇总运行
vision/                 # 视觉模块（独立 git 仓库，参考）
文档/                   # 设计文档（实现方案.md / 电控对接说明.md 等）
archive/                # 已归档旧版/死代码（PlanarDrawApp/runLArmIK_2D/TopLevelSystem 等）
```

## 运行测试

```matlab
addpath('ArmSimulator2D'); addpath('test');
test_all
```

覆盖：障碍几何解析对照、三场景梯度校验（解析 vs 中心差分 < 1e-5）、接口冒烟（收敛/避障/位姿反解/快照/回调/取消/不可达）、auto 调度链、误差模型、RL 冒烟、任务桥 mock 端到端。

## 交互 GUI

```matlab
addpath('ArmSimulator2D');
app = ArmSimApp();          % 新接口 GUI（重构版，见下）
```

`ArmSimApp.m`：基于新接口的交互仿真窗口——
- **模型参数面板**：关节数 N / 杆长 / 限位 / 安全距离 / 迭代上限
- **障碍编辑**：圆 `[x,y,r]` 与可旋转矩形 `[x,y,θ,w,h]` 表格增删改，实时可视化
- **目标位姿** `[x,y,θ]`：十字可拖拽，θ 编辑
- **单次求解**：5 方法 + auto，轨迹回放
- **【模拟视觉任务】**：选择任务类型（move_near_target / pick_target / pick_and_place / dock_to_interface / home）→ 设置目标与放置点 → “下发任务并执行”构造 TaskCommand 并走完整任务链路（展开运动序列 → 逐段求解 → 夹爪动作 GRASPING 闭合 / RELEASING 张开 → 状态流 RECEIVED→ACCEPTED→PLANNING→EXECUTING→COMPLETED 显示于日志区 → 逐段动画）→ “导出 motorCmd”保存 .mat/.csv 供电控（dSPACE）加载；可选填 outbox/inbox 目录演示真实文件桥闭环

旧 `PlanarDrawApp.m` 基于旧接口（runLArmIK_2D），保留仅作迁移对照，新开发请用 `ArmSimApp`。

## 运行依赖

- MATLAB R2017b+（实测 R2026a；依赖 `vecnorm`、`jsondecode`）
- 无需 Optimization Toolbox（新核心无 quadprog 依赖）
- 与视觉联调需 Python（SpaceSnakeVisionUI，D405 mock/实机）
