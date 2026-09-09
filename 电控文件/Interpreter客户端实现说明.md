# ControlDesk Interpreter 客户端实现说明（26 参控制帧接收 → 写 dSPACE 变量）

> **文件**：`interpreter_motor_client.py`（仓库根，配合 `ArmSimulator2D\motorCmdToControlParams.m`/`tcpSendControl.m`）。
> **角色**：运行在 **ControlDesk 的「Interpreter 窗口」(内置 Python)** 里，作为 **TCP 客户端**连接项目侧服务器，接收 **MOTOR_CMD(26 参)帧**，并写入 dSPACE 模型变量。

---

## 1. 它在整条链路中的位置
```
[视觉/规划侧]  taskExecute → motor_cmd → motorCmdToControlParams(26参)
        │  TCP 服务器 用 tcpEncodeFrame 发 MOTOR_CMD 帧（[4字节大端长度][JSON]）
        ▼
[ControlDesk Interpreter 客户端] interpreter_motor_client.py
        │  >I 解帧 → 写 26 参到 dSPACE 变量（含 trigger / run）
        ▼
[dSPACE 实时模型]  6 电机 + 1 舵机 → 10kHz 闭环 → 回传 STATE
```
- 复用本项目帧协议：`[4 字节大端长度][UTF-8 JSON]`，JSON 体 = `{"v":1,"type":"MOTOR_CMD","seq":..,"command_id":"..","data":{..}}`。
- 与 `tcpEncodeFrame.m` 的 `be32` 大端长度**完全一致**（用 `struct.unpack('>I', ...)`）。

## 2. 前置
- [ ] ControlDesk **主版**（MCD-3 自动化；Operator 版不支持）。
- [ ] 在 Interpreter 里确认 `import socket, struct, json` 可用（标准库通常可用）。
- [ ] dSPACE 模型含变量：6 电机 `motor_1..6_enable / motor_1..6_angle / motor_1..6_direction / motor_1..6_trigger` + `servo_angle / servo_enable`（从 ControlDesk 变量浏览器复制**完整路径**，勿手敲）。
- [ ] 模型已 Build 下载、**RUN**；网线连主机调试口（TCP 走 ControlDesk 宿主 PC 网络）。

## 3. 帧与 26 参载荷
`data` = 逐帧（每个运动档拆成【读帧】→【驱动帧】两帧，均带同一组 `motors`/`servo`/`run`）：
```json
"data": {
  "run":     1,                                   // 服务端控制运行参数：1=运行 0=暂停(客户端不写驱动，不重连)
  "motors":[ {"enable":1,"angle_deg":0.0,"direction":1,"trigger":0}, ×6 ],
  "servo":  {"angle_deg":0.0,"enable":1},
  "init":   [0.0,0.0,0.0,0.0,0.0,0.0],            // 仅首发帧：初始绝对关节角(度)，作累计基准
  "mount_sign": [1,-1,1,-1,1,-1]                  // 各电机安装方向符号：奇数+1 / 偶数-1（交叠反向安装）
}
```
| 输入 | 变量 | 取值/单位 |
|---|---|---|
| 电机 j 使能 | `motor_<j>_enable` | 0/1 |
| 电机 j 相对角 | `motor_<j>_angle` | 度（相对，UI 输入回车生效） |
| 电机 j 方向 | `motor_<j>_direction` | 1正/0反（已按安装方向修正：奇数 1,3,5 习惯 1=逆时针/0=顺时针；偶数 2,4,6 交叠反向安装，方向位已取反——见 `mount_sign`） |
| 电机 j 触发 | `motor_<j>_trigger` | 0=读入目标寄存器(不动)；1=驱动到寄存器目标 |
| 舵机绝对角 | `servo_angle` | 度 |
| 舵机使能 | `servo_enable` | 0/1 |
| 运行参数 | `run` | 0/1（服务端控制） |

> 26 参 = 6×4(电机，含 trigger) + 2(舵机)；`init` 为辅助基准，`run` 为服务端运行参数，`mount_sign` 为安装方向符号（均非 26 参内计数）。

**安装方向修正（`mount_sign`）：** 臂段尺寸一致但**交叠反向安装**（弹簧状），故偶数电机 2,4,6 的方向位与 model 正向相反。`motorCmdToControlParams` 已按 `direction = (mount_sign(j)·dq(j) ≥ 0)` 计算方向位——偶数电机方向位自动取反，发给电控的数据即为修正后方向。电控侧驱动时乘回 `mount_sign(j)` 即恢复 model 关节角（本侧/测试 mock 已如此处理）。

**trigger 驱动语义（电控侧 10kHz 执行；各电机独立）：**
- `trigger=0` → 电机把 `angle_deg`（按 `direction` 取符号）读入目标寄存器，**不做运动**。
- `trigger=1` → 电机驱动到寄存器中的目标，**累计运动**。

**帧序列（`motorCmdToControlParams.m` 已生成）：** 每个有运动的档顺序发两帧——先读帧（全电机 `trigger=0`）再驱动帧（运动电机 `trigger=enable`）；无运动的档只发单帧占位。客户端逐帧透传给 dSPACE，由电控按 trigger 执行。

## 4. 实现要点（`interpreter_motor_client.py`）
```python
import socket, struct, json
HOST='127.0.0.1'; PORT=9100; VAR_ROOT='Application/SnakeArm_HIL'
s = socket.create_connection((HOST, PORT)); s.settimeout(0.05)   # 短超时，别卡 UI
buf = b''
V = Application.ActiveExperiment.Platforms[1].ActiveVariableDescription.Variables
while True:
    try: chunk = s.recv(4096)
    except socket.timeout: continue
    if not chunk: break
    buf += chunk
    while len(buf) >= 4:
        n = struct.unpack('>I', buf[:4])[0]        # 大端长度
        if len(buf) < 4+n: break                    # 半包
        body, buf = buf[4:4+n], buf[4+n:]
        msg = json.loads(body)
        if msg.get('type') == 'MOTOR_CMD':
            d = msg['data']
            run = int(d.get('run', 1))              # 服务端运行参数
            if run:                                 # 运行：写 26 参（含 trigger）
                for j, m in enumerate(d.get('motors', []), 1):
                    V[f'{VAR_ROOT}/motor_{j}_enable'].ValueConverted = m['enable']
                    V[f'{VAR_ROOT}/motor_{j}_angle'].ValueConverted  = m['angle_deg']
                    V[f'{VAR_ROOT}/motor_{j}_direction'].ValueConverted = m['direction']
                    V[f'{VAR_ROOT}/motor_{j}_trigger'].ValueConverted = m['trigger']
                sv = d.get('servo', {})
                V[f'{VAR_ROOT}/servo_angle'].ValueConverted = sv.get('angle_deg', 0.0)
                V[f'{VAR_ROOT}/servo_enable'].ValueConverted = sv.get('enable', 0)
            # run=0 -> 暂停：不写驱动变量（保持当前位姿），但继续收帧、不重连
```
- **解帧纯函数**：`struct.unpack('>I', n)` 取长度前缀 → `4+n` 字节 JSON → `json.loads`；粘包/半包由外层 `while + buf` 处理。
- **写变量**：`Application.ActiveExperiment.Platforms[1].ActiveVariableDescription.Variables[path].ValueConverted`（进程内直写，延迟最低）。
- **run 暂停**：服务端 `run=0` 时本端不写驱动相关变量（电控保持当前位姿），`run=1` 恢复写变量；TCP 连接保持、不重连。

## 5. 部署
1. 改 `HOST/PORT/VAR_ROOT`（服务器地址 / 端口 / 变量路径前缀）。
2. 把脚本逻辑粘贴到 ControlDesk Interpreter（或 `python interpreter_motor_client.py`，若 Interpreter 可执行外部脚本）。
3. 项目侧起 TCP 服务器后，Interpreter 客户端连接、收帧、逐参写变量。

## 6. 注意 / 排错
| 项 | 说明 |
|---|---|
| **别阻塞 ControlDesk** | Interpreter 常跑在 UI 线程；用 `settimeout(0.05)` + continue，勿 `while True: recv()` 硬卡 |
| **帧对齐** | 务必 `>I`(大端) + JSON，与 `tcpEncodeFrame`/`tcpDecodeFrame` 一致，否则错位 |
| **变量写不进** | 模型未把变量设为 Global Parameter / 未勾 Allow online modification |
| **连接失败** | 地址/端口错、服务器未起、网口未连 |
| **角度单位** | `MOTOR_CMD.angle_deg` 为**度**；`STATE.joints` 为 **rad**（读回需 rad→deg） |
| **trigger 没动** | 确认读帧 trigger=0 先到、驱动帧 trigger=1 后到；run=0 会暂停驱动 |

## 7. 配套文件
- `interpreter_motor_client.py` — 本说明代码。
- `ArmSimulator2D\motorCmdToControlParams.m` / `tcpSendControl.m` — 项目侧 26 参生成与下发。
- `mock_elec_server.py` / `mockElecControlServer.m` — 电控 mock（本地回环替身，按 trigger 驱动）。
- `test\test_loop_vision_algo_elec.m` / `test\test_chain_full_loop.m` — 全流程回环测试。
- `TASK_COMMAND_INTERFACE.md` — 帧/遥测规范。
