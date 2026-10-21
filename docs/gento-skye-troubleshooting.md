# Gento Skye 启动故障链与排查方法

文档总入口和全部坑的汇总表见 [gento-skye-README.md](gento-skye-README.md)。本文按时间记录每一次出错、原因、修复和运行结果。

本文记录 2026-10-21 在这台 GPUFree 机器上将 Gento Skye 接入
ASPIRE / BEHAVIOR-1K 时遇到的启动问题。目标是区分：

- 仿真环境是否创建成功；
- seed 36 是否加载完成；
- 机器人是否完成第二次复位；
- `policy.py` 是否真正开始执行；
- 进入策略之后是否能抓取。

不要把这些阶段的错误合并成一个“Isaac 打不开”问题。每修复一个启动门槛，
程序会继续向下运行，然后才可能暴露下一层问题。

## 1. 结论

本机版本与 ASPIRE 仓库锁定版本一致，不是 Isaac Sim 版本不兼容：

| 项目 | 版本 |
| --- | --- |
| Isaac Sim | `5.1.0.0` |
| Omniverse Kit | `107.3.3`，git hash `69cbf6ad` |
| OmniGibson | `3.9.2` |
| Python | `3.11` |
| BEHAVIOR 资产 | `3.9.0` |

当前程序此前一直进不到下一步，是因为所有运行都在
`BehaviorAdapter.reset()` 阶段失败。`outcome.json` 的共同特征是：

```json
{
  "steps_executed": 0,
  "program_ok": false,
  "termination_reason": "harness_error"
}
```

`steps_executed: 0` 表示 `outputs/gento-bringup/policy.py` 一行都还没有执行。
此时修改策略程序不能解决问题，应继续修环境复位。

截至本文更新，已依次修复：

1. 缺少左手辅助抓取射线点；
2. 环境构造时找不到 Gento 预采样底盘位姿；
3. camera 控制器使用增量命令，不能被 `q_to_action()` 保持；
4. 29 维复位关节数组使用了错误顺序。

## 2. 一次运行的实际调用顺序

`cap-harness run` 并不是先执行 `policy.py`。实际顺序是：

```text
cap-harness run
  │
  ├─ 创建 BehaviorAdapter
  │
  ├─ adapter.reset(metadata, seed)
  │    │
  │    ├─ _ensure_environment()
  │    │    └─ og.Environment(config)
  │    │         ├─ 加载场景、机器人、任务
  │    │         ├─ play()
  │    │         └─ post_play_load()
  │    │              └─ env.reset()      ← 构造函数内部已经复位一次
  │    │
  │    ├─ env.reset()                     ← 适配器的基础复位
  │    ├─ _load_task_instance(seed=36)   ← 加载任务实例和 Gento 底盘
  │    ├─ env.reset()                     ← 从 seed 36 快照重新开局
  │    ├─ 打开夹爪
  │    ├─ _raw_step_settle()             ← q_to_action + env.step
  │    ├─ GENTO_HOLD（若启用）
  │    └─ 相机和 odom 初始化
  │
  └─ 执行 policy.py
```

所以环境构造、任务加载、第二次复位、控制器 settle 中的任意异常，都会使
`steps_executed` 保持为 0。

## 3. 错误一：`KeyError: 'left'`

### 现象

Kit 日志先出现末端推断 warning：

```text
Expected computed z_lower / z_upper bounds ... negative / positive,
but instead got: 0.100487, 0.034374
```

随后在物理步里出现：

```text
KeyError: 'left'
Exception occurred during pre-physics step
```

### 原因

`grasping_mode: assisted` 会在每个物理步里执行辅助抓取：

```python
self.assisted_grasp_start_points["left"]
self.assisted_grasp_end_points["left"]
```

Gento 的 yaml 原来没有这些点。OmniGibson 尝试根据手指碰撞网格自动推断，
但 `left_eef_link` 的原点和 Z 轴不满足推断条件。推断失败只打印 warning，
不会停止加载；后面访问空字典的 `"left"` 时才真正退出。

### 修复

在：

```text
/root/gpufree-data/aspire/behavior-data/
omnigibson-robot-assets/models/gento_skye/gento_skye.yaml
```

的 `manipulation` 下显式写入：

```yaml
assisted_grasp_start_points:
  left:
  - [Arm_L10_Link, [0.0, -0.14, 0.042]]
  - [Arm_L10_Link, [0.0, -0.18, 0.042]]
  right:
  - [Arm_R10_Link, [0.0, -0.14, 0.042]]
  - [Arm_R10_Link, [0.0, -0.18, 0.042]]
assisted_grasp_end_points:
  left:
  - [Arm_L11_Link, [0.0, -0.14, -0.042]]
  - [Arm_L11_Link, [0.0, -0.18, -0.042]]
  right:
  - [Arm_R11_Link, [-0.0013, 1.933, 0.042]]
  - [Arm_R11_Link, [-0.0013, 1.973, 0.042]]
```

这些坐标是对应手指连杆的局部坐标，单位是米，不是世界坐标。

只想先看窗口时可以临时使用：

```python
"grasping_mode": "physical"
```

但 `physical` 模式不会生成辅助射线。需要 ASPIRE 辅助抓取时必须改回
`assisted`，并保证左右手起点和终点都存在。

### 右手 `Arm_R11_Link` 的坐标

`Arm_R11_Link` 的网格和 cuRobo 碰撞球在它自己的坐标系里，y 都在 1.9–2.0 m。
这不是错误。URDF 里 `Arm_R11_Joint` 的原点是 `xyz="0.0013483 1.793 0"`，
`rpy="-3.1416 0 0"`，即平移 1.793 m 再绕 X 翻转 180°。换算回父连杆
`Arm_R9_Link` 后，网格 y 落在 −0.21 到 −0.11 m，碰撞球中心是
(0.001, −0.160, −0.056)，和左手 `Arm_L11_Link` 对称。

因为局部 y、z 是翻转的，右手终点不能照抄左手的 `[0, -0.14, -0.042]`。
父连杆里的 (0, −0.14, −0.042) 换算到 `Arm_R11_Link` 是 (−0.0013, 1.933, 0.042)。
上面的 yaml 已经按这个换算写入。不要去“修”这颗碰撞球。

## 4. 错误二：找不到 Gento 预采样位姿

### 现象

```text
No generic or model-specific presampled robot pose found for gento_skye!
```

运行目录显示：

```json
{
  "steps_executed": 0,
  "termination_reason": "harness_error"
}
```

### 原因

seed 36 的任务实例文件：

```text
behavior-data/2026-challenge-task-instances/scenes/
house_double_floor_lower/json/
house_double_floor_lower_task_turning_on_radio_instances/
house_double_floor_lower_task_turning_on_radio_0_36_template-tro_state.json
```

包含：

```json
"robot_poses": {
  "R1Pro": [...],
  "Fetch": [...]
}
```

没有 `"robot"`，也没有 `"gento_skye"`。

`BehaviorTask.reset()` 在 `use_presampled_robot_pose=True` 时只接受：

1. 通用键 `"robot"`；
2. 当前型号键 `"gento_skye"`。

它不会自动使用 `"R1Pro"`。

更关键的是，`og.Environment(config)` 的构造函数内部已经调用一次
`env.reset()`。这发生在 `BehaviorAdapter._ensure_environment()` 返回之前，
因此在 `BehaviorAdapter.reset()` 开头再写位姿已经太晚。

此前反复尝试写场景 metadata 仍无效，就是因为代码写在异常发生之后，
或者在 `scene.reset()` 时又被 `_initial_file` 快照覆盖。

### 正确修复

`src/cap_harness/behavior/config.py`：

```python
"use_presampled_robot_pose": False,
```

这样环境构造时不要求官方预采样位姿。

真正的 seed 36 位姿由 `_load_task_instance()` 加载：

1. 把任务 JSON 的键转成小写；
2. 依次查找 `robot`、`gento_skye`、`r1pro`；
3. seed 36 命中 `r1pro`；
4. `_gento_base_pose()` 将 R1 位姿换算成 Gento 位姿；
5. 在内存里写入 `state["robot"]`；
6. 更新场景初始快照，再执行第二次复位。

任务 JSON 文件本身没有修改。

### 已验证

修复后的真实运行成功打印：

```text
GENTO_JOINTS [...]
```

这证明以下阶段都已经通过：

- 环境构造；
- 构造函数内部复位；
- 适配器第一次复位；
- seed 36 读取；
- Gento 底盘位姿转换；
- 第二次复位。

因此后续日志再失败时，不应继续修改 `robot_poses`。

## 5. 错误三：camera 控制器不是绝对控制

### 现象

最新一次运行的 `outcome.json`：

```json
{
  "error": {
    "message": "Controller [camera] should be a JointController/HolonomicBaseJointController with use_delta_commands=False!",
    "type": "AssertionError"
  },
  "steps_executed": 0
}
```

这次已经打印了 `GENTO_JOINTS`，然后在复位 settle 阶段失败。

### 原因

`_raw_step_settle()` 会调用：

```python
robot.q_to_action(current_joint_target)
```

`q_to_action()` 必须把完整关节目标转换成一个可以保持当前姿态的动作。
它要求所有控制器都是：

- `JointController`，底盘也允许 `HolonomicBaseJointController`；
- `use_delta_commands=False`。

OmniGibson 的默认 camera 控制器虽然是 `JointController`，但默认配置为：

```python
"use_delta_commands": True
```

此前 `config.py` 只覆盖了底盘、腰、双臂和夹爪，没有覆盖 camera，
所以 camera 保留了增量控制。

### 修复

在 `controller_config` 中加入：

```python
"camera": deepcopy(_POSITION_JOINT_CONTROLLER),
```

其中 `_POSITION_JOINT_CONTROLLER` 是：

```python
{
    "name": "JointController",
    "motor_type": "position",
    "command_input_limits": None,
    "use_delta_commands": False,
    "use_impedances": False,
}
```

现在底盘、腰、camera、双臂和双夹爪都可以被 `q_to_action()` 作为绝对目标保持。

## 6. 错误四：29 维复位数组顺序错误

### 现象

以前按以下分组理解 29 个数：

```text
6 底座 + 3 腰 + 2 头 + 左臂 7 + 左夹爪 2 + 右臂 7 + 右夹爪 2
```

但真实运行时关节顺序不是这个分组。按错误顺序写数组会导致：

- 左臂角写进右臂；
- 手臂角写进头部；
- 头部、升降和双臂姿态一起异常；
- 看起来像 URDF 或 R1 权重维度不匹配。

这不是策略权重问题，而是 `reset_joint_pos` 与 articulation DOF 顺序不一致。

### 真实顺序

最新 Kit 日志打印：

```text
0  base_footprint_x_joint
1  base_footprint_y_joint
2  base_footprint_z_joint
3  base_footprint_rx_joint
4  base_footprint_ry_joint
5  base_footprint_rz_joint
6  waist_joint_1
7  waist_joint_2
8  waist_joint_3
9  Head_J1_Joint
10 Joint1_L
11 Joint1_R
12 Head_J2_Joint
13 Joint2_L
14 Joint2_R
15 Joint3_L
16 Joint3_R
17 Joint4_L
18 Joint4_R
19 Joint5_L
20 Joint5_R
21 Joint6_L
22 Joint6_R
23 Joint7_L
24 Joint7_R
25 Arm_L10_Joint
26 Arm_L11_Joint
27 Arm_R10_Joint
28 Arm_R11_Joint
```

左右臂是逐关节交错排列，两个头关节也被左右臂第一关节隔开。

### 修复

已经同步重排：

- `src/cap_harness/behavior/config.py` 的 `RESET_JOINT_POSITIONS`；
- `gento_skye.yaml` 的 `default_joint_pos`。

两份均已验证：

```text
长度 = 29
内容完全相同
```

以后修改家位置时，必须按上面的真实顺序填写，不能按功能分组拼接。

前 6 个虚拟底座关节不能用来调整机器人在房间里的出生点。
房间位置由 `_gento_base_pose()` 和 `set_position_orientation()` 决定。

## 7. 为什么窗口会关闭

窗口属于 `cap-harness` 启动的同一个 Isaac 进程。只要 reset 抛异常，
`run.py` 就进入 finalization 并关闭环境，所以窗口跟着关闭。

这不是 Isaac 的自动复位按钮，也不是窗口配置本身的问题。

要显示窗口，在启动前、同一个 shell 中执行：

```bash
export OMNIGIBSON_HEADLESS=0
```

日志里出现 `--no-window` 表示该次进程仍然是无头模式。

要在完成复位后暂停：

```bash
export GENTO_HOLD=1
```

适配器到达暂停点后会打印：

```text
GENTO_HOLD: 窗口停在这里……
```

此时程序是有意等待终端输入，`policy.py` 还没有开始。看完窗口后在启动
`cap-harness` 的同一终端按回车，流程才继续。

最早的版本用 `input()` 等回车。Isaac 的界面和 Python 在同一个主线程上，
`input()` 阻塞时界面不刷新，窗口看起来停着但点不动。现在的版本在等待期间
反复调用 `og.sim.render()`：界面照常刷新、可以操作，物理不推进，机器人停在
复位姿态。等待前还会在终端打印 `GENTO_EEF` 几行：左右末端在世界里的 X/Y/Z
轴方向，以及每根手指中心在末端坐标里的位置。两根手指的 z 都为正、y 一正一负，
就说明末端轴是对的，不用在窗口里点也能判断。

若只想验证能否进入策略，执行：

```bash
unset GENTO_HOLD
```

## 8. 如何判断现在卡在哪一层

每次只看该次运行目录，不要继续看旧日志：

```bash
ls -dt outputs/gento-bringup/0036/run-* | head -1
```

读取：

```bash
RUN="$(ls -dt outputs/gento-bringup/0036/run-* | head -1)"
cat "$RUN/outcome.json"
```

判断规则：

| 现象 | 阶段 |
| --- | --- |
| 没有 Isaac 窗口，Kit 参数有 `--no-window` | 环境变量问题 |
| presampled pose 错误 | 环境构造 / BehaviorTask reset |
| 没有 `GENTO_JOINTS` | 还没完成 seed 36 加载 |
| 有 `GENTO_JOINTS`，随后 camera controller 错误 | settle / `q_to_action` |
| 有 `GENTO_HOLD`，终端不动 | 正在等待人工回车，不是异常 |
| `steps_executed: 0` | 策略尚未开始 |
| `steps_executed > 0` | 已经进入 `policy.py` |
| `program_ok: true` | 策略正常返回 |
| 退出码 139 | 只表示 Kit 收尾 segfault，继续以 `outcome.json` 为准 |

Kit 日志目录：

```text
/root/gpufree-data/aspire/behavior-appdata/local/logs/
Kit/OmniGibson/3.9/kit_*.log
```

搜索有用信息：

```bash
rg "cap-harness run|Traceback|KeyError|AssertionError|GENTO" \
  /root/gpufree-data/aspire/behavior-appdata/local/logs/Kit/OmniGibson/3.9/kit_*.log
```

## 9. 当前已经完成的修复

当前代码状态：

- `use_presampled_robot_pose=False`；
- `_load_task_instance()` 使用 R1Pro 样本换算 Gento 底盘；
- 内存 metadata 增加通用 `robot` 键；
- `grasping_mode="assisted"`，`grasping_direction="upper"`（Gento 手指关节值越大越闭合）；
- yaml 已包含左右手辅助射线点，右手终点按 `Arm_R11_Joint` 的翻转换算过；
- camera 使用绝对位置 `JointController`；
- 29 维复位数组已按真实 articulation 顺序重排，手指关节复位为 0（张开）；
- `config.py` 和 Gento yaml 的复位数组完全一致；
- 头相机在 USD 里转 180° 并前移 1 cm，能看到房间；
- 启动变量包含 cuRobo 需要的 `LD_LIBRARY_PATH`；
- `GENTO_HOLD=1` 时暂停期间持续渲染，窗口可以操作，并打印 `GENTO_EEF` 末端轴；
- behavior 相关测试 `67 passed`。

- cuRobo 碰撞球按网格重新拟合（215 颗），忽略列表按数据重建，19 个底盘附件移出碰撞模型；
- `GENTO_PLAN_DIAG=1` 时打印 cuRobo 诊断。

14:41 那次 bring-up 六项全部为真，`move_to_pose` 成功（见第 11 节末）。第二层已完成。总框架在 [gento-skye-workflow.md](gento-skye-workflow.md)，全部改动清单在 [gento-skye-README.md](gento-skye-README.md) 第 3 节。

数据集 `robot_poses` JSON 没有修改。

## 10. 重新验证

每个新 shell 先执行：

```bash
cd /root/gpufree-data/aspire/ASPIRE
export PATH="$HOME/.local/bin:$PATH"
export CAP_HARNESS="$PWD/.venv-behavior/bin/cap-harness"
export OMNIGIBSON_DATA_PATH=/root/gpufree-data/aspire/behavior-data
export OMNIGIBSON_APPDATA_PATH=/root/gpufree-data/aspire/behavior-appdata
export OMNIGIBSON_GPU_ID=0
export CUDA_VISIBLE_DEVICES=0
ulimit -c 0
export OMNIGIBSON_HEADLESS=0
```

验证流程是否能进入策略时：

```bash
unset GENTO_HOLD
```

运行：

```bash
mkdir -p outputs/gento-bringup
"$CAP_HARNESS" run \
  --benchmark behavior --suite turning_on_radio --task-id 0 --seed 36 \
  --program outputs/gento-bringup/policy.py \
  --output-root outputs/gento-bringup --flat-run-dir \
  --max-steps 2000 --camera-width 512 --camera-height 512
```

验收条件：

```text
steps_executed > 0
```

如果还需要看末端轴，再执行：

```bash
export GENTO_HOLD=1
```

重新运行，等终端打印 `GENTO_HOLD` 后，在 Stage 搜索：

```text
left_eef_link
right_eef_link
```

按 `W` 显示轴：红 X、绿 Y、蓝 Z。

## 11. 进入策略后 bring-up 暴露的两个问题

本节按时间顺序记录，每一小节写的是当时的判断。后面的小节会推翻前面的结论（例如 `Arm_R11_Link`、前三轮的忽略列表），以本节最后的“第四轮”和 [gento-skye-collision-spheres.md](gento-skye-collision-spheres.md) 为准。

2026-10-21 09:09 那次运行第一次进入策略：`program_ok: true`，118 步。`report` 里相机、两只夹爪、腰、底盘转向都通过，只有 `reach_ok` 为假。检查关键帧时又发现头相机全黑。

### 头相机全黑

三个相机都能取到图，但 `head.png` 每个像素都是 0，深度图全部在 6–23 mm。腕相机正常。

`head_depth_camera_Link` 的外壳网格在连杆原点下方，z 从 −23 mm 到 +2 mm，镜头面朝连杆 +Z。USD 里这个连杆的 +Z 指向机器人正前方，+Y 朝上。Isaac 的相机沿自身 −Z 看，导入配置给的是单位四元数，所以相机正对着自己的外壳。

修法是让相机绕 Y 轴转 180°：

- `behavior-data/omnigibson-robot-assets/models/gento_skye/usd/gento_skye.usda` 里 `head_depth_camera_Link` 下的 `Camera` 节点，`xformOp:orient` 从 `(1, 0, 0, 0)` 改成 `(0, 0, 1, 0)`（USD 的顺序是 wxyz）。
- `gento-skye-import/gento_skye_source_config.yaml` 里同一相机的 `orientation` 改成 xyzw `[0, 1, 0, 0]`，以后重导入时不会丢。

09:37 那次运行验证了转向：头相机仍是黑的，但深度从 6–23 mm 不等变成了处处 1.25 mm，说明相机已经朝前，正对着外壳前表面（网格 z 最大约 +2 mm）。所以还要把相机沿连杆 +Z 前移 1 cm：

- USD 同一 `Camera` 节点的 `xformOp:translate` 从 `(0, 0, 0)` 改成 `(0, 0, 0.01)`。
- 导入配置的 `position` 改成 `[0, 0, 0.01]`。

改完重跑 bring-up，打开 `media/keyframes/*/head.png`。应能看到房间，`head-depth.npy` 的中位数应在米级。深度仍是几毫米，就再前移；画面上下颠倒，把两处朝向都改成绕 X 轴转 180°：USD `(0, 1, 0, 0)`，配置 `[1, 0, 0, 0]`。

`left_eye_camera_Link`、`right_eye_camera_Link`、`waist_depth_camera_Link` 也是单位四元数，ASPIRE 不用这几个相机，没有改。

### `move_to_pose` 返回 `planner_failed`

程序输出里有：

```text
planning_failed: integrated pose planning provider failed:
nvrtc: error: failed to open libnvrtc-builtins.so.12.6.
```

这不是机器人或规划问题，cuRobo 根本没开始规划。PyTorch 自带 12.6 版 `libnvrtc`，编译内核时按名字找 `libnvrtc-builtins.so.12.6`。这个文件在虚拟环境里：

```text
.venv-behavior/lib/python3.11/site-packages/nvidia/cuda_nvrtc/lib/
```

系统的 `LD_LIBRARY_PATH` 只有 `/usr/local/cuda/lib64`，那里是 12.8 版。启动前加上：

```bash
export LD_LIBRARY_PATH="$PWD/.venv-behavior/lib/python3.11/site-packages/nvidia/cuda_nvrtc/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

这一行已经写进 [gento-skye-startup.md](gento-skye-startup.md) 的变量段。必须在启动 `cap-harness run` 之前导出，进程启动后再设置不起作用。

09:37 那次运行加了这一行，nvrtc 错误消失，cuRobo 开始规划，但 `reach_ok` 仍为假：

```text
planning_failed: cuRobo found no collision-free solution (IK Fail; ignored objects: none).
A collision probe also flags the arm's current configuration ...
```

这才是真正的规划失败。后半句说明复位姿态本身就被判为碰撞：从一个已经碰撞的姿态出发，目标只抬 5 cm 也解不出来。右手 `Arm_R11_Link` 的碰撞球是对的（见第 3 节），不是它。先在窗口里看复位姿态，两只手臂有没有互相交叉、插进躯干，或碰到门框。复位手臂角是从另一套遥操作家位置带过来的，还没有按 Gento 调过。

### 运行记录：10:02 和 12:23

**10:02（`run-20261021T100210.847248Z-4cc43959`）没有结果。** 目录里没有 `outcome.json`。这次用的还是旧的 `input()` 暂停，窗口卡死后进程被中断，不用看。

**12:23（`run-20261021T122343.301042Z-3ab9e329`）跑完了。** 这是第一次用新暂停代码停住窗口的运行。

| 项 | 结果 |
| --- | --- |
| `program_ok` | true，`termination_reason: program_completed` |
| `steps_executed` | 118 |
| reset 用时 | 532.7 s，含停在窗口里人工查看的时间；不停窗时约 150 s |
| 三个相机 | 都有 |
| 两只夹爪 `open_gripper` | 返回成功，但这次还是旧方向，实际是闭合（见下） |
| 腰 `move_torso` / `reset_torso` | 成功，误差约 0.009 rad |
| 底盘转向 | 成功，目标 +0.4 rad，实际 0.32 rad，剩余误差 0.078 rad |
| 左手抬高 5 cm `move_to_pose` | 失败，`planner_failed`，执行 0 步 |

`move_to_pose` 的输出：

```text
planning_failed: cuRobo found no collision-free solution (IK Fail; ignored objects: none).
A collision probe also flags the arm's current configuration ...
```

头相机：六个关键帧的 `head.png` 平均亮度 99–118（以前是 0）；`head-depth.npy` 中位数 0.92 m，最大 6.16 m，没有 inf。复位时头部朝向壁炉、壁挂电视和电视柜，看不到收音机。收音机在沙发前的茶几上，在机器人左后方，agent 第一段代码需要先转身或移动才能看见。

这次运行早于下面的夹爪方向修复，所以它的腕相机画面里两指仍然几乎贴在一起。下一次运行要重新确认腕相机里两指分开，以及 `reach_ok`。

### 运行记录：12:34（夹爪方向修复之后）

`run-20261021T123425.645822Z-b868d8a9`：`program_ok` 为真，118 步，reset 150.3 s。

- 夹爪方向修复生效。`open_gripper` 之后两个腕相机里，两根手指分别在画面左右边缘，中间是空的；12:23 那次两指几乎贴在一起。
- 头相机正常，复位时朝向壁炉和电视。
- 腰、底盘转向的结果和前两次相同。
- `move_to_pose` 仍是同一个错误：`IK Fail`，并提示当前姿态被判为碰撞。

左腕相机里，壁炉的木立柱几乎贴着镜头，说明复位时左手离壁炉很近。状态是 `IK Fail`，不是 cuRobo 的 `Start State` 错误，所以 cuRobo 并没有把起点本身判为非法；只是目标（当前末端上方 5 cm，朝向不变）找不到无碰撞的逆解。可能的原因有三种：手臂碰到壁炉等家具、复位手臂姿态自身碰撞或贴着关节限位、cuRobo 的运动学和仿真对不上。这三种只能在有 GPU 的真实进程里区分。

### 运行记录：12:57

`run-20261021T125715.292125Z-efe69014`：`program_ok` 为真，118 步，`termination_reason: program_completed`，reset 188.5 秒。`program-result.json` 与 12:34 相同：三个相机都在，两只夹爪、腰、底盘转向成功，`reach_ok` 为假。标准输出仍是 `IK Fail`，并提示当前姿态被碰撞探测标出。这次运行本身没有 `GENTO_PLAN` 行。碰撞对象在后面 13:07 那次诊断里分开了，见本节后半。

### 复位姿态的 cuRobo 诊断

`planning.py` 新增 `diagnose_current_configuration()`。设置 `GENTO_HOLD=1` 时在暂停前自动运行；不想停窗口时设置 `GENTO_PLAN_DIAG=1`，复位后照样打印。输出都以 `GENTO_PLAN` 开头：

| 行 | 含义 |
| --- | --- |
| `与场景碰撞` / `含自身碰撞的总判定` | 用 OmniGibson 的 `check_collisions` 分别关、开自身碰撞各查一次 |
| `与场景碰撞的连杆` | 每次只保留一个连杆的碰撞球，看哪些连杆碰到家具或墙 |
| `自身重叠最深的连杆对` | 碰撞球两两相交、且不在 `self_collision_ignore` 里的连杆对，按重叠深度排序 |
| `关节限位` | cuRobo 看到的关节值超出或距限位不足 0.02 的关节 |
| `正运动学 …` | cuRobo 算出的末端位置和仿真里的末端位置相差多少 |

第一次调用要预热 cuRobo，大约 1 分钟。

取结果：

```bash
grep "GENTO_PLAN" $(ls -t /root/gpufree-data/aspire/behavior-appdata/local/logs/Kit/OmniGibson/3.9/kit_*.log | head -1) | sed 's/.*GENTO_PLAN/GENTO_PLAN/'
```

#### 诊断第一版的报错：`'KinematicModelState' object has no attribute 'ee_position'`

13:00 左右那次运行只打印了一行：

```text
GENTO_PLAN 诊断失败: AttributeError: 'KinematicModelState' object has no attribute 'ee_position'
```

机器上有两份 cuRobo，接口不同：

- 运行时加载的是 `.venv-behavior/lib/python3.11/site-packages/curobo/`，来自 `github.com/StanfordVL/curobo` 提交 `78612f4`，OmniGibson 指定的版本。
- `ASPIRE/third_party/curobo/` 是 NVlabs 较新的版本，类名和字段名改过。BEHAVIOR 进程内的规划用不到它。

即使在装好的版本里，也有两个相似的类：`MotionGen.compute_kinematics()` 返回 `KinematicModelState`，末端字段叫 `ee_pos_seq`、`ee_quat_seq`，连杆叫 `link_pos_seq`；`CudaRobotModel.get_state()` 返回 `CudaRobotModelState`，末端字段才叫 `ee_position`。第一版代码拿后者的字段名去读前者。

另外，第一版把所有结果攒到最后一次性返回，所以最后一项报错时，前面算好的碰撞结果也没打印出来。

修复：

- 字段名改成 `ee_pos_seq` / `link_pos_seq`。
- 五项检查（碰撞判定、场景碰撞连杆、自身重叠、关节限位、正运动学）各自 `try`，算完一项立刻打印；失败的一项只打印 `××检查失败: …`，不影响其他项。

以后改 BEHAVIOR 的 cuRobo 代码，先确认加载路径再查字段：

```bash
.venv-behavior/bin/python -c "import curobo; print(curobo.__file__)"
```

#### 诊断结果（13:07 那次运行）与修复

```text
与场景碰撞: True；含自身碰撞的总判定: True
与场景碰撞的连杆: YQL_Link, YHL_Link
自身重叠最深的连杆对（未在忽略列表里）:
  base_link <-> waist_Link_2  重叠 66.1 cm
  base_link <-> waist_Link_3  重叠 50.5 cm
  Link3_R <-> base_link  重叠 34.7 cm
  Link4_R <-> base_link  重叠 34.5 cm
  Link3_L <-> base_link  重叠 34.1 cm
  Link4_L <-> base_link  重叠 34.0 cm
  Link5_R <-> base_link  重叠 31.7 cm
  Link5_L <-> base_link  重叠 31.4 cm
关节限位: waist_joint_2=-0.000 贴近限位 [-1.571, 0.000]
正运动学 left_eef_link: cuRobo 与仿真相差 0.0 cm
正运动学 right_eef_link: cuRobo 与仿真相差 0.0 cm
```

结论：

- **运动学没问题。** 两只手 0.0 cm，cuRobo 的模型和仿真一致。
- **场景碰撞不是壁炉，是轮子压地。** `YQL_Link`、`YHL_Link` 是右侧两个轮子。轮轴离地 0.07 m，轮子碰撞球半径也是 0.07 m，球底贴地；cuRobo 把 2 cm 以内都算碰撞（`COLLISION_ACTIVATION_DISTANCE_M = 0.02`）。
- **自身碰撞的根因是 `base_link` 只有一颗球**：中心 (0.006, 0, 0.774)，半径 0.735 m，直径接近 1.5 m，把整台机器人包在里面。腰和两只手臂永远和它重叠，所以任何目标都是 `IK Fail`。R1 Pro 的 `base_link` 是 6 颗半径 15–18 cm 的球，轮子没有球。
- `waist_joint_2` 正好停在上限 0，不影响求解，暂不改。

`base_link` 网格分层测量（`meshes/skye/base_link.STL`）：底盘 x −0.33…0.35 m、y ±0.27 m、z 0.04…0.40 m；立柱 x −0.17…0.01 m、y ±0.13 m、z 0.40…1.51 m，在腰部滑轨后面。

修改（4 份 cuRobo yaml 和导入配置 `gento_skye_source_config.yaml` 的 `curobo` 段同步；原文件备份在 `models/gento_skye/misc/curobo-backup-20261021/`）：

| 项 | 原来 | 现在 |
| --- | --- | --- |
| `base_link` 碰撞球 | 1 颗，r = 0.735 m | 底盘 6 颗：x ∈ {−0.17, 0, 0.17}，y ∈ {−0.11, 0.11}，z = 0.21，r = 0.16（底部离地 5 cm）；立柱 7 颗：x = −0.084，y = 0，z = 0.50…1.40 每 0.15 m，r = 0.12 |
| 轮子 `ZQL/YQL/ZHL/YHL_Link` 半径 | 0.07 | 0.04（底部离地 3 cm，轮子本身已在底盘球范围内） |
| `self_collision_ignore.base_link` | 含 `waist_Link_1_prismatic` | 再加 `waist_Link_2`、`waist_Link_3`（躯干贴着立柱滑动，按设计就接触） |

代价：躯干向前弯腰时和底盘之间不再做自身碰撞检查。以后用 `waist_joint_2` 大幅弯腰时注意。

#### 第二轮诊断（13:21 那次运行）与修复

改完碰撞球后：

```text
与场景碰撞: False；含自身碰撞的总判定: True
与场景碰撞的连杆: 无
自身重叠最深的连杆对（未在忽略列表里）:
  Link2_R <-> waist_Link_3  重叠 8.1 cm
  Link2_L <-> waist_Link_3  重叠 7.9 cm
  Link5_L <-> Link7_L  重叠 6.7 cm
  Link5_R <-> Link7_R  重叠 6.7 cm
  Arm_L9_Link <-> TCP_Link_L  重叠 4.3 cm
  Arm_R9_Link <-> TCP_Link_R  重叠 4.3 cm
  Arm_R9_Link <-> Link_Right_wrist  重叠 4.1 cm
  Arm_L9_Link <-> Link_Left_wrist  重叠 4.1 cm
正运动学 两只手: 0.0 cm
```

场景碰撞和 `base_link` 的问题都已消失，`reach_ok` 仍为假。剩下的都是结构上本来就挨着的连杆。目标只比当前姿态高 5 cm，这些重叠在目标姿态里同样存在，所以逆解永远被拒。

导入器生成的 `self_collision_ignore` 只有直接父子对。R1 Pro 还忽略隔一节的手腕连杆（`left_arm_link5` ↔ `left_arm_link7`）和躯干对肩部（`torso_link4` ↔ `left_arm_link1`）。Gento 的 `Link7 → TCP_Link → Link8 → {Link_*_wrist, Arm_*9_Link}` 全是 fixed 关节，五个连杆相对位置不变，但只忽略了相邻的几对。

每只手补了 14 对，左右对称，共 28 对（4 份 cuRobo yaml 和导入配置同步）：

- `Link7`、`TCP_Link`、`Link8`、`Link_*_wrist`、`Arm_*9_Link` 两两之间全部忽略；
- `Link5` ↔ `Link7`；
- `waist_Link_3` ↔ `Link2`；
- 两根手指分别 ↔ `Link8`、`TCP_Link`、`Link_*_wrist`。

代价：肩部第二节和躯干之间、手腕两节之间不再做自身碰撞检查。手臂其他连杆和躯干、两臂之间的检查仍然保留。

导入配置用脚本改写会丢注释，每次改完要把头相机、腕相机、碰撞球、忽略列表四段注释补回（`grep -c "^ *#"` 应为 14 行左右）。改前的副本：`models/gento_skye/misc/curobo-backup-20261021/`。

#### 第三轮诊断（13:30 那次运行）与修复

```text
与场景碰撞: False；含自身碰撞的总判定: True
自身重叠最深的连杆对（未在忽略列表里）:
  waist_Link_1_prismatic <-> waist_Link_3  重叠 3.6 cm
  YQL_Link <-> base_link  重叠 2.4 cm
  ZHL_Link <-> base_link  重叠 2.4 cm
  ZQL_Link <-> base_link  重叠 2.4 cm
  YHL_Link <-> base_link  重叠 2.4 cm
```

上一轮的 8 对全部消失。只列出 5 对（上限 8），说明这就是复位姿态下剩余的全部重叠：

- `waist_Link_1_prismatic` ↔ `waist_Link_3`：腰部隔一节，和 `Link5` ↔ `Link7` 同类。
- 四个轮子 ↔ `base_link`：新加的底盘球包住了轮子顶部。轮子装在底盘里，R1 Pro 也忽略轮子和底盘。

两类都加进 `self_collision_ignore`（4 份 cuRobo yaml 和导入配置同步，5 对）。这次改导入配置的脚本会把注释原样插回，不用再手补。

诊断新增第六项“逆运动学自检”：直接对左手求两次逆解，目标分别是当前位置和上抬 5 cm，朝向不变，用 ARM embodiment，开场景碰撞检查。输出 `左手逆运动学（原地）: 有解/无解` 和 `左手逆运动学（上抬 5 cm）: 有解/无解`。

- 两次都有解：`move_to_pose` 应该成功；仍失败就查 `planning.py` 里目标位姿从 odom 到世界的换算。
- 原地无解：仍有碰撞或限位问题没被诊断列出。
- 原地有解、上抬无解：手臂在复位姿态下已经接近够不着的边缘，改复位手臂角。

#### 第四轮：重新拟合碰撞球，忽略列表按数据重建

整体设计、和其他规划器的对比、重新生成步骤见 [gento-skye-collision-spheres.md](gento-skye-collision-spheres.md)。下面的“15904 在快速算法以内”漏算了每只手 32 颗抓取预留球，含预留球是 30304，走的是慢速算法，详见那份文档第 8 节。

前三轮的做法有问题：导入器给每个连杆只生成一颗外接球，长或扁的连杆被“吹胖”好几厘米（`waist_Link_3` 两侧各胖约 9 cm，`Link5` 两侧各胖约 6 cm），相邻零件的球本来就压在一起。我用“忽略”把这些重叠绕过去，等于让底盘↔腰、躯干↔肩部 `Link2` 这些真可能撞上的组合也不再检查。这一轮改成先把球拟合准，再只忽略有依据的连杆对。

工具都在 `gento-skye-import/`：

| 脚本 | 作用 |
| --- | --- |
| `fit_collision_spheres.py` | 按网格拟合碰撞球，打印每个连杆的漏覆盖和虚胖，`--write` 存到 `fitted_collision_spheres.yaml` |
| `check_collision_spheres.py` | 不用 GPU 的离线检查：用 URDF 算正运动学，报告需检查的球对数、复位重叠、离地间隙、随机姿态下总是重叠的连杆对；`--never-close-samples` 找采样中从未靠近的连杆对 |
| `apply_collision_spheres.py` | 写回 4 份 cuRobo yaml 和导入配置，写之前自动备份 |

**拟合方法。** STL 是由许多零件拼成的开口外壳（例如 `waist_Link_3` 有 24 块零件，不封闭，法向只有一半朝外），直接判断内外不可靠。改成：拆成独立零件、各取凸包，拿这些凸包的并集当实体；球心取在实体内部，半径取“到表面的距离 + 允许的虚胖”；每次加一颗能把最多表面点拉进容差的球，直到表面上任一点离最近的球都不超过容差。cuRobo 自带的 `fit_spheres_to_mesh` 试过，漏覆盖达到 5–20 cm，不能用。

| 部分 | 容差（漏覆盖） | 虚胖 | 说明 |
| --- | --- | --- | --- |
| 手臂、夹爪、肩、腰第二节 | 1 cm | ≤ 1.5 cm | 默认 |
| 躯干 `waist_Link_3`、头 `head_Link_2` | 1.5 cm | ≤ 2 cm | |
| 腰板 `waist_Link_1_prismatic` | 2 cm | ≤ 1.5 cm | |
| 底盘 `base_link` | 3 cm（实测最差 5.6 cm，贴地部分被压小） | ≤ 3.4 cm | 最多 40 颗；每颗球底离地 ≥ 3 cm |

`base_link` 球多，是因为它是 68 × 55 cm 的底盘加 1.47 m 高的立柱，立柱截面只有 18 × 25 cm，球大不了，只能排一长串。

**底盘附件移出碰撞模型。** 四个轮子、四个转向叉、两个悬挂、八个脚轮和 `XJ_Link` 共 19 个连杆。它们的关节不在 cuRobo 的 29 个规划关节里，规划时和底盘是一体的，外形也被 `base_link` 的球盖住了。R1 Pro 的轮子也没有碰撞球。

**忽略列表重建。** 只保留：

1. URDF 里直接父子相连的连杆；
2. 刚体组内部：按 URDF 关节类型自动分组（固定关节，或 cuRobo 锁住的手指、底盘 z/rx/ry 关节）。左右手腕各 7 个连杆（`Link7`、`TCP_Link`、`Link8`、腕部、`Arm_*9_Link`、两根手指）；
3. 实测始终贴合的两类：`Link5`↔`Link7`（球只胖 1.5 cm 时仍在所有姿态下重叠，最深 8.5 cm，两节手腕外壳本来就嵌在一起），`waist_Link_1_prismatic`↔`waist_Link_3`（所有姿态下重叠，最深 0.9 cm，在虚胖误差内）；
4. 在关节限位内随机采样 1 万个姿态（四分之一推到限位两端），球面间距始终大于 5 cm 的连杆对（MoveIt 配置助手 “never in collision” 的同一做法）。

撤掉的忽略：底盘↔`waist_Link_2`/`waist_Link_3`、`waist_Link_3`↔`Link2_L`/`Link2_R`。它们在新球下复位不重叠，恢复检查。左右手臂之间只有 `Link1`–`Link4` 互相忽略（中间隔着躯干），前臂、手腕、夹爪之间全部检查。

**连杆对和球对。** 忽略列表按连杆对写；cuRobo 快速自身碰撞算法的上限 16384 按球对算，一对连杆的球对数是两边球数相乘。`base_link` 40 颗球，和一个 10 颗球的手臂连杆就是 400 个球对。cuRobo 读忽略列表时按连杆逐个读，最后做 `min(d, dᵀ)`，所以写在一边即可。

**写回后的离线检查（`check_collision_spheres.py --ignore current --samples 2000`）：**

```text
spheres: 215   checked sphere pairs: 15904   fast-kernel limit: 16384
reset floor clearance: lowest sphere bottom 3.0 cm (want >= 3)
reset overlaps: 0
random samples: 2000; link pairs overlapping in >= 90% of them: none
```

34 个碰撞连杆（32 个机器人连杆 + 2 个抓取物体占位），154 条忽略。改前的配置备份在 `models/gento_skye/misc/curobo-backup-20261021/before-apply-222039/`。

注意：15904 离上限只差 3%。以后再给手臂或躯干加球，会超出快速算法上限，cuRobo 会退到较慢的算法，结果仍然正确。

下一次运行再跑 `GENTO_PLAN_DIAG=1`，期望 `含自身碰撞的总判定: False`，两次逆解都有解，`reach_ok` 为真。

#### 第四轮在 Isaac 里的结果（14:41 那次运行）：bring-up 全部通过

`run-20261021T144134.541951Z-09c1c68c`：`program_ok` 为真，147 步。

```text
与场景碰撞: False；含自身碰撞的总判定: False
  Link2_L <-> waist_Link_3  重叠 0.0 cm
  Link2_R <-> waist_Link_3  重叠 0.0 cm
正运动学 两只手: 0.0 cm
左手逆运动学（原地）: 有解
左手逆运动学（上抬 5 cm）: 有解
```

bring-up 程序的六项检查全部为真：三个相机、两只夹爪张开、腰部升降、底盘转向、`reach_ok`。`move_to_pose` 执行 29 步，末端误差约 7.7 mm，左腕相机画面随抬手整体下移。

肩部和躯干的球在复位时刚好相切（深度不到 0.05 mm），cuRobo 判为不碰撞。cuRobo 日志里有 `using slower kernel. Number of spheres: 279`，和预计一致；一次规划 `provider.curobo` 3.05 s。

按输出决定改哪里：

- 碰的是家具：改 `_gento_base_pose()` 的 `dx, dy` 让底盘后退，或改复位手臂角让手收回来。
- 自身重叠：改 `RESET_JOINT_POSITIONS` 里对应关节。
- 关节贴限位：把该关节的复位值往中间移。
- 正运动学相差超过 1 cm：cuRobo 配置和机器人模型对不上，先查 cuRobo 的 yaml，不要改任务程序。

### 12:23 那次停住窗口后的结论

- 头相机修好了。Viewport 1 能看到壁炉、电视和地板，画面方向正常。Viewport 2、3、6 是双目和腰部相机，ASPIRE 不用，仍然是黑的。
- 末端轴是对的。两只手的 `GENTO_EEF` 都显示：两根手指中心在末端坐标里 z = +0.122，y = ±0.016。Z 指向指尖，Y 横跨两指，不用重新导入。末端原点在手指中心后方约 12 cm。
- 夹爪的开合方向原来是反的。URDF 里 `Arm_L10_Joint` 的轴是 `0 0 -1`，`Arm_L11_Joint` 是 `0 0 1`：关节值从 0 升到 0.04 时，两指向中间靠拢。上面 y = ±0.016 正是 0.04 时的位置；0 时两指中心相距约 11 cm。ASPIRE 原来按 R1 Pro 的习惯把上限当成张开，所以 `open_gripper` 实际是在闭合，腕相机里两指几乎贴在一起。

夹爪方向的修改：

- `config.py` 的机器人配置加 `"grasping_direction": "upper"`。这是 OmniGibson 自带的参数，表示上限是闭合，辅助抓取据此判断“正在抓”。
- `adapter.py` 的 `_read_layout` 读到 `upper` 时互换张开和闭合，公开接口里 0 = 闭合、1 = 张开的约定不变。
- `codec.py` 的布局校验原来要求张开值大于闭合值，改成只要求两者不相等。
- 复位数组最后四个手指关节从 0.04 改成 0（张开），`config.py` 和 `gento_skye.yaml` 同步。
- 测试 `test_upper_grasping_direction_treats_the_lower_finger_limit_as_open` 锁住这个行为。

`move_to_pose` 仍报当前姿态被判为碰撞。cuRobo 配置本来就忽略同一只手两指之间的碰撞，所以不是夹爪闭合造成的。复位时两臂向前平伸，右手末端靠近壁炉，要继续看是手臂碰到躯干、两臂互碰，还是碰到了家具。

（后续结论：不是家具，也不是手臂角，是 cuRobo 碰撞球太胖，见上面 13:07 起的诊断和第四轮。）

## 12. 不再阻止启动、但仍需继续调整的问题

下面问题会影响任务质量，但不应使 `policy.py` 保持 0 步：

1. `_gento_base_pose()` 的偏航减 90° 已经生效，复位能走完。`dx, dy, dz` 仍是 0，机器人还在 R1 的采样点；IK 失败不是这个位置造成的，真的穿家具时再改平移；
2. 头相机和末端轴已经在窗口里核对过；
3. 当前手臂家位置只是带入的参考姿态。肩部和躯干的碰撞球在复位时刚好相切，没有余量；
4. 自动末端射线推断仍会 warning，当前依赖 yaml 显式点；
5. cuRobo 自身碰撞走慢速算法（含抓取预留球 30304 个球对），一次规划约 3 s；立柱后方碰撞模型比实物胖约 15 cm。见 [gento-skye-collision-spheres.md](gento-skye-collision-spheres.md) 第 8、14 节；
6. 腕部相机外参是导入时估计值；
7. SAM3 和 Contact-GraspNet 服务尚未启动，bring-up 不需要，第三层执行
   `segment_text` / `generate_grasps` 时才需要；
8. 右手只做过离线检查（碰撞球、射线点换算），还没有在仿真里实际用右手规划或抓取过。

排查时先确认 `steps_executed > 0`，再调整这些任务效果问题。不要在复位仍失败时
同时修改底盘、手臂、射线和策略，否则无法判断是哪一项产生了新结果。
