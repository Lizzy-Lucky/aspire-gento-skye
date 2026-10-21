# 在 Gento Skye 上跑通 ASPIRE（BEHAVIOR 仿真）

文档总入口（全流程、改过的文件、踩过的坑）见 [gento-skye-README.md](gento-skye-README.md)。整条链路分层见 [gento-skye-workflow.md](gento-skye-workflow.md)。复位阶段已经遇到的错误、根因和修复记录在 [gento-skye-troubleshooting.md](gento-skye-troubleshooting.md)。cuRobo 碰撞球的设计和验证见 [gento-skye-collision-spheres.md](gento-skye-collision-spheres.md)。这份说明只做一件事：让 ASPIRE 的写代码 agent 在 Isaac Sim 里指挥 Gento Skye 完成 `turning_on_radio`（捡起红色收音机）。安装、数据集和 URDF 导入已经完成，细节在 [gpufree-behavior-gento-skye.md](gpufree-behavior-gento-skye.md)。

ASPIRE 没有一份按关节维数保存的策略权重。例子 `examples/behavior/turning_on_radio_seed1.py` 是在 R1 Pro 上测过的 Python 程序，里面的末端朝向、手指间距和伸手距离不能原样用在 Gento 上。SAM3 和 Contact-GraspNet 继续用现有权重。要重新准备的是这台机器人的复位姿态、底座位置、末端坐标，以及 cuRobo 能规划。

做到下面四条，执行引擎就算就绪，然后再开 agent。**2026-10-21 14:41 那次运行（`run-20261021T144134.541951Z-09c1c68c`）四条全部通过，第 0–4 步已完成，下一步是第 5 步。**

1. 身体朝上，头相机能看到房间。底盘仍站在 R1 Pro 的采样点上，`dx, dy, dz` 仍是 0，左手靠近壁炉立柱。
2. 头、左腕、右腕三个相机都有画面。头相机已朝前并前移 1 cm。
3. 两只夹爪能张开，腰能动，底盘能转身，左手抬高 5 cm 成功。抬手曾经一直 `IK Fail`，原因是 cuRobo 的碰撞球，重做过程见 [gento-skye-collision-spheres.md](gento-skye-collision-spheres.md)。
4. 末端 Z 轴从指尖伸出，Y 轴从一根手指指向另一根。12:23 那次停窗已经核对，不用重导。

正式任务再往后。学习用 seed 26–35，评测用 seed 1–25。调试机器人时只用 seed 36，不要碰 1–25。

同一时间只能有一个 Isaac 进程。进程退出码 139 可以忽略，以该次运行目录里的 `outcome.json` 为准。不要在 `.venv-behavior` 里执行 `uv sync`。

## 0. 每次开一个新 shell 先导出这些变量

每次启动的命令集中在 [gento-skye-startup.md](gento-skye-startup.md)。

在 `/root/gpufree-data/aspire/ASPIRE` 下执行。数据路径用这台机器的，不要用技能文档里的 `$HOME/behavior-data`。

```bash
cd /root/gpufree-data/aspire/ASPIRE
export PATH="$HOME/.local/bin:$PATH"
export CAP_HARNESS="$PWD/.venv-behavior/bin/cap-harness"
export OMNIGIBSON_DATA_PATH=/root/gpufree-data/aspire/behavior-data
export OMNIGIBSON_APPDATA_PATH=/root/gpufree-data/aspire/behavior-appdata
export OMNIGIBSON_GPU_ID=0
export CUDA_VISIBLE_DEVICES=0
export LD_LIBRARY_PATH="$PWD/.venv-behavior/lib/python3.11/site-packages/nvidia/cuda_nvrtc/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ulimit -c 0
```

`LD_LIBRARY_PATH` 给 cuRobo 找 PyTorch 自带的 `libnvrtc-builtins.so.12.6`。缺这一行时 `move_to_pose` 会在规划开始前失败。完整说明在 [gento-skye-startup.md](gento-skye-startup.md)。

看姿态时让窗口开着：

```bash
export OMNIGIBSON_HEADLESS=0
```

窗口里确认过姿态之后，后面的批量运行改成：

```bash
export OMNIGIBSON_HEADLESS=1
```

确认感知服务在听。两个都要是 `200`：

```bash
for p in 8114 8115; do
  echo "port $p: $(curl -s -o /dev/null -w '%{http_code}' --max-time 3 http://127.0.0.1:$p/openapi.json)"
done
```

不是 200 就按 [gpufree-behavior-gento-skye.md](gpufree-behavior-gento-skye.md) 里「SAM3 与 Contact-GraspNet」一节把监督进程拉起来。显存不够时先停掉它，看完姿态再开。Isaac 和这两个服务可以同时留在这张 4090 上，但不要再开 PyRoKi 或 HTTP 版 cuRobo。

开跑之前确认没有第二个 Isaac：

```bash
pgrep -af 'cap-harness run' | grep -v pgrep
```

有输出就等它结束，不要再启动一次。

## 1. 复位关节角：顺序已经核对

运行时 `robot.joints` 的顺序不是「6 底座 + 3 腰 + 2 头 + 左臂 7 + 左夹爪 + 右臂 7 + 右夹爪」。左右臂逐关节交错，两个头关节夹在左右第一关节之间。`config.py` 的 `RESET_JOINT_POSITIONS` 和 `gento_skye.yaml` 的 `default_joint_pos` 已经按这个顺序写成同一组数。yaml 里的 `tucked_default_joint_pos` 和 `untucked_default_joint_pos` 是这份列表的别名。

当前手臂角是带入的遥操作参考姿态。手指复位是 0。这台机器人 `grasping_direction` 是 `upper`：关节值 0 是张开，0.04 m 是闭合。不要再把 0.04 写成张开。

再改某一个关节时，两处一起改，并保持下面的下标：

```text
0-5    base_footprint_x y z rx ry rz      保持 0。房间位置不在这里改
6-8    waist_joint_1/2/3                  当前 0.5, 0, 0
9      Head_J1_Joint                      当前 0
10-11  Joint1_L, Joint1_R                 当前 1.57, -1.57
12     Head_J2_Joint                      当前 0
13-14  Joint2_L, Joint2_R                 当前 -1.13, -1.13
15-16  Joint3_L, Joint3_R                 当前 -1.18, 1.18
17-18  Joint4_L, Joint4_R                 当前 -1.57, -1.57
19-20  Joint5_L, Joint5_R                 当前 -0.423, 0.423
21-22  Joint6_L, Joint6_R                 当前 -0.147, -0.147
23-24  Joint7_L, Joint7_R                 当前 0.298, -0.298
25-28  Arm_L10, Arm_L11, Arm_R10, Arm_R11 当前全 0（张开）
```

`adapter.py` 的 `reset` 里仍会打印 `GENTO_JOINTS`。对不上就把数组按打印顺序重排，不要按 URDF 文件里的出现顺序猜。前 6 个数改了也不会改变机器人在房间里的位置。

## 2. 只在适配器里改底座位置

数据集 JSON 里的 `robot_poses.robot` 保持原样。Gento 和 R1 Pro 读的是同一份采样，改文件会把 R1 Pro 的出生点一起改掉。

加载时有两次摆位。`adapter.py` 的 `_load_task_instance` 先摆一次，随后 `env.reset()` 再按写进场景 metadata 的那份重摆一次。只改 `set_position_orientation` 的实参，第二次会把第一次盖掉。要改的是写进 metadata 的那一份。

`_gento_base_pose` 已经在 `adapter.py` 模块顶层，`_load_task_instance` 会改 `available[0]` 再 `write_task_metadata`。第二次复位读的是这份 metadata。函数不要挪进 `for` 循环。现在的函数体是：

```python
def _gento_base_pose(position, orientation):
    """由 R1 Pro 采样得到 Gento 的底座位姿。数据集文件不改。"""
    import math

    px, py, pz = (float(value) for value in position)
    ox, oy, oz, ow = (float(value) for value in orientation)
    yaw = math.atan2(
        2.0 * (ow * oz + ox * oy),
        1.0 - 2.0 * (oy * oy + oz * oz),
    )
    yaw -= math.pi / 2
    # 在窗口里一次只改一个数。dx、dy 是世界坐标米；dz 抬高 base_link 原点。
    dx, dy, dz = 0.0, 0.0, 0.0
    half = yaw / 2.0
    return (
        [px + dx, py + dy, pz + dz],
        [0.0, 0.0, math.sin(half), math.cos(half)],
    )
```

四元数是 xyzw。滚转和俯仰清成 0，再把采样偏航减 90°。`dx, dy, dz` 仍是 0，所以机器人还在 R1 Pro 的采样点上，头朝壁炉。之前的 `IK Fail` 不是壁炉造成的（场景碰撞来自轮子球贴地，自身碰撞来自 `base_link` 的大球），碰撞球重做之后抬手已经成功。以后真的穿家具，再每次大约 0.3 m 改一个数。

用窗口跑第 3 步的程序，seed 用 36。看三件事：

- 轮子接触地面。浮空就减小 `dz`，陷进地板就增大 `dz`。R1 Pro 的 z 几乎是 0。
- 身体的头朝上。清掉滚转和俯仰之后仍然侧躺，说明 `base_link` 的上方向和世界 Z 不一致，要在这只函数里再乘一个固定旋转，不要去转数据集里的四元数。
- 身体在过道里，不穿门框、不穿家具。穿模就改 `dx` 或 `dy`，每次 0.3 m 左右。

窗口里拖动机器人不会保存。每次复位都按这只函数重摆。

## 3. 用一个短程序确认相机、夹爪、腰和规划

把下面的程序存成 `outputs/gento-bringup/policy.py`。它只调用公开接口，用来确认执行引擎，不是任务策略。

```python
import math

report = {"stage": "start"}
obs = get_observation()
report["cameras"] = sorted(obs.cameras)
meta = behavior.get_controller_metadata()
torso = [float(value) for value in meta["torso_positions"]]
lower = [float(value) for value in meta["torso_lower_bounds"]]
upper = [float(value) for value in meta["torso_upper_bounds"]]
report["torso_count"] = len(torso)
report["base"] = [float(value) for value in behavior.get_base_pose()]

opened = open_gripper(arm="primary")
report["primary_open"] = bool(opened.ok)
opened = open_gripper(arm="secondary")
report["secondary_open"] = bool(opened.ok)

target = []
for index, value in enumerate(torso):
    nudged = value + (0.05 if index == 0 else 0.0)
    if nudged < lower[index]:
        nudged = lower[index]
    if nudged > upper[index]:
        nudged = upper[index]
    target.append(nudged)
waist = behavior.move_torso(target)
report["waist_ok"] = bool(waist.ok)
behavior.reset_torso()

x, y, yaw = behavior.get_base_pose()
turn = behavior.navigate_to_pose(x, y, yaw + 0.4, planner="servo")
report["turn_ok"] = bool(turn.ok)
report["moved_yaw"] = float(turn.diagnostics.get("moved_yaw", 0.0))

pose = get_robot_state().end_effector_poses["primary"]
lifted = Pose(
    position=(pose.position[0], pose.position[1], pose.position[2] + 0.05),
    quaternion_wxyz=tuple(float(value) for value in pose.quaternion_wxyz),
    frame=pose.frame,
)
reach = move_to_pose(lifted, arm="primary", max_steps=600)
report["reach_ok"] = bool(reach.ok)
if reach.error is not None:
    print(reach.error)
report["stage"] = "bringup"
result = report
```

`torso_count` 应该是 3。接口说明里写的「四个躯干关节」是 R1 Pro 的，Gento 的腰是 `waist_joint_1/2/3`。传 4 个数会报错。

运行：

```bash
mkdir -p outputs/gento-bringup
"$CAP_HARNESS" run \
  --benchmark behavior --suite turning_on_radio --task-id 0 --seed 36 \
  --program outputs/gento-bringup/policy.py \
  --output-root outputs/gento-bringup --flat-run-dir \
  --max-steps 2000 --camera-width 512 --camera-height 512
```

第一次启动要编译着色器，可能要几分钟。结束后在 `outputs/gento-bringup/0036/` 下面最新的一次运行里看：

- `source/program-result.json`：`cameras` 含 `head`、`left_wrist`、`right_wrist`；`primary_open`、`secondary_open`、`waist_ok`、`turn_ok`、`reach_ok` 都为真。
- `media/videos/head.mp4`、`left_wrist.mp4`、`right_wrist.mp4` 有画面。头相机能看到房间，腕相机能看到夹爪附近。
- `outcome.json` 里 `program_ok` 为真。这一步不要求 `task_success`。

历史：12:57 那次 `reach_ok` 为假（`IK Fail`）。`libnvrtc-builtins.so.12.6` 那条用 `LD_LIBRARY_PATH` 消掉；剩下的 `IK Fail` 来自 cuRobo 的碰撞球（`base_link` 一颗半径 0.735 m 的大球、轮子球贴地）。前三轮用忽略列表绕开重叠的做法不对，第四轮按网格重新拟合了碰撞球，见 [gento-skye-collision-spheres.md](gento-skye-collision-spheres.md)。14:41 那次六项全部为真：`move_to_pose` 执行 29 步，末端误差约 7.7 mm。每次运行的原始输出见 [gento-skye-troubleshooting.md](gento-skye-troubleshooting.md) 第 11 节。

以后规划失败时，加 `GENTO_PLAN_DIAG=1` 再跑，看终端里的 `GENTO_PLAN` 诊断。

`Arm_R11_Link` 的碰撞球在局部坐标里 y 约 1.95 m，这是因为 `Arm_R11_Joint` 平移 1.793 m 后翻转了 180°，换算回去和左手对称，不用改。同一只手两指之间的碰撞本来就被忽略。

缺某个相机时，适配器会在复位阶段报 `robot has no camera on link ...`。三个连杆名已经写在 `adapter.py` 的 `CAMERA_LINKS` 里：`head_depth_camera_Link`、`left_wrist_camera_Link`、`right_wrist_camera_Link`。腕相机是导入时加在 `Link8_L` / `Link8_R` 上的，位置是估算的。抓取从头部相机开始，腕相机只用来在近处确认，估算的安装位置不挡住这一步。

## 4. 核对末端坐标

这一步在 12:23 那次 `GENTO_HOLD` 里已经核对过。两只手的手指中心在末端坐标里 z = +0.122，y = ±0.016。Z 指向指尖，Y 横跨两指，末端原点在手指中心后方约 12 cm。不用为了当前的 `IK Fail` 重新导入。

公开接口约定：抓取位姿的 +Z 是接近方向，从指尖向外；Y 从一侧手指指向另一侧。导入配置 `gento-skye-import/gento_skye_source_config.yaml` 里，`left_eef_link` 和 `right_eef_link` 相对 `TCP_Link_L`、`TCP_Link_R` 的旋转是单位四元数。

以后轴被改乱时，再改这份配置里对应的 `orientation`（xyzw），然后按 [gpufree-behavior-gento-skye.md](gpufree-behavior-gento-skye.md) 里的导入命令重新导入。重导会重写 `behavior-data/omnigibson-robot-assets/models/gento_skye/`，导入前先把第 1 步的 `gento_skye.yaml` 复位角抄出来，导入后再写回去，并再跑一次第 3 步。

## 5. 让 agent 在这台机器人上写程序

第 3 步已于 14:41 全部通过，可以进入这一步。开始前：另开终端启动 SAM3 和 Contact-GraspNet（第 0 步最后的端口检查要两个都是 200）；在运行任务的终端执行 `unset GENTO_PLAN_DIAG GENTO_HOLD`，免得每次多花一分钟做诊断或停在窗口。agent 读的是这次运行的视频、`outcome.json` 和程序自己的 `report`，然后改下一块代码。流程以技能文档为准：

`.claude/skills/iterative-debugging/behavior/SKILL.md`

先用诊断 seed 把环跑通，再进入正式学习。诊断 seed 的结果不要写进评测结论。

```bash
export SUITE=turning_on_radio
export SEED=36
export STAGE=1
export GPU=0
export MAX_STEPS=6000
export RUN_ROOT=outputs/runs/gento-skye-radio
SEED_DIR="$RUN_ROOT/behavior/$SUITE/stage$STAGE/seed_$(printf '%02d' "$SEED")"
mkdir -p "$SEED_DIR/attempts" "$RUN_ROOT/behavior/skill-library"
```

`$SEED_DIR/policy.py` 从空策略开始，一次只加一块。第一块只让头相机找收音机，并记下有没有找到。不要把 `examples/behavior/turning_on_radio_seed1.py` 贴进去。那个文件在 seed 1–3 上测过，那些 seed 属于评测分区；它里面的 0.8 m 伸手距离、12.7 cm 手指间距和复位四元数是 R1 Pro 的测量。

第一块可以是：

```python
import math

report = {"stage": "start"}

# Code block 1: 从复位姿态看头部相机里有没有红色收音机。
masks = segment_text("head", "red radio")
report["stage"] = "search"
report["found"] = bool(masks.ok and masks.segmentations)
print("masks", len(masks.segmentations) if masks.ok else 0)

result = report
```

运行命令（和技能文档相同，环境变量用第 0 步的）：

```bash
A=$(printf 'attempt_%03d' $(ls "$SEED_DIR/attempts" | grep -c . || true))
mkdir -p "$SEED_DIR/attempts/$A"
"$CAP_HARNESS" run --benchmark behavior --suite "$SUITE" --task-id 0 --seed "$SEED" \
  --program "$SEED_DIR/policy.py" \
  --output-root "$SEED_DIR/attempts/$A" --flat-run-dir \
  --max-steps "$MAX_STEPS" --camera-width 512 --camera-height 512 \
  > "$SEED_DIR/attempts/$A.log" 2>&1
```

看这次运行的 `source/program-result.json` 和 `media/videos/head.mp4`。`found` 为假是正常的：机器人可能在另一个房间，头还没转向目标。下一块再根据画面决定是转身还是开走，用 `behavior.navigate_to_pose`。底座目标用 `behavior.base_pose_is_free` 先测，再导航。

往后每一块都插在 `result = report` 上面，整份 `policy.py` 从头再跑。规划失败、够不到、抓住又掉，都记在 `$RUN_ROOT/behavior/skill-library/` 里，写明是在 `turning_on_radio` 的哪一次 seed 上测到的。距离和角度在 Gento 上重新量，不要从 R1 Pro 的例子里抄。

`move_to_pose` 和 `plan_motion` 的 `max_steps` 要给到 600–900。默认 120 只够放下很短的轨迹，正常伸手会直接返回失败，手臂不动。

抓取从 `head` 相机来。Contact-GraspNet 的候选经常从下方或侧面接近，逆运动学会失败。把它当作候选之一，抓取位姿主要用物体点云自己摆：接近方向沿末端 +Z，从上往下。

成功的定义已经在宿主侧写好：夹爪握住目标，并且目标比加载时至少高出 5 mm。程序里读不到这个判定。人看 `evaluation/protocol.json` 里的 `checks.held` 和 `max_clearance_m`。只看到离地高度、没有握住，通常是手臂把物体碰开了。

seed 36 上能稳定完成「看见、走近、抓住、抬起」之后，再按技能文档做正式流程：seed 26 到 35 逐个学习，写共享技能库，冻结，然后才用 seed 1–25 评测。评测种子每个都从空的 `policy.py` 开始，只读冻结后的技能库。

## 这一阶段先不要做的事

- 不要改 `2026-challenge-task-instances` 里的 `robot_poses`。
- 不要把 `examples/behavior/` 里的程序当作 Gento 的策略运行或抄进评测。
- 不要为 Gento 重训 SAM3 或 Contact-GraspNet。
- 不要在 Isaac 窗口里摆好机器人然后保存 USD。下次复位会覆盖。
- 不要同时开第二个 `cap-harness run`。
