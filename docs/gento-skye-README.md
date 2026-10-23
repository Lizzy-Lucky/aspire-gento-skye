# Gento Skye × ASPIRE（BEHAVIOR 仿真）文档总入口

把自己的机器人 Gento Skye 接进 ASPIRE 的 BEHAVIOR 仿真，让写代码的 agent 指挥它完成 `turning_on_radio`（捡起红色收音机）。这份文档是总入口：先看这里，再按链接去看细节。

最后更新：2026-10-21。当前进度：**第二层 bring-up 全部通过（14:41），下一步进入第三层，让 agent 写任务代码。**

## 1. 文档列表和阅读顺序

| 顺序 | 文档 | 什么时候看 |
| --- | --- | --- |
| 1 | 本文 | 先看，了解全流程、改过什么、踩过哪些坑 |
| 2 | [gento-skye-workflow.md](gento-skye-workflow.md) | 整件事分三层，现在在哪一层 |
| 3 | [gento-skye-startup.md](gento-skye-startup.md) | 每次新开终端、每次运行之前 |
| 4 | [gento-skye-aspire.md](gento-skye-aspire.md) | 按步骤操作：复位角、底盘位置、bring-up 程序、末端轴、让 agent 写代码 |
| 5 | [gento-skye-troubleshooting.md](gento-skye-troubleshooting.md) | 运行出错、窗口关掉、`steps_executed` 为 0、规划失败；每次运行的原始记录 |
| 6 | [gento-skye-collision-spheres.md](gento-skye-collision-spheres.md) | 改 cuRobo 碰撞球、忽略列表，或者想换到 MoveIt |
| 7 | [gpufree-behavior-gento-skye.md](gpufree-behavior-gento-skye.md) | 重装环境、重下数据、重新导入机器人 |
| 8 | [gento-skye-assisted-grasp.md](gento-skye-assisted-grasp.md) | 怎样算抓住了收音机；辅助抓取射线怎么定义、怎么用 |

## 2. 全流程

按实际做过的顺序列出来。“记录在”是这一步的详细说明所在。

| # | 步骤 | 结果 | 记录在 |
| --- | --- | --- | --- |
| 1 | 机器、CUDA、uv；装 `.venv-behavior`（Isaac Sim 5.1.0、OmniGibson 3.9.2） | 完成 | gpufree-behavior 文档「机器」「BEHAVIOR 运行时」 |
| 2 | pyarrow 降到 21（Isaac 5.1 锁 NumPy 1.26） | 完成，`bootstrap_behavior.sh` 已加约束 | gpufree-behavior 文档「BEHAVIOR 运行时」 |
| 3 | 下载数据集（ModelScope，本机下不了 Hugging Face） | 完成 | gpufree-behavior 文档「数据集」 |
| 4 | 装 SAM3（8114）和 Contact-GraspNet（8115） | 完成，任务阶段才启动 | gpufree-behavior 文档「SAM3 与 Contact-GraspNet」；startup 第 3 节 |
| 5 | 导入 Gento Skye URDF（绕开 pymeshlab） | 完成 | gpufree-behavior 文档「Gento Skye 导入」 |
| 6 | `registry.py` 把 BEHAVIOR 任务的机器人从 `r1pro` 改成 `gento_skye`；适配器的相机连杆改成 Gento 的 | 完成 | 本文第 3 节 |
| 7 | 每次运行前的环境变量 | 固定下来 | startup 第 1 节 |
| 8 | 底盘出生位姿：沿用 R1 Pro 的采样，偏航减 90°，数据集不改 | 完成；`dx, dy, dz` 仍是 0 | aspire 第 2 步；troubleshooting 第 4 节 |
| 9 | 复位阶段的四个错误：辅助抓取射线点、预采样位姿、camera 控制器、29 维关节顺序 | 全部修复 | troubleshooting 第 3–6 节 |
| 10 | 窗口停住检查（`GENTO_HOLD`）；`input()` 卡死窗口 → 改成边渲染边等 | 完成 | troubleshooting 第 7 节 |
| 11 | 第一次进入策略：头相机全黑 → USD 里转 180° 并前移 1 cm | 完成 | troubleshooting 第 11 节 |
| 12 | cuRobo 找不到 `libnvrtc-builtins.so.12.6` → `LD_LIBRARY_PATH` | 完成 | startup 第 1 节；troubleshooting 第 11 节 |
| 13 | 停窗核对末端轴：Z 指向指尖，Y 横跨两指 | 通过，不用重导 | aspire 第 4 步 |
| 14 | 夹爪开合方向反了 → `grasping_direction: upper`，复位手指 0 | 完成 | troubleshooting 第 11 节 |
| 15 | 右手 `Arm_R11_Link` 局部坐标是翻转的（不是错误）；右手射线终点按翻转换算 | 完成 | troubleshooting 第 3 节 |
| 16 | 写 cuRobo 诊断（`GENTO_PLAN_DIAG`） | 完成 | troubleshooting 第 11 节；collision-spheres 第 3.2 节 |
| 17 | 碰撞球四轮：大外接球 → 用忽略绕过（不对）→ 按网格重新拟合、忽略按数据重建 | 完成 | collision-spheres 全文 |
| 18 | bring-up 全部通过：相机、夹爪、腰、转身、左手抬 5 cm | 通过（14:41） | troubleshooting 第 11 节末；collision-spheres 第 1 节 |
| 19 | 导出 MoveIt 用的 URDF/SRDF | 完成 | collision-spheres 第 12 节 |
| 20 | 第三层：agent 在 seed 36 上从空 `policy.py` 写起 | **下一步** | aspire 第 5 步；workflow 第三层 |

## 3. 改过的文件

### 3.1 ASPIRE 代码（`/root/gpufree-data/aspire/ASPIRE`，均未提交）

| 文件 | 改了什么 | 为什么 |
| --- | --- | --- |
| `scripts/bootstrap_behavior.sh` | 约束加 `pyarrow>=21.0.0,<22` | pyarrow 22+ 要求 NumPy 2 |
| `src/cap_harness/providers/sam3/service.py` | 发现本地 `sam3.pt` 时设 `HF_HUB_OFFLINE=1` | 本机连不上 Hugging Face |
| `src/cap_harness/behavior/registry.py` | `robots = ("gento_skye",)` | BEHAVIOR 任务加载 Gento 而不是 R1 Pro |
| `src/cap_harness/behavior/config.py` | `RESET_JOINT_POSITIONS` 按运行时关节顺序重排、手指 0；`use_presampled_robot_pose: False`；`grasping_mode: assisted`；`grasping_direction: upper`；`camera` 控制器改成绝对位置 | troubleshooting 第 4–6、11 节 |
| `src/cap_harness/behavior/adapter.py` | `CAMERA_LINKS` 改成 Gento 的三个相机；`_gento_base_pose`；`_read_task_instance_state`；`robot_poses` 写入通用 `robot` 键；打印 `GENTO_JOINTS`；`GENTO_HOLD` 停窗（持续渲染 + 打印 `GENTO_EEF`）；`GENTO_PLAN_DIAG` 诊断；`_read_layout` 按 `grasping_direction` 互换张开和闭合 | troubleshooting 第 4、7、11 节 |
| `src/cap_harness/behavior/codec.py` | 夹爪布局校验从“张开 > 闭合”改成“两者不相等” | Gento 的张开值比闭合值小 |
| `src/cap_harness/behavior/planning.py` | 新增 `diagnose_current_configuration()`（碰撞、自身重叠、限位、正运动学、逆解自检） | troubleshooting 第 11 节 |
| `tests/behavior_fakes.py` | 假机器人的相机名换成 Gento 的 | 跟 `CAMERA_LINKS` 一致 |
| `tests/test_behavior_adapter.py` | 新增夹爪方向互换的测试 | |
| `tests/test_behavior_pickup_witness.py` | 新增配置检查：不要求预采样位姿、camera 是绝对控制 | |

behavior 相关测试 67 个全部通过：`.venv-behavior/bin/python -m pytest $(ls tests/test_behavior*.py) -q`。

### 3.2 机器人资产（`/root/gpufree-data/aspire/behavior-data/omnigibson-robot-assets/models/gento_skye/`）

| 文件 | 改了什么 |
| --- | --- |
| `gento_skye.yaml` | `default_joint_pos` 与 `config.py` 一致（29 个数，运行时顺序）；`manipulation` 下加 `assisted_grasp_start_points` / `assisted_grasp_end_points`（左右手，右手终点按 `Arm_R11_Joint` 翻转换算） |
| `usd/gento_skye.usda` | `head_depth_camera_Link/Camera`：`xformOp:orient` 改成 `(0, 0, 1, 0)`，`xformOp:translate` 改成 `(0, 0, 0.01)` |
| `curobo/*.yaml`（4 份） | 碰撞连杆去掉 19 个底盘附件；每个连杆换成拟合的球（共 215 颗）；`self_collision_ignore` 重建（154 条） |
| `misc/curobo-backup-20261021/` | 备份：导入原版、第一轮手改后、第四轮写回前 |

数据集 `2026-challenge-task-instances/` 里的 `robot_poses` **没有改**。

### 3.3 导入工作目录（`/root/gpufree-data/aspire/gento-skye-import/`）

| 文件 | 说明 |
| --- | --- |
| `gento_skye_source_config.yaml` | 头相机偏移（绕 Y 转 180°、前移 1 cm）；`curobo.collision_spheres` 和 `self_collision_ignore` 与 cuRobo 配置同步；注释说明原因 |
| `fit_collision_spheres.py` | 按网格拟合碰撞球 |
| `check_collision_spheres.py` | 离线检查碰撞球（不需要 GPU） |
| `apply_collision_spheres.py` | 写回 cuRobo 配置和导入配置 |
| `export_moveit.py` | 导出 MoveIt 用的 URDF/SRDF |
| `fitted_collision_spheres.yaml`、`never_close_pairs.yaml` | 拟合和采样结果 |
| `moveit/` | 导出结果 |

### 3.4 备份进 git 的副本

`ASPIRE/gento_skye/` 存放 3.2、3.3 节以及 `gento_skye_description/` 的副本（USD 和数据集除外），和代码、文档一起提交在 `gento-skye` 分支上。原位置的文件改过之后，提交前先运行 `bash gento_skye/sync_from_workspace.sh` 同步。换机器时的恢复步骤见 `gento_skye/README.md`。

`/root/gpufree-data/` 在释放 GPUFree 实例时不保存，做镜像时也不带，所以改动要推送到远程仓库才算保住。这台机器直连不了 GitHub（HTTPS 和 SSH 都超时），`github.com` 被全局改写到第三方代理 `github.chenc.dev`；推送时令牌会经过它。Gitee 的 HTTPS 可以直连。

## 4. 重新导入机器人之前和之后

重新导入（`import_custom_robot.py`，命令在 gpufree-behavior 文档）会重写 `usd/gento_skye.usda` 和 4 份 cuRobo 配置。`gento_skye.yaml` 会不会被重写，在导入脚本里没有找到写它的代码，没有核实。所以三类都要备份、核对。

导入前：

```bash
B=/root/gpufree-data/aspire/behavior-data/omnigibson-robot-assets/models/gento_skye
mkdir -p $B/misc/before-reimport-$(date +%m%d-%H%M)
cp -r $B/gento_skye.yaml $B/curobo $B/misc/before-reimport-$(date +%m%d-%H%M)/
```

USD 太大（约 491 MB），不备份；它由导入配置重新生成。

导入后逐项核对：

| 检查 | 方法 | 不对时 |
| --- | --- | --- |
| 头相机朝前、不黑 | 导入配置里 `head_depth_camera_Link` 的偏移已是转 180° + 前移 1 cm；跑 bring-up 看 `head.png` | 按 troubleshooting 第 11 节手改 USD |
| `gento_skye.yaml` 的复位角 | 和 `config.py` 的 `RESET_JOINT_POSITIONS` 逐个比对（29 个数，运行时顺序） | 从备份抄回 |
| `gento_skye.yaml` 的辅助射线点 | `manipulation.assisted_grasp_start_points/end_points` 还在 | 从备份抄回，否则会出 `KeyError: 'left'` |
| cuRobo 碰撞球 | `check_collision_spheres.py --ignore current --samples 2000`：215 颗球、复位零重叠、离地 3 cm | 导入配置的 `curobo` 段已同步，一般不用改；不对就重跑 `apply_collision_spheres.py` |
| 末端轴 | `GENTO_HOLD=1` 停窗看 `GENTO_EEF`：两根手指 z 为正、y 一正一负 | aspire 第 4 步 |
| 关节顺序 | 日志里的 `GENTO_JOINTS` 和 aspire 第 1 步的下标表一致 | 按打印顺序重排两处数组 |

最后跑一次 bring-up，六项检查都要为真。

## 5. 踩过的坑

| # | 现象 | 原因 | 解决 | 详见 |
| --- | --- | --- | --- | --- |
| 1 | 执行 `export` 后终端没有任何反应 | `export` 成功时本来就不输出 | 不是错误。新开终端要重新导出 | startup 第 1 节 |
| 2 | 没有 Isaac 窗口；日志里有 `--no-window` | 适配器在没设置时默认 `OMNIGIBSON_HEADLESS=1` | 启动前在同一终端 `export OMNIGIBSON_HEADLESS=0` | startup 第 1 节 |
| 3 | 进程退出码 139（segfault） | Kit 收尾时崩溃 | 忽略，看 `outcome.json` | startup 第 2 节 |
| 4 | 改适配器后 Python 导入就报错 | 粘贴代码时缩进错了 | 改完先 `python -m py_compile src/cap_harness/behavior/adapter.py` | — |
| 5 | `KeyError: 'left'`，物理步异常 | Gento 没有辅助抓取射线点，自动推断失败 | yaml 写入左右手起止点 | troubleshooting 第 3 节 |
| 6 | `No generic or model-specific presampled robot pose found for gento_skye!` | 任务实例只有 `R1Pro` 键；环境构造时就复位一次，早于适配器写位姿 | `use_presampled_robot_pose: False`，适配器加载 seed 后自己换算 | troubleshooting 第 4 节 |
| 7 | `Controller [camera] should be a JointController ... use_delta_commands=False` | camera 控制器默认增量控制 | 配置里显式给 camera 绝对位置控制器 | troubleshooting 第 5 节 |
| 8 | 头、手臂、升降姿态乱 | 29 维复位数组按“功能分组”写，运行时顺序是左右臂交错 | 按 `GENTO_JOINTS` 打印的顺序重排 | troubleshooting 第 6 节 |
| 9 | 窗口停住了但点不动 | 用 `input()` 等回车，主线程阻塞，界面不刷新 | 等待期间循环 `og.sim.render()` | troubleshooting 第 7 节 |
| 10 | 头相机全黑，深度 6–23 mm | USD 相机沿自身 −Z 看，单位朝向看进外壳 | 绕 Y 转 180° | troubleshooting 第 11 节 |
| 11 | 转向后头相机仍黑，深度处处 1.25 mm | 相机原点在外壳里，正对外壳前表面 | 前移 1 cm | troubleshooting 第 11 节 |
| 12 | `move_to_pose` 报 `failed to open libnvrtc-builtins.so.12.6` | PyTorch 的 nvrtc 找不到同版本 builtins | `LD_LIBRARY_PATH` 指向虚拟环境的 `nvidia/cuda_nvrtc/lib` | startup 第 1 节 |
| 13 | `open_gripper` 后腕相机里两指贴在一起 | Gento 手指关节值越大越闭合，ASPIRE 按 R1 Pro 当成张开 | `grasping_direction: upper`；适配器互换；复位手指 0 | troubleshooting 第 11 节 |
| 14 | 以为右手 `Arm_R11_Link` 网格偏了 2 m | 关节平移 1.793 m 再翻转 180°，换算回去和左手对称 | 不改网格和碰撞球；右手射线终点按翻转换算 | troubleshooting 第 3 节 |
| 15 | 诊断报 `'KinematicModelState' object has no attribute 'ee_position'` | 查了 `third_party/curobo`（新版）而不是运行时加载的版本；两个相似的类字段名不同 | 用 `ee_pos_seq`；先 `print(curobo.__file__)` 再查字段 | troubleshooting 第 11 节 |
| 16 | 诊断一处报错，前面算好的结果也丢了 | 结果攒到最后一次性打印 | 每项单独容错、算完就打印 | troubleshooting 第 11 节 |
| 17 | `move_to_pose` 一直 `IK Fail` | 导入器给每个连杆只生成一颗外接球；`base_link` 那颗半径 0.735 m 包住全身；轮子球贴地 | 按网格重新拟合多球 | collision-spheres 第 3 节 |
| 18 | 用忽略列表去绕开重叠 | 球太胖造成的重叠被当成结构接触；会漏掉真碰撞（底盘↔腰、躯干↔肩） | 撤掉；只忽略有依据的四类 | collision-spheres 第 4、7 节 |
| 19 | cuRobo 自带的球拟合漏覆盖 5–20 cm | 网格是多块零件拼成的开口外壳 | 改用各零件凸包的并集 + 贪心覆盖 | collision-spheres 第 5 节 |
| 20 | 说“球对在快速算法上限以内”，实际超限 | 漏算了每只手 32 颗抓取预留球（`extra_collision_spheres`） | 实际 30304 对，走慢速算法，结果正确，一次规划约 3 s | collision-spheres 第 8 节 |
| 21 | 立柱后方碰撞模型比实物胖约 15 cm | 凹形零件取凸包会把凹处填实；“虚胖”是对凸包量的，看不出来 | 暂不改；需要时用 V-HACD 分解 | collision-spheres 第 5.7 节 |
| 22 | 用脚本改导入配置后注释没了 | `yaml.safe_dump` 不保留注释 | `apply_collision_spheres.py` 自动插回；手改后用 `grep -c "^ *#"` 核对 | collision-spheres 第 13 节 |
| 23 | 导出 MoveIt 时传感器连杆一直“碰撞” | IMU、相机在外壳里，带着原网格又没有忽略条目 | 不在 cuRobo 碰撞列表的连杆去掉碰撞体 | collision-spheres 第 12 节 |
| 24 | 每次运行都留下视频 | 默认录三路相机 mp4 | 需要时加 `--no-videos`；旧运行目录整个删，别只删视频 | startup 第 2 节 |
| 25 | 复位时报 `No space left on device`，`harness_error`，0 步 | OmniGibson 每次运行在 `tempfile.mkdtemp()`（默认 `/tmp`，系统盘只有 30 GB）解出约 1.1 GB 的 USD；Kit 退出时 segfault（139），清理没执行，每跑一次漏 1.1 GB，25 次就把系统盘塞满 | 运行前 `export TMPDIR=/root/gpufree-data/aspire/isaac-tmp`（数据盘），每次运行后清掉该目录下的 `tmp*`；`outputs/runs/gento-skye-radio/run_attempt.sh` 已经这样做。已经塞满时，确认没有 Isaac 在跑，再删 `/tmp/tmp*` 里含 `.usd` 的大目录 | startup 第 1 节 |

## 6. 运行开关速查

| 开关 | 作用 | 什么时候用 |
| --- | --- | --- |
| `OMNIGIBSON_HEADLESS=0` / `1` | 有窗口 / 无窗口 | 看机器人时 0；批量跑 1 |
| `LD_LIBRARY_PATH=...cuda_nvrtc/lib:...` | cuRobo 编译内核 | 每次都要 |
| `GENTO_HOLD=1` | 复位后停住窗口，打印 `GENTO_EEF` 和 `GENTO_PLAN`，在终端按回车继续 | 需要在窗口里看姿态时 |
| `GENTO_PLAN_DIAG=1` | 不停窗，只打印 `GENTO_PLAN` 诊断（多约 1 分钟） | 规划失败时 |
| `--no-videos` | 不录 mp4，关键帧和步骤记录照常 | 不需要看视频时 |

取诊断输出：

```bash
grep "GENTO_PLAN\|GENTO_EEF\|GENTO_JOINTS" $(ls -t /root/gpufree-data/aspire/behavior-appdata/local/logs/Kit/OmniGibson/3.9/kit_*.log | head -1) | sed 's/.*\(GENTO_[A-Z]*\)/\1/'
```

## 7. 版本和官方文档

| 项 | 版本 |
| --- | --- |
| Isaac Sim | 5.1.0.0，Kit 107.3.3（`69cbf6ad`） |
| OmniGibson / BEHAVIOR-1K | 3.9.2，资产 3.9.0 |
| cuRobo（运行时） | StanfordVL/curobo 提交 `78612f4`，在 `.venv-behavior/.../site-packages/curobo/`；`third_party/curobo` 是另一个较新的版本，BEHAVIOR 进程内规划用不到它 |
| Python / torch / numpy | 3.11 / 2.7.0+cu126 / 1.26.4 |

本机和 ASPIRE 锁定的版本一致。Isaac Sim 文档用 5.1.0 版：<https://docs.isaacsim.omniverse.nvidia.com/5.1.0/index.html>；不要看 6.0 或不带版本号的页面。OmniGibson 以仓库 `third_party/BEHAVIOR-1K/OmniGibson/` 的源码为准，官网文档可能比 3.9.2 新。

## 8. 下一步

1. 另开终端启动 SAM3 和 Contact-GraspNet（startup 第 3 节），两个端口都返回 200。
2. `unset GENTO_PLAN_DIAG GENTO_HOLD`。
3. 按 aspire 第 5 步，在 seed 36 上从空 `policy.py` 写第一段：头相机找红色收音机。

还没解决、但不阻塞第三层的事：底盘仍在 R1 Pro 的采样点（`dx, dy, dz` = 0），手臂复位角是带入的参考姿态，自身碰撞走慢速算法，立柱后方碰撞模型偏胖，腕相机外参是估计值。见 troubleshooting 第 12 节和 collision-spheres 第 14 节。
