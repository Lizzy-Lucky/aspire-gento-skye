# turning_on_radio 的 policy：是什么、怎么走、改了什么

本文说明 Gento Skye 上 `turning_on_radio` Stage 1 的策略程序。分析对象是目前最完整的一份：

`outputs/runs/gento-skye-radio/behavior/turning_on_radio/stage1/seed_29/policy.py`

它从诊断 seed 36 的空程序一块一块长出来，经 seed 26、27、28 的修改，到 seed 29 第一次就成功。每次运行的原文留在该次目录的 `source/program.py`。各次改了什么、结果如何，见各 seed 的 `seed-summary.md`。通宵对 harness 和碰撞模型的改动见 [gento-skye-overnight-changes.md](gento-skye-overnight-changes.md)。怎样算抓住见 [gento-skye-assisted-grasp.md](gento-skye-assisted-grasp.md)。

## 1. policy 是什么

policy 是一段任务策略程序，不是运动规划器，也不是训练出来的模型。

它只决定「下一步要什么」：去哪看、认哪个物体、手要伸到哪个位姿、夹空了怎么办。真正算关节轨迹的是 cuRobo。程序通过 `solve_ik`、`plan_motion`、`move_to_pose` 把目标位姿交出去。规划器内部的算法和权重没有改过。

没有做算法训练，也没有做模型训练。文档里的「Stage 1 训练」指的是在 seed 26–35 上试错，把能跨场景用的经验写进 `behavior/skill-library/`。ASPIRE 最终交付的是这本技能库，不是某一份 `policy.py`。某一份程序和当时的场景绑在一起，换一个 seed 往往要再改阈值。

程序里没有用对话式 VLM 来决定下一步。视觉只调用 SAM3：把一张相机图和一句文字（例如 `"red radio"`、`"red strap"`）发给 `segment_text`，它返回掩码和分数。`localize_object` 也是先走 SAM3，再把掩码投影成点云、估计几何。Contact-GraspNet 的服务开着，这份程序没有调用它。抓取姿态是几何算出来的：手指横跨提手长轴，从上往下。

官方成功要同时满足两件事：某只手的辅助关节连着收音机，并且收音机比场景加载时高出至少 10 cm。程序读不到这个判定。它能看到的是夹爪开合宽度 `gripper_positions`。成绩以 `outcome.json` 的 `task_success` 和 `evaluation/protocol.json` 为准。

## 2. 整段怎么走

九块代码按顺序跑。源码里 6b 的函数写在 6a 前面，真正找提手的循环在 6a 之后才执行。某一块没把 `stage` 设到下一块要求的值，后面的块直接跳过。

```mermaid
flowchart TD
  B1[1 头相机看一次] --> B2[2 没看到就转身]
  B2 --> B3[3 对准并测量]
  B3 --> B4[4 开到桌边]
  B4 --> B5[5 降腰并规划预抓]
  B5 --> B6[6 手移到预抓]
  B6 --> B6a[6a 腕相机找机身]
  B6a --> B6b[6b 腕相机找提手]
  B6b --> B7[7 下降并闭合]
  B7 --> B8[8 抬腰]
```

| 块 | 决定的事 | 交给谁执行 | 通过后的 stage |
| --- | --- | --- | --- |
| 1–2 | 头相机找红色收音机；看不到就原地每次转 45°，最多 8 次 | `segment_text`，底盘 `navigate_to_pose(..., planner="servo")` | `localized` |
| 3 | 转到画面中心，裁剪点云，量中心和尺寸 | SAM3 + `estimate_geometry` | `measured` |
| 4 | 开到桌边，再蠕行到约 0.68 m | 先 `planner="curobo"`，失败改 servo | `at_table` |
| 5 | 腰降到 0.25 m，近处重测，挑手臂和俯仰角 | `move_torso`，然后 `solve_ik` + `plan_motion` | `pregrasp_planned` |
| 6 | 手移到预抓位姿；差 3–5 cm 仍算到达 | `move_to_pose` | `at_pregrasp` |
| 6a | 腕相机找机身，看不到就抬高、偏移再看 | `segment_text`，`move_to_pose` | 仍是 `at_pregrasp`，或 `radio_not_found_by_wrist` |
| 6b | 四个文字提示里挑细的、高出桌面的部位，手移到它上方 | `segment_text`，`move_to_pose` | 仍是 `at_pregrasp`，或 `no_handle` / `recenter_failed` |
| 7 | 下降、闭合；夹爪读数 < 0.03 就转 90° 再夹一次 | `move_to_pose`，`close_gripper` | `closed` |
| 8 | 腰升高 0.15 m；腰动不了才改用手臂 | `move_torso`，失败才 `move_to_pose` | `lifted_attempt` |

## 3. 每次迭代改的是策略

相邻两次的差别，是「问规划器要哪个目标」，以及「什么时候相信感知」。规划器还是同一个。改动落在四类：

1. **目标位姿。** 蠕行从 0.62 m 改到 0.68 m，因为再近会把收音机碰起来。腰先试过前倾 0.3 rad，够不着，改成把 `waist_joint_1` 降到 0.25 m。这是在改交给规划器的起点和目标。
2. **认什么、丢什么。** 远处头相机中心常偏 15–45 cm，所以到桌边再测。腕部后几次检测只能微调：跳动超过 6 cm、分数低于 0.3、框贴在画面边上，都丢掉。提手最高点必须高出桌面 9 cm，否则桌上的电源线会被当成提手。
3. **失败后的下一步。** 夹爪读数 < 0.03 就回到预抓位姿，绕竖直轴转 90° 再夹。头相机在桌边看不到时，腕相机当前视角、抬高 12 cm、再在前后左右偏移处看。规划器每次仍只收一个位姿。
4. **执行器的 bug，不是策略。** IK 关节被多 batch 一次、抓取时收音机留在碰撞世界里、手指水平排除余量 5 cm、碰撞球缓冲，改的是 `planning.py` 和 cuRobo yaml。同一份 policy 重跑。这些会改变所有 seed 的规划结果。seed 36、26、27 成功时还没有后来那组碰撞缓冲。

到 seed 29 时，各 seed 的结果：

| seed | 起点 | 尝试 | 结果 |
| --- | --- | --- | --- |
| 36（诊断，不计入成绩） | 空程序 | 24 | 成功 |
| 26 | seed 36 的程序 | 13 | 成功 |
| 27 | seed 26 的程序 | 8 | 成功 |
| 28 | 前面积累的程序 | 14 | 未成功 |
| 29 | seed 28 的程序 | 1 | 第一次就成功 |

## 4. 各块流程图

### 1 头相机看一次

```mermaid
flowchart TD
  A["segment_text 头相机, red radio"] --> B{"分数至少 0.35, 点至少 150, 高度在 0.25 到 1.0 m"}
  B -->|是| C["found = true"]
  B -->|否| D["found = false, 记下分数或高度"]
```

搜索门限是 `search_accept = 0.35`。真目标大约 0.56–0.83。seed 28 里窗外粉色物体是 0.248，用原来的 0.2 会被当成收音机。

### 2 没看到就转身

```mermaid
flowchart TD
  A{"found?"} -->|是| Z[跳过]
  A -->|否| B["原地转 45 度, servo"]
  B --> C["再测一次"]
  C --> D{"合格?"}
  D -->|是| E["停止, localize_object"]
  D -->|否且未满 8 次| B
  D -->|8 次仍没有| F["后面的块不跑"]
  E --> G["stage = localized"]
```

这一块从诊断 seed 就有。`localize_object` 只是再调一次 SAM3 并估计几何，低分也会返回成功，所以真正的门槛在上一块。

### 3 对准并测量

```mermaid
flowchart TD
  A{"found?"} -->|否| Z[跳过]
  A -->|是| B["框中心相对光心算出转角"]
  B --> C["底盘转到正对"]
  C --> D["再分割, 掩码转点云"]
  D --> E["裁掉中心以外 0.25 m"]
  E --> F["得到中心、尺寸、朝向"]
  F --> G["stage = measured"]
```

远处这一下经常偏 15–45 cm，所以到桌边还会重测。裁剪半宽 `crop_half_m = 0.25` 是为了去掉桌子以外的点。

### 4 开到桌边

```mermaid
flowchart TD
  A{"stage 是 measured?"} -->|否| Z[跳过]
  A -->|是| B["分割 coffee table"]
  B --> C{"分数至少 0.2, 且桌边外 0.3 m 能站?"}
  C -->|是| D["目标 = 桌边停靠"]
  C -->|否| E["目标 = 朝收音机 0.6 m"]
  D --> F["先 cuRobo 导航"]
  E --> F
  F --> G{"到了?"}
  G -->|否| H["改 servo 再开"]
  G -->|是| I{"平面距离小于 1 m?"}
  H --> I
  I -->|否| J["stage = drive_short"]
  I -->|是| K{"当前姿态 IK 失败?"}
  K -->|是| L["沿朝向后退, 每次 5 cm, 最多 5 次"]
  K -->|否| M["朝收音机每次 3 cm, 蠕行到 0.68 m"]
  M --> N{"这一步 IK 失败?"}
  N -->|是| O["退回上一步"]
  N -->|否| P["再转到收音机偏左约 15 度"]
  L --> Q["停在这里"]
```

蠕行到 0.62 m 会碰到收音机，改成 `creep_target_m = 0.68`。桌边 cuRobo 判不可通行，所以失败后改用 servo。偏左 `left_bias_rad = 0.26`（约 15°）是为了让左肩更靠近物体。

### 5 降腰并规划预抓

```mermaid
flowchart TD
  A{"stage 是 at_table?"} -->|否| Z[跳过]
  A -->|是| B["降腰前, 头相机再对一次并测量"]
  B --> C{"点少于 1000 或高度小于 10 cm?"}
  C -->|是| D["丢掉这次"]
  C -->|否| E["留下 table_meas"]
  D --> F["腰降到 0.25 m"]
  E --> F
  F --> G["降腰后再测"]
  G --> H{"近处测量比降腰前点更多?"}
  H -->|否| I["改用降腰前的测量"]
  H -->|是| J["用降腰后的中心"]
  I --> K{"和远处中心差超过 15 cm?"}
  J --> K
  K -->|是| L["重新开到新中心前 0.75 m, 再测一次"]
  K -->|否| M["按物体在左或右排手臂"]
  L --> M
  M --> N["俯仰 90, 70, 55, 40 度"]
  N --> O["solve_ik, 通过再 plan_motion"]
  O --> P{"有一条轨迹?"}
  P -->|是| Q["stage = pregrasp_planned"]
  P -->|否| R["腰改 0.38, 0.25, 0.15 m, 竖直向下再试"]
  R --> S{"仍没有?"}
  S -->|是| T["只探针可达范围, 不进仿真"]
  S -->|否| Q
```

这一块改的是交给规划器的起点和目标：

- 腰从复位 0.509 m 降到 `lift_height = 0.25`。复位高度下手竖直向下只能往前约 0.46 m。0.30 m 约 0.68 m，0.15 m 约 0.76 m（离线 URDF，`gento-skye-import/reach_map.py`）。
- 远处中心不可信。降腰后再测；和远处中心差超过 15 cm 就重开到 `redrive_dist_m = 0.75`。0.72 m 时折叠的手臂会碰到 `waist_Link_3`。
- 重开后再测：和重开前的中心差超过 12 cm，或分数低于 0.3，丢掉。
- 四个俯仰角 `tilts_deg = [90, 70, 55, 40]` 和 cuRobo 调用方式基本没动。先试靠近物体那一侧的手臂。成功的几次都是 90°、第一条轨迹就通过。
- 关节路径回退被关掉（源码里 `if False`）。那条路径会偏 15–60 cm：harness 规划时腰是自由的，`solve_ik` 只返回手臂关节。
- 末端原点到指心 `finger_offset_m = 0.122`。预抓在目标沿接近方向再退 `pregrasp_clear_m = 0.14`。指心落在物体顶面下 `grasp_drop_m = 0.02`。

### 6 手移到预抓

```mermaid
flowchart TD
  A{"stage 是 pregrasp_planned?"} -->|否| Z[跳过]
  A -->|是| B["move_to_pose, 最多 900 步"]
  B --> C{"到位, 但误差大于 3 cm?"}
  C -->|是| D["再走一次"]
  C -->|否| E{"成功, 或误差小于 5 cm?"}
  D --> E
  E -->|是| F["stage = at_pregrasp"]
  E -->|否| G["停在 pregrasp_planned, 后面不夹"]
```

差 3–5 cm 仍算到达。seed 26 成功那次，下降停在目标上方 3.3 cm，夹爪已经夹住了。`reach_max_steps = 900`，默认 120 步只够很短的轨迹。

### 6a 腕相机找机身

```mermaid
flowchart TD
  A{"stage 是 at_pregrasp?"} -->|否| Z[跳过]
  A -->|是| B["当前视角分割 red radio"]
  B --> C{"分数至少 0.2 且点至少 300?"}
  C -->|是| D["移到点云中位上方"]
  C -->|否| E["抬高 12 cm 再看"]
  E --> F{"还没有?"}
  F -->|是| G["前后左右各 15 cm, 且抬高 12 cm"]
  F -->|否| D
  G --> H{"找到?"}
  H -->|否| I["stage = radio_not_found_by_wrist"]
  H -->|是| D
  D --> J{"再测, 中心又移过 4 cm?"}
  J -->|是| K["再移一次"]
  J -->|否| L["进入找提手"]
```

这是 seed 27 加上的。那个场景里头相机在桌边分数只有 0.02–0.03，腕相机抬高 12 cm 才看到机身（0.35–0.44）。`search_raise_m = 0.12`，`search_step_m = 0.15`。

### 6b 腕相机找提手

```mermaid
flowchart TD
  A{"仍是 at_pregrasp?"} -->|否| Z[跳过]
  A -->|是| B["四个提示: red strap, black cable, handle, radio handle"]
  B --> C{"高出桌面至少 9 cm, 中间尺寸不超过 16 cm, 框不是整幅画面?"}
  C -->|否| D["丢掉"]
  C -->|是| E["记下候选, 贴边的分数乘 0.7"]
  E --> F["换观察位置: 原地, 抬高 12 cm, 长轴两端, 再前后左右 12 cm"]
  F --> G{"有效分至少 0.6?"}
  G -->|是| H["停止张望"]
  G -->|否且还有位置| F
  H --> I["最多取 3 个, 分高的先试"]
  I --> J["手指横跨长轴, 移到提手上方 12 cm"]
  J --> K{"再测: 分至少 0.3, 不贴边, 跳动在 1.5 到 6 cm?"}
  K -->|是| L["用新中心再对一次"]
  K -->|否| M["保留上一次"]
  L --> N{"对上了?"}
  M --> N
  N -->|否| O["换下一个候选"]
  N -->|是| P["更新抓取位姿, 留给第 7 块"]
  O --> Q{"还有候选?"}
  Q -->|否| R["stage = no_handle 或 recenter_failed"]
```

这一块用来区分「该夹的细部位」和「不该夹的东西」：

- 机身比约 8 cm 的开口宽，夹中心会夹空，所以改夹带子、铁丝提手或边缘。
- 腕部框面积超过画面一半，或宽、高小于 25 像素，直接丢掉。横跨或竖跨画面 90% 以上的框，分数乘 0.3。
- 桌上电源线的 `"black cable"` 也能到约 0.5，但只高出桌面约 6 cm。`handle_min_rise_m = 0.09`。带子大约 +12 cm，提手大约 +22 cm。
- 整台收音机也会被标成 handle（中间尺寸约 0.22 m）。三个尺寸排好后，中间那个不得超过 `handle_max_mid_extent_m = 0.16`。带子约 0.131，提手约 0.07，边缘约 0.046。
- 后一次检测：分数低于 0.3、框贴边、跳动超过 6 cm，或跳动小于 1.5 cm，都保留上一次。避免手漂到电源线上，也避免几乎没动的重测覆盖掉已经对准的点。
- 铁丝提手在长轴一端，所以先看两端（中心 ± 长边的一半，并抬高 12 cm），再看前后左右各 12 cm。有效分达到 0.6 就停止张望。最多试分数最高的 3 个。
- 指心高度取 `max(提手顶 − 0.02, 桌面 + 0.02)`，末端再沿 +Z 退后 0.122 m。预抓在这之上 0.12 m。对位误差小于 3 cm 也算对准。

### 7 下降并闭合

```mermaid
flowchart TD
  A{"stage 是 at_pregrasp 且找到提手?"} -->|否| Z[跳过]
  A -->|是| B["沿接近方向下降, 容差 1 cm"]
  B --> C["闭合夹爪, 读开合宽度"]
  C --> D{"宽度大于 0.03?"}
  D -->|是| E["认为指间有东西"]
  D -->|否| F["张开, 回到预抓, 绕竖直轴转 90 度"]
  F --> G["再下降, 再闭合, 只再试一次"]
  E --> H["stage = closed"]
  G --> H
```

提手轴向从掩码看不准。`empty_close_below = 0.03`：读数接近 0 是指间空的，中间值是夹着东西，接近 1 是张着或顶住了比开口更宽的部位。seed 26 第 13 次第一次朝向读数已经是 0.212，没有用到第二次。seed 27 第 8 次是 0.966，夹到了几乎和开口一样宽的边缘。读数只说明指间有没有东西，不说明辅助关节是否连上收音机。下降停在容差外（`CONTROLLER_FAILED`）仍会闭合。

### 8 抬腰

```mermaid
flowchart TD
  A{"stage 是 closed?"} -->|否| Z[跳过]
  A -->|是| B["waist_joint_1 升高 0.15 m"]
  B --> C{"腰动了?"}
  C -->|是| D["stage = lifted_attempt"]
  C -->|否| E["改用手臂向上 15 cm"]
  E --> F{"手臂一步都没走?"}
  F -->|是| G["solve_ik 再 plan_motion 再执行"]
  F -->|否| D
  G --> D
```

抬起优先用腰，避免再做一次手臂规划。`lift_m = 0.15` 盖得住官方要求的 10 cm。程序写完 `stage = lifted_attempt` 就结束，自己不知道这次算不算成功。
