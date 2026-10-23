# Stage 1 结束后如何使用这次训练的内容

这次训练是 `turning_on_radio` 在 Gento Skye 上的 ASPIRE Stage 1。协议原文在 `.claude/skills/iterative-debugging/behavior/`（`SKILL.md`、`main-agent-prompt.md`、`subagent-prompt.md`）。通宵改了哪些东西，见 [gento-skye-overnight-changes.md](gento-skye-overnight-changes.md)。

## 1. 先理解三样东西

**policy（`policy.py`）** 是某一个 seed 上的一段 Python 程序。它通过公开 API 指挥机器人：搜索、靠近、重测、抓取、抬起。它是一块一块长出来的：每次加一块或改一块，从头重跑整个程序，看机器人实际做了什么，再写下一块。它和具体场景绑得很紧，比如阈值和搜索顺序都是在那个 seed 上调出来的。**协议把它当一次性的东西**。

**技能库（`behavior/skill-library/*.md`）** 是写给 agent 读的经验手册，不是代码。每一条的结构是：什么情况下触发、实际跑过的做法或代码片段、在哪个任务的哪个 seed 第几次测到的。它记录的是能跨 seed 迁移的知识，比如「`localize_object` 低分也返回 ok」「Gento 末端原点在手指后 12 cm」「桌面抓取先降躯干」。

**两者的关系**：技能库是 agent 写 policy 时的参考书。Stage 1 里，agent 在每个学习 seed 上写 policy，再把有效的东西沉淀进技能库。Stage 2 里，一个全新的 agent 只拿着技能库，从空 policy 开始，在没见过的 seed 上现场写出 policy。**ASPIRE 最终交付的是技能库，不是 policy。** Stage 2 的成绩衡量的就是这本手册够不够用。

**为什么要冻结**：评估要回答的是「这本手册能不能让新 agent 解决没见过的场景」。如果评估过程中继续改手册，比如看了 seed 5 的失败再补一条，手册就是照着考题改的，分数会虚高，也说明不了泛化能力。所以评估前拍一个只读快照，并记下 sha256，每个评估 seed 开跑前都核对一遍。

冻结之后技能库仍然有用，用法有三种：

1. **评估时用**：每个评估 seed 的 agent 读冻结快照来写 policy。
2. **共享库继续增长**：冻结的只是快照（`turning_on_radio/skill-library-frozen/`）。共享的 `skill-library/` 仍然可写，下一个任务（比如 `picking_up_trash`）的 Stage 1 在它的基础上继续积累。
3. **跨任务、跨轮次沉淀**：两次完整运行之间，把经过验证的条目提升到 `.claude/skills/iterative-debugging/behavior/skills/`，成为以后所有运行的起点。

冻结范围：技能库、协议 prompt、harness 配置、机器人模型都不再改。policy 不冻结，每个评估 seed 自己从零写。

## 2. Stage 1 收尾（seed 35 之后）

**1. 检查台账**：`outputs/runs/gento-skye-radio/behavior/turning_on_radio/campaign-state.md` 里，26–35 每个 seed 都应该有一行。失败的 seed 也是证据，照实记。

**2. 把 harness 和模型定稿并提交**。通宵的 harness 修复和 cuRobo yaml 改动还没有 commit：

```bash
cd /root/gpufree-data/aspire/ASPIRE
git status --short
git diff --stat
git add src/cap_harness/behavior/planning.py gento_skye/robot_assets/gento_skye/curobo/*.yaml docs/
git commit -m "Gento Skye: harness IK/exclusion fixes and cuRobo self-collision tuning from Stage 1"
```

运行时读的是 `behavior-data/omnigibson-robot-assets/models/gento_skye/curobo/` 里的 yaml，它和仓库副本要保持一致：

```bash
for f in default arm arm_no_torso; do
  cmp behavior-data/omnigibson-robot-assets/models/gento_skye/curobo/gento_skye_description_curobo_$f.yaml \
      ASPIRE/gento_skye/robot_assets/gento_skye/curobo/gento_skye_description_curobo_$f.yaml
done
```

**3. 用最终的 harness 和模型重跑每个成功 seed 的最终 policy**（推荐）。在 seed 目录里多跑一次，不要改 `policy.py`：

```bash
cd /root/gpufree-data/aspire/ASPIRE
bash outputs/runs/gento-skye-radio/show_attempt.sh \
  outputs/runs/gento-skye-radio/behavior/turning_on_radio/stage1/seed_26 26 <下一个尝试号>
```

如果旧的成功在新模型下不再成功，说明冻结前的模型改动有副作用，先查清楚再冻结。

**4. 整理技能库**：去重；确认每条都写了 `(turning_on_radio, seed N, attempt M)`；只适用于收音机的数字（提手高度、prompt 分数等）要标明，和 harness / 机器人层面的事实分开。

**5. 冻结**：

```bash
cd /root/gpufree-data/aspire/ASPIRE
RUN_ROOT=outputs/runs/gento-skye-radio
CAMP="$RUN_ROOT/behavior/turning_on_radio"
cp -r "$RUN_ROOT/behavior/skill-library" "$CAMP/skill-library-frozen"
chmod -R a-w "$CAMP/skill-library-frozen"
( cd "$CAMP" && find skill-library-frozen -type f -print0 | sort -z \
    | xargs -0 sha256sum > frozen-manifest.sha256 )
```

## 3. Stage 2 评估（seed 1–25）

规则：

- 每个评估 seed 用一个**全新的** agent，起始 policy 为空，只读 `skill-library-frozen/`。
- 只能往后追加代码块，每次尝试只写一块，不能改或删前面的块。默认预算 12 次尝试。
- seed 之间不传递任何信息：不要把一个 seed 的 policy、summary 或视频给另一个 seed 的 agent 看。
- 25 个 seed 全部跑完之前，不要去看 Stage 2 的 policy、summary 或视频。
- 只有在运行本身无效时才能重跑某个 Stage 2 seed，比如启动崩溃、GPU 上有第二个仿真。不能为了刷分重跑。

每个 seed 开跑前：

```bash
cd /root/gpufree-data/aspire/ASPIRE
( cd outputs/runs/gento-skye-radio/behavior/turning_on_radio && sha256sum -c frozen-manifest.sha256 ) \
  || echo "FROZEN LIBRARY CHANGED - STOP"
pgrep -af 'cap-harness run' | grep -v pgrep          # 必须为空
for p in 8114 8115; do curl -s -o /dev/null -w "%{http_code}\n" http://127.0.0.1:$p/openapi.json; done   # 都要 200
```

给 agent 的 prompt：取 `.claude/skills/iterative-debugging/behavior/subagent-prompt.md` 里 `TEMPLATE START` 到 `TEMPLATE END` 之间的内容，变量填 `SUITE=turning_on_radio`、`SEED=<1..25>`、`STAGE=2`、`GPU=0`、`RUN_ROOT=outputs/runs/gento-skye-radio`、`MAX_STEPS=6000`。目录是 `$RUN_ROOT/behavior/turning_on_radio/stage2/seed_NN/`。

跑一次尝试可以用现成脚本。脚本里任务名写死为 `turning_on_radio`，换任务时要改脚本：

```bash
bash outputs/runs/gento-skye-radio/show_attempt.sh \
  outputs/runs/gento-skye-radio/behavior/turning_on_radio/stage2/seed_01 1 1
```

单卡机器只能串行，同一时间只能有一个 Isaac。一次尝试约 6–7 分钟，25 个 seed × 最多 12 次，最坏情况要 30 个小时以上。

## 4. 打分

```bash
cd /root/gpufree-data/aspire/ASPIRE
python3 .claude/skills/iterative-debugging/behavior/score_campaign.py \
  outputs/runs/gento-skye-radio/behavior --stage stage2 --per-seed
```

只报两个数，都按 seed 计：

- **success**：这个 seed 的最终 policy 至少成功过一次。
- **navigation**：这个 seed 的最终 policy 在至少一次运行里既调用过夹爪，又调用过 `move_to_pose` 或 `execute_trajectory`。

「最终 policy」由脚本对每次运行的 `source/program.py` 求 hash 自动识别，不要手数尝试次数。基础设施故障（启动崩溃、显存被占、服务挂了）不计入，但要写明有几次、是什么原因。

## 5. 在自己的测试里复用

| 东西 | 怎么用 | 注意 |
| --- | --- | --- |
| harness 修复（`planning.py`） | 直接保留，和策略无关 | 不修的话 `solve_ik` 每次都报错，抓取时目标物体排除不掉 |
| `TMPDIR` 设置 / `run_attempt.sh` | 所有批量运行都要带上 | 否则跑二三十次就会把系统盘写满 |
| 诊断开关 `GENTO_*_DIAG` | 规划或 IK 失败时临时打开 | 会拖慢运行，正式跑时关掉 |
| cuRobo yaml 改动 | 等 Stage 1 结束确认有效后再定稿 | 有备份可回滚：`behavior-data/.../gento_skye/misc/curobo-backup-buffer-20261021/` |
| 技能库 | 作为写 Gento 抓取程序时的知识库，给 agent 或人看 | 带具体数字的条目只在收音机任务上测过 |
| 某个 seed 的 `policy.py` | 演示、回归测试：在同一个 seed 上重跑，检查 harness 改动有没有改坏 | **不要拿它直接跑评估 seed 并报成绩**，这违反协议。换场景大概率失败，因为阈值是按场景调的 |
| 离线工具 `reach_map.py`、`offline_ik.py` | 不开 Isaac 也能查可达范围和 IK 可行性 | 只用 URDF，不考虑场景碰撞 |

在任意 seed 上手动跑一个 policy（演示或回归用；诊断 seed 用 36–50，不要用 1–25）：

```bash
cd /root/gpufree-data/aspire/ASPIRE
export PATH="$HOME/.local/bin:$PATH" CAP_HARNESS="$PWD/.venv-behavior/bin/cap-harness"
export OMNIGIBSON_DATA_PATH=/root/gpufree-data/aspire/behavior-data
export OMNIGIBSON_APPDATA_PATH=/root/gpufree-data/aspire/behavior-appdata
export OMNIGIBSON_GPU_ID=0 CUDA_VISIBLE_DEVICES=0 OMNIGIBSON_HEADLESS=1
export LD_LIBRARY_PATH="$PWD/.venv-behavior/lib/python3.11/site-packages/nvidia/cuda_nvrtc/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export TMPDIR=/root/gpufree-data/aspire/isaac-tmp && mkdir -p "$TMPDIR"
ulimit -c 0
"$CAP_HARNESS" run --benchmark behavior --suite turning_on_radio --task-id 0 --seed 37 \
  --program outputs/runs/gento-skye-radio/behavior/turning_on_radio/stage1/seed_27/policy.py \
  --output-root outputs/demo --flat-run-dir \
  --max-steps 6000 --camera-width 512 --camera-height 512
find "$TMPDIR" -mindepth 1 -maxdepth 1 -name 'tmp*' -exec rm -rf {} +
```

看结果：`outputs/demo/0037/run-*/outcome.json` 和 `evaluation/protocol.json`。

## 6. 下一个任务（`picking_up_trash`）

- 直接沿用共享的 `behavior/skill-library/`，不要清空。
- 里面带 `turning_on_radio` 标记的条目，对新任务只是假设：harness 和机器人层面的事实通常能迁移，物体相关的数字（高度、宽度、prompt）通常不能。
- 罐子在地上，不在桌上。躯干高度、可达范围、导航到地面物体，都要在新任务的 seed 26–35 上重新测。
- 新任务结束后，在 `picking_up_trash/` 下单独冻结一份快照和 manifest。

## 7. 两次完整运行之间：把经验沉淀成长期技能

两个任务都评估完之后（绝不能在 Stage 2 进行中做），把共享库里经过多个 seed 验证的条目，按主题提升到 `.claude/skills/iterative-debugging/behavior/skills/` 对应的文件里，保留来源标注。之后新开的运行可以从这里起步，不必每次从空库开始重新摸索 harness 层面的事实。
