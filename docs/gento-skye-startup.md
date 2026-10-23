# 每次启动 Gento Skye 仿真

新开一个终端就要做一遍。`export` 只存在于当前这个终端，成功时没有任何输出。关掉终端或新开一个，变量就没了。

文档总入口见 [gento-skye-README.md](gento-skye-README.md)。安装、数据集和机器人导入已经完成，见 [gpufree-behavior-gento-skye.md](gpufree-behavior-gento-skye.md)。当前进度以 [gento-skye-workflow.md](gento-skye-workflow.md) 为准：第二层 bring-up 已于 2026-10-21 14:41 全部通过，下一步是第三层，让 agent 写任务代码。如果运行一直停在 reset、窗口自动关闭或 `steps_executed` 为 0，按 [gento-skye-troubleshooting.md](gento-skye-troubleshooting.md) 排查。

同一时间只能有一个 Isaac。不要在 `.venv-behavior` 里执行 `uv sync`。

## 1. 进入仓库并设置变量

```bash
cd /root/gpufree-data/aspire/ASPIRE
export PATH="$HOME/.local/bin:$PATH"
export CAP_HARNESS="$PWD/.venv-behavior/bin/cap-harness"
export OMNIGIBSON_DATA_PATH=/root/gpufree-data/aspire/behavior-data
export OMNIGIBSON_APPDATA_PATH=/root/gpufree-data/aspire/behavior-appdata
export OMNIGIBSON_GPU_ID=0
export CUDA_VISIBLE_DEVICES=0
export LD_LIBRARY_PATH="$PWD/.venv-behavior/lib/python3.11/site-packages/nvidia/cuda_nvrtc/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export TMPDIR=/root/gpufree-data/aspire/isaac-tmp && mkdir -p "$TMPDIR"
ulimit -c 0
```

`TMPDIR` 这一行必须有：OmniGibson 每次运行都在临时目录里解出约 1.1 GB 的 USD，Kit 退出时崩溃（退出码 139）不会清理，留在 `/tmp` 会在二三十次运行后塞满 30 GB 的系统盘，之后每次复位都报 `No space left on device`。放到数据盘上，并定期删掉 `$TMPDIR/tmp*`（确认没有 Isaac 在跑时）。

`LD_LIBRARY_PATH` 这一行给 cuRobo 用。PyTorch 自带 12.6 版的 `libnvrtc`，编译规划内核时要找同目录下的 `libnvrtc-builtins.so.12.6`；系统路径里只有 CUDA 12.8 的版本。缺这一行时，`move_to_pose` 返回 `planner_failed`，输出里有 `failed to open libnvrtc-builtins.so.12.6`。

要在窗口里看机器人：

```bash
export OMNIGIBSON_HEADLESS=0
```

不看窗口、只看输出目录里的视频时：

```bash
export OMNIGIBSON_HEADLESS=1
```

不设置这个变量时，适配器默认是 `1`（无窗口）。日志里出现 `--no-window` 就是没开窗口。

两个调试开关，平时不要开：

| 开关 | 作用 |
| --- | --- |
| `export GENTO_HOLD=1` | 复位完成后停住窗口（需要 `OMNIGIBSON_HEADLESS=0`）。终端先打印 `GENTO_EEF`（左右末端坐标轴、手指位置）和 `GENTO_PLAN`（cuRobo 诊断），再打印 `GENTO_HOLD`。窗口可以正常操作；看完在启动程序的那个终端按回车，程序继续 |
| `export GENTO_PLAN_DIAG=1` | 不停窗，只打印 `GENTO_PLAN` 诊断：场景碰撞、自身重叠、关节限位、正运动学、左手逆解自检。多花约 1 分钟 |

用完关掉：

```bash
unset GENTO_HOLD GENTO_PLAN_DIAG
```

诊断输出很多，不用翻终端，直接从 Isaac 日志里取：

```bash
grep "GENTO_PLAN\|GENTO_EEF\|GENTO_JOINTS" $(ls -t /root/gpufree-data/aspire/behavior-appdata/local/logs/Kit/OmniGibson/3.9/kit_*.log | head -1) | sed 's/.*\(GENTO_[A-Z]*\)/\1/'
```

确认没有另一个仿真正在跑。没有输出就表示可以启动：

```bash
pgrep -af 'cap-harness run' | grep -v pgrep
```

有输出就等它结束，不要再启动一次。

## 2. 看站姿时跑 bring-up

程序文件是 `outputs/gento-bringup/policy.py`。这条命令不调用分割和抓取，不用先开 8114 和 8115。seed 用 36。

```bash
mkdir -p outputs/gento-bringup
"$CAP_HARNESS" run \
  --benchmark behavior --suite turning_on_radio --task-id 0 --seed 36 \
  --program outputs/gento-bringup/policy.py \
  --output-root outputs/gento-bringup --flat-run-dir \
  --max-steps 2000 --camera-width 512 --camera-height 512
```

第一次启动会编译着色器，可能要几分钟，窗口才会出现。加载过程中终端会持续有日志。

改完下面任一处，都要重新执行上面的 `cap-harness run`。只 `export` 不会让场景里的机器人动。

| 要改的 | 文件 |
| --- | --- |
| 底盘朝向、在房间里的位置 | `src/cap_harness/behavior/adapter.py` 的 `_gento_base_pose` |
| 升降、弯腰、手臂、夹爪 | `src/cap_harness/behavior/config.py` 的 `RESET_JOINT_POSITIONS`，并同步 `gento_skye.yaml` 的 `default_joint_pos` |

跑完后看最新一次目录 `outputs/gento-bringup/0036/run-*/`：

- `outcome.json` 里的 `program_ok`
- `source/program-result.json`
- `media/videos/` 下的 `head.mp4`、`left_wrist.mp4`、`right_wrist.mp4`

进程退出码 139 可以忽略，以 `outcome.json` 为准。`steps_executed` 为 0 表示 `policy.py` 还没开始执行，问题在复位阶段。

每次运行都会新建一个 `run-<时间>-<编号>/` 目录，以前的不会被覆盖或自动删除。一次完整的 bring-up 大约 6 MB，主要是三路相机视频；在复位阶段就失败的运行没有视频，只有几十 KB。不想录视频时，在命令里加 `--no-videos`，关键帧、步骤记录和 `outcome.json` 照常保留。要删旧运行就删整个目录，不要只删里面的视频，否则 `manifest.json` 里列的文件会对不上。

## 3. 需要分割或抓取时再开感知服务

bring-up 不用这一节，第三层（agent 写任务代码）开始要用。agent 开始执行 `segment_text` 或 `generate_grasps` 之前，在另一个终端里启动，并先做第 1 节的 `cd` 和 `export`：

```bash
export HF_HOME=/root/gpufree-data/aspire/hf-home
export HF_HUB_OFFLINE=1
export CAP_HARNESS_PYTHON=/usr/bin/python3
bash scripts/supervise_services.sh --profile rtx4090 --providers sam3,contact_graspnet --ready-timeout 600
```

两个都要是 `200`：

```bash
for p in 8114 8115; do
  echo "port $p: $(curl -s -o /dev/null -w '%{http_code}' --max-time 3 http://127.0.0.1:$p/openapi.json)"
done
```

这两个服务和 Isaac 共用同一张 4090。不要再开 PyRoKi 或 HTTP 版 cuRobo。显存不够时，先停掉这个监督进程，看完站姿再开。
