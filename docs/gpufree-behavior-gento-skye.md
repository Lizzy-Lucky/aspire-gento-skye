# GPUFree 容器：BEHAVIOR-1K 安装与 Gento Skye 导入

文档总入口：[gento-skye-README.md](gento-skye-README.md)。

2026-10-20，在 `gpufree-container` 上完成。只做了 BEHAVIOR-1K / Isaac Sim 这条线。LIBERO-Pro、Robosuite 和真机代码没有动。

## 机器

| 项 | 值 |
| --- | --- |
| 系统 | Ubuntu 22.04.5，glibc 2.35，root |
| GPU | 单卡 RTX 4090 24 GB，驱动 580.126.09，计算能力 8.9 |
| CUDA | `/usr/local/cuda-12.8`，`nvcc` 可用。cuRobo 必须用 CUDA 12.x |
| 系统盘 | overlay，约 30 GB，不要放大数据 |
| 数据盘 | `/root/gpufree-data`，数据集、虚拟环境、网格都在这里 |
| 网络 | 访问不了 `huggingface.co` 和 `hf-mirror.com`。PyPI 走上海交大镜像。ModelScope 和 `storage.googleapis.com` 可以连通 |

同一时间只能有一个 Isaac 进程。无头运行使用 `OMNIGIBSON_HEADLESS=1` 和 `OMNIGIBSON_GPU_ID=0`。

## 环境变量

在 `/root/gpufree-data/aspire/ASPIRE` 下执行：

```bash
export PATH="$HOME/.local/bin:$PATH"
export UV_DEFAULT_INDEX=https://mirror.sjtu.edu.cn/pypi/web/simple
export CAP_HARNESS_BEHAVIOR_DATA=/root/gpufree-data/aspire/behavior-data
export CAP_HARNESS_BEHAVIOR_APPDATA=/root/gpufree-data/aspire/behavior-appdata
export CAP_HARNESS_BEHAVIOR_CACHE=/root/gpufree-data/aspire/uv-cache-behavior
export CAP_HARNESS_CUDA_HOME=/usr/local/cuda-12.8
export OMNIGIBSON_DATA_PATH="$CAP_HARNESS_BEHAVIOR_DATA"
export OMNIGIBSON_APPDATA_PATH="$CAP_HARNESS_BEHAVIOR_APPDATA"
export OMNIGIBSON_HEADLESS=1
export OMNIGIBSON_GPU_ID=0
```

启动仿真前再执行 `ulimit -c 0`，避免 core dump 占满磁盘。如果 shell 里 `python3` 被别名到 `/root/isaacsim/python.sh`，跑仓库脚本时用 `bash scripts/...` 或虚拟环境里的解释器，不要走这个别名。

## BEHAVIOR 运行时

虚拟环境：`/root/gpufree-data/aspire/ASPIRE/.venv-behavior`（约 18 GB）。

| 包 | 版本 |
| --- | --- |
| Python | 3.11.17 |
| torch | 2.7.0+cu126 |
| numpy | 1.26.4 |
| isaacsim | 5.1.0.0 |
| omnigibson | 3.9.2 |
| cap-harness | 0.1.0 |
| cuRobo | 按 RTX 4090 编译，架构 `sm_89`，profile `rtx4090` |

指纹：`.venv-behavior/.cap-fingerprint.json`，值 `082beae4e51d`。再跑 bootstrap 会跳过重装，除非加 `--rebuild`。

Isaac Sim 用的是 BEHAVIOR-1K v3.9.2 锁定的 5.1.0 Python 3.11 wheel，由 `scripts/bootstrap_behavior.sh` 装进 `.venv-behavior`。装好后不要在这个环境里 `uv sync`，否则会卸掉 Isaac wheel。

本地改动：第一次导入 OmniGibson 时，pyarrow 26 要求 NumPy 2，而 Isaac Sim 5.1 把环境锁在 NumPy 1.26.4。已把 pyarrow 降到 21.0.0，并在 `scripts/bootstrap_behavior.sh` 的约束里加了 `pyarrow>=21.0.0,<22`。

## 数据集

本机下不了 Hugging Face。三个包来自 ModelScope 上的同一仓库 `behavior-1k/zipped-datasets`。`omnigibson.key` 来自 Google Cloud Storage，44 字节，已放在数据根目录。用户已用 `--accept-dataset-tos` 接受 BEHAVIOR Data Bundle 条款（非商业学术研究、加密资源、只能在 OmniGibson 内使用、不得再分发）。

下载脚本：`/root/gpufree-data/aspire/behavior-zips/download_from_modelscope.py`  
日志：`/root/gpufree-data/aspire/behavior-zips/download.log`

脚本会断点续传。29.3 GB 的 `behavior-1k-assets-3.9.0.zip` 用 `curl -I -L` 跟随到 CDN 时，HEAD 会卡满 60 秒（curl 退出码 28）。脚本已改成只取 ModelScope 第一跳的 `Location`。这次实际速度约 19.7 MB/s。

解压后的检查路径：

- `behavior-data/omnigibson-robot-assets/models/r1pro/r1pro.yaml`
- `behavior-data/behavior-1k-assets/VERSION`（内容 `3.9.0`）
- `behavior-data/2026-challenge-task-instances/metadata/task.jsonl`
- `behavior-data/omnigibson.key`

确认数据集后再跑：

```bash
bash scripts/bootstrap_behavior.sh --accept-dataset-tos --gpu-id 0
```

2026-10-20 这次运行识别到 RTX 4090，指纹仍是 `082beae4e51d`，跳过了重装，并打印 `datasets present`。日志在 `/root/gpufree-data/aspire/bootstrap-behavior-rerun.log`。

## 结构检查

```bash
source .venv-behavior/bin/activate
ulimit -c 0
cap-harness doctor --environment-root "$VIRTUAL_ENV" \
  --expected-environment-name .venv-behavior --runtime behavior \
  --reset --reset-embodiment behavior --reset-suite turning_on_radio --seed 1 \
  --output validation-artifacts/behavior/doctor.json
```

报告：`validation-artifacts/behavior/doctor.json`。第一次启动会把着色器编译到 `OMNIGIBSON_APPDATA_PATH`，这次 reset 约 7.5 分钟。

环境和仿真相关的 13 项通过，包括数据集、无头 GPU 绑定，以及 `turning_on_radio`（seed 1，task 0，任务语言存在）。当时 `services` 失败，是因为 8114、8115、8116 还没有进程。BEHAVIOR 的运动规划走进程内 cuRobo，不需要 PyRoki（8116）。完整抓取程序需要 SAM3 和 Contact-GraspNet，见下一节。

进程在写出报告之后，于 Isaac 插件卸载阶段段错误退出（exit 139）。这是 Kit 写完结果后常见的退出码。看 `outcome.json` 或 doctor 报告，不要只看进程退出码。

## SAM3 与 Contact-GraspNet

2026-10-20 装好。Python 包走上海交大 PyPI 镜像，SAM3 权重走 ModelScope。两个服务是独立进程，和 Isaac Sim 不是同一个程序，但共用这张 4090。

| 服务 | 虚拟环境 | 端口 | 权重 |
| --- | --- | --- | --- |
| SAM3 | `.venv-sam3`（Python 3.10，torch 2.9.1） | 8114 | `hf-home` 里的 `facebook/sam3`，`sam3.pt` 3,450,062,241 字节 |
| Contact-GraspNet | `.venv-graspnet`（Python 3.10，torch 2.9.1） | 8115 | 仓库里已有的 `third_party/contact_graspnet_pytorch/checkpoints/contact_graspnet/checkpoints/model.pt` |

权重缓存：`/root/.cache/huggingface` 指向 `/root/gpufree-data/aspire/hf-home`。SAM3 的 revision 是 `f8ee5d16d5c30d0e87b1d22cbd358ce97761df56`。`src/cap_harness/providers/sam3/service.py` 发现本地 `sam3.pt` 后会设 `HF_HUB_OFFLINE=1`，避免再访问 Hugging Face。

安装（已成功，`BOOTSTRAP_EXIT=0`，日志 `/root/gpufree-data/aspire/provider-bootstrap.log`）：

```bash
export PATH="$HOME/.local/bin:$PATH"
export UV_DEFAULT_INDEX=https://mirror.sjtu.edu.cn/pypi/web/simple
export UV_CACHE_DIR=/root/gpufree-data/aspire/uv-cache-providers
export CAP_HARNESS_PYTHON=/usr/bin/python3
export CAP_HARNESS_PROFILE=rtx4090
bash scripts/bootstrap_providers.sh --providers sam3,contact_graspnet --profile rtx4090
```

启动（当前已在跑，`/healthz` 返回 `{"ok":true}`，日志 `/root/gpufree-data/aspire/provider-supervise.log`）：

```bash
export HF_HOME=/root/gpufree-data/aspire/hf-home
export HF_HUB_OFFLINE=1
export CAP_HARNESS_PYTHON=/usr/bin/python3
bash scripts/supervise_services.sh --profile rtx4090 --providers sam3,contact_graspnet --ready-timeout 600
```

两个模型大约占 4.5 GB 显存。再开 Isaac 时同一张卡还要留给仿真；显存不够就先停掉这个监督进程。不要为 BEHAVIOR 再开 PyRoki 或 HTTP 版 cuRobo。

## Gento Skye 导入

源包：`/root/gpufree-data/aspire/gento_skye_description`（约 132 MB，75 个 STL）。使用的 URDF 是 `urdf/gento_skye.urdf`，机器人名 `gento_skye`。56 个 `package://gento_skye_description/meshes/...` 引用全部落在包内：`meshes/skye/` 是底盘、轮子、腰、头和双臂，`meshes/gripper/` 是夹爪。

导入配置和中间文件在 `/root/gpufree-data/aspire/gento-skye-import/`：

- `gento_skye_source_config.yaml`
- `gento_skye.urdf`（网格已改成绝对路径，并去掉两个 mimic）
- `meshes/collision/`：每个连杆一个凸包 OBJ
- `import.log`

官方 `import_custom_robot.py` 在简化凸包时导入 `pymeshlab`，这里报 `undefined symbol: _ZdlPvm, version Qt_5`。因此先用 trimesh 对每个可视网格做凸包，再按最远点保留不超过 60 个顶点，写成与可视网格不同的碰撞文件。导入脚本发现专用碰撞体后会保留它们，不再调用 pymeshlab。凸包体积约为完整凸包的 93%–99%。

导入命令（已成功，`IMPORT_EXIT=0`）：

```bash
cd /root/gpufree-data/aspire/ASPIRE
ulimit -c 0
.venv-behavior/bin/python \
  third_party/BEHAVIOR-1K/OmniGibson/omnigibson/examples/robots/import_custom_robot.py \
  --config /root/gpufree-data/aspire/gento-skye-import/gento_skye_source_config.yaml
```

处理方式：

- 8 个轮子关节 `ZQ ZQL YQ YQL ZH ZHL YH YHL` 做成固定关节，并加上全向底盘 `base_footprint_{x,y,rz}`。
- 腰关节 `waist_joint_1/2/3` 作为躯干。
- 头关节 `Head_J1_Joint`、`Head_J2_Joint`。
- 左右臂 `Joint1`–`Joint7`，夹爪 `Arm_L10/L11`、`Arm_R10/R11`。去掉 mimic 后两根手指可分别驱动。
- 末端标记 `left_eef_link`、`right_eef_link` 挂在 `TCP_Link_L`、`TCP_Link_R` 上。
- 相机、IMU 不做碰撞。`Arm_R11_Joint` 的原点 `y=1.793` 加上绕 X 轴 180° 后与网格对齐，导入时没有改这个数。

Isaac 对若干固定关节的轴方向给出 “not body aligned” 警告，转换仍然完成。

注册结果：

`/root/gpufree-data/aspire/behavior-data/omnigibson-robot-assets/models/gento_skye/`

| 文件 | 说明 |
| --- | --- |
| `usd/gento_skye.usda` | 约 491 MB |
| `gento_skye.yaml` | RobotDefinition，启动时按文件名自动注册，模型名 `gento_skye` |
| `curobo/gento_skye_description_curobo_{default,base,arm,arm_no_torso}.yaml` | 规划配置 |

导入当天（2026-10-20）的状态：`gento_skye.yaml` 按功能分组写成 6 个底盘虚拟关节、3 个腰、2 个头、左臂 7、左夹爪 2、右臂 7、右夹爪 2，共 29 维，复位姿态全是 0；`self_collisions` 设为 false；还没有在场景里生成过这台机器人。

**后来的改动（2026-10-21）：**

- 29 维的真实顺序以运行时 `robot.joints` 为准：6 个底盘、3 个腰，然后头和左右臂逐关节交错，最后 4 个手指。`default_joint_pos` 已按这个顺序重写，和 `config.py` 的 `RESET_JOINT_POSITIONS` 一致。见 [gento-skye-troubleshooting.md](gento-skye-troubleshooting.md) 第 6 节。
- `gento_skye.yaml` 里加了辅助抓取射线点；USD 里头相机转 180° 并前移 1 cm；cuRobo 碰撞球全部按网格重新拟合。
- `src/cap_harness/behavior/registry.py` 的 BEHAVIOR 任务机器人已从 `r1pro` 改成 `gento_skye`。

全部改动、重新导入前后要核对的项目见 [gento-skye-README.md](gento-skye-README.md) 第 3、4 节。**重新导入会重写 USD 和 cuRobo 配置**，导入前先按那里的步骤备份。
