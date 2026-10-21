# gento_skye/：仓库外文件的副本

Gento Skye 的一部分文件不在 ASPIRE 仓库里。这个目录存放它们的副本，和 ASPIRE 的代码、文档改动一起提交，防止 GPUFree 实例释放后丢失（`/root/gpufree-data/` 在释放实例时不保存，做镜像时也不带）。

说明文档在 `docs/gento-skye-README.md`。

## 目录

| 这里 | 原位置（工作区 `/root/gpufree-data/aspire`） | 内容 |
| --- | --- | --- |
| `import/` | `gento-skye-import/` | 导入配置 `gento_skye_source_config.yaml`、碰撞球脚本（拟合、检查、写回、导出 MoveIt）、拟合结果、MoveIt 导出、导入用的 URDF 和碰撞网格 |
| `robot_assets/gento_skye/` | `behavior-data/omnigibson-robot-assets/models/gento_skye/`（**不含** `usd/`） | `gento_skye.yaml`、4 份 cuRobo 配置、`misc/` 里的备份 |
| `description/gento_skye_description/` | `gento_skye_description/` | 原始 URDF 和网格 |

没有放进来的：

- `models/gento_skye/usd/gento_skye.usda`（469 MB，超过 GitHub/Gitee 的单文件上限）。用导入配置重新生成，见下文。
- `behavior-data/` 里的数据集。BEHAVIOR 数据条款禁止再分发，按 `docs/gpufree-behavior-gento-skye.md` 重新下载。
- `outputs/` 里的运行记录（已在 `.gitignore`）。

## 提交前同步

原位置的文件改过之后，先同步再提交：

```bash
cd /root/gpufree-data/aspire/ASPIRE
bash gento_skye/sync_from_workspace.sh
git add -A gento_skye docs src tests scripts
git commit -m "..."
```

## 在新机器上恢复

先按 `docs/gpufree-behavior-gento-skye.md` 装好环境、下好数据集，然后：

```bash
WS=/root/gpufree-data/aspire
cd $WS/ASPIRE
cp -r gento_skye/description/gento_skye_description $WS/
cp -r gento_skye/import $WS/gento-skye-import
```

导入配置里 URDF 的网格是绝对路径（`$WS/gento-skye-import/gento_skye.urdf`），换了工作区路径要先改。然后重新导入，生成 USD：

```bash
.venv-behavior/bin/python third_party/BEHAVIOR-1K/OmniGibson/omnigibson/examples/robots/import_custom_robot.py \
  --config $WS/gento-skye-import/gento_skye_source_config.yaml
```

导入会重写 `models/gento_skye/` 下的 USD 和 cuRobo 配置。导入完成后，用这里的副本覆盖回去：

```bash
MODEL=$WS/behavior-data/omnigibson-robot-assets/models/gento_skye
cp gento_skye/robot_assets/gento_skye/gento_skye.yaml $MODEL/
cp gento_skye/robot_assets/gento_skye/curobo/*.yaml $MODEL/curobo/
```

最后按 `docs/gento-skye-README.md` 第 4 节逐项核对：头相机、复位角、辅助射线点、碰撞球、末端轴、关节顺序，再跑一次 bring-up。

头相机：导入配置里已经写好转 180° 和前移 1 cm，重新生成的 USD 应该直接正确。如果 `head.png` 还是黑的，按 `docs/gento-skye-troubleshooting.md` 第 11 节手改 USD 里 `head_depth_camera_Link/Camera` 的两行。
