# 建图任务自动保存工具

本仓库默认 `avia.yaml` 的 `pcd_save.pcd_save_en=false`，普通启动不会通过原算法的保存逻辑生成整轮 PCD。录制、筛选和离线导出脚本需要另外使用，不能等同于“以后每次建图结束自动保存”；板端实际是否开启原生保存应以它加载的配置及启动参数为准。

本目录是新增的独立工具：额外启动保存节点，并用现有二进制和两份原 YAML 启动 FAST-LIVO2。它不修改原源码、原配置或算法参数，不打开原生 PCD 保存，不覆写雷达/IMU 话题，也不接入旧筛选转发链。

本工具已加入 `slam-backup/map_autosave/`。工程根目录的 `start_mapping_saved.sh` 转到本目录的同名启动器；原 `run_fastlivo.sh` 仍是普通建图入口，不会自动调用本工具。

默认运行目标仍是香橙派：ROS 2 Humble、已编译工作区 `/home/orangepi/dl/livox_ws`。电脑上的 `/home/yoda/slam-backup` 当前只有源码，没有 `install/setup.bash` 和已编译的 mapper，不能把“已加入本地仓库”当成“已经在香橙派更新并运行”。本次集成没有进行板端部署或实物建图。此前独立部署路径 `/home/orangepi/map_autosave_20261022` 的历史记录不作为本次运行状态证明。

## 使用

以下启动命令由用户在准备开始建图时执行，部署过程不会自动执行。

1. 按原方式运行板端原 `start_all.sh`，单独启动传感器。
2. 不要再同时运行原 FAST-LIVO2 启动脚本或 launch。新工具自己启动一个 mapper；检测到已有 mapper 时会拒绝启动，不接管或停止它。
3. 在另一个终端进入香橙派上实际部署的工程根目录，再执行下面的命令。它会启动保存节点和一个 mapper，不会启动雷达、IMU 或二维建图节点：

```bash
bash ./start_mapping_saved.sh
```

脚本使用 ROS 2 Humble、原工作区 `/home/orangepi/dl/livox_ws`、`ROS_DOMAIN_ID=30`，直接运行原 `fastlivo_mapping`，读取原安装目录中的 `avia.yaml` 和 `camera_pinhole.yaml`。需要板端已有的 ROS 消息包、`rclpy`、NumPy、PyYAML。

每次启动创建独立任务目录：

```text
/home/orangepi/slam_runs/日期_时间_PID/
```

没有固定运行时限，也不会因漂移或断流自动停止。正常结束时，在这个启动终端按一次 **Ctrl+C，并等待退出和收尾**。工具仅向自己启动的 mapper 发送停止请求；不会强制杀进程。确认任务 `status.json` 中 `pcd_state` 为 `finalized`，且已生成 `map_3d.pcd`。关闭 SSH 不是正常保存结束操作；收到 SIGHUP 时工具继续运行。

## 保存内容

| 文件 | 含义 |
| --- | --- |
| `map_3d.pcd` | 正常收尾后的二进制 PCD，包含本工具实际收到并成功写入的有效点 |
| `map_3d.partial.pcd`、同名 `.checkpoint.json` | 运行中或未完成任务的数据与持久化边界 |
| `trajectory.csv` | 实际收到的 `/aft_mapped_to_init` 位姿 |
| `map_2d.yaml`、其引用的 `grid_*.pgm` | 若收到有效 `/map`，保存外部二维栅格的最新快照；新一代成功提交后删除旧图片 |
| `status.json` | 点数、消息计数、坐标系、保存错误、磁盘和文件审计等状态 |
| `mapper.log`、`saver.log`、`ros_log/` | 本次运行日志 |
| `config_snapshot/`、`previous_logs/`、`native_logs/` | 原配置只读副本、启动前日志副本、正常收尾时可复制的原生日志 |

`/home/orangepi/slam_runs/latest.txt` 记录最近创建的任务路径。任务相互独立，不覆盖或自动删除旧任务。

## 异常和恢复

默认每 5 秒尝试同步 PCD 数据和 checkpoint。异常退出、断电或磁盘故障后，只能恢复最后一个成功提交的 checkpoint；不能保证最多只损失 5 秒，也不能恢复未收到的消息。

确认该任务的保存器已停止后，可使用以下恢复命令，将示例任务路径换成实际目录：

```bash
bash ./start_mapping_saved.sh \
  --recover /home/orangepi/slam_runs/20261022_153000_12345
```

上面的恢复命令同样从部署后的工程根目录运行。恢复会保留已提交的点、截去未提交尾部，生成 `map_3d.pcd` 和 `recovery.json`。它不会重启 mapper，不会覆盖已存在的最终 PCD。损坏或缺失 checkpoint 时会拒绝猜测恢复。

启动前可用磁盘不足 1 GiB 时拒绝新任务。运行中在 checkpoint 检查到不足 512 MiB，或遇到写入错误时，停止追加保存并报警，任务保持 `partial`，**不会因此自动停止 mapper**。原 mapper 和 ROS 日志仍可能增长；这些阈值不能保证磁盘永不写满。保存器内部故障也不会自动停止仍运行的 mapper，应查看 `status.json` 和日志确认其状态。

## 保存范围与验证

- 3D 来自 `/cloud_registered` 的已发布帧。BEST_EFFORT 和有限队列可能丢帧，算法自身也可能筛选点；它不等同于完整内部地图或原生 PCD。`finalized` 仅表示接收数据已规范收尾，不证明无丢帧、无漂移或精度达标。
- 不做全图内存累计或体素降采样；重复观测会重复写入，文件随任务增长。保存节点和定期同步仍有 CPU、内存和磁盘 I/O 开销。
- 2D 是现有外部 `/map` 的快照。工具不生成、不重置、不拥有该地图或 costmap；其中可能含前轮历史或已加载旧地图。旧时间戳或时间戳为 0 不能证明地图属于本轮；来源和时间戳保存在元数据中。没有收到兼容的有效 `/map` 时不生成二维文件。
- 原文件前后 hash 用于核对文件内容；读取失败时审计结果可能未知，不能据此宣称原文件已验证一致。
- 离线测试涵盖点云解析、流式 PCD、恢复、二维栅格和模拟生命周期。测试使用假的 ROS 和 mapper，不会启动真实传感器；测试通过不等同于真实 ROS 通信或 4 GB 实时性能验证。

在工程根目录复查离线测试。`PYTHONDONTWRITEBYTECODE=1` 避免写入 Python 缓存；命令不会启动实际节点：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s ./map_autosave -p 'test_*.py' -v
```

## 集成文件

- 根目录 `start_mapping_saved.sh`：新入口，按自身位置查找本工具，可从其他工作目录调用。
- `map_autosave/start_mapping_saved.sh`：加载板端原 ROS 环境，转交命令行参数。
- `run_task.py`：独立保存节点和本轮 mapper 的生命周期管理。
- `pcd_io.py`：逐批写入 PCD、检查点和恢复，不把整张地图攒在内存里。
- `test_*.py`：保留原离线测试。

核心保存代码、原来的板端环境路径和运行参数保持原版。未修改 `FASTLIVO2_ROS2-master`、`sensor_sync`、雷达驱动或原有 YAML。运行任务的数据默认保存在香橙派 `/home/orangepi/slam_runs`，不写到电脑 C 盘。
