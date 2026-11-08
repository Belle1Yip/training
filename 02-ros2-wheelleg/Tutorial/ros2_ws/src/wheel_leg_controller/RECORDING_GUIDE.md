# 录屏、bag 和提交证据指南

这份指南只解决“怎么自己录出可信证据”。提交物以官方教程第 5.7、5.8 节为准。

## 1. 官方要求

第 5.7 节要求：

- 原地平衡至少 5 s，机体 pitch 偏差小于 10°；
- 加分项：平衡状态下以 0.3 m/s 前进 3 m 不摔；可选做加速度限制、推扰和 `/joy`。

第 5.8 节要求：

1. 新包 `src/wheel_leg_controller/`（`ament_python`），`colcon build` 通过；
2. 包内 `README.md` 说明控制结构、K/参数来源、实测平衡时长与 pitch 曲线；
3. 一段 10–20 s 录屏，或 `ros2 bag record /imu /joint_states /actuator_cmd` 的 bag 文件。

也就是说，录屏和 bag 二选一即可；时间紧张时优先录 bag，因为它比录屏更容易核对话题和时长。

## 2. 录制前准备

三个终端都要先执行：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Venom-Train/training/02-ros2-wheelleg/Tutorial/ros2_ws
source install/setup.bash
```

先构建并启动：

```bash
# 终端 A：仿真器
ros2 run wheel_leg_sim wheel_leg_sim

# 终端 B：控制器
ros2 run wheel_leg_controller wheel_leg_controller
```

如果只想录数据、不需要 GUI：

```bash
ros2 run wheel_leg_sim wheel_leg_sim --ros-args -p viewer:=false
```

确认链路正常：

```bash
ros2 node list
ros2 topic hz /joint_states
ros2 topic hz /actuator_cmd
```

应看到 `wheel_leg_sim` 和 `wheel_leg_controller` 两个节点，话题频率稳定。录屏时如果希望
“墙钟 15 s ≈ 仿真 15 s”，给仿真器加实时模式：

```bash
ros2 run wheel_leg_sim wheel_leg_sim --ros-args -p realtime:=true -p rtf:=1.0
```

## 3. 方案 A：录 ros2 bag（推荐）

### 3.1 开始录制

终端 C：

```bash
mkdir -p ~/Venom-Train/evidence
ros2 bag record -o ~/Venom-Train/evidence/balance_manual \
  /imu /joint_states /actuator_cmd
```

看到 `Recording...` 后，让机器人原地平衡 15–20 s，再按一次 `Ctrl+C` 停止 bag。

### 3.2 检查 bag

```bash
ros2 bag info ~/Venom-Train/evidence/balance_manual
```

提交前确认：

- `Duration` 在 10–20 s 左右；
- 三个话题都存在；
- 文件不是 0 字节。

如果已经录成 50 s 左右，官方要求的 10–20 s 就不满足。优先重新录一段 15 s；不想重跑时，
可以把旧 bag 回放并重新录制一段裁剪版：

```bash
# 终端 1：先启动录制
ros2 bag record -o ~/Venom-Train/evidence/balance_15s \
  /imu /joint_states /actuator_cmd

# 终端 2：从第 10 s 开始播放 15 s
ros2 bag play ~/Venom-Train/evidence/balance_manual \
  --start-offset 10 --playback-duration 15 --disable-keyboard-controls
```

播放结束后在终端 1 按 `Ctrl+C`，再用 `ros2 bag info` 确认新 bag 的 Duration 在 15 s 左右。
这只是裁剪旧数据；如果时间允许，重新跑一次并重新录更直接。

如果录屏和 bag 都想留，建议文件名写清日期，例如 `balance_20261108`，不要用
`test`、`final2`、`newnew` 这类无法辨认的名字。

## 4. 方案 B：录屏

### 4.1 安装工具

Ubuntu 24.04 Wayland 下可以安装 `wf-recorder`：

```bash
sudo apt update
sudo apt install -y wf-recorder
```

也可以用 OBS Studio：

```bash
sudo apt install -y obs-studio
```

### 4.2 用 wf-recorder 录全屏

先启动 GUI 仿真器和控制器，等机器人站稳，再在另一个终端启动：

```bash
wf-recorder -f ~/Venom-Train/evidence/balance_demo.mp4
```

录 15–20 s 后按 `Ctrl+C`。播放检查（或直接在文件管理器里双击）：

```bash
xdg-open ~/Venom-Train/evidence/balance_demo.mp4
```

### 4.3 用 OBS 录 MuJoCo 窗口

1. 打开 OBS，来源选择“窗口采集”或“屏幕采集（PipeWire）”；
2. 选中 MuJoCo 的 `wheel_leg_sim` 窗口；
3. 开始录制，等机器人平衡 15–20 s；
4. 停止录制，确认视频里能看到机器人、地面和姿态变化。

录屏不需要录声音，也不需要剪得很花；能看清机器人和运行过程即可。

## 5. 生成自己的 pitch 曲线和 README 数据

终端 C：

```bash
python3 src/wheel_leg_controller/tools/evaluate_balance.py \
  --duration 20 --settle 5 --output /tmp/balance_metrics.csv

python3 src/wheel_leg_controller/tools/plot_balance.py \
  --input /tmp/balance_metrics.csv \
  --output src/wheel_leg_controller/doc/balance_pitch.png
```

把终端输出的 `duration_s`、`max_abs_pitch_deg`、`rms_pitch_deg`、腿长范围写进
`README.md` 的“我的实测”一节。图必须来自你自己当次 CSV，不要复用旧图。

## 6. 加分项的自己的录法

### 6.1 平地 0.3 m/s 走 3 m

生成一个去掉障碍的测试场景（不修改冻结的仿真节点）：

```bash
sed '/<include file="obstacles.xml"\/>/d' \
  install/wheel_leg_sim/share/wheel_leg_sim/robots/robot.xml \
  > /tmp/robot_flat.xml

ros2 run wheel_leg_sim wheel_leg_sim --ros-args \
  -p viewer:=false -p robot_xml:=/tmp/robot_flat.xml
```

控制器启动后，在另一个终端发速度并录 bag：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

也可以用 `tools/evaluate_bonus.py --mode cmd_vel --speed 0.3 --distance 3.0` 自动记录 CSV。

### 6.2 `/joy` 手柄

有实体手柄：

```bash
sudo apt install -y ros-jazzy-joy
ros2 run joy joy_node
ros2 topic echo /joy --once
```

确认 `axes`、`buttons` 后，用左摇杆前进、右摇杆转向，录一段 10–20 s 的视频或 bag。
没有实体手柄时，用 `evaluate_bonus.py --mode joy` 发合成 `/joy`，同样能验证控制器路径。

### 6.3 推扰

本地仓库里的测试 harness（不随控制包提交）：

```bash
python3 ../tests/push_test_harness.py --force 20 --duration 0.15 \
  --push-time 8 --total-time 16 \
  --output /tmp/push_test.csv \
  --ros-args -p viewer:=false
```

同时启动控制器，录下机器人被推后恢复的过程。这个 harness 只在测试时注入
`data.xfrc_applied`，不修改 `wheel_leg_sim`。

### 6.4 默认障碍场景

默认 `robot.xml` 带 5 段坡，坡间有约 16 cm 落差。用 GUI 以 `0.5 m/s` 连续通过，
录一段完整通过视频；这个场景不要按 0.3 m/s 验收，低速会在坡脚卡住。

## 7. 提交前检查

- [ ] `colcon build` 通过；
- [ ] 平地平衡 ≥ 5 s，pitch < 10°；
- [ ] README 写了自己的控制结构、K/参数来源和实测数据；
- [ ] `doc/balance_pitch.png` 是自己当次生成；
- [ ] 有 10–20 s 录屏或三话题 bag；
- [ ] `/cmd_vel`、`/joy`、推扰按需要选做，并有自己的证据；
- [ ] 不提交旧调试 CSV、旧 bag、旧截图；
- [ ] 填写收集表并上传 bag/视频链接。
