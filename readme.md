# Tiangong 2 Pro 机器人项目

> **注意**：本项目设计为在**控制端 PC** 上运行，用于远程监控和发送指令，**并非直接运行在机器人电脑上**。

本项目是一个基于 ROS 2 的机器人工作空间，主要包含了 **Tiangong 2 Pro** 机器人的描述文件、可视化工具以及相关的消息和库。

## 项目结构

```text
src/
├── bodyctrl_msgs/      # 机器人控制相关的自定义消息和服务（电机、电源、IMU、手部控制等）
└── tiangong2pro_urdf/  # 机器人模型文件 (URDF/Xacro)、模型显示与交互控制的 Launch 文件
```

## 功能特性

- **机器人描述**: 提供 Tiangong 2 Pro 完整和带手部版本的 URDF/Xacro 模型。
- **TF 树发布**: 内部集成 `robot_state_publisher`，能够实时发布完整的机器人坐标变换（TF 树）。
- **可视化**: 支持在 RViz 中查看机器人骨架及关节状态。
- **交互控制**: 提供交互式 GUI 用于调试和控制关节移动。

## 依赖安装

在使用本项目之前，请确保已安装 **ROS 2 Humble** 版本及其相关依赖。

### 系统依赖

可以通过 `rosdep` 自动安装大部分依赖（推荐）：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 主要依赖项 (APT 安装)

如果你需要手动安装主要依赖，请运行以下命令：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-xacro \
  ros-humble-sensor-msgs \
  ros-humble-visualization-msgs \
  python3-pyqt5
```

## 编译方法

在工作空间根目录下运行：

```bash
colcon build --symlink-install
# 或者
colcon build --packages-select tiangong2pro_urdf
source install/setup.bash
```

## 网络配置

在运行控制或查看带手部的完整模型前，需要通过网线连接机器人，并配置本地网络：

- **连接方式**: 使用网线直接连接机器人。
- **IP 配置**: 将本地网卡的静态 IP 设置为 `192.168.41.X`（例如 `192.168.41.100`），子网掩码 `255.255.255.0`。

## 使用说明

### 查看机器人模型

运行以下命令在 RViz 中查看不带手部的机器人模型：

```bash
ros2 launch tiangong2pro_urdf display.launch.py
```

查看带手部的完整模型（需网线连接且192.168.41.1运行了body_control）：

```bash
ros2 launch tiangong2pro_urdf display_with_hands.launch.py
```

### 交互式控制

提供了一个基于 Qt 的图形界面，用于控制真实机器人的关节，并提供一个半透明的“幽灵”机器人（Ghost Robot）来预览目标姿态。

**特性**：
- **实时同步**: GUI 启动时会自动同步当前真实机器人的关节位置。
- **目标预览**: 拖动滑块时，RViz 中会显示半透明的 Ghost 机器人，展示预期的目标姿势，方便用户在执行前确认。
- **安全执行**: 点击 "Execute" 按钮后，会弹出安全确认框，确认后才会向真实机器人下发运动指令。
- **无污染 TF**: Ghost 机器人完全通过 Marker 可视化实现，不发布额外的 TF 帧，保持 TF 树整洁。

**启动步骤**:

1. **机器人端 (192.168.41.1)**:
   确保机器人底层控制程序已启动（通常是开机自启，如果需要手动重启）：
   ```bash
   sudo su
   systemctl stop proc_manager.service
   source /home/ubuntu/ros2ws/install/setup.bash
   ros2 launch body_control body.launch.py
   ```

2. **本机端**:
   ```bash
   ros2 launch tiangong2pro_urdf interactive_control.launch.py
   ```
   启动后会自动打开 RViz 和 Qt 控制面板。勾选 RViz 左侧 Displays 面板中的 `Ghost Robot` (MarkerArray) 即可看到预览模型。

3. **配合抓取示例使用**

   ```bash
   ros2 launch tiangong2pro_urdf grasp_pose.launch.py
   ```
   启动后会自动打开 RViz 和 Qt 控制面板。并且已添加了识别物体的位姿和抓取点的位姿两个Pose，在识别到物体时即会显示物体位置和抓取点的位姿。


注：如果提示找不到 bodyctrl_msgs 包，可手动安装根目录下的这个包，如果是 arm 架构可安装 ros-humble-bodyctrl-msgs_0.0.0-0jammy_arm64.deb，如果是 x86 架构可安装 ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb，安装命令：

```bash
sudo dpkg -i xxx.deb
```