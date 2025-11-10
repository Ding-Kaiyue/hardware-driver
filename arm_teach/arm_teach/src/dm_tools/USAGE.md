# DM Tools 使用指南

## 独立编译和使用 (推荐)

如果不需要ROS2功能，可以独立编译dm_tools包：

### 编译步骤

```bash
cd dm_arm_v2/src/dm_tools

# 创建编译目录
mkdir -p build_standalone
cd build_standalone

# 配置和编译
cmake ..
make

# 测试工具
./dev_sn
```

### 预期输出

```
U2CANFD_DEV 6:
  VID: 0x34b7
  PID: 0x6877
  SN: A4F890D5ACE224020802CF2D55081601
```

### 安装到系统 (可选)

```bash
# 安装到 /usr/local/bin
sudo make install

# 之后可以在任何地方使用
dev_sn
```

## ROS2集成使用

如果需要ROS2集成：

```bash
# 在ROS2工作空间中编译
cd dm_arm_v2
colcon build --packages-select dm_tools
source install/setup.bash

# 使用ROS2命令运行
ros2 run dm_tools dev_sn
```

## 工具说明

### dev_sn - 设备序列号检测

**功能**: 检测系统中连接的达妙USB转CANFD设备

**识别条件**:
- VID: 0x34B7 (达妙科技)
- PID: 0x6877 (USB转CANFD设备)

**输出信息**:
- 设备编号
- VID/PID
- 序列号 (用于motor_driver配置)

**权限要求**:
- 需要USB设备访问权限
- 建议设置udev规则:

```bash
sudo nano /etc/udev/rules.d/99-usb.rules
# 添加内容:
SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6877", ATTRS{serial}=="A4DACC30545114611B570C2C92F14231", SYMLINK+="ttyLeft", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6877", ATTRS{serial}=="68AC9316B2021452D97FB3E429B8DB5D", SYMLINK+="ttyRight", MODE="0666"

# 重新加载规则
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 故障排除

### 1. 没有检测到设备
- 检查USB连接
- 确认设备权限
- 验证设备是否为达妙USB转CANFD

### 2. 编译失败
- 确认已安装libusb-1.0-dev:
  ```bash
  sudo apt install libusb-1.0-0-dev
  ```

### 3. 权限错误
- 设置udev规则 (见上方)
- 或临时使用sudo运行

## 集成到motor_driver

将检测到的序列号用于机械臂控制器：

```bash
# 1. 检测序列号
./dev_sn

# 2. 复制显示的SN，如: A4F890D5ACE224020802CF2D55081601

# 3. 在motor_driver配置中使用
ros2 launch motor_driver arm_controller.launch.py \
    device_serial_number:=A4F890D5ACE224020802CF2D55081601
```

完美分离！dm_tools现在是一个独立的工具包，可以独立编译和使用。
