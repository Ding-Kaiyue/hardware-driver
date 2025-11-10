# DM Tools Package

达妙电机系统工具包，包含各种实用工具。

## 包含工具

### `dev_sn` - 设备序列号检测工具

检测并显示连接的达妙USB转CANFD设备的序列号信息。

#### 功能
- 自动扫描系统中所有USB设备
- 识别达妙USB转CANFD设备 (VID: 0x34B7, PID: 0x6877)
- 显示设备的VID、PID和序列号信息

#### 使用方法

```bash
# 编译工具包
colcon build --packages-select dm_tools

# 运行设备检测工具
ros2 run dm_tools dev_sn
```

#### 输出示例
```
U2CANFD_DEV 0:
  VID: 0x34b7
  PID: 0x6877
  SN: 14AA044B241402B10DDBDAFE448040BB

U2CANFD_DEV 1:
  VID: 0x34b7
  PID: 0x6877
  SN: 26BC155A352613C21EECEBFF559151CC
```

#### 注意事项
- 确保USB设备已正确连接
- 需要适当的USB设备访问权限
- 如果没有找到设备，请检查设备连接和驱动安装

## 依赖

- libusb-1.0-dev
- ROS2 (ament_cmake)

## 安装

作为ROS2工作空间的一部分：

```bash
cd dm_arm_v2
colcon build --packages-select dm_tools
source install/setup.bash
```

## 许可证

Apache-2.0