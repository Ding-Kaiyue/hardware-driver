# Damiao Arm Controller Node

这是一个简化的ROS2节点，专门用于控制达妙7自由度机械臂。基于原有的main.cpp代码重构而成。

## 功能特性

### 🎯 简化设计
- **纯ROS2节点**: 不使用lifecycle，直接启动即可使用
- **标准接口**: 订阅`sensor_msgs/JointState`作为控制输入
- **实时反馈**: 发布当前关节状态到`/joint_states`
- **安全控制**: 支持电机使能/失能，位置和速度限制

### 🤖 7自由度配置
基于main.cpp的实际硬件配置：
- **Joint 1-3**: DM4340电机 (高扭矩)
- **Joint 4-6**: DM4310电机 (中等扭矩)  
- **Joint 7**: DMH3510电机 (高精度腕部)

### 🛡️ 安全机制
- 位置范围限制: ±2π radians
- 速度限制: ±5.0 rad/s (可配置)
- 软件急停: Ctrl+C安全关闭
- 参数验证: 输入命令范围检查

## 编译和运行

### 1. 编译工作空间

使用简化的配置文件:
```bash
cd dm_arm_v2

# 使用简化版配置
cp src/motor_driver/CMakeLists_simple.txt src/motor_driver/CMakeLists.txt
cp src/motor_driver/package_simple.xml src/motor_driver/package.xml

# 编译
colcon build --packages-select motor_driver
source install/setup.bash
```

### 2. 配置设备

修改配置文件中的设备序列号:
```bash
nano src/motor_driver/config/arm_config.yaml
# 修改device_serial_number为你的设备序列号
```

### 3. 启动节点

```bash
# 启动机械臂控制器
ros2 launch motor_driver arm_controller.launch.py device_serial_number:=YOUR_SN

# 或直接运行节点
ros2 run motor_driver arm_controller_node --ros-args --params-file src/motor_driver/config/arm_config.yaml
```

## 使用方法

### 电机使能/失能

```bash
# 启用所有电机
ros2 topic pub /enable_motors std_msgs/msg/Bool "data: true" --once

# 失能所有电机  
ros2 topic pub /enable_motors std_msgs/msg/Bool "data: false" --once
```

### 发送关节位置命令

```bash
# 发送7个关节的位置命令
ros2 topic pub /joint_command sensor_msgs/msg/JointState \
  "header:
    stamp: 
      sec: 0
      nanosec: 0
    frame_id: ''
  name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

### 监控关节状态

```bash
# 查看当前关节状态
ros2 topic echo /joint_states

# 查看电机状态
ros2 topic echo /motor_status

# 启动可视化工具
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

### 编程控制示例

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import math

class ArmControllerTest(Node):
    def __init__(self):
        super().__init__('arm_test')
        
        # 发布器
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_command', 10)
        self.enable_pub = self.create_publisher(Bool, 'enable_motors', 10)
        
        # 订阅器
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        
        # 定时器
        self.timer = self.create_timer(0.1, self.control_loop)
        self.time = 0.0
        
        # 启用电机
        self.enable_motors()
    
    def enable_motors(self):
        msg = Bool()
        msg.data = True
        self.enable_pub.publish(msg)
        self.get_logger().info('Motors enabled')
    
    def control_loop(self):
        # 正弦波轨迹
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        
        # 生成正弦波位置
        amplitude = 0.5
        frequency = 0.1
        msg.position = [
            amplitude * math.sin(2 * math.pi * frequency * self.time),
            amplitude * math.sin(2 * math.pi * frequency * self.time + math.pi/4),
            amplitude * math.sin(2 * math.pi * frequency * self.time + math.pi/2),
            amplitude * math.sin(2 * math.pi * frequency * self.time + 3*math.pi/4),
            amplitude * math.sin(2 * math.pi * frequency * self.time + math.pi),
            amplitude * math.sin(2 * math.pi * frequency * self.time + 5*math.pi/4),
            amplitude * math.sin(2 * math.pi * frequency * self.time + 3*math.pi/2)
        ]
        
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7
        
        self.joint_cmd_pub.publish(msg)
        self.time += 0.1
    
    def joint_state_callback(self, msg):
        # 处理关节状态反馈
        pass

def main():
    rclpy.init()
    node = ArmControllerTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 话题和参数

### 订阅话题
- `/joint_command` (sensor_msgs/JointState): 关节位置命令
- `/enable_motors` (std_msgs/Bool): 电机使能命令

### 发布话题  
- `/joint_states` (sensor_msgs/JointState): 当前关节状态
- `/motor_status` (std_msgs/Bool): 电机使能状态

### 参数
- `device_serial_number`: USB设备序列号
- `control_frequency`: 控制频率 (默认1000Hz)
- `position_tolerance`: 位置容差 (默认0.01 rad)
- `velocity_limit`: 速度限制 (默认5.0 rad/s)
- `kp_gains`: 位置增益数组
- `kd_gains`: 速度增益数组

## 故障排除

### 常见问题

1. **节点启动失败**
   ```bash
   # 检查设备权限
   ls -l /dev/bus/usb/*/
   
   # 检查设备序列号
   ros2 param get /arm_controller device_serial_number
   ```

2. **电机不响应**
   ```bash
   # 检查电机是否使能
   ros2 topic echo /motor_status
   
   # 重新使能电机
   ros2 topic pub /enable_motors std_msgs/msg/Bool "data: true" --once
   ```

3. **控制不平滑**
   ```bash
   # 检查控制频率
   ros2 topic hz /joint_states
   
   # 调整控制增益
   ros2 param set /arm_controller kp_gains "[10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 8.0]"
   ```

### 调试命令

```bash
# 查看节点信息
ros2 node info /arm_controller

# 查看所有参数
ros2 param list /arm_controller

# 实时监控话题
ros2 topic hz /joint_states
ros2 topic bw /joint_command

# 检查系统状态
top -p $(pgrep arm_controller)
```

## 安全注意事项

1. **渐进测试**: 从小幅度动作开始测试
2. **监控温度**: 长时间运行时注意电机温度
3. **急停准备**: 确保Ctrl+C能够安全停止
4. **范围限制**: 不要超出关节物理限位
5. **网络延迟**: 注意ROS2通信延迟对控制的影响

基于原有的main.cpp架构，这个节点提供了简单可靠的ROS2机械臂控制接口！