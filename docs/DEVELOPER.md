# 硬件驱动库 - 开发者文档

## 项目结构

```
hardware_driver/
├── include/                        # 公共头文件
│   ├── hardware_driver.hpp         # 主头文件
│   └── hardware_driver/            # 内部头文件
│       ├── bus/                    # 总线接口
│       ├── driver/                 # 驱动接口
│       ├── interface/              # 硬件接口
│       └── protocol/               # 协议文件
├── src/                           # 源码实现
│   ├── bus/                       # 总线实现
│   ├── driver/                    # 驱动实现
│   ├── interface/                 # 硬件接口实现
│   └── protocol/                  # 协议实现
├── examples/                      # 示例代码
├── tests/                         # 测试代码
├── cmake/                         # CMake配置
├── build.sh                       # 构建脚本
├── CMakeList.txt                  
└── README.md                      # 用户文档
```

## 构建系统

### 依赖项
- CMake 3.8+
- C++17 编译器
- pthread 库
- GTest (用于测试)

### 构建步骤
```bash
# 克隆项目
git clone https://github.com/Ding-Kaiyue/hardware_driver.git
cd hardware_driver

# 构建
./build.sh

# 或手动构建
mkdir build && cd build
cmake ..
make
```

### 测试
```bash
# 运行所有测试
cd build
make test

# 运行特定测试
./test_robot_hardware
./test_motor_driver_impl
```

## 架构设计

### 核心组件

1. **HardwareDriver** - 用户主要接口类
   - 提供简单的API
   - 内部管理RobotHardware实例
   - 线程安全的接口

2. **RobotHardware** - 硬件抽象层
   - 管理电机驱动和总线
   - 处理多线程状态反馈
   - 提供完整的控制接口

3. **MotorDriver** - 电机驱动层
   - 实现电机控制协议
   - 处理CAN通信
   - 管理电机状态

4. **BusInterface** - 总线通信层
   - 抽象CAN总线接口
   - 支持CANFD和EtherCAT
   - 处理底层通信

5. **Protocol** - 通信协议层
   - 定义电机通信协议
   - 处理数据包格式
   - 实现协议解析

### 线程模型

```
主线程 (用户调用)
    ↓
HardwareDriver (API层)
    ↓
RobotHardware (控制层)
    ↓
┌─────────────────┬─────────────────┬─────────────────┐
│   控制线程       │   反馈请求线程    │   反馈处理线程    │
│ (发送控制命令)    │ (定期请求状态)    │ (处理状态反馈)    │
└─────────────────┴─────────────────┴─────────────────┘
    ↓
MotorDriver (驱动层)
    ↓
BusInterface (通信层)
    ↓
硬件 (CAN接口)
```

## 开发指南

### 添加新功能

1. **在头文件中声明**
   ```cpp
   // include/hardware_driver/interface/robot_hardware.hpp
   class RobotHardware {
   public:
       void new_function(const std::string& interface, uint32_t motor_id, ...);
   };
   ```

2. **在实现文件中实现**
   ```cpp
   // src/interface/robot_hardware.cpp
   void RobotHardware::new_function(const std::string& interface, uint32_t motor_id, ...) {
       // 实现逻辑
   }
   ```

3. **在用户接口中暴露**
   ```cpp
   // include/hardware_driver.hpp
   class HardwareDriver {
   public:
       void new_function(const std::string& interface, uint32_t motor_id, ...) {
           robot_hardware_->new_function(interface, motor_id, ...);
       }
   };
   ```

### 添加新协议

1. **创建协议文件**
   ```cpp
   // src/protocol/new_protocol.hpp
   class NewProtocol {
   public:
       static void encode_command(...);
       static void decode_response(...);
   };
   ```

2. **在驱动中使用**
   ```cpp
   // src/driver/motor_driver_impl.cpp
   #include "protocol/new_protocol.hpp"
   ```

### 添加新总线类型

1. **继承BusInterface**
   ```cpp
   // src/bus/new_bus_impl.hpp
   class NewBusImpl : public BusInterface {
   public:
       NewBusImpl(const std::vector<std::string>& interfaces);
       // 实现虚函数
   };
   ```

2. **在工厂中注册**
   ```cpp
   // src/interface/robot_hardware.cpp
   auto bus = std::make_shared<bus::NewBusImpl>(interfaces);
   ```

## 发布流程

> **注意**：正式版本发布由项目维护者负责，开发者请勿直接发布。

### 发布检查清单
- [ ] 所有测试通过
- [ ] 文档更新
- [ ] 版本号更新
- [ ] 发布包创建
- [ ] Debian包构建
- [ ] GitHub发布

### 开发者贡献流程
1. Fork项目
2. 创建功能分支
3. 提交Pull Request
4. 等待代码审查
5. 维护者决定是否合并和发布

## 代码规范

### 命名规范
- **类名**: PascalCase (如 `HardwareDriver`)
- **函数名**: snake_case (如 `control_motor_in_velocity_mode`)
- **变量名**: snake_case (如 `motor_config`)
- **常量**: UPPER_SNAKE_CASE (如 `DEFAULT_TIMEOUT`)

### 文件组织
- **头文件**: 只包含声明，不包含实现
- **实现文件**: 包含完整的实现逻辑
- **测试文件**: 对应实现文件的测试

### 注释规范
```cpp
/**
 * @brief 函数功能描述
 * @param param1 参数1描述
 * @param param2 参数2描述
 * @return 返回值描述
 */
```

## 调试指南

### 常见问题

1. **编译错误**
   - 检查C++17标准
   - 检查依赖库
   - 检查头文件包含路径

2. **运行时错误**
   - 检查CAN接口配置
   - 检查权限设置
   - 检查电机连接

3. **性能问题**
   - 检查线程配置
   - 检查通信频率
   - 检查内存使用

### 调试工具

```bash
# 检查库依赖
ldd build/lib/libhardware_driver.so

# 检查符号表
nm build/lib/libhardware_driver.so | grep HardwareDriver

# 运行测试
cd build
./test_robot_hardware
./test_motor_driver_impl
./test_motor_protocol
./test_canfd_bus_integration

# 运行所有测试（如果有测试脚本）
./build.sh里面有运行所有测试的脚本

# 性能分析
valgrind --tool=callgrind ./simple_example

# 查看性能报告
# 使用kcachegrind图形界面查看
kcachegrind callgrind.out.*

# 或者使用命令行工具
callgrind_annotate callgrind.out.*

# 内存泄漏检查
# 检查内存泄漏
valgrind --tool=memcheck --leak-check=full ./simple_example

# 检查内存访问错误
valgrind --tool=memcheck ./simple_example

# 其他有用的选项
# 详细的内存检查
valgrind --tool=memcheck --leak-check=full --show-leak-kinds=all ./simple_example

# 检查未初始化的变量
valgrind --tool=memcheck --track-origins=yes ./simple_example

# 生成详细报告
valgrind --tool=callgrind --callgrind-out-file=profile.out ./simple_example
```

## 版本管理

### 版本号格式
- **主版本号**: 不兼容的API修改
- **次版本号**: 向下兼容的功能性新增
- **修订号**: 向下兼容的问题修正

### 发布检查清单
- [ ] 所有测试通过
- [ ] 文档更新
- [ ] 版本号更新
- [ ] 发布包创建
- [ ] Debian包构建
- [ ] GitHub发布

## 贡献指南

1. **Fork项目**
2. **创建功能分支**
3. **编写代码和测试**
4. **提交Pull Request**
5. **代码审查**
6. **合并到主分支**

## 许可证

本项目采用开源许可证，具体许可证信息请查看项目根目录的 `LICENSE` 文件。

### 开源贡献
欢迎社区贡献代码！请遵循以下流程：
1. Fork项目
2. 创建功能分支
3. 提交Pull Request
4. 等待代码审查
5. 维护者决定是否合并 
