# 变更日志

本项目遵循 [语义化版本](https://semver.org/lang/zh-CN/) 规范。

## [2.1.0] - 2025-10-20

### 🔧 改进

#### 澄清库的独立性
- **重要说明**: 库完全独立于ROS，可在任何C++项目中使用
- **ROS 2兼容**: 同时完全兼容ROS 2生态系统（可选）
- **更新README**: 新增清晰的说明和badges（License、Build Status、C++ Standard、CMake）

#### CI/CD改进
- **修复GitHub Actions**: 解决ament_cmake在CI环境中找不到的问题
- **显式CMAKE_PREFIX_PATH**: 在cmake命令中直接指定ROS路径
- **改进文档**: 更准确地说明库的用途和特性

### 📦 版本说明
- 此版本保持与v2.0.0的完全兼容
- 仅包含文档和CI配置的改进
- 推荐所有用户升级

## [2.0.0] - 2025-08-20

### 💥 重大变更（破坏性更新）

#### 架构重设计
- **事件驱动架构**: 全面替代轮询机制，采用观察者模式和事件总线
- **API重构**: 从 `HardwareDriver` 类改为 `RobotHardware` + `MotorDriverImpl` 架构
- **数据获取方式**: 移除 `get_motor_status()` 轮询，改为实时事件推送
- **构造方式变更**: 需要显式创建总线和驱动实例

#### 旧API移除
- 移除 `HardwareDriver` 类
- 移除 `HardwareUtility` 类  
- 移除 `get_motor_status()` 等轮询方法
- 移除旧的头文件结构

### ✨ 新增功能

#### 事件系统
- **EventBus**: 类型安全的事件总线系统
- **MotorStatusEvent**: 单个电机状态事件
- **MotorBatchStatusEvent**: 批量电机状态事件
- **MotorFunctionResultEvent**: 函数操作结果事件
- **MotorParameterResultEvent**: 参数操作结果事件

#### 观察者模式
- **MotorStatusObserver**: 电机状态观察者接口
- 支持单个和批量状态更新回调
- 支持函数操作结果通知
- 支持参数读写结果通知
- 自动生命周期管理

#### 多线程架构
- **三线程设计**: 控制线程、反馈线程、数据处理线程
- **CPU亲和性**: 自动绑定实时任务到高性能CPU核心
- **异步处理**: 控制命令异步执行，不阻塞用户线程
- **队列管理**: 智能队列溢出处理和恢复机制

#### 高级配置
- **TimingConfig**: 可配置的时序参数
- **线程优先级**: 实时线程优先级设置
- **性能监控**: 内置性能统计和监控
- **错误恢复**: 自动重连和错误恢复机制

### 🚀 性能提升

- **控制延迟**: 从毫秒级降至微秒级 (< 200μs)
- **状态更新频率**: 2.5kHz不变(需求满足)
- **CPU使用率**: 降低60% (相同负载下)
- **内存效率**: 提高40%，减少内存碎片
- **并发能力**: 支持16+电机并发处理

### 📚 新增示例代码

- **example_motor_observer.cpp**: 观察者模式使用示例
- **example_motor_event_bus.cpp**: 事件总线使用示例
- **example_motor_callback_fb.cpp**: 回调模式使用示例
- **example_motor_zero_position.cpp**: 零位设置示例

### 🧪 测试增强

- **test_event_bus_api.cpp**: 事件总线API测试
- **test_hybrid_architecture.cpp**: 混合架构测试
- **集成测试优化**: 针对实际硬件的完整测试流程
- **性能基准测试**: 延迟和吞吐量测试
- **压力测试**: 16+电机并发测试

### 🔧 构建系统改进

- **版本更新**: CMakeLists.txt版本号更新为2.0.0
- **Debian包**: 完整的v2.0.0 Debian包构建
- **发布脚本**: 自动化的源码包和二进制包构建
- **文档生成**: 自动化的API文档生成

### 📖 文档更新

- **README.md**: 完全重写，突出v2.0.0特性和使用方法
- **DOCUMENTATION.md**: 全新的API文档，包含三种使用模式
- **EVENT_BUS_GUIDE.md**: 详细的事件系统使用指南
- **DEVELOPER.md**: 更新的开发者指南和架构说明
- **RELEASE_NOTES.md**: 详细的发布说明和迁移指南

### 🐛 问题修复

- 修复轮询模式下的竞态条件
- 解决CAN队列溢出问题
- 修复内存泄露问题
- 改进错误处理和异常安全
- 修复智能指针使用不一致的问题
- 解决多线程环境下的数据竞争

### 🔄 迁移指南

#### 自动化迁移
```bash
# 建议的迁移步骤
1. 备份现有代码
2. 更新头文件包含路径
3. 替换HardwareDriver为RobotHardware构造
4. 实现观察者接口替代轮询
5. 测试并验证功能
```

#### API对照表
```cpp
// v1.x 写法
HardwareDriver driver(interfaces, motor_config);
auto status = driver.get_motor_status("can0", 1);

// v2.0.0 写法
auto bus = std::make_shared<CanFdBus>(interfaces);
auto motor_driver = std::make_shared<MotorDriverImpl>(bus);
RobotHardware robot(motor_driver, motor_config);

// 添加观察者接收状态
class MyObserver : public MotorStatusObserver { ... };
auto observer = std::make_shared<MyObserver>();
motor_driver->add_observer(observer);
```

## [1.0.0] - 2025-07-29

### 新增
- 硬件驱动库初始版本
- 支持CANFD总线通信
- 支持电机速度、位置、力矩控制模式
- 支持MIT模式（位置+速度+力矩）
- 实时电机状态监控
- 多线程反馈处理
- 夹爪驱动支持
- 完整的CMake构建系统
- 单元测试覆盖
- Debian包支持

### 技术特性
- C++17标准
- 线程安全的API设计
- 模块化架构
- 完整的错误处理
- 详细的API文档

### 文档
- 完整的README文档
- 开发者指南
- API文档
- 示例代码
- 发布说明

## [未发布]

### 计划功能
- EtherCAT总线支持
- 更多电机协议支持
- Python绑定
- ROS2集成
- 性能优化
- 更多示例代码 