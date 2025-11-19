# 异步轨迹执行使用指南

## 核心概念

**单接口单轨迹** - 同一接口同时只能执行一条轨迹，不同接口可以并行执行各自的轨迹。

## 快速开始

### 基本使用

```cpp
#include "hardware_driver/interface/robot_hardware.hpp"

// 创建RobotHardware
auto motor_driver = hardware_driver::createCanFdMotorDriver({"can0"});
std::map<std::string, std::vector<uint32_t>> config;
config["can0"] = {1, 2, 3, 4, 5, 6};
RobotHardware robot(motor_driver, config);

// 创建轨迹
Trajectory trajectory;
trajectory.joint_names = {"motor_1", "motor_2", "motor_3", "motor_4", "motor_5", "motor_6"};
// 添加轨迹点...

// 异步执行
auto exec_id = robot.execute_trajectory_async(
    "can0",
    trajectory,
    [](const std::string& id, bool success, const std::string& error) {
        if (success) {
            std::cout << "执行完成!" << std::endl;
        } else {
            std::cout << "执行失败: " << error << std::endl;
        }
    }
);

if (!exec_id.empty()) {
    std::cout << "轨迹已启动，ID: " << exec_id << std::endl;
} else {
    std::cout << "启动失败" << std::endl;
}
```

## 主要方法

| 方法 | 功能 | 返回值 |
|------|------|--------|
| `execute_trajectory_async` | 异步执行轨迹 | execution_id（空表示失败） |
| `pause_trajectory` | 暂停执行 | bool |
| `resume_trajectory` | 恢复执行 | bool |
| `cancel_trajectory` | 取消执行 | bool |
| `get_execution_progress` | 查询进度 | bool |
| `wait_for_completion` | 等待完成 | bool |
| `pause_all_trajectories` | 暂停全部 | void |
| `resume_all_trajectories` | 恢复全部 | void |
| `cancel_all_trajectories` | 取消全部 | void |
| `get_active_trajectory_count` | 活跃任务数 | size_t |
| `enable_trajectory_progress_display` | 启用进度显示 | void |

## 常见用例

### 1. 简单执行

```cpp
auto exec_id = robot.execute_trajectory_async(
    "can0",
    trajectory,
    [](const std::string& id, bool success, const std::string& error) {
        std::cout << (success ? "成功" : "失败") << std::endl;
    }
);
```

### 2. 带进度回调

```cpp
auto exec_id = robot.execute_trajectory_async(
    "can0",
    trajectory,
    [](const std::string& id, bool success, const std::string& error) {
        // 完成回调
    },
    [](const std::string& id, const TrajectoryExecutionProgress& progress) {
        printf("进度: %.1f%% (%zu/%zu)\n",
               progress.progress_percentage,
               progress.current_point_index,
               progress.total_points);
    },
    500  // 每500ms更新一次
);
```

### 3. 暂停和恢复

```cpp
auto exec_id = robot.execute_trajectory_async(
    "can0", trajectory,
    [](const std::string& id, bool success, const std::string& error) {}
);

// 2秒后暂停
std::this_thread::sleep_for(std::chrono::seconds(2));
robot.pause_trajectory(exec_id);

// 暂停2秒
std::this_thread::sleep_for(std::chrono::seconds(2));

// 恢复
robot.resume_trajectory(exec_id);

// 等待完成
robot.wait_for_completion(exec_id);
```

### 4. 查询进度

```cpp
TrajectoryExecutionProgress progress;
if (robot.get_execution_progress(exec_id, progress)) {
    std::cout << "进度: " << progress.progress_percentage << "%" << std::endl;
    std::cout << "耗时: " << progress.elapsed_time.count() << "ms" << std::endl;
    std::cout << "剩余: " << progress.estimated_remaining_time.count() << "ms" << std::endl;
}
```

### 5. 多接口并行执行

```cpp
// 创建两个接口
auto motor_driver = hardware_driver::createCanFdMotorDriver({"can0", "can1"});
std::map<std::string, std::vector<uint32_t>> config;
config["can0"] = {1, 2, 3};
config["can1"] = {4, 5, 6};
RobotHardware robot(motor_driver, config);

// 在两个接口上并行执行不同的轨迹
auto exec1 = robot.execute_trajectory_async("can0", traj1, callback1);
auto exec2 = robot.execute_trajectory_async("can1", traj2, callback2);

// 两个轨迹在不同接口上并行执行
robot.wait_for_completion(exec1);
robot.wait_for_completion(exec2);
```

## 关键约束

### ❌ 错误用法 - 同接口同时执行两条轨迹

```cpp
auto exec1 = robot.execute_trajectory_async("can0", traj1, callback);
auto exec2 = robot.execute_trajectory_async("can0", traj2, callback);  // 会失败！返回空字符串

// exec2是空的，不能用于后续操作
if (exec2.empty()) {
    std::cerr << "错误：同接口无法同时执行两条轨迹" << std::endl;
}
```

### ✅ 正确用法 - 等待第一条完成后再执行第二条

```cpp
auto exec1 = robot.execute_trajectory_async("can0", traj1,
    [&robot, &traj2](const std::string& id, bool success, const std::string& error) {
        if (success) {
            // 第一条完成后启动第二条
            robot.execute_trajectory_async("can0", traj2,
                [](const std::string& id, bool success, const std::string& error) {
                    // 第二条完成
                }
            );
        }
    }
);
```

## 状态转换图

```
    IDLE
      ↓
   RUNNING ← → PAUSED
      ↓           ↓
      COMPLETED   CANCELLED
          ↓       ↓
          ERROR (exception)
```

## 进度信息

```cpp
struct TrajectoryExecutionProgress {
    TrajectoryExecutionState state;                      // 当前状态
    size_t current_point_index;                          // 当前点索引
    size_t total_points;                                 // 总点数
    double progress_percentage;                          // 进度百分比 (0-100)
    std::chrono::milliseconds elapsed_time;              // 已用时间
    std::chrono::milliseconds estimated_remaining_time;  // 估计剩余时间
    std::string error_message;                           // 错误信息
};
```

## 线程安全性

所有公有方法都是线程安全的，可以从多个线程调用：

```cpp
// 线程1：执行轨迹
std::thread t1([&]() {
    auto exec_id = robot.execute_trajectory_async("can0", traj, callback);
});

// 线程2：查询进度
std::thread t2([&]() {
    TrajectoryExecutionProgress progress;
    robot.get_execution_progress(exec_id, progress);
});

// 线程3：暂停/恢复
std::thread t3([&]() {
    robot.pause_trajectory(exec_id);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    robot.resume_trajectory(exec_id);
});

t1.join();
t2.join();
t3.join();
```

## 常见问题

**Q: 为什么返回空的execution_id？**

A: 检查以下几点：
- 接口名称是否正确
- 轨迹是否为空
- 该接口是否已有运行中的轨迹

**Q: 暂停后恢复，轨迹时序会乱吗？**

A: 不会。框架会自动计算暂停时间并调整时间基准。

**Q: 可以从回调函数中调用其他方法吗？**

A: 可以，但避免调用wait_for_completion等阻塞方法。

**Q: 执行完成后execution_id还能用吗？**

A: 可以用get_execution_progress查询历史状态，但不能暂停/恢复。

## 完整示例

详见 `examples/async_trajectory_example.cpp`
