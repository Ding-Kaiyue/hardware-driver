#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <iomanip>
#include "hardware_driver/interface/robot_hardware.hpp"

/**
 * 异步轨迹执行使用示例
 *
 * 演示如何：
 * 1. 异步执行轨迹
 * 2. 监听执行完成回调
 * 3. 监听进度更新回调
 * 4. 暂停/恢复轨迹执行
 * 5. 取消轨迹执行
 */

// 辅助函数：创建示例轨迹
Trajectory create_example_trajectory(double duration_seconds, size_t num_points) {
    Trajectory trajectory;
    trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    double time_interval = duration_seconds / (num_points - 1);

    for (size_t i = 0; i < num_points; ++i) {
        TrajectoryPoint point;
        point.time_from_start = i * time_interval;

        // 简单的正弦波轨迹
        double phase = (i * M_PI) / (num_points - 1);
        for (size_t j = 0; j < 6; ++j) {
            point.positions.push_back(std::sin(phase) * (j + 1) * 0.5);  // 0.5-3.0 rad
            point.velocities.push_back(0.0);
            point.accelerations.push_back(0.0);
        }

        trajectory.points.push_back(point);
    }

    return trajectory;
}

int main() {
    // 创建电机驱动（这里使用工厂函数）
    std::vector<std::string> interfaces = {"can0"};  // 根据实际情况修改
    auto motor_driver = hardware_driver::createCanFdMotorDriver(interfaces);

    // 配置接口和电机ID
    std::map<std::string, std::vector<uint32_t>> interface_motor_config;
    interface_motor_config["can0"] = {1, 2, 3, 4, 5, 6};

    // 创建RobotHardware实例
    RobotHardware robot_hardware(motor_driver, interface_motor_config);

    std::cout << "\n========== 异步轨迹执行示例 ==========" << std::endl;

    // ============ 示例1：基本的异步执行 ============
    std::cout << "\n【示例1】基本的异步轨迹执行（内置进度条）" << std::endl;
    {
        auto trajectory = create_example_trajectory(5.0, 50);  // 5秒，50个点

        // 不提供回调，使用内置进度条显示
        auto exec_id = robot_hardware.execute_trajectory_async(
            "can0",
            trajectory
        );

        if (exec_id.empty()) {
            std::cout << "错误：无法启动轨迹执行" << std::endl;
        } else {
            std::cout << "✓ 轨迹执行已启动 [ID: " << exec_id << "]" << std::endl;

            // 等待执行完成
            robot_hardware.wait_for_completion(exec_id);
            std::cout << "\n✓ 轨迹执行完成" << std::endl;
        }
    }

    // ============ 示例2：不显示进度条的执行 ============
    std::cout << "\n【示例2】不显示进度条的执行（show_progress=false）" << std::endl;
    {
        auto trajectory = create_example_trajectory(3.0, 30);  // 3秒，30个点

        // show_progress=false，不显示进度条
        auto exec_id = robot_hardware.execute_trajectory_async(
            "can0",
            trajectory,
            false  // 不显示进度条
        );

        if (!exec_id.empty()) {
            std::cout << "✓ 轨迹执行中（后台运行，无进度显示）..." << std::endl;
            robot_hardware.wait_for_completion(exec_id);
            std::cout << "✓ 轨迹执行完成" << std::endl;
        }
    }

    // ============ 示例3：暂停和恢复 ============
    std::cout << "\n【示例3】暂停和恢复执行" << std::endl;
    {
        auto trajectory = create_example_trajectory(10.0, 100);  // 10秒，100个点

        // 不提供回调，使用内置进度条显示
        auto exec_id = robot_hardware.execute_trajectory_async(
            "can0",
            trajectory
        );

        if (!exec_id.empty()) {
            // 执行2秒后暂停
            std::this_thread::sleep_for(std::chrono::seconds(2));
            std::cout << "  ⏸ 暂停执行..." << std::endl;
            robot_hardware.pause_trajectory(exec_id);

            // 暂停2秒
            std::this_thread::sleep_for(std::chrono::seconds(2));

            // 恢复执行
            std::cout << "  ▶ 恢复执行..." << std::endl;
            robot_hardware.resume_trajectory(exec_id);

            // 等待完成
            robot_hardware.wait_for_completion(exec_id);
        }
    }

    // ============ 示例4：取消执行 ============
    std::cout << "\n【示例4】取消执行" << std::endl;
    {
        auto trajectory = create_example_trajectory(10.0, 100);

        // 不提供回调，使用内置进度条显示
        auto exec_id = robot_hardware.execute_trajectory_async(
            "can0",
            trajectory
        );

        if (!exec_id.empty()) {
            // 执行3秒后取消
            std::this_thread::sleep_for(std::chrono::seconds(3));
            std::cout << "  ⊗ 取消执行..." << std::endl;
            robot_hardware.cancel_trajectory(exec_id);

            // 等待取消完成
            robot_hardware.wait_for_completion(exec_id);
        }
    }

    // ============ 示例5：查询执行状态 ============
    std::cout << "\n【示例5】查询执行状态" << std::endl;
    {
        auto trajectory = create_example_trajectory(5.0, 50);

        // 不提供回调，使用内置进度条显示
        auto exec_id = robot_hardware.execute_trajectory_async(
            "can0",
            trajectory
        );

        if (!exec_id.empty()) {
            // 在执行过程中查询状态
            for (int i = 0; i < 3; ++i) {
                std::this_thread::sleep_for(std::chrono::seconds(1));

                TrajectoryExecutionProgress progress;
                if (robot_hardware.get_execution_progress(exec_id, progress)) {
                    std::cout << "  状态查询 #" << (i + 1) << ":" << std::endl;
                    std::cout << "    进度: " << progress.progress_percentage << "%" << std::endl;
                    std::cout << "    当前点: " << progress.current_point_index << "/"
                              << progress.total_points << std::endl;
                    std::cout << "    已用时间: " << progress.elapsed_time.count() << "ms" << std::endl;
                    std::cout << "    剩余时间: " << progress.estimated_remaining_time.count() << "ms" << std::endl;
                }
            }

            robot_hardware.wait_for_completion(exec_id);
        }
    }

    // ============ 示例6：活跃任务管理 ============
    std::cout << "\n【示例6】活跃任务管理（同接口单轨迹约束）" << std::endl;
    {
        // 创建一条较长的轨迹确保轨迹1还在运行时尝试启动轨迹2
        auto trajectory1 = create_example_trajectory(10.0, 100);  // 10秒的轨迹
        auto trajectory2 = create_example_trajectory(3.0, 30);

        // 启动轨迹1
        auto exec_id1 = robot_hardware.execute_trajectory_async("can0", trajectory1);
        std::cout << "  启动轨迹1: " << (exec_id1.empty() ? "失败" : "成功") << std::endl;

        // 给执行线程一点时间来启动（确保状态从IDLE变为RUNNING）
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 立即尝试在同一接口启动轨迹2，这会失败（因为轨迹1还在运行）
        auto exec_id2 = robot_hardware.execute_trajectory_async("can0", trajectory2);
        std::cout << "  启动轨迹2 (应该失败): " << (exec_id2.empty() ? "失败 ✓" : "成功 ✗") << std::endl;

        // 查询活跃任务（轨迹1应该还在运行）
        auto active_ids = robot_hardware.get_active_execution_ids();
        std::cout << "  活跃任务数: " << active_ids.size() << " (应该是1)" << std::endl;

        std::cout << "  活跃任务ID列表:" << std::endl;
        for (const auto& id : active_ids) {
            std::cout << "    - " << id << std::endl;
        }

        // 等待轨迹1完成
        if (!exec_id1.empty()) {
            robot_hardware.wait_for_completion(exec_id1);
        }

        std::cout << "\n✓ 示例6完成" << std::endl;
    }

    std::cout << "\n========== 示例执行完成 ==========" << std::endl;

    return 0;
}

/**
 * 编译说明：
 * g++ -std=c++17 -I/path/to/include \
 *     async_trajectory_example.cpp \
 *     /path/to/lib/libhardware_driver.a \
 *     -pthread -o async_trajectory_example
 *
 * 关键特性：
 * ✓ 非阻塞异步执行 - 返回execution_id后立即返回
 * ✓ 回调通知 - 完成和进度更新都通过回调
 * ✓ 暂停/恢复 - 支持暂停后恢复，时间会自动调整
 * ✓ 取消操作 - 可以随时取消正在执行的轨迹
 * ✓ 状态查询 - 可以查询当前执行进度和估计剩余时间
 * ✓ 单接口单轨迹 - 同一接口同时只能执行一条轨迹
 * ✓ 线程安全 - 所有操作都是线程安全的
 * ✓ 精确时序 - 保持原有的时序精度
 */
