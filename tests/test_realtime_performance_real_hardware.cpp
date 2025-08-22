#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <map>

#include "performance_test_framework.hpp"
#include "driver/motor_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"
#include "hardware_driver/interface/robot_hardware.hpp"
#include "hardware_driver/event/event_bus.hpp"

using namespace hardware_driver;
using namespace hardware_driver::motor_driver;
using namespace hardware_driver::bus;
using namespace performance_test;

// 真实硬件延迟测量观察者
class RealHardwareLatencyObserver : public MotorStatusObserver {
public:
    std::map<std::string, std::map<uint32_t, Motor_Status>> status_map;
    std::mutex status_mutex;
    std::atomic<int> status_update_count{0};
    
    // 延迟测量相关
    std::vector<double> notification_latencies_;
    std::mutex latency_mutex_;
    
    void on_motor_status_update(const std::string& interface, 
                               uint32_t motor_id, 
                               const Motor_Status& status) override {
        std::lock_guard<std::mutex> lock(status_mutex);
        status_map[interface][motor_id] = status;
        status_update_count++;
        
        // 记录通知延迟
        auto now = Clock::now();
        if (packet_timestamp_ != TimePoint{}) {
            auto latency = std::chrono::duration<double, std::micro>(now - packet_timestamp_).count();
            std::lock_guard<std::mutex> latency_lock(latency_mutex_);
            notification_latencies_.push_back(latency);
        }
    }
    
    void set_packet_timestamp(TimePoint timestamp) {
        packet_timestamp_ = timestamp;
    }
    
    std::vector<double> get_notification_latencies() const {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(latency_mutex_));
        return notification_latencies_;
    }
    
    void clear_latencies() {
        std::lock_guard<std::mutex> lock(latency_mutex_);
        notification_latencies_.clear();
    }
    
private:
    TimePoint packet_timestamp_;
};

// 真实硬件性能测试夹具
class RealHardwarePerformanceTestFixture : public ::testing::Test, public PerformanceTestBase {
protected:
    void SetUp() override {
        // 检查是否有真实的CAN硬件
        if (!has_real_can_hardware()) {
            GTEST_SKIP() << "跳过真实硬件测试: 未检测到CAN接口";
        }
        
        // 创建真实的CAN-FD总线
        std::vector<std::string> interfaces = {"can0"};
        canfd_bus_ = std::make_shared<CanFdBus>(interfaces);
        canfd_bus_->init();
        
        // 创建电机驱动
        motor_driver_ = std::make_shared<MotorDriverImpl>(canfd_bus_);
        
        // 创建事件总线
        event_bus_ = std::make_shared<event::EventBus>();
        motor_driver_->set_event_bus(event_bus_);
        
        // 创建并注册延迟观察者
        latency_observer_ = std::make_shared<RealHardwareLatencyObserver>();
        motor_driver_->add_observer(latency_observer_);
        
        // 配置电机 - 使用真实硬件的电机ID (1和9)
        std::map<std::string, std::vector<uint32_t>> motor_config = {
            {"can0", {1, 9}}
        };
        motor_driver_->set_motor_config(motor_config);
        
        // 创建机器人硬件接口
        robot_hardware_ = std::make_shared<RobotHardware>(motor_driver_, motor_config, event_bus_);
        
        // 等待系统初始化
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "🔌 真实硬件性能测试已初始化，CAN接口: can0，电机ID: 1, 9" << std::endl;
    }
    
    void TearDown() override {
        // 清理资源
        robot_hardware_.reset();
        motor_driver_.reset();
        canfd_bus_.reset();
        
        std::cout << "🧹 真实硬件性能测试清理完成" << std::endl;
    }
    
    // 检查是否有真实的CAN硬件
    static bool has_real_can_hardware() {
        return system("ip link show can0 2>/dev/null | grep -q 'can0'") == 0;
    }
    
    // 测试真实硬件控制延迟
    PerformanceStats run_real_hardware_control_latency_test() {
        const int iterations = 500;  // 减少迭代次数，避免对真实硬件造成过大负载
        std::cout << "🔄 开始真实硬件控制延迟测试，迭代次数: " << iterations << std::endl;
        
        for (int i = 0; i < iterations; i++) {
            // 记录命令开始时间
            auto command_start = Clock::now();
            
            // 发送控制命令到电机1
            robot_hardware_->control_motor_in_velocity_mode("can0", 1, 10.0f);
            
            // 等待一小段时间，让命令通过CAN总线
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            
            // 计算延迟
            auto command_end = Clock::now();
            double latency = std::chrono::duration<double, std::micro>(command_end - command_start).count();
            record_latency(latency);
            
            // 控制测试频率，避免过载
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            
            // 每50次迭代输出进度
            if (i % 50 == 0) {
                std::cout << "." << std::flush;
            }
        }
        std::cout << " 完成!" << std::endl;
        
        return calculate_stats();
    }
    
    // 测试真实硬件反馈延迟
    PerformanceStats run_real_hardware_feedback_latency_test() {
        const int iterations = 200;
        std::cout << "🔄 开始真实硬件反馈延迟测试，迭代次数: " << iterations << std::endl;
        
        latency_observer_->clear_latencies();
        
        for (int i = 0; i < iterations; i++) {
            // 记录请求开始时间
            latency_observer_->set_packet_timestamp(Clock::now());
            
            // 发送参数读取命令，触发反馈
            robot_hardware_->motor_parameter_read("can0", 1, 0x1001);
            
            // 等待反馈
            int initial_count = latency_observer_->status_update_count;
            auto timeout = Clock::now() + std::chrono::milliseconds(10);
            
            while (latency_observer_->status_update_count == initial_count && 
                   Clock::now() < timeout) {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
            
            // 控制测试频率
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            
            if (i % 20 == 0) {
                std::cout << "." << std::flush;
            }
        }
        std::cout << " 完成!" << std::endl;
        
        // 使用观察者的延迟数据
        auto observer_latencies = latency_observer_->get_notification_latencies();
        latencies_.clear();
        latencies_ = observer_latencies;
        
        return calculate_stats();
    }
    
    // 测试双电机同步控制延迟
    PerformanceStats run_dual_motor_sync_test() {
        const int iterations = 200;
        std::cout << "🔄 开始双电机同步控制延迟测试，迭代次数: " << iterations << std::endl;
        
        for (int i = 0; i < iterations; i++) {
            auto start = Clock::now();
            
            // 发送双电机同步位置控制命令
            std::vector<double> positions = {i * 0.1, i * 0.2};
            robot_hardware_->send_realtime_position_command("can0", positions);
            
            auto end = Clock::now();
            double latency = std::chrono::duration<double, std::micro>(end - start).count();
            record_latency(latency);
            
            // 控制测试频率
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            
            if (i % 20 == 0) {
                std::cout << "." << std::flush;
            }
        }
        std::cout << " 完成!" << std::endl;
        
        return calculate_stats();
    }
    
    // 测试真实CAN总线吞吐量
    PerformanceStats run_can_throughput_test() {
        const int iterations = 1000;
        const int commands_per_iteration = 10;
        std::cout << "🔄 开始CAN总线吞吐量测试，迭代次数: " << iterations << std::endl;
        
        auto start_time = Clock::now();
        
        for (int i = 0; i < iterations; i++) {
            auto iter_start = Clock::now();
            
            // 在一次迭代中发送多个命令
            for (int j = 0; j < commands_per_iteration; j++) {
                robot_hardware_->control_motor_in_velocity_mode("can0", 1, j * 1.0f);
                robot_hardware_->control_motor_in_velocity_mode("can0", 9, j * 1.0f);
            }
            
            auto iter_end = Clock::now();
            double latency = std::chrono::duration<double, std::micro>(iter_end - iter_start).count();
            record_latency(latency);
            
            // 控制整体频率
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            
            if (i % 100 == 0) {
                std::cout << "." << std::flush;
            }
        }
        
        auto end_time = Clock::now();
        double total_time = std::chrono::duration<double>(end_time - start_time).count();
        double commands_per_second = (iterations * commands_per_iteration * 2) / total_time;
        
        std::cout << " 完成!" << std::endl;
        std::cout << "总吞吐量: " << commands_per_second << " commands/sec" << std::endl;
        
        return calculate_stats();
    }
    
protected:
    std::shared_ptr<CanFdBus> canfd_bus_;
    std::shared_ptr<MotorDriverImpl> motor_driver_;
    std::shared_ptr<event::EventBus> event_bus_;
    std::shared_ptr<RealHardwareLatencyObserver> latency_observer_;
    std::shared_ptr<RobotHardware> robot_hardware_;

    // 强制停机所有电机
    void force_stop_all_motors() {
        if (robot_hardware_) {
            std::cout << "🛑 强制停机所有电机..." << std::endl;
            
            // 先设置速度为0
            robot_hardware_->control_motor_in_velocity_mode("can0", 1, 0.0f);
            robot_hardware_->control_motor_in_velocity_mode("can0", 9, 0.0f);
            
            // 等待速度命令生效
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // 发送紧急停止命令（如果有的话）
            // robot_hardware_->emergency_stop("can0");
            
            // 禁用电机
            robot_hardware_->disable_motor("can0", 1);
            robot_hardware_->disable_motor("can0", 9);
            
            // 再次等待确保禁用生效
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            std::cout << "✅ 电机停机完成" << std::endl;
        }
    }
};

// 真实硬件性能测试用例

// 1. 真实硬件控制延迟测试 - 目标: P99 <500μs, P95 <300μs, P50 <200μs (相对宽松，因为有真实硬件延迟)
TEST_F(RealHardwarePerformanceTestFixture, real_hardware_control_latency) {
    auto stats = run_real_hardware_control_latency_test();
    print_performance_report("real_hardware_control_latency", stats);
    save_performance_report("../real_hardware_performance_report.csv", "real_hardware_control_latency", stats);
    verify_performance_targets(stats, 500.0, 300.0, 200.0);
    clear_test_data();
}

// 2. 真实硬件反馈延迟测试 - 目标: P99 <5000μs, P95 <2000μs, P50 <1000μs (具身智能要求)
TEST_F(RealHardwarePerformanceTestFixture, real_hardware_feedback_latency) {
    // 基于参数反馈事件测量延迟
    std::atomic<bool> ready{false};
    std::mutex m;
    std::condition_variable cv;
    Clock::time_point start_tp;

    auto handler = event_bus_->subscribe<event::MotorParameterResultEvent>(
        [&](const std::shared_ptr<event::MotorParameterResultEvent>&) {
            auto end_tp = Clock::now();
            double latency_us = std::chrono::duration<double, std::micro>(end_tp - start_tp).count();
            record_latency(latency_us);
            {
                std::lock_guard<std::mutex> lk(m);
                ready.store(true, std::memory_order_relaxed);
            }
            cv.notify_one();
        }
    );

    const int iterations = 200;
    for (int i = 0; i < iterations; ++i) {
        {
            std::lock_guard<std::mutex> lk(m);
            ready.store(false, std::memory_order_relaxed);
            start_tp = Clock::now();
        }
        // 读一个存在的、响应较快的寄存器地址（示例0x1001）
        robot_hardware_->motor_parameter_read("can0", 1, 0x1001);

        std::unique_lock<std::mutex> lk(m);
        if (!cv.wait_for(lk, std::chrono::milliseconds(20), [&]{ return ready.load(std::memory_order_relaxed); })) {
            // 超时视为高延迟一次
            record_latency(20000.0);
        }
        lk.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    auto stats = calculate_stats();
    print_performance_report("real_hardware_feedback_latency", stats);
    save_performance_report("../real_hardware_performance_report.csv", "real_hardware_feedback_latency", stats);
    // 具身智能高实时性要求：P50<500us, P95<1000us, P99<2000us（真正的高性能具身智能标准）
    verify_performance_targets(stats, 2000.0, 1000.0, 500.0);
    clear_test_data();
}

// 3. 双电机同步控制延迟测试 - 目标: P99 <600μs, P95 <400μs, P50 <250μs
TEST_F(RealHardwarePerformanceTestFixture, dual_motor_sync) {
    auto stats = run_dual_motor_sync_test();
    print_performance_report("dual_motor_sync", stats);
    save_performance_report("../real_hardware_performance_report.csv", "dual_motor_sync", stats);
    verify_performance_targets(stats, 600.0, 400.0, 250.0);
    clear_test_data();
}

// 4. CAN总线吞吐量测试 - 目标: P99 <1000μs, P95 <800μs, P50 <500μs
TEST_F(RealHardwarePerformanceTestFixture, can_throughput) {
    auto stats = run_can_throughput_test();
    print_performance_report("can_throughput", stats);
    save_performance_report("../real_hardware_performance_report.csv", "can_throughput", stats);
    verify_performance_targets(stats, 1000.0, 800.0, 500.0);
    clear_test_data();
}

// 主函数
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    std::cout << "🚀 开始真实硬件性能测试..." << std::endl;
    
    // 检查硬件连接
    if (system("ip link show can0 2>/dev/null | grep -q 'can0'") != 0) {
        std::cout << "❌ 错误: 未检测到can0接口" << std::endl;
        std::cout << "请确保:" << std::endl;
        std::cout << "  1. CAN硬件已连接" << std::endl;
        std::cout << "  2. 运行 'sudo ip link set can0 up'" << std::endl;
        std::cout << "  3. 检查 'ip link show can0'" << std::endl;
        return 1;
    }
    
    std::cout << "✅ 检测到CAN接口can0，连接的电机ID: 1, 9" << std::endl;
    
    // 创建性能报告文件头
    std::ofstream report_file("real_hardware_performance_report.csv");
    if (report_file.is_open()) {
        report_file << "Test Name,Sample Count,Min Latency,Max Latency,Mean Latency,Std Deviation,P50 Latency,P95 Latency,P99 Latency\n";
        report_file.close();
    }
    
    int result = RUN_ALL_TESTS();
    
    std::cout << "\n✅ 真实硬件性能测试完成！" << std::endl;
    std::cout << "详细报告已保存到: real_hardware_performance_report.csv" << std::endl;
    
    // 全局测试结束后强制停机所有电机
    std::cout << "\n🛑 测试完成，强制停机所有电机..." << std::endl;
    
    // 创建临时实例来执行停机
    try {
        std::vector<std::string> interfaces = {"can0"};
        auto temp_bus = std::make_shared<CanFdBus>(interfaces);
        auto temp_driver = std::make_shared<MotorDriverImpl>(temp_bus);
        
        // 创建电机配置
        std::map<std::string, std::vector<uint32_t>> motor_config;
        motor_config["can0"] = {1, 9};
        
        // 创建事件总线
        auto temp_event_bus = std::make_shared<event::EventBus>();
        
        auto temp_hardware = std::make_shared<RobotHardware>(temp_driver, motor_config, temp_event_bus);
        
        // 强制停机
        temp_hardware->control_motor_in_velocity_mode("can0", 1, 0.0f);
        temp_hardware->control_motor_in_velocity_mode("can0", 9, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        temp_hardware->disable_motor("can0", 1);
        temp_hardware->disable_motor("can0", 9);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        std::cout << "✅ 全局电机停机完成" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "⚠️ 全局停机时出现异常: " << e.what() << std::endl;
    }
    
    return result;
} 