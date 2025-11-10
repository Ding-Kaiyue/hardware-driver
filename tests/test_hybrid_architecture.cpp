#include <gtest/gtest.h>
#include "driver/motor_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"
#include "hardware_driver/event/event_bus.hpp"
#include "hardware_driver/event/motor_events.hpp"
#include <memory>
#include <vector>
#include <atomic>
#include <chrono>
#include <thread>

using namespace hardware_driver;
using namespace hardware_driver::motor_driver;
using namespace hardware_driver::event;
using namespace hardware_driver::bus;

// 实时观察者 - 用于实时响应
class RealtimeMotorObserver : public MotorEventHandler {
public:
    std::atomic<int> individual_updates{0};
    std::atomic<int> batch_updates{0};
    std::atomic<int> function_results{0};
    std::atomic<int> parameter_results{0};

    void on_motor_status_update(const std::string& /* interface */,
                           uint32_t /* motor_id */,
                           const Motor_Status& /* status */) override {
        individual_updates++;
    }

    void on_motor_status_update(const std::string& /* interface */,
                           const std::map<uint32_t, Motor_Status>& /* status_all */) override {
        batch_updates++;
    }

    void on_motor_function_result(const std::string& /* interface */,
                              uint32_t /* motor_id */,
                              uint8_t /* op_code */,
                              bool /* success */) override {
        function_results++;
    }

    void on_motor_parameter_result(const std::string& /* interface */,
                               uint32_t /* motor_id */,
                               uint16_t /* address */,
                               uint8_t /* data_type */,
                               const std::any& /* data */) override {
        parameter_results++;
    }
};

// 事件总线订阅者 - 用于非实时处理
class EventBusSubscriber {
public:
    std::atomic<int> status_events{0};
    std::atomic<int> batch_events{0};
    std::atomic<int> function_events{0};
    std::atomic<int> parameter_events{0};

    std::shared_ptr<EventHandler> status_handler;
    std::shared_ptr<EventHandler> batch_handler;
    std::shared_ptr<EventHandler> function_handler;
    std::shared_ptr<EventHandler> parameter_handler;

    void subscribe(std::shared_ptr<EventBus> bus) {
        // 订阅单个电机状态
        status_handler = bus->subscribe<MotorStatusEvent>(
            [this](const std::shared_ptr<MotorStatusEvent>& /* event */) {
                status_events++;
            });

        // 订阅批量电机状态
        batch_handler = bus->subscribe<MotorBatchStatusEvent>(
            [this](const std::shared_ptr<MotorBatchStatusEvent>& /* event */) {
                batch_events++;
            });

        // 订阅函数结果
        function_handler = bus->subscribe<MotorFunctionResultEvent>(
            [this](const std::shared_ptr<MotorFunctionResultEvent>& /* event */) {
                function_events++;
            });

        // 订阅参数结果
        parameter_handler = bus->subscribe<MotorParameterResultEvent>(
            [this](const std::shared_ptr<MotorParameterResultEvent>& /* event */) {
                parameter_events++;
            });
    }

    void unsubscribe(std::shared_ptr<EventBus> bus) {
        if (status_handler) {
            bus->unsubscribe<MotorStatusEvent>(status_handler);
        }
        if (batch_handler) {
            bus->unsubscribe<MotorBatchStatusEvent>(batch_handler);
        }
        if (function_handler) {
            bus->unsubscribe<MotorFunctionResultEvent>(function_handler);
        }
        if (parameter_handler) {
            bus->unsubscribe<MotorParameterResultEvent>(parameter_handler);
        }
    }
};

class HybridArchitectureTest : public ::testing::Test {
protected:
    void SetUp() override {
        event_bus_ = std::make_shared<EventBus>();
        realtime_observer_ = std::make_shared<RealtimeMotorObserver>();
        event_subscriber_ = std::make_unique<EventBusSubscriber>();
        event_subscriber_->subscribe(event_bus_);
    }

    void TearDown() override {
        if (event_subscriber_) {
            event_subscriber_->unsubscribe(event_bus_);
        }
        event_subscriber_.reset();
        realtime_observer_.reset();
        event_bus_.reset();
    }

    std::shared_ptr<EventBus> event_bus_;
    std::shared_ptr<RealtimeMotorObserver> realtime_observer_;
    std::unique_ptr<EventBusSubscriber> event_subscriber_;

    // 创建测试用的电机状态
    Motor_Status create_test_status(float position = 0.0f, float velocity = 0.0f) {
        Motor_Status status;
        status.enable_flag = 1;
        status.motor_mode = 4;
        status.position = position;
        status.velocity = velocity;
        status.effort = 0.5f;
        status.temperature = 250;
        status.limit_flag = 0;
        status.error_code = 0;
        return status;
    }
};

// 测试1：单个电机状态的混合处理
TEST_F(HybridArchitectureTest, IndividualMotorStatusUpdate) {
    // 创建电机状态
    Motor_Status status = create_test_status(45.0f, 6.0f);

    // 模拟实时观察者直接调用（高优先级）
    realtime_observer_->on_motor_status_update("can0", 1, status);

    // 模拟事件总线发布（低优先级）
    event_bus_->emit<MotorStatusEvent>("can0", 1, status);

    // 验证两个路径都收到了事件
    EXPECT_EQ(realtime_observer_->individual_updates.load(), 1);
    EXPECT_EQ(event_subscriber_->status_events.load(), 1);

    std::cout << "Individual motor status: realtime=" << realtime_observer_->individual_updates.load()
              << ", event_bus=" << event_subscriber_->status_events.load() << std::endl;
}

// 测试2：批量电机状态的混合处理
TEST_F(HybridArchitectureTest, BatchMotorStatusUpdate) {
    // 创建批量状态
    std::map<uint32_t, Motor_Status> batch_status;
    for (uint32_t id = 1; id <= 3; ++id) {
        batch_status[id] = create_test_status(id * 10.0f, id * 0.5f);
    }

    // 实时路径
    realtime_observer_->on_motor_status_update("can0", batch_status);

    // 事件总线路径
    event_bus_->emit<MotorBatchStatusEvent>("can0", batch_status);

    // 验证两个路径都收到了批量事件
    EXPECT_EQ(realtime_observer_->batch_updates.load(), 1);
    EXPECT_EQ(event_subscriber_->batch_events.load(), 1);

    std::cout << "Batch motor status: realtime=" << realtime_observer_->batch_updates.load()
              << ", event_bus=" << event_subscriber_->batch_events.load() << std::endl;
}

// 测试3：函数操作结果的混合传播
TEST_F(HybridArchitectureTest, FunctionResultPropagation) {
    // 实时路径
    realtime_observer_->on_motor_function_result("can0", 1, 0x03, true);  // 清除错误

    // 事件总线路径
    event_bus_->emit<MotorFunctionResultEvent>("can0", 1, 0x03, true);

    // 验证结果传播
    EXPECT_EQ(realtime_observer_->function_results.load(), 1);
    EXPECT_EQ(event_subscriber_->function_events.load(), 1);

    std::cout << "Function result: realtime=" << realtime_observer_->function_results.load()
              << ", event_bus=" << event_subscriber_->function_events.load() << std::endl;
}

// 测试4：参数操作结果的混合传播
TEST_F(HybridArchitectureTest, ParameterResultPropagation) {
    // 实时路径
    realtime_observer_->on_motor_parameter_result("can0", 1, 0x1000, 0x01, std::any(100));

    // 事件总线路径
    event_bus_->emit<MotorParameterResultEvent>("can0", 1, 0x1000, 0x01, std::any(100));

    // 验证参数结果传播
    EXPECT_EQ(realtime_observer_->parameter_results.load(), 1);
    EXPECT_EQ(event_subscriber_->parameter_events.load(), 1);

    std::cout << "Parameter result: realtime=" << realtime_observer_->parameter_results.load()
              << ", event_bus=" << event_subscriber_->parameter_events.load() << std::endl;
}

// 测试5：混合架构性能测试
TEST_F(HybridArchitectureTest, HybridArchitecturePerformance) {
    const int num_iterations = 100;
    const std::string interface = "can0";
    Motor_Status status = create_test_status(45.0f, 6.0f);

    // 测试实时观察者性能
    auto start_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_iterations; ++i) {
        realtime_observer_->on_motor_status_update(interface, 1, status);
    }

    auto realtime_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - start_time);

    // 测试事件总线性能
    start_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_iterations; ++i) {
        event_bus_->emit<MotorStatusEvent>(interface, 1, status);
    }

    auto eventbus_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - start_time);

    // 验证所有事件都被处理
    EXPECT_EQ(realtime_observer_->individual_updates.load(), num_iterations);
    EXPECT_EQ(event_subscriber_->status_events.load(), num_iterations);

    // 性能应该是合理的
    EXPECT_LT(realtime_duration.count(), 50000);  // < 50ms
    EXPECT_LT(eventbus_duration.count(), 100000);  // < 100ms

    std::cout << "Realtime observer: " << realtime_duration.count() << "μs for " << num_iterations << " updates\n";
    std::cout << "Event bus: " << eventbus_duration.count() << "μs for " << num_iterations << " updates\n";
    std::cout << "Overhead ratio: " << (static_cast<double>(eventbus_duration.count()) / realtime_duration.count())
              << "x\n";
}

// 测试6：批量处理效率
TEST_F(HybridArchitectureTest, BatchProcessingEfficiency) {
    const int num_motors = 6;
    const int num_batches = 50;

    // 创建批量数据
    std::map<uint32_t, Motor_Status> batch_status;
    for (uint32_t id = 1; id <= num_motors; ++id) {
        batch_status[id] = create_test_status(id * 10.0f, id * 0.5f);
    }

    // 测试实时观察者批量性能
    auto start_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_batches; ++i) {
        realtime_observer_->on_motor_status_update("can0", batch_status);
    }

    auto realtime_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - start_time);

    // 测试事件总线批量性能
    start_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_batches; ++i) {
        event_bus_->emit<MotorBatchStatusEvent>("can0", batch_status);
    }

    auto eventbus_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - start_time);

    // 验证所有批量事件都被处理
    EXPECT_EQ(realtime_observer_->batch_updates.load(), num_batches);
    EXPECT_EQ(event_subscriber_->batch_events.load(), num_batches);

    std::cout << "Batch realtime: " << realtime_duration.count() << "μs for " << num_batches
              << " batches × " << num_motors << " motors\n";
    std::cout << "Batch event bus: " << eventbus_duration.count() << "μs for " << num_batches
              << " batches × " << num_motors << " motors\n";
}

// 测试7：多事件类型的混合处理
TEST_F(HybridArchitectureTest, MixedEventTypeHandling) {
    Motor_Status status = create_test_status(30.0f, 5.0f);

    // 发送多种事件
    realtime_observer_->on_motor_status_update("can0", 1, status);
    realtime_observer_->on_motor_function_result("can0", 1, 0x04, true);
    realtime_observer_->on_motor_parameter_result("can0", 1, 0x1000, 0x01, std::any(100));

    event_bus_->emit<MotorStatusEvent>("can0", 1, status);
    event_bus_->emit<MotorFunctionResultEvent>("can0", 1, 0x04, true);
    event_bus_->emit<MotorParameterResultEvent>("can0", 1, 0x1000, 0x01, std::any(100));

    // 验证所有事件都被处理
    EXPECT_EQ(realtime_observer_->individual_updates.load(), 1);
    EXPECT_EQ(realtime_observer_->function_results.load(), 1);
    EXPECT_EQ(realtime_observer_->parameter_results.load(), 1);

    EXPECT_EQ(event_subscriber_->status_events.load(), 1);
    EXPECT_EQ(event_subscriber_->function_events.load(), 1);
    EXPECT_EQ(event_subscriber_->parameter_events.load(), 1);

    std::cout << "Mixed events processed successfully\n";
}

// 测试8：多订阅者支持
TEST_F(HybridArchitectureTest, MultipleSubscriberSupport) {
    // 创建额外的订阅者
    auto extra_subscriber = std::make_unique<EventBusSubscriber>();
    extra_subscriber->subscribe(event_bus_);

    Motor_Status status = create_test_status(45.0f, 6.0f);

    // 发送事件
    event_bus_->emit<MotorStatusEvent>("can0", 1, status);

    // 两个订阅者都应该收到事件
    EXPECT_EQ(event_subscriber_->status_events.load(), 1);
    EXPECT_EQ(extra_subscriber->status_events.load(), 1);

    extra_subscriber->unsubscribe(event_bus_);
    std::cout << "Multiple subscribers: both received 1 event\n";
}

// 测试9：事件优先级处理
TEST_F(HybridArchitectureTest, EventPriorityHandling) {
    Motor_Status status = create_test_status(30.0f, 5.0f);

    // 实时路径应该立即处理
    realtime_observer_->on_motor_status_update("can0", 1, status);
    EXPECT_EQ(realtime_observer_->individual_updates.load(), 1);

    // 事件总线路径可能有延迟
    event_bus_->emit<MotorStatusEvent>("can0", 2, status);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_EQ(event_subscriber_->status_events.load(), 1);

    std::cout << "Priority handling: realtime immediate, event bus processed\n";
}

// 测试10：实际应用场景 - 电机控制流程
TEST_F(HybridArchitectureTest, RealWorldMotorControlFlow) {
    // 模拟一个完整的电机控制流程

    // 1. 启用电机（实时处理）
    Motor_Status enabled_status = create_test_status(0.0f, 0.0f);
    enabled_status.enable_flag = 1;
    realtime_observer_->on_motor_status_update("can0", 1, enabled_status);

    // 2. 发送控制命令（事件总线）
    Motor_Status moving_status = create_test_status(0.0f, 6.0f);
    event_bus_->emit<MotorStatusEvent>("can0", 1, moving_status);

    // 3. 执行函数操作（两路并行）
    realtime_observer_->on_motor_function_result("can0", 1, 0x03, true);
    event_bus_->emit<MotorFunctionResultEvent>("can0", 1, 0x03, true);

    // 4. 停止电机（实时处理）
    Motor_Status stopped_status = create_test_status(0.0f, 0.0f);
    realtime_observer_->on_motor_status_update("can0", 1, stopped_status);

    // 验证整个流程
    EXPECT_EQ(realtime_observer_->individual_updates.load(), 2);  // 2次状态更新
    EXPECT_EQ(realtime_observer_->function_results.load(), 1);    // 1次函数结果

    EXPECT_EQ(event_subscriber_->status_events.load(), 1);        // 1次事件总线状态
    EXPECT_EQ(event_subscriber_->function_events.load(), 1);      // 1次事件总线函数

    std::cout << "Real-world motor control flow: completed successfully\n";
}
