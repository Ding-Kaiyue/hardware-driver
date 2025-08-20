#include <gtest/gtest.h>
#include "driver/motor_driver_impl.hpp"
#include "hardware_driver/event/event_bus.hpp"
#include "hardware_driver/event/motor_events.hpp"
#include "event/event_bridge.hpp"
#include <memory>
#include <vector>
#include <atomic>
#include <chrono>
#include <thread>

using namespace hardware_driver;
using namespace hardware_driver::motor_driver;
using namespace hardware_driver::event;
using namespace hardware_driver::bus;

// Mock bus interface for testing
class MockBusInterface : public BusInterface {
public:
    void init() override {}
    bool send(const GenericBusPacket& packet) override { 
        (void)packet; // 避免未使用参数警告
        return true; 
    }
    bool receive(GenericBusPacket& packet) override { 
        (void)packet; // 避免未使用参数警告
        return false; 
    }
    void async_receive(const std::function<void(const GenericBusPacket&)>& callback) override {
        (void)callback; // 避免未使用参数警告
    }
    std::vector<std::string> get_interface_names() const override { 
        return {"mock_can0"}; 
    }
};

// Test observer for real-time components
class TestRealtimeObserver : public MotorStatusObserver {
public:
    std::atomic<int> individual_updates{0};
    std::atomic<int> batch_updates{0};
    std::atomic<int> function_results{0};
    
    void on_motor_status_update(const std::string& interface, 
                               uint32_t motor_id, 
                               const Motor_Status& status) override {
        (void)interface; (void)motor_id; (void)status; // 避免未使用参数警告
        individual_updates++;
    }
    
    void on_motor_status_update(const std::string& interface,
                               const std::map<uint32_t, Motor_Status>& status_all) override {
        (void)interface; (void)status_all; // 避免未使用参数警告
        batch_updates++;
    }
    
    void on_motor_function_result(const std::string& interface,
                                 uint32_t motor_id,
                                 uint8_t op_code,
                                 bool success) override {
        (void)interface; (void)motor_id; (void)op_code; (void)success; // 避免未使用参数警告
        function_results++;
    }
};

// Test event subscriber for non-real-time components
class TestEventSubscriber {
public:
    std::atomic<int> status_events{0};
    std::atomic<int> batch_events{0};
    std::atomic<int> function_events{0};
    
    TestEventSubscriber(std::shared_ptr<EventBus> bus) : bus_(bus) {
        // 使用简化的订阅API
        status_handler_ = bus_->subscribe<MotorStatusEvent>(
            [this](const auto& event) { (void)event; status_events++; });
        batch_handler_ = bus_->subscribe<MotorBatchStatusEvent>(
            [this](const auto& event) { (void)event; batch_events++; });
        function_handler_ = bus_->subscribe<MotorFunctionResultEvent>(
            [this](const auto& event) { (void)event; function_events++; });
    }
    
private:
    std::shared_ptr<EventBus> bus_;
    std::shared_ptr<EventHandler> status_handler_;
    std::shared_ptr<EventHandler> batch_handler_;
    std::shared_ptr<EventHandler> function_handler_;
};

class HybridArchitectureTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_bus = std::make_shared<MockBusInterface>();
        motor_driver = std::make_shared<MotorDriverImpl>(mock_bus);
        event_bus = std::make_shared<EventBus>();
        event_bridge = std::make_shared<EventBridge>(event_bus);
        
        realtime_observer = std::make_shared<TestRealtimeObserver>();
        event_subscriber = std::make_unique<TestEventSubscriber>(event_bus);
    }
    
    void TearDown() override {
        motor_driver.reset();
        event_bus.reset();
        event_bridge.reset();
        realtime_observer.reset();
        event_subscriber.reset();
    }
    
    std::shared_ptr<MockBusInterface> mock_bus;
    std::shared_ptr<MotorDriverImpl> motor_driver;
    std::shared_ptr<EventBus> event_bus;
    std::shared_ptr<EventBridge> event_bridge;
    std::shared_ptr<TestRealtimeObserver> realtime_observer;
    std::unique_ptr<TestEventSubscriber> event_subscriber;
};

TEST_F(HybridArchitectureTest, ObserverRegistrationAndRemoval) {
    // Test observer registration
    motor_driver->add_observer(realtime_observer);
    motor_driver->add_observer(event_bridge);
    
    // Test observer removal
    motor_driver->remove_observer(realtime_observer);
    motor_driver->remove_observer(event_bridge);
    
    SUCCEED();
}

TEST_F(HybridArchitectureTest, IndividualMotorStatusUpdate) {
    // Setup
    motor_driver->add_observer(realtime_observer);
    motor_driver->add_observer(event_bridge);
    
    // Simulate motor status update
    Motor_Status test_status = {};
    test_status.enable_flag = 1;
    test_status.motor_mode = 3;
    test_status.position = 1.23f;
    test_status.limit_flag = 0;
    test_status.temperature = 25;
    test_status.velocity = 0.5f;
    test_status.voltage = 240;
    test_status.effort = 0.0f;
    test_status.error_code = 0;
    
    realtime_observer->on_motor_status_update("can0", 1, test_status);
    event_bridge->on_motor_status_update("can0", 1, test_status);
    
    // Allow some time for event processing
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Verify updates
    EXPECT_EQ(realtime_observer->individual_updates.load(), 1);
    EXPECT_EQ(event_subscriber->status_events.load(), 1);
}

TEST_F(HybridArchitectureTest, BatchMotorStatusUpdate) {
    // Setup
    motor_driver->add_observer(realtime_observer);
    motor_driver->add_observer(event_bridge);
    
    // Create batch status data
    std::map<uint32_t, Motor_Status> status_all;
    for (uint32_t id = 1; id <= 3; ++id) {
        Motor_Status status = {};
        status.enable_flag = 1;
        status.motor_mode = 3;
        status.position = id * 0.1f;
        status.limit_flag = 0;
        status.temperature = 25 + id;
        status.velocity = id * 0.05f;
        status.voltage = 240 + id;
        status.effort = 0.0f;
        status.error_code = 0;
        status_all[id] = status;
    }
    
    // Simulate batch update
    realtime_observer->on_motor_status_update("can0", status_all);
    event_bridge->on_motor_status_update("can0", status_all);
    
    // Allow some time for event processing
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Verify batch updates
    EXPECT_EQ(realtime_observer->batch_updates.load(), 1);
    EXPECT_EQ(event_subscriber->batch_events.load(), 1);
}

TEST_F(HybridArchitectureTest, FunctionResultPropagation) {
    // Setup
    motor_driver->add_observer(realtime_observer);
    motor_driver->add_observer(event_bridge);
    
    // Simulate function result
    realtime_observer->on_motor_function_result("can0", 1, 0x01, true);
    event_bridge->on_motor_function_result("can0", 1, 0x01, true);
    
    // Allow some time for event processing
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Verify function results
    EXPECT_EQ(realtime_observer->function_results.load(), 1);
    EXPECT_EQ(event_subscriber->function_events.load(), 1);
}

TEST_F(HybridArchitectureTest, HybridArchitecturePerformance) {
    // Setup both direct observers and event bus
    motor_driver->add_observer(realtime_observer);
    motor_driver->add_observer(event_bridge);
    
    const int num_iterations = 1000;
    const std::string interface = "can0";
    
    // Measure direct observer performance
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_iterations; ++i) {
        Motor_Status status = {};
        status.enable_flag = 1;
        status.motor_mode = 3;
        status.position = i * 0.001f;
        status.limit_flag = 0;
        status.temperature = 25;
        status.velocity = 0.0f;
        status.voltage = 240;
        status.effort = 0.0f;
        status.error_code = 0;
        realtime_observer->on_motor_status_update(interface, 1, status);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto direct_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    // Measure event bus performance
    start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_iterations; ++i) {
        Motor_Status status = {};
        status.enable_flag = 1;
        status.motor_mode = 3;
        status.position = i * 0.001f;
        status.limit_flag = 0;
        status.temperature = 25;
        status.velocity = 0.0f;
        status.voltage = 240;
        status.effort = 0.0f;
        status.error_code = 0;
        event_bridge->on_motor_status_update(interface, 1, status);
    }
    
    end_time = std::chrono::high_resolution_clock::now();
    auto event_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    // Allow time for async event processing
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Verify all updates were processed
    EXPECT_EQ(realtime_observer->individual_updates.load(), num_iterations);
    EXPECT_EQ(event_subscriber->status_events.load(), num_iterations);
    
    // Performance should be reasonable (less than 100ms for 1000 operations)
    EXPECT_LT(direct_duration.count(), 100000);  // < 100ms
    EXPECT_LT(event_duration.count(), 500000);   // < 500ms (event bus has overhead)
    
    std::cout << "Direct observer: " << direct_duration.count() << "μs for " << num_iterations << " updates\n";
    std::cout << "Event bus: " << event_duration.count() << "μs for " << num_iterations << " updates\n";
}

TEST_F(HybridArchitectureTest, BatchInterfaceEfficiency) {
    // Setup
    motor_driver->add_observer(realtime_observer);
    motor_driver->add_observer(event_bridge);
    
    const int num_motors = 10;
    const int num_batches = 100;
    
    // Create batch data
    std::map<uint32_t, Motor_Status> status_all;
    for (uint32_t id = 1; id <= num_motors; ++id) {
        Motor_Status status = {};
        status.enable_flag = 1;
        status.motor_mode = 3;
        status.position = id * 0.1f;
        status.limit_flag = 0;
        status.temperature = 25 + id;
        status.velocity = 0.0f;
        status.voltage = 240 + id;
        status.effort = 0.0f;
        status.error_code = 0;
        status_all[id] = status;
    }
    
    // Measure batch performance
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_batches; ++i) {
        realtime_observer->on_motor_status_update("can0", status_all);
        event_bridge->on_motor_status_update("can0", status_all);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto batch_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    // Allow time for event processing
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Verify batch updates
    EXPECT_EQ(realtime_observer->batch_updates.load(), num_batches);
    EXPECT_EQ(event_subscriber->batch_events.load(), num_batches);
    
    // Batch should be more efficient than individual calls
    EXPECT_LT(batch_duration.count(), num_batches * num_motors * 10);  // Should be much faster than individual calls
    
    std::cout << "Batch processing: " << batch_duration.count() << "μs for " 
              << num_batches << " batches × " << num_motors << " motors\n";
}

TEST_F(HybridArchitectureTest, EventBusSubscriptionManagement) {
    // Test multiple subscribers
    auto subscriber1 = std::make_unique<TestEventSubscriber>(event_bus);
    auto subscriber2 = std::make_unique<TestEventSubscriber>(event_bus);
    
    motor_driver->add_observer(event_bridge);
    
    // Send test event
    Motor_Status status = {};
    status.enable_flag = 1;
    status.motor_mode = 3;
    status.position = 0.0f;
    status.limit_flag = 0;
    status.temperature = 25;
    status.velocity = 0.0f;
    status.voltage = 240;
    status.effort = 0.0f;
    status.error_code = 0;
    event_bridge->on_motor_status_update("can0", 1, status);
    
    // Allow time for processing
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Both subscribers should receive the event
    EXPECT_EQ(subscriber1->status_events.load(), 1);
    EXPECT_EQ(subscriber2->status_events.load(), 1);
    
    // Original subscriber should also get it
    EXPECT_EQ(event_subscriber->status_events.load(), 1);
}