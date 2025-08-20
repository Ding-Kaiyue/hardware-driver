#include <gtest/gtest.h>
#include "hardware_driver/event/event_bus.hpp"
#include "hardware_driver/event/motor_events.hpp"
#include <memory>
#include <atomic>

using namespace hardware_driver::event;

// 简化的事件订阅者用于测试
class TestEventSubscriber {
public:
    std::atomic<int> motor_status_count{0};
    std::atomic<int> batch_status_count{0};
    
    TestEventSubscriber(std::shared_ptr<EventBus> bus) : bus_(bus) {
        // 使用简化的订阅API测试
        motor_handler_ = bus_->subscribe<MotorStatusEvent>(
            [this](const auto& event) { 
                motor_status_count++;
                last_motor_id = event->get_motor_id();
                last_interface = event->get_interface();
            });
            
        batch_handler_ = bus_->subscribe<MotorBatchStatusEvent>(
            [this](const auto& event) { 
                batch_status_count++;
                last_batch_size = event->get_status_all().size();
                last_batch_interface = event->get_interface();
            });
    }
    
    ~TestEventSubscriber() {
        // 测试显式取消订阅
        if (motor_handler_) {
            bus_->unsubscribe<MotorStatusEvent>(motor_handler_);
        }
        if (batch_handler_) {
            bus_->unsubscribe<MotorBatchStatusEvent>(batch_handler_);
        }
    }
    
    // 测试用的公共变量
    uint32_t last_motor_id = 0;
    std::string last_interface;
    size_t last_batch_size = 0;
    std::string last_batch_interface;
    
private:
    std::shared_ptr<EventBus> bus_;
    std::shared_ptr<EventHandler> motor_handler_;
    std::shared_ptr<EventHandler> batch_handler_;
};

class EventBusApiTest : public ::testing::Test {
protected:
    void SetUp() override {
        bus = std::make_shared<EventBus>();
    }
    
    std::shared_ptr<EventBus> bus;
};

TEST_F(EventBusApiTest, SimplifiedSubscribeApi) {
    auto subscriber = std::make_unique<TestEventSubscriber>(bus);
    
    // 创建测试用的电机状态数据
    hardware_driver::motor_driver::Motor_Status status1 = {1, 2, 100.0f, 0, 45, 50.0f, 240, 0.5f, 0};
    hardware_driver::motor_driver::Motor_Status status2 = {1, 3, 200.0f, 0, 50, 75.0f, 250, 0.8f, 0};
    
    // 发布单个电机状态事件
    bus->emit<MotorStatusEvent>("test_can0", 1, status1);
    bus->emit<MotorStatusEvent>("test_can0", 2, status2);
    
    // 验证单个事件接收
    EXPECT_EQ(subscriber->motor_status_count.load(), 2);
    EXPECT_EQ(subscriber->last_motor_id, 2);
    EXPECT_EQ(subscriber->last_interface, "test_can0");
    
    // 创建批量状态事件
    std::map<uint32_t, hardware_driver::motor_driver::Motor_Status> batch_status;
    batch_status[1] = status1;
    batch_status[2] = status2;
    bus->emit<MotorBatchStatusEvent>("test_can1", batch_status);
    
    // 验证批量事件接收
    EXPECT_EQ(subscriber->batch_status_count.load(), 1);
    EXPECT_EQ(subscriber->last_batch_size, 2);
    EXPECT_EQ(subscriber->last_batch_interface, "test_can1");
}

TEST_F(EventBusApiTest, EventBusStatistics) {
    auto subscriber = std::make_unique<TestEventSubscriber>(bus);
    
    // 发布一些事件
    hardware_driver::motor_driver::Motor_Status status = {1, 2, 100.0f, 0, 45, 50.0f, 240, 0.5f, 0};
    bus->emit<MotorStatusEvent>("test_can0", 1, status);
    
    std::map<uint32_t, hardware_driver::motor_driver::Motor_Status> batch_status;
    batch_status[1] = status;
    bus->emit<MotorBatchStatusEvent>("test_can0", batch_status);
    
    // 检查统计信息
    auto stats = bus->get_statistics();
    EXPECT_EQ(stats.total_handlers, 2);  // motor_handler + batch_handler
    EXPECT_EQ(stats.events_published, 2);
    EXPECT_EQ(stats.total_topic_handlers, 0);  // 没有使用主题订阅
}

TEST_F(EventBusApiTest, UnsubscribeWorksCorrectly) {
    auto subscriber = std::make_unique<TestEventSubscriber>(bus);
    
    // 发布事件测试订阅有效
    hardware_driver::motor_driver::Motor_Status status = {1, 2, 100.0f, 0, 45, 50.0f, 240, 0.5f, 0};
    bus->emit<MotorStatusEvent>("test_can0", 1, status);
    EXPECT_EQ(subscriber->motor_status_count.load(), 1);
    
    // 销毁订阅者（应该调用unsubscribe）
    subscriber.reset();
    
    // 再次发布事件，应该没有处理器接收
    bus->emit<MotorStatusEvent>("test_can0", 2, status);
    
    // 创建新的订阅者验证事件继续正常工作
    auto new_subscriber = std::make_unique<TestEventSubscriber>(bus);
    bus->emit<MotorStatusEvent>("test_can0", 3, status);
    EXPECT_EQ(new_subscriber->motor_status_count.load(), 1);
    EXPECT_EQ(new_subscriber->last_motor_id, 3);
}

TEST_F(EventBusApiTest, MultipleSubscribersReceiveEvents) {
    auto subscriber1 = std::make_unique<TestEventSubscriber>(bus);
    auto subscriber2 = std::make_unique<TestEventSubscriber>(bus);
    
    hardware_driver::motor_driver::Motor_Status status = {1, 2, 100.0f, 0, 45, 50.0f, 240, 0.5f, 0};
    bus->emit<MotorStatusEvent>("test_can0", 1, status);
    
    // 两个订阅者都应该接收到事件
    EXPECT_EQ(subscriber1->motor_status_count.load(), 1);
    EXPECT_EQ(subscriber2->motor_status_count.load(), 1);
    EXPECT_EQ(subscriber1->last_motor_id, 1);
    EXPECT_EQ(subscriber2->last_motor_id, 1);
}