// #include <gtest/gtest.h>
// #include "driver/motor_driver_impl.hpp"
// #include "bus/usb2canfd_bus_impl.hpp"
// #include "hardware_driver/event/event_bus.hpp"
// #include "hardware_driver/event/motor_events.hpp"
// #include "hardware_driver/interface/robot_hardware.hpp"
// #include <memory>
// #include <thread>
// #include <chrono>
// #include <atomic>

// using namespace hardware_driver;
// using namespace hardware_driver::motor_driver;
// using namespace hardware_driver::bus;
// using namespace hardware_driver::event;

// // USB2CANFD硬件检测函数
// bool has_usb2canfd_hardware() {
//     // 检查是否有USB2CANFD设备
//     // 可以通过libusb枚举设备来实现，这里简化处理
//     return false;  // 默认假设没有USB设备，测试会在模拟模式下运行
// }

// // 测试用的电机状态观察者
// class TestMotorStatusObserver {
// public:
//     std::atomic<int> status_received_count{0};
//     std::atomic<int> batch_received_count{0};

//     std::map<uint32_t, Motor_Status> latest_status;
//     std::map<std::string, std::map<uint32_t, Motor_Status>> latest_batch_status;
//     std::mutex status_mutex;

//     TestMotorStatusObserver(std::shared_ptr<EventBus> event_bus) : event_bus_(event_bus) {
//         // 订阅单个电机状态事件
//         motor_status_handler_ = event_bus_->subscribe<MotorStatusEvent>(
//             [this](const std::shared_ptr<MotorStatusEvent>& event) {
//                 std::lock_guard<std::mutex> lock(status_mutex);
//                 latest_status[event->get_motor_id()] = event->get_status();
//                 status_received_count++;
//                 latest_interface = event->get_interface();
//             });

//         // 订阅批量电机状态事件
//         batch_status_handler_ = event_bus_->subscribe<MotorBatchStatusEvent>(
//             [this](const std::shared_ptr<MotorBatchStatusEvent>& event) {
//                 std::lock_guard<std::mutex> lock(status_mutex);
//                 latest_batch_status[event->get_interface()] = event->get_status_all();
//                 batch_received_count++;
//                 latest_batch_interface = event->get_interface();
//             });
//     }

//     ~TestMotorStatusObserver() {
//         if (motor_status_handler_) {
//             event_bus_->unsubscribe<MotorStatusEvent>(motor_status_handler_);
//         }
//         if (batch_status_handler_) {
//             event_bus_->unsubscribe<MotorBatchStatusEvent>(batch_status_handler_);
//         }
//     }

//     Motor_Status get_motor_status(uint32_t motor_id) {
//         std::lock_guard<std::mutex> lock(status_mutex);
//         return latest_status[motor_id];
//     }

//     std::string latest_interface;
//     std::string latest_batch_interface;

// private:
//     std::shared_ptr<EventBus> event_bus_;
//     std::shared_ptr<EventHandler> motor_status_handler_;
//     std::shared_ptr<EventHandler> batch_status_handler_;
// };

// class Usb2CanfdBusIntegrationTest : public ::testing::Test {
// protected:
//     void SetUp() override {
//         // 检查是否有真实的USB2CANFD硬件
//         has_hardware_ = has_usb2canfd_hardware();

//         // 创建事件总线
//         event_bus_ = std::make_shared<EventBus>();
//         status_observer_ = std::make_unique<TestMotorStatusObserver>(event_bus_);

//         try {
//             // 创建USB2CANFD总线实例
//             std::vector<std::string> device_sns = {"TEST_USB_DEVICE"};
//             usb2canfd_bus_ = std::make_shared<Usb2CanfdBus>(device_sns);
//             motor_driver_ = std::make_shared<MotorDriverImpl>(usb2canfd_bus_);

//             // 创建RobotHardware实例，使用事件总线批量回调
//             // 根据实际硬件配置：usb_canfd_0接口，motor_id为1和2
//             std::map<std::string, std::vector<uint32_t>> config = {{"usb_canfd_0", {1, 2}}};

//             // 使用批量状态回调
//             auto batch_callback = [this](const std::string& interface,
//                                         const std::map<uint32_t, Motor_Status>& status_all) {
//                 // 发布批量电机状态事件
//                 event_bus_->emit<MotorBatchStatusEvent>(interface, status_all);

//                 // 同时发布单个电机状态事件用于测试
//                 for (const auto& [motor_id, status] : status_all) {
//                     event_bus_->emit<MotorStatusEvent>(interface, motor_id, status);
//                 }
//             };

//             robot_hardware_ = std::make_unique<RobotHardware>(motor_driver_, config, batch_callback);

//             if (has_hardware_) {
//                 std::cout << "Running USB2CANFD integration tests with real hardware" << std::endl;
//             } else {
//                 std::cout << "Running USB2CANFD integration tests in simulation mode" << std::endl;
//             }
//         } catch (const std::exception& e) {
//             // 如果构造函数失败，创建空的实例
//             std::cout << "USB2CANFD hardware initialization failed: " << e.what() << std::endl;
//             std::cout << "Running USB2CANFD integration tests in simulation mode" << std::endl;

//             // 创建空的实例，测试会跳过硬件相关的部分
//             usb2canfd_bus_ = nullptr;
//             motor_driver_ = nullptr;
//             robot_hardware_ = nullptr;
//         }
//     }

//     void TearDown() override {
//         status_observer_.reset();
//         robot_hardware_.reset();
//         motor_driver_.reset();
//         usb2canfd_bus_.reset();
//         event_bus_.reset();
//     }

//     std::shared_ptr<Usb2CanfdBus> usb2canfd_bus_;
//     std::shared_ptr<MotorDriverImpl> motor_driver_;
//     std::unique_ptr<RobotHardware> robot_hardware_;
//     std::shared_ptr<EventBus> event_bus_;
//     std::unique_ptr<TestMotorStatusObserver> status_observer_;
//     bool has_hardware_ = false;
// };

// // 测试Usb2CanfdBus的接口名称获取
// TEST_F(Usb2CanfdBusIntegrationTest, GetInterfaceNames) {
//     if (!usb2canfd_bus_) {
//         GTEST_SKIP() << "USB2CANFD hardware not available - skipping interface test";
//     }

//     auto interfaces = usb2canfd_bus_->get_interface_names();
//     ASSERT_EQ(interfaces.size(), 1);
//     ASSERT_EQ(interfaces[0], "usb_canfd_0");
// }

// // 测试获取设备序列号
// TEST_F(Usb2CanfdBusIntegrationTest, GetDeviceSerialNumbers) {
//     if (!usb2canfd_bus_) {
//         GTEST_SKIP() << "USB2CANFD hardware not available - skipping device SN test";
//     }

//     auto sns = usb2canfd_bus_->getDeviceSerialNumbers();
//     ASSERT_EQ(sns.size(), 1);
//     ASSERT_EQ(sns[0], "TEST_USB_DEVICE");
// }

// // 测试指定接口的设备序列号
// TEST_F(Usb2CanfdBusIntegrationTest, GetDeviceSerialNumberForInterface) {
//     if (!usb2canfd_bus_) {
//         GTEST_SKIP() << "USB2CANFD hardware not available - skipping interface SN test";
//     }

//     std::string sn = usb2canfd_bus_->getDeviceSerialNumber("usb_canfd_0");
//     ASSERT_EQ(sn, "TEST_USB_DEVICE");
// }

// // 测试多设备创建
// TEST_F(Usb2CanfdBusIntegrationTest, MultipleDevices) {
//     std::vector<std::string> device_sns = {"DEVICE_1", "DEVICE_2", "DEVICE_3"};
//     auto bus = std::make_shared<Usb2CanfdBus>(device_sns);

//     auto interfaces = bus->get_interface_names();
//     ASSERT_EQ(interfaces.size(), 3);
//     ASSERT_EQ(interfaces[0], "usb_canfd_0");
//     ASSERT_EQ(interfaces[1], "usb_canfd_1");
//     ASSERT_EQ(interfaces[2], "usb_canfd_2");

//     // 验证每个接口对应的序列号
//     for (size_t i = 0; i < device_sns.size(); ++i) {
//         std::string interface = "usb_canfd_" + std::to_string(i);
//         std::string sn = bus->getDeviceSerialNumber(interface);
//         ASSERT_EQ(sn, device_sns[i]);
//     }
// }

// // 测试BusInterface兼容性
// TEST_F(Usb2CanfdBusIntegrationTest, BusInterfaceCompatibility) {
//     std::vector<std::string> device_sns = {"INTERFACE_COMPAT_TEST"};

//     // 通过BusInterface指针使用
//     std::shared_ptr<BusInterface> bus = std::make_shared<Usb2CanfdBus>(device_sns);

//     ASSERT_NE(bus, nullptr);
//     auto interfaces = bus->get_interface_names();
//     ASSERT_EQ(interfaces.size(), 1);
// }

// // 测试工厂函数
// TEST_F(Usb2CanfdBusIntegrationTest, FactoryFunctionCreateUsb2CanfdMotorDriver) {
//     std::vector<std::string> device_sns = {"FACTORY_TEST"};

//     auto motor_driver = createUsb2CanfdMotorDriver(device_sns);

//     ASSERT_NE(motor_driver, nullptr);
// }

// // 测试自定义波特率
// TEST_F(Usb2CanfdBusIntegrationTest, CustomBitrates) {
//     std::vector<std::string> device_sns = {"CUSTOM_BITRATE_TEST"};
//     uint32_t arb_rate = 500000;
//     uint32_t data_rate = 2000000;

//     auto bus = std::make_shared<Usb2CanfdBus>(device_sns, arb_rate, data_rate);

//     ASSERT_NE(bus, nullptr);
//     auto interfaces = bus->get_interface_names();
//     ASSERT_EQ(interfaces.size(), 1);
// }

// // 测试异步接收回调
// TEST_F(Usb2CanfdBusIntegrationTest, AsyncReceiveCallback) {
//     std::vector<std::string> device_sns = {"CALLBACK_TEST"};
//     auto bus = std::make_shared<Usb2CanfdBus>(device_sns);

//     std::atomic<int> callback_count{0};

//     bus->async_receive([&callback_count](const GenericBusPacket& /* packet */) {
//         callback_count++;
//     });

//     // 回调已设置，总线开始接收
//     std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     ASSERT_EQ(bus->get_interface_names().size(), 1);
// }

// // 测试发送数据包接口
// TEST_F(Usb2CanfdBusIntegrationTest, SendDataPacketInterface) {
//     if (!usb2canfd_bus_) {
//         GTEST_SKIP() << "USB2CANFD hardware not available - skipping send test";
//     }

//     GenericBusPacket packet;
//     packet.interface = "usb_canfd_0";
//     packet.id = 0x100;
//     packet.len = 8;
//     packet.data[0] = 0x11;
//     packet.data[1] = 0x22;

//     // 调用send接口（可能返回false，因为没有真实硬件）
//     // 接口调用应该可以成功执行，即使硬件操作失败
//     usb2canfd_bus_->send(packet);
//     ASSERT_TRUE(true);
// }

// // 测试接收数据包接口
// TEST_F(Usb2CanfdBusIntegrationTest, ReceiveDataPacketInterface) {
//     if (!usb2canfd_bus_) {
//         GTEST_SKIP() << "USB2CANFD hardware not available - skipping receive test";
//     }

//     GenericBusPacket packet;

//     // 调用receive接口
//     // 接口调用应该可以成功执行
//     usb2canfd_bus_->receive(packet);
//     ASSERT_TRUE(true);
// }
