#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "protocol/damiao.h"
#include <csignal>
#include <memory>
#include <vector>
#include <atomic>
#include <thread>
#include <chrono>
#include <array>

// 默认序列号 - 可通过launch参数覆盖
#define DEFAULT_SN_CODE "A4F890D5ACE224020802CF2D55081601"

class ArmControllerNode : public rclcpp::Node
{
public:
    ArmControllerNode() : Node("arm_controller_R")
    {
        RCLCPP_INFO(get_logger(), "Initializing Damiao 7-DOF Right Arm Controller...");
        
        // 声明参数
        declare_parameters();
        
        // 初始化硬件
        if (!initialize_hardware()) 
        {
            RCLCPP_ERROR(get_logger(), "Failed to initialize hardware!");
            rclcpp::shutdown();
            return;
        }
        
        // 创建订阅者和发布者
        setup_ros_interfaces();
        
        // 启动控制循环
        start_control_loop();
        
        RCLCPP_INFO(get_logger(), "Right Arm Controller initialized successfully!");
    }
    
    ~ArmControllerNode()
    {
        shutdown_hardware();
    }

private:
    // 硬件相关
    std::shared_ptr<damiao::Motor_Control> motor_control_;
    std::atomic<bool> running_{true};
    
    // 关节配置 - 基于main.cpp的7关节配置
    struct JointConfig {
        std::string name;
        uint16_t can_id;
        uint16_t master_id;
        damiao::DM_Motor_Type motor_type;
    };
    
    std::array<JointConfig, 7> joint_configs_ = {{
        {"joint1_R", 0x01, 0x11, damiao::DM4340},
        {"joint2_R", 0x02, 0x22, damiao::DM4340},
        {"joint3_R", 0x03, 0x33, damiao::DM4340},
        {"joint4_R", 0x04, 0x44, damiao::DM4310},
        {"joint5_R", 0x05, 0x55, damiao::DM4310},
        {"joint6_R", 0x06, 0x66, damiao::DM4310},
        {"joint7_R", 0x07, 0x77, damiao::DMH3510}
    }};
    
    // 当前关节状态和目标状态
    std::array<double, 7> current_positions_{};
    std::array<double, 7> current_velocities_{};
    std::array<double, 7> current_efforts_{};
    std::array<double, 7> target_positions_{};
    std::array<double, 7> target_velocities_{};
    std::array<double, 7> target_efforts_{};
    
    // ROS2 接口
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_target_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    // 参数
    std::string device_serial_number_;
    std::string tty_device_;
    double control_frequency_;
    double position_tolerance_;
    double velocity_limit_;
    std::array<double, 7> kp_gains_;
    std::array<double, 7> kd_gains_;
    
    void declare_parameters()
    {
        // 硬件参数
        device_serial_number_ = declare_parameter<std::string>(
            "device_serial_number", DEFAULT_SN_CODE);
        tty_device_ = declare_parameter<std::string>(
            "tty_device", "");  // 空字符串表示使用序列号自动查找
        control_frequency_ = declare_parameter<double>("control_frequency", 1000.0);
        
        // 控制参数
        position_tolerance_ = declare_parameter<double>("position_tolerance", 0.01);
        velocity_limit_ = declare_parameter<double>("velocity_limit", 5.0);
        
        // 控制增益
        auto kp_default = std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        auto kd_default = std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        
        auto kp_gains = declare_parameter<std::vector<double>>("kp_gains", kp_default);
        auto kd_gains = declare_parameter<std::vector<double>>("kd_gains", kd_default);
        
        for (size_t i = 0; i < 7; ++i) {
            kp_gains_[i] = (i < kp_gains.size()) ? kp_gains[i] : 5.0;
            kd_gains_[i] = (i < kd_gains.size()) ? kd_gains[i] : 0.5;
        }
        
        kd_gains_[3] = 0.2; // joint4
        kd_gains_[4] = 0.1; // joint5  
        kd_gains_[5] = 0.05; // joint6

        RCLCPP_INFO(get_logger(), "Parameters loaded - Device: %s, TTY: %s, Frequency: %.1f Hz",
                    device_serial_number_.c_str(),
                    tty_device_.empty() ? "auto-detect" : tty_device_.c_str(),
                    control_frequency_);
    }
    
    bool initialize_hardware()
    {
        try {
            std::string actual_device_sn = device_serial_number_;

            // 如果指定了tty设备，尝试根据tty设备映射到序列号
            if (!tty_device_.empty()) {
                RCLCPP_INFO(get_logger(), "Attempting to map TTY device %s to USB2CANFD device...",
                           tty_device_.c_str());

                // 这里可以添加TTY设备到序列号的映射逻辑
                // 例如通过udev规则或者设备枚举来查找对应的USB2CANFD设备
                actual_device_sn = get_device_serial_from_tty(tty_device_);

                if (actual_device_sn.empty()) {
                    RCLCPP_WARN(get_logger(), "Could not map TTY device %s to USB2CANFD serial number, using default",
                               tty_device_.c_str());
                    actual_device_sn = device_serial_number_;
                }
            }

            RCLCPP_INFO(get_logger(), "Initializing Right Arm hardware with device: %s",
                        actual_device_sn.c_str());

            // 准备电机配置数据
            std::vector<damiao::DmActData> motor_data;
            for (const auto& joint : joint_configs_) {
                motor_data.push_back({
                    .motorType = joint.motor_type,
                    .mode = damiao::MIT_MODE,
                    .can_id = joint.can_id,
                    .mst_id = joint.master_id
                });
            }

            // 创建电机控制器
            motor_control_ = std::make_shared<damiao::Motor_Control>(
                1000000,  // 1M nominal baudrate
                5000000,  // 5M data baudrate
                actual_device_sn,
                &motor_data
            );
            
            if (!motor_control_) {
                RCLCPP_ERROR(get_logger(), "Failed to create motor control interface");
                return false;
            }
            
            // 等待硬件初始化完成
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            
            // 验证电机连接
            for (const auto& joint : joint_configs_) {
                auto motor = motor_control_->getMotor(joint.can_id);
                if (!motor) {
                    RCLCPP_ERROR(get_logger(), "Failed to get motor for %s (CAN ID: 0x%02X)", 
                                joint.name.c_str(), joint.can_id);
                    return false;
                }
            }
            
            // 初始化目标位置为当前位置
            std::fill(target_positions_.begin(), target_positions_.end(), 0.0);
            std::fill(target_velocities_.begin(), target_velocities_.end(), 0.0);
            
            RCLCPP_INFO(get_logger(), "Hardware initialized successfully - 7 motors ready");
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Hardware initialization failed: %s", e.what());
            return false;
        }
    }
    
    void setup_ros_interfaces()
    {
        // 订阅关节目标控制命令
        joint_target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "joint_target_R", rclcpp::QoS(10),
            std::bind(&ArmControllerNode::joint_target_callback, this, std::placeholders::_1));

        // 发布当前关节状态
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "joint_states_R", rclcpp::QoS(10));
        
        RCLCPP_INFO(get_logger(), "ROS2 interfaces setup complete");
    }
    
    void start_control_loop()
    {
        auto control_period = std::chrono::nanoseconds(
            static_cast<int64_t>(1e9 / control_frequency_));
        control_timer_ = create_wall_timer(
            control_period,
            std::bind(&ArmControllerNode::control_loop_callback, this));
        
        // 状态发布定时器 (500Hz, 2ms)
        publish_timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ArmControllerNode::publish_callback, this));
        
        RCLCPP_INFO(get_logger(), "Control loop started at %.1f Hz, Publishing at 50 Hz", control_frequency_);
    }
    
    void joint_target_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!motor_control_) {
            RCLCPP_ERROR(get_logger(), "Motor control not initialized");
            return;
        }

        // 验证消息格式 - 只需要至少6个关节，忽略joint7（夹爪）的控制
        if (msg->name.size() < 6 || msg->position.size() < 6)
        {
            RCLCPP_WARN(get_logger(), "Invalid joint_target message: expected at least 6 joints, got %zu names, %zu positions",
                       msg->name.size(), msg->position.size());
            return;
        }

        // 只控制前6个关节，忽略joint7（夹爪）
        const size_t max_joints = std::min({msg->name.size(), msg->position.size(), (size_t)6});
        for (size_t i = 0; i < max_joints; ++i)
        {
            if (msg->name[i] == joint_configs_[i].name)
            {
                auto motor = motor_control_->getMotor(joint_configs_[i].can_id);
                if (!motor) {
                    RCLCPP_ERROR(get_logger(), "Failed to get motor for joint %s (CAN ID: 0x%02X)",
                                joint_configs_[i].name.c_str(), joint_configs_[i].can_id);
                    continue;
                }

                try {
                    float pos = static_cast<float>(msg->position[i]);
                    float vel = (i < msg->velocity.size()) ? static_cast<float>(msg->velocity[i]) : 0.0f;
                    float effort = (i < msg->effort.size()) ? static_cast<float>(msg->effort[i]) : 0.0f;

                    // 使用MIT控制模式
                    motor_control_->control_mit(*motor,
                        static_cast<float>(kp_gains_[i]),  // kp
                        static_cast<float>(kd_gains_[i]),  // kd
                        pos,    // 目标位置
                        vel,    // 目标速度
                        effort  // 力矩前馈
                    );
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "Error controlling motor %s: %s",
                                joint_configs_[i].name.c_str(), e.what());
                }
            }
        }
    }
    
    void control_loop_callback()
    {
        if (!running_ || !motor_control_) 
        {
            return;
        }
        
        try 
        {
            // 读取当前状态
            update_current_states();
            
        } catch (const std::exception& e) 
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Control loop error: %s", e.what());
        }
    }
    
    void publish_callback()
    {
        try 
        {
            // 发布关节状态
            publish_joint_states();
            
        } 
        catch (const std::exception& e) 
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Publish error: %s", e.what());
        }
    }
    
    void update_current_states()
    {                
        for (size_t i = 0; i < 7; ++i) 
        {
            auto motor = motor_control_->getMotor(joint_configs_[i].can_id);
            if (motor) 
            {

                motor_control_->refresh_motor_status(*motor);
                
                current_positions_[i] = motor->Get_Position();
                current_velocities_[i] = motor->Get_Velocity();
                current_efforts_[i] = motor->Get_tau();
            }
        }
    }
    
    void send_control_commands()
    {
        for (size_t i = 0; i < 7; ++i) 
        {
            auto motor = motor_control_->getMotor(joint_configs_[i].can_id);
            if (motor) 
            {
                // MIT控制模式：位置 + 速度 + 力矩控制
                motor_control_->control_mit(*motor,
                    static_cast<float>(kp_gains_[i]),      // kp
                    static_cast<float>(kd_gains_[i]),      // kd  
                    static_cast<float>(target_positions_[i]),   // 目标位置
                    static_cast<float>(target_velocities_[i]),  // 目标速度
                    0.0f  // 力矩前馈 (暂时设为0)
                );
            }
        }
    }
    
    void publish_joint_states()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = now();
        msg.header.frame_id = "";
        
        for (size_t i = 0; i < 7; ++i) {
            // 发布所有关节，保持原始关节名称
            msg.name.push_back(joint_configs_[i].name);

            if (i == 6) {  // joint7是夹爪，需要将电机角度映射到直线距离
                // 将夹爪电机角度映射到直线距离，这个距离就是joint7的"角度"值
                double gripper_angle = current_positions_[i];
                double gripper_distance = map_gripper_angle_to_distance(gripper_angle);

                msg.position.push_back(gripper_distance);  // 映射后的直线距离作为joint7的位置
                msg.velocity.push_back(current_velocities_[i] * 0.001);  // 转换速度单位
                msg.effort.push_back(current_efforts_[i]);
            } else {
                // 其他关节直接使用电机数据
                double position = current_positions_[i];
                double velocity = current_velocities_[i];

                // joint2和joint3需要取反方向
                if (i != 0) {  // joint2 (索引为1) 和 joint3 (索引为2)
                    position = -position;
                    velocity = -velocity;
                }

                msg.position.push_back(position);  // 弧度
                msg.velocity.push_back(velocity); // rad/s
                msg.effort.push_back(current_efforts_[i]);      // Nm
            }
        }
        
        joint_state_pub_->publish(msg);
    }
    
private:
    // 将夹爪角度映射到平移距离
    double map_gripper_angle_to_distance(double angle) {
        // 根据实际观察到的电机角度范围更新，映射关系反过来
        // 输入角度范围：1.8134（夹紧）到 1.1586（张开）
        // 输出距离范围：0mm（夹紧）到 20mm（张开）
        const double angle_closed = -1.1851;  // 夹紧时的角度（实际观察值）
        const double angle_open = -1.4319;    // 张开时的角度（实际观察值）
        const double distance_max = 0.02;    // 最大张开距离（20mm = 0.02m）

        // 线性映射：angle_closed -> 0m, angle_open -> 0.02m
        double normalized = (angle - angle_closed) / (angle_open - angle_closed);
        double distance = normalized * distance_max;

        // 由于URDF中joint7的axis是"0 0 -1"（负方向），需要反转输出
        // 这样与joint_state_publisher_gui的行为保持一致
        double urdf_compatible_distance = distance_max - distance;

        // 确保输出在合理范围内
        return std::clamp(urdf_compatible_distance, 0.0, distance_max);
    }
    
    // TTY设备到序列号映射的辅助函数
    std::string get_device_serial_from_tty(const std::string& tty_device)
    {
        // 这是一个简化的实现，实际应用中可能需要更复杂的设备发现逻辑
        // 可以通过以下方式实现：
        // 1. 解析/sys/class/tty/设备信息
        // 2. 使用udev查询设备属性
        // 3. 通过libusb枚举USB设备

        // 这里返回空字符串表示无法映射，将使用默认序列号
        RCLCPP_WARN(get_logger(), "TTY to serial number mapping not implemented yet");
        return "";
    }

    void shutdown_hardware()
    {
        running_ = false;

        if (motor_control_) {
            try {
                motor_control_->disable_all();
                RCLCPP_INFO(get_logger(), "Hardware shutdown complete");
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Error during shutdown: %s", e.what());
            }
        }
    }
};

// 信号处理
std::shared_ptr<ArmControllerNode> g_node = nullptr;

void signal_handler(int signum)
{
    RCLCPP_INFO(rclcpp::get_logger("arm_controller_R"),
                "Received signal %d, shutting down Right Arm Controller...", signum);

    if (g_node) {
        rclcpp::shutdown();
    }
}

int main(int argc, char** argv)
{
    // 设置信号处理
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    try {
        // 创建节点
        g_node = std::make_shared<ArmControllerNode>();
        
        // 运行节点
        RCLCPP_INFO(g_node->get_logger(), "Right Arm Controller Node is running...");
        rclcpp::spin(g_node);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("arm_controller_R"),
                     "Node error: %s", e.what());
    }

    // 清理
    g_node.reset();
    rclcpp::shutdown();

    RCLCPP_INFO(rclcpp::get_logger("arm_controller_R"), "Right Arm Controller shutdown complete");
    return 0;
}
