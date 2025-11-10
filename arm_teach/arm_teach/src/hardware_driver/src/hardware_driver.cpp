#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "hardware/usb2canfd_manager.h"
#include "protocol/raytron_motor_ctrl.h"
#include "protocol/gripper_ctrl.h"
#include "protocol/teaching_device.h"

#include <chrono>
#include <thread>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <iomanip>

/**
 * @class HardwareDriverNode
 * @brief ROS2 hardware driver node for DM-CANFD USB adapter
 *
 * 功能:
 * - 订阅 joint_command 话题（JointState消息）接收关节控制命令
 * - 发布 joint_states 话题发送关节反馈数据
 * - 智能控制模式选择：
 *   - position != 0 && velocity == 0: 位置控制
 *   - position == 0 && velocity != 0: 速度控制
 *   - position != 0 && velocity != 0: MIT模式
 *   - position == 0 && velocity == 0: 失能
 * - 关节配置:
 *   - 关节 0-5: Raytron 电机 1-6
 *   - 关节 6: 夹爪
 */
class HardwareDriverNode : public rclcpp::Node
{
public:
    HardwareDriverNode() : Node("hardware_driver_node")
    {
        // ========== 声明参数 ==========
        this->declare_parameter<std::string>("device_path", "/dev/ttyACM0");
        this->declare_parameter<int>("nom_baud", 1000000);
        this->declare_parameter<int>("dat_baud", 5000000);
        this->declare_parameter<double>("publish_rate", 50.0);  // Hz

        // 获取参数
        device_path_ = this->get_parameter("device_path").as_string();
        nom_baud_ = this->get_parameter("nom_baud").as_int();
        dat_baud_ = this->get_parameter("dat_baud").as_int();
        publish_rate_ = this->get_parameter("publish_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "Hardware Driver Node Initializing...");
        RCLCPP_INFO(this->get_logger(), "Device Path: %s", device_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Nominal Baud: %d, Data Baud: %d", nom_baud_, dat_baud_);
 
        // ========== 关节配置 ==========
        joint_names_ = {
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6",
            "gripper"
        };

        motor_ids_ = {1, 2, 3, 4, 5, 6};
        gripper_id_ = 7;

        // ========== 控制参数配置 ==========
        this->declare_parameter<std::vector<double>>("motor_kp", {0.05, 0.05, 0.05, 0.05, 0.05, 0.05});
        this->declare_parameter<std::vector<double>>("motor_kd", {0.01, 0.01, 0.01, 0.01, 0.01, 0.01});

        motor_kp_ = this->get_parameter("motor_kp").as_double_array();
        motor_kd_ = this->get_parameter("motor_kd").as_double_array();

        // 验证参数数量
        if (motor_kp_.size() != motor_ids_.size()) {
            RCLCPP_WARN(this->get_logger(), "motor_kp size mismatch, resizing to %zu", motor_ids_.size());
            motor_kp_.resize(motor_ids_.size(), 0.05);
        }
        if (motor_kd_.size() != motor_ids_.size()) {
            RCLCPP_WARN(this->get_logger(), "motor_kd size mismatch, resizing to %zu", motor_ids_.size());
            motor_kd_.resize(motor_ids_.size(), 0.01);
        }

        RCLCPP_INFO(this->get_logger(), "Motor Kp: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    motor_kp_[0], motor_kp_[1], motor_kp_[2], motor_kp_[3], motor_kp_[4], motor_kp_[5]);
        RCLCPP_INFO(this->get_logger(), "Motor Kd: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    motor_kd_[0], motor_kd_[1], motor_kd_[2], motor_kd_[3], motor_kd_[4], motor_kd_[5]);

        // 初始化关节状态
        joint_positions_.resize(joint_names_.size(), 0.0);
        joint_velocities_.resize(joint_names_.size(), 0.0);
        joint_efforts_.resize(joint_names_.size(), 0.0);

        // 初始化电机上一次状态跟踪（用于检测模式变化）
        motor_last_states_.resize(motor_ids_.size());

        // ========== 创建发布者和订阅者 ==========
        // 发布关节状态
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", rclcpp::QoS(10)
        );

        // 发布示教状态
        teaching_status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/teaching_status", rclcpp::QoS(10)
        );

        // 订阅关节控制命令（使用JointState消息）
        joint_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states_R",
            rclcpp::QoS(10),
            std::bind(&HardwareDriverNode::onJointCommand, this, std::placeholders::_1)
        );

        // ========== 初始化硬件 ==========
        try {
            usb_manager_ = Usb2CanfdManager::createFromDevicePath(
                device_path_,
                nom_baud_,
                dat_baud_
            );

            if (!usb_manager_) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create USB2CANFD manager");
                throw std::runtime_error("USB2CANFD initialization failed");
            }

            if (!usb_manager_->isReady()) {
                RCLCPP_ERROR(this->get_logger(), "USB2CANFD hardware is not ready");
                throw std::runtime_error("USB2CANFD not ready");
            }

            // 设置接收回调
            usb_manager_->setReceiveCallback(
                std::bind(&HardwareDriverNode::onCanFrameReceived, this, std::placeholders::_1)
            );

            RCLCPP_INFO(this->get_logger(), "Hardware initialized: %s",
                       usb_manager_->getHardwareIdentifier().c_str());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Hardware initialization error: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // ========== 创建定时器发布关节状态 ==========
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_),
            std::bind(&HardwareDriverNode::publishJointState, this)
        );

        // ========== 创建控制线程 ==========
        control_thread_running_ = true;
        control_thread_ = std::thread(&HardwareDriverNode::controlLoop, this);

        RCLCPP_INFO(this->get_logger(), "Hardware Driver Node initialized successfully");
    }

    ~HardwareDriverNode()
    {
        control_thread_running_ = false;
        if (control_thread_.joinable()) {
            control_thread_.join();
        }
    }

private:
    // ========== ROS2 相关 ==========
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr teaching_status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ========== 配置参数 ==========
    std::string device_path_;
    int nom_baud_;
    int dat_baud_;
    double publish_rate_;

    // ========== 关节配置 ==========
    std::vector<std::string> joint_names_;
    std::vector<uint8_t> motor_ids_;
    uint8_t gripper_id_;

    // ========== 控制参数 ==========
    std::vector<double> motor_kp_;  // 每个电机的Kp参数
    std::vector<double> motor_kd_;  // 每个电机的Kd参数

    // ========== 示教模式状态 ==========
    std::atomic<bool> teaching_mode_enabled_{false};  // 示教模式是否启用
    std::mutex teaching_mode_mutex_;

    // ========== 关节状态数据 ==========
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;
    std::mutex joint_state_mutex_;

    // ========== 电机/夹爪数据 ==========
    std::unordered_map<uint8_t, raytron::MotorData> motor_data_;
    gripper::GripperData gripper_data_{};
    std::mutex motor_data_mutex_;
    std::mutex gripper_data_mutex_;

    // ========== 硬件相关 ==========
    std::shared_ptr<Usb2CanfdManager> usb_manager_;
    raytron::RaytronProtocolConverter protocol_converter_;
    gripper::GripperProtocolConverter gripper_converter_;
    teaching_device::TeachingDeviceProtocol teaching_protocol_;

    // ========== 电机控制状态跟踪（用于检测模式变化）==========
    struct MotorLastState {
        raytron::ControlMode last_mode = raytron::MODE_SPEED;
        bool last_enabled = false;
    };

    std::vector<MotorLastState> motor_last_states_;
    std::mutex motor_control_mutex_;

    // ========== 控制线程 ==========
    std::thread control_thread_;
    std::atomic<bool> control_thread_running_{false};
    std::atomic<bool> pause_feedback_loop_{false};  // 暂停反馈请求，让高优先级命令通过
    size_t motor_request_index_{0};

    // ========== 回调函数 ==========

    /**
     * @brief CAN 帧接收回调
     */
    void onCanFrameReceived(const canfd_frame& frame)
    {
        uint32_t actual_can_id = frame.can_id & 0x1FFFFFFF;

        // 处理示教设备命令：接收到 0xA01 -> 解析命令并响应 0xA02
        if (actual_can_id == teaching_device::CMD_RECEIVE_ID)
        {
            teaching_device::CommandCode command;
            if (teaching_protocol_.decodeCommand(actual_can_id, frame.data, frame.len, command))
            {
                // 处理命令
                handleTeachingCommand(command);

                // 立即发送响应帧（高优先级）
                canfd_frame response_frame = teaching_protocol_.encodeResponse(command);
                if (usb_manager_ && usb_manager_->isReady())
                {
                    usb_manager_->sendFrame(response_frame, true);  // is_high_priority = true
                    RCLCPP_INFO(this->get_logger(), "Received teaching command: %s, sent response",
                                teaching_protocol_.commandToString(command));
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to decode teaching command from 0xA01");
            }
        }
        // 处理 Raytron 电机反馈 (反馈ID = 0x0300 + 电机ID)
        else if ((actual_can_id & 0xFF00) == raytron::FEEDBACK_BASE_ID)
        {
            uint8_t motor_id = actual_can_id & 0xFF;
            raytron::MotorData motor_data{};
            protocol_converter_.decodeFeedback(actual_can_id, frame.data, frame.len, motor_data);

            {
                std::lock_guard<std::mutex> lock(motor_data_mutex_);
                motor_data_[motor_id] = motor_data;
            }

            RCLCPP_DEBUG(this->get_logger(),
                        "Motor #%d - Pos: %.4f, Vel: %.4f, Cur: %.4f",
                        motor_id, motor_data.position, motor_data.velocity, motor_data.current);
        }
        // 处理夹爪反馈
        else if (actual_can_id == gripper_id_) {
            gripper::GripperData gripper_data{};
            gripper_converter_.decodeStatus(actual_can_id, frame.data, frame.len, gripper_data);

            {
                std::lock_guard<std::mutex> lock(gripper_data_mutex_);
                gripper_data_ = gripper_data;
            }

            RCLCPP_DEBUG(this->get_logger(),
                        "Gripper #%d - Pos: %d, Force: %d, State: 0x%02x",
                        gripper_id_, gripper_data.position, gripper_data.force, gripper_data.state);
        }
    }

    /**
     * @brief 关节命令订阅回调 - 支持智能模式选择
     *
     * JointState 消息格式:
     *   position[0-5]: 电机 1-6 的目标位置 (rad)
     *   position[6]: 夹爪目标位置 (0-1, 0=夹紧, 1=完全张开)
     *   velocity[0-5]: 电机 1-6 的目标速度 (rad/s)
     *   velocity[6]: 夹爪速度（未使用）
     *   effort: 力矩/电流（未使用）
     *
     * 控制模式智能选择:
     * - position != 0 && velocity == 0: 位置控制模式
     * - position == 0 && velocity != 0: 速度控制模式
     * - position != 0 && velocity != 0: MIT模式（位置+速度）
     * - position == 0 && velocity == 0: 发送失能命令
     *
     * 模式切换时会先发送失能命令 (使能标志=0)，然后发送使能的新模式命令
     */
    void onJointCommand(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!usb_manager_ || !usb_manager_->isReady()) {
            RCLCPP_WARN(this->get_logger(), "Received command but hardware not ready");
            return;
        }

        if (msg->position.size() != joint_names_.size()) {
            RCLCPP_WARN(this->get_logger(),
                       "Invalid command size: expected %zu, got %zu",
                       joint_names_.size(), msg->position.size());
            return;
        }

        size_t valid_size = msg->position.size();
        if (msg->velocity.size() < valid_size) {
            RCLCPP_WARN(this->get_logger(),
                       "JointState velocity array too small: %zu < %zu",
                       msg->velocity.size(), valid_size);
            return;
        }

        // 暂停反馈循环，让控制命令立即发送
        pause_feedback_loop_.store(true);

        // 等待一小段时间确保反馈循环看到暂停标志
        std::this_thread::sleep_for(std::chrono::microseconds(200));

        std::lock_guard<std::mutex> lock(motor_control_mutex_);
        const float EPSILON = 1e-6f;

        // 处理电机命令 (前 6 个关节)
        for (size_t i = 0; i < motor_ids_.size() && i < valid_size; ++i) 
        {
            uint8_t motor_id = motor_ids_[i];
            float pos_cmd = static_cast<float>(msg->position[i]);
            float vel_cmd = (i < msg->velocity.size()) ? static_cast<float>(msg->velocity[i]) : 0.0f;

            bool has_pos = std::abs(pos_cmd) > EPSILON;
            bool has_vel = std::abs(vel_cmd) > EPSILON;

            // 判断新的控制模式
            raytron::ControlMode new_mode;
            // if (has_pos && !has_vel) 
            // {
            //     new_mode = raytron::MODE_ABSOLUTE_POS;  // 位置控制
            // } 
            // else if (!has_pos && has_vel) 
            // {
            //     new_mode = raytron::MODE_SPEED;  // 速度控制
            // } 
            // else 
            // {
            new_mode = raytron::MODE_FORCE_POSITION;  // MIT模式（力位混合）
            // }
            

            // 模式改变时先发送失能命令
            if (motor_last_states_[i].last_enabled && motor_last_states_[i].last_mode != new_mode) 
            {
                RCLCPP_INFO(this->get_logger(),
                           "Motor #%d mode change detected, sending disable command", motor_id);
                canfd_frame disable_frame = protocol_converter_.encodeControl(
                    motor_id, raytron::FLAG_DISABLE, raytron::MODE_SPEED,
                    0.0f, 0.0f, 0.0f
                );
                usb_manager_->sendFrame(disable_frame, true);  // High priority
            }

            // 发送使能的控制命令
            // float pos_val = (new_mode == raytron::MODE_SPEED) ? 0.0f : pos_cmd;
            // float vel_val = (new_mode == raytron::MODE_ABSOLUTE_POS) ? 0.0f : vel_cmd;

            pos_cmd *= 180.0f / static_cast<float>(M_PI);  // 转换为度
            vel_cmd *= 180.0f / static_cast<float>(M_PI);  // 转换为度/秒


            if(i == 2 || i == 5)
            {
                pos_cmd = -pos_cmd;
                vel_cmd = -vel_cmd;
            }
            canfd_frame control_frame = protocol_converter_.encodeControl(
                motor_id,
                raytron::FLAG_ENABLE,
                new_mode,
                pos_cmd,
                vel_cmd,
                0.0f,
                static_cast<float>(motor_kp_[i]),  // Kp参数
                static_cast<float>(motor_kd_[i])   // Kd参数
            );

            usb_manager_->sendFrame(control_frame, true);

            std::this_thread::sleep_for(std::chrono::microseconds(500));
            
            // 日志记录
            if (has_pos && !has_vel) 
            {
                RCLCPP_DEBUG(this->get_logger(), "Motor #%d position control: %.4f rad",
                            motor_id, pos_cmd);
            } 
            else if (!has_pos && has_vel) 
            {
                RCLCPP_DEBUG(this->get_logger(), "Motor #%d velocity control: %.4f rad/s",
                            motor_id, vel_cmd);
            } 
            else 
            {
                RCLCPP_DEBUG(this->get_logger(), "Motor #%d MIT control: pos=%.4f rad, vel=%.4f rad/s",
                            motor_id, pos_cmd, vel_cmd);
            }

            // 更新状态
            motor_last_states_[i].last_mode = new_mode;
            motor_last_states_[i].last_enabled = true;
        }

        // 处理夹爪命令 (第 7 个关节)
        if (valid_size > motor_ids_.size()) {
            float gripper_pos_normalized = static_cast<float>(msg->position[motor_ids_.size()]);
            uint8_t gripper_pos = static_cast<uint8_t>(
                std::clamp(gripper_pos_normalized * 255.0f, 0.0f, 255.0f)
            );

            canfd_frame gripper_frame = gripper_converter_.encodeSimpleControl(
                gripper_id_, gripper_pos, 150
            );

            if (usb_manager_->sendFrame(gripper_frame)) 
            {
                RCLCPP_DEBUG(this->get_logger(),
                            "Gripper: pos=%d (normalized: %.4f)", gripper_pos, gripper_pos_normalized);
            } 
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to send gripper command");
            }
        }

        // 恢复反馈循环
        pause_feedback_loop_.store(false);
    }

    /**
     * @brief 处理示教命令的内部逻辑
     * @param command 命令码
     */
    void handleTeachingCommand(teaching_device::CommandCode command)
    {
        std::lock_guard<std::mutex> lock(teaching_mode_mutex_);

        auto status_msg = std_msgs::msg::String();

        switch (command)
        {
            case teaching_device::CMD_START_TEACHING:
                teaching_mode_enabled_.store(true);
                status_msg.data = "start_teaching";
                RCLCPP_INFO(this->get_logger(), "Teaching mode ENABLED - motor control active");
                break;

            case teaching_device::CMD_END_TEACHING:
                teaching_mode_enabled_.store(false);
                status_msg.data = "end_teaching";
                RCLCPP_INFO(this->get_logger(), "Teaching mode DISABLED - motor control blocked");
                break;

            case teaching_device::CMD_ENTER_CALIBRATION:
                status_msg.data = "enter_calibration";
                RCLCPP_INFO(this->get_logger(), "Entered CALIBRATION mode");
                break;

            case teaching_device::CMD_RECORD_MAX_ANGLE:
                status_msg.data = "record_max_angle";
                RCLCPP_INFO(this->get_logger(), "Recording MAX angle");
                break;

            case teaching_device::CMD_RECORD_MIN_ANGLE:
                status_msg.data = "record_min_angle";
                RCLCPP_INFO(this->get_logger(), "Recording MIN angle");
                break;

            default:
                status_msg.data = "unknown";
                RCLCPP_WARN(this->get_logger(), "Unknown teaching command code: %d", command);
                break;
        }

        // 发布状态消息到ROS话题
        teaching_status_pub_->publish(status_msg);
    }

    /**
     * @brief 发布关节状态
     */
    void publishJointState()
    {
        auto msg = std::make_unique<sensor_msgs::msg::JointState>();
        msg->header.stamp = this->now();
        msg->name = joint_names_;

        // 更新关节状态
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);

            // 电机反馈 -> 前 6 个关节
            {
                std::lock_guard<std::mutex> motor_lock(motor_data_mutex_);
                for (size_t i = 0; i < motor_ids_.size(); ++i) {
                    uint8_t motor_id = motor_ids_[i];
                    if (motor_data_.find(motor_id) != motor_data_.end()) {
                        joint_positions_[i] = motor_data_[motor_id].position;
                        joint_velocities_[i] = motor_data_[motor_id].velocity;
                        joint_efforts_[i] = motor_data_[motor_id].current;
                        std::cout << "Motor ID: " << static_cast<int>(motor_id)
                                  << " Pos: " << std::fixed << std::setprecision(4) << motor_data_[motor_id].position
                                  << " Vel: " << std::fixed << std::setprecision(4) << motor_data_[motor_id].velocity
                                  << " Cur: " << std::fixed << std::setprecision(4) << motor_data_[motor_id].current
                                  << " ERROR CODE: " << motor_data_[motor_id].error_code 
                                  << std::endl;
                    }
                }
            }

            // 夹爪反馈 -> 第 7 个关节
            {
                std::lock_guard<std::mutex> gripper_lock(gripper_data_mutex_);
                joint_positions_[motor_ids_.size()] = gripper_data_.position / 255.0;  // normalize to 0-1
                joint_velocities_[motor_ids_.size()] = gripper_data_.velocity;
                joint_efforts_[motor_ids_.size()] = gripper_data_.force;
            }

            msg->position = joint_positions_;
            msg->velocity = joint_velocities_;
            msg->effort = joint_efforts_;
        }

        joint_state_pub_->publish(std::move(msg));
    }

    /**
     * @brief 控制循环线程
     * 定期请求各个关节的反馈数据
     */
    void controlLoop()
    {
        while (control_thread_running_)
        {
            // 如果有高优先级命令正在发送，暂停反馈请求
            if (pause_feedback_loop_.load()) {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }

            if (usb_manager_ && usb_manager_->isReady())
            {
                // 轮流请求电机反馈
                if (!motor_ids_.empty())
                {
                    for(uint8_t i = 0; i < motor_ids_.size(); ++i)
                    {
                        // 再次检查暂停标志
                        if (pause_feedback_loop_.load()) {
                            break;
                        }

                        uint8_t motor_id = motor_ids_[motor_request_index_];

                        motor_request_index_ = (motor_request_index_ + 1) % motor_ids_.size();

                        canfd_frame feedback_req = protocol_converter_.encodeRequestFeedback(motor_id);

                        usb_manager_->sendFrame(feedback_req, false);  // Low priority - allow joint commands to preempt

                        RCLCPP_DEBUG(this->get_logger(), "Requested feedback from motor %d", motor_id);

                        // 添加微小延迟确保发送完全 (100微秒)
                        std::this_thread::sleep_for(std::chrono::microseconds(100));
                    }
                }
            }
            else 
            {
                // 硬件未就绪，等待后重试
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
