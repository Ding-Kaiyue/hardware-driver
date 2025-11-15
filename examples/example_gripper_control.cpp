/*********************************************************************
 * @file        example_gripper_control.cpp
 * @brief       夹爪控制交互式示例程序
 *
 * 本示例演示如何使用硬件驱动库控制不同类型的夹爪，包括：
 * - OmniPicker 全向夹爪
 * - PGC 夹爪
 * - 自定义参数控制
 *********************************************************************/

#include "hardware_driver/interface/robot_hardware.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include <map>
#include <string>
#include <limits>

// 清空输入缓冲区
void clear_input() {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

// 获取用户输入的整数（带范围检查）
int get_input(const std::string& prompt, int min, int max) {
    int value;
    while (true) {
        std::cout << prompt;
        if (std::cin >> value) {
            if (value >= min && value <= max) {
                clear_input();
                return value;
            } else {
                std::cout << "输入超出范围，请输入 " << min << " 到 " << max << " 之间的数字" << std::endl;
            }
        } else {
            std::cout << "无效输入，请输入一个数字" << std::endl;
            clear_input();
        }
    }
}

// 显示主菜单
void show_menu() {
    std::cout << "\n======================================" << std::endl;
    std::cout << "       夹爪控制交互式菜单" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "1. 打开夹爪" << std::endl;
    std::cout << "2. 关闭夹爪" << std::endl;
    std::cout << "3. 自定义控制（设置位置、速度、力）" << std::endl;
    std::cout << "0. 退出程序" << std::endl;
    std::cout << "======================================" << std::endl;
}

// 显示夹爪类型选择菜单
void show_gripper_types() {
    std::cout << "\n请选择夹爪类型:" << std::endl;
    std::cout << "0. OmniPicker 全向夹爪" << std::endl;
    std::cout << "1. PGC 夹爪" << std::endl;
}

// 获取夹爪类型名称
std::string get_gripper_type_name(uint8_t type) {
    switch (type) {
        case 0: return "OmniPicker";
        case 1: return "PGC";
        default: return "未知";
    }
}

int main() {
    std::cout << "=== 夹爪控制交互式示例程序 ===" << std::endl;
    std::cout << "本示例允许您交互式地控制不同类型的夹爪" << std::endl;
    std::cout << "\n初始化硬件驱动..." << std::endl;

    try {
        // 创建电机驱动（RobotHardware需要电机驱动）
        auto motor_driver = hardware_driver::createCanFdMotorDriver({"can0"});

        // 配置接口和电机ID映射（这里为空，因为我们只演示夹爪）
        std::map<std::string, std::vector<uint32_t>> interface_motor_config;
        interface_motor_config["can0"] = {};  // 空列表，不控制电机

        // 创建 RobotHardware 实例
        RobotHardware robot(motor_driver, interface_motor_config);

        // 创建并设置夹爪驱动
        auto gripper_driver = hardware_driver::createCanFdGripperDriver({"can0"});
        robot.set_gripper_driver(gripper_driver);

        std::cout << "硬件驱动初始化完成！" << std::endl;

        // 主循环
        while (true) {
            show_menu();
            int choice = get_input("请选择操作 (0-3): ", 0, 3);

            if (choice == 0) {
                std::cout << "\n退出程序..." << std::endl;
                break;
            }

            // 选择夹爪类型
            show_gripper_types();
            int gripper_type = get_input("请选择夹爪类型 (0-1): ", 0, 1);
            std::string gripper_name = get_gripper_type_name(gripper_type);

            switch (choice) {
                case 1: {
                    // 打开夹爪
                    int velocity = get_input("请输入速度 (0-100%): ", 0, 100);
                    int effort = get_input("请输入力 (0-100%): ", 0, 100);

                    std::cout << "\n正在打开 " << gripper_name << " 夹爪..." << std::endl;
                    std::cout << "  速度: " << velocity << "%" << std::endl;
                    std::cout << "  力: " << effort << "%" << std::endl;

                    robot.open_gripper("can0", gripper_type, velocity, effort);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    std::cout << "✓ 命令已发送" << std::endl;
                    break;
                }

                case 2: {
                    // 关闭夹爪
                    int velocity = get_input("请输入速度 (0-100%): ", 0, 100);
                    int effort = get_input("请输入力 (0-100%): ", 0, 100);

                    std::cout << "\n正在关闭 " << gripper_name << " 夹爪..." << std::endl;
                    std::cout << "  速度: " << velocity << "%" << std::endl;
                    std::cout << "  力: " << effort << "%" << std::endl;

                    robot.close_gripper("can0", gripper_type, velocity, effort);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    std::cout << "✓ 命令已发送" << std::endl;
                    break;
                }

                case 3: {
                    // 自定义控制
                    int position = get_input("请输入位置 (0-100%, 0=完全打开, 100=完全关闭): ", 0, 100);
                    int velocity = get_input("请输入速度 (0-100%): ", 0, 100);
                    int effort = get_input("请输入力 (0-100%): ", 0, 100);

                    std::cout << "\n正在控制 " << gripper_name << " 夹爪..." << std::endl;
                    std::cout << "  位置: " << position << "%" << std::endl;
                    std::cout << "  速度: " << velocity << "%" << std::endl;
                    std::cout << "  力: " << effort << "%" << std::endl;

                    robot.control_gripper("can0", gripper_type, position, velocity, effort);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    std::cout << "✓ 命令已发送" << std::endl;
                    break;
                }
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "\n错误: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\n夹爪控制示例程序结束。" << std::endl;
    return 0;
}
