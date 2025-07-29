# 硬件驱动库

一个简单易用的硬件驱动库，提供统一的API来控制电机和获取状态。

## 快速安装

### 方法1: 源码安装（推荐）
```bash
# 克隆仓库
git clone https://github.com/Ding-Kaiyue/hardware-driver.git
cd hardware-driver

# 编译安装
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 方法2: 发布包安装
```bash
# 下载发布包
wget https://github.com/Ding-Kaiyue/hardware-driver/releases/download/v1.0.0/hardware_driver_v1.0.0.tar.gz

# 解压并安装
tar -xzf hardware_driver_v1.0.0.tar.gz
cd hardware_driver_release
sudo ./install.sh
```

### 方法3: APT安装
```bash
# 添加仓库密钥
wget -qO - https://ding-kaiyue.github.io/hardware-driver/gpg.key | sudo apt-key add -

# 添加仓库
echo "deb [arch=amd64] https://ding-kaiyue.github.io/hardware-driver jammy main" | sudo tee /etc/apt/sources.list.d/hardware-driver.list

# 更新并安装
sudo apt update
sudo apt install libhardware-driver0 libhardware-driver-dev
```

### 方法4: Debian包安装
```bash
# 下载并安装
wget https://github.com/Ding-Kaiyue/hardware-driver/releases/download/v1.0.0/libhardware-driver0_1.0.0_amd64.deb
wget https://github.com/Ding-Kaiyue/hardware-driver/releases/download/v1.0.0/libhardware-driver-dev_1.0.0_amd64.deb
sudo dpkg -i libhardware-driver0_1.0.0_amd64.deb
sudo dpkg -i libhardware-driver-dev_1.0.0_amd64.deb
sudo apt-get install -f
```

## 快速开始

```cpp
#include "hardware_driver.hpp"
#include <iostream>

int main() {
    // 配置硬件
    std::vector<std::string> interfaces = {"can0"};
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 2, 3, 4}}
    };
    
    try {
        // 创建驱动
        hardware_driver::HardwareDriver driver(interfaces, motor_config);
        
        // 使能电机
        driver.enable_motor("can0", 1, 4);
        
        // 控制电机
        driver.control_motor_in_velocity_mode("can0", 1, 5.0);
        
        // 获取状态
        auto status = driver.get_motor_status("can0", 1);
        std::cout << "位置: " << status.position << std::endl;
        
        // 失能电机
        driver.disable_motor("can0", 1);
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
```

## 编译

```bash
# 使用g++编译您的程序
g++ -std=c++17 -lhardware_driver -lpthread your_program.cpp -o your_program

# 使用pkg-config（如果已安装）
g++ -std=c++17 $(pkg-config --cflags --libs hardware_driver) your_program.cpp -o your_program
```

## 主要功能

- **多种控制模式**: 速度、位置、力矩、MIT模式
- **实时状态监控**: 获取电机位置、速度、力矩等状态
- **多接口支持**: 支持多个CAN接口
- **线程安全**: 内部多线程处理，API线程安全
- **简单易用**: 只需包含一个头文件

## API概览

```cpp
// 使能/失能电机
driver.enable_motor("can0", 1, 4);
driver.disable_motor("can0", 1);

// 控制电机
driver.control_motor_in_velocity_mode("can0", 1, 10.0);    // 速度模式
driver.control_motor_in_position_mode("can0", 1, 10.0);    // 位置模式
driver.control_motor_in_effort_mode("can0", 1, 5.0);       // 力矩模式
driver.control_motor_in_mit_mode("can0", 1, 1.0, 2.0, 3.0); // MIT模式

// 获取状态
auto status = driver.get_motor_status("can0", 1);
auto all_status = driver.get_all_motor_status("can0");
```

## 依赖项

- C++17 编译器
- pthread 库
- CAN 接口支持

## 故障排除

1. **CAN接口未找到**: 检查CAN接口配置
2. **权限错误**: 使用sudo运行程序
3. **库文件未找到**: 运行 `sudo ldconfig`

## 获取帮助

如果您在使用过程中遇到问题，可以通过以下方式获取帮助：

### GitHub Issues
- **使用问题**: [提交使用问题](https://github.com/Ding-Kaiyue/hardware-driver/issues/new?template=usage_question.md) - 提交使用中的问题，我们会提供指导
- **Bug 报告**: [报告 Bug](https://github.com/Ding-Kaiyue/hardware-driver/issues/new?template=bug_report.md) - 报告发现的 Bug，我们会尽快修复
- **功能建议**: [提出建议](https://github.com/Ding-Kaiyue/hardware-driver/issues/new?template=feature_request.md) - 提出新功能建议，我们会认真考虑

### 联系方式
- **Email**: kaiyue.ding@raysense.com
- **微信**: d18292819833
- **商业合作**: 欢迎联系进行定制开发和技术支持

## 📚 文档

更多文档请查看 [docs/](docs/) 目录，包括API参考、开发者指南等。

## 许可证

MIT License - 详见 [LICENSE](LICENSE) 文件 