# Release Notes

## v1.0.0 - 初始开源版本

### 🎉 重大更新

- **完全开源**: 采用 MIT 许可证，欢迎社区贡献
- **源码开放**: 所有源代码完全开放，支持自由使用和修改
- **社区友好**: 添加完整的贡献指南和行为准则

### ✨ 新功能

- **多种安装方式**: 支持源码安装、发布包安装、APT安装、Debian包安装
- **完整的API**: 支持电机控制、状态监控、多种控制模式
- **丰富的文档**: 详细的使用说明和API文档
- **开源友好**: 完全开放源码，欢迎社区贡献

### 🔧 技术特性

- **C++17 标准**: 使用现代C++特性
- **线程安全**: 内部多线程处理，API线程安全
- **多接口支持**: 支持多个CAN接口
- **实时控制**: 支持速度、位置、力矩、MIT模式

### 📦 安装方式

#### 1. 源码安装（推荐）
```bash
git clone https://github.com/Ding-Kaiyue/hardware-driver.git
cd hardware-driver
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

#### 2. 发布包安装
```bash
wget https://github.com/Ding-Kaiyue/hardware-driver/releases/download/v1.0.0/hardware_driver_v1.0.0.tar.gz
tar -xzf hardware_driver_v1.0.0.tar.gz
cd hardware_driver_release
sudo ./install.sh
```

#### 3. APT安装
```bash
# 添加仓库密钥
wget -qO - https://ding-kaiyue.github.io/hardware-driver/gpg.key | sudo apt-key add -

# 添加仓库
echo "deb [arch=amd64] https://ding-kaiyue.github.io/hardware-driver jammy main" | sudo tee /etc/apt/sources.list.d/hardware-driver.list

# 更新并安装
sudo apt update
sudo apt install libhardware-driver0 libhardware-driver-dev
```

#### 4. Debian包安装
```bash
# 下载并安装
wget https://github.com/Ding-Kaiyue/hardware-driver/releases/download/v1.0.0/libhardware-driver0_1.0.0_amd64.deb
wget https://github.com/Ding-Kaiyue/hardware-driver/releases/download/v1.0.0/libhardware-driver-dev_1.0.0_amd64.deb
sudo dpkg -i libhardware-driver0_1.0.0_amd64.deb
sudo dpkg -i libhardware-driver-dev_1.0.0_amd64.deb
sudo apt-get install -f
```

### 🐛 Bug 修复

- 修复了CAN接口初始化问题
- 改进了错误处理机制
- 优化了内存管理

### 📚 文档更新

- 添加了完整的 README.md
- 创建了贡献指南 (CONTRIBUTING.md)
- 添加了行为准则 (CODE_OF_CONDUCT.md)
- 提供了详细的开发者文档 (DEVELOPER.md)

### 🤝 社区

- 欢迎所有形式的贡献
- 支持 Issue 报告和功能建议
- 接受 Pull Request
- 提供商业技术支持

### 📄 许可证

- 采用 MIT 许可证
- 允许自由使用、修改和分发
- 保留版权声明

### 🔗 链接

- **GitHub**: https://github.com/Ding-Kaiyue/hardware-driver
- **文档**: https://github.com/Ding-Kaiyue/hardware-driver/blob/master/README.md
- **贡献**: https://github.com/Ding-Kaiyue/hardware-driver/blob/master/.github/CONTRIBUTING.md

### 📞 联系方式

- **Email**: kaiyue.ding@raysense.com
- **微信**: d18292819833
- **商业合作**: 欢迎联系进行定制开发和技术支持

---

感谢所有贡献者和用户的支持！ 