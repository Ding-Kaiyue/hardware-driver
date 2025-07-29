# 文档结构说明

## 用户文档

### README.md (主文档)
- **目标读者**: 所有用户
- **内容**: 快速安装、基本使用、API概览
- **特点**: 简洁明了，重点突出

### examples/simple_example.cpp (示例代码)
- **目标读者**: 开发者
- **内容**: 完整的使用示例
- **特点**: 可直接编译运行

## 开发者文档

### DEVELOPER.md (开发者指南)
- **目标读者**: 项目维护者和贡献者
- **内容**: 项目结构、架构设计、开发指南、发布流程
- **特点**: 详细的技术文档

### 发布包中的文档
- **hardware_driver_release/README.md**: 安装说明
- **hardware_driver_release/README_INSTALL.md**: 详细安装指南

## 文档层次

```
用户文档 (简单易用)
├── README.md                    # 快速开始
├── examples/simple_example.cpp  # 使用示例
└── 发布包文档                   # 安装说明

开发者文档 (详细技术)
├── DEVELOPER.md                 # 完整开发指南
├── CMakeLists.txt              # 构建配置
├── build.sh                    # 构建脚本
└── 发布脚本                    # 发布工具
```

## 文档维护

### 更新用户文档
- 修改 `README.md`
- 更新示例代码
- 重新生成发布包

### 更新开发者文档
- 修改 `DEVELOPER.md`
- 更新构建脚本
- 更新发布流程

### 发布新版本
1. 更新版本号
2. 更新文档
3. 创建发布包
4. 上传到GitHub 