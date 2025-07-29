#!/bin/bash

echo "=== 构建Debian包 ==="

# 检查依赖
if ! command -v dpkg-buildpackage &> /dev/null; then
    echo "错误: 需要安装 devscripts 和 build-essential"
    echo "运行: sudo apt install devscripts build-essential"
    exit 1
fi

# 更新changelog中的日期
sed -i "s/\$(date -R)/$(date -R)/" debian/changelog

# 构建包
echo "构建Debian包..."
dpkg-buildpackage -b -us -uc

if [ $? -eq 0 ]; then
    echo "=== 构建成功 ==="
    echo "生成的包文件:"
    ls -la ../*.deb ../*.dsc ../*.tar.gz ../*.changes 2>/dev/null || true
    
    echo ""
    echo "安装方法:"
    echo "  sudo dpkg -i ../libhardware-driver0_*.deb"
    echo "  sudo dpkg -i ../libhardware-driver-dev_*.deb"
    echo "  sudo apt-get install -f  # 安装依赖"
else
    echo "构建失败！"
    exit 1
fi 