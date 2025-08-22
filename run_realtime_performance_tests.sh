#!/bin/bash

# 实时性能测试运行脚本
# 使用方法: ./run_realtime_performance_tests.sh [选项]

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 默认参数
BUILD_DIR="build"
TEST_NAME="test_realtime_performance"
REPORT_DIR="performance_reports"
VERBOSE=false
CLEAN_BUILD=false
RUN_SINGLE_TEST=""

# 帮助信息
show_help() {
    echo "🚀 硬件驱动库实时性能测试脚本"
    echo ""
    echo "使用方法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help              显示此帮助信息"
    echo "  -b, --build-dir DIR     指定构建目录 (默认: build)"
    echo "  -t, --test-name NAME    指定测试名称 (默认: test_realtime_performance)"
    echo "  -r, --report-dir DIR    指定报告目录 (默认: performance_reports)"
    echo "  -v, --verbose           详细输出"
    echo "  -c, --clean-build       清理构建目录后重新构建"
    echo "  -s, --single-test TEST  只运行指定的单个测试"
    echo ""
    echo "示例:"
    echo "  $0                                    # 运行所有测试"
    echo "  $0 -v -c                             # 详细输出，清理构建"
    echo "  $0 -s control_latency                # 只运行控制延迟测试"
    echo "  $0 -r custom_reports                 # 使用自定义报告目录"
    echo ""
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -b|--build-dir)
            BUILD_DIR="$2"
            shift 2
            ;;
        -t|--test-name)
            TEST_NAME="$2"
            shift 2
            ;;
        -r|--report-dir)
            REPORT_DIR="$2"
            shift 2
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -c|--clean-build)
            CLEAN_BUILD=true
            shift
            ;;
        -s|--single-test)
            RUN_SINGLE_TEST="$2"
            shift 2
            ;;
        *)
            echo -e "${RED}错误: 未知选项 $1${NC}"
            show_help
            exit 1
            ;;
    esac
done

# 打印配置信息
echo -e "${BLUE}🔧 测试配置:${NC}"
echo "  构建目录: $BUILD_DIR"
echo "  测试名称: $TEST_NAME"
echo "  报告目录: $REPORT_DIR"
echo "  详细输出: $VERBOSE"
echo "  清理构建: $CLEAN_BUILD"
if [[ -n "$RUN_SINGLE_TEST" ]]; then
    echo "  单测试: $RUN_SINGLE_TEST"
fi
echo ""

# 检查构建目录
if [[ ! -d "$BUILD_DIR" ]]; then
    echo -e "${YELLOW}⚠️  构建目录不存在，创建中...${NC}"
    mkdir -p "$BUILD_DIR"
fi

# 检查是否在正确的目录
if [[ ! -f "CMakeLists.txt" ]]; then
    echo -e "${RED}❌ 错误: 请在项目根目录运行此脚本${NC}"
    exit 1
fi

# 清理构建目录（如果需要）
if [[ "$CLEAN_BUILD" == true ]]; then
    echo -e "${YELLOW}🧹 清理构建目录...${NC}"
    rm -rf "$BUILD_DIR"/*
fi

# 进入构建目录
cd "$BUILD_DIR"

# 配置CMake
echo -e "${BLUE}🔧 配置CMake...${NC}"
if [[ "$VERBOSE" == true ]]; then
    cmake .. -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release
else
    cmake .. -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release > /dev/null
fi

# 构建项目
echo -e "${BLUE}🔨 构建项目...${NC}"
if [[ "$VERBOSE" == true ]]; then
    make -j$(nproc)
else
    make -j$(nproc) > /dev/null
fi

# 检查测试可执行文件是否存在
TEST_EXECUTABLE="tests/$TEST_NAME"
if [[ ! -f "$TEST_EXECUTABLE" ]]; then
    echo -e "${RED}❌ 错误: 测试可执行文件不存在: $TEST_EXECUTABLE${NC}"
    echo "可用的测试文件:"
    ls -la tests/ 2>/dev/null || echo "  无测试文件"
    exit 1
fi

# 创建报告目录
mkdir -p "../$REPORT_DIR"

# 保存当前目录的绝对路径
CURRENT_DIR=$(pwd)
REPORT_DIR_ABSOLUTE="$CURRENT_DIR/../$REPORT_DIR"
cd "../$REPORT_DIR"

# 生成时间戳
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
REPORT_FILE="realtime_performance_${TIMESTAMP}.csv"

# 创建报告文件头
echo "Test Name,Sample Count,Min Latency (μs),Max Latency (μs),Mean Latency (μs),Std Deviation (μs),P50 Latency (μs),P95 Latency (μs),P99 Latency (μs),Timestamp" > "$REPORT_FILE"

# 运行测试
echo -e "${BLUE}🚀 开始运行实时性能测试...${NC}"

if [[ -n "$RUN_SINGLE_TEST" ]]; then
    echo -e "${BLUE}🎯 运行单测试: $RUN_SINGLE_TEST${NC}"
    if [[ "$VERBOSE" == true ]]; then
        "$CURRENT_DIR/$TEST_EXECUTABLE" --gtest_filter="*$RUN_SINGLE_TEST*"
    else
        "$CURRENT_DIR/$TEST_EXECUTABLE" --gtest_filter="*$RUN_SINGLE_TEST*" 2>&1 | tee "test_output_${TIMESTAMP}.log"
    fi
else
    echo -e "${BLUE}🎯 运行所有测试...${NC}"
    if [[ "$VERBOSE" == true ]]; then
        "$CURRENT_DIR/$TEST_EXECUTABLE"
    else
        "$CURRENT_DIR/$TEST_EXECUTABLE" 2>&1 | tee "test_output_${TIMESTAMP}.log"
    fi
fi

# 检查测试结果
if [[ $? -eq 0 ]]; then
    echo -e "${GREEN}✅ 实时性能测试完成！${NC}"
    
    # 检查是否生成了性能报告
    if [[ -f "realtime_performance_report.csv" ]]; then
        echo -e "${GREEN}📊 性能报告已生成: realtime_performance_report.csv${NC}"
        
        # 显示报告摘要
        echo -e "${BLUE}📈 性能报告摘要:${NC}"
        echo "============================================================"
        tail -n +2 realtime_performance_report.csv | while IFS=',' read -r test_name sample_count min max mean std p50 p95 p99; do
            echo "测试: $test_name"
            echo "  样本数: $sample_count"
            echo "  延迟范围: ${min}μs - ${max}μs"
            echo "  平均延迟: ${mean}μs"
            echo "  P99延迟: ${p99}μs"
            echo "  ----------------------------------------"
        done
    fi
    
    # 显示最终报告文件
    echo -e "${GREEN}📁 最终报告文件: $REPORT_FILE${NC}"
    
else
    echo -e "${RED}❌ 测试执行失败！${NC}"
    echo "请检查测试输出日志: test_output_${TIMESTAMP}.log"
    exit 1
fi

# 显示系统信息
echo -e "${BLUE}💻 系统信息:${NC}"
echo "  操作系统: $(uname -a)"
echo "  CPU核心数: $(nproc)"
echo "  内存: $(free -h | grep Mem | awk '{print $2}')"
echo "  当前时间: $(date)"

# 检查CAN硬件
echo -e "${BLUE}🔌 CAN硬件检查:${NC}"
if ls /sys/class/net/can* >/dev/null 2>&1; then
    echo -e "${GREEN}  ✅ 检测到CAN接口:${NC}"
    ls /sys/class/net/can* | while read interface; do
        echo "    - $interface"
    done
else
    echo -e "${YELLOW}  ⚠️  未检测到CAN接口，将使用模拟环境${NC}"
fi

echo -e "${GREEN}🎉 实时性能测试脚本执行完成！${NC}" 