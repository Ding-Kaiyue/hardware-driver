#ifndef __PERFORMANCE_TEST_FRAMEWORK_HPP__
#define __PERFORMANCE_TEST_FRAMEWORK_HPP__

#include <chrono>
#include <vector>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <cmath>
#include <gtest/gtest.h>

namespace performance_test {

// 高精度时钟类型
using Clock = std::chrono::high_resolution_clock;
using TimePoint = Clock::time_point;
using Duration = std::chrono::duration<double, std::micro>;

// 性能统计结构
struct PerformanceStats {
    double min_latency;
    double max_latency;
    double mean_latency;
    double p50_latency;
    double p95_latency;
    double p99_latency;
    double std_deviation;
    int sample_count;
    
    PerformanceStats() : min_latency(0), max_latency(0), mean_latency(0), 
                        p50_latency(0), p95_latency(0), p99_latency(0), 
                        std_deviation(0), sample_count(0) {}
};

// 性能测试基类
class PerformanceTestBase {
protected:
    std::vector<double> latencies_;
    
    // 记录延迟
    void record_latency(double latency_us) {
        latencies_.push_back(latency_us);
    }
    
    // 计算性能统计
    PerformanceStats calculate_stats() const {
        if (latencies_.empty()) {
            return PerformanceStats{};
        }
        
        PerformanceStats stats;
        stats.sample_count = latencies_.size();
        
        // 排序用于计算分位数
        std::vector<double> sorted_latencies = latencies_;
        std::sort(sorted_latencies.begin(), sorted_latencies.end());
        
        // 基本统计
        stats.min_latency = sorted_latencies.front();
        stats.max_latency = sorted_latencies.back();
        
        // 平均值
        double sum = std::accumulate(sorted_latencies.begin(), sorted_latencies.end(), 0.0);
        stats.mean_latency = sum / stats.sample_count;
        
        // 分位数
        stats.p50_latency = sorted_latencies[stats.sample_count * 0.5];
        stats.p95_latency = sorted_latencies[stats.sample_count * 0.95];
        stats.p99_latency = sorted_latencies[stats.sample_count * 0.99];
        
        // 标准差
        double variance = 0.0;
        for (double latency : sorted_latencies) {
            variance += std::pow(latency - stats.mean_latency, 2);
        }
        stats.std_deviation = std::sqrt(variance / stats.sample_count);
        
        return stats;
    }
    
    // 打印性能报告
    void print_performance_report(const std::string& test_name, const PerformanceStats& stats) const {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "📊 " << test_name << " 性能测试报告" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "样本数量: " << stats.sample_count << std::endl;
        std::cout << "延迟统计 (μs):" << std::endl;
        std::cout << "  最小值: " << std::fixed << std::setprecision(2) << stats.min_latency << std::endl;
        std::cout << "  最大值: " << std::fixed << std::setprecision(2) << stats.max_latency << std::endl;
        std::cout << "  平均值: " << std::fixed << std::setprecision(2) << stats.mean_latency << std::endl;
        std::cout << "  标准差: " << std::fixed << std::setprecision(2) << stats.std_deviation << std::endl;
        std::cout << "  50%分位: " << std::fixed << std::setprecision(2) << stats.p50_latency << std::endl;
        std::cout << "  95%分位: " << std::fixed << std::setprecision(2) << stats.p95_latency << std::endl;
        std::cout << "  99%分位: " << std::fixed << std::setprecision(2) << stats.p99_latency << std::endl;
        std::cout << std::string(60, '=') << std::endl;
    }
    
    // 保存性能报告到文件
    void save_performance_report(const std::string& filename, const std::string& test_name, 
                                const PerformanceStats& stats) const {
        std::ofstream file(filename, std::ios::app);
        if (file.is_open()) {
            file << test_name << ","
                 << stats.sample_count << ","
                 << stats.min_latency << ","
                 << stats.max_latency << ","
                 << stats.mean_latency << ","
                 << stats.std_deviation << ","
                 << stats.p50_latency << ","
                 << stats.p95_latency << ","
                 << stats.p99_latency << std::endl;
            file.close();
        }
    }
    
    // 验证性能指标
    void verify_performance_targets(const PerformanceStats& stats, 
                                   double p99_target, double p95_target, double p50_target) {
        EXPECT_LT(stats.p99_latency, p99_target) 
            << "P99延迟 " << stats.p99_latency << "μs 超过目标 " << p99_target << "μs";
        
        EXPECT_LT(stats.p95_latency, p95_target) 
            << "P95延迟 " << stats.p95_latency << "μs 超过目标 " << p95_target << "μs";
        
        EXPECT_LT(stats.p50_latency, p50_target) 
            << "P50延迟 " << stats.p50_latency << "μs 超过目标 " << p50_target << "μs";
    }
    
    // 清理测试数据
    void clear_test_data() {
        latencies_.clear();
    }
};

// 性能测试宏
#define PERFORMANCE_TEST(test_name, target_p99, target_p95, target_p50) \
    TEST_F(PerformanceTestFixture, test_name) { \
        auto stats = run_##test_name##_test(); \
        print_performance_report(#test_name, stats); \
        verify_performance_targets(stats, target_p99, target_p95, target_p50); \
        clear_test_data(); \
    }

// 延迟测量宏
#define MEASURE_LATENCY(operation, result_var) \
    do { \
        auto start = Clock::now(); \
        operation; \
        auto end = Clock::now(); \
        result_var = std::chrono::duration<double, std::micro>(end - start).count(); \
    } while(0)

} // namespace performance_test

#endif // __PERFORMANCE_TEST_FRAMEWORK_HPP__ 