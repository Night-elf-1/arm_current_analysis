/**********************************************************************************************
 * @File name: main.cpp
 * @Author: 黄文开 wenkai.h@qq.com
 * @Version: V1.0
 * @Description: 485总线设备
 * @Date: 2022-09-07  16:17:47
 * @Other: 无
 * @LastEditors: 黄文开 wenkai.h@qq.com
 * @LastEditTime: new Date()
 * @copyright: © SHIBO TECH LTD. 2020-2030.All rights reserved.
 **********************************************************************************************/
// #include "ros/ros.h"
#include "csignal"
#include <glog/logging.h>
// #include "ros/package.h"
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem.hpp>
#include <thread>
#include <functional>
#include <iostream>
#include <chrono>
#include <Eigen/Dense>
#include "eigen3/Eigen/Core"
#include "../include/MotorDAQ/MotorDAQ.h"

using namespace std;
using namespace Eigen;

// 全局变量，标志是否收到SIGTERM信号
volatile sig_atomic_t g_receivedSigTerm = 0;
constexpr int TIMER_INTERVAL_MS = 100;

// SIGTERM信号处理函数
void sigtermHandler(int signal) {
    std::cout << "Received SIGTERM signal." << std::endl;
    g_receivedSigTerm = 1;
}

VectorXd loadWeights(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "无法打开文件: " << filename << endl;
        return VectorXd();
    }

    vector<double> weights;
    string line;
    while (getline(file, line)) {
        weights.push_back(stod(line));
    }

    file.close();

    VectorXd w(weights.size());
    for (size_t i = 0; i < weights.size(); ++i) {
        w[i] = weights[i];
    }

    return w;
}

double trapezoidalIntegration(const vector<double>& y, double dx) {
    double area = 0.0;
    for (size_t i = 1; i < y.size(); ++i) {
        area += (y[i] + y[i - 1]) * dx / 2.0;
    }
    return area;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    // ros::init(argc, argv, "MotorDAQ");
    // ros::NodeHandle nh;

    //! glog初始化
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    //! log输出路径设置
    // std::string logPath = ros::package::getPath("MotorDAQ") + "/log";
    // if (!boost::filesystem::exists(logPath)) {
    //     boost::filesystem::create_directory(logPath);
    // }
    //! 设置日志文件路径
    // FLAGS_log_dir = logPath;
    //! 缓存的最大时长，超时会写入文件
    FLAGS_logbufsecs = 0;
    //! 单个日志文件最大，单位M
    FLAGS_max_log_size = 10;
    // LOG(INFO) << "********初始化日志文件完成********";
    
    MotorDAQ *m_MotorDAQ = new MotorDAQ();
    //! 设置循环频率为40Hz
    // ros::Rate rate(10); 
    //！ 设置SIGTERM信号处理函数
    std::signal(SIGTERM, sigtermHandler);
    auto next_trigger_time = std::chrono::steady_clock::now();
    //! 激光测距仪扫描线程
    while (!g_receivedSigTerm)
    {
        // 更新下一次触发的时间点
        next_trigger_time += std::chrono::milliseconds(TIMER_INTERVAL_MS);
        m_MotorDAQ->run();

        // 等待直到下一次触发时间
        std::this_thread::sleep_until(next_trigger_time);
    }
    
    double currentAreas;
    for (size_t i = 0; i < arms.size(); i++)
    {
        currentAreas += trapezoidalIntegration(arms[i], 1);
    }
    currentAreas = currentAreas / 8;
    
    delete m_MotorDAQ;

    return 0;
}
