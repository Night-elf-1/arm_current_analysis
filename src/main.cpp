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
//#include "MotorDAQ.h"
#include "../include/MotorDAQ/MotorDAQ.h"

using namespace std;

// 全局变量，标志是否收到SIGTERM信号
volatile sig_atomic_t g_receivedSigTerm = 0;

// SIGTERM信号处理函数
void sigtermHandler(int signal) {
    std::cout << "Received SIGTERM signal." << std::endl;
    g_receivedSigTerm = 1;
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
    //! 激光测距仪扫描线程
    while (!g_receivedSigTerm)
    {
        m_MotorDAQ->run();
        //! 处理服务请求
        // ros::spinOnce();
        //! 以固定频率休眠
        // rate.sleep();
    }
    delete m_MotorDAQ;
    // sleep(1);
    // ros::shutdown();
    return 0;
}
