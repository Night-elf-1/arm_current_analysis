/**
 * @file        MotorDAQ.cpp
 * @brief       电机数据采集
 * @details     
 * @author      黄文开 wenkai.h@qq.com
 * @date        2024-08-07  11:29:49
 * @version     V1.0
 * @copyright   © SHIBO TECH LTD. 2020-2030.All rights reserved. 世泊智能装备科技有限公司
 ************************************************************************************
 * @attention
 * 硬件平台：工控机
 * @par 修改日志
 * <table>
 * <tr><th>Date          <th>Version    <th>Author    <th>Description
 * <tr><td>2024-08-07    <td>V1.0       <td>wenkai    <td>创建初始版本
 * <table>
 ************************************************************************************
 */
#include "MotorDAQ.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>

using namespace std;

static WORD ConnectNo = 3; //连接号，可选 0--7 
static WORD PortNum = 2;
static std::string Daddr = "192.168.11.11"; 
static const double Dirve_FD144S_MaxA = 120;
static const double Dirve_FD134S_MaxA = 80;
// 驱动电机         1Arms=[2048/(驱动器峰值电流Ipeak/1.414)]dec 峰值电流=120Ap
static const double Dirve_FD144S_A_Unit = 2048/Dirve_FD144S_MaxA/1.414;
// 夹臂和转向电机    1Arms=[2048/(驱动器峰值电流Ipeak/1.414)]dec 峰值电流=80Ap
static const double Dirve_FD134S_A_Unit = 2048/Dirve_FD134S_MaxA/1.414;

MotorDAQ::MotorDAQ(ros::NodeHandle *nh)
: m_nh(nh)
, m_readFlag(true)
, m_QueueFirstHasData(false)
, m_QueueSecondHasData(false)
, m_fileName("")
{
    m_subBehavior = m_nh->subscribe("BehaviorAction", 100, &MotorDAQ::behaviorActionCallback, this);
    connectInit();
    m_thread = new std::thread(&MotorDAQ::runSave, this);
}

MotorDAQ::~MotorDAQ()
{
    short rtn;
    rtn = smc_board_close(ConnectNo);
    std::cout<<"rtn5555 = "<<rtn<<std::endl;
    if (rtn != 0) {
        printf("smc_board_init rtn = %d\n",rtn);
    }
    m_thread->join();
}

void MotorDAQ::behaviorActionCallback(const std_msgs::String& msg)
{
    m_mutex.lock();
    m_action = msg.data;
    // std::cout << "topic" << std::endl;
    m_mutex.unlock();
}

void MotorDAQ::saveDAQToFile(const std::string &fileName)
{
    std::ofstream outfile(fileName, std::ios_base::app);
    if (!outfile.is_open()) {
        std::cerr << "Unable to open file." << std::endl;
        return;
    }
    // 设置输出的浮点数精度为两位小数
    outfile << fixed << setprecision(2);
    if (m_QueueFirstHasData) {
        while (!m_QueueFirst.empty()) {
            DataFrame data = m_QueueFirst.front();
            m_QueueFirst.pop();
            // 将数据帧写入文件
            outfile << data.timestamp << " | " 
            << data.action << " | " 
            << data.currentAxis[0] << " | " 
            << data.currentAxis[1] << " | " 
            << data.currentAxis[2] << " | " 
            << data.currentAxis[3] << " | " 
            << data.currentAxis[4] << " | " 
            << data.currentAxis[5] << " | " 
            << data.currentAxis[6] << " | " 
            << data.currentAxis[7] << " | " 
            << data.currentAxis[8] << " | " 
            << data.currentAxis[9] << " | " 
            << data.currentAxis[10] << " | " 
            << data.currentAxis[11] << " | " 
            << data.currentAxis[12] << " | " 
            << data.currentAxis[13] << " | " 
            << data.currentAxis[14] << " | " 
            << data.currentAxis[15] 
            << std::endl;
        }
        m_QueueFirstHasData = false;
    }

    if (m_QueueSecondHasData) {
        while (!m_QueueSecond.empty()) {
            DataFrame data = m_QueueSecond.front();
            m_QueueSecond.pop();
            // 将数据帧写入文件
            outfile << data.timestamp << " | " 
            << data.action << " | " 
            << data.currentAxis[0] << " | " 
            << data.currentAxis[1] << " | " 
            << data.currentAxis[2] << " | " 
            << data.currentAxis[3] << " | " 
            << data.currentAxis[4] << " | " 
            << data.currentAxis[5] << " | " 
            << data.currentAxis[6] << " | " 
            << data.currentAxis[7] << " | " 
            << data.currentAxis[8] << " | " 
            << data.currentAxis[9] << " | " 
            << data.currentAxis[10] << " | " 
            << data.currentAxis[11] << " | " 
            << data.currentAxis[12] << " | " 
            << data.currentAxis[13] << " | " 
            << data.currentAxis[14] << " | " 
            << data.currentAxis[15] 
            << std::endl;
        }
        m_QueueSecondHasData = false;
    }
    outfile.close();
}

void MotorDAQ::run(void)
{
    short ret;
    DWORD Value;
    DataFrame data;
    // 获取当前时间
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(epoch);
    epoch -= seconds;
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
    
    // 格式化时间戳
    time_t now_c = std::chrono::system_clock::to_time_t(now);
    char timestamp[100];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", localtime(&now_c));
    std::sprintf(timestamp, "%s:%03d", timestamp, milliseconds.count());
    data.timestamp = timestamp;
    data.action = m_action;
    ret = nmcs_read_txpdo_extra(ConnectNo, PortNum, steeringWheel1, 1, &Value);
    // std::cout << ret << "," << data.currentAxis[0] << " | ";
    data.currentAxis[steeringWheel1] = static_cast<int16_t>(Value)/Dirve_FD134S_A_Unit;
    ret = nmcs_read_txpdo_extra(ConnectNo, PortNum, drivingWheel1, 1, &Value);
    data.currentAxis[drivingWheel1] = static_cast<int16_t>(Value)/Dirve_FD144S_A_Unit;
    // std::cout << ret << "," << data.currentAxis[1] << std::endl;
    ret = nmcs_read_txpdo_extra(ConnectNo, PortNum, rightarm2, 1, &Value);
    // std::cout << ret << "," << data.currentAxis[2] << " | ";
    data.currentAxis[rightarm2] = static_cast<int16_t>(Value)/Dirve_FD134S_A_Unit;
    ret = nmcs_read_txpdo_extra(ConnectNo, PortNum, rightarm1, 1, &Value);
    // std::cout << ret << "," << data.currentAxis[3] << " | ";
    data.currentAxis[rightarm1] = static_cast<int16_t>(Value)/Dirve_FD134S_A_Unit;
    ret = nmcs_read_txpdo_extra(ConnectNo, PortNum, leftarm2, 1, &Value);
    // std::cout << ret << "," << data.currentAxis[4] << " | ";
    data.currentAxis[leftarm2] = static_cast<int16_t>(Value)/Dirve_FD134S_A_Unit;
    ret = nmcs_read_txpdo_extra(ConnectNo, PortNum, leftarm1, 1, &Value);
    // std::cout << ret << "," << data.currentAxis[5] << " | ";
    data.currentAxis[leftarm1] = static_cast<int16_t>(Value)/Dirve_FD134S_A_Unit;
    ret = nmcs_read_txpdo_extra(ConnectNo, PortNum, steeringWheel2, 1, &Value);
    // std::cout << ret << "," << data.currentAxis[6] << " | ";
    data.currentAxis[steeringWheel2] = static_cast<int16_t>(Value)/Dirve_FD134S_A_Unit;
    ret = nmcs_read_txpdo_extra(ConnectNo, PortNum, drivingWheel2, 1, &Value);
    // std::cout << ret << "," << data.currentAxis[7] << " | ";
    data.currentAxis[drivingWheel2] = static_cast<int16_t>(Value)/Dirve_FD144S_A_Unit;
    ret = nmcs_read_txpdo_extra(ConnectNo, PortNum, steeringWheel3, 1, &Value);
    // std::cout << ret << "," << data.currentAxis[8] << " | ";
    data.currentAxis[steeringWheel3] = static_cast<int16_t>(Value)/Dirve_FD134S_A_Unit;
    ret = nmcs_read_txpdo_extra(ConnectNo, PortNum, drivingWheel3, 1, &Value);
    // std::cout << ret << "," << data.currentAxis[9] << " | ";
    data.currentAxis[drivingWheel3] = static_cast<int16_t>(Value)/Dirve_FD144S_A_Unit;
    ret = nmcs_read_txpdo_extra(ConnectNo, PortNum, rightarm4, 1, &Value);
    // std::cout << ret << "," << data.currentAxis[10] << " | ";
    data.currentAxis[rightarm4] = static_cast<int16_t>(Value)/Dirve_FD134S_A_Unit;
    ret = nmcs_read_txpdo_extra(ConnectNo, PortNum, rightarm3, 1, &Value);
    // std::cout << ret << "," << data.currentAxis[11] << " | ";
    data.currentAxis[rightarm3] = static_cast<int16_t>(Value)/Dirve_FD134S_A_Unit;
    ret = nmcs_read_txpdo_extra(ConnectNo, PortNum, leftarm4, 1, &Value);
    // std::cout << ret << "," << data.currentAxis[12] << " | ";
    data.currentAxis[leftarm4] = static_cast<int16_t>(Value)/Dirve_FD134S_A_Unit;
    ret = nmcs_read_txpdo_extra(ConnectNo, PortNum, leftarm3, 1, &Value);
    // std::cout << ret << "," << data.currentAxis[13] << " | ";
    data.currentAxis[leftarm3] = static_cast<int16_t>(Value)/Dirve_FD134S_A_Unit;
    ret = nmcs_read_txpdo_extra(ConnectNo, PortNum, steeringWheel4, 1, &Value);
    // std::cout << ret << "," << data.currentAxis[14] << " | ";
    data.currentAxis[steeringWheel4] = static_cast<int16_t>(Value)/Dirve_FD134S_A_Unit;
    ret = nmcs_read_txpdo_extra(ConnectNo, PortNum, drivingWheel4, 1, &Value);
    // std::cout << ret << "," << data.currentAxis[15] << " | " << std::endl;
    data.currentAxis[drivingWheel4] = static_cast<int16_t>(Value)/Dirve_FD144S_A_Unit;
    
    if (m_readFlag) { 
        if (m_QueueFirst.size() < 1024) {
            m_QueueFirst.push(data);
        } else {
            m_readFlag = false;
            m_QueueFirstHasData = true;
            std::cout << "m_QueueFirst够1024" << std::endl;
            m_QueueSecond.push(data);
        }
    } else {
        if (m_QueueSecond.size() < 1024) {
            m_QueueSecond.push(data);
        } else {
            m_readFlag = true;
            m_QueueSecondHasData = true;
            std::cout << "m_QueueSecond够1024" << std::endl;
            m_QueueFirst.push(data);
        }
    }
}

void MotorDAQ::runSave(void)
{
    while (m_nh->ok()) {
        if (m_fileName == "") {
            m_fileName = generateFileNameWithTimestamp();
        } else {
            if (getFileSize(m_fileName) > TARGET_FILE_SIZE) {
                m_fileName = generateFileNameWithTimestamp();
            }
        }
        saveDAQToFile(m_fileName);
        sleep(1);
    }
}

void exit_func(void) 
{
    smc_board_close(ConnectNo);
}

void MotorDAQ::connectInit(void)
{
    char* daddr = const_cast<char *>(Daddr.data()); 
    short rtn =1;
    static unsigned long errcode = 0;
    rtn = smc_board_init(ConnectNo, PortNum, daddr, 0);
    std::cout<<"rtn = "<<rtn<<std::endl;
    if (rtn != 0) {
        printf("smc_board_init rtn = %d\n",rtn);
    }
    
    atexit(exit_func);

}


string MotorDAQ::generateFileNameWithTimestamp() 
{
    auto now = chrono::system_clock::now();
    auto now_time_t = chrono::system_clock::to_time_t(now);
    auto now_tm = *localtime(&now_time_t);

    // 格式化时间戳为字符串
    stringstream ss;
    ss << put_time(&now_tm, "%Y-%m-%d_%H-%M-%S");
    string timestamp_str = ss.str();

    // 构造文件名，例如 "data_2024-08-07_15-30-00.txt"
    string filename = "currentData_" + timestamp_str + ".txt";
    return filename;
}

// 函数：获取文件大小（字节数）
long long MotorDAQ::getFileSize(const string& filename) {
    ifstream file(filename, ios::binary | ios::ate);
    if (!file.is_open()) {
        cerr << "Failed to open file: " << filename << endl;
        return -1; // 返回-1表示文件打开失败
    }

    // 获取当前位置，即文件尾部位置，即文件大小
    long long fileSize = file.tellg();

    file.close();

    return fileSize;
}