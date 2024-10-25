/**
 * @file        MotorDAQ.h
 * @brief       电机数据采集
 * @details     
 * @author      黄文开 wenkai.h@qq.com
 * @date        2024-08-07  11:28:46
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
#ifndef _MotorDAQ_H_
#define _MotorDAQ_H_
// #include "ros/ros.h"
#include "LTSMC.h"
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
// #include "std_msgs/String.h"

#define TARGET_FILE_SIZE        (10 * 1024 * 1024)
void exit_func(void);

enum MotionAxis
{
    /*  夹臂编号
        leftarm1    前车   rightarm1
        leftarm2          rightarm2
        leftarm3          rightarm3
        leftarm4    后车   rightarm4
    */

    /*  转向轮编号
        前车    steeringWheel1
               steeringWheel2
               steeringWheel3
        后车    steeringWheel4
    */

    /*  驱动轮编号
        前车    drivingWheel1
               drivingWheel2
               drivingWheel3
        后车    drivingWheel4
    */
    steeringWheel1 = 0,
    drivingWheel1,
    rightarm2,
    rightarm1,
    leftarm2,
    leftarm1,
    steeringWheel2,
    drivingWheel2,
    steeringWheel3,
    drivingWheel3,
    rightarm4,
    rightarm3,
    leftarm4,
    leftarm3,
    steeringWheel4,
    drivingWheel4,
};
//! 时间  行为  电流值 

// 定义数据帧结构
struct DataFrame {
    std::string timestamp;
    std::string action;
    double currentAxis[16];
};


// SMC_API short __stdcall nmcs_write_rxpdo_extra(WORD ConnectNo,WORD PortNum,WORD address,WORD DataLen,DWORD Value);
// SMC_API short __stdcall nmcs_read_rxpdo_extra(WORD ConnectNo,WORD PortNum,WORD address,WORD DataLen,DWORD* Value);
// SMC_API short __stdcall nmcs_read_txpdo_extra(WORD ConnectNo,WORD PortNum,WORD address,WORD DataLen,DWORD* Value);


class MotorDAQ
{
public:
    MotorDAQ();
    ~MotorDAQ();
    void getCurrentAll(void);
    void saveDAQToFile(const std::string& fileName);
    void run(void);
    void runSave(void);
private:
    // void behaviorActionCallback(const std_msgs::String& msg);
    void connectInit(void);
    void disconnect(void);
    // void exit_func(void);
    std::string generateFileNameWithTimestamp(void); 
    long long getFileSize(const std::string& filename);
private:
    // ros::NodeHandle *m_nh;
    // ros::Subscriber m_subBehavior;
    std::string m_action;
    std::queue<DataFrame> m_QueueFirst;
    std::queue<DataFrame> m_QueueSecond;
    std::mutex m_mutex;
    std::condition_variable cv;
    std::thread *m_thread;  /*!< 数据保存线程 */
    bool m_readFlag;
    bool m_QueueFirstHasData;
    bool m_QueueSecondHasData;
    std::string m_fileName;
};


#endif //_MotorDAQ_H_