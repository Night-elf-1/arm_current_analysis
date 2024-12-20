#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <regex>
#include <map>
#include <Eigen/Dense>
#include "eigen3/Eigen/Core"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

using namespace std;
using namespace Eigen;

// 分割字符串
std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// 提取电流数据
std::vector<std::vector<std::vector<double>>> extractCurrentData(const std::string& filename){
    std::ifstream file(filename);

    // 检查文件是否成功打开
    if (!file.is_open()) {
        std::cerr << "无法打开文件 " << filename << std::endl;
    }

    // 容器结构 datas
    std::vector<std::vector<std::vector<double>>> datas;
    std::vector<std::vector<double>> currentGroup;
    std::string line;

    // 标志记录当前是否需要保存数据
    bool saveNextLines = false;
    int lineCount = 0;

    // 逐行读取文件
    while (std::getline(file, line)) {
        // 检查是否是分隔符
        if (line.find("====================================") != std::string::npos) {
            saveNextLines = true;  // 开始保存接下来的 8 行数据
            currentGroup.clear();
            lineCount = 0;
            continue;
        }

        if (saveNextLines) {
            // 处理数据行
            std::vector<double> row;
            std::istringstream ss(line);
            std::string value;

            while (std::getline(ss, value, ',')) {
                try {
                    row.push_back(std::stod(value));
                } catch (const std::invalid_argument&) {
                    std::cerr << "无效的数字: " << value << std::endl;
                }
            }

            currentGroup.push_back(row);
            lineCount++;

            // 保存满 8 行后，结束本组数据保存
            if (lineCount == 8) {
                datas.push_back(currentGroup);
                saveNextLines = false;
            }
        }
    }

    file.close();
    return datas;
}

// 提取梯形面积数据
std::vector<double> extractData_trapzArea(const std::string& filename) {
    std::vector<double> data;
    std::ifstream file(filename);       // 读取的文件的路径
    std::string line;

    while (std::getline(file, line)) {      // 使用std::getline函数从文件中逐行读取数据，直到到达文件末尾
        // 检查是否包含“总电流均值”
        size_t found = line.find("总面积均值");     // 这行代码在当前行中查找子字符串"总面积均值"。如果找到了这个子字符串（found不等于std::string::npos），则执行下面的代码块。
        if (found != std::string::npos) {
            // 分割字符串以获取数值
            auto tokens = split(line, '=');
            if (tokens.size() > 1) {
                // 尝试将字符串转换为double
                try {
                    double value = std::stod(tokens[1]);
                    data.push_back(value);
                } catch (const std::invalid_argument& e) {
                    // 如果转换失败，打印错误信息
                    std::cerr << "Invalid argument: " << e.what() << '\n';
                } catch (const std::out_of_range& e) {
                    // 如果数值超出范围，打印错误信息
                    std::cerr << "Out of range: " << e.what() << '\n';
                }
            }
        }
    }

    return data;
}

// 提取电流均值数据
std::vector<double> extractData_currentMean(const std::string& filename) {
    std::vector<double> data;
    std::ifstream file(filename);       // 读取的文件的路径
    std::string line;

    while (std::getline(file, line)) {      // 使用std::getline函数从文件中逐行读取数据，直到到达文件末尾
        // 检查是否包含“总电流均值”
        size_t found = line.find("总电流均值");     // 这行代码在当前行中查找子字符串"总面积均值"。如果找到了这个子字符串（found不等于std::string::npos），则执行下面的代码块。
        if (found != std::string::npos) {
            // 分割字符串以获取数值
            auto tokens = split(line, '=');
            if (tokens.size() > 1) {
                // 尝试将字符串转换为double
                try {
                    double value = std::stod(tokens[1]);
                    data.push_back(value);
                } catch (const std::invalid_argument& e) {
                    // 如果转换失败，打印错误信息
                    std::cerr << "Invalid argument: " << e.what() << '\n';
                } catch (const std::out_of_range& e) {
                    // 如果数值超出范围，打印错误信息
                    std::cerr << "Out of range: " << e.what() << '\n';
                }
            }
        }
    }

    return data;
}

// 训练模型
VectorXd trainModel(const vector<double>& weights, const vector<double>& areas) {
    int n = weights.size();
    MatrixXd X(n, 2);
    VectorXd y(n);

    for (int i = 0; i < n; ++i) {
        X(i, 0) = areas[i];
        X(i, 1) = 1.0;  // 截距项
        y(i) = weights[i];
    }

    // 使用正规方程解线性回归
    VectorXd w = (X.transpose() * X).ldlt().solve(X.transpose() * y);
    return w;
}

// 准备训练数据
void prepareTrainingData(const vector<double>& currentMean_1, 
                        const vector<double>& currentMean_2,
                        vector<double>& training_currents,
                        vector<double>& training_weights) {
    // 将电流均值数据和对应的重量组织成训练数据
    for (const double& current : currentMean_1) {
        training_currents.push_back(current);
        training_weights.push_back(1.05);  // 1.05吨的数据
    }
    
    for (const double& current : currentMean_2) {
        training_currents.push_back(current);
        training_weights.push_back(1.2);   // 1.2吨的数据
    }
    // 后续根据数据量还可以增加for循环
}

void saveWeights(const VectorXd& w, const string& filename) {
    ofstream file(filename);
    if (!file.is_open()) {
        cerr << "无法打开文件: " << filename << endl;
        return;
    }

    for (int i = 0; i < w.size(); ++i) {
        file << w[i] << endl;
    }

    file.close();
}

// 画图
void drawPicture(std::vector<double>& x_1, std::vector<double>& data_1, std::vector<double> data_2, std::vector<double>& x_2, 
                    std::vector<double> data_3, std::vector<double>& x_3){
    plt::figure_size(1200,1000);
    // plt::ylim(10, 14);
    plt::named_plot("Line 1 (1.05T)", x_1, data_1, "b-");  // 使用 named_plot 给线条命名
    plt::named_plot("Line 2 (1.2T)", x_2, data_2, "r-");
    plt::named_plot("Line 3 (2.2T)", x_3, data_3, "g-");
    plt::title("Current Mean Compare(1.2T and 1.05T and 2.2T)");
    plt::xlabel("Time");
    plt::ylabel("Current Mean");
    plt::grid(true);
    plt::legend();
    plt::show();
}

void drawCurrentPicture(std::vector<std::vector<double>>& datas){
    std::vector<std::vector<double>> x(datas.size());
    // 初始化每个子向量的大小和填充值
    for (size_t i = 0; i < datas.size(); ++i) {
        x[i].resize(datas[i].size());  // 设置子向量的大小
        for (size_t j = 0; j < datas[i].size(); ++j) {
            x[i][j] = j + 1;  // 填充值，从 1 开始递增
        }
    }
    
    plt::figure_size(1200,1000);
    // plt::ylim(5, 18);
    plt::named_plot("arm1 (1.2T)", x[0], datas[0], "b-");                   // 使用 named_plot 给线条命名
    plt::named_plot("arm2 (1.2T)", x[1], datas[1], "r-");
    plt::named_plot("arm3 (1.2T)", x[2], datas[2], "g-");
    plt::named_plot("arm4 (1.2T)", x[3], datas[3], "c-");
    plt::named_plot("arm5 (1.2T)", x[4], datas[4], "m-");
    plt::named_plot("arm6 (1.2T)", x[5], datas[5], "y-");
    plt::named_plot("arm7 (1.2T)", x[6], datas[6], "k-");
    plt::named_plot("arm8 (1.2T)", x[7], datas[7], "w-");
    plt::title("Current Data(1.2T)");
    plt::xlabel("Time");
    plt::ylabel("Current data");
    plt::grid(true);
    plt::legend();
    plt::show();
}

// 四舍五入并取整
int roundAndTruncate(double value) {
    return static_cast<int>(std::round(value));
}
// 统计每个整数出现的次数
std::map<int, int> countOccurrences(const std::vector<double>& data) {
    std::map<int, int> counts;
    for (double value : data) {
        int intValue = roundAndTruncate(value);
        counts[intValue]++;
    }
    return counts;
}

// 绘制直方图
void drawHistogram(const std::map<int, int>& counts) {
    std::vector<int> values;
    std::vector<int> frequencies;

    for (const auto& pair : counts) {
        values.push_back(pair.first);
        frequencies.push_back(pair.second);
    }

    plt::figure_size(800, 600);
    plt::bar(values, frequencies);
    plt::title("Current Mean distribution frequency(1.2T)");
    plt::xlabel("Current Mean Values");
    plt::ylabel("Frequency");
    plt::show();
}

// 滤波函数
std::vector<std::vector<std::vector<double>>> lvbo(std::vector<std::vector<std::vector<double>>>& datas) {
    const int MEDIAN_WINDOW_SIZE = 5;     // 中值滤波窗口大小
    const int MEAN_WINDOW_SIZE = 3;       // 均值滤波窗口大小
    const double OUTLIER_THRESHOLD = 3.0;  // 异常值判断阈值（标准差的倍数）
    
    std::vector<std::vector<std::vector<double>>> filteredData;
    filteredData.resize(datas.size());  // 第一层：组数据
    
    // 处理每组数据
    for (size_t groupIndex = 0; groupIndex < datas.size(); ++groupIndex) {
        filteredData[groupIndex].resize(datas[groupIndex].size());  // 第二层：8个夹臂
        
        // 处理每个夹臂的数据
        for (size_t armIndex = 0; armIndex < datas[groupIndex].size(); ++armIndex) {
            const auto& currentArmData = datas[groupIndex][armIndex];
            std::vector<double>& filteredArmData = filteredData[groupIndex][armIndex];
            filteredArmData.reserve(currentArmData.size());
            
            // 如果数据点太少，直接复制原始数据
            if (currentArmData.size() < MEDIAN_WINDOW_SIZE) {
                filteredArmData = currentArmData;
                continue;
            }
            
            // 第一步：中值滤波处理极值
            std::deque<double> medianWindow;
            std::vector<double> medianFiltered;
            medianFiltered.reserve(currentArmData.size());
            
            // 初始填充窗口
            for (int i = 0; i < MEDIAN_WINDOW_SIZE && i < currentArmData.size(); ++i) {
                medianWindow.push_back(currentArmData[i]);
            }
            
            // 对每个数据点进行中值滤波
            for (size_t i = 0; i < currentArmData.size(); ++i) {
                // 创建临时窗口进行排序
                std::vector<double> tempWindow(medianWindow.begin(), medianWindow.end());
                std::sort(tempWindow.begin(), tempWindow.end());
                
                // 获取中值
                double median = tempWindow[tempWindow.size() / 2];
                medianFiltered.push_back(median);
                
                // 滑动窗口
                if (i + MEDIAN_WINDOW_SIZE < currentArmData.size()) {
                    medianWindow.pop_front();
                    medianWindow.push_back(currentArmData[i + MEDIAN_WINDOW_SIZE]);
                }
            }
            
            // 第二步：均值滤波处理一般波动
            std::deque<double> meanWindow;
            
            // 初始填充均值滤波窗口
            for (int i = 0; i < MEAN_WINDOW_SIZE && i < medianFiltered.size(); ++i) {
                meanWindow.push_back(medianFiltered[i]);
            }
            
            // 对每个数据点进行均值滤波
            for (size_t i = 0; i < medianFiltered.size(); ++i) {
                // 计算当前窗口的均值
                double sum = std::accumulate(meanWindow.begin(), meanWindow.end(), 0.0);
                double mean = sum / meanWindow.size();
                
                // 计算标准差
                double sqSum = std::accumulate(meanWindow.begin(), meanWindow.end(), 0.0,
                    [mean](double acc, double val) {
                        double diff = val - mean;
                        return acc + diff * diff;
                    });
                double stdDev = std::sqrt(sqSum / meanWindow.size());
                
                // 如果当前值偏离均值太多，则使用均值替代
                double currentValue = medianFiltered[i];
                if (std::abs(currentValue - mean) > OUTLIER_THRESHOLD * stdDev) {
                    filteredArmData.push_back(mean);
                } else {
                    filteredArmData.push_back(currentValue);
                }
                
                // 滑动窗口
                if (i + MEAN_WINDOW_SIZE < medianFiltered.size()) {
                    meanWindow.pop_front();
                    meanWindow.push_back(medianFiltered[i + MEAN_WINDOW_SIZE]);
                }
            }
        }
    }
    
    return filteredData;
}

// 滤波2 在agv中跑
std::vector<std::vector<double>> lvbo2(std::vector<std::vector<double>>& datas) {
    const int MEDIAN_WINDOW_SIZE = 5;     // 中值滤波窗口大小
    const int MEAN_WINDOW_SIZE = 3;       // 均值滤波窗口大小
    const double OUTLIER_THRESHOLD = 3.0;  // 异常值判断阈值（标准差的倍数）
    
    std::vector<std::vector<double>> filteredData;
    filteredData.resize(datas.size());
    
    // 处理每个夹臂的数据
    for (size_t i = 0; i < datas.size(); ++i) {
        const auto& currentArm = datas[i];
        std::vector<double>& filteredArm = filteredData[i];
        filteredArm.reserve(currentArm.size());
        
        // 如果数据点太少，直接返回原始数据
        if (currentArm.size() < MEDIAN_WINDOW_SIZE) {
            filteredArm = currentArm;
            continue;
        }
        
        // 第一步：中值滤波处理极值
        std::deque<double> medianWindow;
        std::vector<double> medianFiltered;
        medianFiltered.reserve(currentArm.size());
        
        // 初始填充窗口
        for (int j = 0; j < MEDIAN_WINDOW_SIZE && j < currentArm.size(); ++j) {
            medianWindow.push_back(currentArm[j]);
        }
        
        // 对每个数据点进行中值滤波
        for (size_t j = 0; j < currentArm.size(); ++j) {
            // 创建临时窗口进行排序
            std::vector<double> tempWindow(medianWindow.begin(), medianWindow.end());
            std::sort(tempWindow.begin(), tempWindow.end());
            
            // 获取中值
            double median = tempWindow[tempWindow.size() / 2];
            medianFiltered.push_back(median);
            
            // 滑动窗口
            if (j + MEDIAN_WINDOW_SIZE < currentArm.size()) {
                medianWindow.pop_front();
                medianWindow.push_back(currentArm[j + MEDIAN_WINDOW_SIZE]);
            }
        }
        
        // 第二步：均值滤波处理一般波动
        std::deque<double> meanWindow;
        
        // 初始填充均值滤波窗口
        for (int j = 0; j < MEAN_WINDOW_SIZE && j < medianFiltered.size(); ++j) {
            meanWindow.push_back(medianFiltered[j]);
        }
        
        // 对每个数据点进行均值滤波
        for (size_t j = 0; j < medianFiltered.size(); ++j) {
            // 计算当前窗口的均值
            double sum = std::accumulate(meanWindow.begin(), meanWindow.end(), 0.0);
            double mean = sum / meanWindow.size();
            
            // 计算标准差
            double sqSum = std::accumulate(meanWindow.begin(), meanWindow.end(), 0.0,
                [mean](double acc, double val) {
                    double diff = val - mean;
                    return acc + diff * diff;
                });
            double stdDev = std::sqrt(sqSum / meanWindow.size());
            
            // 如果当前值偏离均值太多，则使用均值替代
            double currentValue = medianFiltered[j];
            if (std::abs(currentValue - mean) > OUTLIER_THRESHOLD * stdDev) {
                filteredArm.push_back(mean);
            } else {
                filteredArm.push_back(currentValue);
            }
            
            // 滑动窗口
            if (j + MEAN_WINDOW_SIZE < medianFiltered.size()) {
                meanWindow.pop_front();
                meanWindow.push_back(medianFiltered[j + MEAN_WINDOW_SIZE]);
            }
        }
    }
    
    return filteredData;
}

// 提前后10%的数据
std::vector<std::vector<std::vector<double>>> extractLastTenPercent(
    const std::vector<std::vector<std::vector<double>>>& CurrentDatas_1) {
    
    std::vector<std::vector<std::vector<double>>> result;
    
    // 遍历第一层
    for (const auto& group : CurrentDatas_1) {
        std::vector<std::vector<double>> groupResult;
        
        // 遍历第二层（8行夹臂数据）
        for (const auto& arm : group) {
            std::vector<double> armResult;
            
            // 计算需要提取的数据长度（后10%）
            size_t dataSize = arm.size();
            size_t extractSize = std::ceil(dataSize * 0.1); // 向上取整
            size_t startIndex = dataSize - extractSize;
            
            // 提取后10%的数据
            armResult.insert(armResult.end(), 
                           arm.begin() + startIndex,
                           arm.end());
            
            groupResult.push_back(armResult);
        }
        
        result.push_back(groupResult);
    }
    
    return result;
}


// 计算电流均值
std::vector<double> calculateCurrentMean(std::vector<std::vector<std::vector<double>>>& tempCurrentDatas){
    std::vector<double> groupMeans;
    groupMeans.reserve(tempCurrentDatas.size()); // 为每组数据预分配空间
    
    // 遍历每组数据（第一层）
    for (const auto& group : tempCurrentDatas) {
        // 存储8个夹臂各自的均值
        std::vector<double> armMeans;
        armMeans.reserve(8);  // 预分配8个夹臂的空间
        
        // 计算每个夹臂的均值
        for (const auto& arm : group) {
            // 计算单个夹臂的均值
            double armSum = std::accumulate(arm.begin(), arm.end(), 0.0);
            double armMean = arm.empty() ? 0.0 : armSum / arm.size();
            armMeans.push_back(armMean);
        }
        
        // 计算8个夹臂均值的平均值
        double groupMean = std::accumulate(armMeans.begin(), armMeans.end(), 0.0);
        groupMean = armMeans.empty() ? 0.0 : groupMean / armMeans.size();
        
        groupMeans.push_back(groupMean);
    }
    
    return groupMeans;
}

// 计算电流梯形积分面积


int main(int argc, char const *argv[])
{
    /* code */
    std::vector<std::vector<std::vector<double>>> CurrentDatas_1;                       // 存放电流数据三维容器
    std::vector<std::vector<std::vector<double>>> CurrentDatas_2;
    std::vector<std::vector<std::vector<double>>> tempCurrentDatas_1;                   // 暂存电流数据
    std::vector<std::vector<std::vector<double>>> tempCurrentDatas_2;
    std::vector<std::vector<std::vector<double>>> filterTempCurrentDatas_1;             // 滤波后电流数据
    std::vector<std::vector<std::vector<double>>> filterTempCurrentDatas_2;
    std::vector<double> trapzArea_1;                                                    // 梯形面积容器
    std::vector<double> trapzArea_2;
    double totalMeanArea;
    std::vector<double> currentMean_1;                                                  // 电流均值容器
    std::vector<double> currentMean_2;
    std::vector<double> currentMean_3;

    // 文件名
    std::string filename_1 = "/home/hamster/mycode/Arm_current_analysis/data/armsdata_J75_11-07(1.05t).txt";    // 1.05吨的电流数据
    std::string filename_2 = "/home/hamster/mycode/Arm_current_analysis/data/armsdata_J75_11-07(1.2t).txt";     // 1.2吨的电流数据
    std::string filename_3 = "/home/hamster/mycode/Arm_current_analysis/data/armsdata_J75_11-07(2.2t).txt";     // 1.2吨的电流数据
    // CurrentDatas_1 = extractCurrentData(filename_1);                                // 提取电流数据
    // CurrentDatas_2 = extractCurrentData(filename_2);                                // 提取电流数据

    // 滤波处理
    // filterTempCurrentDatas_1 = lvbo(CurrentDatas_1);
    // filterTempCurrentDatas_2 = lvbo(CurrentDatas_2);

    // std::vector<double> tempcurrentmean_1;
    // std::vector<double> tempcurrentmean_2;
    // tempcurrentmean_1 = calculateCurrentMean(filterTempCurrentDatas_1);
    // tempcurrentmean_2 = calculateCurrentMean(filterTempCurrentDatas_2);                           // 计算电流均值
    
    // 提取面积
    // trapzArea_1 = extractData_trapzArea(filename_1);                                // 提取1.35t
    // trapzArea_2 = extractData_trapzArea(filename_2);                                // 提取1.05t

    // 提取电流均值
    currentMean_1 = extractData_currentMean(filename_1);
    currentMean_2 = extractData_currentMean(filename_2);
    currentMean_3 = extractData_currentMean(filename_3);

    // ==========================================================================================================
    // 准备训练数据
    // vector<double> training_currents;
    // vector<double> training_weights;
    // prepareTrainingData(currentMean_1, currentMean_2, training_currents, training_weights);             // 输入数据

    // // 训练模型
    // VectorXd w = trainModel(training_weights, training_currents);

    // // 保存权重文件
    // saveWeights(w, "/home/hamster/mycode/Arm_current_analysis/data/model_weights.txt");

    // cout << "模型训练完成！" << endl;
    // cout << "斜率 (w0): " << w(0) << endl;
    // cout << "截距 (w1): " << w(1) << endl;

    // double realTimeCurrent;                                     // 实时电流均值

    // char continue_predict = 'y';
    // while (continue_predict == 'y' || continue_predict == 'Y')
    // {
    //     cout << "输入实时电流均值: ";
    //     cin >> realTimeCurrent;

    //     // 预测重量
    //     double estimatedWeight = w(0) * realTimeCurrent + w(1);
    //     cout << "预测重量: " << estimatedWeight << " 吨" << endl;

    //     cout << "是否继续预测？(y/n): ";
    //     cin >> continue_predict;
    // }
    // ==========================================================================================================

    // 获取x轴的信息 画图用
    std::vector<double> x_1(currentMean_1.size());                                      // X轴数据
    for (size_t i = 0; i < currentMean_1.size(); ++i) {
        x_1[i] = i + 1;
    }
    std::vector<double> x_2(currentMean_2.size());                                      // X轴数据
    for (size_t i = 0; i < currentMean_2.size(); ++i) {
        x_2[i] = i + 1;
    }
    std::vector<double> x_3(currentMean_3.size());                                      // X轴数据
    for (size_t i = 0; i < currentMean_3.size(); ++i) {
        x_3[i] = i + 1;
    }

    // 画图
    drawPicture(x_1, currentMean_1, currentMean_2, x_2, currentMean_3, x_3);                                    // 画折线图
    // drawHistogram(count_2);                                                         // 频率直方图
    // drawCurrentPicture(filterData);                                             // 绘制电流8个夹臂数据
    // drawCurrentPicture(CurrentDatas_1[10]);

    return 0;
}
