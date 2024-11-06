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

// 画图
void drawPicture(std::vector<double>& x_1, std::vector<double>& data_1, std::vector<double> data_2, std::vector<double>& x_2){
    plt::figure_size(800,600);
    plt::named_plot("Line 1 (1.35T)", x_1, data_1, "b-");  // 使用 named_plot 给线条命名
    plt::named_plot("Line 2 (1.05T)", x_2, data_2, "r-");
    plt::title("Current Mean Compare(1.35T and 1.05T)");
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
    plt::named_plot("arm1 (1.35T)", x[0], datas[0], "b-");                   // 使用 named_plot 给线条命名
    plt::named_plot("arm2 (1.35T)", x[1], datas[1], "r-");
    plt::named_plot("arm3 (1.35T)", x[2], datas[2], "g-");
    plt::named_plot("arm4 (1.35T)", x[3], datas[3], "c-");
    plt::named_plot("arm5 (1.35T)", x[4], datas[4], "m-");
    plt::named_plot("arm6 (1.35T)", x[5], datas[5], "y-");
    plt::named_plot("arm7 (1.35T)", x[6], datas[6], "k-");
    plt::named_plot("arm8 (1.35T)", x[7], datas[7], "w-");
    plt::title("Current Data(1.35T)");
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
    plt::title("Current trapezoid integral area distribution frequency(1.05T)");
    plt::xlabel("Trapz Area Values");
    plt::ylabel("Frequency");
    plt::show();
}

int main(int argc, char const *argv[])
{
    /* code */
    std::vector<std::vector<std::vector<double>>> CurrentDatas_1;
    std::vector<std::vector<std::vector<double>>> CurrentDatas_2;
    std::vector<double> trapzArea_1;
    std::vector<double> trapzArea_2;
    double totalMeanArea;
    std::vector<double> currentMean_1;
    std::vector<double> currentMean_2;

    // 文件名
    std::string filename_1 = "/home/hamster/mycode/Arm_current_analysis/data/armsdata_J75_11-06(1.35t).txt";
    std::string filename_2 = "/home/hamster/mycode/Arm_current_analysis/data/armsdata_J75_11-06(1.05t).txt";
    CurrentDatas_1 = extractCurrentData(filename_1);                                  // 提取电流数据
    CurrentDatas_2 = extractCurrentData(filename_2);

    // 提取面积
    trapzArea_1 = extractData_trapzArea(filename_1);                                // 提取1.35t
    trapzArea_2 = extractData_trapzArea(filename_2);                                // 提取1.05t
    auto count_1 = countOccurrences(trapzArea_1);                                   // 统计频率
    auto count_2 = countOccurrences(trapzArea_2);                                   // 统计频率
    for (int i = 0; i < trapzArea_1.size(); i++)
    {
        totalMeanArea += trapzArea_1[i];
    }
    totalMeanArea /= trapzArea_1.size();
    // std::cout << "总平均面积 = " << totalMeanArea << std::endl;

    // 提取电流均值
    currentMean_1 = extractData_currentMean(filename_1);
    currentMean_2 = extractData_currentMean(filename_2);

    // 获取x轴的信息 画图用
    std::vector<double> x_1(trapzArea_1.size());                                      // X轴数据
    for (size_t i = 0; i < trapzArea_1.size(); ++i) {
        x_1[i] = i + 1;
    }
    std::vector<double> x_2(trapzArea_2.size());                                      // X轴数据
    for (size_t i = 0; i < trapzArea_2.size(); ++i) {
        x_2[i] = i + 1;
    }

    // 画图
    // drawPicture(x_1, currentMean_1, currentMean_2, x_2);
    // drawHistogram(count_2);                                                         // 频率直方图
    // drawCurrentPicture(CurrentDatas_1[6]);                                             // 绘制电流8个夹臂数据
    

    return 0;
}
