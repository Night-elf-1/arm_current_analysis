#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <regex>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <map>
#include <Eigen/Dense>
#include "eigen3/Eigen/Core"
#include "matplotlibcpp.h"

using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;

class WeightPredictor {
private:
    VectorXd weights_;              // 模型权重
    deque<double> history_window_;  // 历史数据窗口
    const size_t window_size_ = 5;  // 窗口大小
    const double smooth_factor_ = 0.3; // 平滑因子

    // 计算加权平均
    double calculateWeightedMean() const {
        if (history_window_.empty()) return 0.0;
        
        double sum = 0.0;
        double weight_sum = 0.0;
        double weight = 1.0;
        
        for (auto it = history_window_.rbegin(); it != history_window_.rend(); ++it) {
            sum += (*it) * weight;
            weight_sum += weight;
            weight *= (1.0 - smooth_factor_);
        }
        
        return sum / weight_sum;
    }

    // 计算特征
    VectorXd extractFeatures() const {
        VectorXd features(2);  // [加权平均值, 当前值]
        
        features(0) = calculateWeightedMean();
        features(1) = 1.0;  // 偏置项
        
        return features;
    }

public:
    // 初始化预测器
    WeightPredictor() {}
    
    // 设置模型权重
    void setWeights(const VectorXd& weights) {
        weights_ = weights;
    }
    
    // 添加新的观测值
    void addMeasurement(double current) {
        history_window_.push_back(current);
        if (history_window_.size() > window_size_) {
            history_window_.pop_front();
        }
    }
    
    // 预测重量
    double predictWeight(double current_value) {
        // 添加新的测量值到历史窗口
        addMeasurement(current_value);
        
        // 提取特征
        VectorXd features = extractFeatures();
        
        // 预测重量
        return weights_.dot(features);
    }
    
    // 清除历史数据
    void reset() {
        history_window_.clear();
    }
};

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

// 改进的训练数据准备函数
void prepareTrainingData(const vector<double>& currentMean_1, 
                        const vector<double>& currentMean_2,
                        vector<double>& training_currents,
                        vector<double>& training_weights) {
    // 计算1.05吨数据的均值和标准差
    double mean_1 = accumulate(currentMean_1.begin(), currentMean_1.end(), 0.0) / currentMean_1.size();
    double stddev_1 = 0.0;
    for (double val : currentMean_1) {
        stddev_1 += (val - mean_1) * (val - mean_1);
    }
    stddev_1 = sqrt(stddev_1 / currentMean_1.size());

    // 计算1.2吨数据的均值和标准差
    double mean_2 = accumulate(currentMean_2.begin(), currentMean_2.end(), 0.0) / currentMean_2.size();
    double stddev_2 = 0.0;
    for (double val : currentMean_2) {
        stddev_2 += (val - mean_2) * (val - mean_2);
    }
    stddev_2 = sqrt(stddev_2 / currentMean_2.size());

    // 只保留在均值±2倍标准差范围内的数据
    for (double current : currentMean_1) {
        if (abs(current - mean_1) <= 2 * stddev_1) {
            training_currents.push_back(current);
            training_weights.push_back(1.05);
        }
    }
    
    for (double current : currentMean_2) {
        if (abs(current - mean_2) <= 2 * stddev_2) {
            training_currents.push_back(current);
            training_weights.push_back(1.2);
        }
    }
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

// 主程序使用示例
int main() {
    // ... (之前的数据读取代码保持不变)
    // 文件名
    std::string filename_1 = "/home/hamster/mycode/Arm_current_analysis/data/armsdata_J75_11-07(1.05t).txt";    // 1.05吨的电流数据
    std::string filename_2 = "/home/hamster/mycode/Arm_current_analysis/data/armsdata_J75_11-07(1.2t).txt";     // 1.2吨的电流数据

    std::vector<double> currentMean_1;                                                  // 电流均值容器
    std::vector<double> currentMean_2;
    // 提取电流均值
    currentMean_1 = extractData_currentMean(filename_1);
    currentMean_2 = extractData_currentMean(filename_2);

    // 准备训练数据
    vector<double> training_currents;
    vector<double> training_weights;
    prepareTrainingData(currentMean_1, currentMean_2, training_currents, training_weights);

    // 训练模型
    VectorXd w = trainModel(training_weights, training_currents);

    // 创建预测器实例
    WeightPredictor predictor;
    predictor.setWeights(w);

    // 实时预测循环
    char continue_predict = 'y';
    while (continue_predict == 'y' || continue_predict == 'Y') {
        double realTimeCurrent;
        cout << "输入实时电流值: ";
        cin >> realTimeCurrent;

        // 预测重量
        double estimatedWeight = predictor.predictWeight(realTimeCurrent);
        cout << "预测重量: " << estimatedWeight << " 吨" << endl;

        cout << "是否继续预测？(y/n): ";
        cin >> continue_predict;
    }

    return 0;
}