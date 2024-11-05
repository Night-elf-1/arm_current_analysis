#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <string>
#include <regex>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// 提取预估车重
std::vector<double> extractEstimatedWeights(const std::string& filename) {
    std::vector<double> weights;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        size_t found = line.find("预估重量");
        if (found != std::string::npos)
        {
            std::istringstream iss(line);
            std::string temp;
            double weight;
            
            // 读取“预估重量 = ”部分并丢弃
            iss >> temp >> temp;
            
            // 读取数字部分
            if (iss >> weight) {
                weights.push_back(weight);
            }
        }
    }
    return weights;
}

std::vector<double> extractData(const std::string& filename) {
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

// 使用matplotlib-cpp绘制直方图
void plotHistogram(const std::map<int, int>& counts) {
    std::vector<int> values;
    std::vector<int> frequencies;

    for (const auto& pair : counts) {
        values.push_back(pair.first);
        frequencies.push_back(pair.second);
    }

    plt::figure_size(1000, 600);
    plt::bar(values, frequencies);
    plt::title("00J75-1.6T current area data");
    plt::xlabel("Current Values");
    plt::ylabel("Frequency");
    plt::show();
}

// 使用matplotlib-cpp绘制数据
void plotWeights(const std::vector<double>& weights) {
    std::vector<double> x(weights.size()); // X轴数据
    std::vector<double> y(weights);      // Y轴数据

    for (size_t i = 0; i < weights.size(); ++i) {
        x[i] = i + 1; // 简单的x轴标签，从1开始
    }

    plt::figure_size(800, 600);
    plt::plot(x, y, "b-"); // 绘制蓝色的线图，带圆圈标记
    plt::title("Estimated Weights");
    plt::xlabel("Sample Index");
    plt::ylabel("Weight (ton)");
    plt::grid(true);
    plt::show();
}

int main() {
    // std::string filename = "/home/hamster/mycode/Arm_current_analysis/data/armsdata_J75_11-01.txt";
    std::string filename = "/home/hamster/mycode/Arm_current_analysis/data/armsdata_J75_11-05.txt";
    auto data = extractData(filename);
    // plotWeights(data);
    // auto counts = countOccurrences(data);
    // plotHistogram(counts);
    
    auto weights = extractEstimatedWeights(filename);
    // plotWeights(weights);

    // std::vector<double> x_weights(weights.size());
    // std::vector<double> x_data(data.size());
    // for (size_t i = 0; i < weights.size(); ++i) {
    //     x_weights[i] = i + 1; // 简单的x轴标签，从1开始
    // }
    // for (size_t i = 0; i < data.size(); ++i) {
    //     x_data[i] = i + 1; // 简单的x轴标签，从1开始
    // }
    // plt::figure_size(800,600);
    // plt::plot(x_weights, weights, "b-");
    // plt::plot(x_data, data, "r-");
    // plt::title("Estimated Weights");
    // plt::xlabel("Sample Index");
    // plt::ylabel("Weight (ton)");
    // plt::grid(true);
    // plt::show();

    return 0;
}