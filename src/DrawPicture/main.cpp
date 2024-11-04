#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <string>
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

std::vector<double> extractData(const std::string& filename) {
    std::vector<double> data;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        // 检查是否包含“总电流均值”
        size_t found = line.find("总电流均值");
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
    plt::title("Histogram of Rounded Current Values");
    plt::xlabel("Current Values");
    plt::ylabel("Frequency");
    plt::show();
}

int main() {
    // std::string filename = "/home/hamster/mycode/Arm_current_analysis/data/armsdata_J75_11-01.txt";
    std::string filename = "/home/hamster/mycode/Arm_current_analysis/data/armsdata.txt";
    auto data = extractData(filename);
    auto counts = countOccurrences(data);
    plotHistogram(counts);
    
    // auto counts = countOccurrences(data);
    // plotHistogram(counts);

    return 0;
}