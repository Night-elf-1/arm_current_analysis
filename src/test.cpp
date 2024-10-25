#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

struct ClampingArmData {
    vector<double> currentData;
    double weight;
};

void readCSV(const string& filename, vector<ClampingArmData>& data) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "无法打开文件: " << filename << endl;
        return;
    }

    string line;
    // 读取第一行（重量）
    getline(file, line);
    stringstream weightStream(line);
    string weightStr;
    while (getline(weightStream, weightStr, ',')) {
        data.emplace_back();
        data.back().weight = stod(weightStr);
    }

    // 读取电流数据
    size_t maxColumns = data.size();
    while (getline(file, line)) {
        stringstream ss(line);
        string cell;
        size_t col = 0;
        while (getline(ss, cell, ',')) {
            if (col < maxColumns) {
                try {
                    double value = stod(cell);
                    data[col].currentData.push_back(value);
                } catch (const invalid_argument& e) {
                    cerr << "无效的电流数据: " << cell << endl;
                    return;
                }
            }
            ++col;
        }
    }

    // 去除每列尾部的0
    for (auto& armData : data) {
        while (!armData.currentData.empty() && armData.currentData.back() == 0) {
            armData.currentData.pop_back();
        }
    }

    file.close();
}

int main() {
    string filename = "/home/hamster/mycode/Arm_current_analysis/data/predictive_table.csv"; // CSV文件路径
    vector<ClampingArmData> clampingArmData;
    readCSV(filename, clampingArmData);

    // 输出读取的数据
    for (const auto& arm : clampingArmData) {
        cout << "重量: " << arm.weight << endl;
        cout << "电流数据: ";
        for (const auto& current : arm.currentData) {
            cout << current << " ";
        }
        cout << endl;
    }

    return 0;
}