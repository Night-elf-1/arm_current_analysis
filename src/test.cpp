#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <limits>

using namespace std;

// 读取CSV文件并解析数据
void readCSV(const string& filename, vector<double>& weights, vector<vector<double>>& currentData) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "无法打开文件: " << filename << endl;
        return;
    }

    string line;
    // 读取第一行（标题或重量）
    getline(file, line);
    // 读取重量
    stringstream weightStream(line);
    string weightStr;
    while (getline(weightStream, weightStr, ',')) {
        try {
            weights.push_back(stod(weightStr));
        } catch (const invalid_argument& e) {
            cerr << "无效的重量数据: " << weightStr << endl;
            return;
        }
    }

    // 逐行读取电流数据
    while (getline(file, line)) {
        stringstream ss(line);
        string cell;
        vector<double> rowData;
        while (getline(ss, cell, ',')) {
            try {
                double value = stod(cell);
                // 如果当前行的列数多于已有的列，添加新的列
                if (rowData.size() < weights.size()) {
                    currentData.push_back({});
                }
                currentData[rowData.size()].push_back(value);
                rowData.push_back(value);
            } catch (const invalid_argument& e) {
                cerr << "无效的电流数据: " << cell << endl;
                return;
            }
        }
    }

    file.close();
}

int main() {
    string filename = "/home/hamster/mycode/Arm_current_analysis/data/predictive_table.csv";
    vector<double> weights;
    vector<vector<double>> currentData;

    readCSV(filename, weights, currentData);

    // 输出读取的数据
    cout << "重量: ";
    for (auto weight : weights) {
        cout << weight << " ";
    }
    cout << endl;

    // cout << "电流数据: " << endl;
    // for (size_t i = 0; i < currentData.size(); ++i) {
    //     for (size_t j = 0; j < currentData[i].size(); ++j) {
    //         cout << currentData[i][j] << " ";
    //     }
    //     cout << endl;
    // }

    return 0;
}