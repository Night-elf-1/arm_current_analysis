#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <numeric>  // 用于std::accumulate
#include <Eigen/Dense>  // 用于矩阵操作

using namespace std;
using namespace Eigen;

// 计算曲线下面积（使用梯形规则）
double calculateArea(const vector<double>& current_data) {
    double area = 0.0;
    for (size_t i = 1; i < current_data.size(); ++i) {
        area += 0.5 * (current_data[i] + current_data[i - 1]);
    }
    return area;
}

// 读取CSV并解析数据
void readCSV(const string& filename, vector<vector<double>>& currentData, vector<double>& weights) {
    ifstream file(filename);
    string line;
    
    bool isFirstRow = true;  // 用于处理第一行的标记
    
    // 逐行读取文件
    while (getline(file, line)) {
        stringstream ss(line);
        string cell;
        vector<double> rowData;
        
        while (getline(ss, cell, ',')) {
            rowData.push_back(stod(cell));  // 将字符串转换为double类型
        }
        
        if (isFirstRow) {
            // 第一行是车辆重量
            weights = rowData;
            isFirstRow = false;
        } else {
            // 其余行是电流数据
            currentData.push_back(rowData);
        }
    }
}

// 主要处理函数，处理数据并进行线性回归
void processCSVData(const string& filename) {
    vector<vector<double>> currentData;  // 存储夹臂电流数据
    vector<double> weights;  // 存储车辆重量
    
    // 读取CSV数据
    readCSV(filename, currentData, weights);
    
    size_t numRows = currentData.size();  // 电流数据的行数
    size_t numColumns = currentData[0].size();  // CSV文件的列数
    size_t numArms = 8;  // 每8列为一组夹臂数据
    size_t numGroups = numColumns / numArms;  // 计算有多少组数据

    // 用于存储每组电流面积的平均值
    vector<double> avgAreas;

    for (size_t group = 0; group < numGroups; ++group) {
        double totalArea = 0.0;
        
        // 计算每一组中夹臂的电流面积
        for (size_t arm = 0; arm < numArms; ++arm) {
            vector<double> armData;
            
            // 提取当前夹臂的电流数据
            for (size_t row = 0; row < numRows; ++row) {
                armData.push_back(currentData[row][group * numArms + arm]);
            }
            
            // 计算该夹臂的面积
            totalArea += calculateArea(armData);
        }
        
        // 计算该组的平均面积
        avgAreas.push_back(totalArea / numArms);
    }

    // 为线性回归准备数据
    MatrixXd X(avgAreas.size(), 2);  // 两列：一列为面积，一列为截距项
    VectorXd y(weights.size());  // 目标变量（车辆重量）
    
    for (size_t i = 0; i < avgAreas.size(); ++i) {
        X(i, 0) = avgAreas[i];  // 第一列：平均面积
        X(i, 1) = 1.0;  // 第二列：截距项
        y(i) = weights[i];  // 车辆重量
    }

    // 线性回归：求解w = (X^T * X) ^ -1 * X^T * y
    VectorXd w = (X.transpose() * X).ldlt().solve(X.transpose() * y);

    // 输出回归模型参数
    cout << "线性回归模型参数:" << endl;
    cout << "截距 (beta_0): " << w(1) << endl;
    cout << "斜率 (beta_1): " << w(0) << endl;

    // 实时预测车辆重量
    vector<double> realTimeCurrentData = { /* 实时数据 */ };
    double realTimeArea = calculateArea(realTimeCurrentData);
    double estimatedWeight = w(0) * realTimeArea + w(1);

    // 输出预测结果
    cout << "预测的车辆重量: " << estimatedWeight << " 吨" << endl;
}

int main() {
    string filename = "predictive_table.csv";  // CSV文件路径
    processCSVData(filename);

    return 0;
}
