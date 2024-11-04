#include "learn_predict_table.h"

void Linear_regression::readCSV(const string& filename, vector<ClampingArmData>& data) {
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

double Linear_regression::trapezoidalIntegration(const vector<double>& y, double dx) {
    double area = 0.0;
    for (size_t i = 1; i < y.size(); ++i) {
        area += (y[i] + y[i - 1]) * dx / 2.0;
    }
    return area;
}

void Linear_regression::calculateTrapzAreas(const vector<ClampingArmData>& data, vector<TrapzAreas>& currentAreas) {
    double numGroups = data.size() / 8;
    cout << "numGroups = " << numGroups << endl;

    if ((data.size() % 8) != 0) {
        cout << "输入的数据有误,不符合标准,请重新检查predictive_table.csv";
        return;
    }

    currentAreas.resize(numGroups);

    for (int i = 0; i < numGroups; i++) {
        double sumArea = 0.0;
        for (int j = 0; j < 8; j++) { // 每8列为一组
            int index = i * 8 + j;
            double weight = data[index].weight;
            const vector<double>& currentData = data[index].currentData;

            // 数据点之间的间距为1
            double dx = 0.1;
            // 梯形积分法计算面积
            double area = trapezoidalIntegration(currentData, dx);
            sumArea += area;
        }

        // 计算每组的面积平均值
        currentAreas[i].veicleWeight = data[i * 8].weight; // 假设每组的第一个重量为该组的重量
        currentAreas[i].veicleWeightCurrentAreas = sumArea / 8.0;
    }

    // 输出结果
    for (const auto& area : currentAreas) {
        cout << "veicleWeight = " << area.veicleWeight << ", veicleWeightCurrentAreas = " << area.veicleWeightCurrentAreas << endl;
    }
}

VectorXd Linear_regression::trainModel(const vector<double>& weights, const vector<double>& areas) {
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

void Linear_regression::saveWeights(const VectorXd& w, const string& filename) {
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

VectorXd Linear_regression::loadWeights(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "无法打开文件: " << filename << endl;
        return VectorXd();
    }

    vector<double> weights;
    string line;
    while (getline(file, line)) {
        weights.push_back(stod(line));
    }

    file.close();

    VectorXd w(weights.size());
    for (size_t i = 0; i < weights.size(); ++i) {
        w[i] = weights[i];
    }

    return w;
}

int main(int argc, char const *argv[])
{
    Linear_regression Lr;

    string filename = "/home/hamster/mycode/Arm_current_analysis/data/predictive_table.csv";            // CSV文件路径
    vector<ClampingArmData> clampingArmData;
    // readCSV(filename, clampingArmData);
    Lr.readCSV(filename, clampingArmData);

    vector<TrapzAreas> currentAreas;
    Lr.calculateTrapzAreas(clampingArmData, currentAreas);

    // 收集训练数据
    vector<double> weights;
    vector<double> areas;
    for (const auto& area : currentAreas) {
        weights.push_back(area.veicleWeight);
        areas.push_back(area.veicleWeightCurrentAreas);
    }

    // 训练线性回归模型
    VectorXd w = Lr.trainModel(weights, areas);

    // 保存权重到文件
    string weightsFile = "../data/weights.txt";
    Lr.saveWeights(w, weightsFile);

    cout << "线性回归模型参数: \n";
    cout << "Slope (m): " << w(0) << ", Intercept (b): " << w(1) << endl;
    cout << "w = " << w << endl;

    // 从文件加载权重
    VectorXd loadedWeights = Lr.loadWeights(weightsFile);
    cout << "loadedWeights = " << loadedWeights << endl;

    // 实时读取平均电流值
    double realTimeArea;
    cout << "输入实际电流值: ";
    cin >> realTimeArea;

    // 使用模型进行预测
    double estimatedWeight = w(0) * realTimeArea + w(1);
    //double estimatedWeight = loadedWeights(0) * realTimeArea + w(1);

    // 显示预测结果
    cout << "预测重量: " << estimatedWeight << endl;

    return 0;
}
