#include "predict_weight.h"

void PredictWeights::receive_data(vector<vector<double>>& arms){
    double arm1, arm2, arm3, arm4, arm5, arm6, arm7, arm8;                  // 这里的ARM1 ~ ARM8替换成八个夹臂的当前帧数据
    arms[0].push_back(arm1);
    arms[1].push_back(arm2);
    arms[2].push_back(arm3);
    arms[3].push_back(arm4);
    arms[4].push_back(arm5);
    arms[5].push_back(arm6);
    arms[6].push_back(arm7);
    arms[7].push_back(arm8);
}

double PredictWeights::trapezoidalIntegration(const vector<double>& y, double dx) {
    double area = 0.0;
    for (size_t i = 1; i < y.size(); ++i) {
        area += (y[i] + y[i - 1]) * dx / 2.0;
    }
    return area;
}

VectorXd PredictWeights::loadWeights(const string& filename) {
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
    PredictWeights pw;

    vector<vector<double>> arms(8);

    while (!pw.isfinish)                        // isfinish 为夹臂完成标志位，结束接受数据
    {
        pw.receive_data(arms);
    }
    
    double current_Areas;
    for (int i = 0; i < arms.size(); i++)
    {
        current_Areas += pw.trapezoidalIntegration(arms[i], 1.0);
    }
    current_Areas = current_Areas / arms.size();

    string weightsFile = "../data/weights.txt";
    VectorXd loadedWeights = pw.loadWeights(weightsFile);
    cout << "loadedWeights = " << loadedWeights << endl;

    // 使用模型进行预测
    double estimatedWeight = loadedWeights(0) * current_Areas + loadedWeights(1);

    cout << "预估重量 = " << estimatedWeight << endl;

    return 0;
}
