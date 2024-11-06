#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

int main() {
    // 文件名
    std::string filename = "/home/hamster/mycode/Arm_current_analysis/data/armsdata_J75_11-05(1.3t).txt";
    std::ifstream file(filename);

    // 检查文件是否成功打开
    if (!file.is_open()) {
        std::cerr << "无法打开文件 " << filename << std::endl;
        return 1;
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

    // 输出一些数据验证读取是否正确
    // for (size_t i = 0; i < datas.size(); ++i) {
    //     std::cout << "组 " << i + 1 << ":\n";
    //     for (const auto& row : datas[i]) {
    //         for (double val : row) {
    //             std::cout << val << " ";
    //         }
    //         std::cout << "\n";
    //     }
    //     std::cout << "------------------------\n";
    // }
    std::cout<< datas.size() << std::endl;
    for (int i = 0; i < datas[33][7].size(); i++)
    {
        /* code */
        std::cout << datas[33][7][i] << std::endl;
    }

    return 0;
}

