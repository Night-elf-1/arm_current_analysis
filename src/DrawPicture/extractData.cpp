// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <vector>
// #include <string>

// // 定义一个结构体来存储单个夹臂的数据
// struct ClampData {
//     std::vector<double> current_values;
//     double mean_current;
//     double current_area;
// };

// // 从文件中读取数据并解析
// std::vector<ClampData> parseDataFromFile(const std::string& filename) {
//     std::vector<ClampData> clamp_data_vector;
//     std::ifstream file(filename);
//     std::string line;

//     if (!file.is_open()) {
//         std::cerr << "Unable to open file" << std::endl;
//         return clamp_data_vector;
//     }

//     // 读取每一行
//     while (getline(file, line)) {
//         if (line.find("====================================") != std::string::npos) {
//             // 如果是分隔符，跳过
//             continue;
//         }

//         std::stringstream ss(line);
//         ClampData clamp_data;
//         double value;

//         // 读取电流值
//         while (ss >> value) {
//             clamp_data.current_values.push_back(value);
//         }

//         // 由于每8个电流值后是均值和面积，所以在这里计算均值和面积
//         if (clamp_data.current_values.size() == 8) {
//             double sum = 0;
//             for (auto& val : clamp_data.current_values) {
//                 sum += val;
//             }
//             clamp_data.mean_current = sum / 8.0;
//             clamp_data.current_area = clamp_data.current_values.back(); // 假设最后一个值是面积
//             clamp_data_vector.push_back(clamp_data);
//             // 重置为下一组数据
//             clamp_data.current_values.clear();
//         }
//     }

//     file.close();
//     return clamp_data_vector;
// }

// int main() {
//     std::string filename = "/home/hamster/mycode/arm_current_analysis/data/armsdata_J75_11-05(1.3t).txt"; // 替换为你的文件名
//     auto clamp_data_vector = parseDataFromFile(filename);

//     std::cout << "size = " << clamp_data_vector.size() << std::endl;
//     // 打印读取的数据，以验证
//     // for (const auto& clamp_data : clamp_data_vector) {
//     //     std::cout << "Mean Current: " << clamp_data.mean_current << std::endl;
//     //     std::cout << "Current Area: " << clamp_data.current_area << std::endl;
//     //     std::cout << "Current Values: ";
//     //     for (const auto& val : clamp_data.current_values) {
//     //         std::cout << val << " ";
//     //     }
//     //     std::cout << std::endl;
//     // }

//     return 0;
// }



// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <vector>
// #include <string>

// // 定义一个容器来存储每组电流数据
// typedef std::vector<double> CurrentData;
// typedef std::vector<CurrentData> AllData;
// std::vector<std::vector<std::vector<double>>> datas;

// AllData parseDataFromFile(const std::string& filename) {
//     std::ifstream file(filename);
//     std::string line;
//     AllData allData;
//     CurrentData currentData;

//     if (!file.is_open()) {
//         std::cerr << "Unable to open file" << std::endl;
//         return allData;
//     }

//     while (getline(file, line)) {
//         // 检查是否是数据行
//         if (line.find("====================================") != std::string::npos) {
//             if (currentData.size() > 0) {
//                 allData.push_back(currentData); // 保存当前组数据
//                 currentData.clear(); // 重置当前组数据
//             }
//             continue;
//         }

//         std::stringstream ss(line);
//         double value;
//         while (ss >> value) {
//             currentData.push_back(value);
//         }
//     }

//     // 确保最后一组数据也被添加
//     if (currentData.size() > 0) {
//         allData.push_back(currentData);
//     }

//     file.close();
//     return allData;
// }

// int main() {
//     std::string filename = "/home/hamster/mycode/arm_current_analysis/data/armsdata_J75_11-05(1.3t).txt"; // 替换为你的文件名
//     auto allData = parseDataFromFile(filename);

//     // 打印读取的数据，以验证
//     // for (size_t i = 0; i < allData.size(); ++i) {
//     //     std::cout << "Group " << i + 1 << ":" << std::endl;
//     //     for (const auto& value : allData[i]) {
//     //         std::cout << value << " ";
//     //     }
//     //     std::cout << std::endl;
//     // }

//     for (int i = 0; i < allData.size(); i++)
//     {
//         if (i == 0)
//         {
//             /* code */
//             for (int j = 0; j < allData[i].size(); j++)
//             {
//                 std::cout << allData[i][j] << std::endl;
//             }
//         }
//     }
    
    
//     return 0;
// }



// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <vector>
// #include <string>

// // 定义数据结构
// typedef std::vector<double> DataLine;        // 一行中的所有电流数据
// typedef std::vector<DataLine> DataGroup;     // 一组中的8行电流数据
// typedef std::vector<DataGroup> AllData;     // 所有组的数据

// AllData parseDataFromFile(const std::string& filename) {
//     std::ifstream file(filename);
//     std::string line;
//     AllData allData;
//     DataGroup currentGroup;

//     if (!file.is_open()) {
//         std::cerr << "Unable to open file" << std::endl;
//         return allData;
//     }

//     while (getline(file, line)) {
//         // 检查是否是组分隔符
//         if (line.find("====================================") != std::string::npos) {
//             if (!currentGroup.empty()) {
//                 allData.push_back(currentGroup); // 保存当前组数据
//                 currentGroup.clear(); // 重置当前组数据
//             }
//             continue;
//         }

//         std::stringstream ss(line);
//         DataLine currentLine;
//         double value;

//         // 解析行数据
//         while (ss >> value) {
//             currentLine.push_back(value);
//         }

//         // 检查是否已存储8行数据
//         if (currentLine.size() > 0 && currentGroup.size() < 8) {
//             currentGroup.push_back(currentLine);
//             currentLine.clear();
//         }
//     }

//     // 确保最后一组数据也被添加
//     if (!currentGroup.empty()) {
//         allData.push_back(currentGroup);
//     }

//     file.close();
//     return allData;
// }

// int main() {
//     std::string filename = "/home/hamster/mycode/arm_current_analysis/data/armsdata_J75_11-05(1.3t).txt"; // 替换为你的文件名
//     AllData allData = parseDataFromFile(filename);

//     // 打印读取的数据，以验证
//     // for (size_t i = 0; i < allData.size(); ++i) {
//     //     std::cout << "Group " << i + 1 << ":" << std::endl;
//     //     for (size_t j = 0; j < allData[i].size(); ++j) {
//     //         const DataLine& line = allData[i][j];
//     //         std::cout << "  Line " << j + 1 << ": ";
//     //         for (size_t k = 0; k < line.size(); ++k) {
//     //             std::cout << line[k] << " ";
//     //         }
//     //         std::cout << std::endl;
//     //     }
//     // }

//     for (int i = 0; i < allData.size(); i++)
//     {
//         if (i == 0)
//         {
//             /* code */
//             for (int j = 0; j < allData[i].size(); j++)
//             {
//                 for (int k = 0; k < allData[i][j].size(); k++)
//                 {
//                     /* code */
//                     std::cout << allData[i][j][k] << std::endl;
//                 }
//             }
//         }
//     }

//     return 0;
// }



// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <vector>
// #include <string>
// #include <cctype>

// int main() {
//     // 文件名
//     std::string filename = "/home/hamster/mycode/arm_current_analysis/data/armsdata_J75_11-05(1.3t).txt";
//     std::ifstream file(filename);

//     // 检查文件是否成功打开
//     if (!file.is_open()) {
//         std::cerr << "无法打开文件 " << filename << std::endl;
//         return 1;
//     }

//     // 容器结构 datas
//     std::vector<std::vector<std::vector<double>>> datas;
//     std::vector<std::vector<double>> currentGroup;
//     std::string line;
//     int rowCount = 0;

//     // 逐行读取文件
//     while (std::getline(file, line)) {
//         // 检查是否是分隔符
//         if (line.find("====================================") != std::string::npos) {
//             if (!currentGroup.empty()) {
//                 datas.push_back(currentGroup);
//                 currentGroup.clear();
//             }
//             rowCount = 0;
//             continue;
//         }

//         // 处理数据行
//         std::vector<double> row;
//         std::istringstream ss(line);
//         std::string value;
//         while (std::getline(ss, value, ',')) {
//             // 移除两端的空格
//             value.erase(0, value.find_first_not_of(' '));
//             value.erase(value.find_last_not_of(' ') + 1);

//             // 检查是否为有效数字
//             if (value.empty() || (!std::isdigit(value[0]) && value[0] != '-' && value[0] != '+')) {
//                 // std::cerr << "跳过非数值内容: " << value << std::endl;
//                 continue; // 跳过非数值
//             }

//             try {
//                 row.push_back(std::stod(value));
//             } catch (const std::invalid_argument& e) {
//                 //std::cerr << "无法转换的值: " << value << std::endl;
//             }
//         }

//         if (!row.empty()) {
//             currentGroup.push_back(row);
//             rowCount++;
//         }

//         // 每8行结束一个组
//         if (rowCount == 8) {
//             datas.push_back(currentGroup);
//             currentGroup.clear();
//             rowCount = 0;
//         }
//     }

//     // 若文件结束时有未存储的组
//     if (!currentGroup.empty()) {
//         datas.push_back(currentGroup);
//     }

//     file.close();

//     // 输出一些数据验证读取是否正确
//     for (int i = 0; i < datas[1][1].size(); i++)
//     {
//         /* code */
//         std::cout << datas[1][1][i] << std::endl;
//     }
    

//     // for (size_t i = 0; i < datas.size(); ++i) {
//     //     if (i == 0)
//     //     {
//     //         /* code */
//     //         std::cout << "组 " << i + 1 << ":\n";
//     //         for (const auto& row : datas[i]) {
//     //             for (double val : row) {
//     //                 std::cout << val << " ";
//     //             }
//     //             std::cout << "\n";
//     //         }
//     //         std::cout << "------------------------\n";
//     //     }
        
        
//     // }

//     return 0;
// }


#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

int main() {
    // 文件名
    std::string filename = "/home/hamster/mycode/arm_current_analysis/data/armsdata_J75_11-05(1.3t).txt";
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

