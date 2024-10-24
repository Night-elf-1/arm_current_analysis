cmake_minimum_required(VERSION 3.10)
project(ClampingArmCurrentAnalysis)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 指定头文件搜索路径
include_directories(include)

# 指定源文件
set(MAIN_SOURCE_FILE src/main.cpp)
set(TEST_SOURCE_FILE src/test.cpp)
set(CALCULATE_SOURCE_FILE src/calculate_.cpp)

# 添加主程序的可执行文件
add_executable(MainExecutable ${MAIN_SOURCE_FILE})

# 添加测试程序的可执行文件
add_executable(TestExecutable ${TEST_SOURCE_FILE})

# 添加计算程序的可执行文件
add_executable(CalculateExecutable ${CALCULATE_SOURCE_FILE})

# 如果您的项目依赖于Eigen库，并且Eigen库位于include目录下
find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIR})
    target_link_libraries(MainExecutable ${EIGEN3_LIBRARIES})
    target_link_libraries(TestExecutable ${EIGEN3_LIBRARIES})
    target_link_libraries(CalculateExecutable ${EIGEN3_LIBRARIES})
else()
    message(WARNING "Eigen3 not found. Please specify the path to Eigen3.")
endif()

# 添加测试（如果需要）
enable_testing()
add_test(NAME Test COMMAND TestExecutable)