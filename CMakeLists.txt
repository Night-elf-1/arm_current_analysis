cmake_minimum_required(VERSION 3.10)

# 项目名称
project(ClampingArmCurrentAnalysis)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 寻找依赖库Eigen
find_package(Eigen3 REQUIRED)

# 指定源文件
set(SOURCE_FILES
    src/main.cpp
)

# 添加可执行文件
include_directories(${EIGEN3_INCLUDE_DIR})
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# 链接Eigen库
target_link_libraries(${PROJECT_NAME} 
    Eigen3::Eigen
)

# 指定编译输出路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
