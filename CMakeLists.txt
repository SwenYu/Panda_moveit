cmake_minimum_required(VERSION 3.5)
project(panda_moveit)

# 设置 C++ 版本
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)  # 添加 tf2_ros 依赖
find_package(tf2 REQUIRED)        # 添加 tf2 依赖
find_package(CURL REQUIRED) 
# 包含目录
include_directories(include)

# 添加可执行文件
add_executable(panda_moveit src/panda.cpp)

# 目标依赖
ament_target_dependencies(panda_moveit
    rclcpp
    std_msgs
    moveit_ros_planning_interface
    tf2_ros
    tf2
    CURL
)
target_link_libraries(panda_moveit ${CURL_LIBRARIES})

# 安装目标
install(TARGETS
    panda_moveit
    DESTINATION lib/${PROJECT_NAME}
)

# 生成包
ament_package()
