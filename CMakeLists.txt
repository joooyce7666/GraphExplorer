cmake_minimum_required(VERSION 3.0.2)
project(graph_exploration)

## 查找依赖的 ROS 包和 Boost 库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  # 添加其他需要的 ROS 依赖包
)

find_package(Boost REQUIRED)

## 声明 ROS 消息、服务和动作，如果有的话
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## 生成添加的消息和服务的依赖项
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

## 配置包的其他依赖
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES dart_exploration
  CATKIN_DEPENDS roscpp rospy tf2_ros tf2_geometry_msgs std_msgs visualization_msgs
  # DEPENDS system_lib
)

## 包括头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
add_compile_options(-std=c++17)

## 声明 C++ 可执行文件
add_executable(graph src/graph.cpp src/function.cpp src/visualization.cpp)

## 添加链接库到可执行文件
target_link_libraries(graph
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## 如果是自动生成的消息或服务，添加依赖项
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
