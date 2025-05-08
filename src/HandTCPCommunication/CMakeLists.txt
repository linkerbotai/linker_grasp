cmake_minimum_required(VERSION 2.8.3)
project(HandTCPCommunication)

set(CMAKE_CXX_STANDARD 11)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions 
  message_generation  # 消息生成依赖
)

# 添加消息文件
add_message_files(
  FILES
  TCPRecvSensorData.msg
  OrderDetectResult.msg
)

# 生成消息（根据实际依赖调整）
generate_messages(
  DEPENDENCIES
  std_msgs
)

# 声明catkin包属性
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    message_runtime  # 必须包含
)

# 包含目录（顺序重要！）
include_directories(
  include
  ${catkin_INCLUDE_DIRS}  # ROS生成消息头文件路径
  ${PROJECT_SOURCE_DIR}/handeyesolver/include  
  ${PCL_INCLUDE_DIRS} 
)

# 链接目录（检查路径拼写）
LINK_DIRECTORIES(
  /usr/lib/x86_64-linux-gnu  # 修正拼写错误："gun" → "gnu"
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

# 子目录和库
add_subdirectory(handeyesolver)

# 主库
add_library(TCPCommunication_Hand src/TCPCommunication_Hand.cpp)

# 可执行文件
add_executable(HandCtrl_node src/HandCtrl_node.cpp)

# 链接库
target_link_libraries(
  HandCtrl_node
  TCPCommunication_Hand
  handeyesolver
  ${catkin_LIBRARIES}
  jsoncpp
  -lRM_Service
)

# 安装规则（保持原样）
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS HandCtrl_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)