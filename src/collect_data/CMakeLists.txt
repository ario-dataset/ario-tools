cmake_minimum_required(VERSION 2.8.3)
project(collect_data)
add_definitions(-w)
# 读取环境变量ROS_DISTRO
set(ROS_DISTRO $ENV{ROS_DISTRO})

# 输出读取到的ROS_DISTRO，便于调试
message(STATUS "-------------ROS_DISTRO: ${ROS_DISTRO}----------------")
# 根据ROS_DISTRO设置USE_NOETIC
if(ROS_DISTRO STREQUAL "noetic")
  set(USE_NOETIC true)
else()
  set(USE_NOETIC false)
endif()

set(CMAKE_BUILD_TYPE "Release")
if(USE_NOETIC)
  set(CMAKE_CXX_FLAGS "-std=c++14")
else()
  set(CMAKE_CXX_FLAGS "-std=c++11")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

if(USE_NOETIC)
  ADD_DEFINITIONS(-D_USENOETIC=\"${USE_NOETIC}\")
endif()

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  rosbag
  )
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
# find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL 1.10.0 REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(jsoncpp REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL EIGEN3 jsoncpp
  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Jsoncpp_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/rapidjson/include
)
set(lib_DIR /usr/lib/x84_64-linux-gnu)
# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${Jsoncpp_INCLUDE_DIRS}
  # ${lib_DIR}
)
message(---------${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/rapidyaml)
# 使用 BUILD_INTERFACE 和 INSTALL_INTERFACE 设置路径
# add_subdirectory 用于将一个子目录添加到构建过程中，并处理该子目录中的CMakeLists.txt文件。
# ${PROJECT_SOURCE_DIR}是一个CMake变量，表示顶级CMakeLists.txt文件所在的目录。
# ryml 目标二进制目录的名称。在生成的构建目录中，CMake会为该子目录创建一个名为 ryml 的子目录，并将所有构建文件放入其中。
# EXCLUDE_FROM_ALL 表示将该子目录从默认的 all 目标中排除。
# 也就是说，除非显式构建，否则不会自动构建该子目录中的目标。这在处理大型项目或第三方库时非常有用，因为你可能不希望在每次构建时都构建这些库。
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/rapidyaml ryml EXCLUDE_FROM_ALL)
# 下面的指令目的是告诉CMake在构建ryml目标时应该包含哪些头文件目录。在ROS功能包中，由于可能存在一些额外的限制或规定，你可能需要手动设置这些属性以确保构建顺利进行。
# set_target_properties 用于设置指定目标的属性。在此示例中，为目标 ryml 设置属性。
# ryml 目标名称。在调用 add_subdirectory 时，这个目标会在指定的子目录中定义。
# PROPERTIES 指定后续的属性和值。
# INTERFACE_INCLUDE_DIRECTORIES 这是一个目标属性，用于指定当其他目标依赖于此目标时应包含的头文件目录。
# 在此示例中，当其他目标依赖于 ryml 时，它们会自动包含指定的头文件目录。
# $<BUILD_INTERFACE:...>: 这是一个生成器表达式，在构建期间被求值。它表示在构建树中使用指定的目录（即构建过程中应包含的目录）。
# ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/rapidyaml/src：这是实际的头文件目录路径。
# ${CMAKE_CURRENT_SOURCE_DIR} 变量指的是当前处理的CMakeLists.txt文件所在的目录
set_target_properties(ryml PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/rapidyaml/src>;$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/rapidyaml/ext/c4core/src>"
)
###########
## Build ##
###########
# 定义自定义函数
function(copy_file_to_target target file)
  # add_custom_command用于向构建系统添加自定义命令。
  add_custom_command(
    # 将命令附加到指定的构建目标${target}。
    # POST_BUILD表示在目标构建完成后执行该命令。
    TARGET ${target} POST_BUILD
    # ${CMAKE_COMMAND}是一个CMake变量，表示CMake自身的命令行工具。
    # -E copy_if_different是一个CMake命令，用于复制文件（如果源文件和目标文件不同的话）。
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${CMAKE_CURRENT_SOURCE_DIR}/config/${file}
      # $<TARGET_FILE_DIR:target>是生成器表达式，用于在CMake构建过程中计算目标文件的目录。
      $<TARGET_FILE_DIR:${target}>
  )
endfunction()

add_executable(${PROJECT_NAME}_dataCapture src/dataCapture.cpp)
add_dependencies(${PROJECT_NAME}_dataCapture ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_dataCapture 
                      ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} 
                      libjsoncpp.a 
                      libusb-1.0.so
                      ryml::ryml
)

add_executable(${PROJECT_NAME}_dataSync src/dataSync.cpp)
add_dependencies(${PROJECT_NAME}_dataSync ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_dataSync 
                      ${catkin_LIBRARIES} 
                      ${PCL_LIBRARIES} 
                      ${OpenCV_LIBRARIES} 
                      libusb-1.0.so.0
                      # ryml::ryml
)

add_executable(${PROJECT_NAME}_dataPublish src/dataPublish.cpp)
add_dependencies(${PROJECT_NAME}_dataPublish ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_dataPublish 
                      ${catkin_LIBRARIES} 
                      ${PCL_LIBRARIES} 
                      ${OpenCV_LIBRARIES}  
                      libjsoncpp.a 
                      libusb-1.0.so
                      # ryml::ryml
)

# add_executable(${PROJECT_NAME}_pointCloudFilter src/pointCloudFilter.cpp)
# add_dependencies(${PROJECT_NAME}_pointCloudFilter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(${PROJECT_NAME}_pointCloudFilter 
#                       ${catkin_LIBRARIES} 
#                       ${PCL_LIBRARIES} 
#                       ${OpenCV_LIBRARIES}
#                       libjsoncpp.a 
#                       libusb-1.0.so
#                       # ryml::ryml
# )

# add_executable(${PROJECT_NAME}_readyaml src/readyaml.cpp src/getCurrentPath.cpp)
# add_dependencies(${PROJECT_NAME}_readyaml ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(${PROJECT_NAME}_readyaml
#                       ${catkin_LIBRARIES} 
#                       ryml::ryml
# )
# copy_file_to_target(${PROJECT_NAME}_readyaml "1.yaml")

# add_executable(${PROJECT_NAME}_readjson src/readjson.cpp src/getCurrentPath.cpp)
# add_dependencies(${PROJECT_NAME}_readjson ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(${PROJECT_NAME}_readjson
#                       ${catkin_LIBRARIES} 
# )
# copy_file_to_target(${PROJECT_NAME}_readjson "1.json")

#############
## install ##
#############
# install(TARGETS ${PROJECT_NAME}_dataCapture
# 	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY
#   launch
#   config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
