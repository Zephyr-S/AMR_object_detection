cmake_minimum_required(VERSION 3.5)
project(object_detection)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Werror -fstack-protector-all)
endif()

message(STATUS "Looking for inference engine configuration file at: ${CMAKE_PREFIX_PATH}")
find_package(InferenceEngine REQUIRED)
if(NOT InferenceEngine_FOUND)
  message(FATAL_ERROR "")
endif()


# Find OpenCV libray if exists
find_package(OpenCV ${OPENCV_AMR_VER} EXACT REQUIRED)
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${OPEN_ZOO_HEADERS}/utils/include
    ${OPEN_ZOO_HEADERS}/models/include
    ${OPEN_ZOO_HEADERS}/pipelines/include
)

if(OpenCV_FOUND)
    add_definitions(-DUSE_OPENCV)
endif()

find_package(ngraph REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

set (dependencies
  rclcpp
  std_msgs
  cv_bridge
  image_geometry
  visualization_msgs
  Eigen3
  image_transport
)


## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME} 
  "msg/Objects.msg" 
  "msg/Object.msg"
  DEPENDENCIES builtin_interfaces sensor_msgs std_msgs visualization_msgs image_geometry
  )

###########
## Build ##
######  
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ament_cmake_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(object_detection_node
  src/ros2_wrapper.cpp
  src/object_detection.cpp
  src/main.cpp)

rosidl_target_interfaces(object_detection_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(object_detection_node ${dependencies})

install(TARGETS 
  object_detection_node
  DESTINATION lib/${PROJECT_NAME}
)                                                                                                                                                                                                                                                                                                                                                            
 

install(TARGETS object_detection_node
  ARCHIVE DESTINATION lib/                                                                                                                                                                                                                                                                                                                                                             
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


set(LIBRARY_DIRS 
  ${ament_cmake_LIBRARIES} 
  ${rclcpp_LIBRARIES} 
  ${InferenceEngine_LIBRARIES} 
  ${cv_bridge_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES} 
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
  -lstdc++fs)

find_library(MONITORS_LIB monitors)
find_library(PIPELINES_LIB pipelines)
find_library(MODELS_LIB models)
find_library(UTILS_LIB utils)
find_library(GFLAGS_LIB gflags_nothreads)

if(NOT MONITORS_LIB OR
NOT PIPELINES_LIB OR
NOT MODELS_LIB OR
NOT UTILS_LIB OR
NOT GFLAGS_LIB
)
  message(FATAL_ERROR "Make sure CMAKE_PREFIX_PATH points to OpenModelZoo Intel64 Compiled libs")
endif()

target_link_libraries(object_detection_node ${LIBRARY_DIRS} ${GFLAGS_LIB} ${UTILS_LIB} ${MONITORS_LIB} ${PIPELINES_LIB} ${MODELS_LIB} ${NGRAPH_LIBRARIES} ${ament_index_cpp_LIBRARIES})


install(DIRECTORY 
  models/ 
  DESTINATION share/${PROJECT_NAME}/models
  )

install(DIRECTORY 
  msg/ 
  DESTINATION share/${PROJECT_NAME}/msg)

install(DIRECTORY 
  rviz/ 
  DESTINATION share/${PROJECT_NAME}/rviz)

install(DIRECTORY 
  src/ 
  DESTINATION share/${PROJECT_NAME}/src)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/launch
)


ament_export_include_directories(${OPEN_ZOO_HEADERS}/utils/include)
ament_export_include_directories(${OPEN_ZOO_HEADERS}/models/include)
ament_export_include_directories(${OPEN_ZOO_HEADERS}/pipelines/include)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest)
  ament_add_gtest(test_inputs tests/test_inputs.cpp src/ros2_wrapper.cpp src/object_detection.cpp TIMEOUT 15)
  ament_target_dependencies(test_inputs ${dependencies})
  rosidl_target_interfaces(test_inputs ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(test_inputs ${LIBRARY_DIRS} ${GFLAGS_LIB} ${UTILS_LIB} ${MONITORS_LIB} ${PIPELINES_LIB} ${MODELS_LIB} ${NGRAPH_LIBRARIES} ${ament_index_cpp_LIBRARIES})
  ament_add_test(test_neg_param
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/tests/launch/test_neg_param.launch.py"
    TIMEOUT 20
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  )
  ament_add_gtest_executable(test_object_detection tests/test_object_detection.cpp src/ros2_wrapper.cpp src/object_detection.cpp)
  ament_target_dependencies(test_object_detection ${dependencies})
  rosidl_target_interfaces(test_object_detection ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(test_object_detection ${LIBRARY_DIRS}  ${GFLAGS_LIB} ${UTILS_LIB} ${MONITORS_LIB} ${PIPELINES_LIB} ${MODELS_LIB} ${NGRAPH_LIBRARIES} ${ament_index_cpp_LIBRARIES})
  ament_add_test(test_object_detection
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/tests/launch/test_object_detection.launch.py"
    TIMEOUT 30
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    ENV
      TEST_EXECUTABLE=$<TARGET_FILE:test_object_detection>
  )
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(core)
ament_export_dependencies(imgproc)
ament_export_dependencies(highgui)
ament_export_dependencies(InferenceEngine)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(OpenCV)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(image_geometry)
ament_export_dependencies(image_transport)
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(${INCLUDE_DIRS})


ament_package()
