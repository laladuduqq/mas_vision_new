# 创建静态库
add_library(auto_aim STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/armor_detector/armor_detector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/armor_detector/number_classifier.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/thread/auto_aim_thread.cpp
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)


# 设置库的包含目录
target_include_directories(auto_aim PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/armor_detector
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 链接所需库
target_link_libraries(auto_aim PUBLIC
    ${OpenCV_LIBS}
    MvCameraControl
    rm_utils
    yaml-cpp::yaml-cpp
    serial
    HikCamera
)


# 将库添加到全局变量中，供主CMakeLists.txt使用
list(APPEND MAIN_PROJECT_LIBRARIES auto_aim)
list(APPEND MAIN_PROJECT_INCLUDE_DIRS 
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/armor_detector
    ${CMAKE_CURRENT_SOURCE_DIR})

set(MAIN_PROJECT_LIBRARIES ${MAIN_PROJECT_LIBRARIES} CACHE INTERNAL "Libraries for main project")
set(MAIN_PROJECT_INCLUDE_DIRS ${MAIN_PROJECT_INCLUDE_DIRS} CACHE INTERNAL "Include directories for main project")