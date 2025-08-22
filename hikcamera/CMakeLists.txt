# hikcamera/CMakeLists.txt
# 创建静态库
add_library(HikCamera STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/HikCamera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/thread/cam_thread.cpp
)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# 手动指定 MVS 路径（安装默认 /opt/MVS）
set(MVS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/SDK")
set(MVS_INCLUDE_DIR "${MVS_ROOT}/include")
set(MVS_LIB_DIR "${MVS_ROOT}/lib/64")

# 设置库的包含目录
target_include_directories(HikCamera PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${MVS_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 链接库目录
target_link_directories(HikCamera PUBLIC 
    ${MVS_LIB_DIR}
)

# 链接所需库
target_link_libraries(HikCamera PUBLIC
    ${OpenCV_LIBS}
    MvCameraControl
    rm_utils
    yaml-cpp::yaml-cpp
)

# 运行时 rpath，防止找不到 .so
target_link_options(HikCamera PUBLIC
    "-Wl,-rpath,${MVS_LIB_DIR}"
)

# 将库添加到全局变量中，供主CMakeLists.txt使用
list(APPEND MAIN_PROJECT_LIBRARIES HikCamera)
list(APPEND MAIN_PROJECT_INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${MVS_INCLUDE_DIR})

set(MAIN_PROJECT_LIBRARIES ${MAIN_PROJECT_LIBRARIES} CACHE INTERNAL "Libraries for main project")
set(MAIN_PROJECT_INCLUDE_DIRS ${MAIN_PROJECT_INCLUDE_DIRS} CACHE INTERNAL "Include directories for main project")