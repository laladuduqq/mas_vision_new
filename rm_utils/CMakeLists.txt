# 创建静态库
add_library(rm_utils STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/performance_monitor/performance_monitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/PubSub/pubsub.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/PubSub/thread/pubsub_thread.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ulog/ulog.cpp
)

find_package(OpenCV REQUIRED)

# 设置库的包含目录
target_include_directories(rm_utils PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/performance_monitor
    ${CMAKE_CURRENT_SOURCE_DIR}/PubSub
    ${CMAKE_CURRENT_SOURCE_DIR}/ulog
)

# 链接所需库
target_link_libraries(rm_utils PUBLIC
    ${OpenCV_LIBS}
)

# 将库添加到全局变量中，供主CMakeLists.txt使用
list(APPEND MAIN_PROJECT_LIBRARIES rm_utils)
list(APPEND MAIN_PROJECT_INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/performance_monitor
    ${CMAKE_CURRENT_SOURCE_DIR}/PubSub
    ${CMAKE_CURRENT_SOURCE_DIR}/ulog)


set(MAIN_PROJECT_LIBRARIES ${MAIN_PROJECT_LIBRARIES} CACHE INTERNAL "Libraries for main project")
set(MAIN_PROJECT_INCLUDE_DIRS ${MAIN_PROJECT_INCLUDE_DIRS} CACHE INTERNAL "Include directories for main project")