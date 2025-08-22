# 创建静态库 only for linux
add_library(serial STATIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/common/src/serial.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/common/src/impl/list_ports/list_ports_linux.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/common/src/impl/unix.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include/serial/serial.h
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include/serial/v8stdint.h
    ${CMAKE_CURRENT_SOURCE_DIR}/thread/serial_thread.cpp
)

find_package(yaml-cpp REQUIRED)

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)

# 设置库的包含目录
target_include_directories(serial PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include/serial
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include/serial/impl
)

# 链接所需库
target_link_libraries(serial PUBLIC
    rt 
    pthread
    rm_utils
    yaml-cpp::yaml-cpp
)

# 将库添加到全局变量中，供主CMakeLists.txt使用
list(APPEND MAIN_PROJECT_LIBRARIES serial)
list(APPEND MAIN_PROJECT_INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include/serial
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include/serial/impl)

set(MAIN_PROJECT_LIBRARIES ${MAIN_PROJECT_LIBRARIES} CACHE INTERNAL "Libraries for main project")
set(MAIN_PROJECT_INCLUDE_DIRS ${MAIN_PROJECT_INCLUDE_DIRS} CACHE INTERNAL "Include directories for main project")