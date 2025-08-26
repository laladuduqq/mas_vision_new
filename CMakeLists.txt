cmake_minimum_required(VERSION 3.10)
project(base LANGUAGES CXX)

# C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 初始化库和包含目录列表
set(MAIN_PROJECT_LIBRARIES "" CACHE INTERNAL "Libraries for main project")
set(MAIN_PROJECT_INCLUDE_DIRS "" CACHE INTERNAL "Include directories for main project")

# 自动查找所有子目录中的 CMakeLists.txt (除了 cmake 目录)
file(GLOB_RECURSE CMAKE_FILES 
    LIST_DIRECTORIES false 
    RELATIVE ${CMAKE_SOURCE_DIR}
    "*/CMakeLists.txt"
)

# 遍历找到的 CMakeLists.txt 文件
foreach(CMAKE_FILE ${CMAKE_FILES})
    # 获取 CMakeLists.txt 所在的目录路径
    get_filename_component(DIR ${CMAKE_FILE} DIRECTORY)
    
    # 排除 cmake 目径及其子目录
    if(NOT DIR MATCHES "^cmake.*")
        # 添加子目录
        add_subdirectory(${DIR})
    endif()
endforeach()

# 添加可执行文件
add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/applications/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/applications/calibration/calibrate_camera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/applications/calibration/calibrate_handeye.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/applications/calibration/calibrate_worldhandeye.cpp
)


# 设置包含目录
if(MAIN_PROJECT_INCLUDE_DIRS)
    target_include_directories(${PROJECT_NAME} PRIVATE ${MAIN_PROJECT_INCLUDE_DIRS})
endif()

# 链接库
if(MAIN_PROJECT_LIBRARIES)
    target_link_libraries(${PROJECT_NAME} ${MAIN_PROJECT_LIBRARIES})
endif()

# 创建软链接到源代码中的config文件夹
if(UNIX AND NOT APPLE)
    # Linux系统使用create_symlink
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_CURRENT_SOURCE_DIR}/config
        ${CMAKE_CURRENT_BINARY_DIR}/config
    )
elseif(APPLE)
    # macOS系统也使用create_symlink
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_CURRENT_SOURCE_DIR}/config
        ${CMAKE_CURRENT_BINARY_DIR}/config
    )
else()
    # Windows系统回退到复制方式
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()