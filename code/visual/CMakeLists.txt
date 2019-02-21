# 声明要求的cmake最低版本
cmake_minimum_required( VERSION 2.8 )
# 添加c++11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 声明一个cmake工程
project(visual)

# 找到后面需要库和头文件的包
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)
aux_source_directory(src DIR_SRCS)
set(PATH ${DIR_SRCS})

add_executable(main ${PATH})
target_link_libraries(main ${OpenCV_LIBS})