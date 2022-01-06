cmake_minimum_required(VERSION 3.0)

project(grasp_zed)

find_package(Boost COMPONENTS system REQUIRED)
find_package(ZED 2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

IF(NOT WIN32)
    add_definitions(-Wno-format-extra-args)
    SET(SPECIAL_OS_LIBS "pthread" "X11")
ENDIF()

add_definitions(-std=c++14 -g -O3)#类似g++的各种后追，进行调试，优化代码，输出报错等

set(DARKNET_ROOT "/home/yxk/workspace/darknet-master")

set(CMAKE_BUILD_TYPE Debug)

include_directories(${CUDA_INCLUDE_DIRS}
        ${ZED_INCLUDE_DIRS}
        ${OPENCV_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/include
        ${DARKNET_ROOT}/include)

 link_directories(
    ${ZED_LIBRARY_DIR}
    ${OpenCV_LIBRARY_DIRS} 
    ${CUDA_LIBRARY_DIRS})


add_executable(${PROJECT_NAME} 
        src/main.cpp 
        src/camera.cpp 
        src/Arm_ikine.cpp
        src/mydetector.cpp)

target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES} 
    ${ZED_LIBRARIES} 
    ${SPECIAL_OS_LIBS}
    ${OpenCV_LIBRARIES} 
    ${CUDA_CUDA_LIBRARY} 
    ${CUDA_CUDART_LIBRARY} 
    ${CUDA_NPP_LIBRARIES_ZED} 
    ${DARKNET_ROOT}/libdarknet.so)
