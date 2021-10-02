cmake_minimum_required(VERSION 2.8.12)

project(stone_detect)

add_definitions(-std=c++11)

#option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/GxCamera/include/include)
include_directories(${PROJECT_SOURCE_DIR}/GxCamera/include)
include_directories(${PROJECT_SOURCE_DIR}/Serial/Serial.h)

find_package(Qt5Core)
find_package(OpenCV)
include_directories(OpenCV_INCLUDE_DIRS)
file(GLOB STONE ${PROJECT_SOURCE_DIR}/GxCamera/GxCamera.cpp ${PROJECT_SOURCE_DIR}/stone.cpp ${PROJECT_SOURCE_DIR}/Serial/Serial.cpp )


add_executable(${PROJECT_NAME} "stone_detect.cpp" ${STONE})

target_link_libraries(${PROJECT_NAME} Qt5::Core)
target_link_libraries(stone_detect ${OpenCV_LIBS})
#pthread GxCamera X11
target_link_libraries(stone_detect -lpthread -lm -lgxiapi -lX11)

add_definitions(-O2 -pthread)
