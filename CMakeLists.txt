cmake_minimum_required(VERSION 3.10)
set(BUILD_SHARED_LIBS ON)
set(CMAKE_CXX_STANDARD 17)              # Compile as C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)     # Require C++17 support

project(AV_riskfield
        VERSION 1.0.0
        LANGUAGES CXX
        )

## opencv
set(OpenCV_INCLUDE_DIRS /home/tan/opencv-3.4.16/include/opencv)
set(OpenCV_INCLUDE_DIRS /home/tan/opencv-3.4.16/include/opencv2)


# 添加头文件
include_directories(include ${OpenCV_INCLUDE_DIRS})

find_package(OpenCV 3.4.16 REQUIRED)

# generate library
file(GLOB_RECURSE MODEL_LIB_SRC
        src/model/*.cpp
)
add_library(model SHARED ${MODEL_LIB_SRC})

add_executable(demo src/main_test.cpp)

target_link_libraries(demo ${OpenCV_LIBS}  model)
include_directories("${PROJECT_SOURCE_DIR}/src/include")


## Compile as C++17
# add_compile_options(-std=c++17)

# People keep running CMake in the wrong folder, completely nuking their project or creating weird bugs.
# This checks if you're running CMake from a folder that already has CMakeLists.txt.
# Importantly, this catches the common case of running it from the root directory.
file(TO_CMAKE_PATH "${PROJECT_BINARY_DIR}/CMakeLists.txt" PATH_TO_CMAKELISTS_TXT)
if (EXISTS "${PATH_TO_CMAKELISTS_TXT}")
    message(FATAL_ERROR "Run CMake from a build subdirectory! \"mkdir build ; cd build ; cmake ..\" \
    Some junk files were created in this folder (CMakeCache.txt, CMakeFiles); you should delete those.")
endif ()

set(CMAKE_BUILD_TYPE Debug)

# Compiler flags.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ggdb -fsanitize=address -fno-omit-frame-pointer -fno-optimize-sibling-calls")

message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")
message(STATUS "CMAKE_EXE_LINKER_FLAGS: ${CMAKE_EXE_LINKER_FLAGS}")
message(STATUS "CMAKE_SHARED_LINKER_FLAGS: ${CMAKE_SHARED_LINKER_FLAGS}")

