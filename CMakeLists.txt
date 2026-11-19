cmake_minimum_required(VERSION 3.16)
project(homework_3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 生成 compile_commands.json，给 VS Code 的 IntelliSense 用。
# 没有这个文件，编辑器找不到 OpenCV 头文件会满屏红线。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV 4 REQUIRED
    COMPONENTS core imgproc imgcodecs highgui videoio aruco calib3d)

add_executable(homework_3
    src/main.cpp
    src/Detector.cpp
    src/pnp_solver.cpp
)

target_include_directories(homework_3 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(homework_3 PRIVATE ${OpenCV_LIBS})

target_compile_options(homework_3 PRIVATE -Wall -Wextra -Wpedantic)
