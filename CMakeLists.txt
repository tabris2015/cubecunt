cmake_minimum_required(VERSION 3.5)

project(cubecunt)

set(CMAKE_CXX_STANDARD 11)

include_directories(./librobotcontrol/library/include)

file(GLOB lib_src "./librobotcontrol/library/src/*.c")
file(GLOB bmp_src "./librobotcontrol/library/src/bmp/*.c")
file(GLOB io_src "./librobotcontrol/library/src/io/*.c")
file(GLOB math_src "./librobotcontrol/library/src/math/*.c")
file(GLOB mpu_src "./librobotcontrol/library/src/mpu/*.c")
file(GLOB pru_src "./librobotcontrol/library/src/pru/*.c")

add_library(robotcontrol SHARED ${lib_src} ${bmp_src} ${io_src} ${math_src} ${mpu_src} ${pru_src} )

add_executable(cubecunt ./src/main.cpp)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

target_link_libraries(cubecunt Threads::Threads robotcontrol)