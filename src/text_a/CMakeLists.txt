
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(text_c LANGUAGES C)

FILE(GLOB app_sources src/*.c)

target_sources(app PRIVATE ${app_sources})
target_include_directories(app PRIVATE ${ZEPHYR_BASE}/include)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dts)

include_directories(${SYSCALL_INCLUDE_DIRS})