cmake_minimum_required(VERSION 2.8.3)
project(ADS1115)

set(PROJECT_BRIEF "\"\"")

# Enable/disable testing
option(TESTS_ENABLED "Enable automatic tests" OFF)

set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/lib")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")

set(CMAKE_CXX_FLAGS_DEBUG "-Wall -O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O2")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, defaulting to Debug")
    set(CMAKE_BUILD_TYPE "Debug")
endif()

add_subdirectory(ADS1115)

## Uncomment for example
add_executable(ADS1115_example example/ADS1115_example.cpp impl/unix/src/unix.cpp)
target_include_directories(${PROJECT_NAME}_example
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/impl/unix/include>
    $<INSTALL_INTERFACE:include>
)
add_dependencies(ADS1115_example ${PROJECT_NAME})
target_link_libraries(ADS1115_example ${PROJECT_NAME})

