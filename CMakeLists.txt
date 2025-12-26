cmake_minimum_required(VERSION 3.16)
project(ShapePreservingSimplification
    VERSION 1.0.0
    LANGUAGES CXX
    DESCRIPTION "Shape-Preserving Mesh Simplification for Urban Building Models"
)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
if(MSVC)
    add_compile_options(/W4 /permissive-)
    add_compile_definitions(_USE_MATH_DEFINES)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Options
option(SPS_BUILD_TESTS "Build unit tests" ON)
option(SPS_BUILD_EXAMPLES "Build examples" ON)
option(SPS_USE_OPENMP "Use OpenMP for parallelization" ON)

# Find packages
find_package(Eigen3 3.3 REQUIRED)

# OpenMP (optional)
if(SPS_USE_OPENMP)
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        message(STATUS "OpenMP found, parallel processing enabled")
    endif()
endif()

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/third_party)

# Collect source files
file(GLOB_RECURSE CORE_SOURCES "src/core/*.cpp")
file(GLOB_RECURSE FILTER_SOURCES "src/filter/*.cpp")
file(GLOB_RECURSE DETECTION_SOURCES "src/detection/*.cpp")
file(GLOB_RECURSE SIMPLIFY_SOURCES "src/simplify/*.cpp")
file(GLOB_RECURSE IO_SOURCES "src/io/*.cpp")

set(LIB_SOURCES
    ${CORE_SOURCES}
    ${FILTER_SOURCES}
    ${DETECTION_SOURCES}
    ${SIMPLIFY_SOURCES}
    ${IO_SOURCES}
)

# Library
add_library(sps_lib STATIC ${LIB_SOURCES})
target_link_libraries(sps_lib PUBLIC Eigen3::Eigen)

if(OpenMP_CXX_FOUND)
    target_link_libraries(sps_lib PUBLIC OpenMP::OpenMP_CXX)
    target_compile_definitions(sps_lib PUBLIC SPS_USE_OPENMP)
endif()

# Main executable
add_executable(sps_simplify src/main.cpp)
target_link_libraries(sps_simplify PRIVATE sps_lib)

# Tests
if(SPS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Examples
if(SPS_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Install
install(TARGETS sps_simplify sps_lib
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY include/sps DESTINATION include)

# Summary
message(STATUS "")
message(STATUS "=== Shape-Preserving Simplification Configuration ===")
message(STATUS "  Version:      ${PROJECT_VERSION}")
message(STATUS "  Build type:   ${CMAKE_BUILD_TYPE}")
message(STATUS "  OpenMP:       ${OpenMP_CXX_FOUND}")
message(STATUS "  Build tests:  ${SPS_BUILD_TESTS}")
message(STATUS "  Build examples: ${SPS_BUILD_EXAMPLES}")
message(STATUS "")
