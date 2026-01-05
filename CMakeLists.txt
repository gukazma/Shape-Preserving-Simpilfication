cmake_minimum_required(VERSION 3.20)

# Project name
project(ShapePreservingSimplification
    VERSION 2.0.0
    LANGUAGES CXX
    DESCRIPTION "Shape-Preserving Mesh Simplification using CGAL"
)

# =============================================================================
# Global Settings
# =============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_BINARY_DIR}/bin)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
if(MSVC)
    add_compile_options(/W4 /permissive-)
    add_compile_definitions(_USE_MATH_DEFINES NOMINMAX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# Options
# =============================================================================
option(SPS_BUILD_TESTS "Build unit tests" ON)
option(SPS_BUILD_EXAMPLES "Build examples" ON)
option(SPS_USE_OPENMP "Use OpenMP for parallelization" ON)

# =============================================================================
# Dependencies (via vcpkg)
# =============================================================================
find_package(Boost REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(CGAL REQUIRED)

message(STATUS "Eigen3 found: ${EIGEN3_VERSION}")
message(STATUS "CGAL found: ${CGAL_VERSION}")

# OpenMP (optional)
if(SPS_USE_OPENMP)
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        message(STATUS "OpenMP found, parallel processing enabled")
    endif()
endif()

# =============================================================================
# Include directories
# =============================================================================
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/third_party)

# =============================================================================
# Collect source files
# =============================================================================
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

# =============================================================================
# Library
# =============================================================================
add_library(sps_lib STATIC ${LIB_SOURCES})

target_link_libraries(sps_lib PUBLIC
    CGAL::CGAL
    Eigen3::Eigen
    Boost::boost
)

# Link CGAL Eigen3 support if available
if(TARGET CGAL::Eigen3_support)
    target_link_libraries(sps_lib PUBLIC CGAL::Eigen3_support)
endif()

if(OpenMP_CXX_FOUND)
    target_link_libraries(sps_lib PUBLIC OpenMP::OpenMP_CXX)
    target_compile_definitions(sps_lib PUBLIC SPS_USE_OPENMP)
endif()

# =============================================================================
# Main executable
# =============================================================================
add_executable(sps_simplify src/main.cpp)
target_link_libraries(sps_simplify PRIVATE sps_lib)

# =============================================================================
# Tests
# =============================================================================
if(SPS_BUILD_TESTS)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
        enable_testing()
        add_subdirectory(tests)
    endif()
endif()

# =============================================================================
# Examples
# =============================================================================
if(SPS_BUILD_EXAMPLES)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/CMakeLists.txt")
        add_subdirectory(examples)
    endif()
endif()

# =============================================================================
# Install
# =============================================================================
install(TARGETS sps_simplify sps_lib
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY include/sps DESTINATION include)

# =============================================================================
# Set startup project (Visual Studio)
# =============================================================================
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT sps_simplify)

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "=== Shape-Preserving Simplification Configuration ===")
message(STATUS "  Version:       ${PROJECT_VERSION}")
message(STATUS "  Build type:    ${CMAKE_BUILD_TYPE}")
message(STATUS "  OpenMP:        ${OpenMP_CXX_FOUND}")
message(STATUS "  Build tests:   ${SPS_BUILD_TESTS}")
message(STATUS "  Build examples: ${SPS_BUILD_EXAMPLES}")
message(STATUS "")
