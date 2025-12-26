if(SPS_BUILD_EXAMPLES)
    add_executable(example_simplify example_simplify.cpp)
    target_link_libraries(example_simplify sps_lib)
endif()
