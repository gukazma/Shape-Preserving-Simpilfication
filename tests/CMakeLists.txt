if(SPS_BUILD_TESTS)
    # Find GTest or use simple tests
    find_package(GTest QUIET)

    if(GTest_FOUND)
        enable_testing()
        add_executable(sps_tests test_basic.cpp)
        target_link_libraries(sps_tests sps_lib GTest::gtest_main)
        add_test(NAME sps_tests COMMAND sps_tests)
    else()
        # Simple test without GTest
        add_executable(sps_test_basic test_basic.cpp)
        target_link_libraries(sps_test_basic sps_lib)
    endif()
endif()
