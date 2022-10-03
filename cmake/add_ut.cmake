function(add_ut)
    cmake_parse_arguments(
        PARSED_ARGS # prefix
        "" # boolean arguments
        "TEST_NAME" # mono-valued arguments
        "TEST_SRCS" # multi-valued arguments
        ${ARGN} # arguments
    )

    add_executable(${PARSED_ARGS_TEST_NAME})

    target_sources(${PARSED_ARGS_TEST_NAME}
        PRIVATE
            main.cpp
            test_path.hpp
            ${PARSED_ARGS_TEST_SRCS}
    )

    target_include_directories(${PARSED_ARGS_TEST_NAME}
        PRIVATE
            ${GTest_INCLUDE_DIRS}
            ${Eigen3_INCLUDE_DIRS}
    )

    target_link_libraries(${PARSED_ARGS_TEST_NAME} 
        PRIVATE
            algorithm
            ${GTest_LIBRARIES}
    )
    add_test(${PARSED_ARGS_TEST_NAME} ${PARSED_ARGS_TEST_NAME})
    
    set_target_properties(${PARSED_ARGS_TEST_NAME} PROPERTIES FOLDER "tests")

    target_compile_definitions(${PARSED_ARGS_TEST_NAME} 
        PRIVATE
            TEST_DATA_DIR="${TEST_DATA_DIR}"
    )

endfunction(add_ut)