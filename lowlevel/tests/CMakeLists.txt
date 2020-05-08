if(UNIX)
    set(BUILD_TARGET "TestRunner")

    set(CMAKE_VERBOSE_MAKEFILE TRUE)

    # PROJECT SOURCES
    set(SRC_TEST
            "main.cpp"

            "${SRC_DIR}/utility/BufferPool.hpp"
            "${CMAKE_CURRENT_SOURCE_DIR}/BufferPool/BufferPool_test.cpp"

            "${SRC_DIR}/utility/Average.hpp"
            "${CMAKE_CURRENT_SOURCE_DIR}/Average/Average_test.cpp"

            "${SRC_DIR}/utility/Queue.hpp"
            "${CMAKE_CURRENT_SOURCE_DIR}/Queue/Queue_test.cpp"
            )

    # Find CPPUtest
    find_package(CppUTest)

    if(NOT CPPUTEST_INCLUDE_DIR)
        message(FATAL_ERROR "CPPUTEST NOT FOUND")
    endif()
    # TARGETS
    # # The test executable
    add_executable(${BUILD_TARGET} ${SRC_TEST})
    target_link_libraries(${BUILD_TARGET} PRIVATE CppUTest CppUTestExt)
endif()