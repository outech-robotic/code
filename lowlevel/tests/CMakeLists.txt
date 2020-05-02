if(UNIX)
    set(BUILD_TARGET "TestRunner")

    # PROJECT SOURCES
    file(GLOB_RECURSE SRC_TEST
            "../com/*.c"
            "../com/*.cpp"
            "../utility/*.c"
            "../utility/*.cpp"
            )

    # Find CPPUtest
    find_package(CppUTest)


    # TARGETS
    # # The test executable
    add_executable(${BUILD_TARGET} main.cpp)
    target_link_libraries(${BUILD_TARGET} PRIVATE CppUTest CppUTestExt)
endif()