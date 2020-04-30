set(BUILD_TARGET "TestRunner")

# Threads required for CppUTests, pthread required
find_package(Threads REQUIRED)

# Find CPPUtest
find_library(CPPUTEST CppUTest "${CPPUTEST_PATH}/lib")
if(CPPUTEST)
    message("-- Found CppUTest Library")
else()
    message("-- CPPUTEST NOT FOUND")
endif()

find_library(CPPUTESTEXT CppUTestExt "${CPPUTEST_PATH}/lib")
if(CPPUTESTEXT)
    message("-- Found CppUTestExt Library")
else()
    message("-- CPPUTESTEXT NOT FOUND")
endif()

include_directories(${CPPUTEST_PATH}/include)

# PROJECT SOURCES
file(GLOB_RECURSE SRC_TEST
        "../com/*.c"
        "../com/*.cpp"
        "../utility/*.c"
        "../utility/*.cpp"
        )

# TARGETS
# # The test executable
add_executable(${BUILD_TARGET} main.cpp)
target_link_libraries(${BUILD_TARGET} PRIVATE ${CPPUTEST} ${CPPUTESTEXT} Threads::Threads)
