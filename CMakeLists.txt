cmake_minimum_required(VERSION 3.0)
project(Datastructures)

set(CMAKE_CXX_STANDARD 17)

add_executable(Datastructures main.cpp
        MyGraph.cpp
        MyGraph.h)

# Ensure the "maps" directory exists in the source tree
if(EXISTS "${CMAKE_SOURCE_DIR}/maps/tiny.graph")
    # Copy the "maps" directory to the build directory
    add_custom_command(
            TARGET Datastructures POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/maps $<TARGET_FILE_DIR:Datastructures>/maps
    )
endif()