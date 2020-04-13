cmake_minimum_required(VERSION 3.12)
project(raytracing LANGUAGES CXX C)

set(PROJECT_ROOT "${PROJECT_SOURCE_DIR}")

include(FetchContent)
FetchContent_Declare(
    atlas
    GIT_REPOSITORY https://github.com/marovira/atlas.git)
FetchContent_Populate(atlas)
add_subdirectory(${atlas_SOURCE_DIR} ${atlas_BINARY_DIR})

set(PROJECT_INCLUDE
    "${PROJECT_ROOT}/raytracer.hpp"
    )
set(PROJECT_SOURCE 
    "${PROJECT_ROOT}/raytracer.cpp"
    )

set(PATH_INCLUDE "${PROJECT_ROOT}/paths.hpp")
configure_file("${PROJECT_ROOT}/paths.hpp.in" ${PATH_INCLUDE})
list(APPEND PROJECT_INCLUDE ${PATH_INCLUDE})

source_group("include" FILES ${PROJECT_INCLUDE})
source_group("source" FILES ${PROJECT_SOURCE})

add_executable(raytracer ${PROJECT_INCLUDE} ${PROJECT_SOURCE} ${PROJECT_SHADER})
target_link_libraries(raytracer PUBLIC atlas::atlas)
