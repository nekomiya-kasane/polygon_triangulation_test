cmake_minimum_required(VERSION 3.14)
project(SeidelAlgorithm)

set(CMAKE_CXX_STANDARD 17)

file(GLOB_RECURSE SeidelSources "./*.cpp")
file(GLOB_RECURSE SeidelHeaders "./*.h")

add_library(seidel STATIC)
target_sources(seidel PRIVATE ${SeidelSources} ${SeidelHeaders})
target_include_directories(seidel PUBLIC "./Seidel")

#file(GLOB_RECURSE SeidelTestSources "Tests/*.h" "Tests/*.cpp")
#add_executable(seidel_test ${SeidelTestSources})
#target_link_libraries(seidel_test PRIVATE seidel gtest)