cmake_minimum_required(VERSION 3.14)
project(SeidelAlgorithm)

set(CMAKE_CXX_STANDARD 20)

# file(GLOB_RECURSE SeidelSources "./*.cpp")
# file(GLOB_RECURSE SeidelHeaders "./*.h")

set(SeidelSources
	"vec2.cpp"
	"trapezoidMapP.cpp"
	"triangulator.cpp"
)
set(SeidelHeaders 
	"vec2.h"
	"primitives.h"
	"allocator.h"
	"trapezoidMapP.h"
	"triangulator.h"

	"allocator.natvis"
)

add_library(seidel_triangulator STATIC)
target_sources(seidel_triangulator PRIVATE ${SeidelSources} PUBLIC ${SeidelHeaders})
target_include_directories(seidel_triangulator PUBLIC "./")

#file(GLOB_RECURSE SeidelTestSources "Tests/*.h" "Tests/*.cpp")
#add_executable(seidel_test ${SeidelTestSources})
#target_link_libraries(seidel_test PRIVATE seidel gtest)

add_subdirectory(tests)