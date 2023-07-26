cmake_minimum_required(VERSION 3.14)
project(SeidelAlgorithm)

set(CMAKE_CXX_STANDARD 20)

option(ENABLE_INTERNAL_TESTS "" ON)

# file(GLOB_RECURSE SeidelSources "./*.cpp")
# file(GLOB_RECURSE SeidelHeaders "./*.h")

set(SeidelSources
	"vec2.cpp"
	"primitives.cpp"
	"trapezoidMapP.cpp"
	"triangulator.cpp"
)
set(SeidelHeaders 
	"vec2.h"
	"primitives.h"
	"allocator.h"
	"trapezoidMapP.h"
	"triangulator.h"

	"views.natvis"
)

add_library(seidel_triangulator STATIC)
target_sources(seidel_triangulator PRIVATE ${SeidelSources} PUBLIC ${SeidelHeaders})
target_include_directories(seidel_triangulator PUBLIC "./")
target_compile_definitions(seidel_triangulator PUBLIC SEIDEL_USE_SIGNED_INDEX)
if (ENABLE_INTERNAL_TESTS)
	target_compile_definitions(seidel_triangulator PUBLIC SEIDEL_INTERNAL_TEST)
	add_subdirectory(tests)
endif()

add_subdirectory(viewer)