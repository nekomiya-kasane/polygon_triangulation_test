cmake_minimum_required(VERSION 3.14)
project(SeidelAlgorithm)

set(CMAKE_CXX_STANDARD 20)

option(ENABLE_INTERNAL_TESTS "" ON)

# file(GLOB_RECURSE SeidelSources "./*.cpp")
# file(GLOB_RECURSE SeidelHeaders "./*.h")

set(SeidelSources
	"src/vec2.cpp"
	"src/primitives.cpp"
	"src/trapezoidMapP.cpp"
	"src/triangulator.cpp"
)
set(SeidelHeaders 
	"src/vec2.h"
	"src/primitives.h"
	"src/allocator.h"
	"src/trapezoidMapP.h"
	"src/triangulator.h"

	"views.natvis"
)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
	add_compile_definitions(_DEBUG)
endif ()

add_library(seidel_triangulator STATIC)
target_sources(seidel_triangulator PRIVATE ${SeidelSources} PUBLIC ${SeidelHeaders})
target_include_directories(seidel_triangulator PUBLIC "./src")
target_compile_definitions(seidel_triangulator PUBLIC SEIDEL_USE_SIGNED_INDEX)
if (MSVC)
  target_compile_options(seidel_triangulator PUBLIC /Zc:__cplusplus)
endif()
if (ENABLE_INTERNAL_TESTS)
	target_compile_definitions(seidel_triangulator PUBLIC SEIDEL_INTERNAL_TEST)
	add_subdirectory(tests)
endif ()

add_subdirectory(viewer)