cmake_minimum_required(VERSION 3.14)
project(SeidelAlgorithm)

set(CMAKE_CXX_STANDARD 20)

option(ENABLE_INTERNAL_TESTS "" ON)

# file(GLOB_RECURSE SeidelSources "./*.cpp")
# file(GLOB_RECURSE SeidelHeaders "./*.h")

set(SeidelSources
	"src/seidel/vec2.cpp"
	"src/seidel/primitives.cpp"
	"src/seidel/trapezoidMapP.cpp"
	"src/seidel/triangulator.cpp"
)
set(SeidelHeaders 
	"src/seidel/vec2.h"
	"src/seidel/primitives.h"
	"src/seidel/allocator.h"
	"src/seidel/trapezoidMapP.h"
	"src/seidel/triangulator.h"

	"src/seidel/views.natvis"
)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
	add_compile_definitions(_DEBUG)
endif ()

add_library(seidel_triangulator STATIC)
target_sources(seidel_triangulator PRIVATE ${SeidelSources} PUBLIC ${SeidelHeaders})
target_include_directories(seidel_triangulator PUBLIC "./src" PRIVATE "./src/seidel")
target_compile_definitions(seidel_triangulator PUBLIC SEIDEL_USE_SIGNED_INDEX)

set(FISTSources
	"src/fist/triangulator.cpp"
	"src/fist/vec2.cpp"
)
set(FISTHeader
	"src/fist/vec2.h"
	"src/fist/allocator.h"
	"src/fist/triangulator.h"

	"src/fist/views.natvis"
)

add_library(fist_triangulator STATIC)
target_sources(fist_triangulator PRIVATE ${FISTSources} PUBLIC ${FISTHeader})
target_include_directories(fist_triangulator PUBLIC "./src" PRIVATE "./src/fist")

if (ENABLE_INTERNAL_TESTS)
	target_compile_definitions(seidel_triangulator PUBLIC SEIDEL_INTERNAL_TEST)
	add_subdirectory(tests)
endif ()

add_subdirectory(viewer)