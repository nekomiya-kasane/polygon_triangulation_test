set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(TestSources
	"basic.cpp"
	"ptptco.cpp"
)

enable_testing()

add_executable(seidel_triangulation_test ${TestSources})
target_link_libraries(
  seidel_triangulation_test PRIVATE
  seidel_triangulator
  GTest::gtest_main
)

add_test(NAME static_test COMMAND seidel_triangulation_test)