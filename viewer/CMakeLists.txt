
include(FetchContent)
FetchContent_Declare(
  raylib
  URL https://github.com/raysan5/raylib/archive/refs/tags/4.5.0.zip
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
)
FetchContent_MakeAvailable(raylib)

FetchContent_Declare(
  raygui
  URL https://github.com/raysan5/raygui/archive/refs/tags/3.6.zip
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
  SOURCE_SUBDIR projects/CMake
)
FetchContent_MakeAvailable(raygui)

# add_library(easyx::easyxa STATIC IMPORTED GLOBAL)
# set_property(TARGET easyx::easyxa PROPERTY IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/VC2015/x64/EasyXa.lib")
# set_property(TARGET easyx::easyxa PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")
# set_property(TARGET easyx::easyxa PROPERTY INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")
# 
# add_library(easyx::easyxw STATIC IMPORTED GLOBAL)
# set_property(TARGET easyx::easyxa PROPERTY IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/VC2015/x64/EasyXa.lib")
# set_property(TARGET easyx::easyxa PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")
# set_property(TARGET easyx::easyxa PROPERTY INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")

set(ViewerSources 
	"src/window.cpp"
	"src/viewable.h"
	"src/viewable.cpp"
)

add_executable(seidel_triangulation_viewer ${ViewerSources})
target_link_libraries(
  seidel_triangulation_viewer PRIVATE
  seidel_triangulator
  raylib
  raygui
)