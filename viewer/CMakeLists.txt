
add_library(easyx::easyxa STATIC IMPORTED GLOBAL)
set_property(TARGET easyx::easyxa PROPERTY IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/VC2015/x64/EasyXa.lib")
set_property(TARGET easyx::easyxa PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")
set_property(TARGET easyx::easyxa PROPERTY INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(easyx::easyxw STATIC IMPORTED GLOBAL)
set_property(TARGET easyx::easyxa PROPERTY IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/VC2015/x64/EasyXa.lib")
set_property(TARGET easyx::easyxa PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")
set_property(TARGET easyx::easyxa PROPERTY INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")

set(ViewerSources 
	"src/window.cpp"
	"src/viewable.h"
	"src/viewable.cpp"
)

add_executable(seidel_triangulation_viewer ${ViewerSources})
target_link_libraries(
  seidel_triangulation_viewer PRIVATE
  seidel_triangulator
  easyx::easyxa
)