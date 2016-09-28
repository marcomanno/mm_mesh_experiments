cmake_minimum_required(VERSION 2.8.5 FATAL_ERROR)

find_package(VTK REQUIRED 
  HINTS "C:/Users/marco/OneDrive/Documents/PROJECTS/ThirdParties/vtk_installRelease")

vtk_module_config(VTK
  vtkCommonCore
  vtkFiltersSources
  vtkInteractionStyle
  vtkRendering${VTK_RENDERING_BACKEND}
)
include(${VTK_USE_FILE})

include_directories (
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)

set (app_name "vtk_cad")
add_executable(${app_name} MACOSX_BUNDLE ${app_name}.cc)
target_link_libraries(${app_name} ${VTK_LIBRARIES} Import PolygonTriangularization Geo Boolean Topology)
