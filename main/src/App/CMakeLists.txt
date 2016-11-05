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

# source code directories
set (directories . )

# collect all header and source files
acg_append_files (headers "*.hh" ${directories})
acg_append_files (sources "*.cc" ${directories})

#Drop the template only cc files
acg_drop_templates(sources)

set (app_name "vtk_cad")
add_executable(${app_name} MACOSX_BUNDLE ${sources} ${headers})
MESSAGE (STATUS "${VTK_LIBRARIES}")
target_link_libraries(${app_name} ${VTK_LIBRARIES} vtkChartsCore vtkViewsContext2D Import PolygonTriangularization Geo Boolean Topology)
