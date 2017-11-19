
project (MeshBooleanApp)

file(GLOB sources "*.cc")
include_directories (..)

add_executable (MeshBooleanApp ${sources})

target_link_libraries (MeshBooleanApp LINK_PUBLIC 
  Base PolygonTriangularization 
  Geo Boolean Import Topology Utils)

# Set output directory to ${BINARY_DIR}/Unittests
set (OUTPUT_DIR "${CMAKE_BINARY_DIR}/MeshBooleanApp")
set_target_properties(MeshBooleanApp PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR}) 
