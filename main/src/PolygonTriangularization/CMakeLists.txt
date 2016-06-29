include (ACGCommon)

include_directories (
  ..
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# source code directories
set (directories . )

# collect all header and source files
acg_append_files (headers "*.hh" ${directories})
acg_append_files (sources "*.cc" ${directories})

#Drop the template only cc files
acg_drop_templates(sources)

acg_add_library (PolygonTriangularization STATIC ${sources} ${headers})


