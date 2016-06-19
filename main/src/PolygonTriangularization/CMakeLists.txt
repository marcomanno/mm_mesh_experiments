include (ACGCommon)

include_directories (
  ../..
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# source code directories
set (directories . )

# collect all header and source files
acg_append_files (headers "*.hh" ${directories})
acg_append_files (sources "*.cc" ${directories})

#Drop the template only cc files
acg_drop_templates(sources)


if (WIN32 AND NOT MINGW)
  acg_add_library (PolygonTriangularization STATIC ${sources} ${headers})
else ()
  acg_add_library (OpenMeshCore SHAREDANDSTATIC ${sources} ${headers})
  set_target_properties (OpenMeshCore PROPERTIES VERSION ${OPENMESH_VERSION_MAJOR}.${OPENMESH_VERSION_MINOR}
                                               SOVERSION ${OPENMESH_VERSION_MAJOR}.${OPENMESH_VERSION_MINOR} )
endif ()


