include (ACGCommon)

include_directories (
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  C:/Users/marco/OneDrive/Documents/PROJECTS/ThirdParties/eigen-eigen-5a0156e40feb
  C:/Users/marco/OneDrive/Documents/PROJECTS/ThirdParties/boost_1_65_1
)

# source code directories
set (directories . )

# collect all header and source files
acg_append_files (headers "*.hh" ${directories})
acg_append_files (sources "*.cc" ${directories})

#Drop the template only cc files
acg_drop_templates(sources)

acg_add_library (Utils STATIC ${sources} ${headers})


