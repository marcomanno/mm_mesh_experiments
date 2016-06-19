include (ACGCommon)

include_directories (
  ..
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Create new target named unittests_hexmeshing
FILE(GLOB UNITTEST_SRC *.cc)
# Create unittest executable
acg_add_executable(unittests ${UNITTEST_SRC})

# Set output directory to ${BINARY_DIR}/Unittests
set (OUTPUT_DIR "${CMAKE_BINARY_DIR}/Unittests")
set_target_properties(unittests PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR}) 

#target_link_libraries(unittests OpenMeshCore OpenMeshTools ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}) 
