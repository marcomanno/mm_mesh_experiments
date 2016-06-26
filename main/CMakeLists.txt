cmake_minimum_required (VERSION 2.6)

enable_testing()

if (NOT WIN32 AND NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

# add our macro directory to cmake search path
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set (CMAKE_DEBUG_POSTFIX "d")

# include our cmake files
include (ACGCommon)

acg_get_version ()


# Disable package building when built as an external library
if(${PROJECT_NAME} MATCHES "OpenMesh")
    include(OpenMeshPackage)
endif()

include (ACGOutput)

# ========================================================================
# Definitions
# ========================================================================

if (WIN32)
  add_definitions(
      -D_USE_MATH_DEFINES -DNOMINMAX
      -D_CRT_SECURE_NO_WARNINGS
  )
endif ()

set(NO_DECREMENT_DEPRECATED_WARNINGS OFF CACHE BOOL "Disables all deprecated warnings warning about decrement operations on normal circulators.")
if(NO_DECREMENT_DEPRECATED_WARNINGS)
  add_definitions( -DNO_DECREMENT_DEPRECATED_WARNINGS )
endif()

# ========================================================================
# Windows build style control
# ========================================================================

if ( WIN32 )
  if ( NOT DEFINED OPENMESH_BUILD_SHARED )
    set( OPENMESH_BUILD_SHARED false CACHE BOOL "Build as shared library(DLL)?" )
  endif()
endif()

# ========================================================================
# Call the subdirectories with there projects
# ========================================================================

add_subdirectory (src/PolygonTriangularization)
add_subdirectory (src/Geo)
add_subdirectory (src/UnitTest)
# ========================================================================
