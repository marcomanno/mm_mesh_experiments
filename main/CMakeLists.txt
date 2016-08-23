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
# Warnings
# ========================================================================

if(MSVC)
  if(CMAKE_CXX_FLAGS MATCHES "/WX-")
    string(REGEX REPLACE "/WX-" "/WX" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /WX")
  endif()
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
endif()

# ========================================================================
# Call the subdirectories with there projects
# ========================================================================

add_subdirectory (src/Boolean)
add_subdirectory (src/Geo)
add_subdirectory (src/PolygonTriangularization)
add_subdirectory (src/Topology)
add_subdirectory (src/UnitTest)
add_subdirectory (src/Utils)
add_subdirectory (src/App)
# ========================================================================
