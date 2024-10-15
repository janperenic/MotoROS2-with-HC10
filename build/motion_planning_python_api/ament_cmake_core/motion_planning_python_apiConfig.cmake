# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_motion_planning_python_api_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED motion_planning_python_api_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(motion_planning_python_api_FOUND FALSE)
  elseif(NOT motion_planning_python_api_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(motion_planning_python_api_FOUND FALSE)
  endif()
  return()
endif()
set(_motion_planning_python_api_CONFIG_INCLUDED TRUE)

# output package information
if(NOT motion_planning_python_api_FIND_QUIETLY)
  message(STATUS "Found motion_planning_python_api: 0.0.1 (${motion_planning_python_api_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'motion_planning_python_api' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${motion_planning_python_api_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(motion_planning_python_api_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${motion_planning_python_api_DIR}/${_extra}")
endforeach()