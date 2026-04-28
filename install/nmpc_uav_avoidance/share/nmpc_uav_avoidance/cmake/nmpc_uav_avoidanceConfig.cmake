# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_nmpc_uav_avoidance_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED nmpc_uav_avoidance_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(nmpc_uav_avoidance_FOUND FALSE)
  elseif(NOT nmpc_uav_avoidance_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(nmpc_uav_avoidance_FOUND FALSE)
  endif()
  return()
endif()
set(_nmpc_uav_avoidance_CONFIG_INCLUDED TRUE)

# output package information
if(NOT nmpc_uav_avoidance_FIND_QUIETLY)
  message(STATUS "Found nmpc_uav_avoidance: 1.0.0 (${nmpc_uav_avoidance_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'nmpc_uav_avoidance' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${nmpc_uav_avoidance_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(nmpc_uav_avoidance_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${nmpc_uav_avoidance_DIR}/${_extra}")
endforeach()
