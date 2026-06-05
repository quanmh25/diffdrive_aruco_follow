# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_follower_scene_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED follower_scene_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(follower_scene_FOUND FALSE)
  elseif(NOT follower_scene_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(follower_scene_FOUND FALSE)
  endif()
  return()
endif()
set(_follower_scene_CONFIG_INCLUDED TRUE)

# output package information
if(NOT follower_scene_FIND_QUIETLY)
  message(STATUS "Found follower_scene: 0.0.0 (${follower_scene_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'follower_scene' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT follower_scene_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(follower_scene_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${follower_scene_DIR}/${_extra}")
endforeach()
