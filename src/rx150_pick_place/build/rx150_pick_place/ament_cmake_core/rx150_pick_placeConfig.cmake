# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rx150_pick_place_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rx150_pick_place_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rx150_pick_place_FOUND FALSE)
  elseif(NOT rx150_pick_place_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rx150_pick_place_FOUND FALSE)
  endif()
  return()
endif()
set(_rx150_pick_place_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rx150_pick_place_FIND_QUIETLY)
  message(STATUS "Found rx150_pick_place: 0.2.0 (${rx150_pick_place_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rx150_pick_place' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rx150_pick_place_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rx150_pick_place_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rx150_pick_place_DIR}/${_extra}")
endforeach()
