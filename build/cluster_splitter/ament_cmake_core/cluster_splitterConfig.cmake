# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cluster_splitter_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cluster_splitter_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cluster_splitter_FOUND FALSE)
  elseif(NOT cluster_splitter_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cluster_splitter_FOUND FALSE)
  endif()
  return()
endif()
set(_cluster_splitter_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cluster_splitter_FIND_QUIETLY)
  message(STATUS "Found cluster_splitter: 0.0.0 (${cluster_splitter_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cluster_splitter' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${cluster_splitter_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cluster_splitter_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${cluster_splitter_DIR}/${_extra}")
endforeach()
