# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_erp42_cmake_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED erp42_cmake_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(erp42_cmake_FOUND FALSE)
  elseif(NOT erp42_cmake_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(erp42_cmake_FOUND FALSE)
  endif()
  return()
endif()
set(_erp42_cmake_CONFIG_INCLUDED TRUE)

# output package information
if(NOT erp42_cmake_FIND_QUIETLY)
  message(STATUS "Found erp42_cmake: 0.0.0 (${erp42_cmake_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'erp42_cmake' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${erp42_cmake_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(erp42_cmake_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "erp42_cmake_config_extras.cmake")
foreach(_extra ${_extras})
  include("${erp42_cmake_DIR}/${_extra}")
endforeach()
