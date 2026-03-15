#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "patchworkpp::ground_seg_cores" for configuration "Release"
set_property(TARGET patchworkpp::ground_seg_cores APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(patchworkpp::ground_seg_cores PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libground_seg_cores.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS patchworkpp::ground_seg_cores )
list(APPEND _IMPORT_CHECK_FILES_FOR_patchworkpp::ground_seg_cores "${_IMPORT_PREFIX}/lib/libground_seg_cores.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
