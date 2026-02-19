#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "velodyne_driver::velodyne_input" for configuration ""
set_property(TARGET velodyne_driver::velodyne_input APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(velodyne_driver::velodyne_input PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libvelodyne_input.so"
  IMPORTED_SONAME_NOCONFIG "libvelodyne_input.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS velodyne_driver::velodyne_input )
list(APPEND _IMPORT_CHECK_FILES_FOR_velodyne_driver::velodyne_input "${_IMPORT_PREFIX}/lib/libvelodyne_input.so" )

# Import target "velodyne_driver::velodyne_driver" for configuration ""
set_property(TARGET velodyne_driver::velodyne_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(velodyne_driver::velodyne_driver PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "tf2_ros::tf2_ros"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libvelodyne_driver.so"
  IMPORTED_SONAME_NOCONFIG "libvelodyne_driver.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS velodyne_driver::velodyne_driver )
list(APPEND _IMPORT_CHECK_FILES_FOR_velodyne_driver::velodyne_driver "${_IMPORT_PREFIX}/lib/libvelodyne_driver.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
