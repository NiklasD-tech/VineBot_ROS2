#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ublox_msgs::ublox_msgs__rosidl_typesupport_c" for configuration "Release"
set_property(TARGET ublox_msgs::ublox_msgs__rosidl_typesupport_c APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ublox_msgs::ublox_msgs__rosidl_typesupport_c PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libublox_msgs__rosidl_typesupport_c.so"
  IMPORTED_SONAME_RELEASE "libublox_msgs__rosidl_typesupport_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ublox_msgs::ublox_msgs__rosidl_typesupport_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_ublox_msgs::ublox_msgs__rosidl_typesupport_c "${_IMPORT_PREFIX}/lib/libublox_msgs__rosidl_typesupport_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
