#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "vineslam::vineslam" for configuration "Release"
set_property(TARGET vineslam::vineslam APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vineslam::vineslam PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libvineslam.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS vineslam::vineslam )
list(APPEND _IMPORT_CHECK_FILES_FOR_vineslam::vineslam "${_IMPORT_PREFIX}/lib/libvineslam.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
