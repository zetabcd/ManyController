#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "NLopt::nlopt_cxx" for configuration "None"
set_property(TARGET NLopt::nlopt_cxx APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(NLopt::nlopt_cxx PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libnlopt_cxx.so.0.11.1"
  IMPORTED_SONAME_NONE "libnlopt_cxx.so.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS NLopt::nlopt_cxx )
list(APPEND _IMPORT_CHECK_FILES_FOR_NLopt::nlopt_cxx "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libnlopt_cxx.so.0.11.1" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
