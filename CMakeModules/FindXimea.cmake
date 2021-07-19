###############################################################################
# Find Ximea camera
#
# This sets the following variables:
# Ximea_FOUND - True if Ximea camera was found.
# Ximea_INCLUDE_DIRS - Directories containing the Ximea camera include files.
# Ximea_LIBRARIES - Libraries needed to use Ximea camera.

find_path(
    Ximea_INCLUDE_DIR xiApi.h
    PATHS
        "${PROGRAM_FILES}/XIMEA/package/include"
        /usr/include
        /user/include
	/opt/XIMEA/include
    PATH_SUFFIXES XIMEA
)

find_library(
    Ximea_LIBRARY
    NAMES xiapi_dng_store
    PATHS
        "${PROGRAM_FILES}/XIMEA/package/libs"
        /usr/lib
        /user/lib
	/opt/XIMEA/lib
    PATH_SUFFIXES ${TELI_PATH_SUFFIXES}
)

set(Ximea_INCLUDE_DIRS ${Ximea_INCLUDE_DIR})
set(Ximea_LIBRARIES ${Ximea_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args( Ximea
  FOUND_VAR Ximea_FOUND
  REQUIRED_VARS Ximea_LIBRARY Ximea_INCLUDE_DIR
)
