INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_CSITOOL csitool)

FIND_PATH(
    CSITOOL_INCLUDE_DIRS
    NAMES csitool/api.h
    HINTS $ENV{CSITOOL_DIR}/include
        ${PC_CSITOOL_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    CSITOOL_LIBRARIES
    NAMES gnuradio-csitool
    HINTS $ENV{CSITOOL_DIR}/lib
        ${PC_CSITOOL_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(CSITOOL DEFAULT_MSG CSITOOL_LIBRARIES CSITOOL_INCLUDE_DIRS)
MARK_AS_ADVANCED(CSITOOL_LIBRARIES CSITOOL_INCLUDE_DIRS)

