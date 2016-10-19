INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_WIFIRECV wifirecv)

FIND_PATH(
    WIFIRECV_INCLUDE_DIRS
    NAMES wifirecv/api.h
    HINTS $ENV{WIFIRECV_DIR}/include
        ${PC_WIFIRECV_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    WIFIRECV_LIBRARIES
    NAMES gnuradio-wifirecv
    HINTS $ENV{WIFIRECV_DIR}/lib
        ${PC_WIFIRECV_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(WIFIRECV DEFAULT_MSG WIFIRECV_LIBRARIES WIFIRECV_INCLUDE_DIRS)
MARK_AS_ADVANCED(WIFIRECV_LIBRARIES WIFIRECV_INCLUDE_DIRS)

