cmake_minimum_required(VERSION 3.1)
project(Vanetza VERSION 0.8)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_BUILD_TYPE "Debug")
set(Boost_DEBUG ON)
set(Boost_USE_STATIC_LIBS ON)
#set(Boost_COMPILER "-mgw11")
#set(Boost_ARCHITECTURE "-x64")
set(BUILD_TESTS OFF)
set(BUILD_SOCKTAP ON)
set(BUILD_BENCHMARK ON)
set(BUILD_CERTIFY ON)
set(BUILD_ITS_BLE ON)
set(ITS_BLE_LOG OFF)

include(CMakePrintHelpers)

set(CMAKE_VERBOSE_MAKEFILE ON)

#set(CMAKE_C_COMPILER "C:/cygwin64/bin/gcc.exe")
#set(CMAKE_CXX_COMPILER "C:/cygwin64/bin/g++.exe")
#set(CMAKE_CXX_COMPILER "C:/msys64/mingw64/bin/g++.exe")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__USE_W32_SOCKETS -D_WIN32_WINNT=0x0A00")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lws2_32")


#include_directories("C:/ProgramData/chocolatey/lib/zig/tools/zig-windows-x86_64-0.9.1/lib/libc/include/generic-glibc" 
#					"C:/ProgramData/chocolatey/lib/zig/tools/zig-windows-x86_64-0.9.1/lib/libc/include/any-linux-any"
#					"C:/ProgramData/chocolatey/lib/zig/tools/zig-windows-x86_64-0.9.1/lib/libc/include/arc-linux-any"
#					"C:/Program Files/vanetza-master-v2"
#					"C:/Users/marie/Documents/COLE/UNIVERSIDAD/Cuarto/TFG/Librerias/boost_1_78_0/boost_1_78_0")

# enable convenient CMake policies if available
if(POLICY CMP0074)
  # find_package() employs <package>_ROOT (since CMake 3.12)
  cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0077)
  # option() honours normal variable (since CMake 3.13)
  cmake_policy(SET CMP0077 NEW)
endif()

# Vanetza makes extensive use of C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Load conan configuration if available
if(EXISTS ${PROJECT_BINARY_DIR}/conanbuildinfo.cmake)
    include(${PROJECT_BINARY_DIR}/conanbuildinfo.cmake)
    conan_basic_setup(TARGETS)
    set(Boost_NO_SYSTEM_PATHS ON)
endif()

# Look up threading library (usually pthread)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads MODULE)

# Build configuration options
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

option(BUILD_TESTS "Build unit tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(gtest)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib/static)

option(VANETZA_INSTALL "Generate install targets for Vanetza" ON)
include(GNUInstallDirs)
include(UseVanetza)

# Enable usage of targets' folder property (good for Visual Studio)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Find project dependencies
set(Boost_COMPONENTS date_time) # used by VanetzaConfig generator
find_package(Boost 1.78.0 REQUIRED COMPONENTS ${Boost_COMPONENTS})
# query optional Boost components one by one quietly
foreach(component IN ITEMS program_options system)
  find_package(Boost 1.78.0 QUIET COMPONENTS ${component})
  if (Boost_${component}_FOUND)
    list(APPEND Boost_COMPONENTS ${component})
  endif()
endforeach()
include(CompatBoostTargets)
find_package(CryptoPP 5.6.1 REQUIRED)
find_package(GeographicLib 1.37 REQUIRED)
find_package(Cohda MODULE QUIET)

option(VANETZA_WITH_OPENSSL "Enable OpenSSL extensions" OFF)
if (VANETZA_WITH_OPENSSL)
  find_package(OpenSSL 1.0.1 REQUIRED)
  include(CompatOpenSSLTargets)
endif()

add_subdirectory(vanetza/access)
add_subdirectory(vanetza/asn1)
add_subdirectory(vanetza/btp)
add_subdirectory(vanetza/common)
add_subdirectory(vanetza/dcc)
add_subdirectory(vanetza/facilities)
add_subdirectory(vanetza/geonet)
add_subdirectory(vanetza/gnss)
add_subdirectory(vanetza/net)
add_subdirectory(vanetza/security)
add_subdirectory(vanetza/units)
add_subdirectory(tools/proxy_fake_feed)

if(TARGET Cohda::headers)
  add_subdirectory(tools/cohda_proxy)
endif()

option(BUILD_SOCKTAP "Build socktap application" OFF)
if(BUILD_SOCKTAP)
  add_subdirectory(tools/socktap)
endif()

option(BUILD_ITS_BLE "Build its_ble application" OFF)
if(BUILD_ITS_BLE)
  add_subdirectory(tools/its_ble)
endif()

option(ITS_BLE_LOG "its_ble: Enable log" ON)
if(ITS_BLE_LOG)
    target_compile_definitions(its_ble PUBLIC "ITS_BLE_LOG")
endif()

option(BUILD_CERTIFY "Build certify application" OFF)
if(BUILD_CERTIFY)
  add_subdirectory(tools/certify)
endif()

option(BUILD_BENCHMARK "Build benchmark application" OFF)
if(BUILD_BENCHMARK)
  add_subdirectory(tools/benchmark)
endif()

# interface library for convenience
get_property(_components GLOBAL PROPERTY VANETZA_COMPONENTS)
add_library(vanetza INTERFACE)
add_library(Vanetza::vanetza ALIAS vanetza)
foreach(_component IN LISTS _components)
  target_link_libraries(vanetza INTERFACE ${_component})
endforeach()



# installation rules
include(CMakePackageConfigHelpers)
set(CMAKECONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/Vanetza")
set(CMAKECONFIG_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/cmake-config")
write_basic_package_version_file(${CMAKECONFIG_BUILD_DIR}/VanetzaConfigVersion.cmake
  COMPATIBILITY ExactVersion)
configure_package_config_file(cmake/VanetzaConfig.cmake.in
  ${CMAKECONFIG_BUILD_DIR}/VanetzaConfig.cmake
  INSTALL_DESTINATION ${CMAKECONFIG_INSTALL_DIR}
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR)

if(VANETZA_INSTALL)
  install(TARGETS vanetza EXPORT ${PROJECT_NAME})

  install(FILES
    ${CMAKECONFIG_BUILD_DIR}/VanetzaConfig.cmake
    ${CMAKECONFIG_BUILD_DIR}/VanetzaConfigVersion.cmake
    DESTINATION ${CMAKECONFIG_INSTALL_DIR})

  install(DIRECTORY ${PROJECT_SOURCE_DIR}/cmake/
    DESTINATION ${CMAKECONFIG_INSTALL_DIR}
    FILES_MATCHING
    PATTERN "MacroFindDependencyComponents.cmake"
    PATTERN "Find*.cmake"
    PATTERN "Compat*Targets.cmake")

  install(EXPORT ${PROJECT_NAME} NAMESPACE Vanetza:: FILE VanetzaTargets.cmake DESTINATION ${CMAKECONFIG_INSTALL_DIR})

  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/vanetza
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "/tests/*.hpp" EXCLUDE)

  # export build tree (allows import by outside projects)
  option(VANETZA_EXPORT_PACKAGE "Export Vanetza build directory to CMake package registry" ON)
  if(VANETZA_EXPORT_PACKAGE)
    export(EXPORT ${PROJECT_NAME} NAMESPACE Vanetza:: FILE VanetzaExports.cmake)
    file(COPY ${CMAKECONFIG_BUILD_DIR}/VanetzaConfigVersion.cmake
      DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    configure_file(cmake/VanetzaExportsConfig.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/VanetzaConfig.cmake @ONLY)
    export(PACKAGE ${PROJECT_NAME})
  endif()
endif(VANETZA_INSTALL)
