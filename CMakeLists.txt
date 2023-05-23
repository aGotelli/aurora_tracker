cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)



project(aurora VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)

#   External utilities to configure the package
include(GNUInstallDirs)
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})


#   Give different names for debug and Release
set(CMAKE_RELEASE_POSTFIX "")
set(CMAKE_DEBUG_POSTFIX "-debug")



#
# Dependencies
#
find_package(mpi_cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(real_time_tools REQUIRED)


find_package(math_tools QUIET)

if(math_tools_FOUND)
    message(STATUS "math_tools found")
else()
    message(STATUS "math_tools not found")
endif()




# Aurora Tracker
set(VTK_DIR "/usr/local/include/vtk-8.2")
set(NDICAPI_DIR "/usr/local/lib/libndicapi.so")
  
find_package(ndicapi QUIET)
find_package(VTK REQUIRED)



add_subdirectory(src)



#   Install the library using the default routine
include(cmake/installation_module.cmake)

