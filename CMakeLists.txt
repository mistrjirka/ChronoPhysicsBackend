#--------------------------------------------------------------
# CMake configuration script for linking an external project to
# Chrono libraries.
# For a particular user project, modify sections 1 - 4 below,
# as appropriate.
#--------------------------------------------------------------
 
cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW)
if(DEFINED CHRONO_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD ${CHRONO_CXX_STANDARD})
else()
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#--------------------------------------------------------------
# 1. Set the project name
#--------------------------------------------------------------

project(ChronoBackend)

#--------------------------------------------------------------
# 2. Find REQUIRED and OPTIONAL Chrono modules
#
# Invoke the find_package function in CONFIG mode:
#    find_package(Chrono
#                 COMPONENTS req_module1 req_module1 ...
#                 OPTIONAL_COMPONENTS opt_module1 opt_module2 ...
#                 CONFIG)
# The following Chrono modules can be requested (case insensitive):
#   Cascade, CSharp, FMI, FSI, GPU, Irrlicht, OpenGL, VSG, Matlab,
#   Modal, Multicore, PardisoMKL, Parsers, Postprocess, Sensor,
#   Synchrono, Vehicle, VehicleCosim.
# A component can be requested either as required or optional
# (see the CMake documentation for find_package).
# 
# Note that you will have to set the variable Chrono_DIR to 
# specify the location of the chrono-config.cmake script, if
# it is not in a default install location. Chrono_DIR can point
# to either a Chrono build tree or a Chrono install tree.
# 
# The following variables are set and can be used further down:
# Chrono_FOUND
#       set to true if Chrono and all required components were found
# CHRONO_STATIC
#       set to ON if Chrono has been built as static library, OFF if shared library
# CHRONO_TARGETS
#       list of exported Chrono targets
# CHRONO_DLL_NAMES
#       list of Chrono DLLs (without path)
# CHRONO_DEPENDENCY_DLLS
#       list of Chrono dependency DLLs (with full path)
# CHRONO_CSHARP_SOURCES
#       list of all SWIG-generated C# scripts corresponding to the requested components
#       (currently, only the core, postprocess, Irrlicht, and Vehicle Chrono modules are wrapped for C#)
# CHRONO_DATA_DIR
#       path to the Chrono data directory
# CHRONO_VEHICLE_DATA_DIR
#       path to the Chrono::Vehicle data directory
# CHRONO_FSI_DATA_DIR
#       path to the Chrono::FSI data directory
# SYNCHRONO_DATA_DIR
#       path to the Chrono::Synchrono data directory
#
# In addition, for each requested component [COMPONENT], the following variable
# is set to true (ON) or false (OFF):
# CHRONO_[COMPONENT]_FOUND
# 
# In this example, we only request the Irrlicht module (required)
# and, for demonstration purposes, the PardisoMKL module (optional)
#--------------------------------------------------------------

find_package(Chrono
             COMPONENTS Irrlicht Vehicle 
             OPTIONAL_COMPONENTS PardisoMKL
             CONFIG)

# Find Eigen library
find_package(Eigen3 REQUIRED)

# Locate Irrlicht library manually
find_library(IRRLICHT_LIBRARY NAMES Irrlicht PATHS /usr/lib)
find_path(IRRLICHT_INCLUDE_DIR NAMES irrlicht.h PATHS /usr/include/irrlicht/)

# Return now if Chrono or a required component was not found.
if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

#eigen
if(NOT Eigen3_FOUND)
    message(FATAL_ERROR "Eigen3 not found. Please install it.")
endif()

#irrlicht
#if(NOT Irrlicht_FOUND)
#    message(FATAL_ERROR "Irrlicht not found. Please install it.")
#endif()

if (NOT IRRLICHT_LIBRARY OR NOT IRRLICHT_INCLUDE_DIR)
    message(FATAL_ERROR "Irrlicht not found. Please specify the correct path.")
endif()
find_package(Bullet REQUIRED)
#--------------------------------------------------------------
# Enable creation of "application bundles" on MacOSX.
#--------------------------------------------------------------

# This is necessary for any Irrlicht-based project (like the example here).
# For OpenGL-based or non-graphics projects, this is optional and the block
# below can be removed (or else explcitly set CMAKE_MACOSX_BUNDLE to 'OFF').
# If creating application bundles, the build output will be named 'myexe.app'.
# Use the convenience script 'run_app.sh' available under 'contrib/appbundle-macosx/'
# to run:
#     start_demo.sh myexe.app

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

#--------------------------------------------------------------
# 3. Specify project sources and add executable
#--------------------------------------------------------------

set(MY_FILES main.cpp simulation_launcher.cpp ros_bridge_driver.hpp ros_bridge.cpp physical_sensors.hpp physical_sensors.cpp terrain_system.hpp TcpPositionServer.cpp)

add_executable(main ${MY_FILES})

#--------------------------------------------------------------
# Set properties for the executable target
#--------------------------------------------------------------

# Here, we define a macro CHRONO_DATA_DIR which will contain the
# path to the Chrono data directory, either in its source tree
# (if using a build version of Chrono), or in its install tree
# (if using an installed version of Chrono).
target_compile_definitions(main PRIVATE "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 

if(MSVC)
    set_target_properties(main PROPERTIES MSVC_RUNTIME_LIBRARY ${CHRONO_MSVC_RUNTIME_LIBRARY})
endif()

#--------------------------------------------------------------
# Link to Chrono targets for the requested modules
#--------------------------------------------------------------
message(STATUS "Bullet includes: ${BULLET_INCLUDE_DIRS}")
message(STATUS "Bullet libs:     ${BULLET_LIBRARIES}")
target_include_directories(main PRIVATE ${CHRONO_THIRD_PARTY_INCLUDE_DIRS} /usr/local/include/chrono_thirdparty /usr/include/bullet/HACD)

target_link_libraries(main PRIVATE ${CHRONO_LIBRARIES} ${CHRONO_TARGETS} Eigen3::Eigen ${IRRLICHT_LIBRARY} ${BULLET_LIBRARIES} )

# Include directories
include_directories(${IRRLICHT_INCLUDE_DIR})
include_directories(/usr/local/include/chrono_thirdparty/HACD)  # Added to locate hacdVersion.h

if(MSVC)
    add_DLL_copy_command()  # Copies Chrono and dependency DLLs to the binary output folder
endif()
