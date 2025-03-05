cmake_minimum_required(VERSION 3.15)
cmake_policy(SET CMP0167 NEW)

#--------------------------------------------------------------
# Set the project name
#--------------------------------------------------------------
project(my_project)

# Set the build type (Debug or Release)
set(CMAKE_BUILD_TYPE Debug) # Or Release
set(CMAKE_CXX_FLAGS_DEBUG "-g") # Add debug flags
set(CMAKE_CXX_FLAGS_RELEASE "-O2") # Add optimization flags for release

include_directories(${CHRONO_INCLUDE_DIRS} /usr/include/bullet)

#--------------------------------------------------------------
# Find the Chrono package and any required or optional modules
#--------------------------------------------------------------
LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
find_package(Chrono
             COMPONENTS Irrlicht Vehicle PostProcess
             CONFIG)

# Return now if Chrono or a required component was not found.
if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

#--------------------------------------------------------------
# Set the C++ standard to match Chrono's
#--------------------------------------------------------------
set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD ${CHRONO_CXX_STANDARD})

#--------------------------------------------------------------
# Add path to Chrono headers and other required directories
#--------------------------------------------------------------
include_directories(${CHRONO_INCLUDE_DIRS})

# Find required packages
find_package(nlohmann_json 3.2.0 REQUIRED)

# Add WebSocket++ (header-only library)
# You might need to adjust the path based on your system
include_directories(/usr/include) # Make sure WebSocket++ headers are found

# Find Boost - required by WebSocket++
find_package(Boost REQUIRED COMPONENTS system thread)

# Add WebSocketpp specific defines
add_definitions(-DASIO_STANDALONE)
add_definitions(-D_WEBSOCKETPP_CPP11_INTERNAL_)

# Add include directories
include_directories(
    ${CHRONO_INCLUDE_DIRS}
    /usr/include/bullet
    ${Boost_INCLUDE_DIRS}
)

# Find Boost - required by WebSocket++
find_package(Boost REQUIRED COMPONENTS system thread)

#--------------------------------------------------------------
# Separate source files that contain implementations
set(SOURCES
    main.cpp
)

# Create the executable with only the source files
add_executable(main ${SOURCES})

# Set properties for your executable target
target_compile_definitions(main PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
target_compile_options(main PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(main PUBLIC ${CHRONO_LINKER_FLAGS})

# Link to Chrono libraries and dependency libraries
target_link_libraries(main 
    ${CHRONO_LIBRARIES} 
    /usr/lib/libGL.so 
    nlohmann_json::nlohmann_json 
    Boost::system 
    Boost::thread
)

# Optionally, install the executable (for packaged projects)
install(TARGETS main DESTINATION bin)
