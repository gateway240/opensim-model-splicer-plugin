cmake_minimum_required(VERSION 3.22)
# This is the version of cmake on ubuntu 22
project(OpenModelSplicerPlugin)

set(PLUGIN_NAME "osimModelSplicerPlugin"
    CACHE STRING "Name of shared library to create")


# Settings.
# ---------
set(TARGET "main" CACHE STRING "main")

# OpenSim uses C++20 language features.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -march=native")
set(GCC_OPT_ENABLE "-funroll-loops")

# generation compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find and hook up to OpenSim.
# ----------------------------
set(OpenSim_DIR "~/opensim-core/cmake")
find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

set(BUILD_SHARED_LIBS "TRUE" CACHE BOOL "Build shared libraries." FORCE)

# Configure this project.
# -----------------------
set(MY_LIBRARY_SOURCES
    lib/Knife.cpp
    lib/Utils.cpp
)

set(MY_LIBRARY_HEADERS
    lib/Knife.h
    lib/Utils.h
)

set (Plugin_HEADERS     ${MY_LIBRARY_HEADERS})
set (Plugin_SOURCES     ${MY_LIBRARY_SOURCES})

# Build Shared Library
add_library(${PLUGIN_NAME} ${Plugin_HEADERS} ${Plugin_SOURCES} )
target_include_directories(${PLUGIN_NAME}
    PUBLIC ${CMAKE_SOURCE_DIR}/lib
)
target_link_libraries(${PLUGIN_NAME} ${OpenSim_LIBRARIES})

set_target_properties(
    ${PLUGIN_NAME} PROPERTIES
    DEFINE_SYMBOL OSIMPLUGIN_EXPORTS
    PROJECT_LABEL "Libraries - ${PLUGIN_NAME}")

# Data files
set(DATA_DIR "${CMAKE_SOURCE_DIR}/data")
file(GLOB DATA_FILES "${DATA_DIR}/*")
add_custom_target(symlink_data ALL)

foreach(datafile ${DATA_FILES})
    get_filename_component(datafilename ${datafile} NAME)
    add_custom_command(
        TARGET symlink_data
        PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E create_symlink
                ${datafile} ${CMAKE_CURRENT_BINARY_DIR}/${datafilename}
        COMMENT "Creating symlink for ${datafilename}"
    )
endforeach()

# Build Demo Main
# If necessary, use the RELATIVE flag, otherwise each source file may be listed
# with full pathname. RELATIVE may makes it easier to extract an executable name
# automatically.
# file( GLOB APP_SOURCES RELATIVE app/*.cxx )
file( GLOB APP_SOURCES app/*.cpp )
foreach( testsourcefile ${APP_SOURCES} )
    # Cut off the file extension and directory path
    get_filename_component( testname ${testsourcefile} NAME_WE )
    add_executable( ${testname} ${testsourcefile} )
    # Make sure YourLib is linked to each app
    target_link_libraries( ${testname} ${PLUGIN_NAME})
    # Ensure each executable depends on the symlink_data target
    add_dependencies(${testname} symlink_data)
endforeach( testsourcefile ${APP_SOURCES} )