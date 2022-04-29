# At LEAST 2.8 but newer is better
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(cpm_scenario VERSION 0.1 LANGUAGES CXX)

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED)

# Make local cmake script available
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
message(STATUS "Loading cmake script from: ${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Include Eigen as an imported target
find_package(Eigen3 REQUIRED NO_MODULE)
message(STATUS "Eigen3 Version: ${EIGEN3_VERSION_STRING}")

# Include PugiXML as an imported target
find_package(PugiXml REQUIRED NO_MODULE)
message(STATUS "PugiXml Version: ${PugiXml_VERSION}")


add_subdirectory(cpm_scenario)
