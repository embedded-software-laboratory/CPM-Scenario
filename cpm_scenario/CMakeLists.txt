
# Define library. Only source files here!
project(libcpm_scenario VERSION 0.1 LANGUAGES CXX)

add_library(cpm_scenario STATIC
        src/ExtendedObject.cpp
        src/ObjectState.cpp
        src/Scenario.cpp
        src/ScenarioWriter.cpp
        src/ScenarioParser.cpp)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(cpm_scenario PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)

# If we have compiler requirements for this library, list them
# here
target_compile_features(cpm_scenario
        PUBLIC cxx_auto_type
        PRIVATE cxx_variadic_templates)

# Depend on a library that we defined in the top-level file
target_link_libraries(cpm_scenario
        pugixml
        Eigen3::Eigen)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS cpm_scenario EXPORT CpmScenarioConfig
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT CpmScenarioConfig DESTINATION share/CpmScenario/cmake)

# This makes the project importable from the build directory
export(TARGETS cpm_scenario FILE CpmScenarioConfig.cmake)

# Make cmake helping functions available
include(CMakePackageConfigHelpers)

# Make sure proper cmake version file is written
set_property(TARGET cpm_scenario PROPERTY VERSION ${CMAKE_PROJECT_VERSION})
write_basic_package_version_file(CpmScenarioConfigVersion.cmake VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)