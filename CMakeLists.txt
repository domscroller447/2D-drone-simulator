# =============================================================================
#  Drone Simulator - Build Configuration
#
#  A 2D drone flight simulator with Newtonian rigid-body dynamics, dual-loop
#  PID control (altitude/position hold), and IMU noise/filtering models.
#  Raylib is fetched and built from source, so the only external prerequisite
#  is a C++17 toolchain and CMake itself.
# =============================================================================

cmake_minimum_required(VERSION 3.14)

project(DroneSimulator
    VERSION     1.0.0
    DESCRIPTION "2D drone flight simulator with PID control and sensor modeling"
    LANGUAGES   CXX
)

# -----------------------------------------------------------------------------
# Global build settings
# -----------------------------------------------------------------------------

# Default to an optimized configuration when the user hasn't chosen one.
# (Relevant for single-config generators: Makefiles, Ninja.)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo")
endif()

# Silences the CMake >= 3.24 "DOWNLOAD_EXTRACT_TIMESTAMP not given" warning
# that FetchContent emits when unpacking the raylib archive.
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

# -----------------------------------------------------------------------------
# Dependency: raylib (fetched and built from source)
# -----------------------------------------------------------------------------
include(FetchContent)

# raylib's own CMakeLists declares a cmake_minimum_required() below what
# CMake 4.x accepts without this compatibility shim.
# TODO: remove once upstream raylib raises its declared floor.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# The simulator only needs raylib's core/graphics modules; skip building its
# bundled examples and example games to keep configure/build times down and
# avoid pulling in unrelated third-party warnings.
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/5.0.zip
)
FetchContent_MakeAvailable(raylib)

# -----------------------------------------------------------------------------
# Executable target
# -----------------------------------------------------------------------------
add_executable(${PROJECT_NAME}
    main.cpp
    Drone.cpp
    Drone.h   # header included for IDE/project-file visibility only
)

# Target-scoped language requirement (modern CMake idiom) instead of the
# global CMAKE_CXX_STANDARD variable. cxx_std_17 is implicitly REQUIRED.
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_EXTENSIONS OFF   # disable compiler-specific (GNU) extensions for portability
)

target_link_libraries(${PROJECT_NAME} PRIVATE raylib)

# Strict diagnostics apply to OUR sources only. raylib is a separate target,
# so its third-party code is never held to (or blamed for) these flags.
target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive->
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
)

# To view graphs, run this in the terminal:
# cd ../.. ; python plot_telemetry.py