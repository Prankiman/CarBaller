cmake_minimum_required(VERSION 3.16)
project(carballer CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Per-compiler flags: MSVC rejects the GCC/Clang warning flags.
if(MSVC)
  # /bigobj: some Bullet translation units exceed the default 64k sections.
  add_compile_options(/bigobj)
  # M_PI (used throughout) is hidden by <cmath> unless _USE_MATH_DEFINES is
  # set; silence the C4996 "unsafe CRT" deprecation noise; keep windows.h
  # min/max macros away from std::min/max.
  add_compile_definitions(_USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS NOMINMAX)
else()
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

# main() is a plain console entry point - don't let SDL rename it to SDL_main
# (which would require linking the platform's SDL2main shim).
add_compile_definitions(SDL_MAIN_HANDLED)

# --- RocketSim (submodule) ---
add_subdirectory(RocketSim)

# --- Dependencies: SDL2 + GLEW ---
# Unix: pkg-config against the system packages (the traditional path).
# Windows (vcpkg) and other pkg-config-less setups: CMake config packages.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND OR PKG_CONFIG_FOUND)
  pkg_check_modules(SDL2 QUIET IMPORTED_TARGET sdl2)
  pkg_check_modules(GLEW QUIET IMPORTED_TARGET glew)
endif()

if(TARGET PkgConfig::SDL2)
  set(CARBALLER_SDL2 PkgConfig::SDL2)
else()
  find_package(SDL2 CONFIG REQUIRED)
  set(CARBALLER_SDL2 SDL2::SDL2)
endif()

if(TARGET PkgConfig::GLEW)
  set(CARBALLER_GLEW PkgConfig::GLEW)
else()
  find_package(GLEW REQUIRED)
  set(CARBALLER_GLEW GLEW::GLEW)
endif()

# GL 1.1 entry points (glClear, glViewport, ...) are linked directly (libGL on
# Unix, opengl32.lib on Windows); everything newer is dispatched through GLEW.
find_package(OpenGL REQUIRED)

# --- Dear ImGui (vendored) ---
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui)
add_library(imgui STATIC
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)
target_link_libraries(imgui PUBLIC ${CARBALLER_SDL2} ${CARBALLER_GLEW})

# --- Game ---
file(GLOB_RECURSE GAME_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_executable(carballer ${GAME_SOURCES})

target_include_directories(carballer PRIVATE
  # project root: resolves #include "RocketSim/src/..."
  ${CMAKE_CURRENT_SOURCE_DIR}
  # local headers: #include "core/...", etc.
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  # vendored nlohmann/json (<nlohmann/json.hpp>) - never depend on a system
  # copy being installed
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
target_compile_definitions(carballer PRIVATE
  CARBALLER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(carballer PRIVATE RocketSim imgui ${CARBALLER_SDL2} ${CARBALLER_GLEW} OpenGL::GL)

# Assets live next to the binary for easy running from build dir
add_custom_command(TARGET carballer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/assets $<TARGET_FILE_DIR:carballer>/assets)
