cmake_minimum_required(VERSION 3.16)
project(carballer CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# --- RocketSim (submodule) ---
add_subdirectory(RocketSim)

# --- Dependencies ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(GLEW REQUIRED glew)

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
target_link_libraries(imgui PUBLIC ${SDL2_LIBRARIES} ${GLEW_LIBRARIES})
target_include_directories(imgui PUBLIC ${SDL2_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS})

# --- Game ---
file(GLOB_RECURSE GAME_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_executable(carballer ${GAME_SOURCES})

# project root: resolves #include "RocketSim/src/..."
target_include_directories(carballer PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_definitions(carballer PRIVATE
  CARBALLER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(carballer PRIVATE RocketSim imgui ${SDL2_LIBRARIES} ${GLEW_LIBRARIES})
target_include_directories(carballer PRIVATE ${SDL2_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS})
target_link_directories(carballer PRIVATE ${SDL2_LIBRARY_DIRS} ${GLEW_LIBRARY_DIRS})

# Assets live next to the binary for easy running from build dir
add_custom_command(TARGET carballer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/assets $<TARGET_FILE_DIR:carballer>/assets)
