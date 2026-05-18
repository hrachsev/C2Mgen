cmake_minimum_required(VERSION 3.16)
project(C2Mgen LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
  add_compile_options(/W4 /permissive-)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)

FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG        3.4
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(
  glm
  GIT_REPOSITORY https://github.com/g-truc/glm.git
  GIT_TAG        1.0.1
)
FetchContent_MakeAvailable(glm)

FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        v1.91.9
)
FetchContent_MakeAvailable(imgui)

set(IMGUI_SOURCES
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/app/Application.cpp
  src/ui/OrbitCamera.cpp
  src/system/NativeFileDialogs.cpp
  src/render/Viewport3D.cpp
  src/io/PlyLoader.cpp
  src/io/ObjWriter.cpp
  src/core/KdTree.cpp
  src/core/Normals.cpp
  src/core/MarchingCubes.cpp
  src/recon/CloudStats.cpp
  src/recon/BallPivoting.cpp
  src/recon/PoissonRecon.cpp
  src/recon/MarchingCubesRecon.cpp
  src/recon/VoxelRemesh.cpp
  src/recon/GreedyProjection.cpp
  src/recon/AutoSelector.cpp
  ${IMGUI_SOURCES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  glfw
  glm::glm
)

find_package(OpenGL REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE OpenGL::GL)

if(APPLE)
  target_link_libraries(${PROJECT_NAME} PRIVATE "-framework Cocoa" "-framework IOKit" "-framework CoreVideo")
endif()

if(MSVC OR WIN32)
  target_compile_definitions(${PROJECT_NAME} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()
