cmake_minimum_required(VERSION 3.10)
project(Flux3D)
set(CMAKE_CXX_STANDARD 17)

set(GLFW_DIR ${CMAKE_SOURCE_DIR}/libs/glfw)
set(GLM_DIR ${CMAKE_SOURCE_DIR}/libs/glm)
set(BULLET_DIR ${CMAKE_SOURCE_DIR}/libs/bullet3)

add_library(glfw STATIC IMPORTED)
set_target_properties(glfw PROPERTIES
    IMPORTED_LOCATION "${GLFW_DIR}/lib/glfw3.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${GLFW_DIR}/include"
)

add_library(glm INTERFACE IMPORTED)
set_target_properties(glm PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${GLM_DIR}"
)

add_library(glad STATIC ${CMAKE_SOURCE_DIR}/glad.c "shader_s.h")
target_include_directories(glad PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_library(stb_image STATIC ${CMAKE_SOURCE_DIR}/stb_image.cpp "shader_s.h")
target_include_directories(stb_image PUBLIC ${CMAKE_SOURCE_DIR})

set(IMGUI_SOURCES
    imgui/imgui.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
    imgui/backends/imgui_impl_glfw.cpp
    imgui/backends/imgui_impl_opengl3.cpp
)
add_library(imgui STATIC ${IMGUI_SOURCES} "shader_s.h")
target_include_directories(imgui PUBLIC 
    imgui 
    imgui/backends
    ${GLFW_DIR}/include
)

add_library(BulletDynamics STATIC IMPORTED)
set_target_properties(BulletDynamics PROPERTIES
    IMPORTED_LOCATION "${BULLET_DIR}/lib/BulletDynamics_Debug.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${BULLET_DIR}/src"
)
add_library(BulletCollision STATIC IMPORTED)
set_target_properties(BulletCollision PROPERTIES
    IMPORTED_LOCATION "${BULLET_DIR}/lib/BulletCollision_Debug.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${BULLET_DIR}/src"
)
add_library(LinearMath STATIC IMPORTED)
set_target_properties(LinearMath PROPERTIES
    IMPORTED_LOCATION "${BULLET_DIR}/lib/LinearMath_Debug.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${BULLET_DIR}/src"
)
add_library(BulletSoftBody STATIC IMPORTED)
set_target_properties(BulletSoftBody PROPERTIES
    IMPORTED_LOCATION "${BULLET_DIR}/lib/BulletSoftBody_Debug.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${BULLET_DIR}/src"
)

add_executable(${PROJECT_NAME} main.cpp "shader_s.h")
target_include_directories(${PROJECT_NAME} PRIVATE
    .
    include
    ${GLFW_DIR}/include
    ${GLM_DIR}
    imgui
    imgui/backends
    ${BULLET_DIR}/src
)
target_link_libraries(${PROJECT_NAME}
    glfw
    glm
    glad
    imgui
    stb_image
    BulletDynamics
    BulletCollision
    BulletSoftBody
    LinearMath
    opengl32
)
if(WIN32)
    target_link_libraries(${PROJECT_NAME} gdi32)
endif()

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_SOURCE_DIR}/textures"
    "$<TARGET_FILE_DIR:${PROJECT_NAME}>/textures"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_SOURCE_DIR}/shaders"
    "$<TARGET_FILE_DIR:${PROJECT_NAME}>/shaders"
)