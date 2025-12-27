cmake_minimum_required(VERSION 3.10)
project(Flux3D)

set(CMAKE_CXX_STANDARD 17)

# Пути к библиотекам
set(GLFW_DIR ${CMAKE_SOURCE_DIR}/libs/glfw)
set(GLM_DIR ${CMAKE_SOURCE_DIR}/libs/glm)

# Ищем библиотеку assimp
find_library(ASSIMP_LIBRARY
    NAMES assimp-vc143-mt.lib assimp.lib assimp-vc142-mt.lib assimp-vc141-mt.lib
    PATHS 
        ${CMAKE_SOURCE_DIR}/libs/assimp
        ${CMAKE_SOURCE_DIR}/libs/assimp/lib/x64
        ${CMAKE_SOURCE_DIR}
    REQUIRED
)

find_path(ASSIMP_INCLUDE_DIR
    NAMES assimp/Importer.hpp
    PATHS 
        ${CMAKE_SOURCE_DIR}/libs/assimp
        ${CMAKE_SOURCE_DIR}
    PATH_SUFFIXES include
    REQUIRED
)

# Bullet Physics - теперь библиотеки в libs/bullet3/lib/
set(BULLET_DIR ${CMAKE_SOURCE_DIR}/libs/bullet3)
set(BULLET_LIB_DIR "${BULLET_DIR}/lib")

message(STATUS "Bullet directory: ${BULLET_DIR}")
message(STATUS "Bullet lib directory: ${BULLET_LIB_DIR}")

# Проверяем наличие библиотек
if(EXISTS "${BULLET_LIB_DIR}/BulletDynamics_Debug.lib" AND
   EXISTS "${BULLET_LIB_DIR}/BulletCollision_Debug.lib" AND
   EXISTS "${BULLET_LIB_DIR}/LinearMath_Debug.lib")
    
    message(STATUS "Found Bullet libraries in: ${BULLET_LIB_DIR}")
    
    # Создаем импортированные библиотеки
    add_library(BulletDynamics STATIC IMPORTED)
    set_target_properties(BulletDynamics PROPERTIES
        IMPORTED_LOCATION "${BULLET_LIB_DIR}/BulletDynamics_Debug.lib"
        INTERFACE_INCLUDE_DIRECTORIES "${BULLET_DIR}/src"
    )
    
    add_library(BulletCollision STATIC IMPORTED)
    set_target_properties(BulletCollision PROPERTIES
        IMPORTED_LOCATION "${BULLET_LIB_DIR}/BulletCollision_Debug.lib"
        INTERFACE_INCLUDE_DIRECTORIES "${BULLET_DIR}/src"
    )
    
    add_library(LinearMath STATIC IMPORTED)
    set_target_properties(LinearMath PROPERTIES
        IMPORTED_LOCATION "${BULLET_LIB_DIR}/LinearMath_Debug.lib"
        INTERFACE_INCLUDE_DIRECTORIES "${BULLET_DIR}/src"
    )
    
    set(BULLET_INCLUDE_DIRS 
        ${BULLET_DIR}/src
    )
    set(BULLET_LIBRARIES
        BulletDynamics
        BulletCollision
        LinearMath
    )
    
else()
    message(FATAL_ERROR "Bullet libraries not found in ${BULLET_LIB_DIR}/")
    message(STATUS "Please run fix_bullet_copy.bat to copy libraries")
    message(STATUS "Expected files:")
    message(STATUS "  ${BULLET_LIB_DIR}/BulletDynamics_Debug.lib")
    message(STATUS "  ${BULLET_LIB_DIR}/BulletCollision_Debug.lib")
    message(STATUS "  ${BULLET_LIB_DIR}/LinearMath_Debug.lib")
endif()

# GLFW
add_library(glfw STATIC IMPORTED)
set_target_properties(glfw PROPERTIES
    IMPORTED_LOCATION "${GLFW_DIR}/lib/glfw3.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${GLFW_DIR}/include"
)

# GLM
add_library(glm INTERFACE IMPORTED)
set_target_properties(glm PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${GLM_DIR}"
)

# GLAD
add_library(glad STATIC ${CMAKE_SOURCE_DIR}/glad.c)
target_include_directories(glad PUBLIC ${CMAKE_SOURCE_DIR}/include)

# STB_IMAGE
add_library(stb_image STATIC ${CMAKE_SOURCE_DIR}/stb_image.cpp)
target_include_directories(stb_image PUBLIC ${CMAKE_SOURCE_DIR})

# IMGUI
set(IMGUI_SOURCES
    imgui/imgui.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
    imgui/backends/imgui_impl_glfw.cpp
    imgui/backends/imgui_impl_opengl3.cpp
)
add_library(imgui STATIC ${IMGUI_SOURCES})
target_include_directories(imgui PUBLIC 
    imgui 
    imgui/backends
    ${GLFW_DIR}/include
)

# Основной исполняемый файл Flux3D
add_executable(${PROJECT_NAME} 
    main.cpp
    model.cpp
)

# Включение заголовков ДЛЯ Flux3D
target_include_directories(${PROJECT_NAME} PRIVATE
    .
    include
    ${GLFW_DIR}/include
    ${GLM_DIR}
    imgui
    imgui/backends
    ${ASSIMP_INCLUDE_DIR}
    ${BULLET_INCLUDE_DIRS}
)

# Линковка для Flux3D
target_link_libraries(${PROJECT_NAME}
    glfw
    glm
    glad
    imgui
    stb_image
    ${ASSIMP_LIBRARY}
    ${BULLET_LIBRARIES}
    opengl32
    kernel32.lib user32.lib gdi32.lib winspool.lib 
    shell32.lib ole32.lib oleaut32.lib uuid.lib 
    comdlg32.lib advapi32.lib
)

# Копирование ресурсов
if(EXISTS ${CMAKE_SOURCE_DIR}/shaders)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/shaders"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>/shaders"
    )
endif()

if(EXISTS ${CMAKE_SOURCE_DIR}/textures)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/textures"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>/textures"
    )
endif()