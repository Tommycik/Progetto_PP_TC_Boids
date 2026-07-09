cmake_minimum_required(VERSION 3.20)
project(ProgettoOpenMP CXX)

set(CMAKE_CXX_STANDARD 17)

# libreria OpenMP
find_package(OpenMP REQUIRED)

# disattiva le librerie dinamiche (.dll) e forza la compilazione statica (.a)
set(BUILD_SHARED_LIBS FALSE CACHE BOOL "" FORCE)

# configura SFML via FetchContent
include(FetchContent)
set(SFML_BUILD_AUDIO FALSE CACHE BOOL "" FORCE)
set(SFML_BUILD_NETWORK FALSE CACHE BOOL "" FORCE)
set(SFML_BUILD_EXAMPLES FALSE CACHE BOOL "" FORCE)
set(SFML_BUILD_DOCS FALSE CACHE BOOL "" FORCE)

FetchContent_Declare(
        SFML
        GIT_REPOSITORY https://github.com/SFML/SFML.git
        GIT_TAG 2.6.1
)
FetchContent_MakeAvailable(SFML)

# crea l'eseguibile
add_executable(Project_1_TC
        main.cpp
        Entities/Boid.cpp
        Entities/Boid.h
)

# librerie
target_link_libraries(Project_1_TC PRIVATE
        OpenMP::OpenMP_CXX
        sfml-graphics
        sfml-window
        sfml-system
)

# flag OpenMP per MinGW
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(Project_1_TC PRIVATE -fopenmp)
    target_link_options(Project_1_TC PRIVATE -fopenmp)
endif()