cmake_minimum_required(VERSION 3.20)
project(ProgettoOpenMP CXX)

set(CMAKE_CXX_STANDARD 17)
# forza la compilazione in modalità Release per attivare le ottimizzazioni di base
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Scegli il tipo di build." FORCE)
endif()
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
        Src/main.cpp
        Src/Entities/Boid.cpp
        Src/Entities/Boid.h
        Src/Config.hpp
        Src/Simulation.hpp
        Src/Simulation.cpp
        Src/Pipelines.cpp
        Src/Pipelines.hpp
)

# librerie
target_link_libraries(Project_1_TC PRIVATE
        OpenMP::OpenMP_CXX
        sfml-graphics
        sfml-window
        sfml-system
)

# flag OpenMP e ottimizzazioni solo in modalità release
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(Project_1_TC PRIVATE $<$<CONFIG:Release>:-O3 -march=native>)
    target_link_options(Project_1_TC PRIVATE -fopenmp)
elseif(MSVC)
    # Applica /O2 /Oi /Ot SOLO se la configurazione è impostata su Release
    target_compile_options(Project_1_TC PRIVATE /openmp $<$<CONFIG:Release>:/O2 /Oi /Ot>)
endif()