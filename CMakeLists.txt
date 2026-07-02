cmake_minimum_required(VERSION 3.20)
project(ProgettoOpenMP CXX)

set(CMAKE_CXX_STANDARD 17)

# 1. Cerca la libreria OpenMP integrata nel compilatore
find_package(OpenMP REQUIRED)

# 2. Configura SFML via FetchContent (scarica solo la parte grafica per evitare errori)
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

# 3. Crea l'eseguibile (corretto il duplicato di main.cpp)
add_executable(Project_1_TC
        main.cpp
        Entities/Boid.cpp
        Entities/Boid.h
)

# 4. Collega sia OpenMP che i moduli di SFML al tuo progetto
target_link_libraries(Project_1_TC PRIVATE
        OpenMP::OpenMP_CXX
        sfml-graphics
        sfml-window
        sfml-system
)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(Project_1_TC PRIVATE -fopenmp)
    target_link_options(Project_1_TC PRIVATE -fopenmp)
endif()