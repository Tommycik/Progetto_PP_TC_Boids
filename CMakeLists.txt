cmake_minimum_required(VERSION 3.20)
project(ProgettoOpenMP CXX)

set(CMAKE_CXX_STANDARD 17)

# Cerca la libreria OpenMP integrata nel compilatore
find_package(OpenMP REQUIRED)

# Sostituisci "main.cpp" con il nome del tuo file principale se è diverso
add_executable(Project_1_TC main.cpp
        main.cpp
        Entities/Boid.cpp
        Entities/Boid.h)

# Collega OpenMP al tuo progetto
if(OpenMP_CXX_FOUND)
    target_link_libraries(Project_1_TC PUBLIC OpenMP::OpenMP_CXX)
endif()