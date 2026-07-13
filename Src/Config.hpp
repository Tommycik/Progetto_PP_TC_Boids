#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <vector>
#include <cmath>
#include <omp.h>
#include <random>
#include <iomanip>
#include <string>
#include <fstream>
#include <SFML/Graphics.hpp>
#include "Entities/Boid.h"

#ifdef _WIN32
#include <windows.h>
#endif

using namespace std;

// parametri della simulazione
const float turnfactor = 0.3f;
const float visualRange = 40.0f;
const float protectedRange = 8.0f;
const float centeringfactor = 0.0005f;
const float avoidfactor = 0.05f;
const float matchingfactor = 0.05f;
const float maxspeed = 6.0f;
const float minspeed = 3.0f;
const float maxbias = 0.01f;
const float bias_increment = 0.00004f;

const float visualRangeSq = visualRange * visualRange;
const float protectedRangeSq = protectedRange * protectedRange;

// spazio in cui si possono muovere i boid
const float gridWidth = 1000.0f;
const float gridHeight = 700.0f;
const float leftmargin = 100.0f;
const float rightmargin = gridWidth - leftmargin;
const float topmargin = 100.0f;
const float bottommargin = gridHeight - topmargin;

// griglia
const float cellSize = visualRange;
const int gridCols = static_cast<int>(gridWidth / cellSize);
const int gridRows = static_cast<int>(gridHeight / cellSize);

// livelli di ottimizzazione
enum OptimizationLevel {
    NAIVE,          // la peggiore usa sqrt e usa molti più cicli di clock
    SQUARED_DIST,   // usa il quadrato della distanza e quindi solo somme e moltiplicazioni, meno cicli necessari
    BOUNDING_BOX,   // controlla subito la distanza se troppa non fa nessun calcolo
    GRID,           // i boids controllano solo la loro tile e quelle adiacenti
};

// funzione per ottenere la cella nella griglia
inline int getCellIndex(float x, float y) {
    int cx = static_cast<int>(x / cellSize);
    int cy = static_cast<int>(y / cellSize);
    // protezione per boid che fluttuano oltre i margini prima di rigirarsi
    if (cx < 0) cx = 0; if (cx >= gridCols) cx = gridCols - 1;
    if (cy < 0) cy = 0; if (cy >= gridRows) cy = gridRows - 1;
    return cy * gridCols + cx;
}

#endif // CONFIG_H