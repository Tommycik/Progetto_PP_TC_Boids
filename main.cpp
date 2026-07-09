#include <iostream>
#include <vector>
#include <cmath>
#include <omp.h>
#include <random>
#include <iomanip>
#include <SFML/Graphics.hpp>
#include <string>
#include <fstream>

#include "Entities/Boid.h"
#ifdef _WIN32
#include <windows.h>
#endif
using namespace std;

// parametri della simulazione
// più alto a causa del delay nell'aggionramento dell'informazioni
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

// funzione per ottenere la cella nella griglia
inline int getCellIndex(float x, float y) {
    int cx = static_cast<int>(x / cellSize);
    int cy = static_cast<int>(y / cellSize);
    // protezione per boid che fluttuano oltre i margini prima di rigirarsi
    if (cx < 0) cx = 0; if (cx >= gridCols) cx = gridCols - 1;
    if (cy < 0) cy = 0; if (cy >= gridRows) cy = gridRows - 1;
    return cy * gridCols + cx;
}

// livelli di ottimizzazione
enum OptimizationLevel {
    NAIVE,          // la peggiore usa sqrt e usa molti più cicli di clock
    SQUARED_DIST,   // usa il quadrato della distanza e quindi solo somme e moltiplicazioni, meno cicli necessari
    BOUNDING_BOX,   // controlla subito la distanza se troppa non fa nessun calcolo
    GRID,           // i boids controllano solo la loro tile e quelle adiacenti
};

// logica sequenziale
void updateBoidsSequential(vector<Boid>& boids, OptimizationLevel opt) {
    int numBoids = boids.size();
    vector<vector<int>> grid(gridCols * gridRows);

    if (opt == GRID) {
        for (int i = 0; i < numBoids; ++i) {
            int cellIdx = getCellIndex(boids[i].getX(), boids[i].getY());
            grid[cellIdx].push_back(i);
        }
    }

    for (int boid = 0; boid < numBoids; ++boid) {
        float xposAvg = 0, yposAvg = 0, xvelAvg = 0, yvelAvg = 0;
        int neighboring_boids = 0;
        float closeDx = 0, closeDy = 0;

        Boid& b = boids[boid];
        float currX = b.getX();
        float currY = b.getY();
        float newVx = b.getVx();
        float newVy = b.getVy();
        float currBias = b.getBiasval();
        int group = b.getScoutGroup();

        if (opt == GRID) {
            int cx = static_cast<int>(currX / cellSize);
            int cy = static_cast<int>(currY / cellSize);

            for (int row = -1; row <= 1; ++row) {
                for (int column = -1; column <= 1; ++column) {
                    int neighborX = cx + column;
                    int neighborY = cy + row;

                    if (neighborX >= 0 && neighborX < gridCols && neighborY >= 0 && neighborY < gridRows) {
                        int cellIdx = neighborY * gridCols + neighborX;
                        const auto& cellBoids = grid[cellIdx];

                        for (int otherId : cellBoids) {
                            if (boid != otherId) {
                                const Boid& other = boids[otherId];
                                float dx = currX - other.getX();
                                float dy = currY - other.getY();

                                if (std::abs(dx) < visualRange && std::abs(dy) < visualRange) {
                                    float distSq = dx * dx + dy * dy;
                                    if (distSq < protectedRangeSq) {
                                        closeDx += dx;
                                        closeDy += dy;
                                    } else if (distSq < visualRangeSq) {
                                        xposAvg += other.getX();
                                        yposAvg += other.getY();
                                        xvelAvg += other.getVx();
                                        yvelAvg += other.getVy();
                                        neighboring_boids++;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } else {
            for (int otherId = 0; otherId < numBoids; ++otherId) {
                if (boid == otherId) continue;

                const Boid &other = boids[otherId];
                float dx = currX - other.getX();
                float dy = currY - other.getY();

                bool inProtected = false;
                bool inVisual = false;

                if (opt == NAIVE) {
                    float dist = std::sqrt(dx * dx + dy * dy);
                    if (dist < protectedRange) inProtected = true;
                    else if (dist < visualRange) inVisual = true;
                } else if (opt == SQUARED_DIST) {
                    float distSq = dx * dx + dy * dy;
                    if (distSq < protectedRangeSq) inProtected = true;
                    else if (distSq < visualRangeSq) inVisual = true;
                } else if (opt == BOUNDING_BOX) {
                    if (std::abs(dx) < visualRange && std::abs(dy) < visualRange) {
                        float distSq = dx * dx + dy * dy;
                        if (distSq < protectedRangeSq) inProtected = true;
                        else if (distSq < visualRangeSq) inVisual = true;
                    }
                }

                if (inProtected) {
                    closeDx += dx;
                    closeDy += dy;
                } else if (inVisual) {
                    xposAvg += other.getX();
                    yposAvg += other.getY();
                    xvelAvg += other.getVx();
                    yvelAvg += other.getVy();
                    neighboring_boids++;
                }
            }
        }

        if (neighboring_boids > 0) {
            xposAvg /= neighboring_boids;
            yposAvg /= neighboring_boids;
            xvelAvg /= neighboring_boids;
            yvelAvg /= neighboring_boids;

            newVx += (xposAvg - currX) * centeringfactor + (xvelAvg - b.getVx()) * matchingfactor;
            newVy += (yposAvg - currY) * centeringfactor + (yvelAvg - b.getVy()) * matchingfactor;
        }
        newVx += closeDx * avoidfactor;
        newVy += closeDy * avoidfactor;

        if (currY < topmargin) newVy += turnfactor;
        if (currY > bottommargin) newVy -= turnfactor;
        if (currX < leftmargin) newVx += turnfactor;
        if (currX > rightmargin) newVx -= turnfactor;

        if (group == 1) {
            if (newVx > 0) currBias = std::min(maxbias, currBias + bias_increment);
            else currBias = std::max(bias_increment, currBias - bias_increment);
            newVx = (1 - currBias) * newVx + (currBias * 1.0f);
            b.setBiasval(currBias);
        } else if (group == 2) {
            if (newVx < 0) currBias = std::min(maxbias, currBias + bias_increment);
            else currBias = std::max(bias_increment, currBias - bias_increment);
            newVx = (1 - currBias) * newVx + (currBias * -1.0f);
            b.setBiasval(currBias);
        }

        float speed = std::sqrt(newVx * newVx + newVy * newVy);
        if (speed < minspeed && speed > 0) {
            newVx = (newVx / speed) * minspeed;
            newVy = (newVy / speed) * minspeed;
        } else if (speed > maxspeed) {
            newVx = (newVx / speed) * maxspeed;
            newVy = (newVy / speed) * maxspeed;
        }

        b.setNewVx(newVx);
        b.setNewVy(newVy);
        b.setNewX(currX + newVx);
        b.setNewY(currY + newVy);
    }

    for (int i = 0; i < numBoids; ++i) {
        boids[i].commit();
    }
}

// logica parallela + vettorizzazione
void updateBoids(vector<Boid>& boids, int threads, OptimizationLevel opt) {
    int numBoids = boids.size();
    omp_set_num_threads(threads);
    vector<vector<int>> grid(gridCols * gridRows);

    if (opt == GRID) {
        for (int i = 0; i < numBoids; ++i) {
            int cellIdx = getCellIndex(boids[i].getX(), boids[i].getY());
            grid[cellIdx].push_back(i);
        }
    }

    // sezione parallela
    #pragma omp parallel
    {
        // scheduling statico a blocchi di 64 per localizzazione cache
        #pragma omp for schedule(static, 64)
        for (int boid = 0; boid < numBoids; ++boid) {
            float xposAvg = 0, yposAvg = 0, xvelAvg = 0, yvelAvg = 0;
            int neighboring_boids = 0;
            float closeDx = 0, closeDy = 0;

            Boid& b = boids[boid];

            float currX = b.getX();
            float currY = b.getY();
            float newVx = b.getVx();
            float newVy = b.getVy();
            float currBias = b.getBiasval();
            int group = b.getScoutGroup();

            if (opt == GRID) {
                int cx = static_cast<int>(currX / cellSize);
                int cy = static_cast<int>(currY / cellSize);

                for (int row = -1; row <= 1; ++row) {
                    for (int column = -1; column <= 1; ++column) {
                        int neighborX = cx + column;
                        int neighborY = cy + row;

                        if (neighborX >= 0 && neighborX < gridCols && neighborY >= 0 && neighborY < gridRows) {
                            int cellIdx = neighborY * gridCols + neighborX;
                            const auto& cellBoids = grid[cellIdx];

                            for (int otherId : cellBoids) {
                                if (boid != otherId) {
                                    const Boid& other = boids[otherId];

                                    float dx = currX - other.getX();
                                    float dy = currY - other.getY();

                                    if (std::abs(dx) < visualRange && std::abs(dy) < visualRange) {
                                        float distSq = dx * dx + dy * dy;

                                        if (distSq < protectedRangeSq) {
                                            closeDx += dx;
                                            closeDy += dy;
                                        } else if (distSq < visualRangeSq) {
                                            xposAvg += other.getX();
                                            yposAvg += other.getY();
                                            xvelAvg += other.getVx();
                                            yvelAvg += other.getVy();
                                            neighboring_boids++;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            } else {
                // direttiva SIMD per vettorizzare l'algoritmo
                #pragma omp simd reduction(+:closeDx,closeDy,xposAvg,yposAvg,xvelAvg,yvelAvg,neighboring_boids)
                for (int otherId = 0; otherId < numBoids; ++otherId) {
                    if (boid == otherId) continue;

                    const Boid &other = boids[otherId];

                    float dx = currX - other.getX();
                    float dy = currY - other.getY();

                    bool inProtected = false;
                    bool inVisual = false;

                    if (opt == NAIVE) {
                        float dist = std::sqrt(dx * dx + dy * dy);
                        if (dist < protectedRange) inProtected = true;
                        else if (dist < visualRange) inVisual = true;
                    } else if (opt == SQUARED_DIST) {
                        float distSq = dx * dx + dy * dy;
                        if (distSq < protectedRangeSq) inProtected = true;
                        else if (distSq < visualRangeSq) inVisual = true;
                    } else if (opt == BOUNDING_BOX) {
                        if (std::abs(dx) < visualRange && std::abs(dy) < visualRange) {
                            float distSq = dx * dx + dy * dy;
                            if (distSq < protectedRangeSq) inProtected = true;
                            else if (distSq < visualRangeSq) inVisual = true;
                        }
                    }

                    if (inProtected) {
                        closeDx += dx;
                        closeDy += dy;
                    } else if (inVisual) {
                        xposAvg += other.getX();
                        yposAvg += other.getY();
                        xvelAvg += other.getVx();
                        yvelAvg += other.getVy();
                        neighboring_boids++;
                    }
                }
            }

            if (neighboring_boids > 0) {
                xposAvg /= neighboring_boids;
                yposAvg /= neighboring_boids;
                xvelAvg /= neighboring_boids;
                yvelAvg /= neighboring_boids;

                newVx += (xposAvg - currX) * centeringfactor + (xvelAvg - b.getVx()) * matchingfactor;
                newVy += (yposAvg - currY) * centeringfactor + (yvelAvg - b.getVy()) * matchingfactor;
            }
            newVx += closeDx * avoidfactor;
            newVy += closeDy * avoidfactor;

            if (currY < topmargin) newVy += turnfactor;
            if (currY > bottommargin) newVy -= turnfactor;
            if (currX < leftmargin) newVx += turnfactor;
            if (currX > rightmargin) newVx -= turnfactor;

            if (group == 1) {
                if (newVx > 0) currBias = std::min(maxbias, currBias + bias_increment);
                else currBias = std::max(bias_increment, currBias - bias_increment);
                newVx = (1 - currBias) * newVx + (currBias * 1.0f);
                b.setBiasval(currBias);
            } else if (group == 2) {
                if (newVx < 0) currBias = std::min(maxbias, currBias + bias_increment);
                else currBias = std::max(bias_increment, currBias - bias_increment);
                newVx = (1 - currBias) * newVx + (currBias * -1.0f);
                b.setBiasval(currBias);
            }

            float speed = std::sqrt(newVx * newVx + newVy * newVy);
            if (speed < minspeed && speed > 0) {
                newVx = (newVx / speed) * minspeed;
                newVy = (newVy / speed) * minspeed;
            } else if (speed > maxspeed) {
                newVx = (newVx / speed) * maxspeed;
                newVy = (newVy / speed) * maxspeed;
            }

            b.setNewVx(newVx);
            b.setNewVy(newVy);
            b.setNewX(currX + newVx);
            b.setNewY(currY + newVy);
        }

        #pragma omp for schedule(static, 64)
        for (int i = 0; i < numBoids; ++i) {
            boids[i].commit();
        }
    }
}

void initFlock(vector<Boid>& boids, int N) {
    boids.clear();
    std::mt19937 gen(12345);
    std::uniform_real_distribution<float> pos_dist(200.0f, 600.0f);
    std::uniform_real_distribution<float> vel_dist(-5.0f, 5.0f);
    for (int i = 0; i < N; ++i) {
        int group = 0;
        if (i < N * 0.1) group = 1;
        else if (i < N * 0.2) group = 2;
        boids.emplace_back(pos_dist(gen), pos_dist(gen), vel_dist(gen), vel_dist(gen), group, 0.001f);
    }
}

void runGUI() {
    vector<Boid> boids;
    initFlock(boids, 5000);
    sf::RenderWindow window(sf::VideoMode(gridWidth, gridHeight), "Boids OpenMP - Ryzen 3600X");
    window.setFramerateLimit(60);
    sf::CircleShape boidShape(3.f, 3);
    boidShape.setFillColor(sf::Color::Cyan);
    omp_set_schedule(omp_sched_static, 64);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
        }
        updateBoids(boids, 12, GRID);
        window.clear(sf::Color(20, 20, 30));

        for (const auto& b : boids) {
            boidShape.setPosition(b.getX(), b.getY());
            float angle = atan2(b.getVy(), b.getVx()) * 180 / 3.14159f;
            boidShape.setRotation(angle + 90.f);
            window.draw(boidShape);
        }
        window.display();
    }
}

void runBenchmark() {
    #ifdef _WIN32
        SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED);
    #endif
    const int FRAMES = 50;
    const int RUNS = 5;

    vector<int> boidCounts = {2000, 5000, 10000, 20000};
    vector<int> threadCounts = {1, 2, 4, 6, 8, 12, 18};

    cout << "\nBOIDS BENCHMARK (AMD Ryzen 5 3600X)" << endl;
    cout << "Frames per ogni simulazione: " << FRAMES << endl;
    vector<OptimizationLevel> opts = {NAIVE, SQUARED_DIST, BOUNDING_BOX, GRID};
    vector<string> optNames = {"NAIVE", "SQUARED_DIST", "BOUNDING_BOX", "GRID"};
    vector<omp_sched_t> schedulers = {omp_sched_static, omp_sched_dynamic};
    vector<string> schedNames = {"STATICO", "DINAMICO"};

    ofstream csvFile("benchmark_results.csv");
    if (!csvFile.is_open()) {
        cerr << "Errore: Impossibile creare il file CSV!" << endl;
        return;
    }

    csvFile << "Ottimizzazione,Scheduling,Boids,Threads,TempoMedio_s,TempoMin_s,TempoMax_s,Speedup,TempoMedioBoid_us\n";

    for (size_t optIdx = 0; optIdx < opts.size(); ++optIdx) {
        for (size_t schedIdx = 0; schedIdx < schedulers.size(); ++schedIdx) {

            cout << "\n==============================================================" << endl;
            cout << "[ OTTIMIZZAZIONE: " << optNames[optIdx] << " | SCHEDULING: " << schedNames[schedIdx] << " ]" << endl;
            cout << "==============================================================" << endl;

            for (int numBoids : boidCounts) {
                cout << "\n--> Test con " << numBoids << " boids (Media su " << RUNS << " tentativi):" << endl;
                cout << setw(8)  << "Threads"
                     << setw(22) << "Tempo tot medio(s)"
                     << setw(12) << "Min (s)"
                     << setw(12) << "Max (s)"
                     << setw(12) << "Speedup"
                     << setw(24) << "T. medio/Boid (us)" << endl;
                cout << "--------------------------------------------------------------" << endl;

                double seqTime = 0.0;

                // calcolo preliminare della baseline sequenziale
                {
                    double elapsedAccumulator = 0.0;
                    for (int run = 0; run < RUNS; ++run) {
                        vector<Boid> boids;
                        initFlock(boids, numBoids);
                        double startTime = omp_get_wtime();
                        for (int frame = 0; frame < FRAMES; ++frame) {
                            updateBoidsSequential(boids, opts[optIdx]);
                        }
                        elapsedAccumulator += (omp_get_wtime() - startTime);
                    }
                    seqTime = elapsedAccumulator / static_cast<double>(RUNS);
                }

                for (int threads : threadCounts) {
                    double elapsedAccumulator = 0.0;
                    double minElapsed = 0.0;
                    double maxElapsed = 0.0;

                    for (int run = 0; run < RUNS; ++run) {
                        vector<Boid> boids;
                        initFlock(boids, numBoids);

                        omp_set_schedule(schedulers[schedIdx], 64);

                        double startTime = omp_get_wtime();
                        for (int frame = 0; frame < FRAMES; ++frame) {
                            updateBoids(boids, threads, opts[optIdx]);
                        }
                        double currentElapsed = omp_get_wtime() - startTime;
                        elapsedAccumulator += currentElapsed;

                        if (run == 0) {
                            minElapsed = currentElapsed;
                            maxElapsed = currentElapsed;
                        } else {
                            if (currentElapsed < minElapsed) minElapsed = currentElapsed;
                            if (currentElapsed > maxElapsed) maxElapsed = currentElapsed;
                        }
                    }

                    double avgElapsed = elapsedAccumulator / static_cast<double>(RUNS);
                    double timePerBoidUs = (avgElapsed * 1000000.0) / (FRAMES * numBoids);

                    cout << setw(8) << threads
                         << setw(22) << fixed << setprecision(4) << avgElapsed
                         << setw(12) << fixed << setprecision(4) << minElapsed
                         << setw(12) << fixed << setprecision(4) << maxElapsed;

                    // Speedup calcolato dividendo il tempo sequenziale nativo per quello corrente parallelo
                    double speedup = seqTime / avgElapsed;
                    cout << setw(11) << fixed << setprecision(2) << speedup << "x";

                    cout << setw(24) << fixed << setprecision(3) << timePerBoidUs << endl;

                    csvFile << optNames[optIdx] << ","
                            << schedNames[schedIdx] << ","
                            << numBoids << ","
                            << threads << ","
                            << fixed << setprecision(6) << avgElapsed << ","
                            << fixed << setprecision(6) << minElapsed << ","
                            << fixed << setprecision(6) << maxElapsed << ","
                            << setprecision(2) << speedup << ","
                            << setprecision(3) << timePerBoidUs << "\n";
                }
            }
        }
    }
}

int main() {
    int choice;
    do {
        cout << "Seleziona Modalita':\n";
        cout << "1. Simulazione Grafica (GUI SFML)\n";
        cout << "2. Esegui Benchmark (Test prestazioni per relazione)\n";
        cout << "3. Esci\n";
        cout << "Scelta: ";
        cin >> choice;

        if (choice == 1) runGUI();
        else if (choice == 2) runBenchmark();
        else if (choice != 3) cout << "Scelta non valida, riprova.\n";
    } while (choice != 3);

    return 0;
}