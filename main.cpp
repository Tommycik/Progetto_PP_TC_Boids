#include <iostream>
#include <vector>
#include <cmath>
#include <omp.h>
#include <random>
#include <iomanip>
#include <SFML/Graphics.hpp>

#include "Entities/Boid.h"
#ifdef _WIN32
#include <windows.h>
#endif
using namespace std;

// parametri della simulazione
//più alto a causa del delay nell'aggionramento dell'informazioni
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
//spazio in cui si possono muovere i boid
const float gridWidth = 1000.0f;
const float gridHeight = 700.0f;
const float leftmargin = 100.0f;
const float rightmargin = gridWidth - leftmargin;
const float topmargin = 100.0f;
const float bottommargin = gridHeight - topmargin;
//griglia
const float cellSize = visualRange;
const int gridCols = static_cast<int>(gridWidth / cellSize);
const int gridRows = static_cast<int>(gridHeight / cellSize);
//funzione per ottenere la cella nella griglia
inline int getCellIndex(float x, float y) {
    int cx = static_cast<int>(x / cellSize);
    int cy = static_cast<int>(y / cellSize);
    // Protezione per boid che fluttuano oltre i margini prima di rigirarsi
    if (cx < 0) cx = 0; if (cx >= gridCols) cx = gridCols - 1;
    if (cy < 0) cy = 0; if (cy >= gridRows) cy = gridRows - 1;
    return cy * gridCols + cx;
}
//livelli di ottimizzazione
enum OptimizationLevel {
    NAIVE,//la peggiore usa sqrt e usa molti più cicli di clock
    SQUARED_DIST, //usa il quadrato della distanza e quindi solo somme e moltiplicazioni, meno cicli necessari
    BOUNDING_BOX, // controla subito la distanza se troppa non fa nessun calcolo
    BOUNDING_BOX_NOT_IF, // uguale a bounding box ma senza ottimizzazione if
    GRID, // i boids controllano solo la loro tile e quelle adiacenti
};

// Logica parallela
void updateBoids(vector<Boid>& boids, int threads, OptimizationLevel opt) {
    int numBoids = boids.size();
    omp_set_num_threads(threads);
    //creazione griglia,usa gli indici cosi d aoccupare meno spazio in memoria
    //creata e distrutta per evitare sincronizazione tra i thread
    vector<vector<int>> grid(gridCols * gridRows);

    // Se la modalità è GRID, riempie le tile con gli indici dei boid
    if (opt == GRID) {
        for (int i = 0; i < numBoids; ++i) {
            int cellIdx = getCellIndex(boids[i].getX(), boids[i].getY());
            grid[cellIdx].push_back(i);
        }
    }
    //sezione parallela
    #pragma omp parallel
        //scheduling statico a blocchi di 64 così ogni thread gestisce i boid contigui in memoria migliorando acesso a cache
        #pragma omp for schedule(static, 64)
        for (int boid = 0; boid < numBoids; ++boid) {
            //azzera le variabili di accumulazione
            float xposAvg = 0, yposAvg = 0, xvelAvg = 0, yvelAvg = 0;
            int neighboring_boids = 0;
            float closeDx = 0, closeDy = 0;

            Boid& b = boids[boid];

            // valori correnti
            float currX = b.getX();
            float currY = b.getY();
            float newVx = b.getVx();
            float newVy = b.getVy();
            float currBias = b.getBiasval();
            int group = b.getScoutGroup();
            //ottimizzazione con la griglia
            if (opt == GRID) {
                // calcola la cella in cui si trova il boid corrente
                int cx = static_cast<int>(currX / cellSize);
                int cy = static_cast<int>(currY / cellSize);

                // ciclo sulle 9 celle totali (quella attuale + le 8 vicine)
                for (int row = -1; row <= 1; ++row) {
                    for (int column = -1; column <= 1; ++column) {
                        int neighborX = cx + column;
                        int neighborY = cy + row;

                        // salta se la cella vicina se è fuori dai confini dell'arena
                        if (neighborX < 0 || neighborX >= gridCols || neighborY < 0 || neighborY >= gridRows)
                            continue;

                        // trova l'indice lineare del vettore della griglia
                        int cellIdx = neighborY * gridCols + neighborX;
                        const auto& cellBoids = grid[cellIdx];

                        // controlla i boid  presenti in questa  tile
                        for (int otherId : cellBoids) {
                            if (boid == otherId) continue;

                            const Boid& other = boids[otherId];

                            // Distanza tra i boids
                            float dx = currX - other.getX();
                            float dy = currY - other.getY();
                            //controlla prima la distanza
                            if (std::abs(dx) < visualRange && std::abs(dy) < visualRange) {
                                float distSq = dx * dx + dy * dy;

                                if (distSq < protectedRangeSq) {
                                    closeDx += dx;
                                    closeDy += dy;
                                }
                                else if (distSq < visualRangeSq) {
                                    xposAvg += other.getX();
                                    yposAvg += other.getY();
                                    xvelAvg += other.getVx();
                                    yvelAvg += other.getVy();
                                    neighboring_boids++;
                                }
                                else {
                                    continue; // Cella giusta, ma fuori dal cerchio visivo
                                }
                            }
                            else {
                                continue; // Fallisce il bounding box nella tile
                            }
                        }
                    }
                }
            }else {
                //controllo degli altri boid con le altre ottimizazioni
                for (int otherId = 0; otherId < numBoids; ++otherId) {
                    if (boid == otherId) continue;

                    const Boid &other = boids[otherId];

                    // Distanza tra boid due boids
                    float dx = currX - other.getX();
                    float dy = currY - other.getY();

                    bool inProtected = false;
                    bool inVisual = false;
                    //varie ottimizzazioni
                    if (opt == NAIVE) {
                        float dist = std::sqrt(dx * dx + dy * dy);
                        if (dist < protectedRange) inProtected = true;
                        else if (dist < visualRange) inVisual = true;
                        else continue;
                    } else if (opt == SQUARED_DIST) {
                        float distSq = dx * dx + dy * dy;
                        if (distSq < protectedRangeSq) inProtected = true;
                        else if (distSq < visualRangeSq) inVisual = true;
                        else continue;
                    } else if (opt == BOUNDING_BOX) {
                        if (std::abs(dx) < visualRange && std::abs(dy) < visualRange) {
                            float distSq = dx * dx + dy * dy;
                            if (distSq < protectedRangeSq) inProtected = true;
                            else if (distSq < visualRangeSq) inVisual = true;
                        } else {
                            continue;
                        }
                    }else if (opt == BOUNDING_BOX_NOT_IF) {
                        if (std::abs(dx) < visualRange && std::abs(dy) < visualRange) {
                            float distSq = dx * dx + dy * dy;
                            if (distSq < protectedRangeSq) inProtected = true;
                            else if (distSq < visualRangeSq) inVisual = true;
                        }
                        //no ottimizzazione if con continue
                    }
                    //boids nel range protetto
                    if (inProtected) {
                        closeDx += dx;
                        closeDy += dy;
                        //boids nel range di visione
                    } else if (inVisual) {
                        xposAvg += other.getX();
                        yposAvg += other.getY();
                        xvelAvg += other.getVx();
                        yvelAvg += other.getVy();
                        neighboring_boids++;
                    }
                }
            }
            // fattori centering/matching
            if (neighboring_boids > 0) {
                xposAvg /= neighboring_boids;
                yposAvg /= neighboring_boids;
                xvelAvg /= neighboring_boids;
                yvelAvg /= neighboring_boids;

                newVx += (xposAvg - currX) * centeringfactor + (xvelAvg - b.getVx()) * matchingfactor;
                newVy += (yposAvg - currY) * centeringfactor + (yvelAvg - b.getVy()) * matchingfactor;
            }
            // fattore di avoidance
            newVx += closeDx * avoidfactor;
            newVy += closeDy * avoidfactor;

            // limiti dati dallo schermo
            if (currY < topmargin) newVy += turnfactor;
            if (currY > bottommargin) newVy -= turnfactor;
            if (currX < leftmargin) newVx += turnfactor;
            if (currX > rightmargin) newVx -= turnfactor;
            // calcolo bias dei due gruppi
            if (group == 1) {
                if (newVx > 0) currBias = std::min(maxbias, currBias + bias_increment);
                else currBias = std::max(bias_increment, currBias - bias_increment);
                newVx = (1 - currBias) * newVx + (currBias * 1.0f);
                //aggiorna il bias
                b.setBiasval(currBias);
            } else if (group == 2) {
                if (newVx < 0) currBias = std::min(maxbias, currBias + bias_increment);
                else currBias = std::max(bias_increment, currBias - bias_increment);
                newVx = (1 - currBias) * newVx + (currBias * -1.0f);
                //aggiorna il bias
                b.setBiasval(currBias);
            }

            // calcolo e limitazione della velocità
            float speed = std::sqrt(newVx * newVx + newVy * newVy);
            if (speed < minspeed && speed > 0) {
                newVx = (newVx / speed) * minspeed;
                newVy = (newVy / speed) * minspeed;
            } else if (speed > maxspeed) {
                newVx = (newVx / speed) * maxspeed;
                newVy = (newVy / speed) * maxspeed;
            }

            // aggiorna valori temporanei
            b.setNewVx(newVx);
            b.setNewVy(newVy);
            b.setNewX(currX + newVx);
            b.setNewY(currY + newVy);
        }
        //aggiorna i valori reali dopo la barriera implicita
        #pragma omp  for schedule(static, 64)
        for (int i = 0; i < numBoids; ++i) {
            boids[i].commit();
        }
}

//inizializza il vettore di boids
void initFlock(vector<Boid>& boids, int N) {
    //pulisce quelli del test precedente
    boids.clear();
    //usa sempre lo stesso seme per avere test riproducibili
    std::mt19937 gen(12345);
    //distribuzioni posizioni e velocità
    std::uniform_real_distribution<float> pos_dist(200.0f, 600.0f);
    std::uniform_real_distribution<float> vel_dist(-5.0f, 5.0f);
    for (int i = 0; i < N; ++i) {
        int group = 0;
        if (i < N * 0.1) group = 1;      
        else if (i < N * 0.2) group = 2;
        //alloca l'oggetto direttamente nel vettore così da risparmiare operazioni
        boids.emplace_back(pos_dist(gen), pos_dist(gen), vel_dist(gen), vel_dist(gen), group, 0.001f);
    }
}

void runGUI() {
    //vettore dei boids
    vector<Boid> boids;
    //viene inizializzato
    initFlock(boids, 5000);
    //viene creata la finestra
    sf::RenderWindow window(sf::VideoMode(gridWidth, gridHeight), "Boids OpenMP - Ryzen 3600X");
    window.setFramerateLimit(60);
    // forma e colore dei boids
    sf::CircleShape boidShape(3.f, 3);
    boidShape.setFillColor(sf::Color::Cyan);
    //setta lo scheduling
    omp_set_schedule(omp_sched_static, 64);
    //main loop
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
        }
        //aggiorna le posizioni
        updateBoids(boids, 12, GRID);
        //pulisce lo schermo
        window.clear(sf::Color(20, 20, 30));
        
        for (const auto& b : boids) {
            //posiziona il boid
            boidShape.setPosition(b.getX(), b.getY());
            //lo ruota nella direzione di movimento
            float angle = atan2(b.getVy(), b.getVx()) * 180 / 3.14159f;
            boidShape.setRotation(angle + 90.f);
            // lo disegna
            window.draw(boidShape);
        }
        //mostra tutto a schermo
        window.display();
    }
}

#include <string>
#include <fstream>
void runBenchmark() {
    // blocca la sospensione del pc per tutta la durata del benchmark
    #ifdef _WIN32
        SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED);
    #endif
        //numero di frame
    const int FRAMES = 50;
    //ogni test è ripetuto varie volte e poi viene fatta la media
    const int RUNS = 5;

    // i vari scaglioni di boids e thread su cui iterare
    vector<int> boidCounts = {2000, 5000, 10000, 20000};
    vector<int> threadCounts = {1, 2, 4, 6, 8, 12, 18};

    cout << "\nBOIDS BENCHMARK (AMD Ryzen 5 3600X)" << endl;
    cout << "Frames per ogni simulazione: " << FRAMES << endl;
    //livelli di ottimizzazione
    vector<OptimizationLevel> opts = {NAIVE, SQUARED_DIST, BOUNDING_BOX,BOUNDING_BOX_NOT_IF, GRID};
    vector<string> optNames = {"NAIVE", "SQUARED_DIST", "BOUNDING_BOX", "BOUNDING_BOX_NOT_IF", "GRID"};
    //tipi di scheduling
    vector<omp_sched_t> schedulers = {omp_sched_static, omp_sched_dynamic};
    vector<string> schedNames = {"STATICO", "DINAMICO"};
    // apertura del file CSV per il salvataggio dei dati
    ofstream csvFile("benchmark_results.csv");
    if (!csvFile.is_open()) {
        cerr << "Errore: Impossibile creare il file CSV!" << endl;
        return;
    }

    // Scrittura dell'header del CSV
    csvFile << "Ottimizzazione,Scheduling,Boids,Threads,TempoMedio_s,TempoMin_s,TempoMax_s,Speedup,TempoMedioBoid_us\n";
    // loop principale sulle ottimizzazioni
    for (size_t optIdx = 0; optIdx < opts.size(); ++optIdx) {
        //loop sui i tipi di scheduling
        for (size_t schedIdx = 0; schedIdx < schedulers.size(); ++schedIdx) {

            cout << "\n==============================================================" << endl;
            cout << "[ OTTIMIZZAZIONE: " << optNames[optIdx] << " | SCHEDULING: " << schedNames[schedIdx] << " ]" << endl;
            cout << "==============================================================" << endl;

            for (int numBoids : boidCounts) {
                //formatta l'intestazione della tabella
                cout << "\n--> Test con " << numBoids << " boids (Media su " << RUNS << " tentativi):" << endl;
                cout << setw(8)  << "Threads"
                     << setw(15) << "Tempo tot medio(s)"
                     << setw(12) << "Min (s)"
                     << setw(12) << "Max (s)"
                     << setw(12) << "Speedup"
                     << setw(24) << "T. medio/Boid (us)" << endl;
                cout << "--------------------------------------------------------------" << endl;

                double seqTime = 0.0;

                for (int threads : threadCounts) {
                    //tempo totale, massimo e minimo
                    double elapsedAccumulator = 0.0;
                    double minElapsed = 0.0;
                    double maxElapsed = 0.0;

                    //esegue il sotto-test per 5 volte per sicurezza
                    for (int run = 0; run < RUNS; ++run) {
                        vector<Boid> boids;
                        initFlock(boids, numBoids);

                        //imposta lo scheduling sotto test
                        // chunk size di default
                        omp_set_schedule(schedulers[schedIdx], 64);

                        //calcola il tempo di esecuzione facendo la differenza
                        double startTime = omp_get_wtime();
                        for (int frame = 0; frame < FRAMES; ++frame) {
                            //aggiorna i boids
                            updateBoids(boids, threads, opts[optIdx]);
                        }
                        double currentElapsed = omp_get_wtime() - startTime;
                        elapsedAccumulator += currentElapsed;
                        //traccia il tempo minimo e massimo tra i vari tentativi
                        if (run == 0) {
                            minElapsed = currentElapsed;
                            maxElapsed = currentElapsed;
                        } else {
                            if (currentElapsed < minElapsed) minElapsed = currentElapsed;
                            if (currentElapsed > maxElapsed) maxElapsed = currentElapsed;
                        }
                    }
                    // fa la media
                    double avgElapsed = elapsedAccumulator / static_cast<double>(RUNS);

                    //tempo per boid
                    double timePerBoidUs = (avgElapsed * 1000000.0) / (FRAMES * numBoids);
                    //stampa output
                    cout << setw(8) << threads
                         << setw(15) << fixed << setprecision(4) << avgElapsed
                         << setw(12) << fixed << setprecision(4) << minElapsed
                         << setw(12) << fixed << setprecision(4) << maxElapsed;


                    double speedup = 1.00;
                    if (threads == 1) {
                        seqTime = avgElapsed;
                        cout << setw(12) << "1.00x";
                    } else {
                        speedup = seqTime / avgElapsed;
                        cout << setw(11) << fixed << setprecision(2) << speedup << "x";
                    }

                    cout << setw(24) << fixed << setprecision(3) << timePerBoidUs << endl;

                    //salva i dati nel file csv
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
    //ciclo principale
    do{
        cout << "Seleziona Modalita':\n";
        cout << "1. Simulazione Grafica (GUI SFML)\n";
        cout << "2. Esegui Benchmark (Test prestazioni per relazione)\n";
        cout << "3. Esci\n";
        cout << "Scelta: ";
        cin >> choice;

        if (choice == 1) runGUI();
        else if (choice == 2) runBenchmark();
        else cout << "Scelta non valida, riprova.\n";
    }while (choice!= 3);

    return 0;
}