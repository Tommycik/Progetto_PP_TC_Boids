#include "Pipelines.hpp"
#include <SFML/Graphics.hpp>
// Gui
void runGUI() {
    vector<Boid> boids;
    initFlock(boids, 5000);
    //inizializza la finestra
    sf::RenderWindow window(sf::VideoMode(gridWidth, gridHeight), "Boids OpenMP - Ryzen 3600X");
    window.setFramerateLimit(60);
    sf::CircleShape boidShape(3.f, 3);
    boidShape.setFillColor(sf::Color::Cyan);
    omp_set_schedule(omp_sched_static, 64);
    //ciclo principale
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
// Benchmark
void runBenchmark() {
    //ottimizza il benchmark cosi da evitare l'interrompimento del sistema
    #ifdef _WIN32
        SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED);
    #endif
    // varie configurazioni
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
    //crea il file csv
    ofstream csvFile("benchmark_results.csv");
    if (!csvFile.is_open()) {
        cerr << "Errore: Impossibile creare il file CSV!" << endl;
        return;
    }

    csvFile << "Ottimizzazione,Scheduling,Boids,Threads,TempoMedio_s,TempoMin_s,TempoMax_s,Speedup,TempoMedioBoid_us\n";
    //ciclo di test
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
                // testa i vari numeri di thread
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
                    //calcola i valori medi e i tempi per boid
                    double avgElapsed = elapsedAccumulator / static_cast<double>(RUNS);
                    double timePerBoidUs = (avgElapsed * 1000000.0) / (FRAMES * numBoids);

                    cout << setw(8) << threads
                         << setw(22) << fixed << setprecision(4) << avgElapsed
                         << setw(12) << fixed << setprecision(4) << minElapsed
                         << setw(12) << fixed << setprecision(4) << maxElapsed;

                    double speedup = seqTime / avgElapsed;
                    cout << setw(11) << fixed << setprecision(2) << speedup << "x";

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
    //chiude il file csv
    csvFile.close();
}