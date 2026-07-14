#include "Simulation.hpp"
//inizializza il flock di boids
void initFlock(vector<Boid>& boids, int N) {
    boids.clear();
    //inizializza i boids casualmente
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
// aggiorna i boids in base all'ottimizzazione scelta in modo sequenziale
void updateBoidsSequential(vector<Boid>& boids, OptimizationLevel opt) {
    int numBoids = boids.size();
    //inizializza la griglia
    vector<vector<int>> grid(gridCols * gridRows);
    if (opt == GRID) {
        for (int i = 0; i < numBoids; ++i) {
            int cellIdx = getCellIndex(boids[i].getX(), boids[i].getY());
            grid[cellIdx].push_back(i);
        }
    }
    //ciclo di update
    for (int boid = 0; boid < numBoids; ++boid) {
        //inizializza le variabili temporanee
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
        //ottimizzazione grid
        if (opt == GRID) {
            int cx = static_cast<int>(currX / cellSize);
            int cy = static_cast<int>(currY / cellSize);
            //si sposta sulle celle adiacenti
            for (int row = -1; row <= 1; ++row) {
                for (int column = -1; column <= 1; ++column) {
                    int neighborX = cx + column;
                    int neighborY = cy + row;

                    if (neighborX >= 0 && neighborX < gridCols && neighborY >= 0 && neighborY < gridRows) {
                        int cellIdx = neighborY * gridCols + neighborX;
                        const auto& cellBoids = grid[cellIdx];
                        // controlla i boid adiacenti
                        for (int otherId : cellBoids) {
                            if (boid != otherId) {
                                const Boid& other = boids[otherId];
                                float dx = currX - other.getX();
                                float dy = currY - other.getY();
                                //controlla se i boid adiacenti sono all'interno del raggio visivo o protetto
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
                //ignora se sta calcolando il boid stesso
                if (boid == otherId) continue;
                const Boid &other = boids[otherId];
                float dx = currX - other.getX();
                float dy = currY - other.getY();

                bool inProtected = false;
                bool inVisual = false;
                //le varie ottimizzazioni
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
                // controlla se il boid è in protected o in visual range
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
        // calcola i vari contributi
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
    // finalizza l'update
    for (int i = 0; i < numBoids; ++i) {
        boids[i].commit();
    }
}
// aggiorna i boids in base all'ottimizzazione scelta in modo paralelo
void updateBoids(vector<Boid>& boids, int threads, OptimizationLevel opt) {
    int numBoids = boids.size();
    // setta il numero di thread
    omp_set_num_threads(threads);
    // inizializza la griglia
    vector<vector<int>> grid(gridCols * gridRows);

    if (opt == GRID) {
        for (int i = 0; i < numBoids; ++i) {
            int cellIdx = getCellIndex(boids[i].getX(), boids[i].getY());
            grid[cellIdx].push_back(i);
        }
    }
    //regione paralela
    #pragma omp parallel
    {
        // calcola l'update per ogni boid
        #pragma omp for schedule(runtime)
        for (int boid = 0; boid < numBoids; ++boid) {
            // variabili temporanee per l'update
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
            // ottimizzazione grid
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
                #pragma omp simd reduction(+:closeDx,closeDy,xposAvg,yposAvg,xvelAvg,yvelAvg,neighboring_boids)
                for (int otherId = 0; otherId < numBoids; ++otherId) {
                    if (boid == otherId) continue;

                    const Boid &other = boids[otherId];

                    float dx = currX - other.getX();
                    float dy = currY - other.getY();

                    bool inProtected = false;
                    bool inVisual = false;
                    // le varie ottimizzazioni
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
                    // controlla se il boid è in protected o in visual range
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
            // calcola i vari contributi per l'update
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
        // finalizza l'update
        #pragma omp for sschedule(runtime)
        for (int i = 0; i < numBoids; ++i) {
            boids[i].commit();
        }
    }
}