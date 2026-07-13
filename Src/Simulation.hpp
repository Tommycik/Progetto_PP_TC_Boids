#ifndef SIMULATION_H
#define SIMULATION_H

#include "Config.hpp"

void initFlock(vector<Boid>& boids, int N);
void updateBoidsSequential(vector<Boid>& boids, OptimizationLevel opt);
void updateBoids(vector<Boid>& boids, int threads, OptimizationLevel opt);

#endif // SIMULATION_H
