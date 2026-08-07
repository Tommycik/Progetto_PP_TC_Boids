# Boids OpenMP Simulation

## Project overview

This project implements a two-dimensional boids simulation and studies the effect of CPU parallelism on flocking behaviour. Each boid updates its motion from three rules. Cohesion moves it toward the local group. Separation avoids very close neighbours. Alignment adjusts its direction toward the direction of the surrounding flock.

The project contains a sequential implementation and an OpenMP implementation. It also compares four neighbour-search profiles. The objective is not only to obtain a faster simulation. The benchmark separates the improvement produced by the algorithm from the improvement produced by additional CPU threads.

## Main implementations

The project evaluates the following profiles.

- **Naive** scans the complete flock and calculates the Euclidean distance with a square root.
- **Squared Distance** removes the square root and compares squared distances.
- **Bounding Box** rejects distant boids before the complete distance test.
- **Grid** divides the simulation area into spatial cells and checks only the current cell and the adjacent cells.

Naive, Squared Distance and Bounding Box keep an all-pairs structure. Grid reduces the number of candidate neighbours before the force calculation starts.

## Parallel structure

Each frame uses two distinct phases. The first phase calculates the pending position and velocity of every boid. All threads read the same current flock and each iteration writes only the pending state of one boid. The second phase commits the pending values after the first phase has completed.

This structure preserves the same frame semantics used by the sequential implementation. It also avoids locks, critical sections and atomic operations. The OpenMP version distributes the outer boid loop with `schedule(runtime)`. The benchmark combines static and dynamic scheduling with chunk sizes 1, 8, 32, 64, 128 and 256.

## Graphical simulation

The graphical mode uses SFML. The default configuration creates 5000 boids in a 1000 by 700 window. It uses the Grid profile, 12 OpenMP threads and dynamic scheduling. The window is limited to 60 frames per second.

The graphical mode is intended to verify the behaviour of the flock and to show that the optimized implementation preserves the expected motion.

## Benchmark

The benchmark executes 50 frames for every configuration and repeats each test five times. It evaluates:

- populations of 2000, 5000, 10000 and 20000 boids;
- 1, 2, 4, 6, 8, 12, 18 and 24 OpenMP threads;
- Naive, Squared Distance, Bounding Box and Grid;
- static and dynamic OpenMP scheduling;
- chunk sizes 1, 8, 32, 64, 128 and 256.

A separate sequential baseline is calculated for every population and optimization profile. The reported speedup therefore compares an OpenMP configuration with the corresponding sequential implementation.

The benchmark writes `benchmark_results.csv` in the program working directory. The file contains the optimization, scheduling policy, number of boids, thread count, chunk size, number of chunks, mean time, minimum time, maximum time, speedup and average time per boid.

## Project structure

- `main.cpp` contains the interactive menu.
- `Pipelines.cpp` contains the graphical mode and benchmark routine.
- `Simulation.cpp` contains the sequential and OpenMP update implementations.
- `Config.hpp` contains the simulation constants and optimization enumeration.
- `Entities/Boid.cpp` and `Entities/Boid.h` define the boid state and pending values.
- `CMakeLists.txt` configures C++17, OpenMP and SFML.

## Requirements

The project requires:

- a C++17 compiler;
- CMake;
- OpenMP support;
- an internet connection during the first CMake configuration because SFML 2.6.1 is downloaded with `FetchContent`.

A Release build must be used for meaningful performance results. Debug builds disable important compiler optimizations and are not suitable for the report.

## Use from CLion

1. Open the project directory in CLion.
2. Wait for the CMake configuration and the SFML download to finish.
3. Select the `Project_1_TC` target.
4. Select the Release configuration.
5. Run the target from CLion.

The program does not require command-line arguments. It displays this menu:

```text
1. Simulazione Grafica (GUI SFML)
2. Esegui Benchmark (Test prestazioni per relazione)
3. Esci
```

Choose the graphical mode to inspect the flock. Choose the benchmark mode to generate the complete CSV dataset.

## Output

The main generated file is:

```text
benchmark_results.csv
```

Its location depends on the working directory selected by CLion. With the default CLion configuration it is normally created inside the selected build directory.

## Interpretation of results

The one-thread OpenMP configuration includes the cost of the parallel region and is not identical to the sequential baseline. Thread scaling is most useful up to the physical-core and logical-thread region of the processor, configurations above the available hardware contexts represent oversubscription and may provide little improvement.

The Grid profile must be interpreted separately from thread-level speedup, it changes the amount of work by reducing the neighbor candidates. OpenMP changes how the remaining work is distributed. The largest total improvement is obtained when both effects are combined.

Chunk size changes the scheduling granularity for the same profile, scheduler, population and thread count. Small chunks create more work assignments. Large chunks reduce assignment overhead but provide fewer opportunities to redistribute uneven iterations.
