#include "Pipelines.hpp"
//ciclo principale
int main() {
    int choice;
    do {
        cout << "Seleziona Modalita':\n";
        cout << "1. Simulazione Grafica \n";
        cout << "2. Esegui Benchmark\n";
        cout << "3. Esci\n";
        cout << "Scelta: ";
        cin >> choice;

        if (choice == 1) runGUI();
        else if (choice == 2) runBenchmark();
        else if (choice != 3) cout << "Scelta non valida, riprova.\n";
    } while (choice != 3);

    return 0;
}