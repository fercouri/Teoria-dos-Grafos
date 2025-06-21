#include <iostream>
#include "Gerenciador.h"
#include <vector>
#include <unordered_map>
#include <string>
#include <sstream>

using namespace std;
int main(int argc, char *argv[])
{
cout << "Digite o tipo de grafo: " << endl;
cout << "direcionado(0 ou 1), " << endl;
cout << "aresta ponderada(0 ou 1) " << endl;
cout << "e vertice ponderado(0 ou 1): " << endl;
bool d,pa,pv;  int num_vertices;
cin>> d >> pa >> pv;
    Grafo* grafo = new Grafo(d,pa,pv);

    cout << "Entre o número de vértices: ";
    cin >> num_vertices;

    for (int i = 0; i < num_vertices; ++i) {
        char id;
        int peso = 1; // Default

        cin >> id;
        if (pv == 1) {
            cin >> peso; // Peso opcional para vértices ponderados
        }

        grafo->addNo(id, peso);
    }
    cout << endl;
    cout << "---------------------" << endl;
    char id_a, id_b;
    int peso = 1; // Default
    for(int i=0; i<num_vertices; i++) {
        cin >> id_a >> id_b;
        if (pa == 1) { // Se for ponderado
            cin >> peso;
        }
        grafo->addAresta(id_a, id_b, peso); // Assume que o ID é único em um caractere
    }

    grafo->printGrafo();
    Gerenciador::comandos(grafo);

    return 0;
}
