#include <iostream>
#include "Gerenciador.h"
#include <vector>
#include <unordered_map>
#include <string>
#include <sstream>
#include <fstream>

using namespace std;

int main(int argc, char** argv) 
{
    if (argc < 2) 
    {
        std::cerr << "Uso: ./execGrupo10 <arquivo_entrada>\n";
        return 1;
    }

    std::ifstream file(argv[1]);
    if (!file.is_open()) 
    {
        std::cerr << "Erro ao abrir o arquivo." << std::endl;
        return 1;
    }
    
    int digrafo, ponderado_vertice, ponderado_aresta;
    file >> digrafo >> ponderado_vertice >> ponderado_aresta;

    digrafo = 0;
    ponderado_vertice = 0;
    ponderado_aresta = 0;
    
    int n_vertices;
    file >> n_vertices;

    Grafo* grafo = new Grafo(digrafo, ponderado_aresta, ponderado_vertice);

    for (int i = 0; i < n_vertices; ++i) 
    {
        char id;
        file >> id;
        grafo->addNo(id, 1);  // peso padrão
    }

    char a, b;
    int peso_aresta = 1;
    std::string token;

    while (file >> token) {
        if (token == "end") break;  // fim da leitura de arestas

        a = token[0];
        file >> b;

        if (ponderado_aresta)
            file >> peso_aresta;
        else
            peso_aresta = 1;

        grafo->addAresta(a, b, peso_aresta);
    }

    grafo->exportarDOT("grafo.dot");
    grafo->printGrafo();
    Gerenciador::comandos(grafo);

    delete grafo;  // libera memória
    return 0;
}
