// Grafo.h
#ifndef GRAFO_H
#define GRAFO_H

#include <set>
#include "No.h"
#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <string>

using namespace std;

class Grafo {
public:
    Grafo(bool direcionado, bool ponderado_aresta, bool ponderado_vertice);
    ~Grafo();

    vector<char> fecho_transitivo_direto(int id_no); // a
    void dfs(char atual, set<char>& visitado); // a
    vector<char> fecho_transitivo_indireto(int id_no); // b
    vector<char> caminho_minimo_dijkstra(int id_no_a, int id_no_b); // c
    vector<char> caminho_minimo_floyd(int id_no, int id_no_b); // d
    Grafo* arvore_geradora_minima_prim(vector<char> ids_nos); // e
    Grafo* arvore_geradora_minima_kruskal(vector<char> ids_nos); // f
    Grafo* arvore_caminhamento_profundidade(int id_no); // g
    int raio(); // h 1
    int diametro(); // h 2
    vector<char> centro(); // h 3
    vector<char> periferia(); // h 4
    vector<char> vertices_de_articulacao(); // i

    void addNo(char id, int peso);
    void addAresta(char id_a, char id_b, int peso);
    No* getNo(char id);
    void printGrafo();

    int ordem = 0;
    bool in_direcionado;
    bool in_ponderado_aresta;
    bool in_ponderado_vertice;
    map<char, list<pair<char, int>>> adj;
    map<char, int> pesosVertices;
    bool existeVertice(char id);  // Verifica se um vértice existe
};

#endif // GRAFO_H
