#include "Grafo.h"
#include <set>
#include "No.h"
#include <vector>
#include <list>
using namespace std;
Grafo::Grafo(bool direcionado, bool ponderado_aresta, bool ponderado_vertice) {
in_direcionado = direcionado;
in_ponderado_aresta = ponderado_aresta;
in_ponderado_vertice = ponderado_vertice;

}

Grafo::~Grafo() {

}
void Grafo::dfs(char atual, std::set<char>& visitado) {
    visitado.insert(atual);

    for (auto vizinho : adj[atual]) {
        if (visitado.find(vizinho.first) == visitado.end()) {
            dfs(vizinho.first, visitado);
        }
    }
}
vector<char> Grafo::fecho_transitivo_direto(int id_no) {
    set<char> visitado;

    // Começa a DFS a partir de id_no
    dfs(id_no, visitado);

    // Remove o nó inicial se quiser só os outros nós
    visitado.erase(id_no);

    // Converte para vector<char>
    vector<char> resultado(visitado.begin(), visitado.end());
    return resultado;
}

vector<char> Grafo::fecho_transitivo_indireto(int id_no) {
    cout<<"Metodo nao implementado"<<endl;
    return {};
}

vector<char> Grafo::caminho_minimo_dijkstra(int id_no_a, int id_no_b) {
    cout<<"Metodo nao implementado"<<endl;
    return {};
}

vector<char> Grafo::caminho_minimo_floyd(int id_no, int id_no_b) {
    cout<<"Metodo nao implementado"<<endl;
    return {};
}

Grafo * Grafo::arvore_geradora_minima_prim(vector<char> ids_nos) {
    cout<<"Metodo nao implementado"<<endl;
    return nullptr;
}

Grafo * Grafo::arvore_geradora_minima_kruskal(vector<char> ids_nos) {
    cout<<"Metodo nao implementado"<<endl;
    return nullptr;
}

Grafo * Grafo::arvore_caminhamento_profundidade(int id_no) {
    cout<<"Metodo nao implementado"<<endl;
    return nullptr;
}

int Grafo::raio() {
    cout<<"Metodo nao implementado"<<endl;
    return 0;
}

int Grafo::diametro() {
    cout<<"Metodo nao implementado"<<endl;
    return 0;
}

vector<char> Grafo::centro() {
    cout<<"Metodo nao implementado"<<endl;
    return {};
}

vector<char> Grafo::periferia() {
    cout<<"Metodo nao implementado"<<endl;
    return {};
}

vector<char> Grafo::vertices_de_articulacao() {
    cout<<"Metodo nao implementado"<<endl;
    return {};
}


void Grafo::addNo(char id, int peso) {
    if (in_ponderado_vertice)
        pesosVertices[id] = peso;
    else
        pesosVertices[id] = 1;

    if (adj.find(id) == adj.end()) {
        adj[id] = list<pair<char, int>>();
    }
ordem++;
}


void Grafo::addAresta(char origem, char destino, int peso) {
    int pesoUsado = in_ponderado_aresta ? peso : 1;
    adj[origem].push_back({destino, pesoUsado});
    if (!in_direcionado) {
        adj[destino].push_back({origem, pesoUsado});
    }
}

No* Grafo::getNo(char id) {

}

void Grafo::printGrafo() {
    cout << "Grafo direcionado: " << in_direcionado << endl;
    cout << "Arestas ponderadas: " << in_ponderado_aresta << endl;
    cout << "Vertices ponderados: " << in_ponderado_vertice << endl;
    cout << "Ordem do grafo: " << ordem;

    cout << "\nLista de Adjacência:\n";
    for (auto& par : adj) {
        char vertice = par.first;
        cout << vertice;
        if (in_ponderado_vertice)
            cout << "(peso: " << pesosVertices[vertice] << ")";
        cout << " -> ";
        for (auto& vizinho : par.second) {
            cout << vizinho.first;
            if (in_ponderado_aresta)
                cout << "(peso: " << vizinho.second << ")";
            cout << " -> ";
        }
        cout << endl;
    }
}

bool Grafo::existeVertice(char id) {
    return adj.find(id) != adj.end();

}
