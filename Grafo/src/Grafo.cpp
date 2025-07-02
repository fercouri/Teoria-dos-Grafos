#include "Grafo.h"
#include <set>
#include "No.h"
#include <vector>
#include <list>
#include <stack>
#include <queue>
#include <tuple>
#include <algorithm>
#include <functional>
#include <climits>

using namespace std;

Grafo::Grafo(bool direcionado, bool ponderado_aresta, bool ponderado_vertice) 
{
in_direcionado = direcionado;
in_ponderado_aresta = ponderado_aresta;
in_ponderado_vertice = ponderado_vertice;
}

Grafo::~Grafo() 
{

}
void Grafo::dfs(char atual, std::set<char>& visitado) 
{
    visitado.insert(atual);
    for (auto vizinho : adj[atual]) {
        if (visitado.find(vizinho.first) == visitado.end()) 
        {
            dfs(vizinho.first, visitado);
        }
    }
}

vector<char> Grafo::fecho_transitivo_direto(int id_no) 
{
    set<char> visitado;
    dfs((char)id_no, visitado);
    visitado.erase((char)id_no); // opcional
    return vector<char>(visitado.begin(), visitado.end());
}

vector<char> Grafo::fecho_transitivo_indireto(int id_no) 
{
    map<char, list<pair<char, int>>> transposto;
    for (auto& par : adj) 
    {
        for (auto& viz : par.second) 
        {
            transposto[viz.first].push_back({par.first, viz.second});
        }
    }
    set<char> visitado;
    stack<char> pilha;
    pilha.push((char)id_no);
    while (!pilha.empty()) 
    {
        char atual = pilha.top(); pilha.pop();
        if (visitado.find(atual) == visitado.end()) 
        {
            visitado.insert(atual);
            for (auto& viz : transposto[atual]) 
            {
                pilha.push(viz.first);
            }
        }
    }
    visitado.erase((char)id_no);
    return vector<char>(visitado.begin(), visitado.end());
}

vector<char> Grafo::caminho_minimo_dijkstra(int id_no_a, int id_no_b)
{
    map<char, int> dist;
    map<char, char> anterior;
    set<char> vertices;
    char origem = (char)id_no_a;
    char destino = (char)id_no_b;

    for (auto& par : adj) 
    {
        dist[par.first] = INT_MAX;
        vertices.insert(par.first);
    }
    dist[origem] = 0;

    while (!vertices.empty()) 
    {
        char u = *vertices.begin();
        for (char v : vertices) 
        {
            if (dist[v] < dist[u]) u = v;
        }
        vertices.erase(u);
        for (auto viz : adj[u]) 
        {
            char v = viz.first;
            int peso = viz.second;
            if (dist[u] != INT_MAX && dist[u] + peso < dist[v]) 
            {
                dist[v] = dist[u] + peso;
                anterior[v] = u;
            }
        }
    }

    vector<char> caminho;
    if (dist[destino] == INT_MAX) return caminho;
    for (char at = destino; at != origem; at = anterior[at]) 
    {
        caminho.push_back(at);
    }
    caminho.push_back(origem);
    reverse(caminho.begin(), caminho.end());
    return caminho;
}

vector<char> Grafo::caminho_minimo_floyd(int id_no_a, int id_no_b) 
{
    map<char, map<char, int>> dist;
    map<char, map<char, char>> prox;

    for (auto& u : adj) {
        for (auto& v : adj) 
        {
            if (u.first == v.first) dist[u.first][v.first] = 0;
            else dist[u.first][v.first] = INT_MAX;
        }
    }

    for (auto& u : adj) 
    {
        for (auto& viz : u.second) 
        {
            dist[u.first][viz.first] = viz.second;
            prox[u.first][viz.first] = viz.first;
        }
    }

    for (auto& k : adj) 
    {
        for (auto& i : adj) 
        {
            for (auto& j : adj) 
            {
                if (dist[i.first][k.first] == INT_MAX || dist[k.first][j.first] == INT_MAX) continue;
                if (dist[i.first][j.first] > dist[i.first][k.first] + dist[k.first][j.first]) {
                    dist[i.first][j.first] = dist[i.first][k.first] + dist[k.first][j.first];
                    prox[i.first][j.first] = prox[i.first][k.first];
                }
            }
        }
    }

    char u = (char)id_no_a;
    char v = (char)id_no_b;
    vector<char> caminho;
    if (prox[u][v] == 0) return caminho;
    caminho.push_back(u);
    while (u != v) {
        u = prox[u][v];
        caminho.push_back(u);
    }
    return caminho;
}

Grafo* Grafo::arvore_geradora_minima_prim(vector<char> ids_nos) {
    if (in_direcionado) return nullptr;

    set<char> em_agm;
    using Aresta = tuple<int, char, char>;
    priority_queue<Aresta, vector<Aresta>, greater<>> fila;
    Grafo* agm = new Grafo(false, true, false);

    for (char id : ids_nos) {
        agm->addNo(id, 0);
    }

    if (ids_nos.empty()) return agm;
    char inicial = ids_nos[0];
    em_agm.insert(inicial);

    for (auto [viz, peso] : adj[inicial]) {
        if (agm->existeVertice(viz))
            fila.emplace(peso, inicial, viz);
    }

    while (em_agm.size() < ids_nos.size() && !fila.empty()) {
        auto [peso, u, v] = fila.top(); fila.pop();
        if (em_agm.find(v) != em_agm.end()) continue;
        agm->addAresta(u, v, peso);
        em_agm.insert(v);
        for (auto [viz, p] : adj[v]) {
            if (agm->existeVertice(viz) && em_agm.find(viz) == em_agm.end())
                fila.emplace(p, v, viz);
        }
    }

    return agm;
}

Grafo* Grafo::arvore_geradora_minima_kruskal(vector<char> ids_nos) {
    if (in_direcionado) return nullptr;

    Grafo* agm = new Grafo(false, true, false);
    for (char id : ids_nos)
        agm->addNo(id, 0);

    struct Aresta { char u, v; int peso; };
    vector<Aresta> arestas;

    for (char u : ids_nos) {
        for (auto [v, p] : adj[u]) {
            if (u < v && agm->existeVertice(v)) {
                arestas.push_back({u, v, p});
            }
        }
    }

    sort(arestas.begin(), arestas.end(), [](Aresta a, Aresta b) {
        return a.peso < b.peso;
    });

    map<char, char> pai;
    for (char id : ids_nos) pai[id] = id;

    auto find = [&](char x) -> char {
        while (pai[x] != x) {
            pai[x] = pai[pai[x]];
            x = pai[x];
        }
        return x;
    };

    auto unir = [&](char a, char b) {
        pai[find(a)] = find(b);
    };

    for (auto [u, v, peso] : arestas) {
        if (find(u) != find(v)) {
            agm->addAresta(u, v, peso);
            unir(u, v);
        }
    }

    return agm;
}

Grafo* Grafo::arvore_caminhamento_profundidade(int id_no) {
    char origem = (char)id_no;
    Grafo* arvore = new Grafo(in_direcionado, in_ponderado_aresta, in_ponderado_vertice);

    std::set<char> visitado;
    std::set<char> em_pilha;

    for (const auto& par : adj) 
    {
        arvore->addNo(par.first, pesosVertices[par.first]);
    }

    std::function<void(char)> dfs;
    dfs = [&](char u) 
    {
        visitado.insert(u);
        em_pilha.insert(u);

        for (auto [v, peso] : adj[u]) {
            if (visitado.find(v) == visitado.end()) 
            {
                arvore->addAresta(u, v, peso);
                dfs(v);
            } else if (em_pilha.find(v) != em_pilha.end()) 
            {
                std::cout << "Aresta de retorno: " << u << " -> " << v << std::endl;
            }
        }
        em_pilha.erase(u);
    };

    dfs(origem);
    return arvore;
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

    cout << "\nLista de Adjac�ncia:\n";
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

