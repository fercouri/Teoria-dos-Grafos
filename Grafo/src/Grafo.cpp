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
    typedef tuple<int, char, char> Aresta;
    priority_queue<Aresta, vector<Aresta>, greater<Aresta>> fila;
    Grafo* agm = new Grafo(false, true, false);

    for (size_t i = 0; i < ids_nos.size(); ++i)
        agm->addNo(ids_nos[i], 0);

    if (ids_nos.empty()) return agm;
    char inicial = ids_nos[0];
    em_agm.insert(inicial);

    for (list<pair<char, int>>::iterator it = adj[inicial].begin(); it != adj[inicial].end(); ++it) {
        if (agm->existeVertice(it->first))
            fila.push(make_tuple(it->second, inicial, it->first));
    }

    while (em_agm.size() < ids_nos.size() && !fila.empty()) {
        int peso;
        char u, v;
        tie(peso, u, v) = fila.top(); fila.pop();

        if (em_agm.find(v) != em_agm.end()) continue;

        agm->addAresta(u, v, peso);
        em_agm.insert(v);

        for (list<pair<char, int>>::iterator it = adj[v].begin(); it != adj[v].end(); ++it) {
            if (agm->existeVertice(it->first) && em_agm.find(it->first) == em_agm.end()) {
                fila.push(make_tuple(it->second, v, it->first));
            }
        }
    }

    return agm;
}

Grafo* Grafo::arvore_geradora_minima_kruskal(vector<char> ids_nos) {
    if (in_direcionado) return nullptr;

    Grafo* agm = new Grafo(false, true, false);
    for (size_t i = 0; i < ids_nos.size(); ++i)
        agm->addNo(ids_nos[i], 0);

    struct Aresta { char u, v; int peso; };
    vector<Aresta> arestas;

    for (size_t i = 0; i < ids_nos.size(); ++i) {
        char u = ids_nos[i];
        for (list<pair<char, int>>::iterator it = adj[u].begin(); it != adj[u].end(); ++it) {
            char v = it->first;
            int peso = it->second;
            if (u < v && agm->existeVertice(v)) {
                arestas.push_back((Aresta){u, v, peso});
            }
        }
    }

    sort(arestas.begin(), arestas.end(), [](const Aresta& a, const Aresta& b) {
        return a.peso < b.peso;
    });

    map<char, char> pai;
    for (size_t i = 0; i < ids_nos.size(); ++i) pai[ids_nos[i]] = ids_nos[i];

    function<char(char)> find = [&](char x) {
        while (pai[x] != x) {
            pai[x] = pai[pai[x]];
            x = pai[x];
        }
        return x;
    };

    function<void(char, char)> unir = [&](char a, char b) {
        pai[find(a)] = find(b);
    };

    for (size_t i = 0; i < arestas.size(); ++i) {
        char u = arestas[i].u;
        char v = arestas[i].v;
        int peso = arestas[i].peso;
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

    set<char> visitado;
    set<char> em_pilha;

    for (map<char, list<pair<char, int>>>::iterator it = adj.begin(); it != adj.end(); ++it) {
        arvore->addNo(it->first, pesosVertices[it->first]);
    }

    function<void(char)> dfs = [&](char u) {
        visitado.insert(u);
        em_pilha.insert(u);

        for (list<pair<char, int>>::iterator it = adj[u].begin(); it != adj[u].end(); ++it) {
            char v = it->first;
            int peso = it->second;

            if (visitado.find(v) == visitado.end()) {
                arvore->addAresta(u, v, peso);
                dfs(v);
            } else if (em_pilha.find(v) != em_pilha.end()) {
                cout << "Aresta de retorno: " << u << " -> " << v << endl;
            }
        }

        em_pilha.erase(u);
    };

    dfs(origem);

    return arvore;
}

int Grafo::raio() 
{
    int r = INT_MAX;
    for (map<char, list<pair<char, int>>>::iterator it = adj.begin(); it != adj.end(); ++it) 
    {
        char u = it->first;
        vector<char> ignorar;
        int exc = 0;
        map<char, int> dist;
        for (map<char, list<pair<char, int>>>::iterator jt = adj.begin(); jt != adj.end(); ++jt)
            dist[jt->first] = INT_MAX;
        dist[u] = 0;

        set<char> visitados;
        while (visitados.size() < adj.size()) {
            char atual = 0;
            int menor = INT_MAX;
            for (map<char, int>::iterator dt = dist.begin(); dt != dist.end(); ++dt) {
                if (visitados.find(dt->first) == visitados.end() && dt->second < menor) {
                    menor = dt->second;
                    atual = dt->first;
                }
            }

            if (atual == 0) break;
            visitados.insert(atual);

            for (list<pair<char, int>>::iterator vit = adj[atual].begin(); vit != adj[atual].end(); ++vit) {
                char v = vit->first;
                int peso = vit->second;
                if (dist[atual] + peso < dist[v])
                    dist[v] = dist[atual] + peso;
            }
        }

        for (map<char, int>::iterator dt = dist.begin(); dt != dist.end(); ++dt) {
            if (dt->second != INT_MAX && dt->second > exc)
                exc = dt->second;
        }

        if (exc < r) r = exc;
    }
    return r;
}

int Grafo::diametro() 
{
    int d = 0;
    for (map<char, list<pair<char, int>>>::iterator it = adj.begin(); it != adj.end(); ++it) 
    {
        char u = it->first;
        vector<char> ignorar;
        int exc = 0;
        map<char, int> dist;
        for (map<char, list<pair<char, int>>>::iterator jt = adj.begin(); jt != adj.end(); ++jt)
            dist[jt->first] = INT_MAX;
        dist[u] = 0;

        set<char> visitados;
        while (visitados.size() < adj.size()) {
            char atual = 0;
            int menor = INT_MAX;
            for (map<char, int>::iterator dt = dist.begin(); dt != dist.end(); ++dt) 
            {
                if (visitados.find(dt->first) == visitados.end() && dt->second < menor) 
                {
                    menor = dt->second;
                    atual = dt->first;
                }
            }

            if (atual == 0) break;
            visitados.insert(atual);

            for (list<pair<char, int>>::iterator vit = adj[atual].begin(); vit != adj[atual].end(); ++vit) 
            {
                char v = vit->first;
                int peso = vit->second;
                if (dist[atual] + peso < dist[v])
                    dist[v] = dist[atual] + peso;
            }
        }

        for (map<char, int>::iterator dt = dist.begin(); dt != dist.end(); ++dt) 
        {
            if (dt->second != INT_MAX && dt->second > exc)
                exc = dt->second;
        }

        if (exc > d) d = exc;
    }
    return d;
}

vector<char> Grafo::centro() 
{
    vector<char> resultado;
    int r = raio();
    for (map<char, list<pair<char, int>>>::iterator it = adj.begin(); it != adj.end(); ++it) 
    {
        char u = it->first;
        map<char, int> dist;
        for (map<char, list<pair<char, int>>>::iterator jt = adj.begin(); jt != adj.end(); ++jt)
            dist[jt->first] = INT_MAX;
        dist[u] = 0;

        set<char> visitados;
        while (visitados.size() < adj.size()) 
        {
            char atual = 0;
            int menor = INT_MAX;
            for (map<char, int>::iterator dt = dist.begin(); dt != dist.end(); ++dt) 
            {
                if (visitados.find(dt->first) == visitados.end() && dt->second < menor) 
                {
                    menor = dt->second;
                    atual = dt->first;
                }
            }
            if (atual == 0) break;
            visitados.insert(atual);
            for (list<pair<char, int>>::iterator vit = adj[atual].begin(); vit != adj[atual].end(); ++vit) 
            {
                if (dist[atual] + vit->second < dist[vit->first])
                    dist[vit->first] = dist[atual] + vit->second;
            }
        }
        int exc = 0;
        for (map<char, int>::iterator dt = dist.begin(); dt != dist.end(); ++dt)
            if (dt->second != INT_MAX && dt->second > exc)
                exc = dt->second;

        if (exc == r) resultado.push_back(u);
    }
    return resultado;
}

vector<char> Grafo::periferia() 
{
    vector<char> resultado;
    int d = diametro();
    for (map<char, list<pair<char, int>>>::iterator it = adj.begin(); it != adj.end(); ++it) 
    {
        char u = it->first;
        map<char, int> dist;
        for (map<char, list<pair<char, int>>>::iterator jt = adj.begin(); jt != adj.end(); ++jt)
            dist[jt->first] = INT_MAX;
        dist[u] = 0;

        set<char> visitados;
        while (visitados.size() < adj.size()) 
        {
            char atual = 0;
            int menor = INT_MAX;
            for (map<char, int>::iterator dt = dist.begin(); dt != dist.end(); ++dt) 
            {
                if (visitados.find(dt->first) == visitados.end() && dt->second < menor) 
                {
                    menor = dt->second;
                    atual = dt->first;
                }
            }
            if (atual == 0) break;
            visitados.insert(atual);
            for (list<pair<char, int>>::iterator vit = adj[atual].begin(); vit != adj[atual].end(); ++vit) 
            {
                if (dist[atual] + vit->second < dist[vit->first])
                    dist[vit->first] = dist[atual] + vit->second;
            }
        }
        int exc = 0;
        for (map<char, int>::iterator dt = dist.begin(); dt != dist.end(); ++dt)
            if (dt->second != INT_MAX && dt->second > exc)
                exc = dt->second;

        if (exc == d) resultado.push_back(u);
    }
    return resultado;
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

    cout << "\nLista de Adjacencia:\n";
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
