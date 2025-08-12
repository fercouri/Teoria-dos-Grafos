#include "Grafo.h"
#include "No.h"
#include <set>
#include <vector>
#include <list>
#include <stack>
#include <queue>
#include <tuple>
#include <algorithm>
#include <functional>
#include <climits>
#include <limits>
#include <random>
#include <chrono>    
#include <map>
#include <fstream>
#include <iostream>

using namespace std;
using namespace std::chrono;

Grafo::Grafo(bool direcionado, bool ponderado_aresta, bool ponderado_vertice)
{
    in_direcionado = direcionado;
    in_ponderado_aresta = ponderado_aresta;
    in_ponderado_vertice = ponderado_vertice;
    ordem = 0;
}

Grafo::~Grafo() {}

void Grafo::exportarDOT(const std::string& nome_arquivo) const {
    std::ofstream out(nome_arquivo);
    if (!out.is_open()) {
        std::cerr << "Erro ao abrir arquivo DOT." << std::endl;
        return;
    }

    out << "graph G {\n";
    out << "    node [shape=circle, style=filled, fillcolor=lightblue];\n";

    // Lista de arestas sem duplicar
    for (auto it = adj.begin(); it != adj.end(); ++it) {
        int u = it->first;
        const auto& vizinhos = it->second;

        for (auto vit = vizinhos.begin(); vit != vizinhos.end(); ++vit) {
            int v = vit->first;  // pega o primeiro do pair (v, _)
            if (u < v) { // evita duplicar em não direcionado
                out << "    " << static_cast<char>(u) << " -- " << static_cast<char>(v) << ";\n";

            }
        }
    }

    out << "}\n";
    out.close();

    std::cout << "Arquivo DOT gerado: " << nome_arquivo << std::endl;
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

vector<char> Grafo::vertices_de_articulacao()
{
    map<char, bool> visitado;
    map<char, int> disc, low;
    map<char, char> pai;
    set<char> articulacoes;
    int tempo = 0;
    for (map<char, list<pair<char, int>>>::iterator it = adj.begin(); it != adj.end(); ++it) {
        visitado[it->first] = false;
        pai[it->first] = 0;
    }

    function<void(char)> dfs = [&](char u)
    {
        visitado[u] = true;
        disc[u] = low[u] = ++tempo;
        int filhos = 0;
        for (list<pair<char, int>>::iterator it = adj[u].begin(); it != adj[u].end(); ++it) {
            char v = it->first;
            if (!visitado[v])
            {
                filhos++;
                pai[v] = u;
                dfs(v);
                low[u] = min(low[u], low[v]);
                // raiz com dois ou mais filhos
                if (pai[u] == 0 && filhos > 1)
                    articulacoes.insert(u);
                // não raiz com low[v] >= disc[u]
                if (pai[u] != 0 && low[v] >= disc[u])
                    articulacoes.insert(u);
            }
            else if (v != pai[u])
            {
                low[u] = min(low[u], disc[v]);
            }
        }
    };
    for (map<char, list<pair<char, int>>>::iterator it = adj.begin(); it != adj.end(); ++it)
    {
        char u = it->first;
        if (!visitado[u])
            dfs(u);
    }
    return vector<char>(articulacoes.begin(), articulacoes.end());
}

Grafo* Grafo::guloso()
{
    Grafo* resultado = new Grafo(false, false, false);  // Cria um novo grafo apenas para retornar os vertices no final
    set<char> Conjdomin;
    set<char> dominados;
    vector<char> vertices;

    auto inicio = high_resolution_clock::now();
    for (auto& entry : adj) {  // Adiciona todos os vértices
        vertices.push_back(entry.first);
    }
    sort(vertices.begin(), vertices.end(), [this](char a, char b) {return adj[a].size() > adj[b].size();}); // Ordena por grau decrescente

    for (char v : vertices) {
        // Verifica se o vértice não está dominado e não é adjacente a nenhum no conjunto
        if (dominados.count(v) == 0) {  // Se vértice não está dominado
            bool isAdjacent = false;
            for (auto& neighbor : adj[v]) {  // Verifica se é adjacente a algum vértice do conjunto
                if (Conjdomin.count(neighbor.first)) {
                    isAdjacent = true;
                    break;
                }
            }

            if (!isAdjacent) {
                Conjdomin.insert(v);  // Adiciona ao conjunto
                resultado->addNo(v, 1);  // Adiciona o vertice ao grafo a ser retornado
                dominados.insert(v);   // Marca como dominado

                for (auto& neighbor : adj[v]) {  // Marca vizinhos como dominados
                    dominados.insert(neighbor.first);
                }
            }
        }
    }
    auto fim = high_resolution_clock::now();
    auto duracao = duration_cast<milliseconds>(fim - inicio);
    float tempo = duracao.count();
    cout << "Tempo de execucao algoritmo guloso: " << tempo << " milissegundos" << endl;
    return resultado;

}

Grafo* Grafo::guloso_rando(float alpha)
{

    int iteracoes = 500;
    Grafo* melhor_resultado = new Grafo(false, false, false); // Cria um novo grafo apenas para retornar os vertices no final
    set<char> melhor_conjunto;
    size_t menor_tamanho = numeric_limits<size_t>::max();


    vector<char> todos_vertices;
    for (auto& entry : adj) {
        todos_vertices.push_back(entry.first);
    }

    auto inicio = high_resolution_clock::now();

    for (int i = 0; i < iteracoes; i++) {
        Grafo* resultado_atual = new Grafo(false, false, false);
        set<char> conjunto_atual;
        set<char> dominados;
        vector<char> vertices = todos_vertices; // Cópia para ordenação

        sort(vertices.begin(), vertices.end(), [this](char a, char b) {return adj[a].size() > adj[b].size();}); // Ordena vértices em grau decrescente

        while (dominados.size() < ordem) {
            vector<pair<char, int>> candidatos;

            // Encontra todos os vértices que podem ser adicionados (não adjacentes ao conjunto atual)
            for (char v : vertices) {
                if (dominados.count(v) == 0 && !adjacente_a_conjunto(v, conjunto_atual)) {
                    // Calcula quantos vértices não dominados seriam cobertos
                    int cobertura = 1; // O próprio vértice
                    for (auto& neighbor : adj[v]) {
                        if (dominados.count(neighbor.first) == 0) {
                            cobertura++;
                        }
                    }
                    candidatos.emplace_back(v, cobertura);
                }
            }

            if (candidatos.empty()) break;

            sort(candidatos.begin(), candidatos.end(),[](const pair<char, int>& a, const pair<char, int>& b) {return a.second > b.second;}); // Ordena candidatos por cobertura decrescente

            int max_cobertura = candidatos[0].second;
            int min_cobertura = candidatos.back().second;
            int limite = max_cobertura - alpha * (max_cobertura - min_cobertura);

            vector<char> rcl;
            for (auto& cand : candidatos) {
                if (cand.second >= limite) {
                    rcl.push_back(cand.first);
                }
            }

            if (!rcl.empty()) {  // Seleciona aleatoriamente da RCL
                random_device rd;
                mt19937 gen(rd());
                uniform_int_distribution<> dist(0, rcl.size() - 1);
                char selecionado = rcl[dist(gen)];

                conjunto_atual.insert(selecionado); // Adiciona ao conjunto
                resultado_atual->addNo(selecionado, 1);
                dominados.insert(selecionado);

                for (auto& neighbor : adj[selecionado]) { // Marca vizinhos como dominados
                    dominados.insert(neighbor.first);
                }
            }
        }

        if (eh_ids(conjunto_atual)) {
            if (conjunto_atual.size() < menor_tamanho) {
                menor_tamanho = conjunto_atual.size();
                delete melhor_resultado; // Libera o anterior
                melhor_resultado = resultado_atual;
                melhor_conjunto = conjunto_atual;
            } else {
                delete resultado_atual;
            }
        } else {
            delete resultado_atual;
        }
    }

    auto fim = high_resolution_clock::now();
    auto duracao = duration_cast<milliseconds>(fim - inicio);
    float tempo = duracao.count();
    cout << "Tempo de execucao: " << tempo << " milissegundos" << endl;

    return melhor_resultado;
}

bool Grafo::adjacente_a_conjunto(char v, const set<char>& conjunto) {
    for (auto& neighbor : adj[v]) {
        if (conjunto.count(neighbor.first)) {
            return true;
        }
    }
    return false;
}

bool Grafo::eh_ids(const set<char>& conjunto) const {
    // Verifica independência
    for (char u : conjunto) {
        for (const auto& viz : adj.at(u)) {
            if (conjunto.count(viz.first)) return false; // dois do conjunto são vizinhos
        }
    }

    // Verifica dominância
    set<char> dominados = conjunto;
    for (char u : conjunto) {
        for (const auto& viz : adj.at(u)) {
            dominados.insert(viz.first);
        }
    }

    return dominados.size() == ordem;
}

Grafo* Grafo::guloso_rando_reativo() {
    auto inicio = high_resolution_clock::now();

    vector<float> alfas = {0.05f, 0.1f, 0.15f, 0.3f, 0.5f};
    vector<float> probs(alfas.size(), 1.0f / alfas.size());
    vector<float> qualidades(alfas.size(), 0.0f);
    vector<int> usos(alfas.size(), 0);

    int bloco = 10;
    int iteracoes = 2500;

    Grafo* melhor_resultado = new Grafo(false, false, false);
    set<char> melhor_conjunto;
    size_t menor_tamanho = numeric_limits<size_t>::max();

    vector<char> todos_vertices;
    for (auto& entry : adj) {
        todos_vertices.push_back(entry.first);
    }

    for (int it = 0; it < iteracoes; it++) {
        // Escolher alpha de forma probabilística
        random_device rd;
        mt19937 gen(rd());
        discrete_distribution<> dist_alpha(probs.begin(), probs.end());
        int idx_alpha = dist_alpha(gen);
        float alpha = alfas[idx_alpha];
        usos[idx_alpha]++;

        Grafo* resultado_atual = new Grafo(false, false, false);
        set<char> conjunto_atual;
        set<char> dominados;
        vector<char> vertices = todos_vertices;

        sort(vertices.begin(), vertices.end(), [this](char a, char b) {
            return adj[a].size() > adj[b].size();
        });

        while (dominados.size() < ordem) {
            vector<pair<char, int>> candidatos;

            for (char v : vertices) {
                if (dominados.count(v) == 0 && !adjacente_a_conjunto(v, conjunto_atual)) {
                    int cobertura = 1;
                    for (auto& neighbor : adj[v]) {
                        if (dominados.count(neighbor.first) == 0) {
                            cobertura++;
                        }
                    }
                    candidatos.emplace_back(v, cobertura);
                }
            }

            if (candidatos.empty()) break;

            sort(candidatos.begin(), candidatos.end(), [](auto& a, auto& b) {
                return a.second > b.second;
            });

            int max_cobertura = candidatos[0].second;
            int min_cobertura = candidatos.back().second;
            int limite = max_cobertura - alpha * (max_cobertura - min_cobertura);

            vector<char> rcl;
            for (auto& cand : candidatos) {
                if (cand.second >= limite) {
                    rcl.push_back(cand.first);
                }
            }

            if (!rcl.empty()) {
                uniform_int_distribution<> dist(0, rcl.size() - 1);
                char selecionado = rcl[dist(gen)];

                conjunto_atual.insert(selecionado);
                resultado_atual->addNo(selecionado, 1);
                dominados.insert(selecionado);
                for (auto& neighbor : adj[selecionado]) {
                    dominados.insert(neighbor.first);
                }
            }
        }

        if (eh_ids(conjunto_atual)) {
            if (conjunto_atual.size() < menor_tamanho) {
                menor_tamanho = conjunto_atual.size();
                delete melhor_resultado; // Libera o anterior
                melhor_resultado = resultado_atual;
                melhor_conjunto = conjunto_atual;
            } else {
                delete resultado_atual;
            }
        } else {
            delete resultado_atual;
        }

        // Atualiza probabilidades a cada bloco
        float qualidade_iter = 1.0f / conjunto_atual.size();
        qualidades[idx_alpha] += qualidade_iter;

        if ((it + 1) % bloco == 0) {
            float soma_qualidades = 0;
            vector<float> medias(alfas.size(), 0.0);

            for (size_t i = 0; i < alfas.size(); ++i) {
                if (usos[i] > 0) {
                    medias[i] = qualidades[i] / usos[i];
                } else {
                    medias[i] = 0.01f; // penalização artificial
                }
                soma_qualidades += medias[i];
            }

            for (size_t i = 0; i < alfas.size(); ++i) {
                probs[i] = medias[i] / soma_qualidades;
                qualidades[i] = 0;
                usos[i] = 0;
            }
        }
    }

    auto fim = high_resolution_clock::now();
    auto duracao = duration_cast<milliseconds>(fim - inicio);
    cout << "Tempo de execucao (guloso randomizado reativo): " 
         << duracao.count() << " ms" << endl;

    return melhor_resultado;
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
