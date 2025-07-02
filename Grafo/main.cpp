#include "Grafo.h"
#include <fstream>
#include <sstream>

void exibir_vetor(const vector<char>& v) 
{
    for (char c : v) std::cout << c << " ";
    std::cout << std::endl;
}

void salvar_em_arquivo(const vector<char>& v, const string& nome_funcao) 
{
    std::ofstream out(nome_funcao + ".txt");
    for (char c : v) out << c << " ";
    out.close();
}

int main(int argc, char** argv) 
{
    if (argc < 2) 
    {
        std::cerr << "Uso: ./execGrupoX <arquivo_entrada>\n";
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
    int n_vertices;
    file >> n_vertices;

    Grafo g(digrafo, ponderado_aresta, ponderado_vertice);

    for (int i = 0; i < n_vertices; ++i) 
    {
        char id;
        int peso = 0;
        file >> id;
        if (ponderado_vertice) file >> peso;
        g.addNo(id, peso);
    }

    char a, b;
    int peso_aresta = 1;
    while (file >> a >> b) 
    {
        if (ponderado_aresta) file >> peso_aresta;
        g.addAresta(a, b, peso_aresta);
    }

    int opcao;
    do 
    {
        std::cout << "\n--- MENU ---\n";
        std::cout << "1. Fecho transitivo direto\n";
        std::cout << "2. Fecho transitivo indireto\n";
        std::cout << "3. Caminho mínimo (Dijkstra)\n";
        std::cout << "4. Caminho mínimo (Floyd-Warshall)\n";
        std::cout << "5. Árvore Geradora Mínima (Prim)\n";
        std::cout << "6. Árvore Geradora Mínima (Kruskal)\n";
        std::cout << "7. Busca em profundidade (com arestas de retorno)\n";
        std::cout << "0. Sair\n";
        std::cout << "Escolha: ";
        std::cin >> opcao;

        if (opcao == 1) 
        {
            char id;
            std::cout << "ID do vértice: ";
            std::cin >> id;
            auto resultado = g.fecho_transitivo_direto(id);
            exibir_vetor(resultado);
            std::cout << "Salvar resultado? (s/n): ";
            char salvar;
            std::cin >> salvar;
            if (salvar == 's') salvar_em_arquivo(resultado, "fecho_direto");
        } 
        else if (opcao == 2) 
        {
            char id;
            std::cout << "ID do vértice: ";
            std::cin >> id;
            auto resultado = g.fecho_transitivo_indireto(id);
            exibir_vetor(resultado);
            std::cout << "Salvar resultado? (s/n): ";
            char salvar;
            std::cin >> salvar;
            if (salvar == 's') salvar_em_arquivo(resultado, "fecho_indireto");
        } 
        else if (opcao == 3) 
        {
            char origem, destino;
            std::cout << "Origem: ";
            std::cin >> origem;
            std::cout << "Destino: ";
            std::cin >> destino;
            auto caminho = g.caminho_minimo_dijkstra(origem, destino);
            exibir_vetor(caminho);
            std::cout << "Salvar resultado? (s/n): ";
            char salvar;
            std::cin >> salvar;
            if (salvar == 's') salvar_em_arquivo(caminho, "dijkstra");
        }
        else if (opcao == 4) 
        {
            char origem, destino;
            std::cout << "Origem: ";
            std::cin >> origem;
            std::cout << "Destino: ";
            std::cin >> destino;
            auto caminho = g.caminho_minimo_floyd(origem, destino);
            exibir_vetor(caminho);
            std::cout << "Salvar resultado? (s/n): ";
            char salvar;
            std::cin >> salvar;
            if (salvar == 's') salvar_em_arquivo(caminho, "floyd");
        } 
        else if (opcao == 5) 
        {
            std::cout << "Digite os vértices do subconjunto X (sem espaços, ex: ABCD): ";
            std::string linha;
            std::cin >> linha;
            std::vector<char> subconjunto(linha.begin(), linha.end());
            Grafo* agm = g.arvore_geradora_minima_prim(subconjunto);
            if (agm == nullptr) 
            {
                std::cout << "AGM não pode ser gerada em grafos direcionados.\n";
                continue;
            }
            std::cout << "AGM (Prim):\n";
            agm->printGrafo();
            std::cout << "Salvar resultado? (s/n): ";
            char salvar;
            std::cin >> salvar;
            if (salvar == 's') 
            {
                std::ofstream out("agm_prim.txt");
                for (const auto& par : agm->adj) 
                {
                    for (const auto& viz : par.second) 
                    {
                        out << par.first << " " << viz.first << " " << viz.second << "\n";
                    }
                }
                out.close();
            }
            delete agm;
        } 
        else if (opcao == 6) 
        {
            std::cout << "Digite os vértices do subconjunto X (sem espaços, ex: ABCD): ";
            string linha;
            std::cin >> linha;
            vector<char> subconjunto(linha.begin(), linha.end());
            Grafo* agm = g.arvore_geradora_minima_kruskal(subconjunto);
            if (agm == nullptr) 
            {
                std::cout << "AGM não pode ser gerada em grafos direcionados.\n";
                continue;
            }
            std::cout << "AGM (Kruskal):\n";
            agm->printGrafo();
            std::cout << "Salvar resultado? (s/n): ";
            char salvar;
            std::cin >> salvar;
            if (salvar == 's') 
            {
                std::ofstream out("agm_kruskal.txt");
                for (const auto& par : agm->adj) 
                {
                    for (const auto& viz : par.second) 
                    {
                        out << par.first << " " << viz.first << " " << viz.second << "\n";
                    }
                }
                out.close();
            }
            delete agm;
        }
        else if (opcao == 7) {
            char origem;
            std::cout << "Digite o vértice de origem: ";
            std::cin >> origem;
            Grafo* arvore = g.arvore_caminhamento_profundidade(origem);
            std::cout << "Árvore de Caminhamento (DFS):\n";
            arvore->printGrafo();
            std::cout << "Salvar resultado? (s/n): ";
            char salvar;
            std::cin >> salvar;
            if (salvar == 's') {
                std::ofstream out("dfs_arvore.txt");
                for (const auto& par : arvore->adj) {
                    for (const auto& viz : par.second) {
                        out << par.first << " " << viz.first << " " << viz.second << "\n";
                    }
                }
                out.close();
            }
            delete arvore;
        }
    }
    
    while (opcao != 0);

    return 0;
}
