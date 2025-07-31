#include "Gerenciador.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include <list>
#include <string>
#include <algorithm>


void Gerenciador::comandos(Grafo* grafo) {
    cout<<"Digite uma das opcoes abaixo e pressione enter:"<<endl<<endl;
    cout<<"(a) Fecho transitivo direto de um no;"<<endl;
    cout<<"(b) Fecho transitivo indireto de um no;"<<endl;
    cout<<"(c) Caminho minimo (Djikstra);"<<endl;
    cout<<"(d) Caminho minimo (Floyd);"<<endl;
    cout<<"(e) Arvore Geradora Minima (Algoritmo de Prim);"<<endl;
    cout<<"(f) Arvore Geradora Minima (Algoritmo de Kruskal);"<<endl;
    cout<<"(g) Arvore de caminhamento em profundidade;"<<endl;
    cout<<"(h) Raio, diametro, centro e periferia do grafo;"<<endl;
    cout<<"(i) Vertices de articulacao;"<<endl;
    cout<<"(j) Algoritmo guloso;"<<endl;
    cout<<"(k) Algoritmo guloso randomizado adaptativo;"<<endl;
    cout<<"(l) Algoritmo guloso randomizado adaptativo reativo;;"<<endl;
    cout<<"(0) Sair;"<<endl<<endl;

    char resp;
    cin >> resp;
    switch (resp) {
        case 'a': {
            char id_no = get_id_entrada();
            vector<char> resultado = grafo->fecho_transitivo_direto(id_no);
            cout << "Fecho transitivo direto: ";
            imprimir_vetor(resultado);
            if (pergunta_imprimir_arquivo("fecho_trans_dir.txt"))
                salvar_em_arquivo(resultado, "fecho_trans_dir.txt");
            break;
        }

        case 'b':{
            char id_no = get_id_entrada();
            vector<char> resultado = grafo->fecho_transitivo_indireto(id_no);
            cout << "Fecho transitivo indireto: ";
            imprimir_vetor(resultado);

            if (pergunta_imprimir_arquivo("fecho_trans_indir.txt"))
                salvar_em_arquivo(resultado, "fecho_trans_indir.txt");
            break;
        }

        case 'c': {
            char origem = get_id_entrada();
            char destino = get_id_entrada();
            vector<char> resultado = grafo->caminho_minimo_dijkstra(origem, destino);
            cout << "Caminho minimo (Dijkstra): ";
            imprimir_vetor(resultado);
            if (pergunta_imprimir_arquivo("caminho_minimo_dijkstra.txt"))
                salvar_em_arquivo(resultado, "caminho_minimo_dijkstra.txt");
            break;
        }

        case 'd': {
            char origem = get_id_entrada();
            char destino = get_id_entrada();
            vector<char> resultado = grafo->caminho_minimo_floyd(origem, destino);
            cout << "Caminho minimo (Floyd): ";
            imprimir_vetor(resultado);

            if (pergunta_imprimir_arquivo("caminho_minimo_floyd.txt"))
                salvar_em_arquivo(resultado, "caminho_minimo_floyd.txt");
            break;
        }
        case 'e': {
            int tam;
            cout << "Digite o tamanho do subconjunto: ";
            cin >> tam;
            if (tam > 0 && tam <= grafo->ordem) {
                vector<char> ids = get_conjunto_ids(grafo, tam);
                Grafo* agm = grafo->arvore_geradora_minima_prim(ids);
                cout << "AGM - Prim:" << endl;
                agm->printGrafo();
                if (pergunta_imprimir_arquivo("agm_prim.txt"))
                    salvar_arestas_em_arquivo(agm, "agm_prim.txt");
                delete agm;
            } else {
                cout << "Valor invalido" << endl;
            }
            break;
        }
        case 'f': {
            int tam;
            cout << "Digite o tamanho do subconjunto: ";
            cin >> tam;
            if (tam > 0 && tam <= grafo->ordem) {
                vector<char> ids = get_conjunto_ids(grafo, tam);
                Grafo* agm = grafo->arvore_geradora_minima_kruskal(ids);
                cout << "AGM - Kruskal:" << endl;
                agm->printGrafo();
                if (pergunta_imprimir_arquivo("agm_kruskal.txt"))
                    salvar_arestas_em_arquivo(agm, "agm_kruskal.txt");
                delete agm;
            } else {
                cout << "Valor invalido" << endl;
            }
            break;
        }
        case 'g': {
            char id_no = get_id_entrada();
            Grafo* dfs_tree = grafo->arvore_caminhamento_profundidade(id_no);
            cout << "Arvore de Caminhamento em Profundidade:" << endl;
            dfs_tree->printGrafo();
            if (pergunta_imprimir_arquivo("arvore_dfs.txt"))
                salvar_arestas_em_arquivo(dfs_tree, "arvore_dfs.txt");
            delete dfs_tree;
            break;
        }
        case 'h': {
            cout << "Raio: " << grafo->raio() << endl;
            cout << "Diâmetro: " << grafo->diametro() << endl;
            cout << "Centro: ";
            imprimir_vetor(grafo->centro());
            cout << "Periferia: ";
            imprimir_vetor(grafo->periferia());
            break;
        }

        case 'i': {
            vector<char> resultado = grafo->vertices_de_articulacao();
            cout << "Vértices de articulação: ";
            Gerenciador::imprimir_vetor(resultado);
            if (Gerenciador::pergunta_imprimir_arquivo("vertices_articulacao.txt"))
                Gerenciador::salvar_em_arquivo(resultado, "vertices_articulacao.txt");
            break;
        }
        case 'j': {
            Grafo* guloso = grafo->guloso();
            cout << "grafo guloso:" << endl;
            guloso->printGrafo();
            if (pergunta_imprimir_arquivo("guloso.txt"))
                salvar_arestas_em_arquivo(guloso, "guloso.txt");
            delete guloso;
            break;
        }
        case 'k': {
            Grafo* guloso_rando = grafo->guloso_rando();
            cout << "grafo guloso randomizado adaptativo:" << endl;
            guloso_rando->printGrafo();
            if (pergunta_imprimir_arquivo("guloso_rando.txt"))
                salvar_arestas_em_arquivo(guloso_rando, "guloso_rando.txt");
            delete guloso_rando;
            break;
        }
        case 'l': {
            Grafo* guloso_randoreat = grafo->guloso_randoreat();
            cout << "grafo guloso randomizado adaptativo reativo:" << endl;
            guloso_randoreat->printGrafo();
            if (pergunta_imprimir_arquivo("guloso_randoreat.txt"))
                salvar_arestas_em_arquivo(guloso_randoreat, "guloso_randoreat.txt");
            delete guloso_randoreat;
            break;
        }

        case '0': {
            exit(0);
        }
        default: {
            cout<<"Opcao invalida"<<endl;
        }
    }

    comandos(grafo);

}

char Gerenciador::get_id_entrada() {
    cout<<"Digite o id de um no: ";
    char id;
    cin>>id;
    cout<<endl;
    return id;
}

vector<char> Gerenciador::get_conjunto_ids(Grafo* grafo, int tam) {
    vector<char> ids;

    while ((int)ids.size() < tam) {
        char id_no = get_id_entrada();

        bool existe = grafo->existeVertice(id_no);
        if (!existe) {
            cout << "Vertice nao existe" << endl;
        } else {
            bool repetido = find(ids.begin(), ids.end(), id_no) != ids.end();
            if (repetido) {
                cout << "Valor repetido" << endl;
            } else {
                ids.push_back(id_no);
            }
        }
    }

    return ids;
}


bool Gerenciador::pergunta_imprimir_arquivo(string nome_arquivo) {

    cout<<"Imprimir em arquivo externo? ("<<nome_arquivo<<")"<<endl;
    cout<<"(1) Sim;"<<endl;
    cout<<"(2) Nao."<<endl;
    int resp;
    cin>>resp;
    cout<<endl;

    switch (resp) {
        case 1:
            return true;
        case 2:
            return false;
        default:
            cout<<"Resposta invalida"<<endl;
            return pergunta_imprimir_arquivo(nome_arquivo);
    }
}
void Gerenciador::imprimir_vetor(const vector<char>& v) {
    for (size_t i = 0; i < v.size(); ++i)
        cout << v[i] << " ";
    cout << endl << endl;
}

void Gerenciador::salvar_em_arquivo(const vector<char>& v, const string& nome_arquivo) {
    ofstream out(nome_arquivo.c_str());
    for (size_t i = 0; i < v.size(); ++i)
        out << v[i] << " ";
    out.close();
}

void Gerenciador::salvar_arestas_em_arquivo(Grafo* g, const string& nome_arquivo) {
    ofstream out(nome_arquivo.c_str());
    for (map<char, list<pair<char, int>>>::iterator it = g->adj.begin(); it != g->adj.end(); ++it) {
        for (list<pair<char, int>>::iterator vit = it->second.begin(); vit != it->second.end(); ++vit) {
            out << it->first << " " << vit->first << " " << vit->second << endl;
        }
    }
    out.close();
}
