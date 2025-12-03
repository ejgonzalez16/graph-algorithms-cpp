#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <string>
#include <cmath>
#include "Grafo.h" // Asegúrate de incluir correctamente el archivo de encabezado o la definición de la clase Grafo

std::list<std::list<std::pair<float, float>>> leerCircuitos(const std::string& filename);
void hacerGrafo(Grafo<std::pair<float, float>>& grafo, std::list<std::pair<float, float>> circuito);
void hacerAristas(Grafo<std::pair<float, float>>& grafo);
float distancia(std::pair<float, float> agujero1, std::pair<float, float> agujero2);
void escribirArchivo(const std::string& filename, const std::string& bufferSalida);

int main(int argc, char* argv[]) {
    // TODO #1: recibir correctamente los archivos TXT
    if (argc < 3) {
        std::cerr << "Uso: ./" << argv[0] << " archivo_entrada.txt archivo_salida.txt" << std::endl;
        return(-1);
    }
    std::string archivoEntrada(argv[1]);
    std::string archivoSalida(argv[2]);
    if (archivoEntrada.size() < 5 || archivoSalida.size() < 5) {
        perror("Los archivos de entrada y salida deben contener extension '.txt'");
        exit(-1);
    }
    if (archivoEntrada.substr(archivoEntrada.length() - 4, archivoEntrada.length()) != ".txt") {
        perror("El archivo de entrada debe contener extension '.txt'");
        exit(-1);
    }
    if (archivoSalida.substr(archivoSalida.length() - 4, archivoSalida.length()) != ".txt") {
        perror("El archivo de salida debe contener extension '.txt'");
        exit(-1);
    }

    std::list<std::list<std::pair<float, float>>> circuitos = leerCircuitos(argv[1]);
    std::list<std::list<std::pair<float, float>>>::iterator it = circuitos.begin();

    // TODO #8: almacenar en un buffer la información que contendrá el archivo de salida
    std::string bufferSalida = std::to_string(circuitos.size()) + "\n";
    std::cout << "Cantidad de circuitos en el archivo: " << circuitos.size() << std::endl;
    int contCircuito = 1;
    for (; it != circuitos.end(); it++) {
        Grafo<std::pair<float, float>> grafo;
        hacerGrafo(grafo, *it);
        // TODO #5: encontrar el recorrido más corto del circuito de agujeros
        std::vector< std::pair<float, float> > agujerosOrdenados = grafo.circuitoVertices(std::make_pair(0, 0));
        // TODO #6: encontrar la distancia total del recorrido más corto del circuito de agujeros
        float distanciaTotal = grafo.distanciaRecorrido(agujerosOrdenados);
        agujerosOrdenados.erase(agujerosOrdenados.end() - 1);
        agujerosOrdenados.erase(agujerosOrdenados.begin());
        // TODO #8: almacenar en un buffer la información que contendrá el archivo de salida
        bufferSalida += std::to_string(agujerosOrdenados.size()) + "\n";
        // TODO #7: informar en pantalla la cantidad de agujeros para cada circuito,
        //          y la distancia total recorrida por el taladro al momento de perforar los agujeros de cada circuito
        std::cout << "\nCantidad de agujeros circuito " << contCircuito << ": " << agujerosOrdenados.size()
            << "\nDistancia total del circuito " << contCircuito << ": " << distanciaTotal << "\n";
        // TODO #8: almacenar en un buffer la información que contendrá el archivo de salida
        for (int i = 0; i < agujerosOrdenados.size(); i++) {
            bufferSalida += std::to_string(agujerosOrdenados[i].first) + " " + std::to_string(agujerosOrdenados[i].second) + "\n";
        }
        contCircuito++;
    }

    escribirArchivo(argv[2], bufferSalida);

    return 0;
}

// TODO #2: leer correctamente el archivo TXT de entrada
std::list<std::list<std::pair<float, float>>> leerCircuitos(const std::string& filename) {
    std::ifstream input(filename.c_str());
    if (!input) {
        perror("No se pudo abrir el archivo especificado");
        exit(-1);
    }

    int numeroDeCircuitos = 0, numeroAgujeros = 0;
    float coordenadaX, coordenadaY;
    std::list<std::list<std::pair<float, float>>> circuitos;
    input >> numeroDeCircuitos;
    for (int i = 0; i < numeroDeCircuitos; i++) {
        input >> numeroAgujeros;
        std::list<std::pair<float, float>> circuito;
        for (int j = 0; j < numeroAgujeros; j++) {
            input >> coordenadaX >> coordenadaY;
            //Crea y añade la coordenada del agujero al circuito
            std::pair<float, float> coordenada(coordenadaX, coordenadaY);
            circuito.push_back(coordenada);
        }
        // Añade el circuito a la lista de circuitos
        circuitos.push_back(circuito);
    }
    //Cierra el archivo
    input.close();
    return circuitos;
}

void hacerGrafo(Grafo<std::pair<float, float>>& grafo, std::list<std::pair<float, float>> circuito) {
    // TODO #3: construir correctamente los vértices de los grafos
    grafo.insertarVertice(std::make_pair(0, 0));
    std::list<std::pair<float, float>>::iterator it = circuito.begin();
    for (; it != circuito.end(); it++) {
        grafo.insertarVertice(*it);
    }
    // TODO #4: construir correctamente las aristas de los grafos
    hacerAristas(grafo);
}

// TODO #4: construir correctamente las aristas de los grafos
void hacerAristas(Grafo<std::pair<float, float>>& grafo) {
    std::vector< std::pair<float, float>> vertices = grafo.getVertices();
    for (int i = 0; i < vertices.size(); i++) {
        for (int j = 0; j < vertices.size(); j++) {
            if (j != i) {
                float distanciaAgujeros = distancia(vertices[i], vertices[j]);
                grafo.insertarArista(vertices[i], vertices[j], distanciaAgujeros);
            }
        }
    }
}

float distancia(std::pair<float, float> agujero1, std::pair<float, float> agujero2) {
    float dx = agujero2.first - agujero1.first;
    float dy = agujero2.second - agujero1.second;
    return std::sqrt(dx * dx + dy * dy);
}

// TODO #9: escribir el archivo de salida con la información almacenada en el buffer
void escribirArchivo(const std::string& filename, const std::string& bufferSalida) {
    std::ofstream output(filename.c_str());
    if (!output) {
        perror("No se pudo crear o abrir el archivo especificado");
        exit(-1);
    }
    
    output << bufferSalida;

    //Cierra el archivo
    output.close();
}