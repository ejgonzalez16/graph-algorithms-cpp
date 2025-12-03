#ifndef __Grafo_h__
#define __Grafo_h__
#include <vector>
#include <list>
#include <map>
#include <iterator>
#include <string>
#include <deque>

template<class T>
class Grafo {
protected:
	std::vector<T> vertices;
	std::vector < std::list< std::pair<T, float >>> adyacencia;
	void swap(int i);
	bool insertarAristaOrdenada(std::pair<T, float> aristaNueva, int posicionOrigen, std::vector < std::list< std::pair<T, float >>>& adyacencia);
	template <class U>
	int buscar(U vec, T elem);
	std::vector<T> cendientes(T origen, std::vector < std::list< std::pair<T, float >>> adyacencia);
	std::vector < std::list< std::pair<T, float >>> adyacenciaInversa();
	bool insertarArista(T origen, T destino, float peso, std::vector < std::list< std::pair<T, float >>>& adyacencia);
	std::vector<T> componenteConectado(std::vector<T> descendientes, std::vector<T> ascendientes, std::vector<T>& noVisitados);
	void nuevoMinimoPrim(std::pair<T, float > aristaActual, float& pesoMinimo, std::pair<T, T>& aristaMinima, T verticeVisitando);
	void nuevoMinimoCircuitos(std::pair<T, float > aristaActual, float& pesoMinimo, T& verticeMinimo);
	void hallarDestinoMasCercano(int posicionActual, std::vector<T> verticesIda, std::deque<T> verticesVuelta,
		float& pesoMinimo, T& verticeMinimo, T verticeBaneado);
public:
	Grafo();
	Grafo(std::vector<T> vertices,
		std::vector < std::list< std::pair<T, float >>> adyacencia);
	std::vector<T> getVertices();
	bool insertarVertice(T vertice);
	int buscarVertice(T vertice);
	void ordenarVertices();
	bool insertarArista(T origen, T destino, float peso);
	float buscarArista(T origen, T destino);
	typename std::list< std::pair<T, float >>::iterator buscarAristaIt(T destino, int posicionOrigen);
	bool eliminarVertice(T vertice);
	bool eliminarArista(T origen, T destino);
	void recorridoPlano();
	void recorridoProfundidad(T origen);
	void recorridoAnchura(T origen);
	int cantidadComponentesConectados();
	std::list<std::vector<T>> componentesConectados();
	int cantidadAristasPuente();
	std::vector< std::pair<T, T> > aristasPuente();
	std::string nivelConexion();
	std::vector<std::pair<T, T>> prim(T origen);
	std::vector<std::pair<T, T>> dijkstra(T origen);
	std::vector<T> circuitoVertices(T origen);
	float distanciaRecorrido(std::vector<T> verticesRecorrido);
};
#include "Grafo.hxx"
#endif