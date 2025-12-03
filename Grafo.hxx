#include "Grafo.h"
#include <queue>
#include <stack>
#include <cmath>

template <class T>
Grafo<T>::Grafo() {}

template <class T>
Grafo<T>::Grafo(std::vector<T> vertices,
	std::vector < std::list< std::pair<T, float >>> adyacencia) {
	this->vertices = vertices;
	this->adyacencia = adyacencia;
}

template <class T>
std::vector<T> Grafo<T>::getVertices() {
	return this->vertices;
}

template <class T>
bool Grafo<T>::insertarVertice(T vertice) {
	if (buscarVertice(vertice) == -1) { //LO INSERTA SI NO EXISTE
		this->vertices.push_back(vertice);
		std::list< std::pair<T, float >> temp;
		adyacencia.push_back(temp);
		ordenarVertices(); //LO ORDENA
		return true;
	}
	return false;
}

template <class T>
int Grafo<T>::buscarVertice(T vertice) {
	int izquierda = 0, derecha = vertices.size() - 1, mitad;
	//hasta que se haya partido todo el vector a la mitad
	while (izquierda <= derecha) {
		//La mitad es izquierda y la cantidad de posiciones a la derecha partido dos
		mitad = izquierda + (derecha - izquierda) / 2;
		//Si lo encuentra en la mitad lo retorna
		if (vertices[mitad] == vertice) {
			return mitad;
		}
		//Descarta la mitad izquierda
		if (vertices[mitad] < vertice) {
			izquierda = mitad + 1;
		}
		//Descarta la mitad derecha
		else {
			derecha = mitad - 1;
		}
	}
	return -1;
}

template <class T>
void Grafo<T>::ordenarVertices() {
	bool ordenado = false;
	//ORDENA EL ULTIMO PARA UBICARLO DONDE VA
	//Asume que el único desordenado es el último (el recien insertado)
	for (int i = this->vertices.size() - 1; i > 0 && !ordenado; i--) {
		if (this->vertices[i] < this->vertices[i - 1]) {
			swap(i);
		}
	}
}

template <class T>
void Grafo<T>::swap(int i) {
	//HACE UN SWAP A LAS ARISTAS Y VERTICES
	T temp = this->vertices[i - 1];
	this->vertices[i - 1] = this->vertices[i];
	this->vertices[i] = temp;
	std::list< std::pair<T, float >> temporal = this->adyacencia[i];
	this->adyacencia[i - 1] = this->adyacencia[i];
	this->adyacencia[i] = temporal;
}

template <class T>
bool Grafo<T>::insertarArista(T origen, T destino, float peso) {
	return insertarArista(origen, destino, peso, this->adyacencia);
}

template <class T>
bool Grafo<T>::insertarArista(T origen, T destino, float peso, std::vector < std::list< std::pair<T, float >>>& adyacenciaModif) {
	std::pair<T, float> aristaNueva(destino, peso);
	//Retorna falso si no existen los vertices
	int posicionOrigen = buscarVertice(origen);
	if (posicionOrigen == -1 || buscarVertice(destino) == -1) {
		return false;
	}
	//Si ya tiene destinos se mira para ordenar los destinos como es debido
	if (adyacenciaModif[posicionOrigen].size() != 0) {
		typename std::list< std::pair<T, float >>::iterator it = adyacenciaModif[posicionOrigen].begin();
		//Si el destino es menor al primer elemento de la lista lo inserta de primero (para tenerlo ordenado)
		if ((*it).first > destino) {
			adyacenciaModif[posicionOrigen].push_front(aristaNueva);
		}
		else {
			return insertarAristaOrdenada(aristaNueva, posicionOrigen, adyacenciaModif);
		}
	}
	else {
		//Lo inserta al final ya que no toca ordenar
		adyacenciaModif[posicionOrigen].push_front(aristaNueva);
	}
	return true;
}

template <class T>
bool Grafo<T>::insertarAristaOrdenada(std::pair<T, float> aristaNueva, int posicionOrigen, std::vector < std::list< std::pair<T, float >>>& adyacenciaModif) {
	//Recorrer la lista de aristas
	typename std::list< std::pair<T, float >>::iterator it = adyacenciaModif[posicionOrigen].begin();
	for (; it != adyacenciaModif[posicionOrigen].end(); it++) {
		//Si la encuentra no la inserta para evitar repetidos
		if ((*it).first == aristaNueva.first) {
			return false;
		}
		//Si el siguiente es mayor lo pone en el medio ya que encontró su lugar :D
		else if ((*it).first > aristaNueva.first) {
			adyacenciaModif[posicionOrigen].insert(it, aristaNueva);
			return true;
		}
	}
	//Si llega al final la pone al final (es el mayor)
	adyacenciaModif[posicionOrigen].push_back(aristaNueva);
	return true;
}

template <class T>
float Grafo<T>::buscarArista(T origen, T destino) {
	//Busca vértice origen
	int posicionOrigen = buscarVertice(origen);
	//Si el origen o destino no existe retorna 0 (no existe la arista)
	if (posicionOrigen == -1 || buscarVertice(destino) == -1) {
		return 0;
	}
	typename std::list< std::pair<T, float >>::iterator it = adyacencia[posicionOrigen].begin();
	//Recorre el vector de adyacencia en la posición encontrada hasta que encuentre el destino
	for (; it != adyacencia[posicionOrigen].end(); it++) {
		if ((*it).first == destino) {
			//Retorna el peso
			return (*it).second;
		}
	}
	//Si no le encuentra nada 0 (no existe la arista)
	return 0;
}

template <class T>
bool Grafo<T>::eliminarVertice(T vertice) {
	int posicionVertice = buscarVertice(vertice);
	//Verificar que el vértice esté en el grafo
	if (posicionVertice != -1) {
		for (int i = 0; i < adyacencia.size(); i++) {
			typename std::list< std::pair<T, float >>::iterator it = adyacencia[i].begin();
			/*Revisar todas las listas internas, buscando pares que
			contengan el vértice y eliminándolos*/
			for (; it != adyacencia[i].end(); it++) {
				if ((*it).first == vertice) {
					adyacencia[i].erase(it);
					break;
				}
			}
		}
		//Utilizar el índice del vértice para extraer la lista correspondiente y eliminarla
		typename std::vector < std::list< std::pair<T, float >>>::iterator itA = adyacencia.begin();
		adyacencia.erase(itA + posicionVertice);
		//Eliminar el vértice del vector
		typename std::vector <T>::iterator itV = vertices.begin();
		vertices.erase(itV + posicionVertice);
	}
	return false;
}

template <class T>
bool Grafo<T>::eliminarArista(T origen, T destino) {
	int posicionOrigen = buscarVertice(origen);
	if (posicionOrigen != -1) {
		// Buscar en esa lista un par que contenga el índice del vértice destino
		typename std::list< std::pair<T, float >>::iterator itElim = buscarAristaIt(destino, posicionOrigen);
		// Si existe, eliminarlo
		if (itElim != adyacencia[posicionOrigen].end()) {
			// Utilizar el índice del vértice origen para extraer la lista correspondiente
			adyacencia[posicionOrigen].erase(itElim);
			return true;
		}
	}
	return false;
}

//Busca y retorna el iterador de una arista, pasandole la posición de origen
template <class T>
typename
std::list< std::pair<T, float >>::iterator Grafo<T>::buscarAristaIt(T destino, int posicionOrigen) {
	//Busca si existe el vertice de origen
	if (buscarVertice(destino) == -1) {
		return adyacencia[posicionOrigen].end();
	}
	typename std::list< std::pair<T, float >>::iterator it = adyacencia[posicionOrigen].begin();
	for (; it != adyacencia[posicionOrigen].end(); it++) {
		//Retorna el iterador cuando ve que el destino es el correspondiente
		if ((*it).first == destino) {
			return it;
		}
	}
	//Retorna el final "null" para indicar que no se encontró (no se puede retornar null)
	return adyacencia[posicionOrigen].end();
}

template <class T>
void Grafo<T>::recorridoPlano() {
	//Imprime todos los vertices a lo guarro
	for (int i = 0; i < vertices.size(); i++) {
		std::cout << vertices[i] << " ";
	}
	std::cout << std::endl;
}

template <class T>
void Grafo<T>::recorridoProfundidad(T origen) {
	if (buscarVertice(origen) != -1) { //Si el origen no existe no hay recorrido :(
		std::vector<T> verticesRecorridos;
		//Una pila para insertar los valores en orden inverso y de esta forma hacer el recorrido
		std::stack<T> pilaVertices;
		//Inserta el origen
		pilaVertices.push(origen);
		while (!pilaVertices.empty()) {
			T verticeActual = pilaVertices.top();
			//Busca el index del vertice actual y recorrer
			int posicionActual = buscarVertice(verticeActual);
			//Dice que ya se recorrio el vertice actual
			if (buscar(verticesRecorridos, verticeActual) == -1) {
				std::cout << verticeActual << " ";
				verticesRecorridos.push_back(verticeActual);
			}
			//Elimina el vertice actual de la pila
			pilaVertices.pop();
			//recorre a los hijos en orden inverso
			typename std::list< std::pair<T, float >>::reverse_iterator it = adyacencia[posicionActual].rbegin();
			for (; it != adyacencia[posicionActual].rend(); it++) {
				//Si no se ha recorrido el vertice lo pone arriba para imprimir
				if (buscar(verticesRecorridos, (*it).first) == -1) {
					pilaVertices.push((*it).first);
				}
			}
		}
		std::cout << std::endl;
	}
}

template <class T>
void Grafo<T>::recorridoAnchura(T origen) {
	if (buscarVertice(origen) != -1) { //Si el origen no existe no hay recorrido :(
		std::vector<T> verticesRecorridos;
		//Una cola para insertar de forma que los hijos se van poniendo en orden en que se encuentran los hijos
		std::queue<T> colaVertices;
		//Lo inserta al final
		colaVertices.push(origen);
		//Indica que el origen ya se recorrió
		verticesRecorridos.push_back(origen);
		T verticeActual;
		int posicionActual;
		while (!colaVertices.empty()) {
			//Se saca el vertice actual
			verticeActual = colaVertices.front();
			//Busca la posición del vertice actual para recorrer
			posicionActual = buscarVertice(verticeActual);
			//Lo imprime
			std::cout << verticeActual << " ";
			//Lo quita de la cola
			colaVertices.pop();
			//recorre normalmente
			typename std::list< std::pair<T, float >>::iterator it = adyacencia[posicionActual].begin();
			for (; it != adyacencia[posicionActual].end(); it++) {
				//Busca si no está en vertices recorridos para evitar repetición
				if (buscar(verticesRecorridos, (*it).first) == -1) {
					//Pone los hijos en recorridos y en la cola y el vector
					colaVertices.push((*it).first);
					verticesRecorridos.push_back((*it).first);
				}
			}
		}
		std::cout << std::endl;
	}
}

template <class T>
template <class U>
int Grafo<T>::buscar(U vec, T elem) {
	//Busqueda uno a uno en un vector no ordenado
	for (int i = 0; i < vec.size(); i++) {
		if (vec[i] == elem) {
			return i;
		}
	}
	return -1;
}

template <class T>
int Grafo<T>::cantidadComponentesConectados() {
	return componentesConectados().size();
}

template <class T>
std::list<std::vector<T>> Grafo<T>::componentesConectados() {
	std::vector<T> noVisitados = vertices;
	std::list<std::vector<T>> componentes;
	while (!noVisitados.empty()) {
		//Se obtiene el vértice que se va a visitar
		T verticeVisitar = noVisitados[0];
		//Obtiene los descendientes y ascendientes del actual
		std::vector<T> descendientes = cendientes(verticeVisitar, this->adyacencia);
		std::vector<T> ascendientes = cendientes(verticeVisitar, adyacenciaInversa());
		//Inserta el componente conectado identificado a componentes
		componentes.push_back(componenteConectado(descendientes, ascendientes, noVisitados));
	}
	return componentes;
}

template <class T>
std::vector<T> Grafo<T>::cendientes(T origen, std::vector < std::list< std::pair<T, float >>> adyacenciaModif) {
	//es el retorno que reemplaza a la impresión y reemplaza a los descendientes
	std::vector<T> cend;
	if (buscarVertice(origen) != -1) { //No busca descendientes si el origen no se encuentra
		//Una pila para insertar los valores en orden inverso y de esta forma hacer el recorrido
		std::stack<T> pilaVertices;
		//Inserta el origen
		pilaVertices.push(origen);
		while (!pilaVertices.empty()) {
			T verticeActual = pilaVertices.top();
			//Busca el index del vertice actual y recorrer
			int posicionActual = buscarVertice(verticeActual);
			//Dice que ya se recorrio el vertice actual
			if (buscar(cend, verticeActual) == -1) {
				cend.push_back(verticeActual);
			}
			//Elimina el vertice actual de la pila
			pilaVertices.pop();
			//recorre la adyacencia pasada (bien sea ascendente o descendente) a los hijos en orden inverso
			typename std::list< std::pair<T, float >>::reverse_iterator it = adyacenciaModif[posicionActual].rbegin();
			for (; it != adyacenciaModif[posicionActual].rend(); it++) {
				//Si no se ha recorrido el vertice lo pone arriba para imprimir
				if (buscar(cend, (*it).first) == -1) {
					pilaVertices.push((*it).first);
				}
			}
		}
	}
	return cend;
}

template <class T>
std::vector < std::list< std::pair<T, float >>> Grafo<T>::adyacenciaInversa() {
	std::vector < std::list< std::pair<T, float >>> adyacenteInverso;
	std::list< std::pair<T, float >> listaVacia;
	for (int i = 0; i < this->vertices.size(); i++) {
		adyacenteInverso.push_back(listaVacia);
	}
	for (int i = 0; i < this->vertices.size(); i++) {
		typename std::list< std::pair<T, float >>::iterator it = this->adyacencia[i].begin();
		for (; it != this->adyacencia[i].end(); it++) {
			//Inserta el origen como destino y el destino como origen en adyacencia inversa, con el mismo peso
			insertarArista(it->first, vertices[i], it->second, adyacenteInverso);
		}
	}
	return adyacenteInverso;
}

template <class T>
std::vector<T> Grafo<T>::componenteConectado(std::vector<T> descendientes, std::vector<T> ascendientes, std::vector<T>& noVisitados) {
	std::vector<T> interseccion;
	//Mira si hay un solo componete conectado
	if (descendientes.size() == ascendientes.size() && descendientes.size() == this->vertices.size()) {
		noVisitados.clear();
		return descendientes;
	}
	//Recorre descendientes
	for (int i = 0; i < descendientes.size(); i++) {
		//Busca que este en el vector de ascendientes para añadir en la intersección
		if (buscar(ascendientes, descendientes[i]) != -1) {
			interseccion.push_back(descendientes[i]);
			//Elimina el descendiente que se encontró en no visitados puesto a que ya es un componente conectado
			noVisitados.erase(noVisitados.begin() + buscar(noVisitados, descendientes[i]));
		}
	}
	return interseccion;
}

template <class T>
int Grafo<T>::cantidadAristasPuente() {
	return aristasPuente().size();
}

template <class T>
std::vector< std::pair<T, T> > Grafo<T>::aristasPuente() {
	// Vector que almacena las aristas puente
	std::vector< std::pair<T, T> > aristasBridge;
	std::pair<T, T> aristaBridge;
	// Obtiene los componentes conectados
	std::list<std::vector<T>> componentesConnected = componentesConectados();
	// Iterador para recorrer los componentes conectados
	std::list<std::vector<int>>::iterator itComp = componentesConnected.begin();
	// Adyacencia inversa para saber los padres directos de los vértices
	std::vector < std::list< std::pair<T, float >>> adyacenteInverso = adyacenciaInversa();
	// Recorrer los componentes conectados
	for (; itComp != componentesConnected.end(); itComp++) {
		int contBridges = 0;
		// Revisa hasta encontrar 2 puentes hacia el componente actual o ver todos los vértices en busca de puentes
		for (int i = 0; i < itComp->size() && contBridges < 2; i++) {
			// Obtengo la posición del vértice actual del componente actual
			int posicionOrigen = buscarVertice((*itComp)[i]);
			// Iterador para recorrer los padres directos del vértice actual
			typename std::list< std::pair<T, float >>::iterator itPadres = adyacenteInverso[posicionOrigen].begin();
			for (; itPadres != adyacenteInverso[posicionOrigen].end() && contBridges < 2; itPadres++) {
				// Si el padre está fuera del componente, se añade el "puente" y se aumenta el contador
				if (buscar(*itComp, (*itPadres).first) == -1) {
					aristaBridge = std::make_pair((*itPadres).first, (*itComp)[i]);
					contBridges++;
				}
			}
		}
		// Si solo se encontró un "puente", se añade la arista puente al vector de aristas puente
		if (contBridges == 1) {
			aristasBridge.push_back(aristaBridge);
		}
	}
	return aristasBridge;
}

// Niveles de conexión: (Disconexo/Débilmente conexo/Fuertemente conexo)
template <class T>
std::string Grafo<T>::nivelConexion() {
	// Vector que almacena las aristas puente
	std::vector< std::pair<T, T> > aristasBridge;
	std::pair<T, T> aristaBridge;
	// Obtiene los componentes conectados
	std::list<std::vector<T>> componentesConnected = componentesConectados();
	if (componentesConnected.size() == 1) {
		return "Fuertemente conexo";
	}
	// Adyacencia doble para convertir el grafo dirigido en un grafo no dirigido
	std::vector < std::list< std::pair<T, float >>> adyacenteDoble = adyacenciaInversa();

	// Juntamos la adyacencia normal con la adyacencia inversa en cada posición de adyacencia doble
	for (int i = 0; i < this->vertices.size(); i++) {
		adyacenteDoble[i].insert(adyacenteDoble[i].end(), this->adyacencia[i].begin(), this->adyacencia[i].end());
	}

	// Si desde cualquier vértice (por ejemplo el vértice 0) no se llega a todos, es disconexo
	if (cendientes(this->vertices[0], adyacenteDoble).size() != this->vertices.size()) {
		return "Disconexo";
	}
	// De lo contrario es débilmente conexo
	return "Debilmente conexo";
}

template <class T>
std::vector<std::pair<T, T>> Grafo<T>::prim(T origen) {
	std::vector<std::pair<T, T>> aristasPrim;
	//Indices de los vértices ya visitados
	std::vector<int> posVisitados;
	//Inserta la posición del origen como ya visitado
	posVisitados.push_back(buscarVertice(origen));
	while (aristasPrim.size() != this->vertices.size() - 1) {
		std::pair<T, T> aristaMinima;
		//Inicializa el peso minimo con el peso de la primera arista del primer vértice
		float pesoMinimo = INFINITY;
		//Recorre los indices de los vértices visitados
		for (int i = 0; i < posVisitados.size(); i++) {
			typename std::list< std::pair<T, float >>::iterator it = this->adyacencia[posVisitados[i]].begin();
			//Recorre las aristas del vertice vistado actual
			for (; it != this->adyacencia[posVisitados[i]].end(); it++) {
				//Verifica si el destino de la arista es uno no visitado
				if (buscar(posVisitados, buscarVertice(it->first)) == -1) {
					//Actualiza el peso minimo y la arista minima actuales
					nuevoMinimoPrim(*it, pesoMinimo, aristaMinima, this->vertices[posVisitados[i]]);
				}
			}
		}
		if (pesoMinimo == INFINITY) {
			return aristasPrim;
		}
		//Inserta la posición del destino de la arista minima
		posVisitados.push_back(buscarVertice(aristaMinima.second));
		//Inserta la arista minima en el vector de recorrido prim
		aristasPrim.push_back(aristaMinima);
	}
	return aristasPrim;
}

template <class T>
void Grafo<T>::nuevoMinimoPrim(std::pair<T, float > aristaActual, float& pesoMinimo, std::pair<T, T>& aristaMinima, T verticeVisitando) {
	//Mira si el peso de la arista actual es menor al peso minimo
	if (aristaActual.second < pesoMinimo) {
		//Actualiza el pesoMinimo y la aristaMinima
		pesoMinimo = aristaActual.second;
		//Hace un pair a partir del vertice que es está visitando y el destino de la arista
		aristaMinima = std::make_pair(verticeVisitando, aristaActual.first);
	}
	//De lo contrario mira si son iguales para ver el segundo criterio de comparación (destino)
	else if (aristaActual.second == pesoMinimo) {
		//Si el destino de la arista actual es menor que el de la arista minima lo actualiza
		if (aristaActual.first < aristaMinima.second) {
			pesoMinimo = aristaActual.second;
			aristaMinima = std::make_pair(verticeVisitando, aristaActual.first);
		}
		//De lo contrario mira si son iguales para ver el tercer criterio de comparación (origen)
		else if (aristaActual.first == aristaMinima.second) {
			//Si el origen de la arista actual es menor que el de la arista minima lo actualiza
			if (verticeVisitando < aristaMinima.first) {
				pesoMinimo = aristaActual.second;
				aristaMinima = std::make_pair(verticeVisitando, aristaActual.first);
			}
		}
	}
}

template <class T>
std::vector<std::pair<T, T>> Grafo<T>::dijkstra(T origen) {
	// Se trabaja con los índices de this->vertices
	std::vector<std::pair<T, T>> aristasDijkstra;
	int posicionActual = buscarVertice(origen);
	T verticeActual = origen;
	// Q => No visitados
	std::vector<T> noVisitados = this->vertices;
	// dist => peso total para llegar desde el origen hasta dicho vértice
	std::vector<float> distanciasPeso(this->vertices.size());
	// Llena todas las distancias con INFINITY, excepto el origen (que llena con 0)
	std::fill(distanciasPeso.begin(), distanciasPeso.end(), INFINITY);
	distanciasPeso[posicionActual] = 0;
	// pred => vertice anterior para llegar al vértice correspondiente al índice
	std::vector<T> predecesores(this->vertices.size());
	// El predecesor del origen es el mismo origen
	predecesores[posicionActual] = origen;

	// Inicializamos la distancia mínima en 0 para que entre por primera vez al while
	float distanciaMinima = 0.0;

	// Mientras que se pueda llegar a un vértice no visitado
	while (distanciaMinima != INFINITY) {
		typename std::list< std::pair<T, float >>::iterator it = this->adyacencia[posicionActual].begin();
		// Recorre los hijos directos del vértice actual
		for (; it != this->adyacencia[posicionActual].end(); it++) {
			// Guarda la posición del destino de la arista
			int posicionDestino = buscarVertice(it->first);
			// La posible distancia nueva es la distancia del vértice actual + la distancia de la arista hacia el destino
			float distanciaNueva = distanciasPeso[posicionActual] + it->second;
			// Si la posible distancia nueva es menor a la distancia del destino
			if (distanciaNueva < distanciasPeso[posicionDestino]) {
				// Se actualiza la distancia y el predecesor del destino
				distanciasPeso[posicionDestino] = distanciaNueva;
				predecesores[posicionDestino] = verticeActual;
			}
		}

		// Se elimina al vértice recién visitado de los no visitados
		noVisitados.erase(noVisitados.begin() + buscar(noVisitados, verticeActual));
		// Caso base distancia mínima infinito para empezar la comparación luego
		distanciaMinima = INFINITY;
		T verticeSiguiente;
		// Recorrer todas las distancias de los vértices
		for (int i = 0; i < distanciasPeso.size(); i++) {
			// Si el vértice no ha sido visitado, se revisa su distancia
			if (buscar(noVisitados, this->vertices[i]) != -1) {
				// Si su distancia es menor a la distancia mínima
				if (distanciasPeso[i] < distanciaMinima) {
					// La nueva distancia mínima y vértice siguiente se actualizan
					distanciaMinima = distanciasPeso[i];
					verticeSiguiente = this->vertices[i];
				}
				// Si su distancia es igual a la distancia mínima y el vértice es menor al vértice siguiente
				else if (distanciasPeso[i] == distanciaMinima && this->vertices[i] < verticeSiguiente) {
					// La nueva distancia mínima y vértice siguiente se actualizan
					distanciaMinima = distanciasPeso[i];
					verticeSiguiente = this->vertices[i];
				}
			}
		}
		// Si la distancia mínima se actualizó
		if (distanciaMinima != INFINITY) {
			// Se actualiza la posición actual para la siguiente iteración
			posicionActual = buscarVertice(verticeSiguiente);
			verticeActual = verticeSiguiente;
		}
	}
	// Se crea una cola para poner todas las aristas en orden
	std::queue<T> colaGrafoDijkstra;
	colaGrafoDijkstra.push(origen);
	while (!colaGrafoDijkstra.empty()) {
		//Se saca el vertice actual
		verticeActual = colaGrafoDijkstra.front();
		//Busca la posición del vertice actual para recorrer
		posicionActual = buscarVertice(verticeActual);
		//Lo quita de la cola
		colaGrafoDijkstra.pop();
		// Recorre las aristas del vértice actual
		for (int i = 0; i < predecesores.size(); i++) {
			//Busca si no está en vertices recorridos para evitar repetición
			if (predecesores[i] == verticeActual && i != posicionActual) {
				aristasDijkstra.push_back(std::make_pair(verticeActual, this->vertices[i]));
				colaGrafoDijkstra.push(this->vertices[i]);
			}
		}
	}

	return aristasDijkstra;
}

template <class T>
std::vector<T> Grafo<T>::circuitoVertices(T origen) {
	// Se trabaja con los índices de this->vertices
	// Se almacenan los vértices de ida, empezando por el origen
	std::vector<T> verticesIda;
	verticesIda.push_back(origen);
	// Se almacenan los vértices de vuelta, finalizando por el origen
	std::deque<T> verticesVuelta;
	verticesVuelta.push_front(origen);

	while (verticesIda.size() + verticesVuelta.size() != this->vertices.size() + 1) {
		// Se utilizan pesoMinimo y verticeMinimo de ida y vuelta para encontrar el vértice más cercano para ida y vuelta
		float pesoMinimoIda = INFINITY, pesoMinimoVuelta = INFINITY;
		T verticeMinimoIda, verticeMinimoVuelta;
		// La posición actual de ida es la posición del último en verticesIda en el vector de vértices del grafo
		int posicionActualIda = buscarVertice(verticesIda[verticesIda.size() - 1]);
		// La posición actual de vuelta es el primero en verticesVuelta en el vector de vértices del grafo
		int posicionActualVuelta = buscarVertice(verticesVuelta[0]);

		// Se envía la información para hallar el destino más cercano de ida
		hallarDestinoMasCercano(posicionActualIda, verticesIda, verticesVuelta, pesoMinimoIda, verticeMinimoIda, origen);
		// Se envía la información para hallar el destino más cercano de vuelta
		hallarDestinoMasCercano(posicionActualVuelta, verticesIda, verticesVuelta, pesoMinimoVuelta, verticeMinimoVuelta, origen);
		
		// Si el más cercano en la ida y la vuelta es el mismo vuelve a buscar el más cercano del que tenga mayor distancia
		if (verticeMinimoIda == verticeMinimoVuelta) {
			// Si le queda más cerca a la ida
			if (pesoMinimoIda <= pesoMinimoVuelta) {
				// Busca al segundo más cercano de la vuelta (baneamos al primero más cercano)
				pesoMinimoVuelta = INFINITY;
				hallarDestinoMasCercano(0, verticesIda, verticesVuelta, pesoMinimoVuelta, verticeMinimoVuelta, verticeMinimoIda);
			}
			else {
				// Busca al segundo más cercano de la ida (baneamos al primero más cercano)
				pesoMinimoIda = INFINITY;
				hallarDestinoMasCercano(verticesIda.size() - 1, verticesIda, verticesVuelta, pesoMinimoIda, verticeMinimoIda, verticeMinimoVuelta);
			}
		}

		// En el caso de que solo quedé 1 vértice sin visitar solo lo insertamos en verticesIda
		if (verticeMinimoIda == verticeMinimoVuelta) {
			verticesIda.push_back(verticeMinimoIda);
		}
		else {
			// Inserta el vértice mínimo de ida al vector de verticesIda
			verticesIda.push_back(verticeMinimoIda);
			// Inserta el vértice mínimo de vuelta al deque de verticesVuelta
			verticesVuelta.push_front(verticeMinimoVuelta);
		}
	}

	// Insertar al final del vector el contenido del deque
	verticesIda.insert(verticesIda.end(), verticesVuelta.begin(), verticesVuelta.end());
	
	return verticesIda;
}

template <class T>
void Grafo<T>::nuevoMinimoCircuitos(std::pair<T, float > aristaActual, float& pesoMinimo, T& verticeMinimo) {
	//Mira si el peso de la arista actual es menor al peso minimo
	if (aristaActual.second < pesoMinimo) {
		//Actualiza el pesoMinimo y el verticeMinimo
		pesoMinimo = aristaActual.second;
		verticeMinimo = aristaActual.first;
	}
	//De lo contrario mira si son iguales para ver el segundo criterio de comparación (destino)
	else if (aristaActual.second == pesoMinimo) {
		//Si el destino de la arista actual es menor que el vértice mínimo lo actualiza
		if (aristaActual.first < verticeMinimo) {
			pesoMinimo = aristaActual.second;
			verticeMinimo = aristaActual.first;
		}
	}
}

template <class T>
void Grafo<T>::hallarDestinoMasCercano(int posicionActual, std::vector<T> verticesIda, std::deque<T> verticesVuelta,
	float& pesoMinimo, T& verticeMinimo, T verticeBaneado) {
	typename std::list< std::pair<T, float >>::iterator it = this->adyacencia[posicionActual].begin();
	//Recorre las aristas del vertice vistado actual
	for (; it != this->adyacencia[posicionActual].end(); it++) {
		//Verifica si el destino de la arista es uno NO VISITADO
		if (buscar(verticesIda, it->first) == -1 && buscar(verticesVuelta, it->first) == -1) {
			// Validar que la arista no tenga como destino al vértice baneado
			if (it->first != verticeBaneado) {
				//Actualiza el peso minimo y el vertice minimo ida
				nuevoMinimoCircuitos(*it, pesoMinimo, verticeMinimo);
			}
		}
	}
}

template <class T>
float Grafo<T>::distanciaRecorrido(std::vector<T> verticesRecorrido) {
	float distanciaPesoTotal = 0.0;
	// Recorrer todos los vértices del recorrido
	for (int i = 0; i < verticesRecorrido.size()-1; i++) {
		int posicionActual = buscarVertice(verticesRecorrido[i]);
		typename std::list< std::pair<T, float >>::iterator it = this->adyacencia[posicionActual].begin();
		// Recorrer todas las aristas del vértice actual
		for (; it != this->adyacencia[posicionActual].end(); it++) {
			// Suma el peso de la arista del vértice actual hacia el vértice siguiente
			if ((*it).first == verticesRecorrido[i+1]) {
				distanciaPesoTotal += it->second;
			}
		}
	}
	return distanciaPesoTotal;
}