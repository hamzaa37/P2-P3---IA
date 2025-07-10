#ifndef COMPORTAMIENTORESCATADOR_H
#define COMPORTAMIENTORESCATADOR_H

#include "comportamientos/comportamiento.hpp"
#include <list>
#include <set>
#include <vector>

// Estructuras para los niveles deliberativos (Nivel 2 y 4)
struct EstadoR {
    int fila;
    int columna;
    int orientacion;
    bool tiene_zapatillas;

    bool operator == (const EstadoR &st) const {
        return (fila == st.fila && columna == st.columna && orientacion == st.orientacion && tiene_zapatillas == st.tiene_zapatillas);
    }

    // Operador para comparar estados, útil para el set de 'explored'
    bool operator < (const EstadoR &st) const {
        if (fila < st.fila) return true;
        else if (fila == st.fila && columna < st.columna) return true;
        else if (fila == st.fila && columna == st.columna && orientacion < st.orientacion) return true;
        else if (fila == st.fila && columna == st.columna && orientacion == st.orientacion && tiene_zapatillas < st.tiene_zapatillas) return true;
        return false;
    }
};

struct NodoR {
    EstadoR st;
    list<Action> secuencia;
    int coste; // Coste del camino para Dijkstra

    // Operador para la cola de prioridad de Dijkstra (ordena de menor a mayor coste)
    bool operator > (const NodoR &n) const {
        return coste > n.coste;
    }
};


class ComportamientoRescatador : public Comportamiento
{
public:
  ComportamientoRescatador(unsigned int size = 0) : Comportamiento(size) {
    // Inicialización para niveles reactivos
    last_action = IDLE;
    tiene_zapatillas = false;
    giro45Izq = 0;
    pasos_esquivando = 0;
    last_level = -1;
    ultima_orientacion_camino = -1;
  }
  ComportamientoRescatador(std::vector<std::vector<unsigned char>> mapaR, std::vector<std::vector<unsigned char>> mapaC) : Comportamiento(mapaR,mapaC) {
    // Inicialización para niveles deliberativos
    hayPlan = false;
  }
  ComportamientoRescatador(const ComportamientoRescatador &comport) : Comportamiento(comport) {}
  ~ComportamientoRescatador() {}

  Action think(Sensores sensores);
  int interact(Action accion, int valor);

  Action ComportamientoRescatadorNivel_0(Sensores sensores);
  Action ComportamientoRescatadorNivel_1(Sensores sensores);
  Action ComportamientoRescatadorNivel_2(Sensores sensores);
  Action ComportamientoRescatadorNivel_3(Sensores sensores);
  Action ComportamientoRescatadorNivel_4(Sensores sensores);

  // Funciones para el Nivel 2
  list<Action> Dijkstra(const EstadoR &inicio, const EstadoR &final);


private:
  // --- Variables de Estado ---
  
  // Para niveles reactivos (0 y 1)
  Action last_action;
  bool tiene_zapatillas;
  int giro45Izq;
  int pasos_esquivando;
  int ultima_orientacion_camino;
  int last_level;

  // Para niveles deliberativos (2 y 4)
  list<Action> plan;
  bool hayPlan;
};
#endif