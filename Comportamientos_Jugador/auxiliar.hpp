#ifndef COMPORTAMIENTOAUXILIAR_H
#define COMPORTAMIENTOAUXILIAR_H

#include "comportamientos/comportamiento.hpp"
#include <list>
#include <set> 
#include <vector>
#include <queue> 


struct EstadoA {
    int f;      
    int c;          
    int brujula;     
    bool zapatillas;  

    bool operator == (const EstadoA &st) const {
        return (f == st.f && c == st.c && brujula == st.brujula && zapatillas == st.zapatillas);
    }

 
    bool operator < (const EstadoA &st) const {
        if (f < st.f) return true;
        else if (f == st.f && c < st.c) return true;
        else if (f == st.f && c == st.c && brujula < st.brujula) return true;
        else if (f == st.f && c == st.c && brujula == st.brujula && zapatillas < st.zapatillas) return true;
        else return false;
    }
};


struct NodoA_tutorial {
    EstadoA estado;
    list<Action> secuencia;

    
    bool operator==(const NodoA_tutorial &node) const {
        return estado == node.estado;
    }

    bool operator<(const NodoA_tutorial &node) const{
	    return estado < node.estado;
    }
};


struct NodoA_AStar {
    EstadoA st;
    list<Action> secuencia;
    int g_cost; // Coste del camino desde el inicio
    int h_cost; // Coste heurístico estimado hasta el final

    // Operador para la cola de prioridad de A* (ordena por f_cost = g+h)
    bool operator > (const NodoA_AStar &n) const {
        return (g_cost + h_cost) > (n.g_cost + n.h_cost);
    }
};


class ComportamientoAuxiliar : public Comportamiento
{
public:
  ComportamientoAuxiliar(unsigned int size = 0) : Comportamiento(size) {
    last_action = IDLE;
    tiene_zapatillas = false;
    giro45Izq = 0;
    last_level = -1;
    ultima_orientacion_camino = -1;
    hayPlan = false;
  }
  ComportamientoAuxiliar(std::vector<std::vector<unsigned char>> mapaR, std::vector<std::vector<unsigned char>> mapaC) : Comportamiento(mapaR,mapaC) {
    hayPlan = false;
  }
  ComportamientoAuxiliar(const ComportamientoAuxiliar &comport) : Comportamiento(comport) {}
  ~ComportamientoAuxiliar() {}

  Action think(Sensores sensores);
  int interact(Action accion, int valor);

 
  Action ComportamientoAuxiliarNivel_0(Sensores sensores);
  Action ComportamientoAuxiliarNivel_1(Sensores sensores);
  Action ComportamientoAuxiliarNivel_2(Sensores sensores);
  Action ComportamientoAuxiliarNivel_3(Sensores sensores); 
  Action ComportamientoAuxiliarNivel_4(Sensores sensores);
  Action ComportamientoAuxiliarNivel_E(Sensores sensores); 


private:


  void VisualizaPlan(const EstadoA &st, const list<Action> &plan);
  list<Action> AnchuraAuxiliar_V2(const EstadoA &inicio, const EstadoA &final);
  EstadoA applyA_tutorial(Action accion, const EstadoA &st);
  bool CasillaAccesibleAuxiliar(const EstadoA &st);
  EstadoA NextCasillaAuxiliar(const EstadoA &st);
  list<Action> AStar(const EstadoA &inicio, const EstadoA &final);
  int calculateHeuristic(const EstadoA &st, const EstadoA &final);
  int calculateCostA(char casilla_inicial, int dif_altura);
  
  // variables de estado
  Action last_action;
  bool tiene_zapatillas;
  int giro45Izq;
  int ultima_orientacion_camino;
  int last_level;
  list<Action> plan;
  bool hayPlan;
};

#endif