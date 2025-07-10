// HAMZA FATA //
#include "../Comportamientos_Jugador/auxiliar.hpp"
#include <iostream>
#include "motorlib/util.h"
#include <utility> // ( para std::pair )
#include <set>
#include <queue> // ( para std::priority_queue )
#include <vector>
#include <list>
#include <cmath>



Action ComportamientoAuxiliar::think(Sensores sensores)
{
	Action accion = IDLE;

	if (sensores.nivel != last_level) {
		giro45Izq = 0; 
        ultima_orientacion_camino = -1;
		last_level = sensores.nivel;
        hayPlan = false;
	}

	switch (sensores.nivel)
	{
	case 0:
		accion = ComportamientoAuxiliarNivel_0(sensores);
		break;
	case 1:
		accion = ComportamientoAuxiliarNivel_1(sensores);
		break;
	case 2:
        accion = ComportamientoAuxiliarNivel_2(sensores);           
        break;  
    case 3:
       // accion = ComportamientoAuxiliarNivel_3(sensores);
        accion = ComportamientoAuxiliarNivel_E(sensores); // Usar el nivel E para el tutorial
        break;
    case 4:
        //accion = ComportamientoAuxiliarNivel_4(sensores);
        break;
	}

	return accion;
}

int ComportamientoAuxiliar::interact(Action accion, int valor)
{
	return 0;
}

Action ComportamientoAuxiliar::ComportamientoAuxiliarNivel_E(Sensores sensores) {
    Action accion = IDLE;

    if (!hayPlan) {
        EstadoA inicio, fin;
        inicio.f = sensores.posF;
        inicio.c = sensores.posC;
        inicio.brujula = sensores.rumbo;
        inicio.zapatillas = tiene_zapatillas;

        fin.f = sensores.destinoF;
        fin.c = sensores.destinoC;

        plan = AnchuraAuxiliar_V2(inicio, fin);
        VisualizaPlan(inicio, plan); // Visualizar el plan encontrado
        
        if (plan.size() > 0) {
            hayPlan = true;
        }
    }

    if (hayPlan && !plan.empty()) {
        accion = plan.front();
        plan.pop_front();
    }
    
    if (plan.empty()) {
        hayPlan = false; // Reiniciar el estado del plan cuando se completa
    }
	
	return accion;
}


// Búsqueda en Anchura  
list<Action> ComportamientoAuxiliar::AnchuraAuxiliar_V2(const EstadoA &inicio, const EstadoA &final) {
    NodoA_tutorial current_node;
    current_node.estado = inicio;

    list<NodoA_tutorial> frontier;
    frontier.push_back(current_node);

    set<NodoA_tutorial> explored;
    explored.insert(current_node);

    bool solutionFound = (current_node.estado.f == final.f && current_node.estado.c == final.c);

    while (!frontier.empty() && !solutionFound) {
        current_node = frontier.front();
        frontier.pop_front();

        // Acción WALK
        NodoA_tutorial child_walk = current_node;
        child_walk.estado = applyA_tutorial(WALK, current_node.estado);

        if (child_walk.estado.f == final.f && child_walk.estado.c == final.c) {
            child_walk.secuencia.push_back(WALK);
            current_node = child_walk;
            solutionFound = true;
        } else if (explored.find(child_walk) == explored.end()) {
            child_walk.secuencia.push_back(WALK);
            frontier.push_back(child_walk);
            explored.insert(child_walk);
        }

        if (solutionFound) break;

        // Acción TURN_SR
        NodoA_tutorial child_turn_sr = current_node;
        child_turn_sr.estado = applyA_tutorial(TURN_SR, current_node.estado);
        if (explored.find(child_turn_sr) == explored.end()) {
            child_turn_sr.secuencia.push_back(TURN_SR);
            frontier.push_back(child_turn_sr);
            explored.insert(child_turn_sr);
        }
    }

    if (solutionFound) {
        return current_node.secuencia;
    } else {
        return list<Action>(); // Devolver plan vacío
    }
}


EstadoA ComportamientoAuxiliar::applyA_tutorial(Action accion, const EstadoA &st) {
    EstadoA next = st;
    switch(accion) {
        case WALK:
            if (CasillaAccesibleAuxiliar(st)) {
                next = NextCasillaAuxiliar(st);
            }
            break;
        case TURN_SR:
            next.brujula = (next.brujula + 1) % 8;
            break;
        default: break;
    }

    // Actualizar el estado de las zapatillas si la nueva casilla las tiene
    if (mapaResultado[next.f][next.c] == 'D') {
        next.zapatillas = true;
    }
    return next;
}

bool ComportamientoAuxiliar::CasillaAccesibleAuxiliar(const EstadoA &st) {
    EstadoA next = NextCasillaAuxiliar(st);
    
    // Comprobar límites del mapa
    if (next.f < 0 || next.f >= mapaResultado.size() || next.c < 0 || next.c >= mapaResultado[0].size()) {
        return false;
    }
    
    char tipo_terreno = mapaResultado[next.f][next.c];
    
    if (tipo_terreno == 'P' || tipo_terreno == 'M') return false; // Precipicio u Obstáculo
    if (tipo_terreno == 'B' && !st.zapatillas) return false;    // Bosque sin zapatillas
    
    int dif_altura = abs(mapaCotas[next.f][next.c] - mapaCotas[st.f][st.c]);
    if (dif_altura > 1) return false; // El Auxiliar solo puede manejar una diferencia de altura de 1

    return true;
}


EstadoA ComportamientoAuxiliar::NextCasillaAuxiliar(const EstadoA &st) {
    EstadoA siguiente = st;
    switch(st.brujula) {
        case norte:     siguiente.f--; break;
        case noreste:   siguiente.f--; siguiente.c++; break;
        case este:      siguiente.c++; break;
        case sureste:   siguiente.f++; siguiente.c++; break;
        case sur:       siguiente.f++; break;
        case suroeste:  siguiente.f++; siguiente.c--; break;
        case oeste:     siguiente.c--; break;
        case noroeste:  siguiente.f--; siguiente.c--; break;
    }
    return siguiente;
}


void ComportamientoAuxiliar::VisualizaPlan(const EstadoA &st, const list<Action> &plan) {
    for (int i = 0; i < mapaConPlan.size(); i++) {
        for (int j = 0; j < mapaConPlan[i].size(); j++) {
            mapaConPlan[i][j] = 0;
        }
    }
    
    EstadoA cst = st;
    for (const Action& act : plan) {
        if (act == WALK) {
            cst = NextCasillaAuxiliar(cst);
            if (cst.f >= 0 && cst.f < mapaConPlan.size() && cst.c >= 0 && cst.c < mapaConPlan[0].size()) {
                mapaConPlan[cst.f][cst.c] = 2;
            }
        } else if (act == TURN_SR) {
            cst.brujula = (cst.brujula + 1) % 8;
        }
    }
}


list<Action> ComportamientoAuxiliar::AStar(const EstadoA &inicio, const EstadoA &final) {
    priority_queue<NodoA_AStar, vector<NodoA_AStar>, greater<NodoA_AStar>> frontier;
    set<EstadoA> explored;

    NodoA_AStar nodo_inicial;
    nodo_inicial.st = inicio;
    nodo_inicial.g_cost = 0;
    nodo_inicial.h_cost = calculateHeuristic(inicio, final);
    frontier.push(nodo_inicial);

    while (!frontier.empty()) {
        NodoA_AStar current_node = frontier.top();
        frontier.pop();

        if (current_node.st.f == final.f && current_node.st.c == final.c) {
            return current_node.secuencia;
        }
        
        if (explored.count(current_node.st)) {
            continue;
        }
        
        explored.insert(current_node.st);
        
        Action acciones[] = {WALK, TURN_SR};
        for (Action act : acciones) {
            NodoA_AStar child_node = current_node;
            child_node.st = applyA_tutorial(act, current_node.st);

            if (!(child_node.st == current_node.st)) {
                int action_cost = calculateCostA(mapaResultado[current_node.st.f][current_node.st.c], 
                                                mapaCotas[child_node.st.f][child_node.st.c] - mapaCotas[current_node.st.f][current_node.st.c]);
                child_node.g_cost = current_node.g_cost + action_cost;
                child_node.h_cost = calculateHeuristic(child_node.st, final);
                child_node.secuencia.push_back(act);
                frontier.push(child_node);
            }
        }
    }
    return list<Action>();
}

int ComportamientoAuxiliar::calculateCostA(char casilla_inicial, int dif_altura) {
    int cost = 0;
    int incr_altura = (dif_altura != 0);
    switch(casilla_inicial) {
        case 'A': cost = 100 + incr_altura * 10; break;
        case 'B': cost = 20 + incr_altura * 5; break;
        case 'S': cost = 2 + incr_altura * 1; break;
        default: cost = 1 + incr_altura * 0; break;
    }
    return cost;
}

int ComportamientoAuxiliar::calculateHeuristic(const EstadoA &st, const EstadoA &final) {
    return abs(st.f - final.f) + abs(st.c - final.c);
}

std::pair<int, int> getCoordsA(Sensores sensores, int i) 
{
    int F = sensores.posF, C = sensores.posC;
    switch (sensores.rumbo) {
        case norte:
            switch(i) {
                case 1: F-=1; C-=1; break; case 2: F-=1; break; case 3: F-=1; C+=1; break;
                case 4: F-=2; C-=2; break; case 5: F-=2; C-=1; break; case 6: F-=2; break; case 7: F-=2; C+=1; break; case 8: F-=2; C+=2; break;
                case 9: F-=3; C-=3; break; case 10: F-=3; C-=2; break; case 11: F-=3; C-=1; break; case 12: F-=3; break; case 13: F-=3; C+=1; break; case 14: F-=3; C+=2; break; case 15: F-=3; C+=3; break;
            }
            break;
        case noreste:
            switch(i) {
                case 1: F-=1; break; case 2: F-=1; C+=1; break; case 3: C+=1; break;
                case 4: F-=2; C-=1; break; case 5: F-=2; break; case 6: F-=2; C+=1; break; case 7: F-=1; C+=2; break; case 8: C+=2; break;
                case 9: F-=3; C-=2; break; case 10: F-=3; C-=1; break; case 11: F-=3; break; case 12: F-=2; C+=2; break; case 13: F-=1; C+=3; break; case 14: C+=3; break; case 15: F+=1; C+=2; break;
            }
            break;
        case este:
            switch(i) {
                case 1: F-=1; C+=1; break; case 2: C+=1; break; case 3: F+=1; C+=1; break;
                case 4: F-=2; C+=2; break; case 5: F-=1; C+=2; break; case 6: C+=2; break; case 7: F+=1; C+=2; break; case 8: F+=2; C+=2; break;
                case 9: F-=3; C+=3; break; case 10: F-=2; C+=3; break; case 11: F-=1; C+=3; break; case 12: C+=3; break; case 13: F+=1; C+=3; break; case 14: F+=2; C+=3; break; case 15: F+=3; C+=3; break;
            }
            break;
        case sureste:
            switch(i) {
                case 1: C+=1; break; case 2: F+=1; C+=1; break; case 3: F+=1; break;
                case 4: F-=1; C+=2; break; case 5: C+=2; break; case 6: F+=1; C+=2; break; case 7: F+=2; C+=1; break; case 8: F+=2; break;
                case 9: F-=2; C+=3; break; case 10: F-=1; C+=3; break; case 11: C+=3; break; case 12: F+=2; C+=2; break; case 13: F+=3; C+=1; break; case 14: F+=3; break; case 15: F+=2; C-=1; break;
            }
            break;
        case sur:
            switch(i) {
                case 1: F+=1; C-=1; break; case 2: F+=1; break; case 3: F+=1; C+=1; break;
                case 4: F+=2; C-=2; break; case 5: F+=2; C-=1; break; case 6: F+=2; break; case 7: F+=2; C+=1; break; case 8: F+=2; C+=2; break;
                case 9: F+=3; C-=3; break; case 10: F+=3; C-=2; break; case 11: F+=3; C-=1; break; case 12: F+=3; break; case 13: F+=3; C+=1; break; case 14: F+=3; C+=2; break; case 15: F+=3; C+=3; break;
            }
            break;
        case suroeste:
            switch(i) {
                case 1: F+=1; break; case 2: F+=1; C-=1; break; case 3: C-=1; break;
                case 4: F+=2; C+=1; break; case 5: F+=2; break; case 6: F+=2; C-=1; break; case 7: F+=1; C-=2; break; case 8: C-=2; break;
                case 9: F+=3; C+=2; break; case 10: F+=3; C+=1; break; case 11: F+=3; break; case 12: F+=2; C-=2; break; case 13: F+=1; C-=3; break; case 14: C-=3; break; case 15: F-=1; C-=2; break;
            }
            break;
        case oeste:
            switch(i) {
                case 1: F+=1; C-=1; break; case 2: C-=1; break; case 3: F-=1; C-=1; break;
                case 4: F+=2; C-=2; break; case 5: F+=1; C-=2; break; case 6: C-=2; break; case 7: F-=1; C-=2; break; case 8: F-=2; C-=2; break;
                case 9: F+=3; C-=3; break; case 10: F+=2; C-=3; break; case 11: F+=1; C-=3; break; case 12: C-=3; break; case 13: F-=1; C-=3; break; case 14: F-=2; C-=3; break; case 15: F-=3; C-=3; break;
            }
            break;
        case noroeste:
            switch(i) {
                case 1: C-=1; break; case 2: F-=1; C-=1; break; case 3: F-=1; break;
                case 4: F+=1; C-=2; break; case 5: C-=2; break; case 6: F-=1; C-=2; break; case 7: F-=2; C-=1; break; case 8: F-=2; break;
                case 9: F+=2; C-=3; break; case 10: F+=1; C-=3; break; case 11: C-=3; break; case 12: F-=2; C-=2; break; case 13: F-=3; C-=1; break; case 14: F-=3; break; case 15: F-=2; C+=1; break;
            }
            break;
    }
    return std::make_pair(F, C);
}

char ViablePorAlturaA(char casilla, int dif, bool zap)
{
	// Mantengo esta lógica aunque no siga el tutorial totalmente, porque al no considerar las zapatillas,no me deja avanzar al puesto base
	if ((abs(dif) <= 1) or (zap and abs(dif) <= 2))
		return casilla;
	else
		return 'P';
}


int VeoCasillaInteresanteA(char i, char c, char d, bool zap, bool permite_senderos = true)
{
	if (c == 'X') return 2;
	else if (i == 'X') return 1;
	else if (d == 'X') return 3;
	else if (!zap) {
		if (c == 'D') return 2;
		else if (i == 'D') return 1;
		else if (d == 'D') return 3;
	}

	if (c == 'C') return 2;
	else if (i == 'C') return 1;
	else if (d == 'C') return 3;
    else if (permite_senderos && c == 'S') return 2;
	else if (permite_senderos && i == 'S') return 1;
	else if (permite_senderos && d == 'S') return 3;
    
	else return 0;
}

void SituarSensorEnMapaA(vector<vector<unsigned char> > &m, vector<vector<unsigned char> > &a, Sensores sensores)
{
    m[sensores.posF][sensores.posC] = sensores.superficie[0];
    a[sensores.posF][sensores.posC] = sensores.cota[0];
    
    for (int i = 1; i <= 15; i++) {
        std::pair<int, int> coords = getCoordsA(sensores, i);
        if (coords.first >= 0 && coords.first < m.size() && coords.second >= 0 && coords.second < m[0].size()) {
            m[coords.first][coords.second] = sensores.superficie[i];
            a[coords.first][coords.second] = sensores.cota[i];
        }
    }
}

Action ComportamientoAuxiliar::ComportamientoAuxiliarNivel_0(Sensores sensores)
{
	Action accion;
	SituarSensorEnMapaA(mapaResultado, mapaCotas, sensores);
	if (sensores.superficie[0] == 'D') tiene_zapatillas = true;

	if (sensores.superficie[0] == 'X') {
		accion = IDLE;
	}
	else if (giro45Izq != 0) {
		accion = TURN_SR;
		giro45Izq--;
	}
	else {
		char i = ViablePorAlturaA(sensores.superficie[1], sensores.cota[1] - sensores.cota[0], tiene_zapatillas);
		char c = ViablePorAlturaA(sensores.superficie[2], sensores.cota[2] - sensores.cota[0], tiene_zapatillas);
		char d = ViablePorAlturaA(sensores.superficie[3], sensores.cota[3] - sensores.cota[0], tiene_zapatillas);
		int pos = VeoCasillaInteresanteA(i, c, d, tiene_zapatillas, false);

		switch (pos)
		{
		case 2:
			if (sensores.agentes[2] == '_') {
				accion = WALK;
                
                if (sensores.superficie[0] == 'C' || sensores.superficie[0] == 'S') {
					ultima_orientacion_camino = sensores.rumbo;
				}
			}
			else {
				accion = TURN_SR;
			}
			break;
		case 1:
			giro45Izq = 6;
			accion = TURN_SR;
			break;
		case 3:
			accion = TURN_SR;
			break;
		case 0: // Lógica para volver al camino
    if (ultima_orientacion_camino != -1) {
        int orientacion_opuesta = (ultima_orientacion_camino + 4) % 8;
        if (sensores.rumbo != orientacion_opuesta) {
            accion = TURN_SR;
        } else {
            accion = WALK;
            // Actualiza la memoria para que el agente sepa que ahora esta es la dirección correcta del camino.
            ultima_orientacion_camino = orientacion_opuesta;
        }
    } else {
        accion = TURN_SR;
    }
    break;
		}
	}

	last_action = accion;
	return accion;
}

Action ComportamientoAuxiliar::ComportamientoAuxiliarNivel_1(Sensores sensores)
{
	Action accion = IDLE;
	SituarSensorEnMapaA(mapaResultado, mapaCotas, sensores);

	bool explorado = false;
	std::pair<int, int> coords;

	// Prioridad 1: Moverse hacia casillas desconocidas ('?')
	coords = getCoordsA(sensores, 2);
	if (mapaResultado[coords.first][coords.second] == '?' && (sensores.superficie[2] == 'C' || sensores.superficie[2] == 'S')) {
		accion = WALK;
		explorado = true;
	}

	if (!explorado) {
		coords = getCoordsA(sensores, 1);
		if (mapaResultado[coords.first][coords.second] == '?' && (sensores.superficie[1] == 'C' || sensores.superficie[1] == 'S')) {
			giro45Izq = 6;
			accion = TURN_SR;
			explorado = true;
		}
	}
	
	if (!explorado) {
		coords = getCoordsA(sensores, 3);
		if (mapaResultado[coords.first][coords.second] == '?' && (sensores.superficie[3] == 'C' || sensores.superficie[3] == 'S')) {
			accion = TURN_SR;
			explorado = true;
		}
	}
	
    // Prioridad 2 (Respaldo): Si no hay '?' a la vista, navegar por terreno conocido
	if (!explorado) {
		char i = ViablePorAlturaA(sensores.superficie[1], sensores.cota[1]-sensores.cota[0], tiene_zapatillas);
		char c = ViablePorAlturaA(sensores.superficie[2], sensores.cota[2]-sensores.cota[0], tiene_zapatillas);
		char d = ViablePorAlturaA(sensores.superficie[3], sensores.cota[3]-sensores.cota[0], tiene_zapatillas);
		int pos = VeoCasillaInteresanteA(i, c, d, tiene_zapatillas, true);

		switch (pos) {
			case 2: accion = WALK; break;
			case 1: giro45Izq = 6; accion = TURN_SR; break;
			case 3: accion = TURN_SR; break;
			case 0: // Lógica para volver al camino
                if (ultima_orientacion_camino != -1) {
                    int orientacion_opuesta = (ultima_orientacion_camino + 4) % 8;
                    if (sensores.rumbo != orientacion_opuesta) {
                        accion = TURN_SR;
                    } else {
                        accion = WALK;
                    }
                } else {
                    accion = TURN_SR;
                }
                break;
		}
	}

    // Guardar la orientación si la acción final es avanzar por un camino
    if (accion == WALK) {
		if (sensores.superficie[0] == 'C' || sensores.superficie[0] == 'S' || sensores.superficie[2] == 'C' || sensores.superficie[2] == 'S') {
			ultima_orientacion_camino = sensores.rumbo;
		}
	}

	// Gestión de giros y colisiones
	if (giro45Izq > 0 && !explorado) {
		accion = TURN_SR;
		giro45Izq--;
	} else if (accion == WALK && sensores.agentes[2] != '_') {
		accion = TURN_SR;
	}
	
	last_action = accion;
	return accion;
}

Action ComportamientoAuxiliar::ComportamientoAuxiliarNivel_2(Sensores sensores) {
	Action accion = IDLE;

	// Lógica para no estorbar al Rescatador.
	// Si el Rescatador está cerca (en las primeras 8 casillas del sensor),
	// el Auxiliar gira para apartarse.
	bool rescatador_cerca = false;
	for (int i = 1; i <= 8; i++) {
		if (sensores.agentes[i] == 'r') {
			rescatador_cerca = true;
			break;
		}
	}

	if (rescatador_cerca) {
		// Si el Rescatador está cerca, gira para no ser un obstáculo.
		accion = TURN_SR;
	}
	else {
		// Si el Rescatador no está cerca, quédate quieto para no interferir.
		accion = IDLE;
	}
	
	return accion;
}
Action ComportamientoAuxiliar::ComportamientoAuxiliarNivel_3(Sensores sensores) {
	Action accion = IDLE;

	if (!hayPlan) {
		EstadoA inicio, fin;
		inicio.f = sensores.posF;
		inicio.c = sensores.posC;
		inicio.brujula = sensores.rumbo;
		inicio.zapatillas = tiene_zapatillas;

		fin.f = sensores.destinoF;
		fin.c = sensores.destinoC;

		plan = AStar(inicio, fin);
		if (plan.size() > 0) {
			hayPlan = true;
		}
	}

	if (hayPlan && plan.size() > 0) {
		accion = plan.front();
		plan.pop_front();
	} else {
		hayPlan = false;
	}
	
	return accion;
}
Action ComportamientoAuxiliar::ComportamientoAuxiliarNivel_4(Sensores sensores){
    
    return IDLE;
}