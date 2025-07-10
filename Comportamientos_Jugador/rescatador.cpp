// HAMZA FATA //
#include "../Comportamientos_Jugador/rescatador.hpp"
#include "motorlib/util.h"
#include <utility> // para std::pair
#include <queue> // para std::priority_queue
#include <set>   // para std::set

// -- FUNCIÓN COORDENADAS --
std::pair<int, int> getCoordsR(Sensores sensores, int i) {
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


Action ComportamientoRescatador::think(Sensores sensores)
{
	Action accion = IDLE;

	// Reiniciar el estado SOLO si el nivel ha cambiado desde el último ciclo
	if (sensores.nivel != last_level) {
		pasos_esquivando = 0;
		giro45Izq = 0;
		ultima_orientacion_camino = -1; // Reiniciar también la nueva variable
		last_level = sensores.nivel;
	}

	switch (sensores.nivel)
	{
	case 0:
		accion = ComportamientoRescatadorNivel_0 (sensores);
		break;
	case 1:
		accion = ComportamientoRescatadorNivel_1 (sensores);
		break;
	case 2:
        accion = ComportamientoRescatadorNivel_2(sensores);
		break;
	case 3:
        accion = ComportamientoRescatadorNivel_3(sensores);
		break;
	case 4:
        //accion = ComportamientoRescatadorNivel_4(sensores);
		break;
	}

	return accion;
}

char ViablePorAlturaR (char casilla, int dif, bool zap){
	if ((abs(dif) <=1) or (zap and abs(dif)<=2))
		return casilla;
	else
		return 'P';
}

int VeoCasillaInteresanteR (char si, char sc, char sd, char ai, char ac, char ad, bool zap, bool permite_senderos = true)
{
	if ( sc == 'X' && ac == '_') return 2;
	else if ( si == 'X' && ai == '_') return 1;
	else if ( sd == 'X' && ad == '_') return 3;
	else if(!zap){
		if (sc == 'D') return 2;
		else if (si == 'D') return 1;
		else if (sd == 'D') return 3;
	}
	
	if (sc == 'C') return 2;
	else if (si == 'C') return 1;
	else if (sd == 'C') return 3;
	else if (permite_senderos && sc == 'S') return 2;
	else if (permite_senderos && si == 'S') return 1;
	else if (permite_senderos && sd == 'S') return 3;
	
	else return 0;
}

void SituarSensorEnMapaR(vector<vector<unsigned char> > &m, vector<vector<unsigned char> > &a, Sensores sensores)
{
    m[sensores.posF][sensores.posC] = sensores.superficie[0];
    a[sensores.posF][sensores.posC] = sensores.cota[0];
    
    for (int i = 1; i <= 15; i++) {
        std::pair<int, int> coords = getCoordsR(sensores, i);
        if (coords.first >= 0 && coords.first < m.size() && coords.second >= 0 && coords.second < m[0].size()) {
            m[coords.first][coords.second] = sensores.superficie[i];
            a[coords.first][coords.second] = sensores.cota[i];
        }
    }
}

// --- Lógica de Búsqueda (Nivel 2) ---

int calculateCostR(char casilla_inicial, int dif_altura, Action accion) {
    int cost = 0;
    int incr_altura = (dif_altura != 0);

    if (accion == WALK) {
        switch(casilla_inicial) {
            case 'A': cost = 100 + incr_altura * 10; break;
            case 'B': cost = 20 + incr_altura * 5; break;
            case 'S': cost = 2 + incr_altura * 1; break;
            default: cost = 1 + incr_altura * 0; break;
        }
    } else if (accion == RUN) {
        switch(casilla_inicial) {
            case 'A': cost = 150 + incr_altura * 15; break;
            case 'T': cost = 35 + incr_altura * 5; break;
            case 'S': cost = 3 + incr_altura * 2; break;
            default: cost = 1 + incr_altura * 0; break;
        }
    } else if (accion == TURN_L) {
        switch(casilla_inicial) {
            case 'A': cost = 30; break;
            case 'T': cost = 5; break;
            default: cost = 1; break;
        }
    } else if (accion == TURN_SR) {
         switch(casilla_inicial) {
            case 'A': cost = 16; break;
            case 'T': cost = 3; break;
            default: cost = 1; break;
        }
    }
    return cost;
}

EstadoR applyR(Action accion, const EstadoR &st, const vector<vector<unsigned char>> &terreno, const vector<vector<unsigned char>> &mapaCotas) {
    EstadoR next = st;
    int new_f = st.fila, new_c = st.columna;
    int steps = (accion == RUN) ? 2 : 1;
    
    // Calcula la nueva posición para WALK o RUN
    if (accion == WALK || accion == RUN) {
        switch(st.orientacion) {
            case 0: new_f -= steps; break;
            case 1: new_f -= steps; new_c += steps; break;
            case 2: new_c += steps; break;
            case 3: new_f += steps; new_c += steps; break;
            case 4: new_f += steps; break;
            case 5: new_f += steps; new_c -= steps; break;
            case 6: new_c -= steps; break;
            case 7: new_f -= steps; new_c -= steps; break;
        }

        // Comprobación de validez de la casilla (simplificada, una real sería más compleja)
        if (new_f >= 0 && new_f < terreno.size() && new_c >=0 && new_c < terreno[0].size()) {
            char tipo_terreno = terreno[new_f][new_c];
            int dif_altura = abs(mapaCotas[new_f][new_c] - mapaCotas[st.fila][st.columna]);
            bool zapatillas_permite = st.tiene_zapatillas && dif_altura <= 2;

            if (tipo_terreno != 'P' && tipo_terreno != 'M' && (dif_altura <= 1 || zapatillas_permite)) {
                next.fila = new_f;
                next.columna = new_c;
            }
        }
    } else if (accion == TURN_L) {
        next.orientacion = (st.orientacion + 6) % 8; // Equivalente a -90 grados
    } else if (accion == TURN_SR) {
        next.orientacion = (st.orientacion + 1) % 8;
    }

    if (terreno[next.fila][next.columna] == 'D') {
        next.tiene_zapatillas = true;
    }
    
    return next;
}

list<Action> ComportamientoRescatador::Dijkstra(const EstadoR &inicio, const EstadoR &final) {
    priority_queue<NodoR, vector<NodoR>, greater<NodoR>> frontier;
    set<EstadoR> explored;
    
    NodoR nodo_inicial;
    nodo_inicial.st = inicio;
    nodo_inicial.coste = 0;
    frontier.push(nodo_inicial);

    while (!frontier.empty()) {
        NodoR current_node = frontier.top();
        frontier.pop();

        if (current_node.st.fila == final.fila && current_node.st.columna == final.columna) {
            return current_node.secuencia; // Solución encontrada
        }

        if (explored.count(current_node.st)) {
            continue; 
        }
        
        explored.insert(current_node.st);

        // Generar hijos (aplicar acciones)
        Action acciones[] = {WALK, RUN, TURN_L, TURN_SR};
        for (Action act : acciones) {
            NodoR child_node = current_node;
            child_node.st = applyR(act, current_node.st, mapaResultado, mapaCotas);

            // Si la acción produjo un cambio de estado
            if (!(child_node.st == current_node.st)) {
                int action_cost = calculateCostR(mapaResultado[current_node.st.fila][current_node.st.columna], 
                                                mapaCotas[child_node.st.fila][child_node.st.columna] - mapaCotas[current_node.st.fila][current_node.st.columna], act);
                child_node.coste = current_node.coste + action_cost;
                child_node.secuencia.push_back(act);
                frontier.push(child_node);
            }
        }
    }

    return list<Action>(); // No se encontró solución
}


int ComportamientoRescatador::interact(Action accion, int valor)
{
	return 0;
}

Action ComportamientoRescatador::ComportamientoRescatadorNivel_0(Sensores sensores)
{
	Action accion ;
	SituarSensorEnMapaR(mapaResultado,mapaCotas,sensores);
	if (sensores.superficie[0] == 'D')tiene_zapatillas = true;

	if (pasos_esquivando > 0) {
		accion = TURN_L;
		pasos_esquivando--;
		last_action = accion;
		return accion;
	}
	
	if (sensores.superficie[0] == 'X') {
		accion = IDLE; 
	}
	else if (giro45Izq !=0){
		accion = TURN_SR;
		giro45Izq--;
	}
	else {
		char i = ViablePorAlturaR(sensores.superficie[1], sensores.cota[1]-sensores.cota[0], tiene_zapatillas);
		char c = ViablePorAlturaR(sensores.superficie[2], sensores.cota[2]-sensores.cota[0], tiene_zapatillas);
		char d = ViablePorAlturaR(sensores.superficie[3], sensores.cota[3]-sensores.cota[0], tiene_zapatillas);

		int pos = VeoCasillaInteresanteR(i, c, d, sensores.agentes[1], sensores.agentes[2], sensores.agentes[3], tiene_zapatillas, false);
		switch (pos)
		{
		case 2: 
			if (sensores.agentes[2] == 'a') {
				pasos_esquivando = 2;
				accion = TURN_L;
			}
			else {
				accion = WALK;
				// Guardar la orientación al avanzar por un camino
				if (sensores.superficie[0] == 'C' || sensores.superficie[0] == 'S') {
					ultima_orientacion_camino = sensores.rumbo;
				}
			}
			break;
		case 1:
			giro45Izq = 1;
			accion = TURN_L;
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

Action ComportamientoRescatador::ComportamientoRescatadorNivel_1(Sensores sensores)
{
	Action accion = IDLE;
	SituarSensorEnMapaR(mapaResultado, mapaCotas, sensores);

	bool explorado = false;

	// Prioridad 1: Moverse hacia casillas desconocidas ('?')
	std::pair<int, int> coords;
	coords = getCoordsR(sensores, 2);
	if (mapaResultado[coords.first][coords.second] == '?' && (sensores.superficie[2] == 'C' || sensores.superficie[2] == 'S')) {
		accion = WALK;
		explorado = true;
	}

	if (!explorado) {
		coords = getCoordsR(sensores, 1);
		if (mapaResultado[coords.first][coords.second] == '?' && (sensores.superficie[1] == 'C' || sensores.superficie[1] == 'S')) {
			giro45Izq = 1;
			accion = TURN_L;
			explorado = true;
		}
	}
	
	if (!explorado) {
		coords = getCoordsR(sensores, 3);
		if (mapaResultado[coords.first][coords.second] == '?' && (sensores.superficie[3] == 'C' || sensores.superficie[3] == 'S')) {
			accion = TURN_SR;
			explorado = true;
		}
	}

	// Prioridad 2: Si no hay '?' a la vista, navegar por terreno conocido
	if (!explorado) {
		char i = ViablePorAlturaR(sensores.superficie[1], sensores.cota[1]-sensores.cota[0], tiene_zapatillas);
		char c = ViablePorAlturaR(sensores.superficie[2], sensores.cota[2]-sensores.cota[0], tiene_zapatillas);
		char d = ViablePorAlturaR(sensores.superficie[3], sensores.cota[3]-sensores.cota[0], tiene_zapatillas);
		int pos = VeoCasillaInteresanteR(i, c, d, sensores.agentes[1], sensores.agentes[2], sensores.agentes[3], tiene_zapatillas, true);

		switch (pos) {
			case 2: 
				accion = WALK; 
				break;
			case 1: 
				giro45Izq = 1; 
				accion = TURN_L; 
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
					}
				} else {
					accion = TURN_SR;
				}
				break;
		}
	}
	
	// Guardar la orientación si la acción final es avanzar
	if (accion == WALK) {
		if (sensores.superficie[0] == 'C' || sensores.superficie[0] == 'S' || sensores.superficie[2] == 'C' || sensores.superficie[2] == 'S') {
			ultima_orientacion_camino = sensores.rumbo;
		}
	}

	// Gestión de giros y colisiones
	if (giro45Izq > 0 && !explorado) {
		accion = TURN_SR;
		giro45Izq--;
	} else if (pasos_esquivando > 0) {
		accion = TURN_L;
		pasos_esquivando--;
	} else if (accion == WALK && sensores.agentes[2] == 'a') {
		pasos_esquivando = 2;
		accion = TURN_L;
	}

	last_action = accion;
	return accion;
}


Action ComportamientoRescatador::ComportamientoRescatadorNivel_2(Sensores sensores) {
	Action accion = IDLE;

	if (!hayPlan) {
		EstadoR inicio, fin;
		inicio.fila = sensores.posF;
		inicio.columna = sensores.posC;
		inicio.orientacion = sensores.rumbo;
		inicio.tiene_zapatillas = tiene_zapatillas;

		fin.fila = sensores.destinoF;
		fin.columna = sensores.destinoC;

		plan = Dijkstra(inicio, fin);
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

Action ComportamientoRescatador::ComportamientoRescatadorNivel_3(Sensores sensores) {
	Action accion = IDLE;

	// Lógica para no estorbar al Auxiliar.
	// Si el Auxiliar está cerca (en las primeras 8 casillas del sensor),
	// el Rescatador gira para apartarse.
	bool auxiliar_cerca = false;
	for (int i = 1; i <= 8; i++) {
		if (sensores.agentes[i] == 'a') {
			auxiliar_cerca = true;
			break;
		}
	}

	if (auxiliar_cerca) {
		// Si el auxiliar está cerca, gira para no ser un obstáculo.
		accion = TURN_SR;
	}
	else {
		// Si el auxiliar no está cerca, quédate quieto para no interferir.
		accion = IDLE;
	}
	
	return accion;
}

Action ComportamientoRescatador::ComportamientoRescatadorNivel_4(Sensores sensores)
{
}