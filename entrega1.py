from simpleai.search import SearchProblem
from simpleai.search import breadth_first, depth_first, greedy, astar
from simpleai.search.viewers import WebViewer, ConsoleViewer, BaseViewer

def ValidarMovimiento(accion, camino):
    #Valida que no pueda moverme a una posicion por la que ya pase
    return accion not in camino

def RobotEnOrilla(pos):
    if pos[0] == 5 or pos[1] == 5 or pos[0] == 0 or pos[1] == 0:
        return True
    else:
        return False

def DistanciaOrillaCercana(pos):
    #Dada una posicion, busca la menor distancia a una orilla
    fila, columna = pos
    menorDistancia = 0

    if abs(0 - fila) < menorDistancia:
        menorDistancia = abs(0-fila)
    if abs(5 - fila) < menorDistancia:
        menorDistancia = abs(5-fila)
    if abs(0 - columna) < menorDistancia:
        menorDistancia = abs(0-fila)
    if abs(5 - columna) < menorDistancia:
        menorDistancia = abs(5-fila)

    return menorDistancia

def GenerarEstado(personas):
    # [[Posicion del robot], [Camino por donde ya pase], [Posicion de las personas], [Si el robot carga o no personas]]
    estado = [(0, 0), (), personas, 0]
    return tuple(estado)


class RescateHieloProblem(SearchProblem):
    def cost(self, state, action, state2):
        return 1

    def is_goal(self, state):
        #Valida que el robot no este cargarndo a nadie, y no queden personas por rescatar
        personas = len(state[2])
        cargado = state[3]

        if cargado == 0 and personas == 0:
            return True
        else:
            return False

    def actions(self, state):
        acciones_disponibles = []
        state = list(state)
        pos = state[0]
        fila = pos[0]
        columna = pos[1]
        caminoRecorrido = state[1]
        posNueva = ()

        if fila > 0: #arriba
            posNueva = (fila-1, columna)
            if ValidarMovimiento(posNueva, caminoRecorrido):
                acciones_disponibles.append(posNueva)


        if fila < 5: #abajo
            posNueva = (fila+1, columna)
            if ValidarMovimiento(posNueva, caminoRecorrido):
                acciones_disponibles.append(posNueva)

        if columna > 0: #izquierda
            posNueva = (fila, columna-1)
            if ValidarMovimiento(posNueva, caminoRecorrido):
                acciones_disponibles.append(posNueva)

        if columna < 5:  # derecha
            posNueva = (fila, columna + 1)
            if ValidarMovimiento(posNueva, caminoRecorrido):
                acciones_disponibles.append(posNueva)

        return acciones_disponibles

    def result(self, state, action):
        camino = list(state[1])
        personas = list(state[2])
        cargado = state[3]

        if RobotEnOrilla(action):
            cargado = 0
        else:
            camino.append(action)

        if action in personas:
            personas.remove(action)
            cargado = 1

        #estado = (action, camino, personas, cargado)
        estado = action, tuple(camino), tuple(personas), cargado

        return estado

    def heuristic(self, state):
        '''Calculo la suma de la distancia del robot hasta la persona mas lejana,
            desde alli a la persona mas cercana
            mas la distancia de la ultima con la orilla mas cercana'''

        filaRobot, columnaRobot = state[0]
        posicionPersonas = list(state[2])
        distanciaMax = 0
        distanciaMin = 100
        posicionMasLejano = []
        posicionMasCercano = []

        if len(posicionPersonas)> 1:
            #Distancia hasta persona mas lejana + distancia de ese a su mas cercano + distancia orilla cercana
            for p in posicionPersonas:
                distancia = abs(filaRobot - p[0]) + abs(columnaRobot - p[1])
                if distancia > distanciaMax:
                    distanciaMax = distancia
                    posicionMasLejano = (p[0], p[1])
            posicionPersonas.remove(posicionMasLejano)

            for p in posicionPersonas:
                distancia = abs(posicionMasLejano[0] - p[0]) + abs(posicionMasLejano[1] - p[1])
                if distancia < distanciaMin:
                    distanciaMin = distancia
                    posicionMasCercano = (p[0], p[1])

            distanciaSalida = DistanciaOrillaCercana(posicionMasCercano)

            return distanciaMax + distanciaMin + distanciaSalida

        elif len(posicionPersonas) == 1:
            #Queda solo una persona. Distancia hasta persona mas lejana + distancia orilla cercana
            distancia = abs(filaRobot - posicionPersonas[0][0]) + abs(columnaRobot - posicionPersonas[0][1])
            distanciaSalida = DistanciaOrillaCercana(posicionPersonas[0])

            return  distancia + distanciaSalida

        else:
            #No quedan personas por rescatar. Distancia hasta la orilla mas cercana
            return DistanciaOrillaCercana((filaRobot,columnaRobot))



def resolver(metodo_busqueda, posiciones_personas):
    inicial = GenerarEstado(posiciones_personas)
    problema = RescateHieloProblem(inicial)

    if metodo_busqueda == "breadth_first":
        return breadth_first(problema, graph_search=True, viewer=ConsoleViewer())

    if metodo_busqueda == "greedy":
        return greedy(problema, graph_search=True, viewer=ConsoleViewer())

    if metodo_busqueda == "depth_first":
        return depth_first(problema, graph_search=True, viewer=ConsoleViewer())

    if metodo_busqueda == "astar":
        return astar(problema, graph_search=True, viewer=ConsoleViewer())

if __name__ == '__main__':
    Inicial =  GenerarEstado(((2, 1), (3, 4), (4, 2)))
    problema = RescateHieloProblem(Inicial)
    print('Prueba astar')
    visor = BaseViewer()
    resultado = astar(problema, graph_search = True, viewer = visor)

    #print("path:", resultado.path())
    print("Profundidad:", len(resultado.path()))
    print("Estado:", resultado.state)
    print("Costo:", resultado.cost)
    print("Estadisticas:" , visor.stats)

