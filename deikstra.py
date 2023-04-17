import sys
 
class Graph(object):
    def __init__(self, nodes, init_graph):
        self.nodes = nodes
        self.graph = self.construct_graph(nodes, init_graph)
        
    def construct_graph(self, nodes, init_graph):
        '''
        Этот метод обеспечивает симметричность графика. Другими словами, если существует путь от узла A к B со значением V, должен быть путь от узла B к узлу A со значением V.
        '''
        graph = {}
        for node in nodes:
            graph[node] = {}
        
        graph.update(init_graph)
        
        for node, edges in graph.items():
            for adjacent_node, value in edges.items():
                if graph[adjacent_node].get(node, False) == False:
                    graph[adjacent_node][node] = value
                    
        return graph
    
    def get_nodes(self):
        "Возвращает узлы графа"
        return self.nodes
    
    def get_outgoing_edges(self, node):
        "Возвращает соседей узла"
        connections = []
        for out_node in self.nodes:
            if self.graph[node].get(out_node, False) != False:
                connections.append(out_node)
        return connections
    
    def value(self, node1, node2):
        "Возвращает значение ребра между двумя узлами."
        return self.graph[node1][node2]
def dijkstra_algorithm(graph, start_node):
    unvisited_nodes = list(graph.get_nodes())
 
    # Мы будем использовать этот словарь, чтобы сэкономить на посещении каждого узла и обновлять его по мере продвижения по графику 
    shortest_path = {}
 
    # Мы будем использовать этот dict, чтобы сохранить кратчайший известный путь к найденному узлу
    previous_nodes = {}
 
    # Мы будем использовать max_value для инициализации значения "бесконечности" непосещенных узлов   
    max_value = sys.maxsize
    for node in unvisited_nodes:
        shortest_path[node] = max_value
    # Однако мы инициализируем значение начального узла 0  
    shortest_path[start_node] = 0
    
    # Алгоритм выполняется до тех пор, пока мы не посетим все узлы
    while unvisited_nodes:
        # Приведенный ниже блок кода находит узел с наименьшей оценкой
        current_min_node = None
        for node in unvisited_nodes: # Iterate over the nodes
            if current_min_node == None:
                current_min_node = node
            elif shortest_path[node] < shortest_path[current_min_node]:
                current_min_node = node
                
        # Приведенный ниже блок кода извлекает соседей текущего узла и обновляет их расстояния
        neighbors = graph.get_outgoing_edges(current_min_node)
        for neighbor in neighbors:
            tentative_value = shortest_path[current_min_node] + graph.value(current_min_node, neighbor)
            if tentative_value < shortest_path[neighbor]:
                shortest_path[neighbor] = tentative_value
                # Обновление лучшего пути
                previous_nodes[neighbor] = current_min_node
 
        # После посещения его соседей мы отмечаем узел как "посещенный"
        unvisited_nodes.remove(current_min_node)
    
    return previous_nodes, shortest_path
def print_result(previous_nodes, shortest_path, start_node, target_node):
    path = []
    node = target_node
    
    while node != start_node:
        path.append(node)
        node = previous_nodes[node]
 
   # Добавить начальный узел вручную
    path.append(start_node)
    
    print("Найден следующий лучший маршрут с ценностью {}.".format(shortest_path[target_node]))
    print(" -> ".join(reversed(path)))
nodes = ["A", "B", "C", "D", "E", "F", "G", "H", "J", "K", "L", "Z", "X", "V", "N", "Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P"]
 
init_graph = {}
for node in nodes:
    init_graph[node] = {}
    
init_graph["A"]["K"] = 7
init_graph["A"]["B"] = 12
init_graph["B"]["J"] = 9
init_graph["B"]["C"] = 10
init_graph["C"]["H"] = 11   
init_graph["C"]["D"] = 11
init_graph["D"]["G"] = 13
init_graph["D"]["E"] = 9
init_graph["E"]["F"] = 12
init_graph["K"]["J"] = 10
init_graph["K"]["N"] = 8
init_graph["J"]["H"] = 11
init_graph["H"]["X"] = 10
init_graph["H"]["G"] = 8
init_graph["G"]["Z"] = 12
init_graph["G"]["F"] = 7
init_graph["F"]["L"] = 16
init_graph["N"]["T"] = 16
init_graph["N"]["V"] = 11
init_graph["V"]["R"] = 15
init_graph["V"]["X"] = 10
init_graph["X"]["E"] = 8
init_graph["X"]["Z"] = 12
init_graph["Z"]["W"] = 11
init_graph["Z"]["L"] = 10
init_graph["L"]["Q"] = 12
init_graph["T"]["P"] = 7
init_graph["T"]["R"] = 2
init_graph["R"]["O"] = 5
init_graph["R"]["E"] = 9
init_graph["E"]["I"] = 14
init_graph["E"]["W"] = 16
init_graph["W"]["U"] = 10
init_graph["W"]["Q"] = 14
init_graph["Q"]["Y"] = 13
init_graph["P"]["O"] = 8
init_graph["O"]["I"] = 10
init_graph["I"]["U"] = 11
init_graph["U"]["Y"] = 12

graph = Graph(nodes, init_graph)
previous_nodes, shortest_path = dijkstra_algorithm(graph=graph, start_node="A")
print_result(previous_nodes, shortest_path, start_node="A", target_node="Y")