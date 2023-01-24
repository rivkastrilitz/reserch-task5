import heapq
import doctest

class ShortestPathSolver:
    def __init__(self, algorithm, output_type='path'):
        self.algorithm = algorithm
        self.output_type = output_type
        
    def solve(self, data, start, end):

        """
        Find the shortest path using the selected algorithm.
      
        Examples:
        >>> matrix = [[0, 2, 3, 0], [0, 0, 0, 3], [0, 0, 0, 1], [0, 0, 0, 0]]
        >>> start = 0
        >>> end = 3
        >>> dijkstra_solver = ShortestPathSolver(DijkstraAlgorithm())
        >>> dijkstra_solver.solve(matrix, start, end)
        [0, 2, 3]
        >>> dijkstra_solver = ShortestPathSolver(DijkstraAlgorithm(), output_type='length')
        >>> dijkstra_solver.solve(matrix, start, end)
        4
        
        """
        if isinstance(self.algorithm, DijkstraAlgorithm):
            graph = self.__convert_matrix_to_graph(data) if type(data) == list else data
            path, length = self.algorithm.find_shortest_path(graph, start, end)
        else:
            graph = self.__convert_matrix_to_adj_list(data) if type(data) == list else data
            path = self.algorithm.find_shortest_path(graph, start, end)
            length = None if path is None else len(path)-1
        
        if self.output_type=='path':
            return path
        else:
            return length
        
    def __convert_matrix_to_graph(self, matrix):
        graph = {}
        for i in range(len(matrix)):
            graph[i] = {}
            for j in range(len(matrix)):
                if matrix[i][j] != 0:
                    graph[i][j] = matrix[i][j]
        return graph
    
    def __convert_matrix_to_adj_list(self, matrix):
        graph = {}
        for i in range(len(matrix)):
            graph[i] = []
            for j in range(len(matrix)):
                if matrix[i][j] != 0:
                    graph[i].append(j)
        return graph


class DijkstraAlgorithm:
     def find_shortest_path(self, graph, start, end):
        # Create a priority queue to store the nodes to visit
        queue = [(0, start)]
        # Create a dictionary to store the shortest distance to each node
        distances = {start: 0}
        # Create a dictionary to store the previous node in the shortest path
        previous = {start: None}
        while queue:
            # Get the node with the smallest distance
            current_distance, current_node = heapq.heappop(queue)
            # Check if we have reached the end node
            if current_node == end:
                # Reconstruct the shortest path
                path = []
                while current_node is not None:
                    path.append(current_node)
                    current_node = previous[current_node]
                return list(reversed(path)), distances[end]
            # Update the distances and previous node for each neighboring node
            for neighbor, weight in graph[current_node].items():
                distance = current_distance + weight
                if neighbor not in distances or distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_node
                    heapq.heappush(queue, (distance, neighbor))
        # If there is no path from start to end
        return None, float('inf')

class BFSAgorithm:
    def find_shortest_path(self, graph, start, end):
        # Create a queue to store the nodes to visit
        queue = [(start, [start])]
        # Create a set to store the visited nodes
        visited = set()
        while queue:
            # Get the next node to visit
            current_node, path = queue.pop(0)
            # Check if we have reached the end node
            if current_node == end:
                return path
            # Skip the node if it has been visited
            if current_node in visited:
                continue
            visited.add(current_node)
            # Add all the neighboring nodes to the queue
            for neighbor in graph[current_node]:
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))
        # If there is no path from start to end
        return None

# Usage example
matrix = [[0, 2, 3, 0], [0, 0, 0, 3], [0, 0, 0, 1], [0, 0, 0, 0]]  # Your graph representation as matrix
start = 0  # Starting node
end = 3  # Ending node

# Use Dijkstra algorithm to get the shortest path
dijkstra_solver = ShortestPathSolver(DijkstraAlgorithm())
shortest_path = dijkstra_solver.solve(matrix, start, end)
print("Dijkstra: Shortest path is", shortest_path)

# Use Dijkstra algorithm to get the shortest path length
dijkstra_solver = ShortestPathSolver(DijkstraAlgorithm(), output_type='length')
shortest_path_length = dijkstra_solver.solve(matrix, start, end)
print("Dijkstra: Shortest path length is", shortest_path_length)

# Use BFS algorithm to get the shortest path
bfs_solver = ShortestPathSolver(BFSAgorithm())
shortest_path = bfs_solver.solve(matrix, start, end)
print("BFS: Shortest path is", shortest_path)

# Use BFS algorithm to get the shortest path length
bfs_solver = ShortestPathSolver(BFSAgorithm(), output_type='length')
shortest_path_length = bfs_solver.solve(matrix, start, end)
print("BFS: Shortest path length is", shortest_path_length)


(failures, tests) = doctest.testmod(report=True, optionflags=doctest.NORMALIZE_WHITESPACE + doctest.ELLIPSIS)
print("{} failures, {} tests".format(failures, tests))