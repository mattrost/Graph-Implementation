# Course: CS261 - Data Structures
# Author: Matthew Rost
# Assignment 6: Graph Implementation (Portfolio Assignment)
# Description: Implementation of a Directed Graph using adjacency
#              matrix. This data structure will have the following methods:
#              add_vertex(), add_edge(), remove_edge(), get_vertices(),
#              get_edges(), is_valid_path(), dfs(), bfs(), has_cycle(),
#              and dijkstra().

class DirectedGraph:
    """
    Class to implement directed weighted graph
    - duplicate edges not allowed
    - loops not allowed
    - only positive edge weights
    - vertex names are integers
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency matrix
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.v_count = 0
        self.adj_matrix = []

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            v_count = 0
            for u, v, _ in start_edges:
                v_count = max(v_count, u, v)
            for _ in range(v_count + 1):
                self.add_vertex()
            for u, v, weight in start_edges:
                self.add_edge(u, v, weight)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        if self.v_count == 0:
            return 'EMPTY GRAPH\n'
        out = '   |'
        out += ' '.join(['{:2}'.format(i) for i in range(self.v_count)]) + '\n'
        out += '-' * (self.v_count * 3 + 3) + '\n'
        for i in range(self.v_count):
            row = self.adj_matrix[i]
            out += '{:2} |'.format(i)
            out += ' '.join(['{:2}'.format(w) for w in row]) + '\n'
        out = f"GRAPH ({self.v_count} vertices):\n{out}"
        return out

    # ------------------------------------------------------------------ #

    def add_vertex(self) -> int:
        """
        Description: Adds a new vertex to the graph. This will add another row
            and column to the adjacency matrix and increase the count. This
            returns the current count of vertices.

        Input: None
        Output: count
        """

        # Increase vertex count
        self.v_count += 1
        matrix = []

        if self.v_count > 1:
            # Iterate to build table to the proper length and add one column to
            # existing vertices.
            for n in range(self.v_count - 1):
                matrix.append(0)
                self.adj_matrix[n].append(0)

        matrix.append(0)

        self.adj_matrix.append(matrix)

        count = self.v_count

        return count

    def add_edge(self, src: int, dst: int, weight=1) -> None:
        """
        Description: Adds a new edge to the graph, Connecting the source to
            the destination. If either vertex does not exist, the weight is
            negative, or src is the same as dst, this method does nothing.
            This will update connection with the weight if there is already
            a weight. If the weight is currently 0, this sets it equal to
            the weight.

        Input: src, dst, weight
        Output: None
        """

        # Fail cases
        if weight < 0:
            return
        if src == dst:
            return
        if src > (self.v_count - 1):
            return
        if dst > (self.v_count - 1):
            return
        if src < 0:
            return
        if dst < 0:
            return

        # Update weight
        self.adj_matrix[src][dst] = weight

        return

    def remove_edge(self, src: int, dst: int) -> None:
        """
        Description: This method removes an edge between two vertices.
            If either vertex does not exist or if there is no edge, this
            does not do anything. Otherwise, this will set the weight
            of the connection to 0.

        Input: src, dst
        Output: None
        """
        if src == dst:
            return
        if src > (self.v_count - 1):
            return
        if dst > (self.v_count - 1):
            return
        if src < 0:
            return
        if dst < 0:
            return

        # Set weight to 0
        self.adj_matrix[src][dst] = 0

        return

    def get_vertices(self) -> []:
        """
        Description: This method creates a list of all of the vertices
            that are in the graph. That list is returned.

        Input: None
        Output: vertices
        """
        vertices = []

        # Vertices are basically just all the indices in our matrix.
        for index in range(0, self.v_count):
            vertices.append(index)

        return vertices

    def get_edges(self) -> []:
        """
        Description: This method creates a list of all of the edges
            that are in the graph. That list of edges is returned.
            The type of each element in that list is a tuple;
            formatted (source, destination, weight).

        Input: None
        Output: edges
        """
        edges = []

        for y in range(self.v_count):
            for x in range(self.v_count):
                # Check every index, if weight greater than 0 it is edge.
                if self.adj_matrix[y][x] > 0:
                    edges.append((y, x, self.adj_matrix[y][x]))

        return edges

    def is_valid_path(self, path: []) -> bool:
        """
        Description: This method takes a list of indices and determines
            if that sequence is a valid path to take in the graph.
            This returns True if so, False if not. An empty path is
            valid.

        Input: path
        Output: Boolean
        """
        for index in range(len(path)):

            # Check if index is valid
            if path[index] > (self.v_count - 1):
                return False
            if path[index] < 0:
                return False

            current = self.adj_matrix[path[index]]

            # If we are not on the last vertex of path, we need to make sure
            # that there is an edge from current to next vertex.
            if index != (len(path) - 1):
                if current[path[index + 1]] <= 0:
                    return False

        return True

    def dfs(self, v_start, v_end=None) -> []:
        """
        Description: This performs a depth-first search and returns
            a list of the visited vertices in the order they were
            visited. v_start is the starting vertex, and v_end is
            the optional ending vertex. If v_end is not entered,
            or is not visited, this will be done as if there is
            no end vertex. The method for picking the next vertex
            is to pick the vertices in ascending alphabetical order.

        Input: v_start, v_end(optional)
        Output: search
        """
        # Heavily based off of ud_graph's method
        # Referenced wikipedia

        search = []
        stack = []

        # Invalid starting vertex
        if v_start > (self.v_count - 1):
            return search

        stack.append(v_start)

        # Stack used to track vertices
        while len(stack) > 0:
            vertex = stack.pop()

            if vertex not in search:
                # Add to searched
                search.append(vertex)

                successors = []

                # Have to iterate over the matrix in this one
                for i in range(self.v_count):
                    if self.adj_matrix[vertex][i] > 0:
                        successors.append(i)

                successors.sort(reverse=True)
                for element in successors:
                    # Add to stack
                    stack.append(element)
                if v_end == vertex:
                    # We have reached the end!
                    return search

        return search

    def bfs(self, v_start, v_end=None) -> []:
        """
        Description: This performs a breadth-first search and returns
            a list of the visited vertices in the order they were
            visited. v_start is the starting vertex, and v_end is
            the optional ending vertex. If v_end is not entered,
            or is not visited, this will be done as if there is
            no end vertex. The method for picking the next vertex
            is to pick the vertices in ascending alphabetical order.

        Input: v_start, v_end(optional)
        Output: search
        """
        # Heavily based off of ud_graph's method
        # Referenced wikipedia

        search = []
        queue = []

        if v_start > (self.v_count - 1):
            return search

        queue.insert(0, v_start)
        search.append(v_start)

        # Queue used to track vertices
        while len(queue) > 0:
            vertex = queue.pop(0)
            if v_end == vertex:
                return search
            successors = []

            # Have to iterate over the matrix in this one as well
            for i in range(self.v_count):
                if self.adj_matrix[vertex][i] > 0:
                    successors.append(i)

            successors.sort()
            for element in successors:
                if element not in search:
                    # Add to the queue
                    queue.append(element)
                    search.append(element)
                if v_end == element:
                    # We have reached the end!
                    return search

        return search

    def has_cycle(self):
        """
        Description: This determines if a graph contains a cycle. If there
            is a cycle, this returns True. If there is not, this returns
            False.

        Input: None
        Output: boolean
        """
        # Based off of ud_graph cycle.

        for index in range(self.v_count):

            # Search is used to track all of the vertices we have looked at already
            search = []
            stack = []
            stack.append(index)

            # Stack is used to track the successors that have not yet been looked at
            while len(stack) > 0:
                vertex = stack.pop()
                if vertex not in search:
                    search.append(vertex)
                    successors = []

                    for i in range(self.v_count):
                        if self.adj_matrix[vertex][i] > 0:
                            successors.append(i)

                    for element in successors:
                        # We can go back to the source!
                        if element == index:
                            return True
                        stack.append(element)

        return False

    def dijkstra(self, src: int) -> []:
        """
        Description: This method performs Dijkstra's algorithm to
            determine the length of the shortest path from a given
            vertex, src, to all other vertices in the graph. This
            builds a list with one value per vertex that contains
            the minimum distance. If it is not possible to reach
            that vertex, this has 'inf' in that index for the vertex.
            This returns that list.

        Input: src
        Output: shortest_path
        """
        # Reference class module and wikipedia article

        # Uses a DFS search
        shortest_path = []
        queue = []

        for element in range(self.v_count):
            # Initiate the shortest path list with inf on vertices,
            # put 0 for source vertex.
            if element == src:
                shortest_path.append(0)
                queue.append([element, 0])
            else:
                shortest_path.append(float('inf'))

        while len(queue) > 0:
            # Popped is use to track both the vertex and distance from
            # the source.
            popped = queue.pop()
            vertex = popped[0]
            distance = popped[1]

            # Verify if we have a shorter distance than the shortest
            # path's stored one.
            if shortest_path[vertex] > distance:
                shortest_path[vertex] = distance

            for i in range(self.v_count):
                # See potential paths.
                if self.adj_matrix[vertex][i] > 0:
                    next_distance = self.adj_matrix[vertex][i] + distance

                    # If our potential move is shorter than the currently stored
                    # one we will save this new short distance and vertex into
                    # our queue.

                    if shortest_path[i] >= next_distance:
                        if len(queue) == 0:
                            queue.append([i, next_distance])
                        else:
                            queue.append([i, next_distance])

        return shortest_path




if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = DirectedGraph()
    print(g)
    for _ in range(5):
        g.add_vertex()
    print(g)

    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    for src, dst, weight in edges:
        g.add_edge(src, dst, weight)
    print(g)


    print("\nPDF - method get_edges() example 1")
    print("----------------------------------")
    g = DirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    test_cases = [[0, 1, 4, 3], [1, 3, 2, 1], [0, 4], [4, 0], [], [2]]
    for path in test_cases:
        print(path, g.is_valid_path(path))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for start in range(5):
        print(f'{start} DFS:{g.dfs(start)} BFS:{g.bfs(start)}')


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)

    edges_to_remove = [(3, 1), (4, 0), (3, 2)]
    for src, dst in edges_to_remove:
        g.remove_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')

    edges_to_add = [(4, 3), (2, 3), (1, 3), (4, 0)]
    for src, dst in edges_to_add:
        g.add_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')
    print('\n', g)


    print("\nPDF - dijkstra() example 1")
    print("--------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
    g.remove_edge(4, 3)
    print('\n', g)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
