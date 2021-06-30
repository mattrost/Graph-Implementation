# Course: CS261 - Data Structures
# Author: Matthew Rost
# Assignment 6: Graph Implementation (Portfolio Assignment)
# Description: Implementation of an Undirected Graph using adjacency
#              list. This data structure will have the following methods:
#              add_vertex(), add_edge(), remove_edge(), remove_vertex(),
#              get_vertices(), get_edges(), is_valid_path(), dfs(), bfs(),
#              count_connected_components(), and has_cycle().


class UndirectedGraph:
    """
    Class to implement undirected graph
    - duplicate edges not allowed
    - loops not allowed
    - no edge weights
    - vertex names are strings
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency list
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.adj_list = dict()

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            for u, v in start_edges:
                self.add_edge(u, v)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        out = [f'{v}: {self.adj_list[v]}' for v in self.adj_list]
        out = '\n  '.join(out)
        if len(out) < 70:
            out = out.replace('\n  ', ', ')
            return f'GRAPH: {{{out}}}'
        return f'GRAPH: {{\n  {out}}}'

    # ------------------------------------------------------------------ #

    def add_vertex(self, v: str) -> None:
        """
        Description: Adds a new vertex to the graph. Vertices can be any
            string. If there is a vertex with the same name, this does
            not do anything.

        Input: v
        Output: None
        """
        if v in self.adj_list:
            return

        # Create a new dictionary key
        else:
            self.adj_list[v] = []

    def add_edge(self, u: str, v: str) -> None:
        """
        Description: Adds a new edge to the graph. If one or both of the
            vertices do not exist, this method will create them and then
            create the edge between them. If u and v are the same or if
            the edge exists, this method does nothing.

        Input: u, v
        Output: None
        """
        if u == v:
            return

        # Create new dictionary keys if necessary
        if u not in self.adj_list:
            self.add_vertex(u)
        if v not in self.adj_list:
            self.add_vertex(v)

        # Add to dictionary values if necessary
        if v not in self.adj_list[u]:
            self.adj_list[u].append(v)
        if u not in self.adj_list[v]:
            self.adj_list[v].append(u)

    def remove_edge(self, v: str, u: str) -> None:
        """
        Description: This removes an edge between two vertices of u
            and v. If there is no edge between these or the vertices
            do not exist, this method does not do anything.

        Input: v, u
        Output: None
        """
        if u == v:
            return

        if u not in self.adj_list:
            return
        if v not in self.adj_list:
            return

        if v not in self.adj_list[u]:
            return
        if u not in self.adj_list[v]:
            return

        # Remove values from key value pairs
        self.adj_list[u].remove(v)
        self.adj_list[v].remove(u)

    def remove_vertex(self, v: str) -> None:
        """
        Description: This removes a vertex and all of the edges
            connected to it. If the vertex doesn't exist, this
            does not do anything.

        Input: v
        Output: None
        """
        if v not in self.adj_list:
            return

        del self.adj_list[v]

        # Check to see if v is in any of the values in the adj_list to remove
        for key in self.adj_list:
            if v in self.adj_list[key]:
                self.adj_list[key].remove(v)

    def get_vertices(self) -> []:
        """
        Description: Returns a list of all the vertices in the graph.

        Input: None
        Output: vertices
        """
        vertices = []

        for key in self.adj_list:
            vertices.append(key)

        return vertices

    def get_edges(self) -> []:
        """
        Description: Returns a list of all the edges in the graph.

        Input: None
        Output: edges
        """
        edges = []

        for key in self.adj_list:
            for value in self.adj_list[key]:
                # Need to see if reversed tuple is in the edges list
                if (value, key) not in edges:
                    edges.append((key, value))

        return edges

    def is_valid_path(self, path: []) -> bool:
        """
        Description: Takes a list of vertex names and returns True
            if the sequence of vertices has a valid path in the graph.
            Empty paths are valid as well. If this is a valid path,
            this returns True. If there is no valid path, False.

        Input: path
        Output:
        """
        for index in range(len(path)):
            current = path[index]

            if current not in self.adj_list:
                return False

            # If we are not on the last vertex of path, we need to make sure
            # that there is an edge from current to next vertex.
            if index != (len(path) - 1):
                next_vertex = path[index + 1]

                if next_vertex not in self.adj_list[current]:
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

        # Referenced wikipedia article and module
        search = []
        stack = []

        if v_start not in self.adj_list:
            return search

        stack.append(v_start)

        # Stack is used to track the successors that have not yet been looked at
        while len(stack) > 0:
            vertex = stack.pop()
            if vertex not in search:
                search.append(vertex)
                successors = self.adj_list[vertex]
                # Reverse so the lowest alphabetical are on top of stack
                successors.sort(reverse=True)
                for element in successors:
                    stack.append(element)
                if v_end == vertex:
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

        # Referenced wikipedia article and module
        search = []
        queue = []

        if v_start not in self.adj_list:
            return search

        queue.insert(0, v_start)
        search.append(v_start)

        # Queue is used to track the successors that have not yet been looked at
        while len(queue) > 0:
            vertex = queue.pop(0)
            if v_end == vertex:
                return search
            successors = self.adj_list[vertex]
            successors.sort()
            for element in successors:
                if element not in search:
                    queue.append(element)
                    search.append(element)
                if v_end == element:
                    return search

        return search

    def count_connected_components(self):
        """
        Description: This counts the number of connected components within
            the graph. A connected component is like the part of the graph
            where vertices are connected by edges. If a vertex is on an
            island, it is on a different component than where the other
            vertices are.

        Input: None
        Output: count
        """
        count = 0
        visited = []

        for key in self.adj_list:
            # Basically just iterate DFS over every key yet to be visited
            if key in visited:
                continue

            count += 1
            stack = [key]

            # Do a DFS over key to see what all it can contact.
            while len(stack) > 0:
                current = self.adj_list[stack.pop()]
                for value in current:
                    if value not in visited:
                        stack.append(value)
                        visited.append(value)

        return count

    def has_cycle(self):
        """
        Description: This determines if a graph contains a cycle. A cycle
            is a closed walk that enters and leaves each vertex at most
            once. Returns True if there is a cycle and False if the graph
            does not.

        Input: None
        Output: boolean
        Return True if graph contains a cycle, False otherwise
        """

        for key in self.adj_list:
            # Search tracks vertices that we have already looked at
            search = []

            # Stack and parents are both popped at the same time to track the parent of the
            # current vertex.
            stack = []
            parents = []
            stack.append(key)

            # Stack is used to track the successors that have not yet been looked at
            while len(stack) > 0:
                if len(parents) > 0:
                    parent = parents.pop()
                else:
                    parent = None

                vertex = stack.pop()
                if vertex not in search:
                    search.append(vertex)
                    successors = self.adj_list[vertex]
                    for element in successors:
                        if element == key and parent != key:
                            # Able to complete the walk, cant have a walk that goes from
                            # one vertex, to another, then back so we check parent.
                            return True
                        parents.append(vertex)
                        stack.append(element)

        return False


if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = UndirectedGraph()
    print(g)

    for v in 'ABCDE':
        g.add_vertex(v)
    print(g)

    g.add_vertex('A')
    print(g)

    for u, v in ['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE', ('B', 'C')]:
        g.add_edge(u, v)
    print(g)


    print("\nPDF - method remove_edge() / remove_vertex example 1")
    print("----------------------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    g.remove_vertex('DOES NOT EXIST')
    g.remove_edge('A', 'B')
    g.remove_edge('X', 'B')
    print(g)
    g.remove_vertex('D')
    print(g)


    print("\nPDF - method get_vertices() / get_edges() example 1")
    print("---------------------------------------------------")
    g = UndirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE'])
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    test_cases = ['ABC', 'ADE', 'ECABDCBE', 'ACDECB', '', 'D', 'Z']
    for path in test_cases:
        print(list(path), g.is_valid_path(list(path)))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = 'ABCDEGH'
    for case in test_cases:
        print(f'{case} DFS:{g.dfs(case)} BFS:{g.bfs(case)}')
    print('-----')
    for i in range(1, len(test_cases)):
        v1, v2 = test_cases[i], test_cases[-1 - i]
        print(f'{v1}-{v2} DFS:{g.dfs(v1, v2)} BFS:{g.bfs(v1, v2)}')


    print("\nPDF - method count_connected_components() example 1")
    print("---------------------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print(g.count_connected_components(), end=' ')
    print()


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG',
        'add FG', 'remove GE')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print('{:<10}'.format(case), g.has_cycle())
