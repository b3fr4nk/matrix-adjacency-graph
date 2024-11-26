import sys

class MatrixGraph:
    def __init__(self, vertices):
        self.graph = []
        self.vertices = vertices

        # Init graph to 0
        for i in range(0, self.vertices):
            self.graph.append([])
            for j in range(0, self.vertices):
                self.graph[i].append(0)

    def displayAdjacencyMatrix(self):
        print("visual representation of matrix", end="")
        for i in range(self.vertices):
            print()
            for j in range(self.vertices):
                print(self.graph[i][j], end="")
        print("\n")

    def addEdge(self, x, y, weight=1):
        # check if this is a valid connection
        if x >= self.vertices  or y >= self.vertices:
            print("Vertex does not exist")
        elif x == y:
            print("Same Vertex")
        else:
            # add Edge between the two vertexes
            self.graph[y][x] = weight
            self.graph[x][y] = weight

    def addVertex(self):
        self.vertices += 1

        new_row = []

        for i in range(self.vertices):
            new_row.append(0)
        self.graph.append(new_row)

        for i in range(self.vertices -1):
            self.graph[i].append(0)
            

    def removeVertex(self, v):
        self.displayAdjacencyMatrix()
        if v > self.vertices:
            print("Vertex does not exist")

        else:

            while v < self.vertices-1:

                # remove vertex from rows
                for i in range(self.vertices):
                    self.graph[i][v] = self.graph[i][v+1]

                
                # remove vertex from columns
                for i in range(self.vertices):
                    self.graph[v][i] = self.graph[v+1][i]

                v += 1

        self.vertices -= 1

# # get the smallest distance for dijkstra's algorithm
#     def smallestDistanceIndex(self, dist):
#         smallest_index = 0
#         for i in range(len(dist)):
#             if dist[i] is not None:
#                 if dist[i] < dist[smallest_index]:
#                     smallest_index = dist[i]
#         return smallest_index

# using dijkstra's algorithm to find the shortest possible distance from start to end
    def shortestPath(self, start, end):
        dist = [float('inf')] * self.vertices
        dist[start] = 0
        to_visit = set(range(self.vertices))

        # actually find the shortest path
        while to_visit:
            # find the smallest distance among those not yet visited
            current = min(to_visit, key=lambda vertex: dist[vertex])

            # if smallest distance is infinite path does not exist
            if dist[current] == float('inf'):
                break

            # Remove the vertex from the to-visit set
            to_visit.remove(current)

            # compute the distance for each possible next step
            for neighbor in to_visit:
                if self.graph[current][neighbor] > 0:  # Check if an edge exists
                    alt = dist[current] + self.graph[current][neighbor]
                    if alt < dist[neighbor]:
                        dist[neighbor] = alt

        # Return the shortest distance to the end vertex
        print(dist)
        return dist[end] if dist[end] != float('inf') else None
            

                
    
# creating objects of class Graph
obj = MatrixGraph(4)

# calling methods
obj.addEdge(0, 1, 2)
obj.addEdge(0, 2, 3)
obj.addEdge(1, 2, 4)
obj.addEdge(2, 3, 2)
obj.addEdge(3, 1, 5)
# the adjacency matrix created
obj.displayAdjacencyMatrix()

# adding a vertex to the graph
obj.addVertex()
# connecting that vertex to other existing vertices
obj.addEdge(4, 1)
obj.addEdge(4, 3)
# the adjacency matrix with a new vertex
obj.displayAdjacencyMatrix()

# print the shortest path between these two points
print(obj.shortestPath(4, 0))

# removing an existing vertex in the graph
# obj.removeVertex(1)
# the adjacency matrix after removing a vertex
obj.displayAdjacencyMatrix()


