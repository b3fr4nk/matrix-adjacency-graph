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

# get the smallest distance for dijkstra's algorithm
    def smallestDistanceIndex(self, dist):
        smallest_index = 0
        for i in range(len(dist)):
            if dist[i] is not None:
                if dist[i] < dist[smallest_index]:
                    smallest_index = dist[i]
        return smallest_index

# using dijkstra's algorithm to find the shortest possible distance from start to end
    def shortestPath(self, start, end):
        dist = []
        to_visit = []

        # initialize the distances between each vertex and the available vertices
        for i in range(self.vertices):
            dist.append(None)
            to_visit.append(i)
        dist[0] = 0

        # actually find the shortest path
        while len(to_visit) > 0:
            # grab the next vertex in the queue
            v = to_visit.pop(self.smallestDistanceIndex(dist))

            # compute the distance for each possible next step
            for n in to_visit:
                if self.graph[v][n] > 0: # check to make sure there is an edge between these two vertices
                    alt = dist[v] + self.graph[v][n]
                    if dist[n] is None:
                        dist[n] = alt
                    elif alt < dist[n]:
                        dist[n] = alt

        return dist[end] - dist[start]
            

                
    
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
print(obj.shortestPath(0, 4))

# removing an existing vertex in the graph
# obj.removeVertex(1)
# the adjacency matrix after removing a vertex
obj.displayAdjacencyMatrix()


