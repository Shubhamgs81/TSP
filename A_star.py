import heapq
from collections import defaultdict
import copy

# Defining Node class
class Node:
    def __init__(self, curr_node_id, parent, h, g, visited):
        # storing id of the state, curr_node_id = node number
        self.curr_node_id = curr_node_id
        # store parent of this state
        self.parent = parent
        # Heuristic, Cost, func
        self.h = h
        self.g = g
        self.f = self.g + self.h
        # to keep track of path and nodes visited
        self.visited = visited

    # Less Than Operator
    def __lt__(self, other):
        return self.f < other.f

# Check for cyclic graph
def check_cyclic(graph):
    for key in graph:
        if (len(graph[key]) < 2):
            return False
    return True

# Function to add edge to a graph
def add_edges(graph, node1, node2, cost):
    graph[node1][node2] = cost
    graph[node2][node1] = cost

# Function to find MST
def prims_MST(graph, path, start):
    pqueue = []
    total_cost = 0
    # ignore the nodes already in the path of the node
    visited = set(path)
    # Push Cost, Node
    heapq.heappush(pqueue, (0, start))

    # to bring start into the mst so that the path back to start node is accounterd for
    if start in visited:
        visited.remove(start)

    while pqueue:
        # pop the min cost to reach node from the heap
        cost, node = heapq.heappop(pqueue)
        if node not in visited:
            visited.add(node)
            total_cost += cost
            # Loop neighbors
            for neighbor, node_cost in graph[node].items():
                if neighbor not in visited:
                    # push the neighbor and cost to reach it
                    heapq.heappush(pqueue, (node_cost, neighbor))

    # check for if the augmented subgraph is connected or not
    for i in graph.keys():
        if i not in visited:
            # if not connected return infinity to suggest no way to reach the destination
            return float('inf')

    return total_cost

# Finding successors and calculating the heuristic
def find_successors(graph, curr_state):
    # storing successors of curr_state
    successors = []
    for successor, cost in graph[curr_state.curr_node_id].items():
        if not curr_state.visited[successor]:
            # copying the visited of parent as these states have been done in this path
            visited = copy.deepcopy(curr_state.visited)

            # making the current neighbour as visited
            visited[successor] = True

            # finding list of unvisited nodes
            unvisited = []
            visited_path=[]
            # enumerate:- index, value
            for i, v in enumerate(visited):
                if not v:
                    unvisited.append(i)
                else:
                    visited_path.append(i)
            # prims_MST of unvisited subgraph
            if(unvisited):
                h_val = prims_MST(graph, visited_path, unvisited[0])
            else:
                h_val=0
            # updating the new values in the new State
            temp_state = Node(successor, curr_state, h_val, curr_state.g + cost, visited)
            successors.append(temp_state)
    return successors

# Find Optimal TSP path
def astar_TSP(graph, numEdge):
    # prims_MST of whole graph
    h_val = prims_MST(graph, [], sorted(graph.keys())[0])
    # Node_id, Parent, h, g, visited_node
    curr_state = Node(0, None, h_val, 0, [False]*numEdge)

    # heap to pick the minimum state node from all the generated nodes based on f-val
    fringe_list = []
    fringe_list.append(curr_state)
    heapq.heapify(fringe_list)

    # if fringe_list becomes empty, no solution exists
    while fringe_list:
        curr_state = heapq.heappop(fringe_list)

        # if all vertices visited and back to start node, soltuion found
        if all(curr_state.visited) and curr_state.curr_node_id == 0:
            cost=curr_state.g
            path = []
            while curr_state.parent is not None:
                path.append(curr_state.curr_node_id)
                curr_state = curr_state.parent
            path.append(0)
            return path, cost

        # find successors of current vertice and append to fringe list
        else:
            successors = find_successors(graph, curr_state)
            for item in successors:
                heapq.heappush(fringe_list, item)

    print("TSP not possible\n")
    return [],0

if __name__ == '__main__':
    # store data values like a map key:value/dict pair
    graph = defaultdict(dict)
    graph_input = None
    path = []

    file = open("Input/set_3.txt", "r")
    lines = file.readline()
    numEdge = int(lines)
    # input file is in the form of adjacency matrix
    # so number of lines = total no of nodes in the graph
    lines = file.readlines()
    file.close()

    # Process Lines
    i = 0
    for line in lines:
        # Create List
        graph_input = line.split(" ")
        graph_input = graph_input[0:len(graph_input)-1]
        if graph_input is not None:
            j = 0
            for cost in graph_input:
                if cost != '-1':
                    add_edges(graph, i, j, float(cost))
                j += 1
        i += 1

    # Free Node Present
    if len(graph) != numEdge:
        print("\nTSP Not possible as graph not connected\n")

    if len(graph) != 0:
        # Call astar_TSP function which uses A* algorithm to exapnd nodes in order of non decresing f-values using
        # Prim's algorithm for MST as admisible Heurestics
        if check_cyclic(graph):
            path,cost = astar_TSP(graph, numEdge)
            print("\nTSP Solved at cost ", cost, '\n')
            print("Path of MSP: ", end=" ")
            print(path, '\n')
        else:
            print("\nTSP Not possible as graph not connected\n")
    else:
        print("\nGiven Graph is empty\n")
