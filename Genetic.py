import random
from collections import defaultdict

INT_MAX = 999999999999.0

# Function to add edge to a graph
def add_edges(graph, node1, node2, cost):
    graph[node1][node2] = cost
    graph[node2][node1] = cost

# Function to read and process file
def readfile(file):
    # store data values like a map key:value/dict pair
    graph = defaultdict(dict)
    graph_input = None
    # Open file and read it
    file = open(file, "r")
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
        graph_input = graph_input[0:len(graph_input)]
        if graph_input is not None:
            j = 0
            for cost in graph_input:
                if cost != '-1':
                    add_edges(graph, i, j, float(cost))
                else:
                    add_edges(graph, i, j, INT_MAX)
                j += 1
        i += 1
    return graph,numEdge

# Check for cyclic graph
def check_cyclic(graph):
    for key in graph:
        if (len(graph[key]) < 2):
            return False
    return True

# Function that provides the evaluation value for each state
def fitness_func(path,graph,numEdge):
    cost=0
    # Calculate total Path Cost
    for i in range(1,numEdge):
        # Check if nodes are connected
        cost+=graph[path[i-1]][path[i]]
    cost+=graph[path[numEdge-1]][path[0]]
    return(1/cost)

# Selection Function to select parents for crossover
def selection_func(fitness_percent,arr,population_size):
    fitness_dict={fitness_percent[i]:arr[i] for i in range(population_size)}
    fitness_percent.sort()
    selection=[fitness_dict[fitness_percent[i]] for i in range(population_size//2,population_size)]
    return fitness_dict[fitness_percent[-1]],selection

# Crossover Function:- 2 Point crossover
# And all Node are unique
def crossover_func(parents,numEdge):
    child=[[],[]]
    a,b=random.sample(range(1, numEdge-1),2)
    if(a>b):
        a,b=b,a 
    for i in range(2):
        cross_section=parents[(i+1)%2][a:b]
        temp = [item for item in parents[i] if item not in cross_section]
        child[i].extend(temp[0:a])
        child[i].extend(cross_section)
        child[i].extend(temp[a:])
    return child

# Mutation Function to swap two node of path
def mutation_func(path,numEdge):
    while True:
        a,b = random.sample(range(1, numEdge),2)
        if a != b:
            path[a],path[b]=path[b],path[a]
            break
    return path

# Genetic Function
def genetic_TSP(graph,numEdge):
    # Defining the population size
    # Initialized maxvalue variable (stores max {1/cost} value)
    # Initializing the number of times we need to repeat evolution
    population_size=50
    maxvalue=-1
    generation_limit=10000
    
    # Initialized array to store first generation parents
    arr=[[] for i in range(population_size)]
    
    # Create Initial Population
    temp=[j for j in range(1,numEdge)]
    for i in range(population_size):
        arr[i]=[0]
        # Shuffle Except starting Node
        random.shuffle(temp)
        arr[i].extend(temp)
    
    counter=0
    # Stores minimal cost path
    path=[]
    
    while(counter<generation_limit):
        fitness=[]
        fitness_percent=[]
        fitness_sum=0
        
        # Calculate Fitness
        for i in range(population_size):
            cost=fitness_func(arr[i],graph,numEdge)
            fitness_sum+=cost
            fitness.append(cost)
            # If greater maxvalue obtained - updates it and optimal path
            if cost>maxvalue:
                maxvalue=cost
                path=arr[i]
        
        # Stores fitness percent value for each parent
        for i in range(population_size):
            fitness_percent.append((fitness[i]/fitness_sum)*100)    

        next_gen_parent=[]
        # Selection of parents
        parent_1,parent_2=selection_func(fitness_percent,arr,population_size)
        for i in range(0,population_size//2):
            # Cross Over
            child=crossover_func([parent_1,parent_2[i]],numEdge)
            # Mutation
            for j in range(2):
                child[j]=mutation_func(child[j],numEdge)
                # Add for next gen steps
                next_gen_parent.append(child[j])
                cost=fitness_func(child[j],graph,numEdge)
                # If greater maxvalue obtained - updates it and optimal path
                if(cost>maxvalue):
                    maxvalue=cost
                    path=child[j]
        arr=next_gen_parent
        counter+=1
    path.append(0)
    return path,1/maxvalue
    
# Main Function
if __name__ == '__main__':
    path = []
    graph,numEdge=readfile("Input/set_3.txt")

    # Free Node Present
    if len(graph) != numEdge:
        print("\nTSP Not possible as graph not connected\n")

    # Graph is not empty
    if len(graph) != 0:
        # Check Hamiltonian cycle
        if check_cyclic(graph):
            path,cost =genetic_TSP(graph,numEdge)
            print("\nTSP Solved at cost ", cost, '\n')
            print("Path of MSP: ", end=" ")
            print(path, '\n')
        else:
            print("\nTSP Not possible as graph not connected\n")
    else:
        print("\nGiven Graph is empty\n")    
