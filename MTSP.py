import numpy as np
from random import uniform
from scipy.spatial import distance_matrix
from sklearn.cluster import AgglomerativeClustering
import matplotlib.pyplot as plt
from ortools.constraint_solver import pywrapcp

numberOfTrucks = 4
numberOfWorkOrders = 100


def newpoint():
    return uniform(-180,180), uniform(-90, 90)

# Distance callback
def create_distance_callback(distMatrix):
    # Create a callback to calculate distances between cities.

    def distance_callback(from_node, to_node):
        return distMatrix[from_node][to_node]

    return distance_callback

def route(locationNames, distances):

    tspSize = len(locationNames)
    numberOfRoutes = 1
    depotIndex = 0

    # Create routing model
    if tspSize > 0:
        routing = pywrapcp.RoutingModel(tspSize, numberOfRoutes, depotIndex)
        searchParameters = pywrapcp.RoutingModel.DefaultSearchParameters()
        # Create the distance callback.
        distCallback = create_distance_callback(distances)
        routing.SetArcCostEvaluatorOfAllVehicles(distCallback)
        # Solve the problem.
        assignment = routing.SolveWithParameters(searchParameters)
        if assignment:
        # Solution distance.
            print("Total distance: " + str(assignment.ObjectiveValue()) + " miles\n")
            # Display the solution.
            # Only one route here; otherwise iterate from 0 to routing.vehicles() - 1
            routeNumber = 0
            index = routing.Start(routeNumber) # Index of the variable for the starting node.
            route = ''
            while not routing.IsEnd(index):
            # Convert variable indices to node indices in the displayed route.
                index = assignment.Value(routing.NextVar(index))
                pathIndex.append(int(locationNames[routing.IndexToNode(index)]))

if __name__ == '__main__':
    depot = [newpoint()]
    stops = [newpoint() for i in range(numberOfWorkOrders)]
    locations = depot + stops
    locationNames = [str(i) for i in range(len(locations))]
    scale = 100
    distances = distance_matrix(locations, locations)
    
    x = [location[0] for location in locations]
    y = [location[1] for location in locations]
    
    clustering = AgglomerativeClustering(n_clusters = numberOfTrucks, affinity = "precomputed", linkage = "average").fit(distances)
    labels = clustering.labels_
    labels[0] = max(labels) + 1
    stopGroups = [[depot[0]] for i in range(max(labels))]
    stopGroupNames = [['0'] for i in range(max(labels))]
    
    for i in range(len(locations)):
        for j in range(max(labels)):
            if labels[i] == j:
                stopGroups[j].append(locations[i])
                stopGroupNames[j].append(str(i))
    
    paths = []
    plt.scatter(x, y, c = labels, cmap = "rainbow")
    
    for i in range(len(stopGroups)):
        pathIndex = []
        route(stopGroupNames[i], distance_matrix(stopGroups[i], stopGroups[i]))
        paths.append(np.array(depot + [locations[index] for index in pathIndex]))
        plt.plot(*paths[i].T)
    
    print(paths)
    
    plt.show()
