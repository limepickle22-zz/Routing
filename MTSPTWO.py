import numpy as np
from random import uniform
from scipy.spatial import distance_matrix
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import matplotlib.pyplot as plt

def newpoint():
    return uniform(-180,180), uniform(-90, 90)

###########################
# Problem Data Definition #
###########################
def create_data_model():
    """Stores the data for the problem"""
    numberOfStops = 20
    data = {}
    # Locations in block units
    locations = [newpoint() for _ in range(numberOfStops)]
    # Multiply coordinates in block units by the dimensions of an average city block, 114m x 80m,
    # to get location coordinates.
    data["locations"] = [(l[0], l[1]) for l in locations]
    data["num_locations"] = len(data["locations"])
    data["num_vehicles"] = 4
    data["depot"] = 0
    return data
#######################
# Problem Constraints #
#######################
def create_distance_callback(data):
    """Creates callback to return distance between points."""
    distMatrix = distance_matrix(data["locations"], data["locations"])
    distances = {}
    for fromNode in range(data["num_locations"]):
        distances[fromNode] = {}
        for toNode in range(data["num_locations"]):
                distances[fromNode][toNode] = distMatrix[fromNode][toNode]

    def distance_callback(fromNode, toNode):
        """Returns the manhattan distance between the two nodes"""
        return distances[fromNode][toNode]

    return distance_callback

def add_distance_dimension(routing, distance_callback):
    """Add Global Span constraint"""
    distance = 'Distance'
    maximumDistance = 3000  # Maximum distance per vehicle.
    routing.AddDimension(
        distance_callback,
        0,  # null slack
        maximumDistance,
        True,  # start cumul to zero
        distance)
    distance_dimension = routing.GetDimensionOrDie(distance)
    # Try to minimize the max distance among vehicles.
    distance_dimension.SetGlobalSpanCostCoefficient(100)
###########
# Printer #
###########
def print_solution(data, routing, assignment):
    totalDistance = 0
    paths = []
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        planOutput = 'Route for vehicle {}:\n'.format(vehicle_id)
        distance = 0
        paths.append([])
        paths[vehicle_id].append([data["locations"][0][0], data["locations"][0][1]])
        while not routing.IsEnd(index):
            planOutput += ' {} ->'.format(routing.IndexToNode(index))
            previousIndex = index
            index = assignment.Value(routing.NextVar(index))
            if (index < data["num_locations"]):
                paths[vehicle_id].append([data["locations"][index][0], data["locations"][index][1]])
            else:
                paths[vehicle_id].append([data["locations"][0][0], data["locations"][0][1]])
        distance += routing.GetArcCostForVehicle(previousIndex, index, vehicle_id)
        planOutput += ' {}\n'.format(routing.IndexToNode(index))
        planOutput += 'Distance of route: {}m\n'.format(distance)
        print(planOutput)
        totalDistance += distance
    print('Total distance of all routes: {}m'.format(totalDistance))
    
    x = [data["locations"][i][0] for i in range(len(data["locations"]))]
    y = [data["locations"][i][1] for i in range(len(data["locations"]))]
    plt.scatter(x, y)
    paths = [np.array(path) for path in paths]
    for path in paths:
        plt.plot(*path.T)
    plt.show()
    
########
# Main #
########
def main():
    """Entry point of the program"""
    # Instantiate the data problem.
    data = create_data_model()
    
    # Create Routing Model
    routing = pywrapcp.RoutingModel(
        data["num_locations"],
        data["num_vehicles"],
        data["depot"])
    # Define weight of each edge
    distance_callback = create_distance_callback(data)
    routing.SetArcCostEvaluatorOfAllVehicles(distance_callback)
    add_distance_dimension(routing, distance_callback)
    # Setting first solution heuristic (cheapest addition).
    searchParameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    searchParameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC) # pylint: disable=no-member
    # Solve the problem.
    assignment = routing.SolveWithParameters(searchParameters)
    print_solution(data, routing, assignment)
    
if __name__ == '__main__':
    main()
