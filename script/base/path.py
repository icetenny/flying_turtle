import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import matplotlib.pyplot as plt


def DistanceMatrix(x1, y1, x2, y2):
    return int(math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))


def create_data_model(depot_point):
    # Sample location
    coor = {0: (10, 300),
            1: (300, 200),
            2: (155, 30),
            3: (400, 450),
            4: (250, 250),
            5: (375, 120)}

    distance_matrix = []
    for from_node in coor:
        row = []
        for to_node in coor:
            row.append(DistanceMatrix(
                coor[from_node][0], coor[from_node][1], coor[to_node][0], coor[to_node][1]))
        distance_matrix.append(row)

    # Create data model
    data = {
        "num_vehicles": 1,
        "depot": depot_point,
        "distance_matrix": distance_matrix,
        "coordinates": coor,
    }
    print("Data model created:")
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    max_route_distance = 0
    route = []
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += f" {manager.IndexToNode(index)} -> "
            route.append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        route.append(manager.IndexToNode(index))
        plan_output += f"{manager.IndexToNode(index)}\n"
        plan_output += f"Distance of the route: {route_distance}m\n"
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print(f"Maximum of the route distances: {max_route_distance}m")
    return route


def plot_route(data, route):
    coordinates = data["coordinates"]
    fig, ax = plt.subplots()
    ax.set_xlim(0, 500)
    ax.set_ylim(0, 500)

    # Plot points
    for point, coor in coordinates.items():
        ax.plot(coor[0], coor[1], 'bo')
        ax.text(coor[0], coor[1], f'{point}', fontsize=12, ha='right')

    # Plot path
    for i in range(len(route) - 1):
        start_point = coordinates[route[i]]
        end_point = coordinates[route[i + 1]]
        ax.plot([start_point[0], end_point[0]], [
                start_point[1], end_point[1]], 'r-')

    plt.show()


def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    depot_point = int(input("Enter start point: "))
    data = create_data_model(depot_point)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        30000000000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name,
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        route = print_solution(data, manager, routing, solution)
        plot_route(data, route)
    else:
        print("No solution found !")


if __name__ == "__main__":
    main()
