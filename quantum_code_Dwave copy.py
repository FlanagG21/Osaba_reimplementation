import classical_algo
import classes
import dimod
from dwave.system import LeapHybridCQMSampler
FILEPATH= ""


def find_route(vehicle, destination, nodes, distances):
    """

    Returns:
        _type_: _description_
    """
    #TODO implement dwave runner for finding a path
    return None

if __name__ == "__main__":
    vehicles, nodes, distances = classical_algo.readInData(FILEPATH)
    vehicleStack = None  # TODO: implement stack
    tpQueue = None       # TODO: implement priority queue
    while not vehicleStack.isempty() and not classical_algo.all_delivered(nodes):
        nextVehicle = vehicleStack.pop()
        nextDest = classical_algo.pickNextRoute(nextVehicle, tpQueue, distances)
        route_duration, route = find_route(nextVehicle, nextDest, nodes, distances)
        for node in route:
            nodes[node.node_id].delivered = True
            nextVehicle.remaining_weight -= node.package_weight
            nextVehicle.remaining_volume -= node.package_volume
        nextVehicle.time_spent += route_duration
        nextVehicle.current_location = nextDest.node_id
        
        if (nextDest.node_id != classical_algo.DEPOT.node_id):
            vehicleStack.push(nextVehicle)

