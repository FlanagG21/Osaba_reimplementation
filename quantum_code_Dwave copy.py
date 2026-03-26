import classical_algo
import classes
import dimod

FILEPATH= ""


def find_route(vehicle, destination, nodes, distances):
    """

    Returns:
        _type_: _description_
    """
    #TODO implement dwave runner for finding a path
    subset, rt = classical_algo.getSubset(vehicle, nodes, distances, destination)
    start_node = vehicle.current_location
    if isinstance(start_node, int):
        start_node = nodes[start_node]

    if destination not in subset:
        subset.append(destination)
    # Create CQM
    cqm = dimod.ConstrainedQuadraticModel()

    x = {}
    for node in subset:
        for p in range(len(subset)):
            x[(node.node_id, p)] = dimod.Binary(f"x_{node.node_id}_{p}")
    for i in subset:
        cqm.add_constraint(
            sum(x[(i.node_id,p)] for p in range(len(subset))) <= 1
        )
    for p in range(len(subset)):
        cqm.add_constraint(
            sum(x[(i.node_id,p)] for i in subset) <= 1
        )
    for p in range(len(subset)-1):
        cqm.add_constraint(
            sum(x[(i.node_id,p)] for i in subset)
            - sum(x[(i.node_id,p+1)] for i in subset) >= 0
        )
    cqm.add_constraint(sum(x[(destination.node_id, p)] for p in range(len(subset))) == 1)
    cqm.add_constraint(
    sum(
        distances[i.node_id][j.node_id] * x[(i.node_id,p)] * x[(j.node_id,p+1)]
        for i in subset for j in subset if i != j
        for p in range(len(subset)-1)
    ) <= rt)
    cqm.add_constraint(
    sum(x[(i.node_id,p)] * i.package_weight for i in subset for p in range(len(subset)))
    <= vehicle.remaining_weight)

    cqm.add_constraint(
        sum(x[(i.node_id,p)] * i.package_volume for i in subset for p in range(len(subset)))
        <= vehicle.remaining_volume)
    # distance objective (o1)
    obj1 = sum(
        distances[i.node_id][j.node_id] * x[(i.node_id,p)] * x[(j.node_id,p+1)]
        for i in subset for j in subset if i != j
        for p in range(len(subset)-1)
    )

    # destination lateness objective (o2)
    obj2 = sum(
        -x[(destination.node_id, p)]
        - sum(x[(destination.node_id, p2)] for p2 in range(p, len(subset)))
        for p in range(len(subset))
    )

    cqm.set_objective(obj1 + 2*obj2)
    solution= solve_cqm(cqm)
    route_ids = extract_route(solution)
    route_nodes = [nodes[nid] for nid in route_ids]
    route_duration = compute_route_duration(route_ids, distances)
    return route_nodes, route_duration


def solve_cqm(cqm):
    sampler = dimod.SimulatedAnnealingCQMSampler()
    solution = sampler.sample_cqm(cqm, num_reads=100)
    best = solution.first  # take the best feasible solution
    return best

def extract_route(best_sample):
    # Create a dict: position → node
    route_dict = {}
    for var, value in best_sample.items():
        if value == 1:
            _, node_id, pos = var.split('_')  # 'x_nodeId_pos'
            pos = int(pos)
            node_id = int(node_id)
            route_dict[pos] = node_id

    # Build the route in order of positions
    route = [route_dict[p] for p in sorted(route_dict)]
    return route

def compute_route_duration(route, distances):
    total_time = 0
    for i in range(len(route) - 1):
        total_time += distances[route[i]][route[i+1]]
    return total_time

if __name__ == "__main__":
    vehicles, nodes, distances = classical_algo.readInData(FILEPATH)
    owned = sorted([v for v in vehicles if v.rental_cost == 0], key=lambda v: v.remaining_weight * v.remaining_volume)
    rental = sorted([v for v in vehicles if v.rental_cost > 0], key=lambda v: v.remaining_weight * v.remaining_volume)
    vehicleStack = owned + rental
    tpQueue = sorted([node for node in nodes if node.is_tp], key=lambda n: n.deadline)
    while not vehicleStack.isempty() and not classical_algo.all_delivered(nodes):
        nextVehicle = vehicleStack.pop()
        nextDest = classical_algo.pickNextRoute(nextVehicle, tpQueue, distances)
        route_duration, route = find_route(nextVehicle, nextDest, nodes, distances)
        for node in route:
            nodes[node.node_id].delivered = True
            nextVehicle.remaining_weight -= node.package_weight
            nextVehicle.remaining_volume -= node.package_volume
            if node.is_tp:
                try:
                    tpQueue.remove(node)
                except ValueError:
                    pass
        nextVehicle.time_spent += route_duration
        nextVehicle.current_location = nextDest.node_id
        
        if (nextDest.node_id != classical_algo.DEPOT.node_id):
            vehicleStack.push(nextVehicle)

