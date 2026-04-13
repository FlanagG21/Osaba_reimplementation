import classical_algo
import classes
import dimod

FILEPATH= "/work/Osaba_reimplementation/Osaba_paper_data/data/D14_P1.pdp"

Final_route = []

def find_route(vehicle, destination, nodes, distances):
    subset, rt = classical_algo.getSubset(vehicle, nodes, distances, destination)
    
    if len(subset) == 0:
        return [], 0

    subset = sorted(
        subset,
        key=lambda n: distances[vehicle.current_location][n.node_id])[:4]

    if destination.node_id != classical_algo.DEPOT.node_id and destination not in subset:
        subset.append(destination)

    if len(subset) == 0:
        return [], 0

    cqm = dimod.ConstrainedQuadraticModel()
    max_stops = min(5, len(subset))
    x = {}
    for node in subset:
        for p in range(max_stops):
            x[(node.node_id, p)] = dimod.Binary(f"x_{node.node_id}_{p}")

    # each node visited at most once
    for i in subset:
        cqm.add_constraint(
            sum(x[(i.node_id, p)] for p in range(max_stops)) <= 1
        )
    # at most one node per position
    for p in range(max_stops):
        cqm.add_constraint(
            sum(x[(i.node_id, p)] for i in subset) <= 1
        )
    # no gaps in sequence
    for p in range(max_stops - 1):
        cqm.add_constraint(
            sum(x[(i.node_id, p)] for i in subset)
            - sum(x[(i.node_id, p + 1)] for i in subset) >= 0
        )

    # destination constraint
    if destination.node_id != classical_algo.DEPOT.node_id:
        cqm.add_constraint(
            sum(x[(destination.node_id, p)] for p in range(max_stops)) == 1
        )
    else:
        cqm.add_constraint(
            sum(x[(i.node_id, p)] for i in subset for p in range(max_stops)) >= 1
        )

    # capacity constraints
    cqm.add_constraint(
        sum(x[(i.node_id, p)] * i.package_weight for i in subset for p in range(max_stops))
        <= vehicle.remaining_weight)
    cqm.add_constraint(
        sum(x[(i.node_id, p)] * i.package_volume for i in subset for p in range(max_stops))
        <= vehicle.remaining_volume)

    # objective 1: minimize distance
    if len(subset) >= 2:
        obj1 = sum(
            distances[i.node_id][j.node_id] * x[(i.node_id, p)] * x[(j.node_id, p + 1)]
            for i in subset for j in subset if i != j
            for p in range(max_stops - 1)
        )
        # time budget constraint
        cqm.add_constraint(obj1 <= rt)
    else:
        obj1 = 0.0 * list(x.values())[0]

    # objective 2: push destination to last position (only for TP nodes)
    if destination.node_id != classical_algo.DEPOT.node_id:
        obj2 = sum(
            -x[(destination.node_id, p)]
            - sum(x[(destination.node_id, p2)] for p2 in range(p, max_stops))
            for p in range(max_stops)
        )
        cqm.set_objective(obj1 + 2 * obj2)
    else:
        cqm.set_objective(obj1)

    solution = solve_cqm(cqm)
    if solution is None:
        return [], 0

    route_ids = extract_route(solution)
    route_nodes = [nodes[nid] for nid in route_ids]
    route_duration = compute_route_duration(route_ids, distances)
    return route_nodes, route_duration

def solve_cqm(cqm):
    sampler = dimod.ExactCQMSolver()
    sampleSet = sampler.sample_cqm(cqm)
    feasible = sampleSet.filter(lambda s: s.is_feasible)
    if len(feasible) == 0:
        return None
    return feasible.first.sample

def extract_route(best_sample):
    route_dict = {}
    print(f"Selected variables: {[var for var, val in best_sample.items() if val == 1]}")
    for var, value in best_sample.items():
        if value == 1:
            _, node_id, pos = var.split('_')
            pos = int(pos)
            node_id = int(node_id)
            route_dict[pos] = node_id
    print(f"Route dict: {route_dict}")
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
    while not len(vehicleStack) == 0 and not classical_algo.all_delivered(nodes):
        nextVehicle = vehicleStack.pop()
        nextDest = classical_algo.pickNextRoute(nextVehicle, tpQueue, distances)
        route, route_duration = find_route(nextVehicle, nextDest, nodes, distances)
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
            vehicleStack.insert(0, nextVehicle)
    

