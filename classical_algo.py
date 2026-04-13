import numpy as np
import pandas as pd
import networkx as nx
import matplotlib as mpt
import math
import classes

DEPOT = None

def isReachable(distances, vehicle, tp):
    """checks if the current vehicle can reach the given node

    Args:
        distance ([[int]]): a matrix of distances from node to node
        vehicle (vehicle): a var to represent the vehicle delivering packages
        tp (node): the destination to check if it is reachable

    Returns:
        bool: true if node is reachable
    """
    # 1st check if capacity limits are okay
    under_weight_capacity = vehicle.remaining_weight >= tp.package_weight
    under_space_capacity = vehicle.remaining_volume >= tp.package_volume
    under_capacity = under_weight_capacity and under_space_capacity
    # then check if time restraints are obeyed
    time_to_candidate = distances[vehicle.current_location][tp.node_id]
    time_back_to_depot = distances[tp.node_id][DEPOT.node_id]
    can_arrive_on_time = (vehicle.time_spent + time_to_candidate) <= tp.deadline
    can_finish_workday = (vehicle.time_spent + time_to_candidate + time_back_to_depot) <= DEPOT.deadline
    return under_capacity and can_arrive_on_time and can_finish_workday

def pickNextRoute(vehicle, tpQueue, distances):
    """a method to select the next route

    Args:
        vehicle (vehicle): the vehicle performing the route
        tpQueue (queue[node]): a priority queue of nodes
        distances ([[int]]): a matrix of distances
    Returns:
        node: the node to create the next route to
    """
    tp = None
    if len(tpQueue) == 0:
        #scenario D, which is no priority nodes left
        tp = DEPOT
        return tp
    for tp in tpQueue:
        if isReachable(distances, vehicle, tp):
            # Scenario A/C - found a reachable TP
            tpQueue.remove(tp)
            return tp
    #Scenario B, no tp nodes are reachable by this vehicle so it goes to depot
    return DEPOT

def compute_distance_matrix(nodes):
    n = len(nodes)
    matrix = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            dx = nodes[i].x - nodes[j].x
            dy = nodes[i].y - nodes[j].y
            matrix[i][j] = math.sqrt(dx**2 + dy**2)
    return matrix

def readInData(filepath):
    """reads in the data in filename

    Args:
        filepath(string): a filepath to the data to read in

    Returns:
        ([vehicle], [node], [[int]]): a tuple of vehicles, nodes and a distance matrix
    """
    trucks = []
    nodes = []
    
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    section = None
    for line in lines:
        line = line.strip()
        
        # detect section headers
        if line in ('TRUCKS', 'DELIVERY_COORD', 'CHARACTERISTICS', 'EOF'):
            section = line
            continue
        
        # skip header metadata
        if ':' in line:
            continue
        
        if section == 'TRUCKS':
            parts = line.split()
            trucks.append(classes.Vehicle(
                max_weight=float(parts[1]),
                max_volume=float(parts[2]),
                rental_cost=float(parts[3])
            ))
        
        elif section == 'DELIVERY_COORD':
            parts = line.split()
            nodes.append({
                'node_id': int(parts[0]),
                'x': float(parts[1]),
                'y': float(parts[2])
            })
        
        elif section == 'CHARACTERISTICS':
            parts = line.split()
            node_id = int(parts[0])
            nodes[node_id]['package_weight'] = float(parts[1])
            nodes[node_id]['package_volume'] = float(parts[2])
            nodes[node_id]['deadline'] = float(parts[3])
    
    # convert node dicts to Node objects
    node_objects = [
        classes.Node(
            node_id=n['node_id'],
            x=n['x'],
            y=n['y'],
            package_weight=n['package_weight'],
            package_volume=n['package_volume'],
            deadline=n['deadline'],
            max_working_day=nodes[0]['deadline']
        )
        for n in nodes
    ]
    
    # compute distance matrix
    distance = compute_distance_matrix(node_objects)
    global DEPOT
    DEPOT= node_objects[0]
    return trucks, node_objects, distance

def all_delivered(nodes):
    """checks if all deliveries have been completed

    Args:
        nodes ([Node]): list of all nodes

    Returns:
        bool: true if all deliveries are complete
    """
    return all(node.delivered for node in nodes if node.node_id != DEPOT.node_id)

def getSubset(vehicle, nodes, distances, destination):
    subset = []

    for node in nodes:
        if node.delivered or node.node_id == DEPOT.node_id:
            continue
        
        if not isReachable(distances, vehicle, node):
            continue
        
        subset.append(node)
    #print(f"Subset: {[n.node_id for n in subset]}")
    time_dest_to_depot = distances[destination.node_id][DEPOT.node_id]
    return subset, min(destination.deadline - vehicle.time_spent,DEPOT.deadline - vehicle.time_spent - time_dest_to_depot)