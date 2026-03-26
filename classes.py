DEPOT_ID = 0

class Vehicle:
    def __init__(self, max_weight, max_volume, rental_cost):
        self.remaining_weight = max_weight
        self.remaining_volume = max_volume
        self.current_location = DEPOT_ID
        self.time_spent = 0
        self.rental_cost = rental_cost

        
class Node:
    def __init__(self, node_id, x, y, package_weight, package_volume, deadline, max_working_day):
        self.node_id = node_id
        self.x = x
        self.y = y
        self.package_weight = package_weight
        self.package_volume = package_volume
        self.deadline = deadline
        self.is_tp = (node_id != DEPOT_ID) and (deadline < max_working_day)
        self.delivered = False