import pandas as pd
import numpy as np
import heapq

"""
Location

Represents a location on the map
label: name of location
latitude, longitude: self-explanatory
"""
class Location:
    def __init__(self, label, latitude, longitude):
        self.label = label
        self.latitude = latitude
        self.longitude = longitude

"""
Edge

Represents an edge connecting two locations
label: name of edge
location1, location2: locations the edge connects
distance: distance one travels along the edge from location1 to location2
"""
class Edge:
    def __init__(self, label, location1, location2, distance):
        self.label = label
        self.location1 = location1
        self.location2 = location2
        self.distance = distance

"""
read_locations

Reads in values from location csv file, creates an instance of Location with the parsed attributes.
"""
def read_locations(location_file):
    df = pd.read_csv(location_file)
    locations = []
    for _, row in df.iterrows():
        label = row['location_label']
        latitude = row['latitude']
        longitude = row['longitude']
        locations.append(Location(label, float(latitude), float(longitude)))
    return locations

"""
read_edges

Reads in values from edge csv file, creates an instance of Edge with the parsed attributes.
"""
def read_edges(edge_file):
    df = pd.read_csv(edge_file, sep='\t')
    edges = []
    for _, row in df.iterrows():
        label = row['edgeLabel']
        location1 = row['locationA']
        location2 = row['locationB']
        distance = row['actualDistance']
        edges.append(Edge(label, location1, location2, distance))
    return edges


class SearchNode:
    def __init__(self, state, parent=None):
        self.state = state
        self.parent = parent
        self.g_cost = 0  # Cost from start to current node
        self.h_cost = 0  # Heuristic cost from current node to goal
        self.f_cost = 0  # Total cost (g_cost + h_cost)

    def __lt__(self, other):
        return self.f_cost < other.f_cost  # Needed for heapq


"""
Nick: ChatGPT generated astar function, this is what I am working on rn
"""
def astar_search(startLoc, goalLoc):
    frontier = []  # Priority queue (heap) for nodes to be explored
    reached = set()  # Set to store explored nodes

    start_node = SearchNode(startLoc)
    start_node.g_cost = 0
    start_node.h_cost = calculate_h_cost(start_node, goalLoc)
    start_node.f_cost = start_node.g_cost + start_node.h_cost

    heapq.heappush(open_set, start_node)

    while frontier:
        current_node = heapq.heappop(frontier)

        if current_node.state == goalLoc:
            # Goal reached, construct and return the path
            return construct_path(current_node)

        reached.add(current_node.state)

        # Generate successor nodes
        for neighbor_state in get_neighbors(current_node.state):
            if neighbor_state in reached:
                continue

            neighbor_node = SearchNode(neighbor_state, parent=current_node)
            neighbor_node.g_cost = current_node.g_cost + distance(current_node, neighbor_node)
            neighbor_node.h_cost = calculate_h_cost(neighbor_node, goal_state)
            neighbor_node.f_cost = neighbor_node.g_cost + neighbor_node.h_cost

            # Check if neighbor is in open_set and has a lower f_cost
            if not is_in_open_set(frontier, neighbor_node) or neighbor_node.g_cost < get_g_cost(frontier,
                                                                                                neighbor_node):
                heapq.heappush(frontier, neighbor_node)

    # No path found
    return None


def RoadTrip (startLoc, goalLoc, LocFile, EdgeFile, resultFile):

    # Read in edges and locations and place information in Edge and Location objects
    edges = read_edges(EdgeFile)
    locations = read_locations(LocFile)


    startLocation = next((loc for loc in locations if loc.label == startLoc), None)
    goalLocation = next((loc for loc in locations if loc.label == goalLoc), None)























