import math
from heapq import heappush, heappop  # Recommended.
import numpy as np

from flightsim.world import World

from .occupancy_map import OccupancyMap # Recommended.

def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))

    cost = 1

    if astar is True:
        heuristic_cost = 0.7 #A star
    else:
        heuristic_cost = 0 #Dijkstra's

    queue = []
    heappush(queue, (0, start_index))
    visiting_cost = {start_index: 0}
    visited = {start_index: None}

    while queue:
        current_cost, current_node = heappop(queue)
        if current_node == goal_index:
            break

        unvisited_nodes = [(current_node[0]+1, current_node[1], current_node[2]),
                           (current_node[0]-1, current_node[1], current_node[2]),
                           (current_node[0], current_node[1]+1, current_node[2]),
                           (current_node[0], current_node[1]-1, current_node[2]),
                           (current_node[0], current_node[1], current_node[2]+1),
                           (current_node[0], current_node[1], current_node[2]-1), #6

                           (current_node[0]+1, current_node[1]+1, current_node[2]),
                           (current_node[0]+1, current_node[1]-1, current_node[2]),
                           (current_node[0]+1, current_node[1], current_node[2]+1),
                           (current_node[0]+1, current_node[1], current_node[2]-1), #10

                           (current_node[0]-1, current_node[1]+1, current_node[2]),
                           (current_node[0]-1, current_node[1]-1, current_node[2]),
                           (current_node[0]-1, current_node[1], current_node[2]+1),
                           (current_node[0]-1, current_node[1], current_node[2]-1), #14

                           (current_node[0], current_node[1]+1, current_node[2]+1),
                           (current_node[0], current_node[1]+1, current_node[2]-1), #16

                           (current_node[0], current_node[1]-1, current_node[2]+1),
                           (current_node[0], current_node[1]-1, current_node[2]-1), #18

                           (current_node[0]+1, current_node[1]+1, current_node[2]+1),
                           (current_node[0]+1, current_node[1]+1, current_node[2]-1),
                           (current_node[0]-1, current_node[1]+1, current_node[2]+1),
                           (current_node[0]-1, current_node[1]+1, current_node[2]-1), #22

                           (current_node[0]+1, current_node[1]-1, current_node[2]+1),
                           (current_node[0]-1, current_node[1]-1, current_node[2]+1),
                           (current_node[0]+1, current_node[1]-1, current_node[2]-1),
                           (current_node[0]-1, current_node[1]-1, current_node[2]-1)] #26

        for unvisited_node in unvisited_nodes:
            node_neighbor = unvisited_node

            if occ_map.is_occupied_index(node_neighbor):
                updated_cost = visiting_cost[current_node] + 10000
            else:
                updated_cost = visiting_cost[current_node] + cost * math.sqrt((current_node[0]-node_neighbor[0])*(current_node[0]-node_neighbor[0]) +
                                                                              (current_node[1]-node_neighbor[1])*(current_node[1]-node_neighbor[1]) +
                                                                              (current_node[2]-node_neighbor[2])*(current_node[2]-node_neighbor[2]))

            if node_neighbor not in visiting_cost or updated_cost < visiting_cost[node_neighbor]:
                priority = updated_cost + heuristic_cost * math.sqrt((node_neighbor[0]-goal_index[0])*(node_neighbor[0]-goal_index[0]) +
                                                                     (node_neighbor[1]-goal_index[1])*(node_neighbor[1]-goal_index[1]) +
                                                                     (node_neighbor[2]-goal_index[2])*(node_neighbor[2]-goal_index[2]))
                heappush(queue, (priority, node_neighbor))
                visiting_cost[node_neighbor] = updated_cost
                visited[node_neighbor] = current_node

    nodes_expanded = len(visited)
    path = goal
    goal_path = goal_index

    if goal_index not in visited:
        path = None

    while goal_path != start_index:
        goal_path = visited[goal_path]
        print(f'---> {goal_path} ', end= '')
        nodes_expanded += 1
        goal_checking = occ_map.index_to_metric_center(goal_path)
        path = np.vstack([goal_checking, path])
    path = np.vstack([start, path])

    # Return a tuple (path, nodes_expanded)
    return path, nodes_expanded