#!/usr/bin/env python

"""
A-star path planner implementation
"""

import math
from queue import PriorityQueue


def a_star(init_coord, goal, map_cells):
    """
    A* path search algorithm implementation.
    Finds a path from start to goal.
    """

    unvisited = PriorityQueue()  # Unvisited nodes priority queue
    unvisited.put((0, init_coord))
    parent = {}  # Parent nodes dictionary
    cost = {}  # Cost from nodes to goal dictionary
    path = []  # To hold the path nodes
    parent[init_coord] = None  # Initial node has no parent
    cost[init_coord] = 0

    while not unvisited.empty():  # While we still have unvisited nodes
        curr = unvisited.get()[1]  # Get the coordinates of the node with highest priority (lowest cost)

        # If the goal node is reached then insert all the path nodes (backtrack to initial node)
        if curr == goal:
            while curr != init_coord:
                path.insert(0, (curr[0], curr[1]))
                curr = parent[curr]

            return path

        for nbr, diag in get_nbrs(curr, parent[curr], map_cells):  # For every neighbour cell of the current cell node
            mv_cost = 1

            # If considering diagonal map cell the adjust movement cost
            if diag:
                mv_cost = math.sqrt(2)  # Diagonal cost is around 1.41

            nbr_g = cost[curr] + mv_cost  # Distance b/w current node (neighbour of current) and start node

            # If the neighbour cell has not been visited yet or
            # its distance to the start node has a smaller value
            # than previously explored then update the 'cost' dictionary
            # and recalculate totalcost
            if nbr not in cost or nbr_g < cost[nbr]:
                cost[nbr] = nbr_g
                total_cost = nbr_g + calc_heuristic(nbr, goal)
                parent[nbr] = curr
                unvisited.put((total_cost, nbr))

    # Return 'False' if no safe path exists to the goal
    return False


# Salesman
def adapted_tsp(init_coord, g_queue, route=None):
    """
    Adapted Travelling Salesman Problem implementation.
    Given a set of goal points and the initial coordinates of the robot
    find the shortest possible route that visits every goal exactly once.
    """

    if route is None:
        route = []
    goal_priorities = PriorityQueue()  # To hold currently available goals starting from the closest

    if g_queue.qsize() > 1:

        # Prioritize goals by their distance to the current
        # location of the robot (starting from the closest)
        for n in range(g_queue.qsize()):
            curr = g_queue.get()[1]
            heur = calc_heuristic(curr, init_coord)
            goal_priorities.put((heur, curr))

        # Get the closest goal, add it to the route and repeat the process
        closest_g = goal_priorities.get()
        route.append(closest_g[1])
        return adapted_tsp(closest_g[1], goal_priorities, route)

    else:
        route.append(g_queue.get()[1])

        return route


def get_nbrs(curr, parent, map_cells):
    """
    Return safe path points list where the robot can move to
    """

    path_pts = []  # To hold all the free neighbour points
    curr_x = round(curr[0], 2)
    curr_y = round(curr[1], 2)
    map_res = .02  # Map resolution

    # Checks all 8 surrounding neighbour points of the current x,y coordinate point
    # True stands for a possible diagonal movement and vice-versa
    nbr_pts = [[-map_res, 0, False],
               [map_res, 0, False],
               [ 0, -map_res, False],
                [	0,	map_res, False],
                [ map_res, 	map_res,  True],
                [ map_res, -map_res,  True],
                [-map_res, -map_res,  True],
                [-map_res,  map_res,  True]]

    # For every neighbour point check if the robot will be able to rotate freely
    # at the current location
    for nbr in nbr_pts:
        path_pts = is_safe_loc(curr_x + nbr[0], curr_y + nbr[1], parent, map_cells, path_pts, nbr[2])

    return path_pts


def is_safe_loc(curr_x, curr_y, parent, map_cells, nbrs, d=False, safe_dist=.12):
    """
    Check if neighbour location will allow for the movement
    and rotation of the robot without crashing
    """

    if is_free((curr_x, curr_y), map_cells) and (curr_x, curr_y) != parent:
        x = (round(curr_x ,2))
        y = (round(curr_y ,2))

        # If the neighbour location has a free circle around it
        # then append it as a free location
        # + X is map X
        nbr_pts = [	[-safe_dist - 0.07, 	  0],
                    [ safe_dist + 0.07, 	  0],
                    [ 	  0, -safe_dist - 0.07],
                    [	  0,  safe_dist + 0.07],
                    [ safe_dist,  safe_dist],
                    [ safe_dist, -safe_dist],
                    [-safe_dist, -safe_dist],
                    [-safe_dist,  safe_dist] ]

        all_free = []
        for nbr in nbr_pts:
                all_free.append(is_free((x + nbr[0], y + nbr[1]), map_cells))

        if False not in all_free:
            nbrs.append([(x, y), d])
        # Little hack because the small door on top of
        # the map would not allow for greater 'safe_dist'
        # It should actually be at least .145 for the
        # safe rotation of the robot
        elif all_free[2] == all_free[3] == False:
            nbrs.append([(x, y + 0.01), d])

    return nbrs


def calc_heuristic(curr, g):
    """
    Calculate the heuristic function to estimate the cost of the cheapest path points
    """

    # Current and goal node x,y coordinates on the map
    curr_x = round(curr[0], 1)
    curr_y = round(curr[1], 1)
    goal_x = g[0]
    goal_y = g[1]

    # Calculate the Manhattan distance heuristic
    heur = abs(curr_x - goal_x) + abs(curr_y - goal_y)

    # Return the heuristic
    return	heur


def is_free(curr, map_cells):
    """
    Check if a particular x,y coordinate on the map is free from obstacles
    """

    # Get the square grid of the map
    grid = map_cells.info

    # Retrieve the cell index of the current node on the map
    col = int(round(curr[0] / grid.resolution + .5*grid.width))
    row = int(round(curr[1] / grid.resolution + .5*grid.height))
    cell = col + row*grid.width  # Current coordinate cell index of the actual map

    # Return True if map pixel (cell) is free from obstacles
    # Every cell that is not free is marked with '1' and '0' otherwise
    if map_cells.data[cell] == 0:
        return True
    else:
        return False
