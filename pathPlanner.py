# Import any libraries required
import random


# The main path planning function. Additional functions, classes, 
# variables, libraries, etc. can be added to the file, but this
# function must always be defined with these arguments and must 
# return an array ('list') of coordinates (col,row).
#DO NOT EDIT THIS FUNCTION DECLARATION
def do_a_star(grid, start, end, display_message):
    #EDIT ANYTHING BELOW HERE

    # AERO60492 Coursework 2 - A* Path Planning
    #
    # A* was created by Hart et al (1968) as part of the Shakey
    # robotics project at Stanford. It is a variant of Dijkstra's
    # algorithm which is goal focused. Dijkstra finds the shortest path by expanding
    # nodes in all directions, which is inefficient.
    # A* improves on this by introducing a heuristic h(n) which estimates the
    # remaining distance to the goal and directs the search towards it.
    #
    # The cost function per node is
    #   f(n) = g(n) + h(n)
    #   g(n) = actual cost of the path from start to n
    #   h(n) = estimated cost from n to goal using Euclidean distance
    #
    # Euclidean distance is used as the heuristic.

    # Get the size of the grid
    COL = len(grid)
    ROW = len(grid[0])
 
    display_message("Running A* from " + str(start) + " to " + str(end)) # Initial message
 
    # Heuristic function h(n). Euclidean distance to goal
    def heuristic(col, row):
        d_col = col - end[0]    # col_goal - col_current
        d_row = row - end[1]    # row_goal - row_current
        return (d_col * d_col + d_row * d_row) ** 0.5 # h(n) = sqrt(d_col^2 + d_row^2)
 
    # Four data structures for A*
    open_list   = {}    # Frontier. Nodes found but not yet fully explored
    closed_list = set() # Interior. Nodes whose optimal cost is confirmed
    g_cost     = {}    # Best path cost from start to current node
    came_from   = {}    # Parent list. Chains back from goal to start at end
 
    # Initialise with start node where g(start)=0 and f(start)=0+h(start)
    g_cost[start] = 0
    open_list[start] = heuristic(start[0], start[1])
 
    # Movement in 4 directions where each step costs 1
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]
 
    # Main loop. Lowest cost nodes are expanded until goal is reached
    while open_list:
 
        # Select node with lowest f(n) = g(n) + h(n) from the frontier
        current = min(open_list, key=open_list.get)
 
        # Goal reached. Trace back through parent nodes to create the path
        if current == end:
            display_message("The destination cell is found")
            path = []
            node = current
            # Loop through each node in parent list to go from goal to start
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()  # Reverse to correct path order
            display_message("Path length: " + str(len(path)) + " cells")
 
            # Send path to GUI for visualisation
            return path
 
        # Move current node from frontier to interior
        del open_list[current]
        closed_list.add(current)
 
        cur_col, cur_row = current
 
        # Check all 4 neighbours
        for d_col, d_row in directions:
            nb_col = cur_col + d_col
            nb_row = cur_row + d_row
            neighbour = (nb_col, nb_row)
 
            # Skip if outside grid bounds
            if nb_col < 0 or nb_col >= COL:
                continue
            if nb_row < 0 or nb_row >= ROW:
                continue
 
            # Skip if 0 as obstacle in grid
            if grid[nb_col][nb_row] == 0:
                continue
 
            # Skip closed nodes. g cost already optimal and can't be improved by other routes
            if neighbour in closed_list:
                continue
 
            # Cost to reach neighbour via current node with each step being 1
            temp_g = g_cost[current] + 1
 
            # Update if new node or a cheaper route has been found
            if neighbour not in g_cost or temp_g < g_cost[neighbour]:
                came_from[neighbour] = current
                g_cost[neighbour]   = temp_g
                # f(n) = g(n) + h(n)
                open_list[neighbour] = temp_g + heuristic(nb_col, nb_row)   
 
    # No path found after going through open set
    display_message("No path found")
    return []

#end of file