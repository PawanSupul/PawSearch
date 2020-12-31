"""
This is a demo script to run as is to understand how the PawSearch library can be used to solve custom search problems.
Following areas are demonstrated in this demo script
    * Problem formulation
        - Node instantiation
        - Connecting nodes to create the graph/tree
        - Heuristic functions
    * Calling search functions
    * Obtaining resulting paths and traversals
"""

# Please refer the demo.png image. It is the graph used in this demo script

from PawSearch.base import *
from PawSearch.Uninformed import BreadthFirstSearch
from PawSearch.Informed import AStarSearch
from PawSearch.Local import LocalBeamSearch

# This library supports directional graphs with respective directional weights. When specifying the node, the names of
# the neighbor nodes and respective directional weights can be set.
# Eg: If A has 2 directional neighbors B and D with costs 5 and 23, while B has directional neighbors E, C and H (No A)
# with costs 4, 2 and 1, we can create this graph using following node specification
# nodeA = Node('A', {'B':5, 'D':23})
# nodeB = Node('B', {'C':2, 'E':4, 'H':1})
# Note that eventhough A=>B theres a connection, from B=>A theres no connection. This creates a directional graph.
#
# If a fully connected graph is needed, all the connections relevant to a node should be specified as neighbors.
# Eg: If A has 2 neighbors B and D with costs 5 and 23, while B has neighbors A and D with costs 5 and 10
# we can create this graph using following node specification
# NodeA = Node('A', {'B':5, 'D':23})
# NodeB = Node('B', {'A':5, 'D':10})

# Nodes specified according to the graph shown in demo.png
nodeA = Node('A', {'B':5, 'D':23})
nodeB = Node('B', {'C':2, 'E':4, 'H':1})
nodeC = Node('C', {'E':6, 'G':18})
nodeD = Node('D',{'E':3, 'F':2})
nodeE = Node('E', {'B':4, 'C':6, 'G':14, 'D':3})
nodeF = Node('F', {'G':5})
nodeG = Node('G', {'E':14})
nodeH = Node('H', {'B':1})

# create the graph by specifying the node names with respective node objects required for the problem. Unnecessary nodes
# can be ignored from the problem in this stage.
graph = {'A':nodeA, 'B':nodeB, 'C':nodeC, 'D':nodeD, 'E':nodeE, 'F':nodeF, 'G':nodeG, 'H':nodeH}

# heuristic value is needed for using informed and local searches. This is a function which indicates the direction to
# the goal using a cost-like parameter.
# If only uninformed searches are needed, this dictionary is not needed.
# Assuming G is the goal node, the heuristic values for all the other nodes are given in heuristic dictionary
heuristic = {'A':15, 'B':11, 'C':6, 'D':10, 'E':6, 'F':15, 'H':2, 'G':0}

# specify start and goal
start = 'A'
goal = 'G'

# Breadth First Search
bfs = BreadthFirstSearch(graph, start, goal)        # problem object
pathfound = bfs.Search()                            # Search for path. returns 1 if path is found
if(pathfound):
    print('\nResult for Breadth First Search algorithm')
    print(f'BFS: Path {bfs.getPath()}')
    print(f'BFS: Traversal {bfs.getTraversal()}')
else:
    print('Path not found')

# A Star Search
ass = AStarSearch(graph, start, goal, heuristic)    # problem object
pathfound = ass.Search()                            # Search for path. returns 1 if path is found
if(pathfound):
    print('\nResult for A Star Search algorithm')
    print(f'ASS: Path {ass.getPath()}')
    print(f'ASS: Traversal {ass.getTraversal()}')
else:
    print('Path not found')

# Local Beam Search
lbs = LocalBeamSearch(graph, start, goal, heuristic, 4) # 4 is the beam width. Please refer the function descriptions
pathfound = lbs.Search()                                  # Search for path. returns 1 if path is found
if(pathfound):
    print('\nResult for Local Beam Search algorithm')
    print(f'LBS: Path {lbs.getPath()}')
    print(f'LBS: Traversal {lbs.getTraversal()}')
else:
    print('Path not found')
