"""Uninformed module: Uninformed search algorithm definitions

This module contains uninformed search algorithm definitions. In here, following uninformed search algorithms are
defined as classes.
    * BreadthFirstSearch        : Breadth first search algorithm with its helping functions
    * DepthFirstSearch          : Depth first search algorithm with its helping functions
    * DepthLimitedSearch        : Depth limited search algorithm with its helping functions
    * IterativeDeepeningSearch  : Iterative deepening search algorithm with its helping functions
    * UniformCostSearch         : Uniform cost search algorithm with its helping functions
    * Dijkstra                  : Dijkstra search algorithm with its helping functions
    * BiDirectionalSearch       : Bi-directional search algorithm with its helping functions

If needed please refer the class and function help for these algorithms.
These classes are defined as separate search algorithms to be used on appropriate graphs. Since these are uninformed
search algorithms, no heuristic input is needed.
This module requires PawSearch.base module to be imported to the python environment

Author: Pawan Samaratunga
Created on Wed Dec 23 2020

"""

import math
from PawSearch.base import *


class BreadthFirstSearch(object):
    """Breadth first search implementation

    This class is used to engulfs the algorithm and the supporting functions for Breadth first search.
    Following public functions are implemented under this class.
        * Search    : Execute breadth first search algorithm
        * getTraversal  : Get the node traversal for breadth first search for the given graph
        * getPath       : Get the path from start to goal
    Following private functions are implemented under this class.
        * __calculatePath : Calculate the path from start to goal
    """

    def __init__(self, nodes, start, goal):
        """Init function for BreadthFirstSearch class

        :param nodes: Node dictionary containing the nodes for the graph
        :param start: Starting node name
        :param goal: Goal node name
        """
        self.start = start
        self.goal = goal
        self.nodes = nodes

    def Search(self):
        """Breadth first search execution function

        This function is used to execute the breadth first search algorithm on the given graph and calculate the path
        from start to goal

        :return: 1 or 0 indicating a path was found or not
        """
        ClearDiscover(self.nodes)
        open_q = queue()
        close_q = queue()
        self.nodes[self.start].setDiscover()
        open_q.push(self.start)
        while not open_q.is_empty():
            v = open_q.pull()
            close_q.push(v)
            if (v == self.goal):
                self.closedlist = close_q
                self.openlist = open_q
                self.path = BreadthFirstSearch.__calculatePath(self)
                return 1
            for n in self.nodes[v].getChild():
                if (self.nodes[n].getDiscover() == 0):
                    open_q.push(n)
                    self.nodes[n].setDiscover()
                    self.nodes[n].setParent(v)
        return 0

    def __calculatePath(self):
        """Calculate the path

        This private function calculates the path from start node to goal node in breadth first search.

        :return: Path node list
        """
        path = []
        goal = self.goal
        open_q = goal
        path.append(open_q)
        while open_q != self.start:
            nodeparent = self.nodes[open_q].getParent()
            path.append(nodeparent)
            open_q = nodeparent
        return path[::-1]

    def getTraversal(self):
        """Traversal nodes

        This function returns the traversal nodes

        :return: traversal node list
        """
        return self.closedlist.result()

    def getPath(self):
        """Path nodes

        This function returns the path nodes

        :return: Path node list
        """
        return self.path

    # def getPath(self):
    #     start = self.start
    #     goal = self.goal
    #     nodes = self.nodes
    #
    #     closed = copy.deepcopy(self.closedlist.result())
    #     goal_a = closed.pop()
    #     assert goal == goal_a
    #     path = [goal_a]
    #     while goal_a != start:
    #         nextnode = closed.pop()
    #         if (goal_a in nodes[nextnode].getChild()):
    #             path.append(nextnode)
    #             goal_a = nextnode
    #     finalpath = path[::-1]
    #     return finalpath


class DepthFirstSearch(object):
    """Depth first search implementation

    This class is used to engulfs the algorithm and the supporting functions for Depth first search.
    Following public functions are implemented under this class.
        * Search        : Execute depth first search algorithm
        * getTraversal  : Get the node traversal for depth first search for the given graph
        * getPath       : Get the path from start to goal
    Following private functions are implemented under this class.
        * __calculatePath : Calculate the path from start to goal
    """

    def __init__(self,nodes, start, goal):
        """Init function for DepthFirstSearch class

        :param nodes: Node dictionary containing the nodes for the graph
        :param start: Starting node name
        :param goal: Goal node name
        """
        self.nodes = nodes
        self.start = start
        self.goal = goal

    def Search(self):
        """Depth first search execution function

        This function is used to execute the depth first search algorithm on the given graph and calculate the path
        from start to goal

        :return: 1 or 0 indicating a path was found or not
        """
        ClearDiscover(self.nodes)
        open_s = stack()
        close_s = stack()
        open_s.push(self.start)
        self.nodes[self.start].setDiscover(1)
        while not open_s.is_empty():
            # print(open_s.result())
            v = open_s.pull()
            close_s.push(v)
            # print(close_s.result())
            if (v == self.goal):
                self.closedlist = close_s
                self.openlist = open_s
                self.path = DepthFirstSearch.__calculatePath(self)
                return 1
            for n in self.nodes[v].getChild():
                if self.nodes[n].getDiscover() == 0:
                    self.nodes[n].setDiscover()
                    open_s.push(n)
                    self.nodes[n].setParent(v)
        return 0

    def __calculatePath(self):
        """Calculate the path

        This private function calculates the path from start node to goal node in Depth first search.

        :return: Path node list
        """
        path = []
        goal = self.goal
        open_q = goal
        path.append(open_q)
        while open_q != self.start:
            nodeparent = self.nodes[open_q].getParent()
            path.append(nodeparent)
            open_q = nodeparent
        return path[::-1]

    def getTraversal(self):
        """Traversal nodes

        This function returns the traversal nodes

        :return: traversal node list
        """
        return self.closedlist.result()

    def getPath(self):
        """Path nodes

        This function returns the path nodes

        :return: Path node list
        """
        return self.path

    # def getPath(self):
    #     start = self.start
    #     goal = self.goal
    #     nodes = self.nodes
    #
    #     closed = copy.deepcopy(self.closedlist.result())
    #     goal_a = closed.pop()
    #     assert goal == goal_a
    #     path = [goal_a]
    #     while goal_a != start:
    #         nextnode = closed.pop()
    #         if (goal_a in nodes[nextnode].getChild()):
    #             path.append(nextnode)
    #             goal_a = nextnode
    #     finalpath = path[::-1]
    #     return finalpath


class DepthLimitedSearch(object):
    """Depth limited search implementation

    This class is used to engulfs the algorithm and the supporting functions for depth limited search.
    Following public functions are implemented under this class.
        * Search    : Execute depth limited search algorithm
        * getTraversal  : Get the node traversal for depth limited search for the given graph
        * getPath       : Get the path from start to goal
    Following private functions are implemented under this class.
        * __calculatePath : Calculate the path from start to goal
    """

    def __init__(self, nodes, start, goal, depth=100):
        """Init function for DepthLimitedSearch class

        :param nodes: Node dictionary containing the nodes for the graph
        :param start: Starting node name
        :param goal: Goal node name
        :param depth: maximum depth until which the search is conducted (default = 100)
        """
        self.nodes = nodes
        self.start = start
        self.goal = goal
        self.depth = depth

    def Search(self):
        """Depth limited search execution function

        This function is used to execute the depth limited search algorithm on the given graph and calculate the path
        from start to goal

        :return: 1 or 0 indicating a path was found or not
        """
        ClearDiscover(self.nodes)
        open_s = stack()
        close_s = stack()
        open_s.push(self.start)
        self.nodes[self.start].setDepth(0)
        self.nodes[self.start].setDiscover(1)
        while not open_s.is_empty():
            v = open_s.pull()
            close_s.push(v)
            if (v == self.goal):
                self.closedlist = close_s
                self.openlist = open_s
                self.path = DepthLimitedSearch.__calculatePath(self)
                return 1
            if (self.nodes[v].getDepth() < self.depth):
                for n in self.nodes[v].getChild():
                    if self.nodes[n].getDiscover() == 0 or self.nodes[v].getDepth()+1 < self.nodes[n].getDepth():
                        self.nodes[n].setDiscover()
                        open_s.push(n)
                        self.nodes[n].setDepth(self.nodes[v].getDepth()+1)
                        self.nodes[n].setParent(v)
        return 0

    def __calculatePath(self):
        """Calculate the path

        This private function calculates the path from start node to goal node in Depth limited search.

        :return: Path node list
        """
        path = []
        goal = self.goal
        open_q = goal
        path.append(open_q)
        while open_q != self.start:
            nodeparent = self.nodes[open_q].getParent()
            path.append(nodeparent)
            open_q = nodeparent
        return path[::-1]

    def getTraversal(self):
        """Traversal nodes

        This function returns the traversal nodes

        :return: traversal node list
        """
        return self.closedlist.result()

    def getPath(self):
        """Path nodes

        This function returns the path nodes

        :return: Path node list
        """
        return self.path

    # def getPath(self):
    #     start = self.start
    #     goal = self.goal
    #     nodes = self.nodes
    #
    #     closed = copy.deepcopy(self.closedlist.result())
    #     goal_a = closed.pop()
    #     assert goal == goal_a
    #     path = [goal_a]
    #     while goal_a != start:
    #         nextnode = closed.pop()
    #         if (goal_a in nodes[nextnode].getChild()):
    #             path.append(nextnode)
    #             goal_a = nextnode
    #     finalpath = path[::-1]
    #     return finalpath


class IterativeDeepeningSearch(object):
    """Iterative deepening search implementation

    This class is used to engulfs the algorithm and the supporting functions for iterative deepening search.
    Following public functions are implemented under this class.
        * Search    : Execute iterative deepening search algorithm
        * getTraversal  : Get the node traversal for iterative deepening search for the given graph
        * getPath       : Get the path from start to goal
    This uses the depth limited search algorithm to conduct the search in depth first manner.
    """

    def __init__(self, nodes, start, goal, maxdepth = 100):
        """Init function for IterativeDeepeningSearch class

        :param nodes: Node dictionary containing the nodes for the graph
        :param start: Starting node name
        :param goal: Goal node name
        :param maxdepth: Maximum depth until which the iterative deepening is conducted (default = 100)
        """
        self.nodes = nodes
        self.start = start
        self.goal = goal
        self.maxdepth = maxdepth

    def Search(self):
        """iterative deepening search execution function

        This function is used to execute the iterative deepening search algorithm on the given graph and calculate the path
        from start to goal

        :return: 1 or 0 indicating a path was found or not
        """
        iterator = 0
        comp_dls = 0
        while comp_dls == 0:
            iterator += 1
            dls = DepthLimitedSearch(self.nodes, self.start, self.goal, iterator)
            comp_dls = dls.Search()
            if(iterator>= self.maxdepth):
                break

        if(comp_dls == 1):
            self.traversal = dls.getTraversal()
            self.path = dls.getPath()
            return 1
        else:
            return 0

    def getTraversal(self):
        """Traversal nodes

        This function returns the traversal nodes

        :return: traversal node list
        """
        return self.traversal

    def getPath(self):
        """Path nodes

        This function returns the path nodes

        :return: Path node list
        """
        return self.path


class UniformCostSearch(object):
    """Uniform cost search implementation

    This class is used to engulfs the algorithm and the supporting functions for Uniform cost search.
    Following public functions are implemented under this class.
        * Search    : Execute Uniform cost search algorithm
        * getTraversal  : Get the node traversal for Uniform cost search for the given graph
        * getPath       : Get the path from start to goal
    Following private functions are implemented under this class.
        * __calculatePath : Calculate the path from start to goal
    """

    def __init__(self, nodes, start, goal):
        """Init function for UniformCostSearch class

        :param nodes: Node dictionary containing the nodes for the graph
        :param start: Starting node name
        :param goal: Goal node name
        """
        self.nodes = nodes
        self.start = start
        self.goal = goal

    def Search(self):
        """Uniform cost search execution function

        This function is used to execute the Uniform cost search algorithm on the given graph and calculate the path
        from start to goal

        :return: 1 or 0 indicating a path was found or not
        """
        ClearDiscover(self.nodes)
        open_q = priorityqueue()
        close_q = stack()
        costname = [0, self.start]
        open_q.push(costname)
        while not open_q.is_empty():
            costnode = open_q.pull()
            close_q.push(costnode[::-1])
            if(costnode[1] == self.goal):
                self.closedlist = close_q
                self.openlist = open_q
                self.path = UniformCostSearch.__calculatePath(self)
                return 1
            for n in self.nodes[costnode[1]].getNeighborWithCost():
                newnode = [costnode[0] + n[0] , n[1]]
                if not any(newnode[1] in x for x in close_q.result()):
                    open_q.push(newnode)
                    self.nodes[newnode[1]].setParent(costnode[1])
        return 0

    def __calculatePath(self):
        """Calculate the path

        This private function calculates the path from start node to goal node in Uniform cost search.

        :return: Path node list
        """
        start = self.start
        goal = self.goal
        nodes = self.nodes # allnodes = {'a': a, 'b': b}

        closed = copy.deepcopy(self.closedlist.result())
        goal_a = closed.pop() # [g, 8]
        goal_name = goal_a[0]
        goal_cost = goal_a[1]
        assert goal == goal_name
        path = [goal_name]
        while goal_name != start:
            nextnode = closed.pop() # [c, 6]
            next_name = nextnode[0]
            next_cost = nextnode[1]
            neighborweights = nodes[next_name].getNeighborWithCost()    # [[e,6], [g,8]]
            for edge in neighborweights:
                if(goal_name in edge) and (goal_cost-next_cost == edge[0]):
                    path.append(next_name)
                    goal_name = next_name
                    goal_cost = next_cost
        return path[::-1]

    def getTraversal(self):
        """Traversal nodes

        This function returns the traversal nodes

        :return: traversal node list
        """
        # return self.closedlist.result()
        traversal = []
        for i in self.closedlist.result():
            traversal.append(i[0])
        return traversal

    def getPath(self):
        """Path nodes

        This function returns the path nodes

        :return: Path node list
        """
        return self.path

    # def calculatePath(self):
    #     path = []
    #     goal = self.goal
    #     open_q = goal
    #     path.append(open_q)
    #     while open_q != self.start:
    #         nodeparent = self.nodes[open_q].getParent()
    #         path.append(nodeparent)
    #         open_q = nodeparent
    #     return path[::-1]


class Dijkstra(object):
    """Dijkstra algorithm implementation

    This class is used to engulfs the algorithm and the supporting functions for Dijkstra algorithm.
    Following public functions are implemented under this class.
        * Search    : Execute Dijkstra algorithm
        * getTraversal  : Get the node traversal for Dijkstra algorithm for the given graph
        * getPath       : Get the path from start to goal
    Following private functions are implemented under this class.
        * __calculatePath : Calculate the path from start to goal
    """

    def __init__(self, nodes, start, goal):
        """Init function for Dijkstra class

        :param nodes: Node dictionary containing the nodes for the graph
        :param start: Starting node name
        :param goal: Goal node name
        """
        self.nodes = nodes
        self.start = start
        self.goal = goal

    def Search(self):
        """Dijkstra algorithm execution function

        This function is used to execute the Dijkstra algorithm on the given graph and calculate the path
        from start to goal

        :return: 1 or 0 indicating a path was found or not
        """
        ClearDiscover(self.nodes)
        distance = {node: math.inf for node in self.nodes}
        distance[self.start] = 0
        costnode = [0, self.start]
        open_q = priorityqueue([costnode])
        while not open_q.is_empty():
            newnode = open_q.pull()
            if(newnode[1] == self.goal):
                self.path = Dijkstra.__calculatePath(self)
                self.distance = distance
                return 1
            if(newnode[0] > distance[newnode[1]]):
                continue
            for n in self.nodes[newnode[1]].getNeighborWithCost():
                newdist = newnode[0] + n[0]
                if(newdist < distance[n[1]]):
                    distance[n[1]] = newdist
                    open_q.push([newdist, n[1]])
                    self.nodes[n[1]].setParent(newnode[1])
        return 0

    def __calculatePath(self, goal = -1):
        """Calculate the path

        This private function calculates the path from start node to goal node in Dijkstra search.

        :return: Path node list
        """
        path = []
        if(goal == -1):
            goal = self.goal
        open_q = goal
        path.append(open_q)
        while open_q != self.start:
            nodeparent = self.nodes[open_q].getParent()
            path.append(nodeparent)
            open_q = nodeparent
        return path[::-1]

    def getTraversal(self):
        """Traversal nodes

        This function returns the traversal nodes

        :return: traversal node list
        """
        traversed = [x for x in self.distance.keys() if (self.distance[x] != math.inf)]
        return traversed

    def getPath(self):
        """Path nodes

        This function returns the path nodes

        :return: Path node list
        """
        return self.path


class BiDirectionalSearch(object):
    """Bi-directional search implementation

    This class is used to engulfs the algorithm and the supporting functions for Bi-directional search.
    Following public functions are implemented under this class.
        * Search    : Execute Bi-directional search algorithm
        * getTraversal  : Get the node traversal for Bi-directional search for the given graph
        * getPath       : Get the path from start to goal
    Following private functions are implemented under this class.
        * __calculatePath : Calculate the path from start to goal
    """

    def __init__(self, nodes, start, goal):
        """Init function for BiDirectionalSearch class

        :param nodes: Node dictionary containing the nodes for the graph
        :param start: Starting node name
        :param goal: Goal node name
        """
        self.nodes = nodes
        self.start = start
        self.goal = goal
        self.intersect = None

    def Search(self):
        """Bi-directional search execution function

        This function is used to execute the Bi-directional search algorithm on the given graph and calculate the path
        from start to goal

        :return: 1 or 0 indicating a path was found or not
        """
        ClearDiscover(self.nodes)
        start = self.start
        goal = self.goal
        nodes = self.nodes
        dir1_open = queue(start)
        dir1_explore = []
        dir1_parents = {}
        dir2_open = queue(goal)
        dir2_explore = []
        dir2_parents = {}
        dir1_close, dir2_close = [], []
        dir1_explore_total, dir2_explore_total = [start], [goal]
        maxiteration = len(nodes)
        ite = 0
        terminate = 0 # 1 = goal found in direction_1, 2 = start found in direction_2
        while ite < maxiteration:
            while not dir1_open.is_empty():
                dir1_node = dir1_open.pull()
                dir1_close.append(dir1_node)
                for n in nodes[dir1_node].getChild():
                    if((n not in dir1_explore_total) and (n not in dir1_explore) and(n not in dir1_close)):
                        dir1_explore.append(n)
                        dir1_parents[n] = dir1_node
                    if(n == goal):
                        terminate = 1   # goal found in direction 1
                        # check if n is goal

            while not dir2_open.is_empty():
                dir2_node = dir2_open.pull()
                dir2_close.append(dir2_node)
                for n in nodes[dir2_node].getChild():
                    if ((n not in dir2_explore_total) and (n not in dir2_explore) and(n not in dir2_close)):
                        dir2_explore.append(n)
                        dir2_parents[n] = dir2_node
                    if(n == start):
                        terminate = 2   # start found in direction 2
                        # check if n is start

            dir1_explore_total.extend(dir1_explore)
            dir2_explore_total.extend(dir2_explore)

            intersectidx = [x in dir2_explore_total for x in dir1_explore_total]
            if any(intersectidx) or terminate > 0:
                if any(intersectidx):
                    self.intersect = dir1_explore_total[intersectidx.index(True)]
                elif(terminate == 1):
                    self.intersect = goal
                elif(terminate == 2):
                    self.intersect = start

                self.dir1_parents = dir1_parents
                self.dir2_parents = dir2_parents
                self.dir2_close = dir2_close
                self.dir1_close = dir1_close
                self.path = BiDirectionalSearch.__calculatePath(self)
                return 1

            dir1_open = queue(dir1_explore)
            dir2_open = queue(dir2_explore)
            dir1_explore, dir2_explore = [], []

            ite += 1
        return 0

    def __calculatePath(self):
        """Calculate the path

        This private function calculates the path from start node to goal node in Bi-directional search.

        :return: Path node list
        """
        dir1_parents = self.dir1_parents
        dir2_parents = self.dir2_parents
        intersect = self.intersect
        if(intersect == self.goal):
            goal = self.goal
            start = self.start
            pathdir1 = [goal]
            while goal != start:
                parent = dir1_parents[goal]
                pathdir1.append(parent)
                goal = parent

            finalpath = pathdir1[::-1]
            return finalpath

        elif(intersect == self.start):
            goal = self.goal
            start = self.start
            pathdir2 = [start]
            while start != goal:
                parent = dir2_parents[start]
                pathdir2.append(parent)
                start = parent

            finalpath = pathdir2
            return finalpath

        elif(intersect is None):
            print('\033[91m' + 'Bidirectional search was unable to converge to a solution' + '\033[0m')
            return []

        else:
            finalpath = []
            newnode1 = copy.deepcopy(intersect)
            pathdir1 = []
            while(newnode1 != self.start):
                parent = dir1_parents[newnode1]
                pathdir1.append(parent)
                newnode1 = parent

            newnode2 = copy.deepcopy(intersect)
            pathdir2 = []
            while(newnode2 != self.goal):
                parent = dir2_parents[newnode2]
                pathdir2.append(parent)
                newnode2 = parent

            finalpath.extend(pathdir1[::-1])
            finalpath.extend([intersect])
            finalpath.extend(pathdir2)
            return finalpath

    def getTraversal(self):
        """Traversal nodes

        This function returns the traversal nodes

        :return: traversal node list
        """
        dir1_close = self.dir1_close
        dir2_close = self.dir2_close
        # return [dir1_close, dir2_close]       # for normal use
        return dir1_close + dir2_close          # for GUI

    def getPath(self):
        """Path nodes

        This function returns the path nodes

        :return: Path node list
        """
        return self.path