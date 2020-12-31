"""Informed module: Informed search algorithm definitions

This module contains informed search algorithm definitions. In here, following informed search algorithms are
defined as classes.
    * GreedySearch          : Greedy best first search algorithm with its helping functions
    * AStarSearch           : A star search algorithm with its helping functions

If needed please refer the class and function help for these algorithms.
These classes are defined as separate search algorithms to be used on appropriate graphs. Since these are informed
search algorithms, heuristic input is needed.
This module requires PawSearch.base module to be imported to the python environment

Author: Pawan Samaratunga
Created on Wed Dec 23 2020

"""

from PawSearch.base import *

class GreedySearch(object):
    """Greedy best first search implementation

    This class is used to engulfs the algorithm and the supporting functions for Greedy best first search.
    Following public functions are implemented under this class.
        * Search    : Execute Greedy best first search algorithm
        * getTraversal  : Get the node traversal for Greedy best first search for the given graph
        * getPath       : Get the path from start to goal
    """

    def __init__(self, nodes, start, goal, heuristic):
        """Init function for GreedySearch class

        :param nodes: Node dictionary containing the nodes for the graph
        :param start: Starting node name
        :param goal: Goal node name
        :param heuristic: Dictionary containing heuristic values for each node
        """
        self.nodes = nodes
        self.start = start
        self.goal = goal
        self.heuristic = heuristic

    def Search(self):
        """Greedy best first search execution function

        This function is used to execute the Greedy best first search algorithm on the given graph and calculate the path
        from start to goal

        :return: 1 or 0 indicating a path was found or not
        """
        ClearDiscover(self.nodes)
        open_s = priorityqueue()
        close_s = stack()
        open_s.push([self.heuristic[self.start], self.start, ''])
        while not open_s.is_empty():
            nodecost, nodename, nodeparent = open_s.pull()
            close_s.push([nodename, nodeparent])
            if (nodename == self.goal):
                self.closedlist = close_s
                self.openlist = open_s
                self.path = GreedySearch.__calculatePath(self)
                return 1
            for n in self.nodes[nodename].getChild():
                if not any(n in x[0] for x in close_s.result()):
                    open_s.push([self.heuristic[n], n, nodename])
        return 0
        ##################### This is similar to hill climb. Incorrect implementation of greedy. version 1
        # ClearDiscover(self.nodes)
        # open_q = priorityqueue()
        # close_q = stack()
        # self.nodes[self.start].setDiscover()
        # startheuristic = self.heuristic[self.start] # [15, a]
        # coststart = [startheuristic, self.start]
        # open_q.push(coststart)
        # while not open_q.is_empty():
        #     costnode = open_q.pull()
        #     close_q.push(costnode[::-1])
        #     open_q.clear()
        #     if (costnode[1] == self.goal):
        #         self.closedlist = close_q
        #         # self.openlist = open_q
        #         return 1
        #     for n in self.nodes[costnode[1]].getChild():
        #         if (self.nodes[n].getDiscover() == 0):
        #             self.nodes[n].setDiscover()
        #             nodeheuristic = self.heuristic[n]
        #             newnode = [nodeheuristic, n]
        #             open_q.push(newnode)
        # return 0

    def __calculatePath(self):
        """Calculate the path

        This private function calculates the path from start node to goal node in Greedy search.

        :return: Path node list
        """
        path = []
        start = self.start
        closedlist = copy.deepcopy(self.closedlist)
        node, nodeparent = closedlist.pull()
        path.append(node)
        while(node != start):
            newnode, newnodeparent = closedlist.pull()
            if(newnode == nodeparent):
                path.append(newnode)
                node = newnode
                nodeparent = newnodeparent
        return path[::-1]


    def getTraversal(self):
        """Traversal nodes

        This function returns the traversal nodes

        :return: traversal node list
        """
        # return self.closedlist.result()       # for normal use
        traversal = []                          # for GUI
        for i in self.closedlist.result():
            traversal.append(i[0])
        return traversal

    def getPath(self):
        """Path nodes

        This function returns the path nodes

        :return: Path node list
        """
        return self.path
        # path = []
        # closed = copy.deepcopy(self.closedlist.result())
        # for costnode in closed:
        #     path.append(costnode[0])
        # return path


class AStarSearch(object):
    """A* search implementation

    This class is used to engulfs the algorithm and the supporting functions for A* search.
    Following public functions are implemented under this class.
        * Search    : Execute A* search algorithm
        * getTraversal  : Get the node traversal for A* search for the given graph
        * getPath       : Get the path from start to goal
    Following private functions are implemented under this class.
        * __calculatePath : Calculate the path from start to goal
    """

    def __init__(self, nodes, start, goal, heuristic):
        """Init function for AStarSearch class

        :param nodes: Node dictionary containing the nodes for the graph
        :param start: Starting node name
        :param goal: Goal node name
        :param heuristic: Dictionary containing heuristic values for each node
        """
        self.nodes = nodes
        self.start = start
        self.goal = goal
        self.heuristic = heuristic

    def Search(self):
        """A* search execution function

        This function is used to execute the A* search algorithm on the given graph and calculate the path
        from start to goal

        :return: 1 or 0 indicating a path was found or not
        """
        ClearDiscover(self.nodes)
        open_q = priorityqueue()
        close_q = stack()
        heuristicstart = self.heuristic[self.start]
        # heuristicstart = 0 # try uncommenting above line
        coststart = [heuristicstart, 0, self.start] # heuristic cost, path cost, node name
        open_q.push(coststart)
        while not open_q.is_empty():
            costnode = open_q.pull()
            close_q.push(costnode[::-1])
            if (costnode[2] == self.goal):
                self.closedlist = close_q
                self.openlist = open_q
                self.path = AStarSearch.__calculatePath(self)
                return 1
            for n in self.nodes[costnode[2]].getNeighborWithCost(): # a.neighborswithweights => [[3,d], [5,b]]
                nodeheuristic = self.heuristic[n[1]]
                newnode = [costnode[1] + n[0] + nodeheuristic, costnode[1] + n[0] ,n[1]]
                if not any(newnode[2] in x for x in close_q.result()):
                    open_q.push(newnode)
        return 0

    def __calculatePath(self):
        """Calculate the path

        This private function calculates the path from start node to goal node in A* search.

        :return: Path node list
        """
        start = self.start
        goal = self.goal
        nodes = self.nodes
        # heuristic = self.heuristic
        closed = copy.deepcopy(self.closedlist.result())

        goal_a = closed.pop()  # [g, 8]
        goal_name = goal_a[0]
        goal_cost = goal_a[1]
        assert goal == goal_name
        path = [goal_name]
        while goal_name != start:
            nextnode = closed.pop()  # [c, 6]
            next_name = nextnode[0]
            next_cost = nextnode[1]
            neighborweights = nodes[next_name].getNeighborWithCost()  # [[6,e], [8,g]]
            for edge in neighborweights:
                # nextheuristic = self.heuristic[next_name]
                if (goal_name in edge) and (goal_cost == next_cost + edge[0]):
                    path.append(next_name)
                    goal_name = next_name
                    goal_cost = next_cost
        return path[::-1]

    def getTraversal(self):
        """Traversal nodes

        This function returns the traversal nodes

        :return: traversal node list
        """
        # return self.closedlist.result()       # for normal use
        traversal = []                          # for GUI
        for i in self.closedlist.result():
            traversal.append(i[0])
        return traversal

    def getPath(self):
        """Path nodes

        This function returns the path nodes

        :return: Path node list
        """
        return self.path