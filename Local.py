"""Local module: Local search algorithm definitions

This module contains local search algorithm definitions. In here, following local search algorithms are
defined as classes.
    * HillClimbing      : Hill climbing search algorithm with its helping functions
    * LocalBeamSearch   : Local beam search algorithm with its helping functions

If needed please refer the class and function help for these algorithms.
These classes are defined as separate search algorithms to be used on appropriate graphs. Since these are local
search algorithms, heuristic input is needed.
This module requires PawSearch.base module to be imported to the python environment

Author: Pawan Samaratunga
Created on Wed Dec 23 2020

"""

import random
import sys
from PawSearch.base import *

class HillClimbing(object):
    """Hill climbing search implementation

    This class is used to engulfs the algorithm and the supporting functions for hill climbing search. Within this
    implementation there are 3 different variations of hill climbing has made available.
        * simple            : First acceptable node is chosen
        * stochastic        : Random acceptable node is chosen
        * steepest_ascent   : Best acceptable node is chosen
    Following public functions are implemented under this class.
        * Search            : Execute breadth first search algorithm
        * getTraversal      : Get the node traversal for breadth first search for the given graph
        * getPath           : Get the path from start to goal
    Following private functions are implemented under this class.
        * ChoseRandomNode   : Chose a random node from a given node list (Future extension for weighted random choice)
    """

    def __init__(self, nodes, start, goal, heuristic, mode='Simple'): # can use mode = 'stochastic
        """Init function for HillClimbing class

        the mode variable contains the mode in which the hill climbing should be conducted. 'simple' mode keeps only the
        first acceptable node out of all the children nodes. 'stochastic' mode keeps only a randomly chosen acceptable
        node out of all the children nodes. 'steepest_ascent' mode keeps only the best of the acceptable node out of
        all the children nodes

        :param nodes: Node dictionary containing the nodes for the graph
        :param start: Starting node name
        :param goal: Goal node name
        :param heuristic: Dictionary containing heuristic values for each node
        :param mode: Hill climbing mode
        """
        self.nodes = nodes
        self.start = start
        self.goal = goal
        self.heuristic = heuristic
        if mode not in ['Simple', 'Stochastic', 'Steepest Ascent']:
            print('\033[91m' + 'The mode specified, is not a valid method for the implemented function' + '\033[0m')
            sys.exit()
        self.mode = mode

    def __ChoseRandomNode(self, nodelist):
        """Chose random node from a node list

        This function is used to obtain a random node from a given node list. Future implementation plan includes
        random choice with weighted probabilities according to the steepness of the changes in heuristic value.

        :param nodelist: List containing all the candidate nodes
        :return: Random node from the list
        """
        # ############################## use this only when weighted stochastic method is required
        # heuristics = [1/x[0] for x in nodelist]
        # weights = [round(x/min(heuristics)) for x in heuristics]
        # return random.choice(nodelist, weights, 1)
        # ##############################
        return random.choice(nodelist)

    def Search(self):
        """Hill climbing search execution function

        This function is used to execute the Hill climbing search algorithm on the given graph and calculate the path
        from start to goal, using the specified mode.

        :return: 1 or 0 indicating a path was found or not
        """
        ClearDiscover(self.nodes)
        sidewayiterator = 0
        close_q = queue()
        open_q = queue()     # should i make this a priority queue or keep as a list
        startheuristic = self.heuristic[self.start]
        coststart = [startheuristic, self.start]  # [15, a]
        open_q.push(coststart)
        while not open_q.is_empty():
            if self.mode == 'Simple':
                costnode = open_q.pull()
            elif self.mode == 'Stochastic':
                costnode = self.__ChoseRandomNode(open_q.result())
            elif self.mode == 'Steepest Ascent':
                open_q = priorityqueue(open_q.result())
                costnode = open_q.pull()

            close_q.push(costnode[::-1])
            open_q.clear()
            if (costnode[1] == self.goal):
                self.closedlist = close_q
                return 1

            for n in self.nodes[costnode[1]].getChild():
                nodeheuristic = self.heuristic[n]
                if(nodeheuristic < costnode[0]):
                    newnode = [nodeheuristic, n]
                    open_q.push(newnode)
                    sidewayiterator = 0
                elif(nodeheuristic == costnode[0] and sidewayiterator < 5): # no more than 5 iterations will be conducted on plateau
                    newnode = [nodeheuristic, n]
                    open_q.push(newnode)
                    sidewayiterator += 1
        return 0

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
        path = []
        closed = copy.deepcopy(self.closedlist.result())
        for costnode in closed:
            path.append(costnode[0])
        return path

class LocalBeamSearch(object):
    """Local beam search implementation

        This class is used to engulfs the algorithm and the supporting functions for Local beam search. In this
        implementation beamwidth variable is the parameter which determines how many nodes to be expanded and searched
        for the best nodes in each iteration. when beamwidth = 1, this is equivalent to hill climbing
        Following public functions are implemented under this class.
            * Search            : Execute breadth first search algorithm
            * getTraversal      : Get the node traversal for breadth first search for the given graph
            * getPath           : Get the path from start to goal
        Following private functions are implemented under this class.
            * __calculatePath : Calculate the path from start to goal
        """

    def __init__(self, nodes, start, goal, heuristic, beamwidth=1):
        """Init function for LocalBeamSearch class

        :param nodes: Node dictionary containing the nodes for the graph
        :param start: Starting node name
        :param goal: Goal node name
        :param heuristic: Dictionary containing heuristic values for each node
        :param beamwidth: number of best nodes to expand during each iteration
        """
        self.nodes = nodes
        self.start = start
        self.goal = goal
        self.heuristic = heuristic
        self.beamwidth = beamwidth
        if(beamwidth<1):
            print('\033[91m' + 'The specified beamwidth value is invalid. Value should be greater than or equal to 1' + '\033[0m')
            sys.exit()

    def Search(self):
        """Local beam search execution function

        This function is used to execute the Local beam search algorithm on the given graph and calculate the path
        from start to goal.

        :return: 1 or 0 indicating a path was found or not
        """
        ClearDiscover(self.nodes)
        sidewayiterator = 0
        step1 = []
        step2 = stack()
        open_q = priorityqueue()     # should i make this a priority queue or keep as a list
        startheuristic = self.heuristic[self.start]
        coststart = [startheuristic, self.start]  # [15, a]
        open_q.push(coststart)
        while not open_q.is_empty():    #[[3,d],[5,b]]
            # open_q = priorityqueue(open_q.result())
            if any(self.goal in x for x in open_q.result()):
                step2.push([self.goal, 0])
                self.step2 = step2
                self.path = LocalBeamSearch.__calculatePath(self)
                return 1

            for k_node in range(min(self.beamwidth, len(open_q.result()))):
                costnode = open_q.pull()
                step2.push(costnode[::-1])

                for n in self.nodes[costnode[1]].getChild():
                    if any(n in x for x in step1) or any(n in y for y in step2.result()):
                        continue
                    nodeheuristic = self.heuristic[n]
                    if(nodeheuristic < costnode[0]):
                        newnode = [nodeheuristic, n]
                        step1.append(newnode)
                        self.nodes[n].setParent(costnode[1])
                        sidewayiterator = 0
                    elif(nodeheuristic == costnode[0] and sidewayiterator < 5): # no more than 5 iterations will be conducted on plateau
                        newnode = [nodeheuristic, n]
                        step1.append(newnode)
                        self.nodes[n].setParent(costnode[1])
                        sidewayiterator += 1
            open_q = priorityqueue(step1)
            step1 = []
        return 0

    def __calculatePath(self):
        """Calculate the path

        This private function calculates the path from start node to goal node in local beam search.

        :return: Path node list
        """
        start = self.start
        nodes = self.nodes
        closed = copy.deepcopy(self.step2.result())
        goal_a = closed.pop()
        goal_name = goal_a[0]
        path = [goal_name]
        while(goal_name != start):
            parent =  nodes[goal_name].getParent()
            while goal_name != parent:
                newnode = closed.pop()
                goal_name = newnode[0]
            path.append(goal_name)
        return path[::-1]

    def getTraversal(self):
        """Traversal nodes

        This function returns the traversal nodes

        :return: traversal node list
        """
        # return self.step2.result()    # for normal use
        traversal = []                  # for GUI
        for i in self.step2.result():
            traversal.append(i[0])
        return traversal


    def getPath(self):
        """Path nodes

        This function returns the path nodes

        :return: Path node list
        """
        return self.path