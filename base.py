"""Base module: Basic object definitions

This module contains basic object definitions required to run the PawSearch search algorithm library. In here, following
basic building blocks are defined.
    * Node          : Defines the node of a graph
    * queue         : Defines queue data structure, which is a FIFO list
    * stack         : Defines stack data structure, which is a LIFO list
    * priorityqueue : Defines priority queue data structure implemented using a heap
These objects are used to define the weights, parents, status of the nodes in a graph, along with data structures for
variable handling.
This module requires "heapq" library to be installed within the python environment.

Author: Pawan Samaratunga
Created on Wed Dec 23 2020

"""

import copy
from heapq import heappop, heappush, heapify

class Node(object):
    """Node object

        This class is used to engulfs the initiation and supporting functions for Node object. Node is defined as the
        basic unit of graph in this library. A Node can have following characteristics.
            * Name      : name of the node
            * Neighbors : neighbors of the node
            * Parent    : Special type of neighbor, the parent node name of the node under consideration.
            * Discover  : Status which memorize whether the node is discovered
            * Depth     : Status which memorize the depth of the node
        Following public functions are implemented under this class.
            * getChild              : Get all the neighbor nodes
            * getNeighborWithCost   : Get all the neighbor nodes with the respective edge costs
            * getNumChild           : Get the number of neighbors for the node
            * setParent             : Set the parent node for the node
            * getParent             : Get the parent node for the node
            * setDiscover           : Set the discover status for the node
            * getDiscover           : Get the discover status for the node
            * setDepth              : Set the depth of the node
            * getDepth              : Get the depth of the node
        when instantiating a node, it can be done by giving a name and the neighboring nodes with respective edge costs
        using a dictonary. If the graph is unweighted, the node can be instantiated with the node name and set
        containing neighboring nodes.
        eg: neighbors as a dictionary - when edge weights are present
            node1 = Node('node_name', {'node_name_x': weight_x, 'node_name_y':weight_y, 'node_name_z':weight_z})
        eg: neighbors as a set - when edge weights are NOT present
            node1 = Node('node_name', {'node_name_x', 'node_name_y', 'node_name_z'})
        """

    def __init__(self, name, node):
        """Init function for Node class

        :param name: name of the node - string
        :param node: neighbors of the node - as a dictionary or a set
        """
        self.name = name
        self.node = node
        self.discover = 0

    def getChild(self):
        """Get neighbor nodes

        :return: neighbor nodes
        """
        if(type(self.node) is set):
            return list(self.node)
        else:
            return list(self.node.keys())

    def getNeighborWithCost(self):
        """Get neighbor nodes with cost values

        :return: Neighbor nodes with costs as a nested list
        """
        if (type(self.node) is set):
            namelist = list(self.node)
            finallist = [[1,x] for x in namelist]
            return finallist
        else:
            keylist = list(self.node.keys())
            finallist = [[self.node[x], x] for x in keylist]
            return finallist

    def getNumChild(self):
        """Number of neighbors

        :return: Number of neighbor nodes
        """
        return len(self.node)

    def setParent(self, parent):
        """Set the name of parent node

        :param parent: name of the parent node
        :return: None
        """
        self.parent = parent

    def getParent(self):
        """Get the name of parent node

        :return: name of the parent node
        """
        return self.parent

    def setDiscover(self, value = 1):
        """Set the discover value for the node

        :param value: value for the discover parameter (default = 1)
        :return: None
        """
        self.discover = value

    def getDiscover(self):
        """Get the discover value for the node

        :return: discover value for the node
        """
        return self.discover

    def setDepth(self, value=0):
        """Set the depth value for the node

        :param value: value for the depth parameter (default = 0)
        :return: None
        """
        self.depth = value

    def getDepth(self):
        """Get the depth value for the node

        :return: Depth value for the node
        """
        return self.depth

    def getWeight(self, child):
        return self.node[child]


class queue(object):
    """Queue object

    This class is used to engulfs the initiation and supporting functions for queue object. Queue is defined as FIFO
    data structure. This is implemented using a list.
    Following public functions are implemented under this class.
        * push      : Insert element to the queue
        * pull      : Retrieve an element from the queue
        * is_empty  : check if the queue is empty
        * clear     : clear the queue and make it empty
        * result    : Get the elements in the queue as a list
    when instantiating a queue, it can be done by giving zero parameters, or giving a single element or a list.
    eg: No elements. Instantiation only
        queue_1 = queue()
    eg: Instantiation with a single element
        queue_1 = queue(element_1)
    eg: Instantiation with a list
        queue_1 = queue(list_1)
    """

    def __init__(self, quelist = None):
        """Init function for queue class

        :param quelist: empty element, single data element or a list to convert into a queue structure (default = None)
        """
        if (quelist == None):
            self.quelist = []
        elif type(quelist) == list:
            self.quelist = copy.deepcopy(quelist)
        else:
            self.quelist = [quelist]

    def push(self, quelist):
        """Insert an element to the queue

        :param quelist: element to be inserted
        :return: None
        """
        self.quelist.append(quelist)

    def pull(self):
        """Retrieve an element from the queue

        :return: Retrieved element
        """
        return self.quelist.pop(0)

    def is_empty(self):
        """Check if the queue is empty

        :return: 0 or 1 indicating the queue is empty or not
        """
        if(len(self.quelist)==0):
            return 1
        else:
            return 0

    def clear(self):
        """Clear the queue and make it empty

        :return: None
        """
        self.quelist = []

    def result(self):
        """Get the elements in queue as a list

        :return: list of elements
        """
        return self.quelist


class stack(object):
    """Stack object

    This class is used to engulfs the initiation and supporting functions for stack object. Stack is defined as LIFO
    data structure. This is implemented using a list.
    Following public functions are implemented under this class.
        * push      : Insert element to the stack
        * pull      : Retrieve an element from the stack
        * is_empty  : check if the stack is empty
        * clear     : clear the stack and make it empty
        * result    : Get the elements in the stack as a list
    when instantiating a stack, it can be done by giving zero parameters, or giving a single element or a list.
    eg: No elements. Instantiation only
        stack_1 = stack()
    eg: Instantiation with a single element
        stack_1 = stack(element_1)
    eg: Instantiation with a list
        stack_1 = stack(list_1)
    """

    def __init__(self, element = None):
        """Init function for stack class

        :param element: empty element, single data element or a list to convert into a stack structure (default = None)
        """
        if (element == None):
            self.element = []
        elif type(element) == list:
            self.element = copy.deepcopy(element)
        else:
            self.element = [element]

    def push(self, element):
        """Insert an element to the stack

        :param element: element to be inserted
        :return: None
        """
        self.element.append(element)

    def pull(self):
        """Retrieve an element from the stack

        :return: Retrieved element
        """
        return self.element.pop()

    def is_empty(self):
        """Check if the stack is empty

        :return: 0 or 1 indicating the stack is empty or not
        """
        if (len(self.element) == 0):
            return 1
        else:
            return 0

    def clear(self):
        """Clear the stack and make it empty

        :return: None
        """
        self.element = []

    def result(self):
        """Get the elements in stack as a list

        :return: list of elements
        """
        return self.element


class priorityqueue(object):
    """Priorityqueue object

    This class is used to engulfs the initiation and supporting functions for priorityqueue object. priorityqueue is
    defined as heap data structure. This is implemented using a heapq library.
    Following public functions are implemented under this class.
        * push      : Insert element to the priorityqueue
        * pull      : Retrieve an element from the priorityqueue
        * is_empty  : check if the priorityqueue is empty
        * clear     : clear the priorityqueue and make it empty
        * result    : Get the elements in the priorityqueue as a list
    when instantiating a priorityqueue, it can be done by giving zero parameters, or giving a single element or a list.
    eg: No elements. Instantiation only
        priorityqueue_1 = priorityqueue()
    eg: Instantiation with a single element
        priorityqueue_1 = priorityqueue(element_1)
    eg: Instantiation with a list
        priorityqueue_1 = priorityqueue(list_1)
    """

    def __init__(self, element = None):
        """Init function for priorityqueue class

        :param element: empty element, single data element or a list to convert into a priorityqueue structure (default = None)
        """
        if (element == None):
            self.element = []
        elif type(element) == list:
            elementlist = copy.deepcopy(element)
            self.element = elementlist
            heapify(self.element)
        else:
            self.element = element

    def push(self, element):
        """Insert an element to the priorityqueue

        :param element: element to be inserted
        :return: None
        """
        heappush(self.element, element)

    def pull(self):
        """Retrieve an element from the priorityqueue

        :return: Retrieved element
        """
        return heappop(self.element)

    def is_empty(self):
        """Check if the priorityqueue is empty

        :return: 0 or 1 indicating the priorityqueue is empty or not
        """
        if(len(self.element)==0):
            return 1
        else:
            return 0

    def clear(self):
        """Clear the priorityqueue and make it empty

        :return: None
        """
        self.element = []

    def result(self):
        """Get the elements in priorityqueue as a list

        :return: list of elements
        """
        return self.element


"""Clear discovered flags"""
def ClearDiscover(dict):
    """Clear Nodes

    This function is used to clear the set parameters of the Nodes. When starting a new search algorithm execution
    you can use this to clear the discover status, and parent nodes of all the nodes in the graph.

    :param dict: Node dictionary containing the nodes for the graph
    :return: None
    """
    for key in dict:
        dict[key].setDiscover(0)
        dict[key].setParent(None)