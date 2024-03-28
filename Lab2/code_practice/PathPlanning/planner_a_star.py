import cv2
import sys
sys.path.append("..")
import PathPlanning.utils as utils
from PathPlanning.planner import Planner
from typing import Type, Optional, List

class HeapPoint():
    def __init__(self, position: Optional[tuple[int, int]] = None, value: Optional[int] = None) -> None:
        self.__position = position
        self.__value    = value

    def setPosition(self, position: Optional[tuple[int, int]] = None) -> None:
        self.__position = position

    def setValue(self, value: Optional[int] = None) -> None:
        self.__value = value

    def setPosValue(self, position: Optional[tuple[int, int]] = None, value: Optional[int] = None) -> None:
        self.__position = position
        self.__value    = value

    def getPosition(self) -> Optional[tuple[int, int]]:
        return self.__position

    def getValue(self) -> Optional[int]:
        return self.__value


class MinHeap():
    def __init__(self, array: Optional[List[tuple[int, int]]] = None) -> None:
        self.heap  = []
        self.length = 0

        if array is not None:
            if len(array) > 0:
                for item in array:
                    self.insert(item)

    def insert(self, item: HeapPoint) -> None:
        self.heap.append(item)
        self.length += 1
        self.__heapUp(self.length-1)

    def extractMin(self) -> HeapPoint:
        minpt = self.heap[0]
        self.__exchange(0, self.length-1)
        self.heap.pop()
        self.length -= 1
        self.__heapDown(0)

        return minpt

    def printHeap(self) -> None:
        print(f"heap = [ ")
        for i in self.heap:
            print(f"{i.getPosition()}, {i.getValue()} ")
        print(f"]")

    def __heapDown(self, pos: Optional[int]) -> None:
        left_child_idx = pos*2+1
        right_child_idx = pos*2+2

        while left_child_idx < self.length:
            min_idx = right_child_idx

            if right_child_idx < self.length:
                if self.heap[left_child_idx].getValue() < self.heap[right_child_idx].getValue():
                    min_idx = left_child_idx
            else:
                min_idx = left_child_idx

            if self.heap[min_idx].getValue() < self.heap[pos].getValue():
                self.__exchange(pos, min_idx)
                pos = min_idx
                left_child_idx = pos*2+1
                right_child_idx = pos*2+2
            else:
                break


    def __heapUp(self, index: Optional[int]) -> None:
        # Keep exchanging the point at index with its paranet if the value is
        # smaller until the root point.
        while index > 0 and self.heap[index].getValue() < self.heap[(index-1)//2].getValue():
            self.__exchange(index, (index-1)//2)
            index = (index-1)//2

    def __exchange(self, pos1: Optional[int], pos2: Optional[int]) -> None:
        tmp = self.heap[pos1]
        self.heap[pos1] = self.heap[pos2]
        self.heap[pos2] = tmp

class PlannerAStar(Planner):
    def __init__(self, m, inter=10):
        super().__init__(m)
        self.inter = inter
        self.initialize()

    def initialize(self):
        self.queue = []
        self.parent = {}
        self.h = {} # Distance from start to node
        self.g = {} # Distance from node to goal
        self.goal_node = None

    def planning(self, start=(100,200), goal=(375,520), inter=None, img=None):
        if inter is None:
            inter = self.inter
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))
        # Initialize
        self.initialize()
        self.queue.append(start)
        self.parent[start] = None
        self.g[start] = 0
        self.h[start] = utils.distance(start, goal)
        while(1):
            # TODO: A Star Algorithm
            minheap = MinHeap()
            minheap.insert(HeapPoint(start, self.g[start]+self.h[start]))
            minheap.printHeap()
            '''test heap code'''
            '''
            minheap = MinHeap()
            minheap.insert(HeapPoint((1, 1), 200))
            minheap.insert(HeapPoint((2, 1), 300))
            minheap.insert(HeapPoint((2, 43), 600))
            minheap.insert(HeapPoint((3, 43), 10))
            minheap.insert(HeapPoint((4, 43), 40))
            minheap.insert(HeapPoint((5, 43), 5))
            minheap.insert(HeapPoint((6, 43), 3333))
            minheap.printHeap()
            print(f"-----")
            minpt = minheap.extractMin()
            print(f"minpt = {minpt.getPosition()}, {minpt.getValue()}")
            minpt = minheap.extractMin()
            print(f"minpt = {minpt.getPosition()}, {minpt.getValue()}")
            minheap.printHeap()
            '''
            break

        # Extract path
        path = []
        p = self.goal_node
        if p is None:
            return path
        while(True):
            path.insert(0,p)
            if self.parent[p] is None:
                break
            p = self.parent[p]
        if path[-1] != goal:
            path.append(goal)
        return path


