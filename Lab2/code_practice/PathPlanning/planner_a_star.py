import cv2
import sys
sys.path.append("..")
import PathPlanning.utils as utils
from PathPlanning.planner import Planner
from typing import Type, Optional, List, Union, Dict

class HeapPoint():
    def __init__(self, position: Optional[tuple[int, int]] = None, value: Optional[Union[int, float]] = None) -> None:
        self.__position = position
        self.__value    = value

    def setPosition(self, position: Optional[tuple[int, int]] = None) -> None:
        self.__position = position

    def setValue(self, value: Optional[Union[int, float]] = None) -> None:
        self.__value = value

    def setPosValue(self, position: Optional[tuple[int, int]] = None, value: Optional[Union[int, float]] = None) -> None:
        self.__position = position
        self.__value    = value

    def getPosition(self) -> Optional[tuple[int, int]]:
        return self.__position

    def getValue(self) -> Optional[Union[int, float]]:
        return self.__value


class MinHeap():
    def __init__(self) -> None:
        self.heap  = []
        self.length = 0
        self.map_pos2value = {}

    def insert(self, item: HeapPoint) -> None:
        self.heap.append(item)
        self.map_pos2value[item.getPosition()] = self.length
        self.length += 1
        self.__heapUp(self.length-1)

    def extractMin(self) -> HeapPoint:
        minpt = self.heap[0]
        self.__exchange(0, self.length-1)
        self.heap.pop()
        self.map_pos2value.pop(minpt.getPosition())
        self.length -= 1
        self.__heapDown(0)

        return minpt

    def isExist(self, pos: tuple[int, int]) -> bool:
        if pos in self.map_pos2value:
            return True
        else:
            return False

    def isEmpty(self) -> bool:
        if self.length == 0:
            return True
        else:
            return False

    def getHeapPtValue(self, pos: tuple[int, int]) -> Union[int, float]:
        return self.heap[self.map_pos2value[pos]].getValue()

    def setNewValue(self, position: tuple[int, int], new_value: Optional[Union[int, float]]) -> None:
        pos = self.map_pos2value[position]
        orig_value = self.heap[pos].getValue()
        self.heap[pos].setValue(new_value)

        if orig_value > new_value:
            self.__heapUp(pos)
        elif orig_value < new_value:
            self.__heapDown(pos)

    def printHeap(self) -> None:
        print(f"heap = [ ")
        for i in self.heap:
            print(f"{i.getPosition()}, {i.getValue()} ")
        print(f"]")

    def printMap(self) -> None:
        sorted_map = {k: v for k, v in sorted(self.map_pos2value.items(), key=lambda item:item[1])}
        print(f"map_pos2value = ")
        for items in sorted_map.items():
            print(f"{items}")

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
        self.map_pos2value[self.heap[pos1].getPosition()] = pos1
        self.heap[pos2] = tmp
        self.map_pos2value[self.heap[pos2].getPosition()] = pos2

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

    def __testMinHeap(self, minheap: MinHeap) -> None:
        print(f"minheap.isEmpty() = {minheap.isEmpty()}")
        minheap.insert(HeapPoint((1, 1), 200))
        minheap.insert(HeapPoint((2, 1), 300))
        minheap.insert(HeapPoint((2, 43), 600))
        minheap.insert(HeapPoint((3, 43), 10))
        minheap.insert(HeapPoint((4, 43), 40))
        minheap.insert(HeapPoint((5, 43), 5))
        minheap.insert(HeapPoint((6, 43), 3333))
        minheap.printHeap()
        minheap.printMap()
        print(f"-----")
        minpt = minheap.extractMin()
        print(f"minpt = {minpt.getPosition()}, {minpt.getValue()}")
        minpt = minheap.extractMin()
        print(f"minpt = {minpt.getPosition()}, {minpt.getValue()}")
        minheap.printHeap()
        minheap.printMap()
        print(f"-----")
        minheap.setNewValue((6, 43), 6666)
        minheap.printHeap()
        minheap.printMap()
        print(f"-----")
        minheap.setNewValue((6, 43), 20)
        minheap.printHeap()
        minheap.printMap()
        print(f"-----")
        print(f"minheap.isEmpty() = {minheap.isEmpty()}")
        minpt = minheap.extractMin()
        minpt = minheap.extractMin()
        minpt = minheap.extractMin()
        minpt = minheap.extractMin()
        minpt = minheap.extractMin()
        minheap.printHeap()
        minheap.printMap()
        print(f"minheap.isEmpty() = {minheap.isEmpty()}")

    def buildAdj(self, img=None) -> Dict[tuple[int, int], List[tuple[int, int]]]:
        row = img.shape[0]
        col = img.shape[1]
        adj_list = {}

        for i in range(row):
            for j in range(col):
                if self.map[int(i),int(j)]<0.5:
                    continue
                else:
                    adj_list[(j, i)] = []

                    if j+1 < col and self.map[int(i),int(j+1)]>=0.5:
                        adj_list[(j, i)].append((j+1, i))
                    if j-1 > -1  and self.map[int(i),int(j-1)]>=0.5:
                        adj_list[(j, i)].append((j-1, i))
                    if i+1 < row  and self.map[int(i+1),int(j)]>=0.5:
                        adj_list[(j, i)].append((j, i+1))
                    if i-1 > -1  and self.map[int(i-1),int(j)]>=0.5:
                        adj_list[(j, i)].append((j, i-1))
                    if i-1 > -1 and j-1 > -1 and self.map[int(i-1),int(j-1)]>=0.5:
                        adj_list[(j, i)].append((j-1, i-1))
                    if i-1 > -1 and j+1 < col and self.map[int(i-1),int(j+1)]>=0.5:
                        adj_list[(j, i)].append((j+1, i-1))
                    if i+1 < row and j-1 > -1 and self.map[int(i+1),int(j-1)]>=0.5:
                        adj_list[(j, i)].append((j-1, i+1))
                    if i+1 < row and j+1 < col and self.map[int(i+1),int(j+1)]>=0.5:
                        adj_list[(j, i)].append((j+1, i+1))

        return adj_list

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
        visited = {}

        print(f"> buildAdj()...")
        adj_list = self.buildAdj(img)
        minheap = MinHeap()
        minheap.insert(HeapPoint(start, self.g[start]+self.h[start]))
        print(f"> A*()...")

        '''test heap code'''
        '''
        self.__testMinHeap(minheap)
        '''

        while(not minheap.isEmpty()):
            # TODO: A Star Algorithm
            minpt = minheap.extractMin()
            minpt_pos = minpt.getPosition()
            visited[minpt_pos] = True

            if minpt_pos[0] == goal[0] and minpt_pos[1] == goal[1]:
                self.goal_node = goal
                break

            for each_neighbor in adj_list[minpt_pos]:
                if each_neighbor in visited:
                    continue

                if each_neighbor in self.g:
                    self.g[each_neighbor] = min(self.g[minpt_pos] + utils.distance(minpt_pos, each_neighbor), self.g[each_neighbor])
                else:
                    self.g[each_neighbor] = self.g[minpt_pos] + utils.distance(minpt_pos, each_neighbor)

                self.h[each_neighbor] = utils.distance(each_neighbor, goal)
                key_value = self.g[each_neighbor] + self.h[each_neighbor]

                if minheap.isExist(each_neighbor):
                    orig_key_value = minheap.getHeapPtValue(each_neighbor)

                    if orig_key_value > key_value:
                        minheap.setNewValue(each_neighbor, key_value)
                        self.parent[each_neighbor] = minpt_pos
                else:
                    minheap.insert(HeapPoint(each_neighbor, key_value))
                    self.parent[each_neighbor] = minpt_pos

        print(f"> extractPath()...")
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

        print(f"len(path) = {len(path)}")
        return path


