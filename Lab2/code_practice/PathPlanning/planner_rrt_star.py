import cv2
import numpy as np
import sys
sys.path.append("..")
import PathPlanning.utils as utils
from PathPlanning.planner import Planner
from typing import Type, Optional, List, Union, Dict

class PlannerRRTStar(Planner):
    def __init__(self, m, extend_len=20):
        super().__init__(m)
        self.extend_len = extend_len

    def _random_node(self, goal, shape):
        r = np.random.choice(2,1,p=[0.5,0.5])
        if r==1:
            return (float(goal[0]), float(goal[1]))
        else:
            rx = float(np.random.randint(int(shape[1])))
            ry = float(np.random.randint(int(shape[0])))
            return (rx, ry)

    def _nearest_node(self, samp_node):
        min_dist = 99999
        min_node = None
        for n in self.ntree:
            dist = utils.distance(n, samp_node)
            if dist < min_dist:
                min_dist = dist
                min_node = n
        return min_node

    def _check_collision(self, n1, n2):
        n1_ = utils.pos_int(n1)
        n2_ = utils.pos_int(n2)
        line = utils.Bresenham(n1_[0], n2_[0], n1_[1], n2_[1])
        for pts in line:
            if self.map[int(pts[1]),int(pts[0])]<0.5:
                return True
        return False

    def _steer(self, from_node, to_node, extend_len):
        vect = np.array(to_node) - np.array(from_node)
        v_len = np.hypot(vect[0], vect[1])
        v_theta = np.arctan2(vect[1], vect[0])
        if extend_len > v_len:
            extend_len = v_len
        new_node = (from_node[0]+extend_len*np.cos(v_theta), from_node[1]+extend_len*np.sin(v_theta))
        if new_node[1]<0 or new_node[1]>=self.map.shape[0] or new_node[0]<0 or new_node[0]>=self.map.shape[1] or self._check_collision(from_node, new_node):
            return False, None
        else:
            return new_node, utils.distance(new_node, from_node)

    def _isDiff(self, node1: tuple[int, int], node2: tuple[int, int]) -> bool:
        if node1[0] == node2[0] and node1[1] == node2[1]:
            return False
        else:
            return True

    def _get_near_node(self, new_node: tuple[int, int], distance: Union[int, float], row: int, col: int) -> List[tuple[int, int]]:
        left_bound_x  = max(new_node[0] - int(distance), 0)
        left_bound_y  = max(new_node[1] - int(distance), 0)
        right_bound_x = min(new_node[0] + int(distance), col)
        right_bound_y = min(new_node[1] + int(distance), row)
        near_node_list = []

        for i in range(int(left_bound_y), int(right_bound_y)):
            for j in range(int(left_bound_x), int(right_bound_x)):
                if (j, i) in self.map_int2float_node:
                    for cand_node in self.map_int2float_node[(j, i)]:
                        if utils.distance(cand_node, new_node) < distance and self._isDiff(cand_node, new_node):
                            near_node_list.append(cand_node)

        return near_node_list

    def _reparent(self, near_node_list: List[tuple[int, int]], new_node: tuple[int, int]) -> None:
        for cand_near_node in near_node_list:
            if self._check_collision(cand_near_node, new_node):
                continue
            cand_cost = self.cost[cand_near_node] + utils.distance(cand_near_node, new_node)
            if cand_cost < self.cost[new_node]:
                self.ntree[new_node] = cand_near_node
                self.cost[new_node]  = cand_cost

    def _rewire(self, near_node_list: List[tuple[int, int]], new_node: tuple[int, int]) -> None:
        for cand_near_node in near_node_list:
            if self._check_collision(new_node, cand_near_node):
                continue
            cand_cost = self.cost[new_node] + utils.distance(new_node, cand_near_node)
            if cand_cost < self.cost[cand_near_node]:
                self.ntree[cand_near_node] = new_node
                self.cost[cand_near_node] = cand_cost

    def planning(self, start, goal, extend_len=None, img=None):
        if extend_len is None:
            extend_len = self.extend_len
        self.ntree = {}
        self.ntree[start] = None
        self.cost = {}
        self.cost[start] = 0
        self.map_int2float_node = {}
        self.map_int2float_node[start] = [start]

        distance = 3*extend_len
        goal_node = None
        for it in range(20000):
            #print("\r", it, len(self.ntree), end="")
            samp_node = self._random_node(goal, self.map.shape)
            near_node = self._nearest_node(samp_node)
            new_node, cost = self._steer(near_node, samp_node, extend_len)
            if new_node is not False:
                self.ntree[new_node] = near_node
                self.cost[new_node] = cost + self.cost[near_node]
                if utils.pos_int(new_node) in self.map_int2float_node:
                    self.map_int2float_node[utils.pos_int(new_node)].append(new_node)
                else:
                    self.map_int2float_node[utils.pos_int(new_node)] = [new_node]
            else:
                continue

            if utils.distance(near_node, goal) < extend_len:
                goal_node = near_node
                break

            # TODO: Re-Parent & Re-Wire
            if new_node is not False:
                near_node_list = self._get_near_node(new_node, distance, self.map.shape[0], self.map.shape[1])
                self._reparent(near_node_list, new_node)
                self._rewire(near_node_list, new_node)

            # Draw
            if img is not None:
                for n in self.ntree:
                    if self.ntree[n] is None:
                        continue
                    node = self.ntree[n]
                    cv2.line(img, (int(n[0]), int(n[1])), (int(node[0]), int(node[1])), (0,1,0), 1)
                # Near Node
                img_ = img.copy()
                cv2.circle(img_,utils.pos_int(new_node),5,(0,0.5,1),3)
                # Draw Image
                img_ = cv2.flip(img_,0)
                cv2.imshow("Path Planning",img_)
                k = cv2.waitKey(1)
                if k == 27:
                    break

        # Extract Path
        path = []
        n = goal_node
        while(True):
            if n is None:
                break
            path.insert(0,n)
            n = self.ntree[n]
        path.append(goal)
        return path
