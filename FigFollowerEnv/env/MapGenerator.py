import numpy as np
from random import randint, shuffle
from collections import namedtuple
import math

class MapGenerator:
    Node = namedtuple('Node', ['x', 'y', 'type', 'angle'])
    def __init__(self, size:int, max_nodes:int, max_steps:int, min_steps = 2):
        self.size = size + 2
        self.max_nodes = max_nodes
        self.max_steps = max_steps
        self.min_steps = min_steps
        self.lower_bound = 0
        self.upper_bound = self.size - 1
        self.transitions = [
            self._create_transition(90),  #Triangle
            self._create_transition(-90), #Square
            self._create_transition(180)  #Pentagon
        ]

    def generate(self):
        nodes = []
        map = np.zeros((self.size, self.size), dtype=int)
        stop = False
        # First Node
        pos = np.array([[self.size//2],[self.size//2]], dtype=int)
        trans = np.array([[1],[0]], dtype=int)
        indices = [0, 1, 2]
        self._add_node(nodes, pos, 'Origin', trans)
        steps = self._walk(map, pos, trans, randint(self.min_steps, self.max_steps))
        self._walk_and_mark(map, pos, trans, steps)
        pos = trans * steps + pos
        # The remainder nodes
        while len(nodes) < self.max_nodes + 1 and not stop:
            shuffle(indices)
            steps = 0
            i = 0
            ori_dir = trans.copy()
            for i in indices:
                trans = self.transitions[i] @ ori_dir
                steps = self._walk(map, pos, trans, randint(self.min_steps, self.max_steps))
                if steps > 0:
                    break
            if steps == 0:
                stop = True
            else:
                figures = ['Triangle', 'Square', 'Pentagon']
                self._add_node(nodes, pos + ori_dir, figures[i], ori_dir)
                self._walk_and_mark(map, pos, trans, steps)
                self._mark_map(map, nodes[-1])
                pos = trans * steps + pos
        last = nodes.pop()
        new_last = MapGenerator.Node(last.x, last.y, 'Circle', last.angle)
        nodes.append(new_last)
        self._mark_map(map, nodes[-1])
        return nodes, map

            
    def _add_node(self, nodes, pos, type, trans):
        x = pos[0,0]
        y = pos[1,0]
        dx = trans[0,0]
        dy = trans[1,0]
        nodes.append(MapGenerator.Node(x, y, type, math.degrees(math.atan2(dy, dx))))

    
    def _create_transition(self, angle):
        theta = math.radians(angle)
        return np.array([[math.cos(theta), -math.sin(theta)],
                         [math.sin(theta),  math.cos(theta)]], dtype=int)


    def _walk(self, map, start, trans, steps):
        best_available_step = 0
        left_T = self._create_transition(-90)
        right_T = self._create_transition(90)
        #print("Start")
        for step in range(steps):
            pos = (trans * step) + start
            left = (left_T @ trans) + pos
            right = (right_T @ trans) + pos
            front = trans + pos
            x = pos[0,0]
            y = pos[1,0]
            #print(f"Pos: {pos}")
            #print(f"L: {left}")
            #print(f"R: {right}")
            #print(f"step: {step} x: {x} y: {y}")
            x_bound_check = x <= self.lower_bound or x >= self.upper_bound
            y_bound_check = y <= self.lower_bound or y >= self.upper_bound
            if map[y, x] < 2 and not x_bound_check and not y_bound_check:
                left_check = map[left[1,0], left[0,0]] == 0
                right_check = map[right[1,0], right[0,0]] == 0
                front_check = map[front[1,0], front[0,0]] == 0
                if front_check and left_check and right_check:
                    best_available_step = step
                    #print(best_available_step)
            else:
                break
        return best_available_step
    
    
    def _mark_map(self, map, node):
        miau = {'Origin':1, 'Triangle':2, 'Square':3, 'Pentagon':4, 'Circle':5}
        map[node.y, node.x] = miau[node.type]

    
    def _walk_and_mark(self, map, start, trans, steps):
        for step in range(steps+1):
            pos = (trans * step) + start
            x = pos[0,0]
            y = pos[1,0]
            map[y, x] = 1


if __name__ == "__main__":
    gen = MapGenerator(10, 10, 5)
    nodes, map = gen.generate()
    print(map)
    print(nodes)