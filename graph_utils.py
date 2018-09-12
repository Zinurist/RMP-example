import numpy as np
import heapq
import random
import time
from mark_constants import *

class GraphSearch():
    def __init__(self):
        pass
    
    
    def search_path(self, space, start, goal):
        raise NotImplemented()

    

def simple_distance(p1, p2, nodes):
    return np.sum(np.abs(p1-p2))

class SingleQuery(GraphSearch):
    def __init__(self, heuristic_func=simple_distance):
        GraphSearch.__init__(self)
        self.h = heuristic_func
    
    def search_path(self, space, start, goal, animate=False):
        assert len(space.dims) == 2
        p1,p2 = np.array(start),np.array(goal)
        
        class Node:
            def __init__(self, p, parent):
                self.p, self.parent = p, parent
            def __str__(self):
                return str(self.p)
        
        factor = 30.0
        max_distance = factor/min(space.dims)
        diff = int(min(space.dims)/factor)
        visited = np.zeros(space.dims, dtype=np.bool)
        
        d1 = self.h(p1, p2, [])
        p1 = Node(p1, None)
        nodes = [(d1,p1)]
        frontier_heap = [(d1, p1)]
        repick = False
        reached_goal = False
        while not reached_goal:
            if animate: space.display()
            time.sleep(0.1)
            
            if len(frontier_heap) == 0:
                repick = True
                frontier_heap = nodes[:]
                
            _,p = frontier_heap.pop(0)
            
            #expand direction (roughly)
            if repick or p.parent is None:
                direction = np.array([0,0])
            else:
                direction = p.p - p.parent.p
                
            for i in range(15):
                v = direction + (np.random.rand(2)-0.5) * max_distance
                new_p = p.p+v
                if not space.in_bounds(new_p): continue
                new_indices = space.at(new_p)
                if not visited[new_indices] and space.check_line(p.p, new_p):
                    p_new = Node(new_p, p)
                    if space.check_line(new_p, p2): 
                        reached_goal = True
                        break
                        
                    heuristic = self.h(new_p, p2, nodes)
                    heapq.heappush(frontier_heap, (heuristic, p_new))
                    heapq.heappush(nodes, (heuristic, p_new))
                    ix,iy = new_indices[0],new_indices[1]
                    ix_min, iy_min = max(ix-diff, 0), max(iy-diff, 0)
                    ix_max, iy_max = min(ix+diff, space.dims[0]), min(iy+diff, space.dims[1])
                    visited[ix_min:ix_max,iy_min:iy_max] = True
                    if animate: space.mark(new_p, mark_vis=GREEN)
        
        path = [p2]
        space.add_line(p_new.p, p2, mark=None, mark_vis=LIGHT_GREEN)
        while p_new.parent is not None:
            path.insert(0, p_new.p)
            space.add_line(p_new.p, p_new.parent.p, mark=None, mark_vis=LIGHT_GREEN)
            p_new = p_new.parent
        path.insert(0,p1.p)
        return path

