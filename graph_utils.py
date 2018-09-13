import numpy as np
import heapq
import random
import time
from mark_constants import *
from math_utils import *
import spaces
import networkx as nx

class PathSearch():
    def __init__(self):
        pass
    
    def search_path(self, space, start, goal):
        raise NotImplemented()


#the nodes argument is needed by SingleQuery (SingleQuery might use other heuristics that need all nodes)
def simple_distance_heuristic(p1, p2, nodes=None): 
    return simple_distance(p1, p2)

class SingleQuery(PathSearch):
    def __init__(self, heuristic_func=simple_distance_heuristic):
        PathSearch.__init__(self)
        self.h = heuristic_func
    
    def search_path(self, space, start, goal, animate=False):
        assert space.dim == 2
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
                    if animate: space.draw_point(new_p, color=GREEN)
        
        path = [p2]
        while p_new.parent is not None:
            path.insert(0, p_new.p)
            p_new = p_new.parent
        path.insert(0,p1.p)
        return path


class Bug0(PathSearch):
    def __init__(self):
        PathSearch.__init__(self)
        
    def search_path(self, space, start, goal):
        assert type(space) is spaces.PolygonSpace
        
        path = [start]
        goal_line = to_line(to_hom(start),to_hom(goal))
        current_point = start
        while True:
            points_data = space.find_crosspoints(current_point, goal)
            
            #look for closest crosspoint = next obstacle
            min_distance = 2.0
            for point,i,polygon in points_data:
                distance = np.sum(np.abs(np.array(current_point) - np.array(point)))
                if distance < min_distance and distance > 1e-09:
                    min_distance = distance
                    next_point_data = (point,i,polygon)
            
            if min_distance > 1.5 or len(points_data) == 0:
                path.append(goal)
                return path
            
            path.append(next_point_data[0])
            
            #start going around object until crossing goal line again
            new_path = self._go_around(next_point_data, start, goal)
            
            path.extend(new_path)
            current_point = new_path[-1]
            
            
    def _go_around(self, next_point_data, start, goal):
        _,k,(points,polylines) = next_point_data
        
        num_points = len(points)-1 #start and end point of polygon are registered twice
        
        #figure out better direction
        point_pos = points[(k+1) % num_points]
        point_neg = points[k]
        distance_pos = np.sum(np.abs(np.array(goal) - np.array(point_pos)))
        distance_neg = np.sum(np.abs(np.array(goal) - np.array(point_neg)))
        if distance_pos < distance_neg:
            direction = +1
            i = (k+1) % num_points
        else:
            direction = -1
            i = k
            k = (k+1) % num_points
        
        current_point = points[i]
        path = [current_point]
        
        while i != k:
            goal_line = to_line(to_hom(current_point), to_hom(goal))
            i = (i+direction) % num_points
            next_point = points[i]
            next_point2 = points[(i+direction) % num_points]
            
            #check if polyline and goalline are pointed in such a way that the robot can move towards goal without hitting the polyline
            #this would require normal vector of the side of the obstacle where we're at
            #idea: check if the next two lines are on the same side w.r.t. the goal line
            #treat goal line as hyperplane w, like in classification: sgn(w^Tc) determines which side c is on
            if np.sign(np.inner(goal_line, to_hom(next_point))) == np.sign(np.inner(goal_line, to_hom(next_point2))):
                return path
            
            path.append(next_point)
            current_point = next_point
            
        raise ValueError('no path found, or maybe invalid polygon')
    
    
class Bug1(PathSearch):
    def __init__(self):
        PathSearch.__init__(self)
        
    def search_path(self, space, start, goal):
        pass
    
    
class Bug2(PathSearch):
    def __init__(self):
        PathSearch.__init__(self)
        
    def search_path(self, space, start, goal):
        assert type(space) is spaces.PolygonSpace
        
        path = [start]
        goal_line = to_line(to_hom(start),to_hom(goal))
        current_point = start
        while True:
            points_data = space.find_crosspoints(current_point, goal)
            
            #look for closest crosspoint = next obstacle
            min_distance = 2.0
            for point,i,polygon in points_data:
                distance = simple_distance(current_point, point)
                if distance < min_distance and distance > 1e-09:
                    min_distance = distance
                    next_point_data = (point,i,polygon)
            
            if min_distance > 1.5 or len(points_data) == 0:
                path.append(goal)
                return path
            
            path.append(next_point_data[0])
            
            #start going around object until crossing goal line again
            new_path = self._go_around(next_point_data, goal_line, start, goal)
            
            path.extend(new_path)
            current_point = new_path[-1]
            
            
    def _go_around(self, next_point_data, goal_line, start, goal):
        _,k,(points,polylines) = next_point_data
        
        num_points = len(points)-1 #start and end point of polygon are registered twice
        
        #figure out better direction
        point_pos = points[(k+1) % num_points]
        point_neg = points[k]
        distance_pos = simple_distance(goal, point_pos)
        distance_neg = simple_distance(goal, point_neg)
        if distance_pos < distance_neg:
            direction,line_dir = +1,0
            i = (k+1) % num_points
        else:
            direction,line_dir = -1,-1
            i = k
            k = (k+1) % num_points
        
        current_point = points[i]
        path = [current_point]
        
        while i != k:
            line = polylines[i+line_dir] #module not necessary, only -1 can happen which is fine in python
            i = (i+direction) % num_points
            next_point = points[i]
            
            crosspoint = calculate_crosspoint(line, goal_line, current_point, next_point, start, goal)
            if crosspoint is not None:
                path.append(tuple(crosspoint))
                return path
            
            path.append(next_point)
            current_point = next_point
            
        raise ValueError('no path found, or maybe invalid polygon')
            
        
    
    

class VisibilityGraph(PathSearch):
    def __init__(self):
        PathSearch.__init__(self)
        self.graph = None
    
    def construct_graph(self, space, start, goal):
        assert type(space) is spaces.PolygonSpace
        self.graph = nx.Graph()
        g = self.graph
        g.add_node(start)
        g.add_node(goal)
        if space.check_line(goal, start):
            g.add_edge(start, goal, weight=simple_distance(start, goal))
        
        #add all polygons
        for points,_ in space.polygons:
            for i in range(len(points)-1):
                #nodes are added automatically
                g.add_edge(points[i], points[i+1], weight=simple_distance(points[i],points[i+1]))
                
                #check connectivity to other visible nodes (only current nodes in the graph need to be considered)
                for node in g.nodes():
                    if space.check_line(node, points[i]):
                        g.add_edge(node, points[i], weight=simple_distance(node,points[i]))
        
        return g #return might be useful
        
    
    def search_path(self, space, start, goal):
        self.construct_graph(space, start, goal)
        path = nx.astar_path(self.graph, start, goal, heuristic=simple_distance, weight='weight')
        return path
    
    def display(self, space, path=None):
        if self.graph is None: return
        assert type(space) is spaces.PolygonSpace
        
        #for now abuse grid space for this, like polyspace
        space_vis = spaces.GridSpace([250]*space.dim)
        
        for line in self.graph.edges():
            space_vis.draw_line(line[0], line[1], color=BLUE)
        
        for points,_ in space.polygons:
            for i in range(len(points)-1):
                space_vis.draw_line(points[i], points[i+1], color=OCCUPIED_VIS)
        
        return space_vis.display(path)
        

        
        
class VoronoiDiagram(PathSearch):
    def __init__(self):
        PathSearch.__init__(self)
        self.voronoi = None
        
    def create_voronoi_diagram(self, space):
        assert type(space) is spaces.GridSpace #NOTE: only marked line below needs to be made compatible with other spaces
        MAX_VAL = 9999999
        self.voronoi = np.zeros(space.dims, np.int) +MAX_VAL
        
        directions = []
        for i in range(space.dim):
            dir_a, dir_b = np.array([0]*space.dim, dtype=np.int), np.array([0]*space.dim, dtype=np.int)
            dir_a[i] = 1
            dir_b[i] = -1
            directions.append(dir_a)
            directions.append(dir_b)
        
        zero = np.array([0]*space.dim, dtype=np.int)
        self.voronoi[tuple(zero)] = 0
        
        #algorithm is similar to dijkstra
        frontier = [(zero, zero)]
        while frontier:
            index, from_dir = frontier.pop()
            
            next_val = self.voronoi[tuple(index)] + 1
                
            for d in directions:
                if np.array_equal(d, from_dir): continue
                next_index = index + d
                
                #NOTE: space.occupied needs to be made compatible with spaces other than grid space
                if space.occupied_index(next_index):
                    if next_val == 1: continue #i.e. current value is 0 (which it should be for pixel at the border)
                    #if not: correct current value and repeat for this pixel
                    self.voronoi[tuple(index)] = 0
                    frontier.append((index, from_dir))
                    break
                
                if self.voronoi[tuple(next_index)] > next_val: #-> shorter path to pixel found
                    self.voronoi[tuple(next_index)] = next_val
                    frontier.append((next_index, -1*d)) #from_dir is opposite direction that we just went
            
        self.voronoi[self.voronoi == MAX_VAL] = 0 #unreachable_pixels -> 0
    
    def search_path(self, space, start, goal):
        self.create_voronoi_diagram(space)
        #TODO search path
        return [start, goal]
    
    def display(self, path=None):
        if self.voronoi is None: return
        
        space_vis = spaces.GridSpace(self.voronoi.shape)
        voronoi = self.voronoi/np.max(self.voronoi) #convert int voronoi to uint8, normalize data for this -> creates prettier images
        space_vis.grid = (voronoi*255).astype(np.uint8)
        space_vis.grid_to_vis()
        
        return space_vis.display(path)