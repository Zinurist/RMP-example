import numpy as np
from PIL import Image
import time
from IPython.display import display, clear_output
import cv2
from mark_constants import *
from math_utils import *


#simple arm with 2 segments, note that theta refers to the angle as percentage of 2*pi (i.e. between 0. and 1.)
class Arm():
    def __init__(self, theta1, theta2, l1, l2):
        self.theta1, self.theta2, self.l1, self.l2 = theta1, theta2, l1, l2
    
    def set_thetas(self, thetas):
        self.theta1, self.theta2 = thetas[0], thetas[1]
    
    def as_coordinates(self):
        v1 = np.array([np.cos(self.theta1*np.pi*2), np.sin(self.theta1*np.pi*2)])
        v2 = np.array([np.cos(self.theta2*np.pi*2), np.sin(self.theta2*np.pi*2)])

        p1 = np.array([0.5, 0.2])
        p2 = p1 + v1*self.l1
        p3 = p2 + v2*self.l2
        return p1, p2, p3
    
class Space():
    def __init__(self, dims):
        self.dims = tuple(dims)
        self.dim = len(dims)
        
    def in_bounds(self, p):
        return (p >=0.0).all() and (p <= 1.0).all()
    
    def at(self, point, as_array=False):
        assert len(point) == self.dim
        indices = []
        for i in range(len(self.dims)):
            indices.append(int(self.dims[i]*point[i]))
            if indices[i] == self.dims[i]: indices[i] -= 1
        if as_array: return np.array(indices, dtype=np.int)
        return tuple(indices)
    
    def draw_point(self, p, color):
        pass
    
    def draw_line(self, p1, p2, color):
        pass
    
    def display(self, path=None):
        pass
    
    def draw_path(self, path, color_path=LIGHT_GREEN, color_start=BLUE, color_goal=LIGHT_BLUE):
        for i in range(len(path)-1):
            self.draw_line(path[i], path[i+1], color_path)
        self.draw_point(path[0], color_start)
        self.draw_point(path[-1], color_goal)
    

class GridSpace(Space):
    def __init__(self, dims):
        Space.__init__(self, dims)
        dims = list(dims)
        self.grid = np.zeros(dims, dtype=np.uint8)
        dims.append(3)
        self.grid_vis = np.zeros(dims, dtype=np.uint8) +255
        self.checkpoint_vis()
    
    
    def checkpoint_vis(self):
        self._checkpoint_vis = np.copy(self.grid_vis)
    
    
    def reset_vis(self):
        self.grid_vis = np.copy(self._checkpoint_vis)
        
    
    def draw_point(self, p, color):
        self.mark(p, mark=None, mark_vis=color)
        
    
    def draw_line(self, p1, p2, color):
        self.add_line(p1, p2, mark=None, mark_vis=color)
        
    
    def mark(self, point, mark=None, mark_vis=None):
        if mark is not None: self.grid[self.at(point)] = mark
        if mark_vis is not None: self.grid_vis[self.at(point)] = mark_vis
    
    
    def mark_indices(self, indices, mark=None, mark_vis=None):
        if mark is not None: self.grid[tuple(indices)] = mark
        if mark_vis is not None: self.grid_vis[tuple(indices)] = mark_vis
    
    
    def occupied(self, point=None):
        if not self.in_bounds(point): return False
        return self.grid[self.at(point)] == OCCUPIED
    
    
    def check_arm(self, arm):
        p1, p2, p3 = arm.as_coordinates()
        return self.check_line(p1, p2) and self.check_line(p2, p3)
    
    
    def check_line(self, p1, p2):
        assert len(self.dims) == 2
        if not(self.in_bounds(p1) and self.in_bounds(p2)): return False
        
        #smallest step size: size of a cell dx,dy
        dx,dy = 1.0/self.dims[0],1.0/self.dims[1]
        d = min(dx,dy)
        v = p2-p1
        v_max = np.max(np.abs(v))
        steps = int(np.ceil(v_max/d))
        v = v / steps
        for i in range(steps+1):
            if self.occupied(p1 + i*v): return False
        return True
        
    
    def add_arm(self, arm, mark=ARM, mark_vis=ARM_VIS):
        p1, p2, p3 = arm.as_coordinates()
        self.add_line(p1, p2, mark=mark, mark_vis=mark_vis)
        self.add_line(p2, p3, mark=mark, mark_vis=mark_vis)
        
        
    def add_line(self, p1, p2, mark=None, mark_vis=None):
        assert len(self.dims) == 2
        p1,p2 = np.array(p1),np.array(p2)
        #smallest step size: size of a cell dx,dy
        dx,dy = 1.0/self.dims[0],1.0/self.dims[1]
        d = min(dx,dy)
        v = p2-p1
        v_max = np.max(np.abs(v))
        steps = int(np.ceil(v_max/d))+1
        v = v / steps
        for i in range(steps):
            p = p1 + i*v
            if not self.in_bounds(p): return
            self.mark(p, mark, mark_vis)
    
    
    def add_rect(self, rect, mark=OCCUPIED, mark_vis=OCCUPIED_VIS):
        assert len(self.dims) == 2
        start = self.at([rect.x, rect.y])
        end = self.at([rect.x+rect.width, rect.y+rect.height])
        self.grid[start[0]:end[0], start[1]:end[1]] = mark
        self.grid_vis[start[0]:end[0], start[1]:end[1]] = mark_vis
        
        
    def animate_arm(self, arm, path, video_file=None, secs_at_start=1, secs_at_end=1):
        
        if video_file is not None:
            fourcc = cv2.VideoWriter_fourcc(*'MPEG')
            writer = cv2.VideoWriter(video_file, fourcc, 30.0, (500,500))
        
        #always take secs_per_point seconds between two points in the path (in the animation)
        secs_per_point = 1
        hertz = 30
        frames = hertz * secs_per_point
        dt = 1./frames
        
        if video_file is not None:
            arm.set_thetas(path[0])
            self.add_arm(arm, mark=None, mark_vis=BLUE)
            arm.set_thetas(path[-1])
            self.add_arm(arm, mark=None, mark_vis=LIGHT_BLUE)
            
            img = np.array(self.display())
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            for i in range(secs_at_start*hertz):
                writer.write(img)
        
        for i in range(len(path)-1):
            p1, p2 = path[i], path[i+1]
            v = p2 - p1
            
            for frame in range(frames):
                #draw arm
                self.reset_vis()
                p = p1 + v*frame*dt
                arm.set_thetas(p)
                self.add_arm(arm, mark=None)
                
                #display/write to video
                img = np.array(self.display())
                if video_file is not None:
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    writer.write(img)
                time.sleep(dt)
                
        if video_file is not None:
            for i in range(secs_at_end*hertz):
                writer.write(img)
            writer.release()
            
        
    def display(self, path=None):
        img = np.swapaxes(self.grid_vis, 1, 0)
        img = Image.fromarray(img, 'RGB')
        img = img.resize((500,500))
        clear_output(wait=True)
        display(img)
        if path is not None: img.save(path)
        return img


    
class PolygonSpace(Space):
    def __init__(self, dim):
        Space.__init__(self, [100]*dim)
        self.polygons = []
        self.points_to_draw = []
        self.lines_to_draw = []
    
    def draw_point(self, p, color):
        self.points_to_draw.append((p, color))
        
    def draw_line(self, p1, p2, color):
        self.lines_to_draw.append((p1,p2,color))
    
    def add_polygon(self, points):
        #calculate line segments in homogenous coords
        polylines = []
        for i in range(len(points)-1):
            p1, p2 = to_hom(points[i]), to_hom(points[i+1])
            polylines.append(to_line(p1,p2))
            
        self.polygons.append((points, polylines))
        
        
    #NOTE: if you have a circle as a robot and you're trying to check if it collides with an object when going along a line,
    #then instead you could model the object bigger (extend it by the radius of the robot) and the robot only as a point
    #the normal check_line here would then work
        
    def check_line(self, p1, p2):
        assert self.dim == 2
        p1, p2 = to_hom(p1), to_hom(p2)
        line = to_line(p1,p2)
        
        #check for collisions with all polygons
        for points,polylines in self.polygons:
            for i in range(len(points)-1):
                crosspoint = calculate_crosspoint(line, polylines[i], p1, p2, points[i], points[i+1])
                if crosspoint is not None:
                    return False
        return True
    
    
    def find_crosspoints(self, p1, p2):
        assert self.dim == 2
        p1, p2 = to_hom(p1), to_hom(p2)
        line = to_line(p1,p2)
        
        crosspoints = []
        for points,polylines in self.polygons:
            for i in range(len(points)-1):
                crosspoint = calculate_crosspoint(line, polylines[i], p1, p2, points[i], points[i+1])
                if crosspoint is not None:
                    crosspoints.append((crosspoint, i, (points,polylines)))
        return crosspoints
    
    
    def check_circle(self, p, r):
        p = np.array(p)
        for points,_ in self.polygons:
            for polyp in points:
                if np.linalg.norm(polyp - p) < r: return False
        return True
        
    
    def display(self, path=None, empty_lists=False):
        #for now abuse grid space for this...
        space = GridSpace([250]*self.dim)
        
        for points,polylines in self.polygons:
            for i in range(len(points)-1):
                space.draw_line(points[i], points[i+1], color=OCCUPIED_VIS)
        
        for line in self.lines_to_draw:
            space.draw_line(line[0], line[1], color=line[2])
        for point in self.points_to_draw:
            space.draw_point(point[0], color=point[1])
        
        if empty_lists:
            self.lines_to_draw = []
            self.points_to_draw = []
                
        return space.display(path)
        
        
        
        
class Rect():
    def __init__(self, x, y, width, height):
        self.x, self.y, self.width, self.height = x, y, width, height
        
#create c-space given the space and arm, dims refers to size of the c-space
def create_arm_c_space(space, arm, dims=[500,500]):
    assert len(dims) == 2 and len(space.dims) == 2
    c_space = GridSpace(dims)
    for t1 in range(dims[0]):
        for t2 in range(dims[1]):
            arm.set_thetas((float(t1)/dims[0], float(t2)/dims[1]))
            free = space.check_arm(arm)
            if not free: 
                c_space.mark_indices((t1,t2), mark=OCCUPIED, mark_vis=OCCUPIED_VIS)
    return c_space