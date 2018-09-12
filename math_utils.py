import numpy as np

def calculate_crosspoint(l1, l2, p1_l1, p2_l1, p1_l2, p2_l2):
    crosspoint = np.cross(l1, l2)
    if abs(crosspoint[2]) < 1e-10:
        return None
    crosspoint = to_norm(crosspoint)
    if in_bounds(crosspoint, p1_l1, p2_l1) and in_bounds(crosspoint, p1_l2, p2_l2):
        return crosspoint
                
def in_bounds(p, p1, p2): 
    xMin, xMax = min(p1[0], p2[0])-1e-09, max(p1[0], p2[0])+1e-09
    yMin, yMax = min(p1[1], p2[1])-1e-09, max(p1[1], p2[1])+1e-09
    return (p[0]>=xMin and p[0]<=xMax and p[1]>=yMin and p[1]<=yMax)
        
def to_hom(p):
    p = list(p)
    p.append(1)
    return np.array(p)

def to_norm(p): 
    return np.array([p[0]/p[2], p[1]/p[2]])

def to_line(p1, p2):
    return np.cross(p1, p2)
