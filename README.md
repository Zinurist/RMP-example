# Collection of robot motion planning examples
## Simple arm

Task: Move arm from blue position (start) to light blue position (goal) without touching the rectangles. 
The arm's movement is described by two angles, one at the base of the robot, and one at joint of the two arms.


![space.png](example/space.png)

First convert to configuration space (c-space): The axes are now the angles (instead of x/y coordinates).
To solve the problem we search for a path from start conf. to goal conf. in the c-space.

![c_space.png](example/c_space.png)

The path tells us how to change the angles of the arm over time to reach the goal position. As animation:

![path.gif](example/path.gif)

## Path finding algorithms


| Single Query (variant) | Bug 0 | Bug 2 |
| ---------------------- | ----- | ----- |
| ![poly_sq.png](example/poly_sq.png) | ![poly_bug0.png](example/poly_bug0.png) | ![poly_bug2.png](example/poly_bug2.png) |
| Explore space and build a graph by randomly expanding nodes | Go around obstacles and leave as soon as possible | Go around obstacles and leave when crossing the line from start to goal again | 


| Visibility graph | | Voronoi |
| ---------------- | --- | ------- |
| ![poly_vis2.png](example/poly_vis2.png) | ![poly_vis1.png](example/poly_vis1.png) | ![poly_vor2.png](example/poly_vor2.png) |
| Build a graph by connecting all vertices that see each other (edges of polygons count as well) |  | Use a voronoi diagram and move along the space that is furthest away from obstacles | 

| Potential function | |  |
| ---------------- | --- | --- |
| ![potfield2.png](example/potfield2.png) | ![poly_pot.png](example/poly_pot.png) |  |
| Create potential functions: One that pushes towards the goal, and one that pushes away from obstacles | The potential functions as heatmap | | 
