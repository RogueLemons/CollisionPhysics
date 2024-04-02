# Polygon Physics with C++
This showcases my interest in physics and code optimization. First you get two videos showing the end result of 2D Physics of polygons colliding with each other. The videos showcase the same size and number of polygons as in the .exe files in the Release and Debug folders (which you can run yourself without building).

Looking through the source code you will find:
- Uniform grid space partitioning to handle more polygons than we could ever need for this demo.
- Separating Axis Theorem for discrete collision detection of convex (regular) polygons
- Finding the collision point.
- Physics (impulse, energy, translational velocity, angular velocity, inertia) for resolving collisions.
- Linear Algebra functions to help resolve collision physics.
- Using SFML to draw all polygons in a live window.
