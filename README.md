# PointRobotDijkstra

Perform Dijkstra's algorithm and visualize the generated path
from start to goal positions, with obstacles avoided using
semi-algebraic models. Ran with Python3.

This program has three required arguments: the start coordinates,
the goal coordinates, and the name of the output video. Both pairs
of coordinates should follow the form "y,x", including the comma
but excluding the quotes, from the top-left corner of the window.
The name of the video should just be a simple name with no extension
(it will be an MP4 product). Example:

./Runner.py 10,20 100,125 output1
