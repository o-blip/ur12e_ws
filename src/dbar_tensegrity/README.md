# D-bar tensegrity

An importable Python skeleton for a planar (2D) D-bar structure.
The geometry module is independent of ROS; ament packaging allows it to live
in the same colcon workspace as the UR12e motion package.

No geometry, dimensions, connectivity, visualization, winding, or robot
integration is implemented. The next step is to specify the intended D-bar
arrangement and dimensions before implementing geometry.py.
