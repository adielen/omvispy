# About
This Python module can be used to visualize mesh objects created with the OpenMesh Python Bindings (http://openmesh.org/). It is based on vispy (http://vispy.org/) and works best with IPython Notebook.

# How to use it
Clone the repository into a directory called omvispy. Make sure that the omvispy directory and the openmesh.so file are in your module search path. Then in IPython Notebook:

```python
%gui qt
from omvispy import *
mesh = TriMesh()
read_mesh(mesh, 'bunny.obj')
render(mesh, 'wireframe')
```

# Camera Controls
Left mouse button: rotate camera
Right mouse button: zoom in on a point of interest
Middle mouse button: reposition the camera
Scroll wheel: zoom in and out