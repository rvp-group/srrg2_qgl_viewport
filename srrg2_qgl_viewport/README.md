# Package `srrg2_qgl_viewport`
QGLViewer-based implementation of a generic viewport that works in shared memory (in a separate thread). The major features provided are:

* clear separation between processing thread (e.g. SLAM thread) and viewing thread - that can run at different framerates
* rendering multiple viewports at the same time
* benchmark mode triggered when all viewports are closed (no read/writes on the shared memory in this case)

## How to build
All our software is tested both with Ubuntu 18.04 and 16.04 (GCC 5 and 7), still the remaining of this guide refers to Ubuntu 18.04.

1. install all the required deb packages
```bash
sudo apt install qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5 freeglut3-dev
```
2. create a Catkin workspace
3. clone the repository [`srrg2_core`](https://github.com/srrg-sapienza/srrg2_core) in `<your_catkin_ws>/src`
4. build this package using Catkin
```bash
cd <your_catkin_ws>
catkin build srrg2_qgl_viewport
```

## How to use
A full example that shows how this viewing system should be used can be found in the executable [test_viewer_core_multicanvas](src/tests/test_viewer_core_multicanvas.cpp).
All the other tests provide examples on the low-level infrastructure of the system.
