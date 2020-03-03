# Package `srrg2_qgl_viewport_ros`
QGLViewer-based implementation of a generic viewport that works via ROS messages (in a separate process). The major features provided are:

* clear separation between processing executable (e.g. SLAM thread) and viewing executable - that can run at different framerates
* rendering multiple viewports at the same time without writing any code
* benchmark mode triggered when all viewports are closed (no read/writes on the shared memory in this case)

## How to build
All our software is tested both with Ubuntu 18.04 and 16.04 (GCC 5 and 7), still the remaining of this guide refers to Ubuntu 18.04.

1. create a Catkin workspace
2. clone and build the packages
[`srrg2_core`](https://github.com/srrg-sapienza/srrg2_core/srrg2_core),
[`srrg2_core_ros`](https://github.com/srrg-sapienza/srrg2_core/srrg2_core_ros)
[`srrg2_qgl_viewport`](https://github.com/srrg-sapienza/srrg2_qgl_viewport/srrg2_qgl_viewport)
3. build this package using Catkin
```bash
cd <your_catkin_ws>
catkin build srrg2_qgl_viewport_ros
```

## How to use
In this design there are 2 different processes that should work together: a `server` - which streams data over the network via ROS messages - and a `client` - that receives the messages and renders them on a viewport.
The `server` represents the SLAM process, that could run on a cheap system (e.g. on the robot); the `client` process instead should run on a powerful machine that supports OpenGL rendering.

Therefore as example of usage we refer to:
* [`test_viewer_core_server`](src/tests/test_viewer_core_server.cpp) for the server process
* [`srrg_viewer_ros_client`](src/app/srrg_viewer_ros_client.cpp) for the client process.

Note that **no other client process should be coded**, since our `srrg_viewer_ros_client` app can be used with every `server` process. If you want to spawn multiple rendering viewport, just run another instance of the `srrg_viewer_ros_client` app :).
