# TreeScan3D-Multi-Threaded-Tree-Detection
ParallelTrunk is an innovative project that leverages the power of 3D computer vision and multi-threaded processing for efficient tree detection. Leverages ROS, PCL and C++.

<div align="center">
    <img src="assets/DepthMap.gif" alt="Image 1" width="500"/>
    <p>Depth Map</p>
</div>
<div align="center">
    <img src="assets/PointCloud.gif" alt="Image 2" width=500"/>
    <p>Point Cloud</p>
</div>

## Dependencies
1) ROS ([Offical Guide](http://wiki.ros.org/noetic/Installation/Ubuntu))
2) PCL ([Offical Guide](https://pointclouds.org/downloads/))

## To Use
1) `catkin build -DCMAKE_BUILD_TYPE=Release -j(nproc)`
