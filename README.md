# TreeScan3D-Multi-Threaded-Tree-Detection
TreeScan3D is an innovative project that leverages the power of 3D computer vision and multi-threaded processing for efficient tree detection. Leverages ROS, PCL and C++.

<div align="center">
    <img src="assets/DepthMap.gif" alt="Depth Map" width="500"/>
    <p>Depth Map</p>
</div>
<div align="center">
    <img src="assets/PointCloud.gif" alt="Point Cloud" width=500"/>
    <p>Point Cloud</p>
</div>

## ⚠️ Dependencies
1) ROS ([Offical Guide](http://wiki.ros.org/noetic/Installation/Ubuntu))
2) PCL ([Offical Guide](https://pointclouds.org/downloads/))

## ⚙️ To Use
1) `catkin build -DCMAKE_BUILD_TYPE=Release`
2) source workspace or `.bashrc` if already added
3) `rosrun detector detectorV2`

## 🎋 3D Bounding Box with instance labelled Point Cloud
<div align="center">
    <img src="assets/InstanceCloud.gif" alt="Instance Cloud" width="500"/>
    <p>Instance Cloud</p>
</div>
<div align="center">
    <img src="assets/3DBoundingBox.gif" alt="Bounding Box" width=500"/>
    <p>3D Bounding Box</p>
</div>

## 🖼️ Visualization
RVIZ can be used to visualize the Bounding Box and the computed instance cloud
<div align="center">
    <img src="assets/RVIZ.png" alt="RVIZ" width="500"/>
    <p>RVIZ Window</p>
</div>


