# 高城图重建说明
```bash
ping 192.168.1.173
```
确保usb拓展对应插槽与雷达IP（192.168.1.173）在同一网段
```bash
# 临时修改插口ip（调试用）
sudo ip addr add 192.168.1.5/24 dev enx5c5310f9933e

# 长期(可能有其他网关兼容性问题，需测试)
sudo nmcli con add type ethernet ifname enx5c5310f9933e con-name "Livox-USB" ipv4.addresses 192.168.1.5/24 ipv4.method manual ipv4.never-default yes
ping 192.168.1.173

# 打开
sudo nmcli con up "Livox-USB"
# 关闭
sudo nmcli con delete "Livox-USB"
```

安装教程参考：
1. https://blog.csdn.net/2301_79152004/article/details/146884287
2. https://github.com/ANYbotics/elevation_mapping

其他资料
1. https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md
2. https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Livox_Viewer_2_User_Manual_chs_v1.2.pdf

livox激光雷达SDK安装：
```bash
mkdir -p third_party
cd third_party
git clone https://github.com/Livox-SDK/Livox-SDK2.git

cd third_party/Livox-SDK2
mkdir build && cd build
cmake .. && make -j
```
## test
config 添加
"host_net_info" : [
      {
        "lidar_ip"  : ["192.168.1.173"],
        ...
```bash
cd build/samples/livox_lidar_quick_start 
./livox_lidar_quick_start ../../../samples/livox_lidar_quick_start/mid360_config.json
```

# ros工作目录
# elevation map 安装

安装依赖
sudo apt-get install libpcl-dev
sudo apt-get install libeigen3-dev
sudo apt-get install ros-$ROS_DISTRO-grid-map
其他依赖
sudo apt-get install ros-$ROS_DISTRO-eigen-conversions

cd <your-ros-ws>/src
git clone https://github.com/anybotics/kindr.git
git clone https://github.com/anybotics/kindr_ros.git
git clone https://github.com/anybotics/elevation_mapping.git
git clone https://github.com/ANYbotics/message_logger.git
git clone https://github.com/ros/geometry.git
git clone https://github.com/anybotics/point_cloud_io.git (可选，用来可视化的)



<!-- cd ..
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build kindr_msgs kindr_ros kindr_rviz_plugins multi_dof_joint_trajectory_rviz_plugins message_logger geometry 
catkin build elevation_mapping -->

git submodule add https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2

# 还要装个lidar ros驱动
git clone https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2
cd src/livox_ros_driver2
source /opt/ros/noetic/setup.sh
./build.sh ROS1 

## 可能遇到的兼容性问题及解决方案
***
“./build.sh“文件
pushd `pwd` > /dev/null
if [ $ROS_VERSION = ${VERSION_ROS1} ]; then
    cd ../../
    catkin_make -DROS_EDITION=${VERSION_ROS1}
    ...
    为
    catkin_make_isolated -DROS_EDITION=${VERSION_ROS1}
    ...

再运行一遍./build.sh ROS1 
此时，为未定义build type的包
  <export>
    <build_type condition="$ROS_VERSION == 1">catkin</build_type>
    ...
最后执行
./build.sh ROS1 
***

```bash
source devel_isolated/setup.bash
```

调整激光雷达高度姿态
```bash
roslaunch livox_ros_driver2 rviz_MID360.launch
```
调整 src/livox_ros_driver2/config/MID360_config.json 
"lidar_configs" : [
    {
      "ip" : "192.168.1.173",
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": XXX.0,
        "pitch": XXX.0,
        "yaw": ...

roslaunch elevation_mapping_demos ground_truth_demo.launch







