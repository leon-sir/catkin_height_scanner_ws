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

## 可选直接安装所有模块（照1005环境配的）
git clone --recurse-submodules https://github.com/leon-sir/catkin_height_scanner_ws.git


安装依赖
sudo apt-get install libpcl-dev
sudo apt-get install libeigen3-dev
sudo apt-get install ros-$ROS_DISTRO-grid-map
其他依赖
sudo apt-get install ros-$ROS_DISTRO-eigen-conversions

## 逐个安葬并修改
cd <your-ros-ws>/src
git clone https://github.com/anybotics/kindr.git
git clone git@github.com:anybotics/kindr.git

git clone https://github.com/anybotics/kindr_ros.git
git clone git@github.com:anybotics/kindr_ros.git

git clone https://github.com/anybotics/elevation_mapping.git
git clone git@github.com:anybotics/elevation_mapping.git

git clone https://github.com/ANYbotics/message_logger.git
git clone git@github.com:ANYbotics/message_logger.git

git clone https://github.com/ros/geometry.git
git clone git@github.com:ros/geometry.git

git clone https://github.com/anybotics/point_cloud_io.git (可选，用来可视化的)
git clone git@github.com:anybotics/point_cloud_io.git

git clone https://github.com/Livox-SDK/livox_ros_driver2.git
git clone git@github.com:Livox-SDK/livox_ros_driver2.git


<!-- cd ..
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build kindr_msgs kindr_ros kindr_rviz_plugins multi_dof_joint_trajectory_rviz_plugins message_logger geometry 
catkin build elevation_mapping -->

# 还要装个lidar ros驱动
# git clone https://github.com/Livox-SDK/livox_ros_driver2.git
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

```bash
roslaunch livox_ros_driver2 rviz_MID360.launch
```

roslaunch elevation_mapping_demos ground_truth_demo.launch
roslaunch elevation_mapping_demos mid360_elevation_mapping.launch


mid360设置：
旋转：MID360_config.json 
		      "extrinsic_parameter" : {
					"roll": 0.0,
					"pitch": 90.0,
					"yaw": 0.0,
					"x": 0,
					"y": 0,
					"z": 0


	或者
	mid360_elevation_mapping.launch 
	     <node pkg="tf" type="static_transform_publisher" 			name="livox_raw_to_frame" output="screen"
          		args="0 0 0 0 0 0 livox_frame livox_raw 100"/>

过滤点云：
mid360_elevation_mapping.launch
    <node pkg="nodelet" type="nodelet" name="passthrough_z" 
          args="load pcl/PassThrough pcl_manager" output="screen">
        <remap from="~input" to="/livox/lidar"/>
        <remap from="~output" to="/livox/lidar_filtered"/>
        <rosparam>
            filter_field_name: z
            filter_limit_min: -1.5
            filter_limit_max: 0.3
            filter_limit_negative: false
            input_frame: livox_frame
            output_frame: livox_frame
        </rosparam>
    </node>
    
延迟：
mid360_robot.yaml:

min_update_rate: 10.0 - 最小更新频率从默认2Hz提高到10Hz
time_tolerance: 0.1 - 时间容差从1秒降到0.1秒
fused_map_publishing_rate: 10.0 - 融合地图发布频率提高到10Hz
long_range.yaml:

resolution: 0.1 - 分辨率从0.05m改为0.1m（减少计算量，如果需要精度可以改回去）
scanning_duration: 0.05 - 添加扫描持续时间参数 

