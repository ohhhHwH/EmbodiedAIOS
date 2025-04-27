小车导航启动
编译
```
colcon build --symlink-install
```

 ## slam启动
1.运行前先装pcl
2.关闭dhcp服务器，分配ip
```
sudo ip addr add 192.168.1.2/24 dev enx00e04c360241
```

```
# unitree_lidar_ros2负责发送PointCloud2消息
source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```
```
#手动进行雷达坐标转换
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 1 --qw 0 --frame-id base_link --child-frame-id unilidar_lidar
```
```
# pointcloud_to_laserscan将PointCloud2消息转换为LaserScan
source install/setup.bash
ros2 launch pointcloud_to_laserscan sample_pointcloud_to_laserscan_launch.py
```

 ```
 #进行plan修复
source install/setup.bash
ros2 launch goal_path_fixer fixers_launch.py
 ```
 
下面是扫描建图的步骤， 如果仅仅需要启动而不需要建图，则不用执行


```
# slam_toolbox负责建图
source install/setup.bash
ros2 launch slam_toolbox online_sync_launch.py
```

## 小车启动
参见src/piper_ranger

## nv2启动

```
#启动rviz
source install/setup.sh
ros2 run rviz2 rviz2 -d  rviz/rviz.config.rviz
```  

```  
# slam_toolbox负责建图  
source install/setup.bash  
ros2 launch slam_toolbox online_sync_launch.py  
```

```
# 运行map_server和amcl系统
source install/setup.sh
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=map/my_map.yaml
```

```
source install/setup.sh
ros2 run nav2_amcl amcl --ros-args --params-file nav2_config/nav2_params.yaml
```
```
# 启动map_server和amcl系统
./activate_map.sh
# ros2 lifecycle set map_server configure
# ros2 lifecycle set map_server activate
# ros2 lifecycle set amcl configure
# ros2 lifecycle set amcl activate
```

```
# 启动Nav2
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false map:=map/my_map.yaml params_file:=nav2_config/nav2_params.yaml
```
