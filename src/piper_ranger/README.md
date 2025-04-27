松灵小车启动步骤
编译
```
colcon build --symlink-install
sudo modprobe gs_usb
```

当第一次运行时
```
sudo bash src/ranger_ros2/ranger_bringup/scripts/setup_can2usb.bash
```
之后每次重新通电后
```
sudo bash src/ranger_ros2/ranger_bringup/scripts/bringup_can2usb.bash
```
之后can口启动，可以通过candump can0来测试

(connect can)
(power on)
启动前准备
```
source /opt/ros/humble/setup.sh

source install/setup.sh

sudo bash src/ranger_ros2/ranger_bringup/scripts/bringup_can2usb.bash
```

运行下面的命令就可以控制小车了，可以发送cmd_val的ros消息来让小车移动，同时订阅名
为odom的消息就能拿到odom格式的里程计数据
```
ros2 launch ranger_bringup ranger_mini_v2.launch.xml
```
观察启动后的界面，可以修改参数，如果需要坐标转换的话，可以用下面的命令启动，之后i
就可以通过tf转换得到odom转换的base_link坐标
```
ros2 launch ranger_bringup ranger_mini_v2.launch.xml
publish_odom_tf:=true
```
可以通过下面的命令启动键盘遥控
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
