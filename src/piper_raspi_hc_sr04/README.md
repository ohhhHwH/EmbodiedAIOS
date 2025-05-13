# HC-SR04 超声波传感器驱动（树莓派版）

基于 https://github.com/engcang/HC-SR04-UltraSonicSensor-ROS-RaspberryPi 修改，支持多个超声波传感器和ROS2网络通信。

## 硬件连接注意事项

1. 由于树莓派的输入最大为 3.3V，所以 Echo 过来的 5V 电压不能直接接到树莓派（会烧坏树莓派），需要使用电阻分压或用 AMS1117 芯片降压
2. 这里我采用了 https://detail.tmall.com/item.htm?detail_redpacket_pop=true&id=13301101281&ltk2=1747036883275pdkov4mb4hg9td9q8b49kk&ns=1&priceTId=214782fe17470368710688979e1ba4&query=5V%E9%99%8D%E5%8E%8B3.3V&skuId=4173505841581&spm=a21n57.1.hoverItem.3&utparam=%7B%22aplus_abtest%22%3A%227b45729bdea65b47781027bf9e88793e%22%7D&xxc=ad_ztc 的电路板进行 5V DC 到 3.3V 的降压
3. HC-SR04 的供电电压为 5V，接树莓派 5V 供电即可

## 功能特点

- 支持多个 HC-SR04 传感器
- ROS2 集成，发布 Range 消息
- 通过 YAML 文件配置
- 简单易用的 Python API

## 安装

```bash
sudo apt-get install ros-humble-sensor-msgs gcc

conda create -n raspi python=3.10
conda activate raspi
pip install RPi.GPIO

```

```bash
cd ~/ros2_ws
colcon build --packages-select piper_raspi_hc_sr04
```

## 配置

传感器配置通过 YAML 文件管理，位于 `sensors.yaml`。配置示例：

```yaml
sensors:
  sensor1:
    trig: 27
    echo: 17
  sensor2:
    trig: 22
    echo: 23
  sensor3:
    trig: 24
    echo: 25
  sensor4:
    trig: 8
    echo: 7
```

每个传感器配置需要：
- `trig`: 触发引脚号（GPIO BCM编号）
- `echo`: 回响引脚号（GPIO BCM编号）

## 使用方法

### 1. 单独使用（不使用ROS2）

```bash
# 运行测试程序
python3 hc_sr04_driver.py
```

### 2. ROS2 使用

#### 在树莓派上：

1. 启动 ROS2 节点：
```bash
ros2 run piper_raspi_hc_sr04 hc_sr04_node
```

2. 查看数据：
```bash
# 查看所有可用的超声波话题
ros2 topic list | grep ultrasonic

# 查看特定传感器的数据
ros2 topic echo /ultrasonic/sensor1
```

## 消息格式

每个传感器发布 `sensor_msgs/Range` 类型的消息，包含以下信息：
- `header.frame_id`: 传感器名称
- `radiation_type`: 超声波类型
- `field_of_view`: 视场角（约5.7度）
- `min_range`: 最小测量距离（0.02米）
- `max_range`: 最大测量距离（4.0米）
- `range`: 测量距离（米）

wheatfox wheatfox17@icloud.com 2025