# uav_slam_nav

无人机视觉 SLAM 建图 + 自主避障导航。

## 功能

- **SLAM 建图**：基于 slam_toolbox，实时构建 2D 栅格地图
- **点云转 LaserScan**：将世界系 LiDAR 点云逆变换到 base_link，切片成 360° /scan
- **自主避障导航**：基于 /scan 的简单避障算法，前方有障碍自动转向

## 演示

![SLAM 建图 + 自主避障导航](docs/slam_nav_demo.gif)

## 启动

```bash
# 1. 启动 bridge（先开仿真器）
roslaunch carlair_ros_bridge main.launch host:=10.0.2.2 auto_takeoff:=true

# 2. 启动 SLAM + 导航
roslaunch uav_slam_nav slam_nav.launch
```

## 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| /map | nav_msgs/OccupancyGrid | SLAM 构建的栅格地图 |
| /scan | sensor_msgs/LaserScan | 360° 激光扫描 |
| /uav/odom | nav_msgs/Odometry | 无人机里程计 |
| /uav/cmd_vel | geometry_msgs/Twist | 速度控制指令 |
