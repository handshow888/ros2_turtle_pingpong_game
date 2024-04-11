# 基于ROS2写的一个乒乓球小游戏
A pingpong game written based on ROS2

## 环境 Environment
Ubuntu 20.04  
ROS2 Foxy

## 编译前 Before Build
把所有文件放在工作空间下的src目录里
Put all files in {your_workspace}/src

## 编译 Build
```bash
cd {your_workspace}
colcon build
```

## 使用方法 How to use
see 使用前必看--说明.docx  

### Terminal 1
```bash
cd {your_workspace}
source install/setup.bash
ros2 launch spawn pingpong.launch.py
```
### Terminal 2
```bash
cd {your_workspace}
source install/setup.bash
ros2 run turtle turtle
```

## PS
初学ros2时写的屎山  
I wrote it when I first started learning ROS2
