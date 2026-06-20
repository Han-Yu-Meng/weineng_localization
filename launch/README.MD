1. 启动系统
   分别开四个不同的终端

Step1. 在 ~/robosense_ws/ 文件夹下：
cd ~/robosense_ws/
./startup.sh
启动雷达驱动

Step2. 在 ~/Desktop/weineng_localization/launch/ 文件夹下，
cd ~/Desktop/weineng_localization/launch/
python3 fastlio.py
启动里程计定位

Step3. 在 ~/Desktop/weineng_localization/launch/ 文件夹下，
cd ~/Desktop/weineng_localization/launch/
python3 adapter.py
启动建图程序（启动后，还需要调用建图指令，才会发布 /map 话题）

Step4. 在 ~/Desktop/nav_ws_engine/ 文件夹下
cd ~/Desktop/nav_ws_engine/
source install/setup.bash
ros2 launch finenav_demo stvl_with_mppi.launch.py 
启动导航程序

2. 记录数据，进行建图
   记录雷达原始数据
   ros2 bag record /rslidar_imu_data /rslidar_points
   建图指令
   ros2 service call /start_mapping std_srvs/srv/Trigger {}
   不断建立新地图，发送到 /map 话题。

3. 进行导航
   和仿真环境中的操控方法一样，操控 
   暂时无法在飞书文档外展示此内容

4. 导航检查

5. 遥控器连接小车，检查是否可以用手柄正常操作小车移动，在这个状态下，往相反的状态拨动右肩键，进入导航模式。

6. 导航软件链路：按照上面的流程操作，发送目标点后，终端启动命令
   ros2 topic echo /cmd_vel
   正常情况应有输出
