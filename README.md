# calibration
## 一、驱动安装
### 1.1相机驱动安装
下载usb_cam.zip，删除build和devel，重新catkin_make
修改此路径下`/usb_cam/src/usb_cam/launch`的`usb_cam-test.launch`文件可以修改输入图像大小，像素格式

`usb_cam-test.launch`

```
<launch>
  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
    <param name="video_device" value="/dev/video0" />
    <param name="image_width" value="1280" />                                                  <!修改image_width-->
    <param name="image_height" value="720" />                                                   <!修改image_height-->
    <param name="pixel_format" value="mjpeg" />                                               <!修改pixel_format-->
    <param name="framerate" value="20" />                                                              <!修改帧数-->
    <param name="color_format" value="yuv422p" />
    <param name="camera_frame_id" value="usb_cam" />
    <param name="io_method" value="mmap"/>
  </node>
  <node name="image_view" pkg="image_view" type="image_view" respawn="false" output="screen">
    <remap from="image" to="/usb_cam/image_raw"/>
    <param name="autosize" value="true" />
  </node>
</launch>
```
**启动测试**
回到根目录
```
source devel/setup.bash 
roslaunch usb_cam usb_cam-test.launch
```
### 1.2IMU驱动安装

**安装依赖项**
sudo apt-get install libdw-dev

imu_utils工具还需要ceres和eigen，本人已预装，具体安装步骤可以自行上网查阅。

**安装code_utils**
这个一定要先安装编译，不然会报错
```
cd IMU/src
git clone https://github.com/gaowenliang/code_utils 
cd ..
catkin_make #编译这个空间
```
运行这个步骤会报错，找不到backward.hpp这个头文件，有三种解决方案，本人使用的是第一种方案：

>方案一：
>
>把src/code_utils/CMakeList.txt中，添加路径：include_directories("include/code_utils")
>
>方案二：
>
>把src/code_utils/src/sumpixel_test.cpp中的#include "backward.hpp"改为#include “code_utils/backward.hpp”
>
>方案三：
>
>把src/code_utils/include/backward.hpp文件扔到src/code_utils/src中
>


**安装imu_utils**
```
cd IMU/src
git clone https://github.com/gaowenliang/imu_utils
cd ..
catkin_make #编译imu_utils
```
**启动节点**
```
source devel/setup.bash
sudo chmod 777 /dev/ttyUSB*
roslaunch fdilink_ahrs ahrs_driver.launch 
```
### 1.3雷达驱动
下载rslidar.zip，在`src/rslidar_sdk/config`路径中修改`config.yaml`适配自己的雷达
`config.yaml`中主要修改
```  
     lidar_type: RS32             #LiDAR type - RS16, RS32, RSBP, RSHELIOS, RSHELIOS_16P, RS128, RS80, RS48, RSP128, RSP80, RSP48, 
                                                    #                          RSM1, RSM1_JUMBO, RSM2, RSE1
      msop_port: 2370           #Msop port of lidar
      difop_port: 8310            #Difop port of lidar
```
随后在工作空间目录
```
catkin_make
source devel/setup.bash
roslaunch rslidar_sdk start.launch
```
## 二、联合标定
### 2.1相机IMU联合标定
基本原理使用VINS标定，VINS在没有预先标定数据使首先会进行标定。
复制`VINS-Mono/src/config/euroc/euroc_config_no_extrinsic.yaml`为`my_config_no_extrinsic.yaml`
修改
```
#1.topic名字以及标定数据的输出路径

#common parameters
imu_topic: "/fdi_imu"
image_topic: "/usb_cam/image_raw"
output_path: "/home/lee/VINS-Mono/"

#2.相机内参

#camera calibration
model_type: PINHOLE
camera_name: camera
image_width: 1280
image_height: 720
distortion_parameters:
   k1: -0.343742
   k2: 0.138236
   p1: 0.000245
   p2: 0.000027
projection_parameters:
   fx: 1255.22916
   fy: 1256.29546
   cx: 661.3379
   cy: 352.14235

#3.有无外参 选择无外参

estimate_extrinsic: 2

#4.IMU内参

#imu parameters       The more accurate parameters you provide, the better performance
acc_n: 0.2          # accelerometer measurement noise standard deviation. #0.2
gyr_n: 0.02         # gyroscope measurement noise standard deviation.     #0.05
acc_w: 0.0002         # accelerometer bias random work noise standard deviation.  #0.02
gyr_w: 2.0e-5       # gyroscope bias random work noise standard deviation.     #4.0e-5
g_norm: 9.81007     # gravity magnitude

```
