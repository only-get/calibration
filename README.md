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
**1.3雷达驱动**
