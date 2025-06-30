# 中国石油大学（北京）SPR战队 RoboMaster2026赛季 视觉自瞄系统

部分基于rmvision项目，中南大学，深圳大学视觉开源
贡献者&维护者：SPR算法组

# 全流程部署指南

## 1. 安装Ubuntu 22.04 LTS
强烈建议安装时选择Minimal Installation，可以少点没用的东西。

## 2. fishros安装ros2 humble desktop
```
wget http://fishros.com/install -O fishros && . fishros
sudo apt update && sudo apt upgrade
sudo apt-get install ros-humble-image-transport-plugins
sudo apt install ros-humble-asio-cmake-module
sudo apt install ros-humble-foxglove-bridge
sudo apt install ros-humble-serial-driver
```

## 3.使用一键安装脚本安装依赖
```
chmod +x install_from_zips.sh
./install_from_zips.sh
```

## 4. 编译安装CH341驱动并配置串口
```
sudo apt remove brltty
```
![image](https://github.com/user-attachments/assets/c4abf805-2ec8-453b-90ed-23c1549c6840)
下载并按照压缩包内readme配置串口驱动

如果提示没有gcc-12，使用apt安装gcc-12
如果提示insmod: ERROR: could not insert module ch341.ko: Unknown symbol in module

则进行
```
modinfo ch341.ko |grep depends
depends:        usbserial
```
然后
```
sudo modprobe usbserial
```
问题应该解决

### 安装完成后：

验证：lsmod | grep ch34
ch341                  24576  0
usbserial              69632  1 ch341

### 以下为手动安装方法，一键安装脚本遇到问题时，可单独对照进行debug

#### 安装spdlog库（版本1.14）
压缩包解压后cd进去
```
mkdir build && cd build
cmake .. && make -j4
sudo make install
```
cmake之后如下方fmt一样，在CmakeCache.txt里面添加-fPIC选项

#### 安装FMT库（版本10.2.1）
//已完成
修改armor\_detector节点里armor\_detector.cpp的代码，在include里添加#include \<fmt/format.h>

压缩包解压后cd进去

修改CMakeLists.txt，在指定位置添加如下行：
```
#Add -fPIC option
add_compile_options(-fPIC)
```

![](docs/p3XZKold31xFgpCrUgQssCxuooFgjEb0PcBcbobKgNI=.png)

cd进去执行：
```
mkdir build && cd build
cmake ..
make
sudo make install
```
然后编辑build/CMakeCache.txt，在此处添加如下参数

![](docs/BSRyhPmF56mW5Yb9JATeD7Ye1237wWlQ6FqYtxfkzeo=.png)

然后重新在build目录执行：
```
make
sudo make install
```
如此，编译应该通过

#### 安装g2o库（版本20230806_git）
压缩包解压后cd进去
```
mkdir build && cd build
cmake .. && make -j4
sudo make install
```
性能过差卡死解决方案：将make -j改为make -j4或更小的数字，任何时候遇到编译性能问题都可如此尝试

编译时同样需要添加-fPIC选项！

#### 安装Ceres-Solver库（版本2.0.0）（可能2.2.0）
压缩包解压后cd进去
```
mkdir build && cd build
cmake .. && make -j4
sudo make install
```
rosdep提示缺少ceres是正常现象不必理会，确保apt中libceres的版本为2.0.0
若编译中Cmake提示找不到tbb相关文件，则卸载当前的libtbb，并按顺序安装libtbb2，libtbb2-dev，libtbbmalloc2-dev

#### 添加串口&相机的权限规则
```
sudo cp camera.rules  /etc/udev/rules.d/
sudo cp serial.rules  /etc/udev/rules.d/
```
添加后重启生效

## 5.安装OpenVINO
引擎搜索Install OpenVINO，选择介于2022-2024之间的版本，Distribution选择APT方式，并按照官网指示完成安装。

## 6.部署测试SPR-Vision-2025
在src上层运行
```
rosdepc update
rosdepc install --from-paths src --ignore-src -r -y
colcon build --symlink-install --parallel-workers 4
```
成功编译后，按照如下方式启动：
```
source install/setup.bash
ros2 launch rm_bringup bringup.launch_mvtest.py
```

## 7. 启动相机节点与调试环境设置与相机标定
标定板PDF生成网站：
https://calib.io/pages/camera-calibration-pattern-generator
注意打印时一定要避免因打印页面缩放导致的尺寸误差！

使用USB连接相机

在工作文件夹运行

source install/setup.bash

👆记得这个命令每次新建终端都要执行一次

ros2 run mindvision\_camera mindvision\_camera\_node

rqt添加

Plugins->Visualization->Image View

Plugins->Configurations->Dynamic Reconfigure

若没找到话题和节点记得点刷新

### 标定部分

安装
```
sudo apt install ros-humble-camera-calibration
```
然后运行
```
ros2 run camera_calibration cameracalibrator --size 7x10 --square 0.03 image:=/image_raw camera:=/mv_camera
```
注意，标定板的规格与尺寸大小需要根据实际情况做出相应修改!
按照进度条指示完全移动标定板，尽量使进度条变满，差不多后点击Calibrate；
计算完成后点击Save，结果文件位于/tmp/calibrationdata.tar.gz

## 8. 单独启动识别节点调试

ros2 run armor\_detector armor\_detector\_node

rqt选择/armor\_detector节点配置，打开debug选项，可在左侧image view看到/detector/result\_img

调整相机对焦和光圈，使其能识别出装甲板且置信度稳定在100%

## 9.串口协议通信调试
所有的数据包均统一为16位的FixPacket，其中帧头0xFF，帧尾0xFE；
发送给电控格式为：帧头0xFF，开火（1字节），Yaw（4字节），Pitch（4字节），Distance（4字节），留空（1字节），帧尾0xFE
从电控接收格式为：帧头0xFF，颜色（1字节），填充（2字节）Pitch（4字节），Yaw（4字节），帧尾0xFE，留空（3字节）
遇到通信错误导致

## 10.云台-相机描述模型尺寸修改
右手系，相机镜片平面中心与云台转动轴中心的相对位置，根据兵种情况修改xyz

## 11.代码编译部署错误备忘
若遇到类似Something went wrong while looing up transform之类的串口通信问题，按照
检查硬件连接->CuteCom检查串口接收通信工作情况->检查数据校验是否成功
的步骤，依次检查与下位机的通信情况

CMake Error at /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:240 (list):
list index: 1 out of range (-1, 0)
的问题，则表示路径中有非Unicode字符，将工作目录移动至无中文路径中删除build重新编译。

若遇到类似error while loading shared libraries: libg2o_core.so: cannot open shared object file
等g2o库等无法找到的问题，首先确保g2o已按照-fPIC参数正确编译并安装成功，若问题仍然存在，则使用如下命令编辑该文件：
sudo gedit /etc/ld.so.conf
并在文本编辑器中添加如下行：
/usr/local/lib
保存并退出，运行
sudo ldconfig
如此，问题应该解决。
原文链接：https://blog.csdn.net/weixin_38258767/article/details/106875766

## 12.各兵种针对部署
针对实际修改：
各台车的相机内参以及相机-云台变换尺寸
自启动脚本的目录路径
```
SPR-Vision-2026
├─ Main_ws
│  └─ src
│     ├─ auto_aim_interfaces
│     │  ├─ CMakeLists.txt
│     │  ├─ msg
│     │  │  ├─ Armor.msg
│     │  │  ├─ Armors.msg
│     │  │  ├─ DebugArmor.msg
│     │  │  ├─ DebugArmors.msg
│     │  │  ├─ DebugLight.msg
│     │  │  ├─ DebugLights.msg
│     │  │  ├─ Target.msg
│     │  │  └─ TrackerInfo.msg
│     │  └─ package.xml
│     ├─ rm_auto_aim
│     │  ├─ README.md
│     │  ├─ armor_detector
│     │  │  ├─ CMakeLists.txt
│     │  │  ├─ README.md
│     │  │  ├─ cmake_modules
│     │  │  │  └─ FindG2O.cmake
│     │  │  ├─ docs
│     │  │  │  ├─ BA.png
│     │  │  │  ├─ blue.png
│     │  │  │  ├─ classify.png
│     │  │  │  ├─ gray_bin.png
│     │  │  │  ├─ hsv_bin.png
│     │  │  │  ├─ model.svg
│     │  │  │  ├─ num_bin.png
│     │  │  │  ├─ num_raw.png
│     │  │  │  ├─ num_roi.png
│     │  │  │  ├─ num_warp.png
│     │  │  │  ├─ origin1.png
│     │  │  │  ├─ origin2.png
│     │  │  │  ├─ pca1.png
│     │  │  │  ├─ pca2.png
│     │  │  │  ├─ raw.png
│     │  │  │  ├─ red.png
│     │  │  │  └─ test.png
│     │  │  ├─ include
│     │  │  │  └─ armor_detector
│     │  │  │     ├─ armor_detector.hpp
│     │  │  │     ├─ armor_detector_node.hpp
│     │  │  │     ├─ armor_pose_estimator.hpp
│     │  │  │     ├─ ba_solver.hpp
│     │  │  │     ├─ graph_optimizer.hpp
│     │  │  │     ├─ light_corner_corrector.hpp
│     │  │  │     ├─ number_classifier.hpp
│     │  │  │     └─ types.hpp
│     │  │  ├─ model
│     │  │  │  ├─ label.txt
│     │  │  │  ├─ lenet.onnx
│     │  │  │  └─ mlp.onnx
│     │  │  ├─ package.xml
│     │  │  ├─ src
│     │  │  │  ├─ armor_detector.cpp
│     │  │  │  ├─ armor_detector_node.cpp
│     │  │  │  ├─ armor_pose_estimator.cpp
│     │  │  │  ├─ ba_solver.cpp
│     │  │  │  ├─ graph_optimizer.cpp
│     │  │  │  ├─ light_corner_corrector.cpp
│     │  │  │  └─ number_classifier.cpp
│     │  │  └─ test
│     │  │     └─ test_detector.cpp
│     │  ├─ armor_detector_test
│     │  │  ├─ CMakeLists.txt
│     │  │  ├─ README.md
│     │  │  ├─ cmake_modules
│     │  │  │  └─ FindG2O.cmake
│     │  │  ├─ docs
│     │  │  │  ├─ BA.png
│     │  │  │  ├─ blue.png
│     │  │  │  ├─ classify.png
│     │  │  │  ├─ gray_bin.png
│     │  │  │  ├─ hsv_bin.png
│     │  │  │  ├─ model.svg
│     │  │  │  ├─ num_bin.png
│     │  │  │  ├─ num_raw.png
│     │  │  │  ├─ num_roi.png
│     │  │  │  ├─ num_warp.png
│     │  │  │  ├─ origin1.png
│     │  │  │  ├─ origin2.png
│     │  │  │  ├─ pca1.png
│     │  │  │  ├─ pca2.png
│     │  │  │  ├─ raw.png
│     │  │  │  ├─ red.png
│     │  │  │  └─ test.png
│     │  │  ├─ include
│     │  │  │  └─ armor_detector
│     │  │  │     ├─ armor_detector.hpp
│     │  │  │     ├─ armor_detector_node.hpp
│     │  │  │     ├─ armor_pose_estimator.hpp
│     │  │  │     ├─ ba_solver.hpp
│     │  │  │     ├─ graph_optimizer.hpp
│     │  │  │     ├─ light_corner_corrector.hpp
│     │  │  │     └─ types.hpp
│     │  │  ├─ model
│     │  │  │  ├─ label.txt
│     │  │  │  ├─ lenet.onnx
│     │  │  │  └─ mlp.onnx
│     │  │  ├─ package.xml
│     │  │  ├─ src
│     │  │  │  ├─ armor_detector.cpp
│     │  │  │  ├─ armor_detector_node.cpp
│     │  │  │  ├─ armor_pose_estimator.cpp
│     │  │  │  ├─ ba_solver.cpp
│     │  │  │  ├─ graph_optimizer.cpp
│     │  │  │  └─ light_corner_corrector.cpp
│     │  │  └─ test
│     │  │     └─ test_detector.cpp
│     │  ├─ armor_solver
│     │  │  ├─ CMakeLists.txt
│     │  │  ├─ README.md
│     │  │  ├─ docs
│     │  │  │  └─ Kalman_filter_model.png
│     │  │  ├─ include
│     │  │  │  └─ armor_solver
│     │  │  │     ├─ armor_solver.hpp
│     │  │  │     ├─ armor_solver_node.hpp
│     │  │  │     ├─ armor_tracker.hpp
│     │  │  │     └─ motion_model.hpp
│     │  │  ├─ package.xml
│     │  │  └─ src
│     │  │     ├─ armor_solver.cpp
│     │  │     ├─ armor_solver_node.cpp
│     │  │     └─ armor_tracker.cpp
│     │  ├─ openvino_armor_detector
│     │  │  ├─ CMakeLists.txt
│     │  │  ├─ LICENSE
│     │  │  ├─ cmake_modules
│     │  │  │  └─ FindG2O.cmake
│     │  │  ├─ config
│     │  │  │  └─ armor_detector.yaml
│     │  │  ├─ include
│     │  │  │  └─ openvino_armor_detector
│     │  │  │     ├─ armor_pose_estimator.hpp
│     │  │  │     ├─ ba_solver.hpp
│     │  │  │     ├─ graph_optimizer.hpp
│     │  │  │     ├─ mono_measure_tool.hpp
│     │  │  │     ├─ openvino_detect_node.hpp
│     │  │  │     ├─ openvino_detector.hpp
│     │  │  │     └─ types.hpp
│     │  │  ├─ launch
│     │  │  │  └─ armor_detector.launch.py
│     │  │  ├─ model
│     │  │  │  ├─ 0708.bin
│     │  │  │  ├─ 0708.onnx
│     │  │  │  └─ 0708.xml
│     │  │  ├─ package.xml
│     │  │  └─ src
│     │  │     ├─ armor_pose_estimator.cpp
│     │  │     ├─ ba_solver.cpp
│     │  │     ├─ graph_optimizer.cpp
│     │  │     ├─ mono_measure_tool.cpp
│     │  │     ├─ openvino_detect_node.cpp
│     │  │     └─ openvino_detector.cpp
│     │  └─ rm_auto_aim
│     │     ├─ CMakeLists.txt
│     │     └─ package.xml
│     ├─ rm_bringup
│     │  ├─ CMakeLists.txt
│     │  ├─ config
│     │  │  ├─ camera_info.yaml
│     │  │  ├─ camera_infonewdame.yaml
│     │  │  ├─ camera_params.yaml
│     │  │  ├─ launch_params.yaml
│     │  │  ├─ node_params
│     │  │  │  ├─ armor_detector_params.yaml
│     │  │  │  ├─ armor_solver_params.yaml
│     │  │  │  ├─ armor_solver_params——heroold.yaml
│     │  │  │  ├─ camera_driver_params.yaml
│     │  │  │  ├─ rune_detector_params.yaml
│     │  │  │  ├─ rune_solver_params.yaml
│     │  │  │  ├─ serial_driver_params.yaml
│     │  │  │  ├─ video_player_params.yaml
│     │  │  │  └─ virtual_serial_params.yaml
│     │  │  ├─ old2camera_info.yaml
│     │  │  ├─ old_camera_info.yaml
│     │  │  └─ oldcamera_info3.yaml
│     │  ├─ launch
│     │  │  ├─ bringup.launch.py
│     │  │  ├─ bringup.launch_hiktest_openvino.py
│     │  │  ├─ bringup.launch_mvtest.py
│     │  │  ├─ bringup.launch_mvtest_nonumber.py
│     │  │  ├─ bringup.launch_mvtest_openvino.py
│     │  │  └─ bringup_navigation.launch.py
│     │  └─ package.xml
│     ├─ rm_hardware_driver
│     │  ├─ rm_camera_driver
│     │  │  ├─ CMakeLists.txt
│     │  │  ├─ README.md
│     │  │  ├─ include
│     │  │  │  ├─ daheng
│     │  │  │  │  ├─ DxImageProc.h
│     │  │  │  │  └─ GxIAPI.h
│     │  │  │  └─ rm_camera_driver
│     │  │  │     ├─ daheng_camera.hpp
│     │  │  │     └─ recorder.hpp
│     │  │  ├─ lib
│     │  │  │  └─ x86_64
│     │  │  │     ├─ GxGVTL.cti
│     │  │  │     ├─ GxU3VTL.cti
│     │  │  │     ├─ libgxiapi.so
│     │  │  │     └─ libgxiapi.so.1.0.1904.8241
│     │  │  ├─ package.xml
│     │  │  ├─ src
│     │  │  │  ├─ daheng_camera.cpp
│     │  │  │  ├─ recorder.cpp
│     │  │  │  └─ video_player.cpp
│     │  │  └─ test
│     │  │     └─ TODO
│     │  ├─ rm_serial_driver
│     │  │  ├─ CMakeLists.txt
│     │  │  ├─ README.md
│     │  │  ├─ include
│     │  │  │  └─ rm_serial_driver
│     │  │  │     ├─ fixed_packet.hpp
│     │  │  │     ├─ fixed_packet_tool.hpp
│     │  │  │     ├─ protocol
│     │  │  │     │  ├─ default_protocol.hpp
│     │  │  │     │  ├─ infantry_protocol.hpp
│     │  │  │     │  ├─ sentry_protocol.hpp
│     │  │  │     │  └─ test_protocol.hpp
│     │  │  │     ├─ protocol.hpp
│     │  │  │     ├─ protocol_factory.hpp
│     │  │  │     ├─ serial_driver_node.hpp
│     │  │  │     ├─ transporter_interface.hpp
│     │  │  │     └─ uart_transporter.hpp
│     │  │  ├─ package.xml
│     │  │  ├─ src
│     │  │  │  ├─ protocol
│     │  │  │  │  ├─ default_protocol.cpp
│     │  │  │  │  ├─ infantry_protocol.cpp
│     │  │  │  │  ├─ sentry_protocol.cpp
│     │  │  │  │  └─ test_protocol.cpp
│     │  │  │  ├─ serial_driver_node.cpp
│     │  │  │  ├─ transporter_driver
│     │  │  │  │  └─ uart_transporter.cpp
│     │  │  │  └─ virtual_serial_node.cpp
│     │  │  └─ test
│     │  │     ├─ dummy_transporter.hpp
│     │  │     └─ test_fixed_packet_tool.cpp
│     │  ├─ ros2-hik-camera
│     │  │  ├─ .clang-format
│     │  │  ├─ .clang-tidy
│     │  │  ├─ CMakeLists.txt
│     │  │  ├─ README.md
│     │  │  ├─ config
│     │  │  │  ├─ camera_info.yaml
│     │  │  │  └─ camera_params.yaml
│     │  │  ├─ hikSDK
│     │  │  │  ├─ include
│     │  │  │  │  ├─ CameraParams.h
│     │  │  │  │  ├─ MvCameraControl.h
│     │  │  │  │  ├─ MvErrorDefine.h
│     │  │  │  │  ├─ MvISPErrorDefine.h
│     │  │  │  │  └─ PixelType.h
│     │  │  │  └─ lib
│     │  │  │     ├─ amd64
│     │  │  │     │  ├─ libFormatConversion.so
│     │  │  │     │  ├─ libMVRender.so
│     │  │  │     │  ├─ libMediaProcess.so
│     │  │  │     │  ├─ libMvCameraControl.so
│     │  │  │     │  └─ libMvUsb3vTL.so
│     │  │  │     └─ arm64
│     │  │  │        ├─ libFormatConversion.so
│     │  │  │        ├─ libMVRender.so
│     │  │  │        ├─ libMediaProcess.so
│     │  │  │        ├─ libMvCameraControl.so
│     │  │  │        └─ libMvUsb3vTL.so
│     │  │  ├─ launch
│     │  │  │  └─ hik_camera.launch.py
│     │  │  ├─ package.xml
│     │  │  └─ src
│     │  │     └─ hik_camera_node.cpp
│     │  └─ ros2-mindvision-camera
│     │     ├─ .clang-format
│     │     ├─ .clang-tidy
│     │     ├─ CMakeLists.txt
│     │     ├─ LICENSE
│     │     ├─ README.md
│     │     ├─ config
│     │     │  ├─ camera_info.yaml
│     │     │  └─ camera_params.yaml
│     │     ├─ docs
│     │     │  └─ rqt.png
│     │     ├─ launch
│     │     │  └─ mv_launch.py
│     │     ├─ mvsdk
│     │     │  ├─ include
│     │     │  │  ├─ CameraApi.h
│     │     │  │  ├─ CameraDefine.h
│     │     │  │  └─ CameraStatus.h
│     │     │  └─ lib
│     │     │     ├─ amd64
│     │     │     │  └─ libMVSDK.so
│     │     │     └─ arm64
│     │     │        └─ libMVSDK.so
│     │     ├─ package.xml
│     │     └─ src
│     │        └─ mv_camera_node.cpp
│     ├─ rm_interfaces
│     │  ├─ CMakeLists.txt
│     │  ├─ msg
│     │  │  ├─ Armor.msg
│     │  │  ├─ Armors.msg
│     │  │  ├─ ChassisCmd.msg
│     │  │  ├─ DebugArmor.msg
│     │  │  ├─ DebugArmors.msg
│     │  │  ├─ DebugLight.msg
│     │  │  ├─ DebugLights.msg
│     │  │  ├─ DebugRuneAngle.msg
│     │  │  ├─ GimbalCmd.msg
│     │  │  ├─ JudgeSystemData.msg
│     │  │  ├─ Measurement.msg
│     │  │  ├─ OperatorCommand.msg
│     │  │  ├─ Point2d.msg
│     │  │  ├─ RuneTarget.msg
│     │  │  ├─ SerialReceiveData.msg
│     │  │  └─ Target.msg
│     │  ├─ package.xml
│     │  └─ srv
│     │     └─ SetMode.srv
│     ├─ rm_robot_description
│     │  ├─ CMakeLists.txt
│     │  ├─ LICENSE
│     │  ├─ README.md
│     │  ├─ docs
│     │  │  └─ rm_vision.svg
│     │  ├─ package.xml
│     │  └─ urdf
│     │     ├─ rm_gimbal.urdf.xacro
│     │     └─ sentry.urdf.xacro
│     ├─ rm_rune
│     │  ├─ README.md
│     │  ├─ rm_rune
│     │  │  ├─ CMakeLists.txt
│     │  │  └─ package.xml
│     │  ├─ rune_detector
│     │  │  ├─ CMakeLists.txt
│     │  │  ├─ README.md
│     │  │  ├─ docs
│     │  │  │  └─ test.png
│     │  │  ├─ include
│     │  │  │  └─ rune_detector
│     │  │  │     ├─ rune_detector.hpp
│     │  │  │     ├─ rune_detector_node.hpp
│     │  │  │     └─ types.hpp
│     │  │  ├─ model
│     │  │  │  ├─ yolox_rune.bin
│     │  │  │  ├─ yolox_rune.onnx
│     │  │  │  ├─ yolox_rune.xml
│     │  │  │  ├─ yolox_rune_3.6m.bin
│     │  │  │  ├─ yolox_rune_3.6m.onnx
│     │  │  │  └─ yolox_rune_3.6m.xml
│     │  │  ├─ package.xml
│     │  │  ├─ src
│     │  │  │  ├─ rune_detector.cpp
│     │  │  │  └─ rune_detector_node.cpp
│     │  │  └─ test
│     │  │     ├─ test_detector.cpp
│     │  │     └─ test_node_startup.cpp
│     │  └─ rune_solver
│     │     ├─ CMakeLists.txt
│     │     ├─ README.md
│     │     ├─ include
│     │     │  └─ rune_solver
│     │     │     ├─ curve_fitter.hpp
│     │     │     ├─ motion_model.hpp
│     │     │     ├─ rune_solver.hpp
│     │     │     ├─ rune_solver_node.hpp
│     │     │     └─ types.hpp
│     │     ├─ package.xml
│     │     ├─ src
│     │     │  ├─ curve_fitter.cpp
│     │     │  ├─ rune_solver.cpp
│     │     │  └─ rune_solver_node.cpp
│     │     └─ test
│     │        └─ test_node_startup.cpp
│     ├─ rm_upstart
│     │  ├─ README.md
│     │  ├─ register_service.sh
│     │  ├─ rm_clean_up.sh
│     │  └─ rm_watch_dog.sh
│     ├─ rm_utils
│     │  ├─ CMakeLists.txt
│     │  ├─ README.md
│     │  ├─ include
│     │  │  └─ rm_utils
│     │  │     ├─ assert.hpp
│     │  │     ├─ common.hpp
│     │  │     ├─ heartbeat.hpp
│     │  │     ├─ logger
│     │  │     │  ├─ README.md
│     │  │     │  ├─ exception.hpp
│     │  │     │  ├─ impl
│     │  │     │  │  ├─ global_mutex.hpp
│     │  │     │  │  ├─ logger_impl.hpp
│     │  │     │  │  └─ writer.hpp
│     │  │     │  ├─ log.hpp
│     │  │     │  ├─ logger_pool.hpp
│     │  │     │  └─ types.hpp
│     │  │     ├─ math
│     │  │     │  ├─ extended_kalman_filter.hpp
│     │  │     │  ├─ manual_compensator.hpp
│     │  │     │  ├─ particle_filter.hpp
│     │  │     │  ├─ pnp_solver.hpp
│     │  │     │  ├─ trajectory_compensator.hpp
│     │  │     │  └─ utils.hpp
│     │  │     └─ url_resolver.hpp
│     │  ├─ package.xml
│     │  └─ src
│     │     ├─ heartbeat.cpp
│     │     ├─ logger
│     │     │  ├─ logger_impl.cpp
│     │     │  ├─ logger_pool.cpp
│     │     │  └─ writer.cpp
│     │     ├─ math
│     │     │  ├─ extended_kalman_filter.cpp
│     │     │  ├─ manual_compensator.cpp
│     │     │  ├─ particle_filter.cpp
│     │     │  ├─ pnp_solver.cpp
│     │     │  ├─ trajectory_compensator.cpp
│     │     │  └─ utils.cpp
│     │     └─ url_resolver.cpp
│     ├─ rmoss_core
│     │  ├─ CMakeLists.txt
│     │  └─ package.xml
│     ├─ rmoss_interfaces
│     │  ├─ CMakeLists.txt
│     │  ├─ LICENSE
│     │  ├─ README.md
│     │  ├─ msg
│     │  │  ├─ ChassisCmd.msg
│     │  │  ├─ Gimbal.msg
│     │  │  ├─ GimbalCmd.msg
│     │  │  ├─ ShootCmd.msg
│     │  │  └─ referee
│     │  │     ├─ GameStatus.msg
│     │  │     ├─ RefereeCmd.msg
│     │  │     └─ RobotStatus.msg
│     │  ├─ package.xml
│     │  └─ srv
│     │     ├─ ControlTask.srv
│     │     ├─ GetCameraInfo.srv
│     │     ├─ GetMode.srv
│     │     ├─ GetTaskStatus.srv
│     │     ├─ SetColor.srv
│     │     └─ SetMode.srv
│     ├─ rmoss_projectile_motion
│     │  ├─ CMakeLists.txt
│     │  ├─ README.md
│     │  ├─ include
│     │  │  └─ rmoss_projectile_motion
│     │  │     ├─ gaf_projectile_solver.hpp
│     │  │     ├─ gimbal_transform_tool.hpp
│     │  │     ├─ gravity_projectile_solver.hpp
│     │  │     ├─ iterative_projectile_tool.hpp
│     │  │     └─ projectile_solver_interface.hpp
│     │  ├─ package.xml
│     │  └─ src
│     │     ├─ gaf_projectile_solver.cpp
│     │     ├─ gimbal_transform_tool.cpp
│     │     ├─ gravity_projectile_solver.cpp
│     │     └─ iterative_projectile_tool.cpp
│     └─ rmoss_util
│        ├─ CMakeLists.txt
│        ├─ README.md
│        ├─ include
│        │  └─ rmoss_util
│        │     ├─ debug.hpp
│        │     ├─ image_utils.hpp
│        │     ├─ mono_measure_tool.hpp
│        │     ├─ task_manager.hpp
│        │     ├─ time_utils.hpp
│        │     └─ url_resolver.hpp
│        ├─ package.xml
│        ├─ src
│        │  ├─ debug.cpp
│        │  ├─ image_utils.cpp
│        │  ├─ mono_measure_tool.cpp
│        │  ├─ task_manager.cpp
│        │  ├─ time_utils.cpp
│        │  └─ url_resolver.cpp
│        └─ test
│           ├─ CMakeLists.txt
│           └─ test_url_resolve.cpp
├─ README.md
├─ Utils
│  ├─ CH341SER_LINUX.ZIP
│  ├─ FindTBB_new.cmake
│  ├─ Sophus-1.22.10.zip
│  ├─ ceres-solver-2.0.0.zip
│  ├─ fmt-10.2.1.zip
│  ├─ rules
│  │  ├─ (Deprecated)ttyusb.rules
│  │  ├─ camera.rules
│  │  └─ serial.rules
│  ├─ spdlog-1.14.0.zip
│  ├─ start
│  │  ├─ autoaim_begin.sh
│  │  ├─ autoaim_begin1.sh
│  │  ├─ record.sh
│  │  └─ 需要添加到Gnome开机自启动配置里面的命令
│  └─ z_g2o-20230806_git.zip
└─ install_and_configure.sh

```
```
SPR-Vision-2026
├─ Main_ws
│  └─ src
│     ├─ auto_aim_interfaces
│     │  ├─ CMakeLists.txt
│     │  ├─ msg
│     │  │  ├─ Armor.msg
│     │  │  ├─ Armors.msg
│     │  │  ├─ DebugArmor.msg
│     │  │  ├─ DebugArmors.msg
│     │  │  ├─ DebugLight.msg
│     │  │  ├─ DebugLights.msg
│     │  │  ├─ Target.msg
│     │  │  └─ TrackerInfo.msg
│     │  └─ package.xml
│     ├─ rm_auto_aim
│     │  ├─ README.md
│     │  ├─ armor_detector
│     │  │  ├─ CMakeLists.txt
│     │  │  ├─ README.md
│     │  │  ├─ cmake_modules
│     │  │  │  └─ FindG2O.cmake
│     │  │  ├─ docs
│     │  │  │  ├─ BA.png
│     │  │  │  ├─ blue.png
│     │  │  │  ├─ classify.png
│     │  │  │  ├─ gray_bin.png
│     │  │  │  ├─ hsv_bin.png
│     │  │  │  ├─ model.svg
│     │  │  │  ├─ num_bin.png
│     │  │  │  ├─ num_raw.png
│     │  │  │  ├─ num_roi.png
│     │  │  │  ├─ num_warp.png
│     │  │  │  ├─ origin1.png
│     │  │  │  ├─ origin2.png
│     │  │  │  ├─ pca1.png
│     │  │  │  ├─ pca2.png
│     │  │  │  ├─ raw.png
│     │  │  │  ├─ red.png
│     │  │  │  └─ test.png
│     │  │  ├─ include
│     │  │  │  └─ armor_detector
│     │  │  │     ├─ armor_detector.hpp
│     │  │  │     ├─ armor_detector_node.hpp
│     │  │  │     ├─ armor_pose_estimator.hpp
│     │  │  │     ├─ ba_solver.hpp
│     │  │  │     ├─ graph_optimizer.hpp
│     │  │  │     ├─ light_corner_corrector.hpp
│     │  │  │     ├─ number_classifier.hpp
│     │  │  │     └─ types.hpp
│     │  │  ├─ model
│     │  │  │  ├─ label.txt
│     │  │  │  ├─ lenet.onnx
│     │  │  │  └─ mlp.onnx
│     │  │  ├─ package.xml
│     │  │  ├─ src
│     │  │  │  ├─ armor_detector.cpp
│     │  │  │  ├─ armor_detector_node.cpp
│     │  │  │  ├─ armor_pose_estimator.cpp
│     │  │  │  ├─ ba_solver.cpp
│     │  │  │  ├─ graph_optimizer.cpp
│     │  │  │  ├─ light_corner_corrector.cpp
│     │  │  │  └─ number_classifier.cpp
│     │  │  └─ test
│     │  │     └─ test_detector.cpp
│     │  ├─ armor_detector_test
│     │  │  ├─ CMakeLists.txt
│     │  │  ├─ README.md
│     │  │  ├─ cmake_modules
│     │  │  │  └─ FindG2O.cmake
│     │  │  ├─ docs
│     │  │  │  ├─ BA.png
│     │  │  │  ├─ blue.png
│     │  │  │  ├─ classify.png
│     │  │  │  ├─ gray_bin.png
│     │  │  │  ├─ hsv_bin.png
│     │  │  │  ├─ model.svg
│     │  │  │  ├─ num_bin.png
│     │  │  │  ├─ num_raw.png
│     │  │  │  ├─ num_roi.png
│     │  │  │  ├─ num_warp.png
│     │  │  │  ├─ origin1.png
│     │  │  │  ├─ origin2.png
│     │  │  │  ├─ pca1.png
│     │  │  │  ├─ pca2.png
│     │  │  │  ├─ raw.png
│     │  │  │  ├─ red.png
│     │  │  │  └─ test.png
│     │  │  ├─ include
│     │  │  │  └─ armor_detector
│     │  │  │     ├─ armor_detector.hpp
│     │  │  │     ├─ armor_detector_node.hpp
│     │  │  │     ├─ armor_pose_estimator.hpp
│     │  │  │     ├─ ba_solver.hpp
│     │  │  │     ├─ graph_optimizer.hpp
│     │  │  │     ├─ light_corner_corrector.hpp
│     │  │  │     └─ types.hpp
│     │  │  ├─ model
│     │  │  │  ├─ label.txt
│     │  │  │  ├─ lenet.onnx
│     │  │  │  └─ mlp.onnx
│     │  │  ├─ package.xml
│     │  │  ├─ src
│     │  │  │  ├─ armor_detector.cpp
│     │  │  │  ├─ armor_detector_node.cpp
│     │  │  │  ├─ armor_pose_estimator.cpp
│     │  │  │  ├─ ba_solver.cpp
│     │  │  │  ├─ graph_optimizer.cpp
│     │  │  │  └─ light_corner_corrector.cpp
│     │  │  └─ test
│     │  │     └─ test_detector.cpp
│     │  ├─ armor_solver
│     │  │  ├─ CMakeLists.txt
│     │  │  ├─ README.md
│     │  │  ├─ docs
│     │  │  │  └─ Kalman_filter_model.png
│     │  │  ├─ include
│     │  │  │  └─ armor_solver
│     │  │  │     ├─ armor_solver.hpp
│     │  │  │     ├─ armor_solver_node.hpp
│     │  │  │     ├─ armor_tracker.hpp
│     │  │  │     └─ motion_model.hpp
│     │  │  ├─ package.xml
│     │  │  └─ src
│     │  │     ├─ armor_solver.cpp
│     │  │     ├─ armor_solver_node.cpp
│     │  │     └─ armor_tracker.cpp
│     │  ├─ openvino_armor_detector
│     │  │  ├─ CMakeLists.txt
│     │  │  ├─ LICENSE
│     │  │  ├─ cmake_modules
│     │  │  │  └─ FindG2O.cmake
│     │  │  ├─ config
│     │  │  │  └─ armor_detector.yaml
│     │  │  ├─ include
│     │  │  │  └─ openvino_armor_detector
│     │  │  │     ├─ armor_pose_estimator.hpp
│     │  │  │     ├─ ba_solver.hpp
│     │  │  │     ├─ graph_optimizer.hpp
│     │  │  │     ├─ mono_measure_tool.hpp
│     │  │  │     ├─ openvino_detect_node.hpp
│     │  │  │     ├─ openvino_detector.hpp
│     │  │  │     └─ types.hpp
│     │  │  ├─ launch
│     │  │  │  └─ armor_detector.launch.py
│     │  │  ├─ model
│     │  │  │  ├─ 0708.bin
│     │  │  │  ├─ 0708.onnx
│     │  │  │  └─ 0708.xml
│     │  │  ├─ package.xml
│     │  │  └─ src
│     │  │     ├─ armor_pose_estimator.cpp
│     │  │     ├─ ba_solver.cpp
│     │  │     ├─ graph_optimizer.cpp
│     │  │     ├─ mono_measure_tool.cpp
│     │  │     ├─ openvino_detect_node.cpp
│     │  │     └─ openvino_detector.cpp
│     │  └─ rm_auto_aim
│     │     ├─ CMakeLists.txt
│     │     └─ package.xml
│     ├─ rm_bringup
│     │  ├─ CMakeLists.txt
│     │  ├─ config
│     │  │  ├─ camera_info.yaml
│     │  │  ├─ camera_infonewdame.yaml
│     │  │  ├─ camera_params.yaml
│     │  │  ├─ launch_params.yaml
│     │  │  ├─ node_params
│     │  │  │  ├─ armor_detector_params.yaml
│     │  │  │  ├─ armor_solver_params.yaml
│     │  │  │  ├─ armor_solver_params——heroold.yaml
│     │  │  │  ├─ camera_driver_params.yaml
│     │  │  │  ├─ rune_detector_params.yaml
│     │  │  │  ├─ rune_solver_params.yaml
│     │  │  │  ├─ serial_driver_params.yaml
│     │  │  │  ├─ video_player_params.yaml
│     │  │  │  └─ virtual_serial_params.yaml
│     │  │  ├─ old2camera_info.yaml
│     │  │  ├─ old_camera_info.yaml
│     │  │  └─ oldcamera_info3.yaml
│     │  ├─ launch
│     │  │  ├─ bringup.launch.py
│     │  │  ├─ bringup.launch_hiktest_openvino.py
│     │  │  ├─ bringup.launch_mvtest.py
│     │  │  ├─ bringup.launch_mvtest_nonumber.py
│     │  │  ├─ bringup.launch_mvtest_openvino.py
│     │  │  └─ bringup_navigation.launch.py
│     │  └─ package.xml
│     ├─ rm_hardware_driver
│     │  ├─ rm_camera_driver
│     │  │  ├─ CMakeLists.txt
│     │  │  ├─ README.md
│     │  │  ├─ include
│     │  │  │  ├─ daheng
│     │  │  │  │  ├─ DxImageProc.h
│     │  │  │  │  └─ GxIAPI.h
│     │  │  │  └─ rm_camera_driver
│     │  │  │     ├─ daheng_camera.hpp
│     │  │  │     └─ recorder.hpp
│     │  │  ├─ lib
│     │  │  │  └─ x86_64
│     │  │  │     ├─ GxGVTL.cti
│     │  │  │     ├─ GxU3VTL.cti
│     │  │  │     ├─ libgxiapi.so
│     │  │  │     └─ libgxiapi.so.1.0.1904.8241
│     │  │  ├─ package.xml
│     │  │  ├─ src
│     │  │  │  ├─ daheng_camera.cpp
│     │  │  │  ├─ recorder.cpp
│     │  │  │  └─ video_player.cpp
│     │  │  └─ test
│     │  │     └─ TODO
│     │  ├─ rm_serial_driver
│     │  │  ├─ CMakeLists.txt
│     │  │  ├─ README.md
│     │  │  ├─ include
│     │  │  │  └─ rm_serial_driver
│     │  │  │     ├─ fixed_packet.hpp
│     │  │  │     ├─ fixed_packet_tool.hpp
│     │  │  │     ├─ protocol
│     │  │  │     │  ├─ default_protocol.hpp
│     │  │  │     │  ├─ infantry_protocol.hpp
│     │  │  │     │  ├─ sentry_protocol.hpp
│     │  │  │     │  └─ test_protocol.hpp
│     │  │  │     ├─ protocol.hpp
│     │  │  │     ├─ protocol_factory.hpp
│     │  │  │     ├─ serial_driver_node.hpp
│     │  │  │     ├─ transporter_interface.hpp
│     │  │  │     └─ uart_transporter.hpp
│     │  │  ├─ package.xml
│     │  │  ├─ src
│     │  │  │  ├─ protocol
│     │  │  │  │  ├─ default_protocol.cpp
│     │  │  │  │  ├─ infantry_protocol.cpp
│     │  │  │  │  ├─ sentry_protocol.cpp
│     │  │  │  │  └─ test_protocol.cpp
│     │  │  │  ├─ serial_driver_node.cpp
│     │  │  │  ├─ transporter_driver
│     │  │  │  │  └─ uart_transporter.cpp
│     │  │  │  └─ virtual_serial_node.cpp
│     │  │  └─ test
│     │  │     ├─ dummy_transporter.hpp
│     │  │     └─ test_fixed_packet_tool.cpp
│     │  ├─ ros2-hik-camera
│     │  │  ├─ .clang-format
│     │  │  ├─ .clang-tidy
│     │  │  ├─ CMakeLists.txt
│     │  │  ├─ README.md
│     │  │  ├─ config
│     │  │  │  ├─ camera_info.yaml
│     │  │  │  └─ camera_params.yaml
│     │  │  ├─ hikSDK
│     │  │  │  ├─ include
│     │  │  │  │  ├─ CameraParams.h
│     │  │  │  │  ├─ MvCameraControl.h
│     │  │  │  │  ├─ MvErrorDefine.h
│     │  │  │  │  ├─ MvISPErrorDefine.h
│     │  │  │  │  └─ PixelType.h
│     │  │  │  └─ lib
│     │  │  │     ├─ amd64
│     │  │  │     │  ├─ libFormatConversion.so
│     │  │  │     │  ├─ libMVRender.so
│     │  │  │     │  ├─ libMediaProcess.so
│     │  │  │     │  ├─ libMvCameraControl.so
│     │  │  │     │  └─ libMvUsb3vTL.so
│     │  │  │     └─ arm64
│     │  │  │        ├─ libFormatConversion.so
│     │  │  │        ├─ libMVRender.so
│     │  │  │        ├─ libMediaProcess.so
│     │  │  │        ├─ libMvCameraControl.so
│     │  │  │        └─ libMvUsb3vTL.so
│     │  │  ├─ launch
│     │  │  │  └─ hik_camera.launch.py
│     │  │  ├─ package.xml
│     │  │  └─ src
│     │  │     └─ hik_camera_node.cpp
│     │  └─ ros2-mindvision-camera
│     │     ├─ .clang-format
│     │     ├─ .clang-tidy
│     │     ├─ CMakeLists.txt
│     │     ├─ LICENSE
│     │     ├─ README.md
│     │     ├─ config
│     │     │  ├─ camera_info.yaml
│     │     │  └─ camera_params.yaml
│     │     ├─ docs
│     │     │  └─ rqt.png
│     │     ├─ launch
│     │     │  └─ mv_launch.py
│     │     ├─ mvsdk
│     │     │  ├─ include
│     │     │  │  ├─ CameraApi.h
│     │     │  │  ├─ CameraDefine.h
│     │     │  │  └─ CameraStatus.h
│     │     │  └─ lib
│     │     │     ├─ amd64
│     │     │     │  └─ libMVSDK.so
│     │     │     └─ arm64
│     │     │        └─ libMVSDK.so
│     │     ├─ package.xml
│     │     └─ src
│     │        └─ mv_camera_node.cpp
│     ├─ rm_interfaces
│     │  ├─ CMakeLists.txt
│     │  ├─ msg
│     │  │  ├─ Armor.msg
│     │  │  ├─ Armors.msg
│     │  │  ├─ ChassisCmd.msg
│     │  │  ├─ DebugArmor.msg
│     │  │  ├─ DebugArmors.msg
│     │  │  ├─ DebugLight.msg
│     │  │  ├─ DebugLights.msg
│     │  │  ├─ DebugRuneAngle.msg
│     │  │  ├─ GimbalCmd.msg
│     │  │  ├─ JudgeSystemData.msg
│     │  │  ├─ Measurement.msg
│     │  │  ├─ OperatorCommand.msg
│     │  │  ├─ Point2d.msg
│     │  │  ├─ RuneTarget.msg
│     │  │  ├─ SerialReceiveData.msg
│     │  │  └─ Target.msg
│     │  ├─ package.xml
│     │  └─ srv
│     │     └─ SetMode.srv
│     ├─ rm_robot_description
│     │  ├─ CMakeLists.txt
│     │  ├─ LICENSE
│     │  ├─ README.md
│     │  ├─ docs
│     │  │  └─ rm_vision.svg
│     │  ├─ package.xml
│     │  └─ urdf
│     │     ├─ rm_gimbal.urdf.xacro
│     │     └─ sentry.urdf.xacro
│     ├─ rm_rune
│     │  ├─ README.md
│     │  ├─ rm_rune
│     │  │  ├─ CMakeLists.txt
│     │  │  └─ package.xml
│     │  ├─ rune_detector
│     │  │  ├─ CMakeLists.txt
│     │  │  ├─ README.md
│     │  │  ├─ docs
│     │  │  │  └─ test.png
│     │  │  ├─ include
│     │  │  │  └─ rune_detector
│     │  │  │     ├─ rune_detector.hpp
│     │  │  │     ├─ rune_detector_node.hpp
│     │  │  │     └─ types.hpp
│     │  │  ├─ model
│     │  │  │  ├─ yolox_rune.bin
│     │  │  │  ├─ yolox_rune.onnx
│     │  │  │  ├─ yolox_rune.xml
│     │  │  │  ├─ yolox_rune_3.6m.bin
│     │  │  │  ├─ yolox_rune_3.6m.onnx
│     │  │  │  └─ yolox_rune_3.6m.xml
│     │  │  ├─ package.xml
│     │  │  ├─ src
│     │  │  │  ├─ rune_detector.cpp
│     │  │  │  └─ rune_detector_node.cpp
│     │  │  └─ test
│     │  │     ├─ test_detector.cpp
│     │  │     └─ test_node_startup.cpp
│     │  └─ rune_solver
│     │     ├─ CMakeLists.txt
│     │     ├─ README.md
│     │     ├─ include
│     │     │  └─ rune_solver
│     │     │     ├─ curve_fitter.hpp
│     │     │     ├─ motion_model.hpp
│     │     │     ├─ rune_solver.hpp
│     │     │     ├─ rune_solver_node.hpp
│     │     │     └─ types.hpp
│     │     ├─ package.xml
│     │     ├─ src
│     │     │  ├─ curve_fitter.cpp
│     │     │  ├─ rune_solver.cpp
│     │     │  └─ rune_solver_node.cpp
│     │     └─ test
│     │        └─ test_node_startup.cpp
│     ├─ rm_upstart
│     │  ├─ README.md
│     │  ├─ register_service.sh
│     │  ├─ rm_clean_up.sh
│     │  └─ rm_watch_dog.sh
│     ├─ rm_utils
│     │  ├─ CMakeLists.txt
│     │  ├─ README.md
│     │  ├─ include
│     │  │  └─ rm_utils
│     │  │     ├─ assert.hpp
│     │  │     ├─ common.hpp
│     │  │     ├─ heartbeat.hpp
│     │  │     ├─ logger
│     │  │     │  ├─ README.md
│     │  │     │  ├─ exception.hpp
│     │  │     │  ├─ impl
│     │  │     │  │  ├─ global_mutex.hpp
│     │  │     │  │  ├─ logger_impl.hpp
│     │  │     │  │  └─ writer.hpp
│     │  │     │  ├─ log.hpp
│     │  │     │  ├─ logger_pool.hpp
│     │  │     │  └─ types.hpp
│     │  │     ├─ math
│     │  │     │  ├─ extended_kalman_filter.hpp
│     │  │     │  ├─ manual_compensator.hpp
│     │  │     │  ├─ particle_filter.hpp
│     │  │     │  ├─ pnp_solver.hpp
│     │  │     │  ├─ trajectory_compensator.hpp
│     │  │     │  └─ utils.hpp
│     │  │     └─ url_resolver.hpp
│     │  ├─ package.xml
│     │  └─ src
│     │     ├─ heartbeat.cpp
│     │     ├─ logger
│     │     │  ├─ logger_impl.cpp
│     │     │  ├─ logger_pool.cpp
│     │     │  └─ writer.cpp
│     │     ├─ math
│     │     │  ├─ extended_kalman_filter.cpp
│     │     │  ├─ manual_compensator.cpp
│     │     │  ├─ particle_filter.cpp
│     │     │  ├─ pnp_solver.cpp
│     │     │  ├─ trajectory_compensator.cpp
│     │     │  └─ utils.cpp
│     │     └─ url_resolver.cpp
│     ├─ rmoss_core
│     │  ├─ CMakeLists.txt
│     │  └─ package.xml
│     ├─ rmoss_interfaces
│     │  ├─ CMakeLists.txt
│     │  ├─ LICENSE
│     │  ├─ README.md
│     │  ├─ msg
│     │  │  ├─ ChassisCmd.msg
│     │  │  ├─ Gimbal.msg
│     │  │  ├─ GimbalCmd.msg
│     │  │  ├─ ShootCmd.msg
│     │  │  └─ referee
│     │  │     ├─ GameStatus.msg
│     │  │     ├─ RefereeCmd.msg
│     │  │     └─ RobotStatus.msg
│     │  ├─ package.xml
│     │  └─ srv
│     │     ├─ ControlTask.srv
│     │     ├─ GetCameraInfo.srv
│     │     ├─ GetMode.srv
│     │     ├─ GetTaskStatus.srv
│     │     ├─ SetColor.srv
│     │     └─ SetMode.srv
│     ├─ rmoss_projectile_motion
│     │  ├─ CMakeLists.txt
│     │  ├─ README.md
│     │  ├─ include
│     │  │  └─ rmoss_projectile_motion
│     │  │     ├─ gaf_projectile_solver.hpp
│     │  │     ├─ gimbal_transform_tool.hpp
│     │  │     ├─ gravity_projectile_solver.hpp
│     │  │     ├─ iterative_projectile_tool.hpp
│     │  │     └─ projectile_solver_interface.hpp
│     │  ├─ package.xml
│     │  └─ src
│     │     ├─ gaf_projectile_solver.cpp
│     │     ├─ gimbal_transform_tool.cpp
│     │     ├─ gravity_projectile_solver.cpp
│     │     └─ iterative_projectile_tool.cpp
│     └─ rmoss_util
│        ├─ CMakeLists.txt
│        ├─ README.md
│        ├─ include
│        │  └─ rmoss_util
│        │     ├─ debug.hpp
│        │     ├─ image_utils.hpp
│        │     ├─ mono_measure_tool.hpp
│        │     ├─ task_manager.hpp
│        │     ├─ time_utils.hpp
│        │     └─ url_resolver.hpp
│        ├─ package.xml
│        ├─ src
│        │  ├─ debug.cpp
│        │  ├─ image_utils.cpp
│        │  ├─ mono_measure_tool.cpp
│        │  ├─ task_manager.cpp
│        │  ├─ time_utils.cpp
│        │  └─ url_resolver.cpp
│        └─ test
│           ├─ CMakeLists.txt
│           └─ test_url_resolve.cpp
├─ README.md
├─ Utils
│  ├─ CH341SER_LINUX.ZIP
│  ├─ FindTBB_new.cmake
│  ├─ Sophus-1.22.10.zip
│  ├─ ceres-solver-2.0.0.zip
│  ├─ fmt-10.2.1.zip
│  ├─ rules
│  │  ├─ (Deprecated)ttyusb.rules
│  │  ├─ camera.rules
│  │  └─ serial.rules
│  ├─ spdlog-1.14.0.zip
│  ├─ start
│  │  ├─ autoaim_begin.sh
│  │  ├─ autoaim_begin1.sh
│  │  ├─ record.sh
│  │  └─ 需要添加到Gnome开机自启动配置里面的命令
│  └─ z_g2o-20230806_git.zip
└─ install_and_configure.sh

```