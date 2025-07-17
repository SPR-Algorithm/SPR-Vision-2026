#!/bin/bash

cd /home/myoukin22/SPR-Vision-2025
source install/setup.bash

sleep 30

while true; do
    # 启动ros2 bag record，后台运行
    ros2 bag record /debug_img /armor_detector_marker /armor_solver_marker &
    BAG_PID=$!
    # 等待60秒
    sleep 60
    # 发送Ctrl+C信号终止ros2 bag record
    kill -2 $BAG_PID
    # 等待进程完全退出
    wait $BAG_PID
done