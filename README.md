

## note
`gz topic -t "/model/x500_mono_cam_0/servo_lidar" -m gz.msgs.Double  -p "data: 0.0"`

### 获取时间戳
经过测试对比，可得出结论，`rclcpp::Clock().now()`无法正确获取仿真时间。所以代码中要获取时间戳时，可调用`get_clock()->now()`和`this->now()`接口。这样可以保证在标志位`use_sim_time`变化时，代码各处使用的时间戳是一致的。

### gazebe graden gz-sim插件查找
https://github1s.com/gazebosim/gz-sim/tree/2ca64d03218e41a8e4c5b78808180ea8ebddcc9f