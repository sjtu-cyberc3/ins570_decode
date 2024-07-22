# 导远电子INS570惯导RS422接口ROS2驱动

## 使用方法：

建议配合使用rclcpp_component以获得最佳运行效率。

指定**串口设备名**，以及rclcpp_component的**container名**即可运行。

```bash
ros2 launch ins570_decode component.launch.yaml rs422_port:=<your serial port name> container_name:=<your components container name>
```

## 消息发布：

消息名|消息类型|消息内容
---|---|---
vel|geometry_msgs::msg::TwistWithCovarianceStamped|北东地(NED)坐标系下的线速度及其协方差<br/>twist中的角速度未赋值
imu|sensor_msgs::msg::Imu|角速度，加速度，四元数，以及方向(orientation)协方差<br/>其中协方差依次对应roll,pitch,yaw三轴，单位是rad/s
fix|sensor_msgs::msg::NavSatFix|经纬度，GPS状态，以及位置协方差<br/>其中协方差依次对应纬度，经度，海拔，单位是m

建议通过设置namespace为消息名增加前缀。例如将namespace设置为/driver/inertial，则消息名/fix变为/driver/inertial/fix。

## 坐标轴定义：

* 导航坐标系为北东地(NED)坐标系，而非ROS2默认的东北天(ENU)坐标系，这一点需要注意。
* 车体坐标系和和惯导设备安装位置有关，默认情况下为x轴朝前，y轴朝右，z轴朝下。
  * 具体建议参考INS570的用户手册，本ROS2驱动代码和手册中保持严格一致，没有加入任何形式的坐标变换。


## 注意事项：

* 代码存在一项关键参数`leap_seconds`，用于指定闰秒的数量。
  * 截止目前为止(2024-07)，一共存在18个闰秒，所以该参数的默认值为18。
  * 如果出现了新的闰秒，请修改该参数。