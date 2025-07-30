#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticTFBroadcaster : public rclcpp::Node
{
public:
    explicit StaticTFBroadcaster()
        : Node("static_turtle_tf2_broadcaster")
    {
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Publish static transforms once at startup
        this->make_transforms();
    }

private:
    void make_transforms()
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        // 设置一个坐标变换的源坐标系
        t.header.frame_id = "world";
        t.child_frame_id = "house";

        t.transform.translation.x = 10.0;
        t.transform.translation.y = 5.0;
        t.transform.translation.z = 0.0;
        // 设置坐标变换中的X、Y、Z向的平移
        t.transform.translation.x = 10.0;
        t.transform.translation.y = 5.0;
        t.transform.translation.z = 0.0;

        // 将欧拉角转换为四元数（roll, pitch, yaw）
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);

        // 设置坐标变换中的X、Y、Z向的旋转（四元数）
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // 广播静态坐标变换
        tf_static_broadcaster_->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char *argv[])
{
    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);
    // 创建ROS2节点对象并进行初始化,循环等待ROS2退出
    rclcpp::spin(std::make_shared<StaticTFBroadcaster>());
    // 关闭ROS2 C++接口
    rclcpp::shutdown();
    return 0;
}