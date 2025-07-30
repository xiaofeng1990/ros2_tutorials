#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"

class TurtleTF2Broadcaster : public rclcpp::Node
{
public:
    TurtleTF2Broadcaster() : Node("turtle_tf2_broadcaster")
    {
        turtle_name_ = this->declare_parameter("turtle_name", "turtle1");
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        std::ostringstream stream;
        stream << "/" << turtle_name_.c_str() << "/pose";
        std::string topic_name = stream.str();
        std::cout << "topic_name: " << topic_name << std::endl;
        // 创建一个订阅者，订阅海龟的位置消息
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(topic_name, 10, std::bind(&TurtleTF2Broadcaster::turtle_pose_callback, this, std::placeholders::_1));
    }

private:
    // 创建一个处理海龟位置消息的回调函数，将位置消息转变成坐标变换
    void turtle_pose_callback(const turtlesim::msg::Pose::SharedPtr msg) const
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "world";
        t.child_frame_id = turtle_name_;

        t.transform.translation.x = msg->x;
        t.transform.translation.y = msg->y;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string turtle_name_;
};

int main(int argc, char *argv[])
{
    // ROS2 Python接口初始化
    rclcpp::init(argc, argv);
    // 创建ROS2节点对象并进行初始化,循环等待ROS2退出
    rclcpp::spin(std::make_shared<TurtleTF2Broadcaster>());
    // 关闭ROS2 C++接口
    rclcpp::shutdown();
    return 0;
}
