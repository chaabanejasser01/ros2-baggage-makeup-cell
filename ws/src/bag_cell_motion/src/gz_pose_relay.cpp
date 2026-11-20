// Gazebo -> ROS: the true poses of the bags, by name.
//
// The ros_gz bridge turns gz.msgs.Pose_V into tf2_msgs/TFMessage but drops the model names,
// so this small relay does it: it listens to /world/<world>/dynamic_pose/info (gz-transport,
// every moving model) and publishes /cell/bag_poses (child_frame_id = model name, only bag_*)
// at 20 Hz. Used as ground truth by the belt (spacing, camera scoring) and by the cart manager
// (where did the bag land). Much faster than calling the 'gz model' command line each time.
#include <chrono>
#include <mutex>
#include <string>

#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using namespace std::chrono_literals;

class PoseRelay : public rclcpp::Node
{
public:
  PoseRelay()
  : Node("gz_pose_relay")
  {
    const auto world = declare_parameter("world", std::string("bag_hall"));
    pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/cell/bag_poses", 10);
    gz_.Subscribe("/world/" + world + "/dynamic_pose/info", &PoseRelay::on_poses, this);
    timer_ = create_wall_timer(50ms, [this]() {
          std::lock_guard<std::mutex> lock(mutex_);
          if (!fresh_) {return;}
          pub_->publish(last_);
          fresh_ = false;
        });
  }

private:
  void on_poses(const gz::msgs::Pose_V & msg)
  {
    tf2_msgs::msg::TFMessage out;
    for (const auto & p : msg.pose()) {
      if (p.name().rfind("bag_", 0) != 0) {continue;}
      geometry_msgs::msg::TransformStamped t;
      t.header.frame_id = "world";
      t.header.stamp.sec = msg.header().stamp().sec();
      t.header.stamp.nanosec = msg.header().stamp().nsec();
      t.child_frame_id = p.name();
      t.transform.translation.x = p.position().x();
      t.transform.translation.y = p.position().y();
      t.transform.translation.z = p.position().z();
      t.transform.rotation.x = p.orientation().x();
      t.transform.rotation.y = p.orientation().y();
      t.transform.rotation.z = p.orientation().z();
      t.transform.rotation.w = p.orientation().w();
      out.transforms.push_back(t);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    last_ = out;
    fresh_ = true;
  }

  gz::transport::Node gz_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  tf2_msgs::msg::TFMessage last_;
  bool fresh_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseRelay>());
  rclcpp::shutdown();
  return 0;
}
