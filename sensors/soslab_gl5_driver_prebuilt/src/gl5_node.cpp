// SOSLAB GL-5 ROS2 driver node built on the SOSLAB_SDK: publishes PointCloud2
// with fields x, y, z, intensity (raw pulse width); parameters as in sensors.yaml.

#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "Lidar.h"
#include "soslabTypedef.h"

class GL5Node : public rclcpp::Node
{
public:
  GL5Node() : Node("soslab_gl5")
  {
    declare_parameter("lidarType", std::string("GL5"));
    declare_parameter("ip_address_device", std::string("10.110.1.2"));
    declare_parameter("ip_port_device", 2020);
    declare_parameter("ip_address_pc", std::string("10.110.1.3"));
    declare_parameter("ip_port_pc", 3000);
    declare_parameter("pc_topic_lidar0", std::string("soslab/pointcloud"));
    declare_parameter("pc_topic_lidar1", std::string("lidar1/pointcloud"));
    declare_parameter("frame_id", std::string("laser"));
    declare_parameter("max_intensity", 3000);  // kept for config compat; unused (raw intensity published)

    const std::string type_str = get_parameter("lidarType").as_string();
    params_.lidarTypeValue = soslab::lidarType::GL5;
    if (type_str == "GL3") params_.lidarTypeValue = soslab::lidarType::GL3;
    else if (type_str != "GL5")
      RCLCPP_WARN(get_logger(), "lidarType '%s' not GL3/GL5, defaulting to GL5", type_str.c_str());

    params_.conectionTypeValue = soslab::connectionType::ETHERNET;
    params_.lidarIP = get_parameter("ip_address_device").as_string();
    params_.lidarPort = static_cast<int>(get_parameter("ip_port_device").as_int());
    params_.pcIP = get_parameter("ip_address_pc").as_string();
    params_.pcPort = static_cast<int>(get_parameter("ip_port_pc").as_int());
    frame_id_ = get_parameter("frame_id").as_string();

    RCLCPP_INFO(get_logger(), "GL5 device=%s:%d pc=%s:%d frame=%s",
                params_.lidarIP.c_str(), params_.lidarPort,
                params_.pcIP.c_str(), params_.pcPort, frame_id_.c_str());

    // RELIABLE publisher: compatible with both reliable and best-effort
    // (sensor-data QoS) subscribers, e.g. pointcloud_to_laserscan and rviz.
    pc_pub_[0] = create_publisher<sensor_msgs::msg::PointCloud2>(
        get_parameter("pc_topic_lidar0").as_string(), rclcpp::QoS(5));
    pc_pub_[1] = create_publisher<sensor_msgs::msg::PointCloud2>(
        get_parameter("pc_topic_lidar1").as_string(), rclcpp::QoS(5));
  }

  ~GL5Node() override
  {
    if (lidar_) {
      lidar_->streamStop();
      lidar_->disconnectLidar();
    }
  }

  bool connect()
  {
    lidar_ = std::make_shared<soslab::Lidar>();
    lidar_->setParameters(params_);

    if (!lidar_->connectLidar()) {
      RCLCPP_ERROR(get_logger(), "GL5 connection failed (device %s:%d).",
                   params_.lidarIP.c_str(), params_.lidarPort);
      return false;
    }
    lidar_->registerGetDataCallBack(
        [this](std::shared_ptr<const soslab::FrameData> f) { frame_callback(f); });

    lidar_->streamStop();
    if (!lidar_->streamStart()) {
      RCLCPP_ERROR(get_logger(), "GL5 streamStart failed.");
      return false;
    }
    RCLCPP_INFO(get_logger(), "GL5 streaming started.");
    return true;
  }

private:
  void frame_callback(const std::shared_ptr<const soslab::FrameData> & scene)
  {
    if (!scene || scene->points.empty() || scene->points[0].empty()) return;

    const std::vector<soslab::Points> & pts = scene->points[0];
    const bool has_int = !scene->intensity.empty() && scene->intensity[0].size() >= pts.size();
    const std::size_t npts = pts.size();

    auto pc = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pc->header.frame_id = frame_id_;
    pc->header.stamp = now();
    pc->height = 1;
    pc->width = static_cast<uint32_t>(npts);
    pc->is_bigendian = false;
    pc->is_dense = true;

    pc->fields.resize(4);
    const char * names[4] = {"x", "y", "z", "intensity"};
    for (int i = 0; i < 4; ++i) {
      pc->fields[i].name = names[i];
      pc->fields[i].offset = static_cast<uint32_t>(4 * i);
      pc->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      pc->fields[i].count = 1;
    }
    pc->point_step = 16;
    pc->row_step = pc->point_step * pc->width;
    pc->data.resize(pc->row_step);

    for (std::size_t i = 0; i < npts; ++i) {
      // SDK reports millimeters (see examples/ros2_ml); publish meters.
      const float x = pts[i].x / 1000.0f;
      const float y = pts[i].y / 1000.0f;
      const float z = pts[i].z / 1000.0f;
      const float inten = has_int ? static_cast<float>(scene->intensity[0][i]) : 0.0f;
      uint8_t * dst = &pc->data[i * pc->point_step];
      std::memcpy(dst + 0, &x, 4);
      std::memcpy(dst + 4, &y, 4);
      std::memcpy(dst + 8, &z, 4);
      std::memcpy(dst + 12, &inten, 4);
    }

    const int id = scene->lidarId < 2 ? scene->lidarId : 0;
    pc_pub_[id]->publish(std::move(pc));
  }

  soslab::lidarParameters params_;
  std::shared_ptr<soslab::Lidar> lidar_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_[2];
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GL5Node>();
  if (!node->connect()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
