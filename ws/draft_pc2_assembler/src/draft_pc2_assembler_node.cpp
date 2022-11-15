#include <cstdio>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "message_filters/subscriber.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "pcl_conversions/pcl_conversions.h"
#include <sensor_msgs/msg/point_cloud2.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/projection_matrix.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class DraftPC2Assembler : public rclcpp::Node
{
public:
  DraftPC2Assembler()
      : Node("draft_pc2_assembler_node")
  {
    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("draft_scan", 10);
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos,
        std::bind(&DraftPC2Assembler::scanCallback, this, std::placeholders::_1));
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_in)
  {
    try
    {
      sensor_msgs::msg::PointCloud2 cloud, cloud_out;
      transformStamped = (*tf_buffer_).lookupTransform("base_link", "laser_frame",tf2::TimePointZero);
      pcl::PCLPointCloud2 pcl_pc;
      projector_.projectLaser(*scan_in, cloud);
      tf2::doTransform(cloud, cloud_out, transformStamped);
      pcl_conversions::toPCL(cloud, pcl_pc);
      (pcl_pc) += (draftCloud);
      draftCloud = pcl_pc;
      pcl_conversions::fromPCL(draftCloud, cloud);
      publisher_->publish(cloud);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  laser_geometry::LaserProjection projector_;
  pcl::PCLPointCloud2 draftCloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  geometry_msgs::msg::TransformStamped transformStamped;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DraftPC2Assembler>());
  rclcpp::shutdown();
  return 0;
}