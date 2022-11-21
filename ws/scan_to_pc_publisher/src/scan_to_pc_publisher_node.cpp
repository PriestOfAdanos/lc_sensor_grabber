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
#include "lc_interfaces/msg/is_recording.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ScanToPC2Publisher : public rclcpp::Node
{
public:
  ScanToPC2Publisher()
      : Node("scan_to_pc_publisher_node")
  {
    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("draft_scan", 10);
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    laser_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos,
        std::bind(&ScanToPC2Publisher::scanCallback, this, std::placeholders::_1));
    // timer_ = this->create_wall_timer(
    //     1s, std::bind(&ScanToPC2Publisher::timerCallback, this));
    is_recording_subscription_ = this->create_subscription<lc_interfaces::msg::IsRecording>(
        "is_recording",
        10,
        std::bind(&ScanToPC2Publisher::isRecordingCallback, this, std::placeholders::_1));
  }

private:
  // void timerCallback()
  // {
  //   sensor_msgs::msg::PointCloud2 cloud;
  //   pcl_conversions::fromPCL(pcl_pc, cloud);
    
  // }

  void isRecordingCallback(const lc_interfaces::msg::IsRecording::SharedPtr msg)
  {
    if (is_recording != msg->is_recording)
    {

      is_recording = msg->is_recording;

      if (is_recording == false)
      {
        // if it is false now, it must mean it was true before, so we know
        //  that recording went from true to false, so we need to save the pcd
        writer.write("exampe_pcd.pcd", draftCloud);
      }
    }
  }
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_in)
  {
    try
    {
      sensor_msgs::msg::PointCloud2 cloud, cloud_out;
      transformStamped = (*tf_buffer_).lookupTransform("base_link", "laser_frame", tf2::TimePointZero);
      projector_.projectLaser(*scan_in, cloud);
      tf2::doTransform(cloud, cloud_out, transformStamped);
      publisher_->publish(cloud_out);
      pcl_conversions::toPCL(cloud_out, pcl_pc);
      if (is_recording)
      {
        (pcl_pc) += (draftCloud);
      }
      draftCloud = pcl_pc;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription_;
  rclcpp::Subscription<lc_interfaces::msg::IsRecording>::SharedPtr is_recording_subscription_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  laser_geometry::LaserProjection projector_;
  pcl::PCLPointCloud2 draftCloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  geometry_msgs::msg::TransformStamped transformStamped;
  rclcpp::TimerBase::SharedPtr timer_;
  bool is_recording;
  pcl::PCDWriter writer;
  pcl::PCLPointCloud2 pcl_pc;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanToPC2Publisher>());
  rclcpp::shutdown();
  return 0;
}