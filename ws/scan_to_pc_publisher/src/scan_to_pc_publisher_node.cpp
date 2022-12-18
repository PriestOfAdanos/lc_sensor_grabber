#include <cstdio>
#include <memory>
#include <chrono>

#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
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
#include <pcl/filters/statistical_outlier_removal.h>

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
  void isRecordingCallback(const lc_interfaces::msg::IsRecording::SharedPtr msg)
  {
    if (is_recording != msg->is_recording)
    {

      is_recording = msg->is_recording;

      if (is_recording == false)
      {
        // SOS: https://pcl.gitbook.io/tutorial/part-2/part02-chapter06/part02-chapter06-triangulation-pcl-cpp
        // if it is false now, it must mean it was true before, so we know
        //  that recording went from true to false, so we need to save the pcd
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // Object for storing the normals.
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        // Object for storing both the points and the normals.
        pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals(new pcl::PointCloud<pcl::PointNormal>);

        pcl::io::savePCDFile("/bags/raw_points.pcd", *draftCloud);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        {
          const pcl::PointCloud<pcl::PointXYZ>::ConstPtr const_cloud = draftCloud->makeShared();
          sor.setInputCloud(const_cloud);
        }
        draftCloud->clear();

        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud);

        // pcl::io::loadPCDFile<pcl::PointXYZ>("/bags/raw_points.pcd", *cloud);
        // const_cloud.clear();
        // Normal estimation.
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
        normalEstimation.setInputCloud(cloud);
        normalEstimation.setRadiusSearch(0.03);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        normalEstimation.setSearchMethod(kdtree);
        normalEstimation.compute(*normals);

        // The triangulation object requires the points and normals to be stored in the same structure.
        pcl::concatenateFields(*cloud, *normals, *cloudNormals);
        // Tree object for searches in this new object.
        pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree2(new pcl::search::KdTree<pcl::PointNormal>);
        kdtree2->setInputCloud(cloudNormals);

        // Triangulation object.
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> triangulation;
        // Output object, containing the mesh.
        pcl::PolygonMesh triangles;
        // Maximum distance between connected points (maximum edge length).
        triangulation.setSearchRadius(0.025);
        // Maximum acceptable distance for a point to be considered,
        // relative to the distance of the nearest point.
        triangulation.setMu(2.5);
        // How many neighbors are searched for.
        triangulation.setMaximumNearestNeighbors(100);
        // Points will not be connected to the current point
        // if their normals deviate more than the specified angle.
        triangulation.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees.
        // If false, the direction of normals will not be taken into account
        // when computing the angle between them.
        triangulation.setNormalConsistency(false);
        // Minimum and maximum angle there can be in a triangle.
        // The first is not guaranteed, the second is.
        triangulation.setMinimumAngle(M_PI / 18);    // 10 degrees.
        triangulation.setMaximumAngle(2 * M_PI / 3); // 120 degrees.

        // Triangulate the cloud.
        triangulation.setInputCloud(cloudNormals);
        triangulation.setSearchMethod(kdtree2);
        triangulation.reconstruct(triangles);

        pcl::io::saveOBJFile("/bags/constructed_mesh.obj", triangles);
      }
      // todo add cleanup after scan is done (delete pointcloud)
    }
  }
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_in)
  {
    try
    {
      pcl::PCLPointCloud2 *pcl_pc2 = new pcl::PCLPointCloud2;
      sensor_msgs::msg::PointCloud2 cloud, cloud_out;
      transformStamped = (*tf_buffer_).lookupTransform("base_link", "laser_frame", tf2::TimePointZero);
      projector_.projectLaser(*scan_in, cloud);
      tf2::doTransform(cloud, cloud_out, transformStamped);
      publisher_->publish(cloud_out);
      pcl_conversions::toPCL(cloud_out, *pcl_pc2);

      // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(*pcl_pc2, *pcl_pc);

      if (is_recording == true)
      {
        (*pcl_pc) += (*draftCloud);
      }
      *draftCloud = *pcl_pc;
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
  pcl::PointCloud<pcl::PointXYZ> *draftCloud = new pcl::PointCloud<pcl::PointXYZ>;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  geometry_msgs::msg::TransformStamped transformStamped;
  rclcpp::TimerBase::SharedPtr timer_;
  bool is_recording;
  pcl::PointCloud<pcl::PointXYZ> *pcl_pc = new pcl::PointCloud<pcl::PointXYZ>;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanToPC2Publisher>());
  rclcpp::shutdown();
  return 0;
}
