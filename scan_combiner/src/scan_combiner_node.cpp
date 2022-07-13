#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace std;
using namespace octomap;


using std::placeholders::_1;

class ScanCombiner : public rclcpp::Node
{
  public:
    ScanCombiner()
    : Node("scan_combiner")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "scan", 10, std::bind(&ScanCombiner::scan_callback, this, _1));
    }

  private:
    void scan_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanCombiner>());
  rclcpp::shutdown();
  return 0;
}