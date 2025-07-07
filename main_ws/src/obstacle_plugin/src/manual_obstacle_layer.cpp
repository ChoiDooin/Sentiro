#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace custom_layers
{

class ManualObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  ManualObstacleLayer() = default;

  void onInitialize() override
  {
    auto node = node_.lock();
    if (!node)
    {
      RCLCPP_ERROR(rclcpp::get_logger("manual_obstacle_layer"), "Node expired!");
      return;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    declareParameter("obstacle_radius", rclcpp::ParameterValue(0.5));
    node->get_parameter(name_ + ".obstacle_radius", obstacle_radius_);

    sub_ = node->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", rclcpp::QoS(10),
      std::bind(&ManualObstacleLayer::obstacleCallback, this, std::placeholders::_1));

    last_min_x_ = last_min_y_ = std::numeric_limits<double>::max();
    last_max_x_ = last_max_y_ = std::numeric_limits<double>::lowest();
    received_obstacle_ = false;

    matchSize();  // 내부 costmap_ 크기 초기화
  }

  void matchSize() override
  {
    CostmapLayer::matchSize();
    costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(*layered_costmap_->getCostmap());
  }

  void obstacleCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    auto node = node_.lock();
    if (!node) return;

    geometry_msgs::msg::PointStamped transformed_point;
    try
    {
      tf_buffer_->transform(*msg, transformed_point, layered_costmap_->getGlobalFrameID(), tf2::durationFromSec(0.5));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(rclcpp::get_logger("manual_obstacle_layer"), "TF transform failed: %s", ex.what());
      return;
    }

    double x = transformed_point.point.x;
    double y = transformed_point.point.y;

    unsigned int mx, my;
    if (!costmap_->worldToMap(x, y, mx, my))
    {
      RCLCPP_WARN(rclcpp::get_logger("manual_obstacle_layer"),
        "Obstacle (%.2f, %.2f) out of map bounds", x, y);
      return;
    }

    int cells = static_cast<int>(obstacle_radius_ / costmap_->getResolution());
    for (int dx = -cells; dx <= cells; ++dx)
    {
      for (int dy = -cells; dy <= cells; ++dy)
      {
        int nx = static_cast<int>(mx) + dx;
        int ny = static_cast<int>(my) + dy;

        if (nx < 0 || ny < 0 || nx >= static_cast<int>(costmap_->getSizeInCellsX()) || ny >= static_cast<int>(costmap_->getSizeInCellsY()))
          continue;

        costmap_->setCost(nx, ny, nav2_costmap_2d::LETHAL_OBSTACLE);

        double wx, wy;
        costmap_->mapToWorld(nx, ny, wx, wy);
        touch(wx, wy, &last_min_x_, &last_min_y_, &last_max_x_, &last_max_y_);
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("manual_obstacle_layer"),
      "Marked obstacle at (%.2f, %.2f) -> map (%d, %d)", x, y, mx, my);

    received_obstacle_ = true;
  }

  void updateBounds(double, double, double, double *min_x, double *min_y, double *max_x, double *max_y) override
  {
    if (!received_obstacle_) return;

    *min_x = std::min(*min_x, last_min_x_);
    *min_y = std::min(*min_y, last_min_y_);
    *max_x = std::max(*max_x, last_max_x_);
    *max_y = std::max(*max_y, last_max_y_);

    last_min_x_ = last_min_y_ = std::numeric_limits<double>::max();
    last_max_x_ = last_max_y_ = std::numeric_limits<double>::lowest();
  }

  void updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                   int min_i, int min_j, int max_i, int max_j) override
  {
    for (int i = min_i; i < max_i; ++i)
    {
      for (int j = min_j; j < max_j; ++j)
      {
        unsigned char cost = costmap_->getCost(i, j);
        if (cost == nav2_costmap_2d::LETHAL_OBSTACLE)
        {
          master_grid.setCost(i, j, cost);
        }
      }
    }

    received_obstacle_ = false;
  }

  void reset() override
  {
    RCLCPP_INFO(rclcpp::get_logger("manual_obstacle_layer"), "Reset called");
    received_obstacle_ = false;
  }

  bool isClearable() override { return false; }

private:
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;  // 🔥 내부 버퍼
  double obstacle_radius_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  bool received_obstacle_;
};

}  // namespace custom_layers

PLUGINLIB_EXPORT_CLASS(custom_layers::ManualObstacleLayer, nav2_costmap_2d::Layer)
