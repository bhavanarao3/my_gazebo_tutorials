#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>

using std::placeholders::_1;

// Forward declaration
class Walker;

// State interface
class WalkerState {
 public:
  virtual void handleLaserScan(Walker* walker, const sensor_msgs::msg::LaserScan& scan) = 0;
  virtual ~WalkerState() = default;
};

// Concrete states
class MovingForwardState : public WalkerState {
 public:
  void handleLaserScan(Walker* walker, const sensor_msgs::msg::LaserScan& scan) override;
};

class RotatingState : public WalkerState {
 public:
  void handleLaserScan(Walker* walker, const sensor_msgs::msg::LaserScan& scan) override;
};

class Walker : public rclcpp::Node {
 public:
  Walker()
      : Node("walker_node"),
        turn_clockwise(true) {
    laser_data_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Walker::LaserDataCB, this, _1));
    velocity_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    setState(std::make_unique<MovingForwardState>());
    RCLCPP_INFO(this->get_logger(), "Walker node initialized.");
  }

  void setState(std::unique_ptr<WalkerState> new_state) {
    current_state = std::move(new_state);
  }

  void moveForward() {
    geometry_msgs::msg::Twist velocity_msg;
    velocity_msg.linear.x = 0.2;
    velocity_msg.angular.z = 0.0;
    velocity_pub->publish(velocity_msg);
    RCLCPP_INFO(this->get_logger(), "Moving forward...");
  }

  void rotateInPlace() {
    geometry_msgs::msg::Twist velocity_msg;
    velocity_msg.linear.x = 0.0;
    velocity_msg.angular.z = turn_clockwise ? -0.5 : 0.5;
    velocity_pub->publish(velocity_msg);
    RCLCPP_INFO(this->get_logger(), "Rotating %s...",
                turn_clockwise ? "clockwise" : "counterclockwise");
    RCLCPP_DEBUG(this->get_logger(), "Angular velocity set to %f",
                 velocity_msg.angular.z);
  }

  void toggleDirection() {
    turn_clockwise = !turn_clockwise;
    RCLCPP_INFO(this->get_logger(), "Switching rotation direction to %s",
                turn_clockwise ? "clockwise" : "counterclockwise");
  }

  bool isTurningClockwise() const { return turn_clockwise; }

 private:
  void LaserDataCB(const sensor_msgs::msg::LaserScan& scan) {
    if (scan.ranges.empty()) {
      RCLCPP_WARN(this->get_logger(), "Laser scan data is empty.");
      return;
    }
    current_state->handleLaserScan(this, scan);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_data_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub;
  std::unique_ptr<WalkerState> current_state;
  bool turn_clockwise;
};

void MovingForwardState::handleLaserScan(Walker* walker, const sensor_msgs::msg::LaserScan& scan) {
  bool obstacle_detected = false;
  for (int i = 330; i < 390; ++i) {
    if (scan.ranges[i % 360] < 0.8) {
      obstacle_detected = true;
      break;
    }
  }

  if (obstacle_detected) {
    RCLCPP_INFO(walker->get_logger(), "Obstacle detected! Starting rotation...");
    walker->setState(std::make_unique<RotatingState>());
    walker->rotateInPlace();
  } else {
    walker->moveForward();
  }
}

void RotatingState::handleLaserScan(Walker* walker, const sensor_msgs::msg::LaserScan& scan) {
  bool path_clear = true;
  for (int i = 330; i < 390; ++i) {
    if (scan.ranges[i % 360] < 0.8) {
      path_clear = false;
      break;
    }
  }

  if (path_clear) {
    RCLCPP_INFO(walker->get_logger(), "Path clear! Moving forward...");
    walker->toggleDirection();
    walker->setState(std::make_unique<MovingForwardState>());
    walker->moveForward();
  } else {
    walker->rotateInPlace();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Walker>());
  rclcpp::shutdown();
  return 0;
}