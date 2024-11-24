#include "walker/walker_node.hpp"

// MovingForwardState Implementation
void MovingForwardState::Execute(const std::shared_ptr<rclcpp::Node>& node,
                                  WalkerStateMachine* state_machine) {
  auto walker_node = std::dynamic_pointer_cast<WalkerNode>(node);
  if (walker_node->IsObstacleAhead()) {
    RCLCPP_INFO(node->get_logger(), "Obstacle detected. Switching to RotatingState.");
    state_machine->SetState(std::make_shared<RotatingState>());
  } else {
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = 0.2;  // Forward speed
    cmd.angular.z = 0.0;
    walker_node->cmd_vel_pub_->publish(cmd);
  }
}

// RotatingState Implementation
RotatingState::RotatingState() : rotate_clockwise_(true) {}

void RotatingState::Execute(const std::shared_ptr<rclcpp::Node>& node,
                             WalkerStateMachine* state_machine) {
  auto walker_node = std::dynamic_pointer_cast<WalkerNode>(node);
  if (!walker_node->IsObstacleAhead()) {
    RCLCPP_INFO(node->get_logger(), "Path clear. Switching to MovingForwardState.");
    rotate_clockwise_ = !rotate_clockwise_;
    state_machine->SetState(std::make_shared<MovingForwardState>());
  } else {
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = 0.0;
    cmd.angular.z = rotate_clockwise_ ? -0.3 : 0.3;
    walker_node->cmd_vel_pub_->publish(cmd);
  }
}

// WalkerStateMachine Implementation
WalkerStateMachine::WalkerStateMachine()
    : current_state_(std::make_shared<MovingForwardState>()) {}

void WalkerStateMachine::SetState(std::shared_ptr<WalkerState> new_state) {
  current_state_ = std::move(new_state);
}

void WalkerStateMachine::Execute(const std::shared_ptr<rclcpp::Node>& node) {
  current_state_->Execute(node, this);
}

// WalkerNode Implementation
WalkerNode::WalkerNode()
    : Node("walker_node"), state_machine_(std::make_shared<WalkerStateMachine>()) {
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&WalkerNode::LaserCallback, this, std::placeholders::_1));

  this->create_wall_timer(std::chrono::milliseconds(100),
                          std::bind(&WalkerNode::Run, this));
}

bool WalkerNode::IsObstacleAhead() {
  if (!laser_data_.empty()) {
    float front_distance = laser_data_[laser_data_.size() / 2];
    return front_distance < 0.5;  // Obstacle threshold
  }
  return false;
}

void WalkerNode::LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  laser_data_ = msg->ranges;
}

void WalkerNode::Run() {
  state_machine_->Execute(shared_from_this());
}

// Main Function
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WalkerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
