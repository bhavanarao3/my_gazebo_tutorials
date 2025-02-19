/**
 * @file walker_node.cpp
 * @brief Implementation of a finite state machine for a TurtleBot3 Walker.
 *
 * The walker alternates between moving forward and rotating to avoid obstacles
 based on laser scan data.
 *
 * MIT License
 * @copyright Copyright (c) 2024 Bhavana B Rao
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
 */

#include <memory>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

// Forward declaration
class Walker;

/**
 * @brief Abstract base class representing a state in the Walker's finite state
 * machine.
 */
class WalkerState {
 public:
  /**
   * @brief Handle incoming laser scan data.
   *
   * @param walker Pointer to the Walker object.
   * @param scan The LaserScan message received.
   */
  virtual void handleLaserScan(Walker* walker,
                               const sensor_msgs::msg::LaserScan& scan) = 0;

  /// Virtual destructor for base class.
  virtual ~WalkerState() = default;
};

/**
 * @brief State where the robot moves forward until it detects an obstacle.
 */
class MovingForwardState : public WalkerState {
 public:
  /**
   * @brief Handles laser scan data while in the moving forward state.
   *
   * If an obstacle is detected in the front, the robot switches to the rotating
   * state.
   *
   * @param walker Pointer to the Walker object.
   * @param scan The LaserScan message received.
   */
  void handleLaserScan(Walker* walker,
                       const sensor_msgs::msg::LaserScan& scan) override;
};

/**
 * @brief State where the robot rotates in place to avoid obstacles.
 */
class RotatingState : public WalkerState {
 public:
  /**
   * @brief Handles laser scan data while in the rotating state.
   *
   * If the path ahead becomes clear, the robot switches back to the moving
   * forward state.
   *
   * @param walker Pointer to the Walker object.
   * @param scan The LaserScan message received.
   */
  void handleLaserScan(Walker* walker,
                       const sensor_msgs::msg::LaserScan& scan) override;
};

/**
 * @brief Main class representing the Walker robot node.
 */
class Walker : public rclcpp::Node {
 public:
  /**
   * @brief Constructor to initialize the Walker node.
   *
   * Sets up ROS2 publishers and subscribers, and initializes the finite state
   * machine.
   */
  Walker() : Node("walker_node"), turn_clockwise(true) {
    laser_data_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Walker::LaserDataCB, this, _1));
    velocity_pub =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    setState(std::make_unique<MovingForwardState>());
    RCLCPP_INFO(this->get_logger(), "Walker node initialized.");
  }

  /**
   * @brief Sets the current state of the Walker.
   *
   * @param new_state The new state to transition to.
   */
  void setState(std::unique_ptr<WalkerState> new_state) {
    current_state = std::move(new_state);
  }

  /**
   * @brief Publishes a velocity command to move forward.
   */
  void moveForward() {
    geometry_msgs::msg::Twist velocity_msg;
    velocity_msg.linear.x = 0.2;
    velocity_msg.angular.z = 0.0;
    velocity_pub->publish(velocity_msg);
    RCLCPP_INFO(this->get_logger(), "Moving forward...");
  }

  /**
   * @brief Publishes a velocity command to rotate in place.
   */
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

  /**
   * @brief Toggles the direction of rotation (clockwise/counterclockwise).
   */
  void toggleDirection() {
    turn_clockwise = !turn_clockwise;
    RCLCPP_INFO(this->get_logger(), "Switching rotation direction to %s",
                turn_clockwise ? "clockwise" : "counterclockwise");
  }

  /**
   * @brief Checks if the robot is currently turning clockwise.
   *
   * @return True if the robot is turning clockwise, false otherwise.
   */
  bool isTurningClockwise() const { return turn_clockwise; }

 private:
  /**
   * @brief Callback function to handle laser scan data.
   *
   * Delegates processing of laser scan data to the current state.
   *
   * @param scan The LaserScan message received.
   */
  void LaserDataCB(const sensor_msgs::msg::LaserScan& scan) {
    if (scan.ranges.empty()) {
      RCLCPP_WARN(this->get_logger(), "Laser scan data is empty.");
      return;
    }
    current_state->handleLaserScan(this, scan);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_data_sub;  ///< Laser scan subscriber
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      velocity_pub;  ///< Velocity command publisher
  std::unique_ptr<WalkerState>
      current_state;    ///< Current state of the finite state machine
  bool turn_clockwise;  ///< Direction of rotation
};

void MovingForwardState::handleLaserScan(
    Walker* walker, const sensor_msgs::msg::LaserScan& scan) {
  bool obstacle_detected = false;
  for (int i = 330; i < 390; ++i) {
    if (scan.ranges[i % 360] < 0.8) {
      obstacle_detected = true;
      break;
    }
  }

  if (obstacle_detected) {
    RCLCPP_INFO(walker->get_logger(),
                "Obstacle detected! Starting rotation...");
    walker->setState(std::make_unique<RotatingState>());
    walker->rotateInPlace();
  } else {
    walker->moveForward();
  }
}

void RotatingState::handleLaserScan(Walker* walker,
                                    const sensor_msgs::msg::LaserScan& scan) {
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

/**
 * @brief Main function to initialize and run the Walker node.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Walker>());
  rclcpp::shutdown();
  return 0;
}
