#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class HouseDraw : public rclcpp::Node
{
public:
    HouseDraw() : Node("house_draw"), count_(0)
    {
        // Publisher to send movement commands to the turtle
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        // Start the drawing process with a timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&HouseDraw::draw_house, this));
        RCLCPP_INFO_STREAM(this->get_logger(), "Drawing a house in turtlesim.");
    }

private:
    // Function to publish Twist messages to move the turtle
    void publish_message(double fwd, double turn)
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = fwd;  // Forward speed
        message.angular.z = turn; // Turning speed
        count_++;

        RCLCPP_INFO(this->get_logger(), "Step %ld. speed: '%.1f' turn: '%.1f'", count_, fwd, turn);
        publisher_->publish(message);

        // Allow turtle to move for a specific duration
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
    }

    // Function to draw a house shape
    void draw_house()
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Drawing house started.");

        // Draw square for the base of the house
        for (int i = 0; i < 4; i++) {
            publish_message(2.0, 0.0);   // Move forward
            publish_message(0.0, M_PI_2); // Rotate 90 degrees
        }

        // Draw the roof as a triangle
        publish_message(2.0, M_PI_4); // Move forward at an angle for the roof side
        publish_message(2.0, -M_PI_2); // Move back at an angle for the other roof side

        RCLCPP_INFO_STREAM(this->get_logger(), "House drawing complete.");
        rclcpp::shutdown(); // Shutdown the node after drawing
    }

    rclcpp::TimerBase::SharedPtr timer_; // Timer for loop
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // Publisher for /cmd_vel
    size_t count_; // Keep track of step count
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HouseDraw>()); // Start the node
    rclcpp::shutdown();
    return 0;
}
