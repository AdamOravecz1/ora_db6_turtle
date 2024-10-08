#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "turtlesim/msg/pose.hpp"

class HouseDraw : public rclcpp::Node
{
public:
    HouseDraw() : Node("house_draw"), count_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&HouseDraw::draw_house, this));
        RCLCPP_INFO_STREAM(this->get_logger(), "Drawing a house in turtlesim.");
    }

private:
    void publish_message(double fwd, double turn)
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = fwd;
        message.angular.z = turn;
        count_++;
        RCLCPP_INFO(this->get_logger(), "Step %ld. speed: '%.1f' turn: '%.1f'", count_, message.linear.x, message.angular.z);
        publisher_->publish(message);
        std::this_thread::sleep_for(std::chrono::seconds(2)); 
    }

    void draw_house()
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Drawing house started.");

        for (int i = 0; i < 4; i++) {
            publish_message(2.0, 0.0); 
            publish_message(0.0, M_PI_2);
        }

        publish_message(2.0, M_PI_4); 
        publish_message(2.0, -M_PI_2); 

        RCLCPP_INFO_STREAM(this->get_logger(), "House drawing complete.");
        rclcpp::shutdown();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HouseDraw>());
    rclcpp::shutdown();
    return 0;
}
