#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/timesync_status.hpp>  // 使用 TimesyncStatus 而不是 timesync
#include <chrono>

using namespace std::chrono_literals;

class ArmTestNode : public rclcpp::Node {
public:
    ArmTestNode() : Node("arm_test_node") {
        // 创建发布器
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
            
        // 订阅时间同步状态（可选）
        timesync_sub_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
            "/fmu/out/timesync_status", 10,
            [this](const px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
                // 如果需要时间同步，可以在这里处理
                last_timesync_ = msg->timestamp;
            });
            
        // 定时器
        timer_ = this->create_wall_timer(
            3000ms, [this]() { this->arm_disarm_sequence(); });
            
        RCLCPP_INFO(this->get_logger(), "Arm Test Node Started");
    }

private:
    void arm_disarm_sequence() {
        timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Arming drone...");
        arm();
        
        // 5秒后上锁
        disarm_timer_ = this->create_wall_timer(
            5000ms, [this]() { 
                RCLCPP_INFO(this->get_logger(), "Disarming drone...");
                disarm();
                rclcpp::shutdown();  // 完成后关闭节点
            });
    }
    
    void arm() {
        send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    }
    
    void disarm() {
        send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    }
    
    void send_vehicle_command(uint32_t command, float param1 = 0.0f) {
        auto msg = px4_msgs::msg::VehicleCommand();
        
        // 使用系统时间
        auto now = this->now();
        msg.timestamp = now.nanoseconds() / 1000; // 转换为微秒
        
        msg.param1 = param1;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        
        vehicle_command_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: %u", command);
    }
    
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr disarm_timer_;
    uint64_t last_timesync_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}