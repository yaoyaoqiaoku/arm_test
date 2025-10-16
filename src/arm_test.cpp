#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <chrono>

using namespace std::chrono_literals;

class ArmTestNode : public rclcpp::Node {
public:
    ArmTestNode() : Node("arm_test_node") {
        // 创建发布器（发布无需特殊QoS，使用默认即可）
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);  // 10是队列大小
            
        // 订阅时间同步状态：使用传感器数据QoS解决兼容性问题
        // 关键修改：用 rclcpp::SensorDataQoS() 指定QoS，并设置队列大小
        rclcpp::QoS qos_sensor = rclcpp::SensorDataQoS().keep_last(10);  // 保留最近10条消息
        timesync_sub_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
            "/fmu/out/timesync_status",
            qos_sensor,  // 使用传感器数据QoS
            [this](const px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
                last_timesync_ = msg->timestamp;  // 保存PX4同步时间戳（微秒）
            });
            
        // 定时器：3秒后开始解锁-上锁序列
        timer_ = this->create_wall_timer(
            3000ms, [this]() { this->arm_disarm_sequence(); });
            
        RCLCPP_INFO(this->get_logger(), "Arm Test Node Started");
    }

private:
    void arm_disarm_sequence() {
        timer_->cancel();  // 取消初始定时器
        RCLCPP_INFO(this->get_logger(), "Arming drone...");
        arm();
        
        // 5秒后执行上锁
        disarm_timer_ = this->create_wall_timer(
            5000ms, [this]() { 
                RCLCPP_INFO(this->get_logger(), "Disarming drone...");
                disarm();
                rclcpp::shutdown();  // 完成后关闭节点
            });
    }
    
    void arm() {
        // 发送解锁命令（参数1为1.0表示解锁）
        send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    }
    
    void disarm() {
        // 发送上锁命令（参数1为0.0表示上锁）
        send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    }
    
    void send_vehicle_command(uint32_t command, float param1 = 0.0f) {
        auto msg = px4_msgs::msg::VehicleCommand();
        
        // 优化：使用与PX4同步的时间戳（避免时间不同步导致命令无效）
        // 如果未收到timesync， fallback到本地时间
        msg.timestamp = (last_timesync_ != 0) ? last_timesync_ : this->now().nanoseconds() / 1000;
        
        msg.param1 = param1;  // 对于ARM_DISARM命令，param1=1解锁，=0上锁
        msg.command = command;
        msg.target_system = 1;    // 目标系统ID（默认1）
        msg.target_component = 1; // 目标组件ID（默认1）
        msg.source_system = 1;    // 源系统ID（当前节点）
        msg.source_component = 1; // 源组件ID（当前节点）
        msg.from_external = true; // 标记为外部命令（非PX4内部）
        
        vehicle_command_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: %u", command);
    }
    
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr disarm_timer_;
    uint64_t last_timesync_ = 0;  // 存储PX4同步时间戳（微秒）
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
